#!/usr/bin/env python3
"""Audit ROS 2 bag timing for LiDAR/IMU replay through an odometry frontend.

The audit distinguishes timestamps recorded in the bag from delays introduced
while playing it.  It checks storage/header gaps, non-monotonic stamps, sensor
clock offsets and drift, causal IMU coverage at each LiDAR delivery, and the
native per-point timestamp range in PointCloud2 messages.  An optional second
bag can be compared record-for-record with the first one after restamping.
"""

import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


NSEC = 1_000_000_000


def stamp_ns(stamp):
    return int(stamp.sec) * NSEC + int(stamp.nanosec)


def finite_stats(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return {"count": 0}
    ordered = sorted(values)

    def percentile(fraction):
        if len(ordered) == 1:
            return ordered[0]
        position = fraction * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        "count": len(ordered),
        "min": ordered[0],
        "median": statistics.median(ordered),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "max": ordered[-1],
        "mean": statistics.fmean(ordered),
        "stddev": statistics.pstdev(ordered) if len(ordered) > 1 else 0.0,
    }


def linear_drift_ppm(times_ns, offsets_sec):
    if len(times_ns) < 2 or len(times_ns) != len(offsets_sec):
        return 0.0
    origin = times_ns[0]
    xs = [(stamp - origin) / NSEC for stamp in times_ns]
    x_mean = statistics.fmean(xs)
    y_mean = statistics.fmean(offsets_sec)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator == 0.0:
        return 0.0
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, offsets_sec))
    return slope / denominator * 1.0e6


def delta_report(storage_ns, header_ns, gap_factor, absolute_gap_sec, limit=20):
    if len(storage_ns) < 2:
        return {"message_count": len(storage_ns), "deltas": {"count": 0}, "gaps": []}

    deltas = [(storage_ns[index] - storage_ns[index - 1]) / NSEC
              for index in range(1, len(storage_ns))]
    positive = [delta for delta in deltas if delta > 0.0]
    expected = statistics.median(positive) if positive else 0.0
    threshold = max(absolute_gap_sec, expected * gap_factor)
    header_deltas = [(header_ns[index] - header_ns[index - 1]) / NSEC
                     for index in range(1, len(header_ns))]
    positive_header = [delta for delta in header_deltas if delta > 0.0]
    expected_header = statistics.median(positive_header) if positive_header else 0.0
    header_threshold = max(absolute_gap_sec, expected_header * gap_factor)
    gaps = []
    header_gaps = []
    non_increasing = []
    for index, delta in enumerate(deltas, start=1):
        item = {
            "index": index,
            "previous_storage_sec": storage_ns[index - 1] / NSEC,
            "storage_sec": storage_ns[index] / NSEC,
            "storage_delta_sec": delta,
        }
        if index < len(header_ns):
            item.update({
                "previous_header_sec": header_ns[index - 1] / NSEC,
                "header_sec": header_ns[index] / NSEC,
                "header_delta_sec": (header_ns[index] - header_ns[index - 1]) / NSEC,
            })
        if delta <= 0.0:
            non_increasing.append(item)
        elif delta > threshold:
            item["estimated_missing_messages_by_storage_schedule"] = (
                max(0, int(round(delta / expected)) - 1) if expected > 0.0 else None
            )
            gaps.append(item)

    for index, delta in enumerate(header_deltas, start=1):
        if delta > header_threshold:
            header_gaps.append({
                "index": index,
                "previous_header_sec": header_ns[index - 1] / NSEC,
                "header_sec": header_ns[index] / NSEC,
                "header_delta_sec": delta,
                "estimated_missing_messages": (
                    max(0, int(round(delta / expected_header)) - 1)
                    if expected_header > 0.0 else None
                ),
            })
    return {
        "message_count": len(storage_ns),
        "expected_period_sec": expected,
        "estimated_rate_hz": 1.0 / expected if expected > 0.0 else 0.0,
        "gap_threshold_sec": threshold,
        "expected_header_period_sec": expected_header,
        "header_gap_threshold_sec": header_threshold,
        "storage_delta_stats_sec": finite_stats(deltas),
        "header_delta_stats_sec": finite_stats(header_deltas),
        "non_increasing_storage_count": len(non_increasing),
        "non_increasing_header_count": sum(delta <= 0.0 for delta in header_deltas),
        "gap_count": len(gaps),
        "header_gap_count": len(header_gaps),
        "estimated_missing_total": sum(
            gap.get("estimated_missing_messages") or 0 for gap in header_gaps
        ),
        "gaps": sorted(gaps, key=lambda item: item["storage_delta_sec"], reverse=True)[:limit],
        "header_gaps": sorted(
            header_gaps, key=lambda item: item["header_delta_sec"], reverse=True
        )[:limit],
        "non_increasing_storage": non_increasing[:limit],
    }


def clock_offset_report(storage_ns, header_ns):
    count = min(len(storage_ns), len(header_ns))
    offsets = [(header_ns[index] - storage_ns[index]) / NSEC for index in range(count)]
    return {
        "header_minus_storage_stats_sec": finite_stats(offsets),
        "header_minus_storage_drift_ppm": linear_drift_ppm(storage_ns[:count], offsets),
        "first_offset_sec": offsets[0] if offsets else None,
        "last_offset_sec": offsets[-1] if offsets else None,
    }


def point_time_view(cloud, field_name):
    matching = [field for field in cloud.fields if field.name == field_name]
    if not matching:
        raise ValueError(f"no point field named {field_name!r}")
    field = matching[0]
    if field.datatype != PointField.FLOAT64 or field.count != 1:
        raise ValueError(
            f"{field_name!r} must be FLOAT64 count=1, got "
            f"datatype={field.datatype} count={field.count}"
        )
    if np is None:
        raise RuntimeError("python3-numpy is required for full per-point timing audit")
    endian = ">" if cloud.is_bigendian else "<"
    return np.ndarray(
        shape=(cloud.height, cloud.width),
        dtype=np.dtype(endian + "f8"),
        buffer=cloud.data,
        offset=field.offset,
        strides=(cloud.row_step, cloud.point_step),
    )


def audit_bag(path, args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    metadata = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in metadata}
    for topic, expected_type in (
        (args.lidar_topic, "sensor_msgs/msg/PointCloud2"),
        (args.imu_topic, "sensor_msgs/msg/Imu"),
    ):
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
        if topic_types[topic] != expected_type:
            raise RuntimeError(
                f"{topic} has type {topic_types[topic]}, expected {expected_type}"
            )

    all_storage = defaultdict(list)
    lidar_storage = []
    lidar_header = []
    imu_storage = []
    imu_header = []
    point_span = []
    point_min_minus_header = []
    point_max_minus_header = []
    point_nonfinite = 0
    point_clouds_examined = 0
    point_errors = []
    total_records = 0

    while reader.has_next():
        topic, serialized, storage_timestamp = reader.read_next()
        storage_timestamp = int(storage_timestamp)
        total_records += 1
        all_storage[topic].append(storage_timestamp)

        if topic == args.lidar_topic:
            cloud = deserialize_message(serialized, PointCloud2)
            header = stamp_ns(cloud.header.stamp)
            lidar_storage.append(storage_timestamp)
            lidar_header.append(header)
            try:
                values = point_time_view(cloud, args.point_time_field)
                finite = np.isfinite(values)
                invalid = int(values.size - np.count_nonzero(finite))
                point_nonfinite += invalid
                if np.any(finite):
                    minimum = float(np.min(values[finite]))
                    maximum = float(np.max(values[finite]))
                    header_sec = header / NSEC
                    point_span.append(maximum - minimum)
                    point_min_minus_header.append(minimum - header_sec)
                    point_max_minus_header.append(maximum - header_sec)
                point_clouds_examined += 1
            except (ValueError, RuntimeError) as error:
                if len(point_errors) < 10:
                    point_errors.append(str(error))
            if len(lidar_storage) % 100 == 0:
                print(f"[{path.name}] audited {len(lidar_storage)} LiDAR clouds...", flush=True)

        elif topic == args.imu_topic:
            imu = deserialize_message(serialized, Imu)
            imu_storage.append(storage_timestamp)
            imu_header.append(stamp_ns(imu.header.stamp))

    lidar_delta = delta_report(
        lidar_storage, lidar_header, args.gap_factor, args.absolute_gap_sec
    )
    imu_delta = delta_report(
        imu_storage, imu_header, args.gap_factor, args.absolute_gap_sec
    )

    sorted_imu_header = sorted(imu_header)
    sorted_imu_by_storage = sorted(zip(imu_storage, imu_header))
    imu_storage_sorted = [item[0] for item in sorted_imu_by_storage]
    imu_header_by_storage = [item[1] for item in sorted_imu_by_storage]
    nearest_header_delta = []
    causal_latest_imu_delta = []
    storage_matched_lidar_minus_imu_header = []

    for lidar_storage_stamp, lidar_header_stamp in zip(lidar_storage, lidar_header):
        index = bisect_left(sorted_imu_header, lidar_header_stamp)
        candidates = []
        if index < len(sorted_imu_header):
            candidates.append(sorted_imu_header[index])
        if index > 0:
            candidates.append(sorted_imu_header[index - 1])
        if candidates:
            nearest = min(candidates, key=lambda stamp: abs(stamp - lidar_header_stamp))
            nearest_header_delta.append((nearest - lidar_header_stamp) / NSEC)

        storage_index = bisect_right(imu_storage_sorted, lidar_storage_stamp) - 1
        if storage_index >= 0:
            causal_latest_imu_delta.append(
                (imu_header_by_storage[storage_index] - lidar_header_stamp) / NSEC
            )

        near_storage_index = bisect_left(imu_storage_sorted, lidar_storage_stamp)
        storage_candidates = []
        if near_storage_index < len(imu_storage_sorted):
            storage_candidates.append(near_storage_index)
        if near_storage_index > 0:
            storage_candidates.append(near_storage_index - 1)
        if storage_candidates:
            closest_index = min(
                storage_candidates,
                key=lambda candidate: abs(imu_storage_sorted[candidate] - lidar_storage_stamp),
            )
            storage_matched_lidar_minus_imu_header.append(
                (lidar_header_stamp - imu_header_by_storage[closest_index]) / NSEC
            )

    maximum_recorded_gap = lidar_delta.get("storage_delta_stats_sec", {}).get("max", 0.0)
    expected_wall_gap = (
        maximum_recorded_gap / args.playback_rate if args.playback_rate > 0.0 else None
    )
    verdicts = []
    if expected_wall_gap is not None and expected_wall_gap >= args.watchdog_gap_sec:
        verdicts.append(
            "The bag contains a LiDAR storage gap large enough to trigger the observed "
            "watchdog delay at the selected playback rate."
        )
    else:
        verdicts.append(
            "No LiDAR storage gap is large enough to explain the observed watchdog delay "
            "at the selected playback rate; investigate rosbag I/O, DDS, duplicate nodes, "
            "or executor scheduling."
        )
    if lidar_delta.get("non_increasing_header_count", 0):
        verdicts.append("LiDAR header timestamps contain duplicates or backward jumps.")
    if point_span and statistics.median(point_span) < 1.0e-6:
        verdicts.append(
            "Per-point LiDAR timestamps have essentially zero median scan span; native "
            "deskew cannot recover motion within these clouds."
        )
    elif point_span:
        verdicts.append(
            "Per-point LiDAR timestamps contain a non-zero within-scan span suitable for "
            "deskew, subject to the reported header alignment."
        )

    report = {
        "bag": str(path),
        "total_records": total_records,
        "topic_types": topic_types,
        "topic_counts": {topic: len(stamps) for topic, stamps in all_storage.items()},
        "settings": {
            "lidar_topic": args.lidar_topic,
            "imu_topic": args.imu_topic,
            "point_time_field": args.point_time_field,
            "playback_rate": args.playback_rate,
            "watchdog_gap_sec": args.watchdog_gap_sec,
        },
        "lidar": {
            **lidar_delta,
            **clock_offset_report(lidar_storage, lidar_header),
            "maximum_expected_wall_gap_at_playback_rate_sec": expected_wall_gap,
            "point_timing": {
                "clouds_examined": point_clouds_examined,
                "errors": point_errors,
                "nonfinite_point_timestamp_count": point_nonfinite,
                "scan_span_stats_sec": finite_stats(point_span),
                "point_min_minus_header_stats_sec": finite_stats(point_min_minus_header),
                "point_max_minus_header_stats_sec": finite_stats(point_max_minus_header),
            },
        },
        "imu": {
            **imu_delta,
            **clock_offset_report(imu_storage, imu_header),
        },
        "cross_sensor": {
            "nearest_imu_header_minus_lidar_header_stats_sec": finite_stats(
                nearest_header_delta
            ),
            "latest_arrived_imu_header_minus_lidar_header_stats_sec": finite_stats(
                causal_latest_imu_delta
            ),
            "storage_matched_lidar_header_minus_imu_header_stats_sec": finite_stats(
                storage_matched_lidar_minus_imu_header
            ),
        },
        "verdicts": verdicts,
        "_storage_timestamps": dict(all_storage),
        "_lidar_headers": lidar_header,
        "_imu_headers": imu_header,
    }
    return report


def compare_reports(primary, comparison):
    topics = sorted(set(primary["topic_counts"]) | set(comparison["topic_counts"]))
    topic_results = {}
    for topic in topics:
        first = primary["_storage_timestamps"].get(topic, [])
        second = comparison["_storage_timestamps"].get(topic, [])
        topic_results[topic] = {
            "primary_count": len(first),
            "comparison_count": len(second),
            "counts_equal": len(first) == len(second),
            "storage_timestamps_identical": first == second,
        }

    def header_shift(first, second):
        count = min(len(first), len(second))
        shifts = [(second[index] - first[index]) / NSEC for index in range(count)]
        return finite_stats(shifts)

    all_counts_equal = all(item["counts_equal"] for item in topic_results.values())
    all_storage_identical = all(
        item["storage_timestamps_identical"] for item in topic_results.values()
    )
    return {
        "all_topic_counts_equal": all_counts_equal,
        "all_storage_timestamps_identical": all_storage_identical,
        "lidar_header_shift_comparison_minus_primary_sec": header_shift(
            primary["_lidar_headers"], comparison["_lidar_headers"]
        ),
        "imu_header_shift_comparison_minus_primary_sec": header_shift(
            primary["_imu_headers"], comparison["_imu_headers"]
        ),
        "topics": topic_results,
    }


def strip_internal(report):
    return {key: value for key, value in report.items() if not key.startswith("_")}


def print_summary(report):
    lidar = report["lidar"]
    imu = report["imu"]
    cross = report["cross_sensor"]
    print(f"\n=== Bag timing audit: {report['bag']} ===")
    print(f"Records: {report['total_records']}")
    print(
        f"LiDAR: {lidar['message_count']} messages, "
        f"{lidar.get('estimated_rate_hz', 0.0):.3f} Hz, "
        f"{lidar.get('gap_count', 0)} storage delivery gaps, "
        f"{lidar.get('header_gap_count', 0)} sensor-header gaps, "
        f"{lidar.get('estimated_missing_total', 0)} estimated truly missing frames"
    )
    lidar_max = lidar.get("storage_delta_stats_sec", {}).get("max", 0.0)
    wall_max = lidar.get("maximum_expected_wall_gap_at_playback_rate_sec")
    print(
        f"Largest LiDAR storage gap: {lidar_max:.6f} s; expected wall gap at "
        f"{report['settings']['playback_rate']}x: {wall_max:.6f} s"
    )
    print(
        f"IMU: {imu['message_count']} messages, "
        f"{imu.get('estimated_rate_hz', 0.0):.3f} Hz, "
        f"{imu.get('gap_count', 0)} recorded gaps"
    )
    sensor_offset = cross[
        "storage_matched_lidar_header_minus_imu_header_stats_sec"
    ]
    print(
        "Storage-matched LiDAR header minus IMU header: "
        f"median={sensor_offset.get('median', float('nan')):.6f} s, "
        f"min={sensor_offset.get('min', float('nan')):.6f} s, "
        f"max={sensor_offset.get('max', float('nan')):.6f} s"
    )
    point_span_stats = lidar["point_timing"]["scan_span_stats_sec"]
    print(
        "Per-cloud point timestamp span: "
        f"median={point_span_stats.get('median', float('nan')):.9f} s, "
        f"max={point_span_stats.get('max', float('nan')):.9f} s"
    )
    if lidar.get("gaps"):
        print("Largest recorded LiDAR gaps:")
        for gap in lidar["gaps"][:5]:
            print(
                f"  index={gap['index']} storage_delta={gap['storage_delta_sec']:.6f}s "
                f"header_delta={gap.get('header_delta_sec', float('nan')):.6f}s "
                "storage_schedule_slots="
                f"{gap.get('estimated_missing_messages_by_storage_schedule')}"
            )
    print("Verdict:")
    for verdict in report["verdicts"]:
        print(f"  - {verdict}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Bag directory to audit")
    parser.add_argument(
        "--compare-bag", type=Path,
        help="Optional original/corrected bag to audit and compare record-for-record",
    )
    parser.add_argument("--lidar-topic", default="/iv_points")
    parser.add_argument("--imu-topic", default="/husky/sensors/imu_0/data")
    parser.add_argument("--point-time-field", default="timestamp")
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument(
        "--watchdog-gap-sec", type=float, default=1.0,
        help="Observed wall-clock watchdog delay to test against (default: 1.0)",
    )
    parser.add_argument("--gap-factor", type=float, default=3.0)
    parser.add_argument("--absolute-gap-sec", type=float, default=0.25)
    parser.add_argument("--json", type=Path, help="Write the complete report as JSON")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.playback_rate <= 0.0:
        print("--playback-rate must be positive", file=sys.stderr)
        return 2
    for path in [args.bag, args.compare_bag]:
        if path is not None and not path.exists():
            print(f"Bag does not exist: {path}", file=sys.stderr)
            return 2

    try:
        primary = audit_bag(args.bag, args)
        print_summary(primary)
        comparison = None
        comparison_result = None
        if args.compare_bag:
            comparison = audit_bag(args.compare_bag, args)
            print_summary(comparison)
            comparison_result = compare_reports(primary, comparison)
            print("\n=== Bag comparison ===")
            print(f"All topic counts equal: {comparison_result['all_topic_counts_equal']}")
            print(
                "All rosbag storage timestamps identical: "
                f"{comparison_result['all_storage_timestamps_identical']}"
            )
            shift = comparison_result[
                "lidar_header_shift_comparison_minus_primary_sec"
            ]
            print(
                "LiDAR header shift (comparison - primary): "
                f"median={shift.get('median', float('nan')):.9f}s "
                f"stddev={shift.get('stddev', float('nan')):.9f}s"
            )

        if args.json:
            output = {"primary": strip_internal(primary)}
            if comparison is not None:
                output["comparison"] = strip_internal(comparison)
                output["bag_comparison"] = comparison_result
            args.json.write_text(json.dumps(output, indent=2), encoding="utf-8")
            print(f"Full JSON report written to {args.json}")
    except Exception as error:  # Keep CLI failures concise while preserving type.
        print(f"Audit failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
