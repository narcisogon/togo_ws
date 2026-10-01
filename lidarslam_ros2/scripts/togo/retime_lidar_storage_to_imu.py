#!/usr/bin/env python3
"""Normalize LiDAR rosbag storage times against the recorded IMU clock.

This tool is intended to run *after* LiDAR message/header timestamps have been
corrected.  It does not alter any serialized message payload.  It estimates the
stable IMU ``header.stamp - storage_timestamp`` offset, then schedules each
LiDAR record from its corrected header using that same offset.  All other
topics retain their original storage timestamps.

Records are buffered in a bounded heap and written chronologically, so moving
a delayed LiDAR record earlier does not create an out-of-order output stream.
"""

import argparse
from collections import defaultdict
import heapq
from pathlib import Path
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


NSEC = 1_000_000_000


def stamp_ns(stamp):
    return int(stamp.sec) * NSEC + int(stamp.nanosec)


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def delta_stats(stamps):
    deltas = [
        (stamps[index] - stamps[index - 1]) / NSEC
        for index in range(1, len(stamps))
    ]
    if not deltas:
        return {"count": 0}
    ordered = sorted(deltas)
    return {
        "count": len(deltas),
        "min": ordered[0],
        "median": statistics.median(ordered),
        "max": ordered[-1],
        "over_0_25_sec": sum(delta > 0.25 for delta in deltas),
        "non_increasing": sum(delta <= 0.0 for delta in deltas),
    }


def first_pass(args):
    reader = open_reader(args.input_bag)
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    required = {
        args.lidar_topic: "sensor_msgs/msg/PointCloud2",
        args.imu_topic: "sensor_msgs/msg/Imu",
    }
    for topic, expected_type in required.items():
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
        if topic_types[topic] != expected_type:
            raise RuntimeError(
                f"{topic} has type {topic_types[topic]}, expected {expected_type}"
            )

    imu_offsets = []
    lidar_records = []
    total_records = 0
    while reader.has_next():
        topic, serialized, storage_timestamp = reader.read_next()
        storage_timestamp = int(storage_timestamp)
        total_records += 1
        if topic == args.imu_topic:
            message = deserialize_message(serialized, Imu)
            imu_offsets.append(stamp_ns(message.header.stamp) - storage_timestamp)
        elif topic == args.lidar_topic:
            message = deserialize_message(serialized, PointCloud2)
            lidar_records.append((storage_timestamp, stamp_ns(message.header.stamp)))
            if len(lidar_records) % 100 == 0:
                print(f"First pass: inspected {len(lidar_records)} LiDAR clouds...", flush=True)

    if not imu_offsets:
        raise RuntimeError("no IMU messages were found")
    if not lidar_records:
        raise RuntimeError("no LiDAR messages were found")

    reference_offset_ns = int(round(statistics.median(imu_offsets)))
    old_lidar_storage = [record[0] for record in lidar_records]
    new_lidar_storage = [record[1] - reference_offset_ns for record in lidar_records]
    shifts = [new - old for old, new in zip(old_lidar_storage, new_lidar_storage)]
    max_backward_ns = max(0, max(-shift for shift in shifts))

    return {
        "topics": topics,
        "total_records": total_records,
        "imu_count": len(imu_offsets),
        "lidar_count": len(lidar_records),
        "reference_offset_ns": reference_offset_ns,
        "max_backward_ns": max_backward_ns,
        "old_lidar_storage": old_lidar_storage,
        "new_lidar_storage": new_lidar_storage,
        "shifts": shifts,
    }


def write_retimed_bag(args, analysis):
    reader = open_reader(args.input_bag)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output_bag), storage_id=args.storage_id
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in analysis["topics"]:
        writer.create_topic(topic)

    # A future record read at original time T cannot move earlier than
    # T-max_backward.  Anything older than that watermark is safe to flush.
    heap = []
    sequence = 0
    lidar_index = 0
    written = 0
    counts = defaultdict(int)
    max_backward_ns = analysis["max_backward_ns"]

    while reader.has_next():
        topic, serialized, original_storage = reader.read_next()
        original_storage = int(original_storage)
        if topic == args.lidar_topic:
            new_storage = analysis["new_lidar_storage"][lidar_index]
            lidar_index += 1
        else:
            new_storage = original_storage

        heapq.heappush(
            heap,
            (new_storage, sequence, topic, serialized),
        )
        sequence += 1
        watermark = original_storage - max_backward_ns - 1
        while heap and heap[0][0] <= watermark:
            timestamp, _, queued_topic, queued_serialized = heapq.heappop(heap)
            writer.write(queued_topic, queued_serialized, timestamp)
            counts[queued_topic] += 1
            written += 1

        if lidar_index and lidar_index % 100 == 0 and topic == args.lidar_topic:
            print(f"Second pass: retimed {lidar_index} LiDAR clouds...", flush=True)

    while heap:
        timestamp, _, topic, serialized = heapq.heappop(heap)
        writer.write(topic, serialized, timestamp)
        counts[topic] += 1
        written += 1

    if lidar_index != analysis["lidar_count"]:
        raise RuntimeError(
            f"LiDAR count changed between passes: {lidar_index} != "
            f"{analysis['lidar_count']}"
        )
    if written != analysis["total_records"]:
        raise RuntimeError(
            f"record count changed while writing: {written} != "
            f"{analysis['total_records']}"
        )
    return counts


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--lidar-topic", default="/iv_points")
    parser.add_argument("--imu-topic", default="/husky/sensors/imu_0/data")
    parser.add_argument("--storage-id", default="mcap")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.input_bag.exists():
        print(f"Input bag does not exist: {args.input_bag}", file=sys.stderr)
        return 2
    if args.output_bag.exists():
        print(
            f"Refusing to overwrite existing output: {args.output_bag}",
            file=sys.stderr,
        )
        return 2

    try:
        analysis = first_pass(args)
        old_stats = delta_stats(analysis["old_lidar_storage"])
        new_stats = delta_stats(analysis["new_lidar_storage"])
        shift_sec = [shift / NSEC for shift in analysis["shifts"]]
        print("\nTiming plan:")
        print(
            "  IMU median header-minus-storage offset: "
            f"{analysis['reference_offset_ns'] / NSEC:.9f} s"
        )
        print(
            "  LiDAR storage shift: "
            f"median={statistics.median(shift_sec):+.9f} s "
            f"min={min(shift_sec):+.9f} s max={max(shift_sec):+.9f} s"
        )
        print(
            f"  Old LiDAR max storage gap: {old_stats.get('max', 0.0):.9f} s "
            f"({old_stats.get('over_0_25_sec', 0)} gaps > 0.25 s)"
        )
        print(
            f"  New LiDAR max storage gap: {new_stats.get('max', 0.0):.9f} s "
            f"({new_stats.get('over_0_25_sec', 0)} gaps > 0.25 s)"
        )
        if new_stats.get("non_increasing", 0):
            raise RuntimeError(
                "corrected LiDAR headers are not strictly increasing; refusing to write"
            )

        counts = write_retimed_bag(args, analysis)
        print(
            f"\nDone: wrote {sum(counts.values())} records to {args.output_bag}"
        )
        print(
            f"Retimed {counts[args.lidar_topic]} LiDAR records; message payloads "
            "and all non-LiDAR storage timestamps were left unchanged."
        )
    except Exception as error:
        print(f"Retiming failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

