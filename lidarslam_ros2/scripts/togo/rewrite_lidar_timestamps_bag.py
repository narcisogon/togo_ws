#!/usr/bin/env python3
"""Copy a ROS 2 bag while applying a fixed offset to LiDAR timestamps.

All records and their rosbag storage timestamps are preserved.  Only the
selected PointCloud2 topic is deserialized and modified.  Both header.stamp
and every value in the selected FLOAT64 point field receive the same offset.
"""

import argparse
import math
from pathlib import Path
import struct
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField

try:
    import numpy as np
except ImportError:  # pragma: no cover - ROS images normally include numpy.
    np = None


NSEC_PER_SEC = 1_000_000_000


def shifted_stamp(stamp, offset_sec: float) -> None:
    total_ns = stamp.sec * NSEC_PER_SEC + stamp.nanosec
    total_ns += int(round(offset_sec * NSEC_PER_SEC))
    if total_ns < 0:
        raise ValueError("timestamp offset produced a time before zero")
    stamp.sec, stamp.nanosec = divmod(total_ns, NSEC_PER_SEC)


def find_float64_field(cloud: PointCloud2, name: str):
    matches = [field for field in cloud.fields if field.name == name]
    if not matches:
        raise ValueError(f"PointCloud2 has no {name!r} field")
    field = matches[0]
    if field.datatype != PointField.FLOAT64 or field.count != 1:
        raise ValueError(
            f"field {name!r} must be FLOAT64 count=1; got "
            f"datatype={field.datatype}, count={field.count}"
        )
    if field.offset + 8 > cloud.point_step:
        raise ValueError(f"field {name!r} extends beyond point_step")
    return field


def shift_point_times(cloud: PointCloud2, field_name: str, offset_sec: float) -> None:
    field = find_float64_field(cloud, field_name)
    expected_size = cloud.row_step * cloud.height
    if len(cloud.data) < expected_size:
        raise ValueError(
            f"cloud data is truncated: {len(cloud.data)} < {expected_size} bytes"
        )

    data = bytearray(cloud.data)
    endian = ">" if cloud.is_bigendian else "<"

    if np is not None:
        values = np.ndarray(
            shape=(cloud.height, cloud.width),
            dtype=np.dtype(endian + "f8"),
            buffer=data,
            offset=field.offset,
            strides=(cloud.row_step, cloud.point_step),
        )
        if not np.all(np.isfinite(values)):
            raise ValueError(f"field {field_name!r} contains non-finite timestamps")
        values += offset_sec
    else:
        fmt = endian + "d"
        for row in range(cloud.height):
            row_start = row * cloud.row_step
            for column in range(cloud.width):
                offset = row_start + column * cloud.point_step + field.offset
                value = struct.unpack_from(fmt, data, offset)[0]
                if not math.isfinite(value):
                    raise ValueError(
                        f"field {field_name!r} contains a non-finite timestamp"
                    )
                struct.pack_into(fmt, data, offset, value + offset_sec)

    cloud.data = data


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--topic", default="/iv_points")
    parser.add_argument("--point-time-field", default="timestamp")
    parser.add_argument(
        "--offset-sec",
        type=float,
        required=True,
        help="Seconds added to LiDAR timestamps; use -6.0 to subtract six seconds.",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="Output rosbag storage plugin (default: mcap).",
    )
    return parser.parse_args()


def main() -> int:
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

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.input_bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    if args.topic not in topic_types:
        print(f"Topic not found in bag: {args.topic}", file=sys.stderr)
        return 2
    if topic_types[args.topic] != "sensor_msgs/msg/PointCloud2":
        print(
            f"{args.topic} is {topic_types[args.topic]}, not PointCloud2",
            file=sys.stderr,
        )
        return 2

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(args.output_bag), storage_id=args.storage_id
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic in topics:
        writer.create_topic(topic)

    cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    corrected = 0
    total = 0
    while reader.has_next():
        topic, serialized, storage_timestamp = reader.read_next()
        total += 1
        if topic == args.topic:
            cloud = deserialize_message(serialized, cloud_type)
            shifted_stamp(cloud.header.stamp, args.offset_sec)
            shift_point_times(cloud, args.point_time_field, args.offset_sec)
            serialized = serialize_message(cloud)
            corrected += 1
            if corrected % 100 == 0:
                print(f"Corrected {corrected} LiDAR clouds...", flush=True)
        writer.write(topic, serialized, storage_timestamp)

    if corrected == 0:
        print("No LiDAR records were corrected.", file=sys.stderr)
        return 1
    print(
        f"Done: copied {total} records and corrected {corrected} LiDAR clouds "
        f"by {args.offset_sec:+.9f} s into {args.output_bag}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

