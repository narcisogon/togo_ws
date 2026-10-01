#!/usr/bin/env python3
"""Receive one /modified_map_timed cloud and summarize its time/index fields."""

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def field_by_name(message, name):
    return next((field for field in message.fields if field.name == name), None)


def strided_values(message, field, dtype):
    count = int(message.width) * int(message.height)
    return np.ndarray(
        shape=(count,),
        dtype=np.dtype(dtype),
        buffer=message.data,
        offset=field.offset,
        strides=(message.point_step,),
    )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/modified_map_timed")
    parser.add_argument("--timeout", type=float, default=7.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node("timed_map_diag_once")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    received = []
    subscription = node.create_subscription(
        PointCloud2, args.topic, lambda message: received.append(message), qos
    )
    del subscription  # The node owns the subscription.
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not received:
            print(f"ERROR: no {args.topic} message received within {args.timeout:.1f}s")
            return 1

        message = received[0]
        time_field = field_by_name(message, "time")
        index_field = field_by_name(message, "submap_index")
        if time_field is None or index_field is None:
            names = [field.name for field in message.fields]
            print(f"ERROR: required fields missing; fields={names}")
            return 1
        if time_field.datatype == PointField.FLOAT64:
            time_dtype = ">f8" if message.is_bigendian else "<f8"
            precision = "FLOAT64"
        elif time_field.datatype == PointField.FLOAT32:
            time_dtype = ">f4" if message.is_bigendian else "<f4"
            precision = "FLOAT32"
        else:
            print(f"ERROR: unsupported time datatype={time_field.datatype}")
            return 1
        if index_field.datatype != PointField.UINT32:
            print(f"ERROR: submap_index datatype={index_field.datatype}, expected UINT32")
            return 1

        times = strided_values(message, time_field, time_dtype).astype(np.float64)
        index_dtype = ">u4" if message.is_bigendian else "<u4"
        indices = strided_values(message, index_field, index_dtype)
        finite = times[np.isfinite(times)]
        if finite.size:
            time_min = float(np.min(finite))
            time_max = float(np.max(finite))
            time_span = time_max - time_min
        else:
            time_min = time_max = time_span = math.nan
        unique_indices = np.unique(indices)
        print(
            f"OK: topic={args.topic} points={times.size} point_step={message.point_step} "
            f"time_type={precision} time_min={time_min:.9f} time_max={time_max:.9f} "
            f"time_span={time_span:.6f}s submaps={unique_indices.size} "
            f"submap_min={int(unique_indices[0]) if unique_indices.size else -1} "
            f"submap_max={int(unique_indices[-1]) if unique_indices.size else -1}"
        )
        if precision == "FLOAT32" and time_max > 1.0e8:
            print("WARNING: FLOAT32 absolute epoch time is too imprecise for a short replay")
            return 2
        if unique_indices.size > 1 and time_span <= 0.0:
            print("WARNING: multiple submaps have collapsed onto one timestamp")
            return 2
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
