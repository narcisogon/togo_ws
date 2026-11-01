#!/usr/bin/env python3
"""Rewrite a recorded simulation bag so that it plays in simulation-time order and at simulation-time pace.

A recording under load (the stereo rig's images at a real-time factor near 0.4) stores messages in the order the
recorder received them, up to a quarter of a second out of order (clock_order.py counts it), and with the wall
clock of the recording as their log time. Played back, that gives /clock steps backwards (tf2 clears its buffer),
LiDAR scans that DLIO drops as non-increasing, IMU samples with negative dt, and a replay that crawls at the
recording's real-time factor. This tool sets every message's log time to its own stamp (the simulation time:
/clock's value, a header's stamp, the latest stamp of a /tf message) and writes the messages in that order,
using a sliding window since the reordering is bounded.

usage: retime_bag.py IN_BAG OUT_BAG [--window-s 1.0]
ground_truth.tum, segments.json, check.json and check.png are copied along (they are in simulation time already).
"""
import argparse
import heapq
import shutil
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(topic, msg):
    if topic == "/clock":
        return msg.clock.sec * 10**9 + msg.clock.nanosec
    if topic in ("/tf", "/tf_static"):
        return max(tr.header.stamp.sec * 10**9 + tr.header.stamp.nanosec for tr in msg.transforms)
    h = getattr(msg, "header", None)
    if h is None:
        return None
    return h.stamp.sec * 10**9 + h.stamp.nanosec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("in_bag", type=Path)
    ap.add_argument("out_bag", type=Path)
    ap.add_argument("--window-s", type=float, default=1.0, help="reordering bound, simulation seconds")
    a = ap.parse_args()
    if a.out_bag.exists():
        raise SystemExit(f"exists: {a.out_bag}")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(a.in_bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    metas = r.get_all_topics_and_types()
    cls = {m.name: get_message(m.type) for m in metas}
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(a.out_bag), storage_id="mcap", storage_preset_profile="zstd_fast"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for m in metas:
        w.create_topic(m)
    window = int(a.window_s * 1e9)
    heap, seq = [], 0
    newest, written, no_stamp, last_written = 0, 0, 0, -1
    backward_after = 0
    counts = {}

    def flush(limit):
        nonlocal written, last_written, backward_after
        while heap and heap[0][0] <= limit:
            key, _, topic, data = heapq.heappop(heap)
            if key < last_written:
                backward_after += 1
            last_written = max(last_written, key)
            w.write(topic, data, key)
            written += 1
            counts[topic] = counts.get(topic, 0) + 1

    pending = []                                     # stamp-less messages seen before the first stamped one
    while r.has_next():
        topic, data, t_rec = r.read_next()
        key = stamp_ns(topic, deserialize_message(data, cls[topic]))
        if key is None or key <= 0:
            no_stamp += 1
            if newest == 0:
                pending.append((topic, data))        # e.g. /tf_static with zero stamps at the very start
                continue
            key = newest                             # rides on the current simulation time
        if newest == 0:
            for ptopic, pdata in pending:            # the held messages go first, at the first simulation time
                seq += 1
                heapq.heappush(heap, (key, seq, ptopic, pdata))
            pending = []
        newest = max(newest, key)
        seq += 1
        heapq.heappush(heap, (key, seq, topic, data))
        flush(newest - window)
    flush(newest + 1)
    del w
    print(f"wrote {a.out_bag}: {written} messages, {no_stamp} without a stamp, "
          f"{backward_after} still out of order (window {a.window_s} s)")
    for k in sorted(counts):
        print(f"  {k}: {counts[k]}")
    for f in ("ground_truth.tum", "segments.json", "check.json", "check.png"):
        if (a.in_bag / f).is_file():
            shutil.copy2(a.in_bag / f, a.out_bag / f)


if __name__ == "__main__":
    main()
