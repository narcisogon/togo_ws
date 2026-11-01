#!/usr/bin/env python3
"""Ordering of the time-carrying messages in a bag as the player will replay them (storage order): backward steps
of /clock and of the stamps of a few topics, which make tf2 clear its buffer ("Detected jump back in time").

usage: clock_order.py BAG_DIR [TOPIC ...]      (default topics: /clock /tf /husky/sensors/seyond/points
                                                /husky/sensors/imu_0/data_raw /husky/sensors/stereo_rig/imu/data_raw)
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
topics = sys.argv[2:] or ["/clock", "/tf", "/husky/sensors/seyond/points", "/husky/sensors/imu_0/data_raw",
                          "/husky/sensors/stereo_rig/imu/data_raw"]
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
cls = {k: get_message(types[k]) for k in topics if k in types}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(cls)))
last, back, count, worst = {}, {}, {}, {}
while reader.has_next():
    topic, data, t_rec = reader.read_next()
    m = deserialize_message(data, cls[topic])
    if topic == "/clock":
        t = m.clock.sec + m.clock.nanosec * 1e-9
    elif topic == "/tf":
        t = max(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9 for tr in m.transforms)
    else:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    count[topic] = count.get(topic, 0) + 1
    if topic in last and t < last[topic]:
        back[topic] = back.get(topic, 0) + 1
        worst[topic] = max(worst.get(topic, 0.0), last[topic] - t)
    last[topic] = max(last.get(topic, t), t)
for k in cls:
    print(f"{k:45s} messages {count.get(k, 0):7d}  backward steps {back.get(k, 0):5d}  largest {worst.get(k, 0.0) * 1000:7.1f} ms")
