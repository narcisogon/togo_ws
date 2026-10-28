#!/usr/bin/env python3
"""Readiness checks for scripts/replay_slam_bag.sh.

Run with the system python3, ROS 2 Jazzy sourced, on the SLAM's ROS domain (29).

  precheck   exit 1 if something already publishes /clock on this domain (a running simulation or bag) or a DLIO
             SLAM is already running: either would mix with the replay.
  wait       exit 0 once the SLAM in the container can take data: dlio_odom_node subscribed to the LiDAR and IMU
             topics and graph_based_slam to /dlio/odometry. Node names are checked, not subscriber counts, because
             another node of the SLAM launch also subscribes to the LiDAR. Exit 1 on --timeout, exit 2 as soon as
             --watch-pid (the local `docker compose exec` that runs the SLAM) has exited.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node

LIDAR = "/husky/sensors/seyond/points"
IMU = "/husky/sensors/imu_0/data_raw"


def alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["precheck", "wait"])
    ap.add_argument("--timeout", type=float, default=180.0, help="wait: seconds (default 180)")
    ap.add_argument("--watch-pid", type=int, default=0, help="wait: give up when this process has exited")
    ap.add_argument("--lidar-topic", default=LIDAR)
    ap.add_argument("--imu-topic", default=IMU)
    a = ap.parse_args()
    needed = ((a.lidar_topic, "dlio_odom_node"), (a.imu_topic, "dlio_odom_node"),
              ("/dlio/odometry", "graph_based_slam"))

    rclpy.init()
    node = Node("replay_slam_check")
    try:
        if a.mode == "precheck":
            t0 = time.time()
            while time.time() - t0 < 3.0:                     # let DDS discovery fill the graph
                rclpy.spin_once(node, timeout_sec=0.2)
            problems = []
            if node.count_publishers("/clock"):
                problems.append("something already publishes /clock on this ROS domain (a running simulation "
                                "or bag replay): stop it first")
            if "dlio_odom_node" in node.get_node_names():
                problems.append("a DLIO SLAM is already running on this ROS domain: stop it first")
            for p in problems:
                print(f"ERROR: {p}", file=sys.stderr)
            return 1 if problems else 0

        t0 = time.time()
        shown = -1e9
        while time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            if a.watch_pid and not alive(a.watch_pid):
                print("ERROR: the SLAM process exited before it was ready", file=sys.stderr)
                return 2
            missing = [f"{name} on {topic}" for topic, name in needed
                       if not any(i.node_name == name for i in node.get_subscriptions_info_by_topic(topic))]
            if not missing:
                print(f"SLAM ready after {time.time() - t0:.1f} s (dlio_odom_node on LiDAR + IMU, "
                      f"graph_based_slam on /dlio/odometry)", flush=True)
                return 0
            if time.time() - shown >= 10:
                shown = time.time()
                print(f"  waiting for SLAM ({time.time() - t0:.0f} s): {', '.join(missing)}", flush=True)
        print(f"ERROR: SLAM not subscribed after {a.timeout:.0f} s", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
