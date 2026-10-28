#!/usr/bin/env python3
"""Cut the start of a recorded simulation bag into a new bag (system python3 with ROS 2 sourced: rosbag2_py).

Usage: trim_bag.py IN_BAG OUT_BAG (--until-sim S | --laps N) [--segments ROUTE_JSON]

Keeps every message received until /clock reached S seconds of sim time (all topics, their recorded QoS, so
/tf_static stays latched), writes an indexed MCAP with zstd chunks, and copies the ground truth (ground_truth.tum)
up to S. --laps N finds S itself: LAP_MARGIN_S after the rover's Nth closest pass by its start pose (from the
input bag's ground_truth.tum). With --segments it also writes segments.json: the sim-time spans of the route's
legs (start -> rocks -> slope -> plain -> start) from the ground truth's closest approach to each waypoint, for
per-terrain scores.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

LAP_MARGIN_S = 3.0        # kept after the closest pass by the start, so the pass itself is in the bag


def lap_ends(gt: np.ndarray, away_m: float = 20.0, near_m: float = 2.0) -> list[float]:
    """Sim times of the closest passes by the start position, one per completed lap: the rover first gets more
    than away_m from the start, then comes back within near_m. gt rows: t x y z qx qy qz qw."""
    d = np.hypot(gt[:, 1] - gt[0, 1], gt[:, 2] - gt[0, 2])
    ends, i = [], 0
    while True:
        away = np.nonzero(d[i:] > away_m)[0]
        if not len(away):
            return ends
        back = np.nonzero(d[i + away[0]:] < near_m)[0]
        if not len(back):
            return ends
        b = i + away[0] + back[0]
        leave = np.nonzero(d[b:] >= near_m)[0]
        e = b + (int(leave[0]) if len(leave) else len(d) - b)
        ends.append(float(gt[b + int(np.argmin(d[b:e])), 0]))
        i = e


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("in_bag", type=Path)
    ap.add_argument("out_bag", type=Path)
    cut_at = ap.add_mutually_exclusive_group(required=True)
    cut_at.add_argument("--until-sim", type=float, help="sim time (s) of the last message kept")
    cut_at.add_argument("--laps", type=int, help="keep this many laps (the end found from the ground truth)")
    ap.add_argument("--segments", type=Path, help="route.json of the recording (waypoints)")
    a = ap.parse_args()
    if a.out_bag.exists():
        raise SystemExit(f"exists: {a.out_bag}")
    if a.laps is not None:
        gt_in = a.in_bag / "ground_truth.tum"
        if not gt_in.is_file():
            raise SystemExit(f"--laps needs {gt_in}")
        ends = lap_ends(np.loadtxt(gt_in, comments="#"))
        if not 1 <= a.laps <= len(ends):
            raise SystemExit(f"--laps {a.laps}: the ground truth has {len(ends)} complete lap(s)")
        a.until_sim = ends[a.laps - 1] + LAP_MARGIN_S
        print(f"lap {a.laps} ends at sim {ends[a.laps - 1]:.2f} s (closest pass by the start); cutting at "
              f"{a.until_sim:.2f} s")

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock

    # 1. bag time at which /clock passes --until-sim
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(a.in_bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=["/clock"]))
    cut = None
    while r.has_next():
        _, data, t = r.read_next()
        c = deserialize_message(data, Clock).clock
        if c.sec + c.nanosec * 1e-9 >= a.until_sim:
            cut = t
            break
    if cut is None:
        raise SystemExit(f"/clock never reaches {a.until_sim} s")
    del r

    # 2. copy everything up to the cut
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(a.in_bag), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(a.out_bag), storage_id="mcap", storage_preset_profile="zstd_fast"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for meta in r.get_all_topics_and_types():
        w.create_topic(meta)
    counts: dict[str, int] = {}
    while r.has_next():
        topic, data, t = r.read_next()
        if t > cut:
            break
        w.write(topic, data, t)
        counts[topic] = counts.get(topic, 0) + 1
    del w
    print(f"wrote {a.out_bag}: {sum(counts.values())} messages up to sim {a.until_sim:.1f} s")
    for k in sorted(counts):
        print(f"  {k}: {counts[k]}")

    # 3. ground truth and route segments
    gt_in = a.in_bag / "ground_truth.tum"
    if gt_in.is_file():
        gt = np.loadtxt(gt_in, comments="#")
        gt = gt[gt[:, 0] <= a.until_sim]
        np.savetxt(a.out_bag / "ground_truth.tum", gt, fmt="%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f",
                   header="timestamp(sim s) x y z qx qy qz qw  -- world frame, base_link of 'togo' (gz ground truth)")
        if a.segments and a.segments.is_file():
            route = json.loads(a.segments.read_text())
            p = gt[:, 1:3]
            v = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(np.diff(gt[:, 0]), 1e-6)]
            t_move = float(gt[np.argmax(v > 0.02), 0])
            marks = [("start", t_move)]
            lo = int(np.argmax(v > 0.02))
            for name in ("rocks", "slope", "plain"):
                w_xy = np.array(route["waypoints"][name])
                k = lo + int(np.argmin(np.linalg.norm(p[lo:] - w_xy, axis=1)))
                marks.append((name, float(gt[k, 0])))
                lo = k
            marks.append(("end", float(gt[-1, 0])))
            segs = {"standstill": [float(gt[0, 0]), t_move]}
            names = ["to_rocks", "rocks_to_slope", "slope_to_plain", "plain_to_start"]
            for name, (m0, m1) in zip(names, zip(marks[:-1], marks[1:])):
                segs[name] = [m0[1], m1[1]]
            (a.out_bag / "segments.json").write_text(json.dumps(segs, indent=1))
            print("segments (sim s):", {k: [round(x, 1) for x in v] for k, v in segs.items()})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
