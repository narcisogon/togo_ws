#!/usr/bin/env python3
"""Check the recorded benchmark bag and export ground truth (system python3 with ROS 2 sourced).

Usage: check_bag.py BAG_DIR ROUTE_JSON
Writes into BAG_DIR: ground_truth.tum (sim time, world frame), check.json, check.png.
"""
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = Path(sys.argv[1])
ROUTE = json.load(open(sys.argv[2]))

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(BAG), storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
want = {"/ground_truth/odom", "/husky/sensors/imu_0/data_raw", "/husky/sensors/seyond/points", "/clock"}
cls = {t: get_message(types[t]) for t in want if t in types}

t_bag = defaultdict(list)          # bag receive time per topic (s)
t_hdr = defaultdict(list)          # header stamp (sim time) for the topics we decode
gt, imu, lidar_finite, clock = [], [], [], []
while reader.has_next():
    topic, data, t = reader.read_next()
    t_bag[topic].append(t * 1e-9)
    if topic not in cls:
        continue
    m = deserialize_message(data, cls[topic])
    if topic == "/clock":
        clock.append(m.clock.sec + m.clock.nanosec * 1e-9)
        continue
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    t_hdr[topic].append(st)
    if topic == "/ground_truth/odom":
        p, q = m.pose.pose.position, m.pose.pose.orientation
        gt.append((st, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    elif topic == "/husky/sensors/imu_0/data_raw":
        a, w = m.linear_acceleration, m.angular_velocity
        imu.append((st, a.x, a.y, a.z, w.x, w.y, w.z))
    elif topic == "/husky/sensors/seyond/points":
        lidar_finite.append((st, m.width * m.height))

out = {"bag": str(BAG), "topics": {}}
for topic in sorted(t_bag):
    tb = np.array(t_bag[topic])
    entry = {"type": types.get(topic), "count": int(len(tb))}
    if len(tb) > 1:
        entry["rate_hz_bag_time"] = round((len(tb) - 1) / (tb[-1] - tb[0]), 2)
        entry["max_gap_s_bag_time"] = round(float(np.max(np.diff(tb))), 3)
    if topic in t_hdr and len(t_hdr[topic]) > 1:
        th = np.array(t_hdr[topic])
        entry["rate_hz_sim_time"] = round((len(th) - 1) / (th[-1] - th[0]), 2)
        entry["max_gap_s_sim_time"] = round(float(np.max(np.diff(th))), 3)
    out["topics"][topic] = entry
all_t = np.concatenate([np.array(v) for v in t_bag.values()])
out["duration_bag_s"] = round(float(all_t.max() - all_t.min()), 1)
if clock:
    out["duration_sim_s"] = round(clock[-1] - clock[0], 1)
    out["mean_rtf"] = round(out["duration_sim_s"] / out["duration_bag_s"], 3)

gt = np.array(gt)
imu = np.array(imu)
# ---------------------------------------------------------------- standstill at the start
v = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2])) / np.maximum(np.diff(gt[:, 0]), 1e-6)
v = np.r_[0.0, v]
move = np.nonzero(v > 0.02)[0]
t_move = gt[move[0], 0] if len(move) else gt[-1, 0]
t_start = min(gt[0, 0], imu[0, 0])
still = gt[gt[:, 0] < t_move]
disp = float(np.max(np.hypot(still[:, 1] - still[0, 1], still[:, 2] - still[0, 2]))) if len(still) else float("nan")
w_imu = imu[imu[:, 0] < t_move]
acc = w_imu[:, 1:4]
gyr = w_imu[:, 4:7]
out["start_standstill"] = {
    "first_motion_after_s": round(float(t_move - t_start), 2),
    "gt_max_displacement_mm": round(disp * 1000, 2),
    "gt_max_speed_mm_s": round(float(np.max(v[gt[:, 0] < t_move])) * 1000, 2) if len(still) else None,
    "imu_samples": int(len(w_imu)),
    "imu_acc_norm_mean": round(float(np.mean(np.linalg.norm(acc, axis=1))), 4),
    "imu_acc_std_xyz": [round(float(s), 5) for s in acc.std(axis=0)],
    "imu_gyro_std_xyz": [round(float(s), 6) for s in gyr.std(axis=0)],
    "lidar_scans": int(sum(1 for s, _ in lidar_finite if s < t_move)),
}
# ---------------------------------------------------------------- the drive
seg = np.hypot(np.diff(gt[:, 1]), np.diff(gt[:, 2]))
seg[seg > 1.0] = 0.0                                     # ignore jumps (none expected)
out["drive"] = {
    "gt_path_length_m": round(float(seg.sum()), 1),
    "max_speed_m_s": round(float(np.percentile(v, 99.5)), 3),
    "elevation_range_m": round(float(gt[:, 3].max() - gt[:, 3].min()), 2),
    "end_to_start_m": round(float(np.hypot(gt[-1, 1] - gt[0, 1], gt[-1, 2] - gt[0, 2])), 2),
    "planned_lap_m": ROUTE["length_m"],
}
# ---------------------------------------------------------------- ground truth TUM (sim time)
np.savetxt(BAG / "ground_truth.tum", gt, fmt="%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f",
           header="timestamp(sim s) x y z qx qy qz qw  -- world frame, base_link of 'togo' (gz ground truth)")
(BAG / "check.json").write_text(json.dumps(out, indent=1))
print(json.dumps(out, indent=1))

# ---------------------------------------------------------------- figure
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, ax = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.2, 1]})
path = np.array(ROUTE["path"])
ax[0].plot(path[:, 0], path[:, 1], "-", color="0.7", lw=3, label="planned loop")
sc = ax[0].scatter(gt[::10, 1], gt[::10, 2], c=gt[::10, 3], s=3, cmap="viridis", label="driven (colour = z)")
ax[0].plot(gt[0, 1], gt[0, 2], "g^", ms=12, label="start (still)")
ax[0].plot(gt[-1, 1], gt[-1, 2], "rs", ms=8, label="end")
ax[0].set_aspect("equal"); ax[0].set_xlabel("world x (m)"); ax[0].set_ylabel("world y (m)")
ax[0].legend(fontsize=8); fig.colorbar(sc, ax=ax[0], fraction=0.04, label="z (m)")
ax[0].set_title(f"{BAG.name}: {out['drive']['gt_path_length_m']} m driven")
tt = gt[:, 0] - t_start
ax[1].plot(tt, v, lw=0.6, label="ground-truth speed (m/s)")
ax[1].plot(tt, gt[:, 3] - gt[0, 3], lw=0.8, label="z - z0 (m)")
ax[1].axvspan(0, t_move - t_start, color="g", alpha=0.15, label=f"standstill {t_move - t_start:.1f} s")
ax[1].set_xlabel("sim time since bag start (s)"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
fig.savefig(BAG / "check.png", dpi=100, bbox_inches="tight")
print("wrote", BAG / "ground_truth.tum", BAG / "check.json", BAG / "check.png")
