#!/usr/bin/env python3
"""Score one SLAM replay against the bag's ground truth (scripts/replay_slam_bag.sh runs it).

Usage: slam_eval.py RUN_BAG GROUND_TRUTH_TUM [--out DIR] [--label TEXT] [--resources CSV] [--segments JSON]
                    [--run-info FILE] [--bag DIR] [--heights DIR]

RUN_BAG is what the SLAM published during the replay: /dlio/odometry (the DLIO frontend, odom -> base_link),
/dlio/frontend_diagnostics (DLIO's per-scan numbers), /hazard/diagnostics (the backend's hazard mapper) and /tf.

How the estimate is compared with ground truth (both frames are gravity-aligned, so only position and heading
are free; aligning roll and pitch too would hide real tilt and vertical errors):
  - DLIO's leading placeholder poses (exact identity, published until its IMU calibration ends) are skipped;
  - ground truth is interpolated to each odometry stamp (sim time): position linearly, orientation by nlerp;
  - 'aligned': ONE rigid 4-DoF transform (x, y, z, yaw) fitted to the whole run -> ATE and error components;
  - 'start-anchored': the 4-DoF transform that makes the first poses coincide -> drift (how far it wanders).
Writes OUT/metrics.json, OUT/errors_vs_distance.csv, OUT/odometry.tum and three pictures:
  map.png      plan view over the true terrain: ground truth, the start-anchored estimate and the aligned estimate
               coloured by horizontal error (left) and signed vertical error (right); the five worst places
  profile.png  elevation along the lap (truth, estimate start-anchored and aligned) and the vertical error
  errors.png   error components along the lap: along-track, cross-track (zig-zag), vertical, heading, roll, pitch
metrics.json sections:
  accuracy       ATE (4-DoF; 6-DoF for reference), components, vertical, drift at the end, RPE 1 m / 10 m,
                 zig-zag (cross-track error minus its 2 s average), per-leg ATE;
                 vertical_vs_climb_pct: start-anchored vertical drift fitted against the climb since the start
                 (a height scale error: + = climbs and descents exaggerated; R^2 says how much it explains);
                 heading_lag_ms: yaw error fitted against the true yaw rate in turns (> 0: the published pose
                 trails its stamp, as when an estimator corrects its current state with an older scan's pose)
  scan_matching  DLIO per-scan diagnostics: GICP time, correction size, fitness, overlap, degeneracy, drops
  resources      RAM / CPU per SLAM process (tools/proc_sampler.py) and DLIO CPU time per scan
  hazard         update time and dropped clouds of the backend's hazard mapper
  loop_closure   the largest map -> odom correction (0 = loop closure off)
Reading the bag needs the system python3 with ROS 2 sourced (rosbag2_py); the maths does not.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ASSOC_MAX_DT = 0.015          # s: for associate(); score() interpolates ground truth instead
RPE_DIST = 10.0               # m
ZIGZAG_WINDOW_S = 2.0
MOVING_SPEED = 0.05           # m/s: ground-truth speed above which the rover counts as driving
TURN_RATE = 0.2               # rad/s: ground-truth yaw rate above which the rover counts as turning
MIN_CLIMB = 0.5               # m: elevation range below which vertical error vs climb is not fitted
# the place where each leg of the benchmark route (the bag's segments.json) starts; other legs keep their name
LEG_START_PLACE = {"to_rocks": "start", "rocks_to_slope": "rocks", "slope_to_plain": "slope", "plain_to_start": "plain"}


def place_marks(segments: dict | None, t: np.ndarray, s: np.ndarray) -> list[tuple[str, float, int]]:
    """(place, distance along the lap, sample index) where each leg starts, for the samples t (stamps) / s."""
    marks = []
    for name, (t0, _) in (segments or {}).items():
        k = int(np.searchsorted(t, t0))
        if 0 < k < len(t):
            marks.append((LEG_START_PLACE.get(name, name.replace("_", " ")), float(s[k]), k))
    return marks

# /dlio/frontend_diagnostics (std_msgs/Float32MultiArray), indices from direct_lidar_inertial_odometry odom.cc
DIAG = {"status": 0, "raw_points": 6, "filtered_points": 7, "correction_t_m": 18, "correction_r_deg": 19,
        "correction_rejected": 20, "fitness": 22, "inliers": 23, "overlap": 24, "hessian_min_eig": 25,
        "hessian_max_eig": 26, "hessian_condition": 27, "gicp_ms": 28, "spin_guard": 31, "imu_age_ms": 32,
        "timing_guard": 33, "stale_scan_drops": 43, "gc_engaged": 49, "lidar_callbacks_started": 56,
        "lidar_callbacks_completed": 57}
PROCESSED = (5.0, 6.0)        # status codes: registration accepted / rejected


# ------------------------------------------------------------------ SE(3) helpers (no ROS)
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """(N, 4) quaternions x, y, z, w -> (N, 3, 3) rotation matrices."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def rot_to_quat(r: np.ndarray) -> np.ndarray:
    """(N, 3, 3) -> (N, 4) x, y, z, w."""
    out = np.empty((len(r), 4))
    for k, m in enumerate(r):
        t = np.trace(m)
        if t > 0:
            s = 2 * np.sqrt(t + 1)
            out[k] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
        else:
            i = int(np.argmax(np.diag(m)))
            j, l = (i + 1) % 3, (i + 2) % 3
            s = 2 * np.sqrt(1 + m[i, i] - m[j, j] - m[l, l])
            v = np.empty(4)
            v[i] = s / 4
            v[j] = (m[j, i] + m[i, j]) / s
            v[l] = (m[l, i] + m[i, l]) / s
            v[3] = (m[l, j] - m[j, l]) / s
            out[k] = v
    return out


def poses(tum: np.ndarray) -> np.ndarray:
    """(N, 8) t x y z qx qy qz qw -> (N, 4, 4) homogeneous transforms."""
    t = np.repeat(np.eye(4)[None], len(tum), 0)
    t[:, :3, :3] = quat_to_rot(tum[:, 4:8])
    t[:, :3, 3] = tum[:, 1:4]
    return t


def umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid R, t (no scale, all three rotation axes) minimising sum |R src_i + t - dst_i|^2."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def rot_z(th: float) -> np.ndarray:
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def align_4dof(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotation about z and translation minimising sum |R src_i + t - dst_i|^2 (gravity-aligned frames)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    th = np.arctan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]), np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    r = rot_z(th)
    return r, mu_d - r @ mu_s


def associate(t_est: np.ndarray, t_gt: np.ndarray, max_dt: float = ASSOC_MAX_DT) -> tuple[np.ndarray, np.ndarray]:
    """Index pairs (i_est, i_gt) of the nearest ground-truth stamp within max_dt."""
    j = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    j = np.where(np.abs(t_gt[j - 1] - t_est) < np.abs(t_gt[j] - t_est), j - 1, j)
    ok = np.abs(t_gt[j] - t_est) <= max_dt
    return np.nonzero(ok)[0], j[ok]


def interpolate_gt(gt: np.ndarray, t_q: np.ndarray, max_gap: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """Ground truth at the query stamps (inside its time range, no gap > max_gap): (indices into t_q, (M, 8))."""
    t = gt[:, 0]
    inside = (t_q >= t[0]) & (t_q <= t[-1])
    idx = np.nonzero(inside)[0]
    tq = t_q[idx]
    i = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, len(t) - 2)
    ok = (t[i + 1] - t[i]) <= max_gap
    idx, tq, i = idx[ok], tq[ok], i[ok]
    a = ((tq - t[i]) / np.maximum(t[i + 1] - t[i], 1e-12))[:, None]
    pos = gt[i, 1:4] * (1 - a) + gt[i + 1, 1:4] * a
    q0, q1 = gt[i, 4:8], gt[i + 1, 4:8]
    q1 = q1 * np.sign(np.sum(q0 * q1, axis=1))[:, None]
    qq = q0 * (1 - a) + q1 * a
    qq /= np.linalg.norm(qq, axis=1, keepdims=True)
    return idx, np.c_[tq, pos, qq]


def drop_placeholders(est: np.ndarray) -> np.ndarray:
    """Without the leading exact-identity poses: DLIO publishes them (from stamp 0) until its IMU calibration
    ends, and its frame starts at the first pose after them, so anchoring on them would use the wrong instant."""
    real = (np.abs(est[:, 1:7]).max(axis=1) > 0) | (est[:, 7] != 1.0)
    return est[int(np.argmax(real)):] if real.any() else est


def compose_map(odom: np.ndarray, map_odom: np.ndarray | None) -> np.ndarray | None:
    """map -> base_link at each odometry stamp: the latest map -> odom at or before it, times odom -> base_link."""
    if map_odom is None or not len(map_odom):
        return None
    k = np.clip(np.searchsorted(map_odom[:, 0], odom[:, 0], side="right") - 1, 0, len(map_odom) - 1)
    t = poses(map_odom)[k] @ poses(odom)
    out = np.empty_like(odom)
    out[:, 0] = odom[:, 0]
    out[:, 1:4] = t[:, :3, 3]
    out[:, 4:8] = rot_to_quat(t[:, :3, :3])
    return out


def rot_angle_deg(r: np.ndarray) -> np.ndarray:
    """Rotation angle of (N, 3, 3) matrices in degrees."""
    c = (np.trace(r, axis1=1, axis2=2) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def yaw_of(r: np.ndarray) -> np.ndarray:
    return np.arctan2(r[:, 1, 0], r[:, 0, 0])


def rpy_of(r: np.ndarray) -> np.ndarray:
    """(N, 3, 3) -> (N, 3) roll, pitch, yaw (rad), ZYX convention."""
    return np.stack([np.arctan2(r[:, 2, 1], r[:, 2, 2]), np.arcsin(np.clip(-r[:, 2, 0], -1, 1)),
                     np.arctan2(r[:, 1, 0], r[:, 0, 0])], 1)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _rpe(pe: np.ndarray, pg: np.ndarray, s: np.ndarray, dist: float, max_pairs: int = 3000) -> dict:
    step = max(1, len(s) // max_pairs)
    i = np.arange(0, len(s), step)
    j = np.searchsorted(s, s[i] + dist)
    ok = j < len(s)
    i, j = i[ok], j[ok]
    if not len(i):
        return {}
    rel_g = np.linalg.inv(pg[i]) @ pg[j]
    rel_e = np.linalg.inv(pe[i]) @ pe[j]
    err = np.linalg.inv(rel_g) @ rel_e
    tr = np.linalg.norm(err[:, :3, 3], axis=1)
    rot = rot_angle_deg(err[:, :3, :3])
    tag = f"rpe_{dist:g}m"
    return {f"{tag}_mean_m": round(float(tr.mean()), 4), f"{tag}_rmse_m": round(float(np.sqrt(np.mean(tr ** 2))), 4),
            f"{tag}_max_m": round(float(tr.max()), 4), f"{tag}_mean_pct": round(float(tr.mean() / dist * 100), 3),
            f"{tag}_vertical_mean_m": round(float(np.abs(err[:, 2, 3]).mean()), 4),
            f"{tag}_rot_mean_deg": round(float(rot.mean()), 4), f"{tag}_rot_max_deg": round(float(rot.max()), 4)}


def score(est: np.ndarray, gt: np.ndarray, rpe_dist: float = RPE_DIST, segments: dict | None = None) -> dict:
    """Accuracy of one trajectory against ground truth (module docstring). Keys starting with '_' are arrays for
    the pictures and the CSV, not metrics."""
    est = drop_placeholders(est)
    idx, g = interpolate_gt(gt, est[:, 0])
    if len(idx) < 10:
        return {"error": f"only {len(idx)} stamps inside the ground truth"}
    e = est[idx]
    pe, pg = poses(e), poses(g)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(g[:, 1:4], axis=0), axis=1))]
    # aligned: one 4-DoF fit over the whole run
    r4, t4 = align_4dof(e[:, 1:4], g[:, 1:4])
    al = (r4 @ e[:, 1:4].T).T + t4
    err_v = al - g[:, 1:4]
    err = np.linalg.norm(err_v, axis=1)
    r6, t6 = umeyama(e[:, 1:4], g[:, 1:4])
    err6 = np.linalg.norm((r6 @ e[:, 1:4].T).T + t6 - g[:, 1:4], axis=1)
    # start-anchored: 4-DoF transform making the first poses coincide
    th0 = yaw_of(pg[:1, :3, :3])[0] - yaw_of(pe[:1, :3, :3])[0]
    ra = rot_z(th0)
    anch = (ra @ e[:, 1:4].T).T + (g[0, 1:4] - ra @ e[0, 1:4])
    drift_v = anch - g[:, 1:4]
    # components in the ground truth's heading frame (gravity-aligned): along, cross, vertical
    yaw_g = yaw_of(pg[:, :3, :3])
    h = np.c_[np.cos(yaw_g), np.sin(yaw_g)]
    along = np.sum(err_v[:, :2] * h, axis=1)
    cross = -np.sin(yaw_g) * err_v[:, 0] + np.cos(yaw_g) * err_v[:, 1]
    vert = err_v[:, 2]
    # orientation errors of the aligned estimate (roll, pitch, yaw)
    r_al = np.einsum("ij,njk->nik", r4, pe[:, :3, :3])
    rpy_err = np.degrees(wrap(rpy_of(r_al) - rpy_of(pg[:, :3, :3])))
    out = {"matched": int(len(idx)), "duration_s": round(float(e[-1, 0] - e[0, 0]), 1),
           "path_length_m": round(float(s[-1]), 1),
           "ate_rmse_m": round(float(np.sqrt(np.mean(err ** 2))), 4), "ate_mean_m": round(float(err.mean()), 4),
           "ate_max_m": round(float(err.max()), 4),
           "ate_rmse_6dof_m": round(float(np.sqrt(np.mean(err6 ** 2))), 4),
           "along_rmse_m": round(float(np.sqrt(np.mean(along ** 2))), 4),
           "cross_rmse_m": round(float(np.sqrt(np.mean(cross ** 2))), 4),
           "vertical_rmse_m": round(float(np.sqrt(np.mean(vert ** 2))), 4),
           "vertical_max_m": round(float(np.max(np.abs(vert))), 4),
           "yaw_err_rmse_deg": round(float(np.sqrt(np.mean(rpy_err[:, 2] ** 2))), 4),
           "roll_err_rmse_deg": round(float(np.sqrt(np.mean(rpy_err[:, 0] ** 2))), 4),
           "pitch_err_rmse_deg": round(float(np.sqrt(np.mean(rpy_err[:, 1] ** 2))), 4),
           "end_gap_m": round(float(np.linalg.norm(e[-1, 1:4] - e[0, 1:4])), 3),
           "end_gap_ground_truth_m": round(float(np.linalg.norm(g[-1, 1:4] - g[0, 1:4])), 3)}
    d_end = drift_v[-1]
    out["drift_end_m"] = round(float(np.linalg.norm(d_end)), 4)
    out["drift_end_xy_m"] = round(float(np.linalg.norm(d_end[:2])), 4)
    out["drift_end_z_m"] = round(float(d_end[2]), 4)
    out["drift_max_z_m"] = round(float(drift_v[np.argmax(np.abs(drift_v[:, 2])), 2]), 4)
    out["drift_end_pct"] = round(float(np.linalg.norm(d_end) / max(s[-1], 1e-9) * 100), 3)
    out["vertical_drift_per_100m"] = round(float(d_end[2] / max(s[-1], 1e-9) * 100), 4)
    dyaw = yaw_of(pe[-1:, :3, :3])[0] + th0 - yaw_of(pg[-1:, :3, :3])[0]
    out["drift_end_yaw_deg"] = round(float(np.degrees(wrap(dyaw))), 3)
    # vertical drift against the climb since the start: a slope means a height scale error (climbs and descents
    # exaggerated or flattened), which a random walk would not show
    climb = g[:, 3] - g[0, 3]
    if np.ptp(climb) > MIN_CLIMB:
        k, c = np.polyfit(climb, drift_v[:, 2], 1)
        ss = float(np.sum((drift_v[:, 2] - drift_v[:, 2].mean()) ** 2))
        out["vertical_vs_climb_pct"] = round(float(k * 100), 2)
        out["vertical_vs_climb_r2"] = round(1 - float(np.sum((drift_v[:, 2] - k * climb - c) ** 2)) / ss, 3) if ss else None
    # heading lag: a pose that trails its stamp by tau has, in turns, a yaw error of -tau * yaw rate
    tg = np.r_[True, np.diff(gt[:, 0]) > 0]
    yaw_t = np.unwrap(yaw_of(quat_to_rot(gt[tg, 4:8])))
    rate = np.interp(e[:, 0], gt[tg, 0], np.gradient(yaw_t, gt[tg, 0]))
    turn = np.abs(rate) > TURN_RATE
    if turn.sum() >= 20:
        out["heading_lag_ms"] = round(float(-np.polyfit(rate[turn], np.radians(rpy_err[turn, 2]), 1)[0] * 1000), 1)
    out.update(_rpe(pe, pg, s, 1.0))
    out.update(_rpe(pe, pg, s, rpe_dist))
    # zig-zag: cross-track error minus its moving average, while driving
    v = np.r_[0.0, np.linalg.norm(np.diff(g[:, 1:3], axis=0), axis=1) / np.maximum(np.diff(g[:, 0]), 1e-6)]
    dt = float(np.median(np.diff(e[:, 0]))) if len(e) > 1 else 0.05
    w = max(3, int(round(ZIGZAG_WINDOW_S / max(dt, 1e-3))) | 1)
    cross_hp = cross - np.convolve(cross, np.ones(w) / w, mode="same")
    moving = v > MOVING_SPEED
    moving[:w] = moving[-w:] = False                       # the moving average is biased at the ends
    if moving.any():
        out["zigzag_rms_m"] = round(float(np.sqrt(np.mean(cross_hp[moving] ** 2))), 4)
        out["zigzag_p95_m"] = round(float(np.percentile(np.abs(cross_hp[moving]), 95)), 4)
    if segments:
        out["segments"] = {}
        for name, (t0, t1) in segments.items():
            m = (e[:, 0] >= t0) & (e[:, 0] <= t1)
            if m.sum() >= 5:
                out["segments"][name] = {"ate_rmse_m": round(float(np.sqrt(np.mean(err[m] ** 2))), 4),
                                         "ate_max_m": round(float(err[m].max()), 4),
                                         "vertical_rmse_m": round(float(np.sqrt(np.mean(vert[m] ** 2))), 4),
                                         "cross_rmse_m": round(float(np.sqrt(np.mean(cross[m] ** 2))), 4)}
    out.update({"_t": e[:, 0], "_s": s, "_gt_xyz": g[:, 1:4], "_aligned_xyz": al, "_anchored_xyz": anch,
                "_err": err, "_along": along, "_cross": cross, "_cross_hp": cross_hp, "_vert": vert,
                "_drift": drift_v, "_rpy_err": rpy_err})
    return out


def _stats(x: np.ndarray, digits: int = 3, pcts=(50, 95, 99)) -> dict:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):
        return {}
    out = {"mean": round(float(x.mean()), digits), "max": round(float(x.max()), digits)}
    for p in pcts:
        out[f"p{p}"] = round(float(np.percentile(x, p)), digits)
    return out


def scan_matching(diag: np.ndarray) -> dict:
    """Per-scan DLIO diagnostics (N, >=58), rows in publication order."""
    if diag is None or not len(diag):
        return {"error": "no /dlio/frontend_diagnostics"}
    st = diag[:, DIAG["status"]]
    p = diag[np.isin(st, PROCESSED)]
    out = {"scans_processed": int(len(p)), "status_counts": {str(int(k)): int(v) for k, v in
                                                              zip(*np.unique(st, return_counts=True))}}
    last = diag[-1]
    out["lidar_callbacks_started"] = int(last[DIAG["lidar_callbacks_started"]])
    out["lidar_callbacks_completed"] = int(last[DIAG["lidar_callbacks_completed"]])
    out["stale_scan_drops"] = int(diag[:, DIAG["stale_scan_drops"]].max())
    if not len(p):
        return out
    out["gicp_ms"] = _stats(p[:, DIAG["gicp_ms"]])
    out["fitness"] = _stats(p[:, DIAG["fitness"]], 5, (50, 95))
    out["overlap"] = _stats(p[:, DIAG["overlap"]], 4, (5, 50))
    out["inliers"] = _stats(p[:, DIAG["inliers"]], 0, (5, 50))
    out["hessian_min_eig"] = _stats(p[:, DIAG["hessian_min_eig"]], 2, (5, 50))
    out["hessian_condition"] = _stats(p[:, DIAG["hessian_condition"]], 1, (50, 95))
    out["correction_mm"] = _stats(p[:, DIAG["correction_t_m"]] * 1000.0, 2, (50, 95, 99))
    out["correction_deg"] = _stats(p[:, DIAG["correction_r_deg"]], 4, (50, 95, 99))
    out["corrections_rejected"] = int((p[:, DIAG["correction_rejected"]] > 0.5).sum())
    out["filtered_points"] = _stats(p[:, DIAG["filtered_points"]], 0, (50,))
    out["raw_points"] = _stats(p[:, DIAG["raw_points"]], 0, (50,))
    out["imu_age_ms"] = _stats(p[:, DIAG["imu_age_ms"]], 2, (50, 95))
    out["spin_guard_scans"] = int((p[:, DIAG["spin_guard"]] > 0.5).sum())
    out["timing_guard_scans"] = int((p[:, DIAG["timing_guard"]] > 0.5).sum())
    out["ground_constraint_scans"] = int((p[:, DIAG["gc_engaged"]] > 0.5).sum())
    return out


def resources(csv_path: Path, t0: float | None, t1: float | None, scans: int | None) -> dict:
    """RSS and CPU per process name over [t0, t1] (epoch s) from tools/proc_sampler.py's CSV."""
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        return {"error": "no samples"}
    t = np.array([float(r["t"]) for r in rows])
    t0 = t.min() if t0 is None else t0
    t1 = t.max() if t1 is None else t1
    out: dict = {"window_s": round(t1 - t0, 1)}
    per: dict = {}
    total_rss: dict[float, float] = {}
    for r in rows:
        tt = float(r["t"])
        per.setdefault(r["name"], {}).setdefault(int(r["pid"]), []).append((tt, float(r["rss_mb"]), float(r["cpu_s"])))
        if t0 <= tt <= t1:
            total_rss[tt] = total_rss.get(tt, 0.0) + float(r["rss_mb"])
    procs = {}
    total_cpu = 0.0
    for name, pids in per.items():
        rss_end = rss_start = rss_peak = cpu = 0.0
        cpu_inst = []
        for samples in pids.values():
            a = np.array(samples)
            m = (a[:, 0] >= t0) & (a[:, 0] <= t1)
            if m.sum() < 2:
                continue
            w = a[m]
            rss_start += w[0, 1]
            rss_end += w[-1, 1]
            rss_peak += w[:, 1].max()
            cpu += w[-1, 2] - w[0, 2]
            cpu_inst.extend(np.diff(w[:, 2]) / np.maximum(np.diff(w[:, 0]), 1e-6) * 100)
        if rss_peak == 0 and cpu == 0:
            continue
        dur = max(t1 - t0, 1e-6)
        procs[name] = {"rss_start_mb": round(rss_start, 1), "rss_end_mb": round(rss_end, 1),
                       "rss_peak_mb": round(rss_peak, 1),
                       "rss_growth_mb_per_min": round((rss_end - rss_start) / dur * 60, 2),
                       "cpu_s": round(cpu, 2), "cpu_pct_mean": round(cpu / dur * 100, 1),
                       "cpu_pct_p95": round(float(np.percentile(cpu_inst, 95)), 1) if cpu_inst else None}
        total_cpu += cpu
    out["processes"] = dict(sorted(procs.items(), key=lambda kv: -kv[1]["rss_peak_mb"]))
    out["total_cpu_pct_mean"] = round(total_cpu / max(t1 - t0, 1e-6) * 100, 1)
    if total_rss:
        out["total_rss_peak_mb"] = round(max(total_rss.values()), 1)
    avail = [float(r["mem_available_mb"]) for r in rows if t0 <= float(r["t"]) <= t1]
    if avail:
        out["wsl_mem_available_min_mb"] = round(min(avail), 0)
    dlio = procs.get("dlio_odom_node")
    if dlio and scans:
        out["dlio_cpu_ms_per_scan"] = round(dlio["cpu_s"] * 1000 / scans, 2)
    return out


def hazard(updates: list[dict]) -> dict:
    if not updates:
        return {"error": "no /hazard/diagnostics"}
    ms = np.array([u.get("total_ms", np.nan) for u in updates], dtype=float)
    out = {"updates": len(updates), "update_ms": _stats(ms, 1, (50, 95))}
    for k in ("dropped_clouds", "dropped_ray_clouds", "raw_hazards", "known_cells"):
        if k in updates[-1]:
            out[k] = updates[-1][k]
    return out


# ------------------------------------------------------------------ bag reading (ROS)
def read_run_bag(path: Path) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ("/dlio/odometry", "/tf", "/dlio/frontend_diagnostics", "/hazard/diagnostics")
    cls = {k: get_message(types[k]) for k in wanted if k in types}
    odom, map_odom, diag, haz = [], [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in cls:
            continue
        m = deserialize_message(data, cls[topic])
        if topic == "/dlio/odometry":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif topic == "/tf":
            for tr in m.transforms:
                if tr.header.frame_id.lstrip("/") == "map" and tr.child_frame_id.lstrip("/") == "odom":
                    p, q = tr.transform.translation, tr.transform.rotation
                    map_odom.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9,
                                     p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif topic == "/dlio/frontend_diagnostics":
            diag.append(list(m.data))
        else:
            try:
                haz.append(json.loads(m.data))
            except (ValueError, TypeError):
                pass
    width = max((len(d) for d in diag), default=0)
    diag_arr = np.array([d + [np.nan] * (width - len(d)) for d in diag]) if diag else None
    return {"odom": np.array(sorted(odom)), "map_odom": np.array(sorted(map_odom)) if map_odom else None,
            "diag": diag_arr, "hazard": haz}


def _read_kv(path: Path | None) -> dict:
    if not path or not path.is_file():
        return {}
    return dict(line.strip().split("=", 1) for line in path.read_text().splitlines() if "=" in line)


def terrain_view(bag: Path | None, heights: Path, xy: np.ndarray, pad: float = 6.0, res: float = 0.1):
    """Hillshade of the true terrain around the path, cached as BAG/terrain_view.npz."""
    cache = bag / "terrain_view.npz" if bag else None
    if cache and cache.is_file():
        d = np.load(cache)
        return d["z"], tuple(d["extent"])
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from gt_check import Terrain
    except ImportError:
        return None, None
    if not heights.is_dir():
        return None, None
    x0, y0 = xy.min(0) - pad
    x1, y1 = xy.max(0) + pad
    xs, ys = np.arange(x0, x1, res), np.arange(y0, y1, res)
    X, Y = np.meshgrid(xs, ys)
    z = Terrain(heights).height(X.ravel(), Y.ravel()).reshape(X.shape)
    extent = (float(xs[0]), float(xs[-1]), float(ys[0]), float(ys[-1]))
    if cache:
        try:
            np.savez_compressed(cache, z=z, extent=np.array(extent))
        except OSError:
            pass
    return z, extent


def worst_spots(mag: np.ndarray, s: np.ndarray, n: int = 5, sep_m: float = 5.0) -> list[int]:
    """Indices of the n largest values that lie at least sep_m apart along the lap (distinct places, not
    neighbouring samples of one bad stretch)."""
    out: list[int] = []
    for k in np.argsort(mag)[::-1]:
        if all(abs(s[k] - s[j]) > sep_m for j in out):
            out.append(int(k))
            if len(out) == n:
                break
    return out


def pictures(out_dir: Path, title: str, acc: dict, arr: dict, segments: dict | None, bag: Path | None,
             heights: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource

    s, t, gtx, al, anch = arr["_s"], arr["_t"], arr["_gt_xyz"], arr["_aligned_xyz"], arr["_anchored_xyz"]
    marks = place_marks(segments, t, s)

    # map.png: plan view over the true terrain; left horizontal error, right vertical error (4-DoF aligned)
    z, extent = terrain_view(bag, heights, gtx[:, :2])
    relief = None
    if z is not None:
        zz = np.where(np.isfinite(z), z, np.nanmean(z))
        relief = LightSource(azdeg=300, altdeg=35).hillshade(zz, vert_exag=3, dx=0.1, dy=0.1)
    horiz = np.hypot(arr["_along"], arr["_cross"])
    vert = arr["_vert"]
    vmax_v = max(0.02, float(np.percentile(np.abs(vert), 99)))
    x0, x1, y0, y1 = extent if extent else (*np.r_[gtx[:, 0].min(), gtx[:, 0].max()], *np.r_[gtx[:, 1].min(), gtx[:, 1].max()])
    fig, axes = plt.subplots(1, 2, figsize=(20, min(14.0, max(5.0, 8.0 * (y1 - y0) / max(x1 - x0, 1e-6) + 1.6))),
                             layout="constrained")
    panels = ((axes[0], horiz, "turbo", 0.0, max(0.05, float(np.percentile(horiz, 99))), "horizontal error (m)",
               f"horizontal error (along {acc['along_rmse_m']:.3f}, cross {acc['cross_rmse_m']:.3f} m RMSE)"),
              (axes[1], vert, "coolwarm", -vmax_v, vmax_v, "vertical error (m, + = estimate above the truth)",
               f"vertical error ({acc['vertical_rmse_m']:.3f} m RMSE, max {acc['vertical_max_m']:.3f} m)"))
    for ax, val, cmap, lo, hi, cbar, head in panels:
        if relief is not None:
            ax.imshow(relief, cmap="gray", origin="lower", extent=extent, alpha=0.9)
        ax.plot(anch[:, 0], anch[:, 1], color="magenta", lw=0.8, alpha=0.8, label="DLIO start-anchored (drift, as RViz shows it)")
        ax.plot(gtx[:, 0], gtx[:, 1], color="black", lw=3, alpha=0.5, label="ground truth")
        sc = ax.scatter(al[:, 0], al[:, 1], c=val, s=5, cmap=cmap, vmin=lo, vmax=hi, label="DLIO 4-DoF aligned", zorder=3)
        fig.colorbar(sc, ax=ax, fraction=0.035, label=cbar)
        for k in worst_spots(np.abs(val), s):
            ax.annotate(f"{val[k]:+.2f} m at {s[k]:.0f} m" if lo < 0 else f"{val[k]:.2f} m at {s[k]:.0f} m",
                        al[k, :2], xytext=(6, 6), textcoords="offset points", fontsize=8, color="red",
                        bbox=dict(fc="white", alpha=0.6, lw=0))
        for name, _, k in marks:
            ax.plot(*gtx[k, :2], "w^", ms=9, mec="k")
            ax.annotate(name, gtx[k, :2], xytext=(6, -12), textcoords="offset points", fontsize=8,
                        color="white", bbox=dict(fc="black", alpha=0.5, lw=0))
        ax.set_aspect("equal")
        ax.set_xlabel("world x (m)")
        ax.set_ylabel("world y (m)")
        ax.set_title(head, fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
    fig.suptitle(f"{title}: ATE {acc['ate_rmse_m']:.3f} m (max {acc['ate_max_m']:.3f} m); labels: error at "
                 f"distance along the lap")
    fig.savefig(out_dir / "map.png", dpi=100, bbox_inches="tight")
    plt.close(fig)

    def shade(axes):
        """A line at each place on every panel, the place names above the top panel."""
        for a in axes:
            for _, sk, _ in marks:
                a.axvline(sk, color="k", lw=0.6, alpha=0.4)
        for name, sk, _ in marks:
            axes[0].text(sk, 1.0, name, transform=axes[0].get_xaxis_transform(), fontsize=8, ha="center",
                         va="bottom")

    # profile.png: elevation along the lap and vertical error
    fig, ax = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    ax[0].plot(s, gtx[:, 2], "k-", lw=2.5, alpha=0.5, label="ground truth")
    ax[0].plot(s, anch[:, 2], lw=1, label="DLIO, start-anchored (drift)")
    ax[0].plot(s, al[:, 2], lw=1, alpha=0.8, label="DLIO, aligned over the run")
    ax[0].set_ylabel("z (m)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_title(f"{title}: vertical RMSE {acc['vertical_rmse_m']:.3f} m, vertical drift at the end "
                    f"{acc['drift_end_z_m']:+.3f} m ({acc['vertical_drift_per_100m']:+.3f} m per 100 m)", pad=16)
    ax[1].plot(s, arr["_vert"], lw=0.8, label="vertical error, aligned")
    climb = (f" ({acc['vertical_vs_climb_pct']:+.1f} % of the climb since the start, R² {acc['vertical_vs_climb_r2']:.2f})"
             if acc.get("vertical_vs_climb_r2") is not None else "")
    ax[1].plot(s, arr["_drift"][:, 2], lw=0.8, label="vertical drift, start-anchored" + climb)
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_xlabel("distance along the lap (m)")
    ax[1].set_ylabel("m")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    shade(ax)
    fig.savefig(out_dir / "profile.png", dpi=100, bbox_inches="tight")
    plt.close(fig)

    # errors.png: components along the lap
    fig, ax = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    ax[0].plot(s, arr["_along"], lw=0.8, label=f"along-track (RMSE {acc['along_rmse_m']:.3f} m)")
    ax[0].plot(s, arr["_cross"], lw=0.8, label=f"cross-track (RMSE {acc['cross_rmse_m']:.3f} m)")
    ax[0].plot(s, arr["_vert"], lw=0.8, label=f"vertical (RMSE {acc['vertical_rmse_m']:.3f} m)")
    ax[0].set_ylabel("aligned error (m)")
    ax[0].set_title(f"{title}: where it goes wrong, by place along the lap", pad=16)
    ax[1].plot(s, arr["_cross_hp"], lw=0.6, color="tab:red",
               label=f"zig-zag: cross-track minus its 2 s average (RMS {acc.get('zigzag_rms_m', float('nan')) * 1000:.1f} mm)")
    ax[1].set_ylabel("m")
    lag = (f" (heading trails the truth by {acc['heading_lag_ms']:.0f} ms in turns)"
           if "heading_lag_ms" in acc else "")
    for k, name in ((2, "yaw"), (0, "roll"), (1, "pitch")):
        ax[2].plot(s, arr["_rpy_err"][:, k], lw=0.8, label=f"{name} error" + (lag if k == 2 else ""))
    ax[2].set_ylabel("deg")
    ax[2].set_xlabel("distance along the lap (m)")
    for a in ax:
        a.axhline(0, color="k", lw=0.5)
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper left")
    shade(ax)
    fig.savefig(out_dir / "errors.png", dpi=100, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_bag", type=Path)
    ap.add_argument("ground_truth", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--label", default="")
    ap.add_argument("--resources", type=Path, default=None, help="tools/proc_sampler.py CSV")
    ap.add_argument("--segments", type=Path, default=None, help="segments.json of the bag")
    ap.add_argument("--run-info", type=Path, default=None, help="run_info.txt (play_start / play_end epoch s)")
    ap.add_argument("--bag", type=Path, default=None, help="the replayed bag (caches the terrain picture there)")
    ap.add_argument("--heights", type=Path, default=Path.home() / "togo_assets/lunar/build/apollo16_core/heights")
    a = ap.parse_args()
    out_dir = a.out or a.run_bag.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    gt = np.loadtxt(a.ground_truth, comments="#")
    run = read_run_bag(a.run_bag)
    if not len(run["odom"]):
        print("ERROR: no /dlio/odometry in the run bag", file=sys.stderr)
        return 1
    info = _read_kv(a.run_info)
    segments = json.loads(a.segments.read_text()) if a.segments and a.segments.is_file() else None
    acc = score(run["odom"], gt, segments=segments)
    arr = {k: acc.pop(k) for k in [k for k in acc if k.startswith("_")]}
    sm = scan_matching(run["diag"])
    metrics = {"meta": {"label": a.label, "run_bag": str(a.run_bag), "ground_truth": str(a.ground_truth),
                        "evaluation": "ground truth interpolated to odometry stamps; 4-DoF (x, y, z, yaw) alignment",
                        **{k: info[k] for k in ("bag", "config", "config_source", "rate", "duration", "started",
                                                "finished", "config_sha1", "host_cpu", "dlio_build") if k in info}},
               "accuracy": acc, "scan_matching": sm}
    t0 = float(info["play_start"]) if "play_start" in info else None
    t1 = float(info["play_end"]) if "play_end" in info else None
    if a.resources and a.resources.is_file():
        metrics["resources"] = resources(a.resources, t0, t1, sm.get("scans_processed"))
    metrics["hazard"] = hazard(run["hazard"])
    mo = run["map_odom"]
    metrics["loop_closure"] = {"map_to_odom_updates": 0 if mo is None else int(len(mo)),
                               "max_correction_m": 0.0 if mo is None else round(float(np.max(np.linalg.norm(mo[:, 1:4], axis=1))), 4)}
    np.savetxt(out_dir / "odometry.tum", run["odom"], fmt="%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f")
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=1))
    if arr:
        with open(out_dir / "errors_vs_distance.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "s", "err", "along", "cross", "cross_hp", "vertical", "drift_x", "drift_y", "drift_z",
                        "roll_err_deg", "pitch_err_deg", "yaw_err_deg", "gt_x", "gt_y", "gt_z"])
            for k in range(len(arr["_t"])):
                w.writerow([f"{arr['_t'][k]:.3f}", f"{arr['_s'][k]:.3f}", f"{arr['_err'][k]:.4f}",
                            f"{arr['_along'][k]:.4f}", f"{arr['_cross'][k]:.4f}", f"{arr['_cross_hp'][k]:.4f}",
                            f"{arr['_vert'][k]:.4f}", *(f"{v:.4f}" for v in arr["_drift"][k]),
                            *(f"{v:.3f}" for v in arr["_rpy_err"][k]), *(f"{v:.3f}" for v in arr["_gt_xyz"][k])])
    print_summary(metrics)
    try:
        pictures(out_dir, a.label or out_dir.name, acc, arr, segments, a.bag, a.heights)
    except Exception as e:  # noqa: BLE001 - the numbers matter more than the pictures
        print(f"(pictures skipped: {e})")
    print(f"metrics: {out_dir / 'metrics.json'}   pictures: map.png, profile.png, errors.png")
    return 0


def print_summary(m: dict) -> None:
    acc, sm, res = m["accuracy"], m["scan_matching"], m.get("resources", {})

    def g(d, *keys):
        for k in keys:
            d = d.get(k, {}) if isinstance(d, dict) else {}
        return d if not isinstance(d, dict) else float("nan")
    print(f"accuracy   ATE {g(acc, 'ate_rmse_m'):.3f} m (max {g(acc, 'ate_max_m'):.3f}; along {g(acc, 'along_rmse_m'):.3f}, "
          f"cross {g(acc, 'cross_rmse_m'):.3f}, vertical {g(acc, 'vertical_rmse_m'):.3f})   zig-zag {g(acc, 'zigzag_rms_m') * 1000:.1f} mm"
          f"   heading lag {g(acc, 'heading_lag_ms'):.0f} ms")
    print(f"drift      end {g(acc, 'drift_end_m'):.3f} m ({g(acc, 'drift_end_pct'):.2f} %), vertical {g(acc, 'drift_end_z_m'):+.3f} m "
          f"(max {g(acc, 'drift_max_z_m'):+.3f}; {g(acc, 'vertical_vs_climb_pct'):+.1f} % of the climb), "
          f"yaw {g(acc, 'drift_end_yaw_deg'):+.2f} deg   RPE 10 m {g(acc, 'rpe_10m_mean_pct'):.2f} %")
    print(f"matching   GICP {g(sm, 'gicp_ms', 'mean'):.1f} ms (p95 {g(sm, 'gicp_ms', 'p95'):.1f})   correction "
          f"{g(sm, 'correction_mm', 'mean'):.1f} mm   scans {sm.get('scans_processed')} / {sm.get('lidar_callbacks_started')}"
          f"   rejected {sm.get('corrections_rejected')}")
    if res and "processes" in res:
        d = res["processes"].get("dlio_odom_node", {})
        print(f"resources  dlio_odom_node RSS {d.get('rss_peak_mb', float('nan')):.0f} MB peak "
              f"(+{d.get('rss_growth_mb_per_min', float('nan')):.1f} MB/min), CPU {d.get('cpu_pct_mean', float('nan')):.0f} %, "
              f"{res.get('dlio_cpu_ms_per_scan', float('nan')):.1f} ms CPU/scan;   all SLAM {res.get('total_cpu_pct_mean', float('nan')):.0f} % CPU, "
              f"{res.get('total_rss_peak_mb', float('nan')):.0f} MB")
    hz = m.get("hazard", {})
    if "update_ms" in hz:
        print(f"hazard     {hz['updates']} updates, {hz['update_ms'].get('mean', float('nan')):.0f} ms mean, "
              f"dropped clouds {hz.get('dropped_clouds')}")
    lc = m.get("loop_closure", {})
    print(f"loop closure: largest map->odom correction {lc.get('max_correction_m')} m")


if __name__ == "__main__":
    sys.exit(main())
