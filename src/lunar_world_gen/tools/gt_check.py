#!/usr/bin/env python3
"""Check a simulation bag's ground truth against the simulator's own sensors and the world's true terrain.

Usage: gt_check.py BAG [--heights DIR] [--every N] [--out DIR]       (system python3 with ROS 2 sourced)

The IMU and the LiDAR are simulated from the same physics state as the ground-truth pose, so with a correct pose,
frame, sensor offsets and timing all three must agree to near numerical precision:
  1 gyro   angular velocity implied by consecutive ground-truth orientations (body frame) vs the IMU gyro
  2 accel  specific force implied by the ground-truth motion at the IMU (lever arm, gravity 1.62 m/s^2) vs the
           IMU accelerometer, both smoothed over 0.3 s
  3 lidar  every Nth scan placed in the world with the ground-truth pose and the base_link -> LiDAR offset from
           /tf_static; each ground return's height minus the true terrain height at that point (the build's
           1 cm height grid; rocks are separate meshes, so returns from rocks sit above it)
           - by range: the median must stay ~0 at every range; the spread grows with range because gpu_lidar
             reads ranges from a depth texture (up to ~half a beam spacing of angular quantisation), measured as
             sigma_z = 0.176 * beam_spacing * range (docs/impl/validation.md); 'expected_spread_mm' is that
           - tilt: a plane fitted to the residuals in the rover's heading frame (forward, left); a roll or pitch
             error of the ground truth (or a LiDAR mounting angle the TF does not know) shows up as a slope
  For 1 and 3 the pose is also shifted in time (-60..+60 ms) to find the time offset that fits best; 0 means
  the ground-truth stamps are right. The LiDAR search uses the scans taken while turning and their returns at
  3-15 m, where a stamp error moves the points the most (a 50 ms error at 0.5 rad/s: ~25 cm at 10 m).
Writes OUT/gt_check.json and OUT/gt_check.png (OUT defaults to the bag folder).
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np

GRAVITY = 1.62
SHIFTS_MS = np.arange(-60, 61, 5)
BEAM_SPACING_RAD = 0.01        # the sim LiDAR: 209 x 122 beams over +-60 x +-35 deg
QUANT_SIGMA_PER_M = 0.176 * BEAM_SPACING_RAD


def ground_tilt(r, u, v, scan, min_returns=200):
    """Slopes (deg) of the plane fitted to ground residuals r over the rover's (forward u, left v) offsets: one fit
    over all returns, and per scan (median and 95th percentile of |slope|). 0 = ground truth level with the LiDAR."""
    def fit(m):
        a = np.c_[np.ones(m.sum()), u[m], v[m]]
        c = np.linalg.lstsq(a, r[m], rcond=None)[0]
        return np.degrees(np.arctan(c[1])), np.degrees(np.arctan(c[2]))
    allf = fit(np.ones(len(r), bool))
    per = np.array([fit(scan == k) for k in np.unique(scan) if (scan == k).sum() >= min_returns])
    out = {"forward_slope_deg": round(float(allf[0]), 4), "left_slope_deg": round(float(allf[1]), 4)}
    if len(per):
        out.update({"scans": int(len(per)),
                    "per_scan_forward_median_abs_deg": round(float(np.median(np.abs(per[:, 0]))), 4),
                    "per_scan_forward_p95_abs_deg": round(float(np.percentile(np.abs(per[:, 0]), 95)), 4),
                    "per_scan_left_median_abs_deg": round(float(np.median(np.abs(per[:, 1]))), 4),
                    "per_scan_left_p95_abs_deg": round(float(np.percentile(np.abs(per[:, 1]), 95)), 4)})
    return out


# ------------------------------------------------------------------ rotations
def quat_to_rot(q):
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = np.moveaxis(q, -1, 0)
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
                     np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
                     np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def log_so3(r):
    """(N, 3, 3) -> (N, 3) rotation vectors."""
    c = np.clip((np.trace(r, axis1=1, axis2=2) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.stack([r[:, 2, 1] - r[:, 1, 2], r[:, 0, 2] - r[:, 2, 0], r[:, 1, 0] - r[:, 0, 1]], -1)
    s = np.where(th < 1e-9, 0.5, th / (2 * np.sin(np.maximum(th, 1e-9))))
    return v * s[:, None]


def interp_pose(t_q, t, p, q):
    """Positions linearly, orientations by normalised quaternion interpolation (nlerp; fine at 50 Hz)."""
    i = np.clip(np.searchsorted(t, t_q) - 1, 0, len(t) - 2)
    a = ((t_q - t[i]) / (t[i + 1] - t[i]))[:, None]
    pos = p[i] * (1 - a) + p[i + 1] * a
    q1 = q[i + 1] * np.sign(np.sum(q[i] * q[i + 1], axis=1))[:, None]
    qq = q[i] * (1 - a) + q1 * a
    return pos, quat_to_rot(qq / np.linalg.norm(qq, axis=1, keepdims=True))


def smooth(x, n):
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, j], k, mode="same") for j in range(x.shape[1])], 1)


# ------------------------------------------------------------------ terrain
class Terrain:
    """The build's final 1 cm height tiles (heights/tile_*.npy + .json), bilinear, each tile loaded on first use.
    Each tile covers its own cell plus a margin (meta.margin_m, 0.5 m) that overlaps its neighbours, so a point
    takes the tile whose own cell contains it; a margin is used only where no cell does (the outer border)."""

    def __init__(self, folder: Path):
        self.tiles = []
        for js in sorted(glob.glob(str(folder / "tile_*.json"))):
            info = json.loads(Path(js).read_text())
            s, x0, y0 = info["spacing"], info["x0"], info["y0"]
            x1, y1 = x0 + (info["nx"] - 1) * s, y0 + (info["ny"] - 1) * s
            mg = float(info.get("meta", {}).get("margin_m", 0.0)) - 1e-6     # 1 um: a point on a cell edge is in it
            self.tiles.append({"x0": x0, "y0": y0, "s": s, "npy": js[:-5] + ".npy", "grid": None,
                               "full": (x0, x1, y0, y1), "cell": (x0 + mg, x1 - mg, y0 + mg, y1 - mg)})

    def height(self, x, y):
        z = np.full(x.shape, np.nan)
        if not x.size:
            return z
        bx0, bx1, by0, by1 = np.nanmin(x), np.nanmax(x), np.nanmin(y), np.nanmax(y)
        for rect in ("cell", "full"):
            for t in self.tiles:
                rx0, rx1, ry0, ry1 = t[rect]
                if rx1 < bx0 or rx0 > bx1 or ry1 < by0 or ry0 > by1:
                    continue
                m = np.isnan(z) & (x >= rx0) & (x <= rx1) & (y >= ry0) & (y <= ry1)
                if m.any():
                    z[m] = self._bilinear(t, x[m], y[m])
        return z

    @staticmethod
    def _bilinear(t, x, y):
        if t["grid"] is None:
            t["grid"] = np.load(t["npy"], mmap_mode="r")
        g = t["grid"]
        fx, fy = (x - t["x0"]) / t["s"], (y - t["y0"]) / t["s"]
        i0 = np.clip(np.floor(fy).astype(int), 0, g.shape[0] - 2)
        j0 = np.clip(np.floor(fx).astype(int), 0, g.shape[1] - 2)
        ay, ax = fy - i0, fx - j0
        return (g[i0, j0] * (1 - ax) * (1 - ay) + g[i0, j0 + 1] * ax * (1 - ay) +
                g[i0 + 1, j0] * (1 - ax) * ay + g[i0 + 1, j0 + 1] * ax * ay)


# ------------------------------------------------------------------ bag
def read_bag(path: Path, every: int):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = ["/ground_truth/odom", "/husky/sensors/imu_0/data_raw", "/husky/sensors/seyond/points", "/tf_static"]
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    cls = {t: get_message(types[t]) for t in topics if t in types}
    gt, imu, scans, static = [], [], [], {}
    n_scan = 0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/husky/sensors/seyond/points":
            n_scan += 1
            if n_scan % every:
                continue
        m = deserialize_message(data, cls[topic])
        if topic == "/ground_truth/odom":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            gt.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif topic == "/husky/sensors/imu_0/data_raw":
            w, a = m.angular_velocity, m.linear_acceleration
            imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, w.x, w.y, w.z, a.x, a.y, a.z))
        elif topic == "/tf_static":
            for tr in m.transforms:
                p, q = tr.transform.translation, tr.transform.rotation
                static[(tr.header.frame_id, tr.child_frame_id)] = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
        else:
            off = {f.name: f.offset for f in m.fields}
            dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                           "offsets": [off["x"], off["y"], off["z"]], "itemsize": m.point_step})
            pts = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
            xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(float)
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, xyz[np.isfinite(xyz).all(1)]))
    return np.array(sorted(gt)), np.array(sorted(imu)), scans, static


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", type=Path)
    ap.add_argument("--heights", type=Path, default=Path.home() / "togo_assets/lunar/build/apollo16_core/heights")
    ap.add_argument("--every", type=int, default=20, help="use every Nth LiDAR scan")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    out_dir = a.out or a.bag
    gt, imu, scans, static = read_bag(a.bag, a.every)
    t, p, q = gt[:, 0], gt[:, 1:4], gt[:, 4:8]
    rot = quat_to_rot(q)
    res = {"bag": str(a.bag), "gt_samples": int(len(gt)), "imu_samples": int(len(imu)), "scans_checked": len(scans)}

    # extrinsics from /tf_static: compose the chain from base_link down to the sensor frame (the LiDAR hangs off
    # mount links, not directly off base_link)
    parent_of = {ch: (par, tr, qq) for (par, ch), (tr, qq) in static.items()}

    def extr(child):
        T = np.eye(4)
        frame = child
        for _ in range(32):
            if frame == "base_link":
                return T[:3, 3].copy(), T[:3, :3].copy()
            if frame not in parent_of:
                return None
            par, tr, qq = parent_of[frame]
            step = np.eye(4)
            step[:3, :3], step[:3, 3] = quat_to_rot(qq[None])[0], tr
            T = step @ T
            frame = par
        return None
    e_imu, e_lid = extr("imu_0_link"), extr("lidar3d_0_laser")
    res["extrinsics"] = {"imu": None if e_imu is None else [round(float(v), 4) for v in e_imu[0]],
                         "lidar": None if e_lid is None else [round(float(v), 4) for v in e_lid[0]]}
    r_imu = e_imu[0] if e_imu else np.zeros(3)
    R_bi = e_imu[1] if e_imu else np.eye(3)

    # ---------------------------------------------------------------- 1 gyro
    dtg = np.diff(t)
    w_gt = log_so3(np.einsum("nji,njk->nik", rot[:-1], rot[1:])) / dtg[:, None]     # body frame
    t_mid = (t[:-1] + t[1:]) / 2
    w_gt_imu = w_gt @ R_bi                                                           # into imu_0_link axes
    gyro = {}
    for s in SHIFTS_MS:
        wi = np.stack([np.interp(t_mid + s / 1000, imu[:, 0], imu[:, k]) for k in (1, 2, 3)], 1)
        gyro[int(s)] = float(np.sqrt(np.mean((wi - w_gt_imu) ** 2)))
    best = min(gyro, key=gyro.get)
    res["gyro"] = {"rms_rad_s_at_0ms": round(gyro[0], 5), "best_shift_ms": best, "rms_rad_s_at_best": round(gyro[best], 5),
                   "imu_rate_std_rad_s": round(float(np.std(imu[:, 1:4])), 4)}

    # ---------------------------------------------------------------- 2 accelerometer (specific force at the IMU)
    p_imu = p + rot @ r_imu
    n = max(3, int(round(0.3 / np.median(dtg))))
    ps = smooth(p_imu, n)
    acc_w = np.gradient(np.gradient(ps, t, axis=0), t, axis=0)
    f_body = np.einsum("nji,nj->ni", rot, acc_w + np.array([0.0, 0.0, GRAVITY]))      # world -> base_link
    f_imu = f_body @ R_bi
    ai = np.stack([np.interp(t, imu[:, 0], imu[:, k]) for k in (4, 5, 6)], 1)
    ai = smooth(ai, n)
    edge = 3 * n
    d = (ai - f_imu)[edge:-edge]
    res["accel"] = {"rms_m_s2": [round(float(v), 4) for v in np.sqrt(np.mean(d ** 2, 0))],
                    "mean_m_s2": [round(float(v), 4) for v in d.mean(0)],
                    "gravity_at_rest_m_s2": round(float(np.linalg.norm(imu[:200, 4:7].mean(0))), 4)}

    # ---------------------------------------------------------------- 3 LiDAR onto the true terrain
    terrain = Terrain(a.heights)
    T_bl = np.eye(4)
    if e_lid:
        T_bl[:3, :3], T_bl[:3, 3] = e_lid[1], e_lid[0]
    keep = []
    for ts, xyz in scans:
        if t[0] + 0.1 < ts < t[-1] - 0.1:
            rng = np.linalg.norm(xyz, axis=1)
            keep.append((ts, xyz[(rng > 1.0) & (rng < 15.0)]))

    def residuals(shift_s, sub, detail=False):
        """Return height minus terrain; with detail also range, (forward, left) offset from the sensor in the
        rover's heading frame and the scan index, as a dict of arrays."""
        cols = {"r": [], "rng": [], "u": [], "v": [], "scan": []}
        for k, (ts, xyz) in enumerate(sub):
            pos, R = interp_pose(np.array([ts + shift_s]), t, p, q)
            Rw = R[0] @ T_bl[:3, :3]
            origin = R[0] @ T_bl[:3, 3] + pos[0]
            pw = xyz @ Rw.T + origin
            cols["r"].append(pw[:, 2] - terrain.height(pw[:, 0], pw[:, 1]))
            if detail:
                yaw = np.arctan2(R[0][1, 0], R[0][0, 0])
                dx, dy = pw[:, 0] - origin[0], pw[:, 1] - origin[1]
                cols["rng"].append(np.linalg.norm(xyz, axis=1))
                cols["u"].append(np.cos(yaw) * dx + np.sin(yaw) * dy)
                cols["v"].append(-np.sin(yaw) * dx + np.cos(yaw) * dy)
                cols["scan"].append(np.full(len(xyz), k))
        r = np.concatenate(cols["r"])
        ok = np.isfinite(r)
        if not detail:
            return r[ok]
        return {name: np.concatenate(v)[ok] for name, v in cols.items()}

    d0 = residuals(0.0, keep, detail=True)                 # all checked scans, all their returns
    r0, rng0 = d0["r"], d0["rng"]
    # by range: the median must stay ~0; the spread should follow the gpu_lidar quantisation (module docstring)
    res["lidar_by_range"] = {}
    for lo, hi in ((1, 3), (3, 6), (6, 10), (10, 15)):
        m = (rng0 >= lo) & (rng0 < hi)
        g = r0[m & (np.abs(r0) < 0.1)]
        if len(g):
            res["lidar_by_range"][f"{lo}-{hi} m"] = {
                "returns": int(m.sum()), "median_m": round(float(np.median(g)), 5),
                "spread_mm": round(float(1.4826 * np.median(np.abs(g - np.median(g))) * 1000), 1),
                "expected_spread_mm": round(float(QUANT_SIGMA_PER_M * np.sqrt(np.mean(rng0[m] ** 2)) * 1000), 1),
                "below_2cm_pct": round(float(np.mean(r0[m] < -0.02) * 100), 1),
                "above_5cm_pct": round(float(np.mean(r0[m] > 0.05) * 100), 1)}
    gm = (np.abs(r0) < 0.05) & (rng0 < 10)                 # ground returns, rocks excluded
    res["lidar_tilt"] = ground_tilt(r0[gm], d0["u"][gm], d0["v"][gm], d0["scan"][gm])
    # time-offset search: a stamp error moves the points only while the rover moves, most in turns and far away,
    # so use the scans taken while turning (all scans if there are too few) and their returns at 3-15 m
    yaw = np.unwrap(np.arctan2(rot[:, 1, 0], rot[:, 0, 0]))
    yaw_rate = np.gradient(yaw, t)
    turning = [(ts, xyz) for ts, xyz in keep if abs(np.interp(ts, t, yaw_rate)) > 0.3]
    turning_only = len(turning) >= 10
    sub = [(ts, xyz[np.linalg.norm(xyz, axis=1) > 3.0]) for ts, xyz in (turning if turning_only else keep)]
    lid = {}
    for s in SHIFTS_MS:
        r = residuals(s / 1000, sub)
        ground = r[np.abs(r) < 0.2]                        # rocks stand above the grid
        lid[int(s)] = (float(np.mean(np.abs(ground))) if len(ground) else np.nan,)
    best_l = min(lid, key=lambda k: lid[k][0])
    res["lidar_shift_search"] = {"scans": len(sub), "turning_only": turning_only,
                                 "mean_abs_residual_mm": {k: round(v[0] * 1000, 2) for k, v in lid.items()}}
    res["gyro_shift_search_rms_rad_s"] = {k: round(v, 5) for k, v in gyro.items()}
    near = r0[np.abs(r0) < 0.05]
    res["lidar"] = {"returns_checked": int(len(r0)), "median_residual_m": round(float(np.median(near)), 5),
                    "median_abs_residual_m": round(float(np.median(np.abs(near))), 5),
                    "within_1cm_pct": round(float(np.mean(np.abs(r0) < 0.01) * 100), 1),
                    "within_2cm_pct": round(float(np.mean(np.abs(r0) < 0.02) * 100), 1),
                    "above_5cm_pct (rocks)": round(float(np.mean(r0 > 0.05) * 100), 1),
                    "below_2cm_pct": round(float(np.mean(r0 < -0.02) * 100), 2),
                    "best_shift_ms": best_l}
    tilt = res["lidar_tilt"]
    ok = (abs(res["gyro"]["best_shift_ms"]) <= 10 and abs(best_l) <= 10 and res["lidar"]["median_abs_residual_m"] < 0.01
          and max(abs(tilt["forward_slope_deg"]), abs(tilt["left_slope_deg"])) < 0.05)
    res["verdict"] = "ground truth consistent with IMU and LiDAR" if ok else "INCONSISTENT: see the numbers"
    (out_dir / "gt_check.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 2, figsize=(14, 9))
        wi = np.stack([np.interp(t_mid, imu[:, 0], imu[:, k]) for k in (1, 2, 3)], 1)
        ax[0, 0].plot(t_mid - t[0], wi[:, 2], lw=0.8, label="IMU gyro z")
        ax[0, 0].plot(t_mid - t[0], w_gt_imu[:, 2], lw=0.8, ls="--", label="from ground truth")
        ax[0, 0].set_title("yaw rate (rad/s)"); ax[0, 0].legend(fontsize=8)
        tt = (t - t[0])[edge:-edge]
        ax[0, 1].plot(tt, ai[edge:-edge, 0], lw=0.8, label="IMU accel x")
        ax[0, 1].plot(tt, f_imu[edge:-edge, 0], lw=0.8, ls="--", label="from ground truth")
        ax[0, 1].set_title("forward specific force (m/s^2, 0.3 s smoothing)"); ax[0, 1].legend(fontsize=8)
        ax[1, 0].hist(np.clip(r0, -0.05, 0.15), bins=200)
        ax[1, 0].set_title("LiDAR return height minus true terrain (m); rocks > 0")
        ax[1, 1].plot(list(gyro), np.array(list(gyro.values())) / max(gyro.values()), "o-", label="gyro RMS (scaled)")
        ax[1, 1].plot(list(lid), np.array([v[0] for v in lid.values()]) / max(v[0] for v in lid.values()), "s-",
                      label="LiDAR mean |residual|, turning scans, 3-15 m (scaled)")
        ax[1, 1].set_xlabel("ground-truth time shift (ms)"); ax[1, 1].set_title("best fit should be at 0 ms")
        ax[1, 1].legend(fontsize=8)
        fig.savefig(out_dir / "gt_check.png", dpi=90, bbox_inches="tight")
    except Exception as e:  # noqa: BLE001
        print(f"(figure skipped: {e})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
