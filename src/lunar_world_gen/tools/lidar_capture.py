#!/usr/bin/python3
"""Capture gpu_lidar point clouds from static sensors injected into a copy of a world SDF.

Run with the SYSTEM python3 (gz-transport13 / gz-msgs10), from WSL:

    /usr/bin/python3 tools/lidar_capture.py WORLD.sdf --out DIR \
        [--pose-agl X,Y,H[,YAW]]... [--pose X,Y,Z[,YAW]]... [--poses-file poses.json] \
        [--auto-poses N] [--grid PREFIX | --config CFG | --ground-from-sdf] \
        [--lidar real|robinw] [--clouds 1] [--skip 2] [--batch 6]

Poses are sensor positions: ``--pose-agl`` gives the height above the ground under the sensor
(ground from ``--grid``, from the core grid found through ``--config`` or from the world's own
static GLB visuals with ``--ground-from-sdf``); ``--pose`` gives absolute z. Yaw is in radians.
``--auto-poses N`` spreads N deterministic poses over the core (centre, then outward-looking
poses 35 m from the centre, then diagonal inward-looking ones) at the rover's LiDAR height.

Each pose becomes one static gpu_lidar model (``tools/bench_world.py`` helpers); poses are run
``--batch`` at a time per server start. For every pose a ``capture_<k>.npz`` is written with the
fields documented in ``lunar_world_gen/validate/lidar_checks.py`` (world-frame points with NaN
for no return, ranges, nominal beam angles verified against the returned directions, sensor
pose); ``capture_run.json`` records load times, renderer evidence and server-log errors.
Analyse with ``python -m lunar_world_gen.validate.lidar_checks DIR``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import bench_world as bw  # noqa: E402


def auto_poses(n: int, h: float, core_half: float = 50.0) -> list[str]:
    """Deterministic poses x,y,h,yaw spread over the core (strings for resolve_poses)."""
    r = 0.7 * core_half
    base = [(0.0, 0.0, 0.0), (r, 0.0, 0.0), (-r, 0.0, math.pi), (0.0, r, math.pi / 2), (0.0, -r, -math.pi / 2),
            (0.5 * r, 0.5 * r, -3 * math.pi / 4), (-0.5 * r, -0.5 * r, math.pi / 4),
            (0.5 * r, -0.5 * r, 3 * math.pi / 4), (-0.5 * r, 0.5 * r, -math.pi / 4)]
    out = []
    for k in range(n):
        x, y, yaw = base[k % len(base)]
        if k >= len(base):  # further poses on a shrinking ring
            f = 1.0 - 0.15 * (k // len(base))
            x, y = x * f, y * f
        out.append(f"{x:.3f},{y:.3f},{h:.3f},{yaw:.6f}")
    return out


def beam_angles(preset: dict) -> tuple[np.ndarray, np.ndarray]:
    az = np.linspace(preset["hmin"], preset["hmax"], int(preset["hs"]))
    el = np.linspace(preset["vmin"], preset["vmax"], int(preset["vs"]))
    return az, el


def verify_angles(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray, az: np.ndarray, el: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    """Check (and if needed reverse) the nominal column/row angle order against the returns."""
    fin = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
    info: dict = {}
    if fin.sum() < 20:
        info["verified"] = False
        return az, el, info
    import warnings
    with np.errstate(all="ignore"):
        a_meas = np.where(fin, np.arctan2(ys, xs), np.nan)
        e_meas = np.where(fin, np.arctan2(zs, np.hypot(xs, ys)), np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # rows/columns without any return (sky)
        col = np.nanmedian(a_meas, axis=0)
        row = np.nanmedian(e_meas, axis=1)
    okc, okr = np.isfinite(col), np.isfinite(row)
    if okc.sum() > 2 and np.corrcoef(col[okc], az[okc])[0, 1] < 0:
        az = az[::-1].copy()
        info["azimuth_reversed"] = True
    if okr.sum() > 2 and np.corrcoef(row[okr], el[okr])[0, 1] < 0:
        el = el[::-1].copy()
        info["elevation_reversed"] = True
    info["max_dev_az_deg"] = float(np.degrees(np.nanmax(np.abs(col[okc] - az[okc])))) if okc.any() else None
    info["max_dev_el_deg"] = float(np.degrees(np.nanmax(np.abs(row[okr] - el[okr])))) if okr.any() else None
    info["verified"] = True
    return az, el, info


def save_capture(path: Path, cloud: dict, pose_rec: dict, preset: dict, meta: dict) -> dict:
    d = cloud["data"]
    xs, ys, zs = (np.asarray(d[k], dtype=np.float64) for k in ("x", "y", "z"))
    az, el = beam_angles(preset)
    if xs.shape != (len(el), len(az)):
        raise ValueError(f"cloud shape {xs.shape} != ({len(el)}, {len(az)})")
    az, el, chk = verify_angles(xs, ys, zs, az, el)
    Ps = np.stack([xs, ys, zs], -1)
    rng_ = np.linalg.norm(Ps, axis=-1)
    fin = np.isfinite(rng_)
    x, y, z, roll, pitch, yaw = pose_rec["pose"]
    R = bw_rpy(roll, pitch, yaw)
    Pw = np.full(Ps.shape, np.nan)
    Pw[fin] = Ps[fin] @ R.T + np.array([x, y, z])
    rng_out = np.where(fin, rng_, np.inf)
    meta = dict(meta)
    meta.update({"stamp": cloud["stamp"], "frame_id": cloud["frame"], "beam_check": chk,
                 "request": pose_rec["request"], "pose_mode": pose_rec["mode"], "lidar": preset})
    np.savez_compressed(path, points=Pw.astype(np.float32), points_sensor=Ps.astype(np.float32),
                        ranges=rng_out.astype(np.float32), azimuth=az, elevation=el,
                        pose=np.asarray(pose_rec["pose"], dtype=np.float64),
                        request=np.asarray(pose_rec["request"], dtype=np.float64),
                        ground_z=np.float64(pose_rec["ground_z"] if pose_rec["ground_z"] is not None else np.nan),
                        stamp=np.float64(cloud["stamp"]), meta_json=json.dumps(meta, default=str))
    return {"file": str(path), "returns": int(fin.sum()), "rays": int(fin.size), "beam_check": chk}


def bw_rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def run_batch(a: argparse.Namespace, world: str, batch: list[dict], k0: int, out: Path, preset: dict) -> dict:
    lidars = [{"name": f"validation_cap_{k0 + i}", "pose": p["pose"], "topic": f"/validation/cap_{k0 + i}",
               "preset": preset, "rate": a.rate} for i, p in enumerate(batch)]
    work = out / f"run_{k0:03d}"
    copy = work / f"capture_{Path(world).stem}.sdf"
    rec: dict = {"poses": [p["pose"] for p in batch], "status": "init"}
    rec["sdf_edit"] = bw.prepare_world(world, copy, lidars, strip_lidars=a.strip_existing_lidars,
                                       remove_models=a.remove_model)
    topics = [f"{L['topic']}/points" for L in lidars]
    watcher = bw.Watcher(rec["sdf_edit"]["world_name"], topics, keep_clouds=a.clouds, skip_clouds=a.skip)
    env = bw.gz_env(a.partition, a.domain, work / "ros_home")
    srv = bw.GzServer(copy, work / "server.log", env, limit_s=int(a.timeout + 60))
    try:
        srv.start()
        rec["pgid"] = srv.pgid
        tend = srv.t0 + a.timeout
        while bw.clock() < tend:
            time.sleep(0.25)
            if srv.proc.poll() is not None and not srv.alive():
                rec["status"] = "server_exited"
                break
            with watcher.lock:
                done = all(len(watcher.clouds[t]) >= a.clouds for t in topics)
            if done:
                rec["status"] = "ok"
                break
        else:
            rec["status"] = "timeout"
        with watcher.lock:
            rec["first_lidar_s"] = {t: round(v - srv.t0, 3) for t, v in watcher.first_lidar.items()}
            clouds = {t: list(v) for t, v in watcher.clouds.items()}
        pid = srv.find_server_pid()
        if pid:
            rec["renderer"] = bw.renderer_libs([p for p, _ in bw.group_pids(srv.pgid)])
            rec["rss_mb"] = round((bw.mem_status(pid).get("VmHWM") or 0) / 1024, 1)
    finally:
        rec["stop"] = srv.stop() if srv.pgid else {}
        rec["log"] = srv.log_summary()
    files = []
    for i, (p, t) in enumerate(zip(batch, topics)):
        for j, cl in enumerate(clouds.get(t, [])):
            name = f"capture_{k0 + i:03d}" + (f"_{j}" if a.clouds > 1 else "") + ".npz"
            meta = {"sdf": str(Path(world).resolve()), "copy_sdf": str(copy), "topic": t, "index": k0 + i,
                    "cloud": j, "partition": a.partition}
            files.append(save_capture(out / name, cl, p, preset, meta))
    rec["files"] = files
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("world")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--pose", action="append", default=[], help="absolute x,y,z[,yaw]")
    ap.add_argument("--pose-agl", action="append", default=[], help="x,y,height_above_ground[,yaw]")
    ap.add_argument("--poses-file", help="JSON list of [x, y, h_agl, yaw] (or dicts with x, y, h, yaw)")
    ap.add_argument("--auto-poses", type=int, default=0, help="N deterministic poses in the core")
    ap.add_argument("--height", type=float, default=bw.ROVER_LIDAR_HEIGHT_M)
    ap.add_argument("--grid", help="ground height grid prefix")
    ap.add_argument("--config", help="generator config: use its core height grid as the ground (if built)")
    ap.add_argument("--ground-from-sdf", action="store_true", help="ground from the world's static GLB visuals")
    ap.add_argument("--ground-sdf", help="ground from the static GLB visuals of this other SDF (e.g. rock-free)")
    ap.add_argument("--lidar", default="real", choices=sorted(bw.LIDAR_PRESETS))
    ap.add_argument("--rate", type=float, default=10.0, help="sensor update rate for captures [Hz]")
    ap.add_argument("--clouds", type=int, default=1, help="clouds saved per pose")
    ap.add_argument("--skip", type=int, default=2, help="clouds skipped first per pose")
    ap.add_argument("--batch", type=int, default=6, help="sensors per server run")
    ap.add_argument("--timeout", type=float, default=300.0, help="per batch: load + capture wall limit [s]")
    ap.add_argument("--strip-existing-lidars", action="store_true")
    ap.add_argument("--remove-model", action="append", default=[])
    ap.add_argument("--partition", default=bw.DEFAULT_PARTITION)
    ap.add_argument("--domain", type=int, default=bw.DEFAULT_DOMAIN)
    a = ap.parse_args(argv)
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    os.environ["GZ_PARTITION"] = a.partition
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    grid = a.grid
    core_half = 50.0
    sdf_ground = a.ground_sdf or (a.world if a.ground_from_sdf else None)
    use_cfg = None
    if a.config:
        from lunar_world_gen.config import load_config
        cfg = load_config(a.config)
        core_half = float(cfg.zones.core.size_m) / 2
        # ground of the build: a single core grid, else the final per-tile heights + the ring grid
        # (before, a build without a single core grid left the sensors at z = 0 + height)
        if not grid and not sdf_ground:
            use_cfg = a.config
    ginfo: dict = {}
    hfn = bw.ground_height_fn(grid, sdf_ground, use_cfg, ginfo)
    agl = list(a.pose_agl)
    if a.poses_file:
        for p in json.loads(Path(a.poses_file).read_text()):
            v = [p["x"], p["y"], p.get("h", a.height), p.get("yaw", 0.0)] if isinstance(p, dict) else list(p)
            agl.append(",".join(str(float(t)) for t in v))
    if a.auto_poses:
        agl += auto_poses(a.auto_poses, a.height, core_half)
    poses = bw.resolve_poses(a.pose, agl, hfn, a.height)
    preset = dict(bw.LIDAR_PRESETS[a.lidar])
    preset["rate"] = a.rate
    run = {"kind": "lidar_capture", "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "world": str(Path(a.world).resolve()),
           "ground_source": ginfo.get("ground_source"), "ground": ginfo, "lidar": preset, "clock": "monotonic",
           "poses": poses, "machine_before": bw.machine_state(), "batches": []}

    def on_signal(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    ok = True
    try:
        for k0 in range(0, len(poses), max(1, a.batch)):
            rec = run_batch(a, a.world, poses[k0:k0 + a.batch], k0, out, preset)
            run["batches"].append(rec)
            ok &= rec["status"] == "ok"
            print(f"[lidar_capture] batch {k0}: {rec['status']} files={len(rec['files'])} "
                  f"first_lidar={rec.get('first_lidar_s')} d3d12={rec.get('renderer', {}).get('d3d12')}", flush=True)
    except KeyboardInterrupt as e:
        run["status"] = f"interrupted ({e})"
        ok = False
    run["status"] = run.get("status", "ok" if ok else "incomplete")
    run["machine_after"] = bw.machine_state()
    (out / "capture_run.json").write_text(json.dumps(run, indent=1, default=str))
    print(f"[lidar_capture] wrote {out}/capture_run.json status={run['status']}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    finally:
        sys.stdout.flush()
        os._exit(code)
