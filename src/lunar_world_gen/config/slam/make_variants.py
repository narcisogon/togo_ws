#!/usr/bin/env python3
"""Regenerate the switch variants in ``variants/`` from ``seyond_dlio_graph_lunar.yaml``.

A variant is the lunar config with a few parameter values changed (the "one config change away" switches of
docs/SLAM_SPEEDUP_PLAN.md); ``replay_slam_bag.sh --config variants/<file>`` and ``benchmark_slam.sh --config``
run them. Keeping them as copies went stale whenever the lunar config changed (2026-10-31: the VIO placement
moved to the rig's IMU and the copies still had imu_0), so this script rebuilds every one of them, verifying that
each key exists exactly once and keeping the line's comment.

Usage (any python3, stdlib only; run from anywhere)::

    python3 src/lunar_world_gen/config/slam/make_variants.py           # (re)write all variants
    python3 src/lunar_world_gen/config/slam/make_variants.py --check   # exit 1 if any is stale
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "seyond_dlio_graph_lunar.yaml"
OUT_DIR = HERE / "variants"

# variant name -> {parameter key: new value}; the key is the text before the colon on its line
VARIANTS: dict[str, dict[str, str]] = {
    "iekf_out_s015": {"odom/iekf/hessianScale": "0.15"},
    "iekf_out_s03": {"odom/iekf/hessianScale": "0.3"},
    "iekf_s1": {"odom/geo/estimator": "iekf", "odom/iekf/hessianScale": "1.0", "odom/gicp/minIterations": "3"},
    "lagfix": {"odom/gicp/minIterations": "3", "odom/geo/correctAtScanTime": "true",
               "odom/geo/scanTimeHorizontalOnly": "true"},
    "frontend_cadence": {"pointcloud/publishEveryNth": "1", "hazard/input_rate_hz": "0.0",
                         "modified_map_timed_min_interval_sec": "0.0"},   # every scan everywhere, as before round 6
    "lidar360": {"extrinsics/baselink2lidar/t": "[-0.403, 0.000, 1.314]",   # the mast's OS1 sensor frame in base_link
                 "hazard/lidar_frame": "lidar360_0_sensor_link",
                 "odom/preprocessing/cropBoxFilter/size": "1.5"},   # the rover's own body is in a 360-degree view from the mast
    "lidar360_lean": {"extrinsics/baselink2lidar/t": "[-0.403, 0.000, 1.314]",   # the mast LiDAR near the Seyond's compute
                      "hazard/lidar_frame": "lidar360_0_sensor_link",
                      "odom/preprocessing/cropBoxFilter/size": "1.5",
                      "odom/preprocessing/voxelFilter/res": "0.30",   # a 360-degree scan to 150 m voxelised at 0.15 m is ~4.5x the Seyond's work
                      "hazard/input_rate_hz": "2.5"},            # a full circle per scan: 2.5 Hz keeps the mapper from saturating
    "ref_load": {"pointcloud/publishEveryNth": "1", "hazard/input_rate_hz": "0.0",
                 "modified_map_timed_min_interval_sec": "0.0", "dem/write_period_sec": "0.0",
                 "hazard/grow_margin_m": "0.0"},   # the pre-round-6 cadences and bounds (with --frontend-map: the old load)
    "vio_out": {"odom/iekf/vio/enabled": "true"},
    "vio_prior": {"odom/iekf/vio/enabled": "true", "odom/gicp/prior/vioMode": "1"},
    "vio_edge": {"odom/iekf/vio/enabled": "true", "odom/gicp/recovery/reseedFromFilter": "true",
                 "use_vio_edge": "true", "vio_edge_auto_scale": "true"},
}


def render(name: str, overrides: dict[str, str], source_lines: list[str]) -> str:
    out, seen = [], {k: 0 for k in overrides}
    for line in source_lines:
        m = re.match(r"^(\s*)([A-Za-z0-9_/]+):\s*([^#\n]*?)(\s*)(#.*)?$", line)
        if m and m.group(2) in overrides:
            key = m.group(2)
            seen[key] += 1
            comment = m.group(5) or ""
            pad = m.group(4) if comment else ""
            line = f"{m.group(1)}{key}: {overrides[key]}{pad}{comment}\n"
        out.append(line)
    missing = [k for k, n in seen.items() if n != 1]
    if missing:
        raise SystemExit(f"variant {name}: keys not found exactly once in the lunar config: {missing}")
    # one line after the source's md5 line says what this file is
    for i, line in enumerate(out):
        if "source md5" in line:
            out.insert(i + 1, "# VARIANT '" + name + "' (config/slam/make_variants.py): "
                       + ", ".join(f"{k}: {v}" for k, v in overrides.items()) + "\n")
            break
    return "".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--check", action="store_true", help="report stale variants, write nothing")
    a = ap.parse_args(argv)
    src = SOURCE.read_text(encoding="utf-8").splitlines(keepends=True)
    OUT_DIR.mkdir(exist_ok=True)
    stale = 0
    for name, overrides in VARIANTS.items():
        path = OUT_DIR / f"seyond_dlio_graph_lunar__{name}.yaml"
        text = render(name, overrides, src)
        current = path.read_text(encoding="utf-8") if path.exists() else None
        if current == text:
            print(f"up to date: {path.name}")
            continue
        stale += 1
        if a.check:
            print(f"STALE: {path.name}")
        else:
            path.write_bytes(text.encode("utf-8"))      # bytes: no newline translation on Windows
            print(f"wrote {path.name}: " + ", ".join(f"{k}={v}" for k, v in overrides.items()))
    return 1 if (a.check and stale) else 0


if __name__ == "__main__":
    sys.exit(main())
