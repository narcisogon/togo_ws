#!/usr/bin/env python3
"""Exactness check of graph_based_slam's DEM: rasterise a saved map.pcd here (cells of `resolution`, terrain =
the lower `percentile` of the cell's heights, cells with fewer than `min_points` returns are NODATA) and compare
cell by cell with the measured_terrain.asc / observation_count.asc the backend wrote from the same map at the
same save. The incremental DEM (2026-11-01) must give the same numbers as the one-shot rasteriser did.

usage: dem_check.py SLAM_OUTPUT_DIR [--resolution 0.2] [--percentile 0.25] [--min-points 3]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from colored_map_view import read_pcd  # noqa: E402
from dem_compare import read_asc  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=Path)
    ap.add_argument("--resolution", type=float, default=0.2)
    ap.add_argument("--percentile", type=float, default=0.25)
    ap.add_argument("--min-points", type=int, default=3)
    a = ap.parse_args()
    rec = read_pcd(a.out / "map.pcd")
    x, y, z = (rec[k].astype(np.float64) for k in ("x", "y", "z"))
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[ok], y[ok], z[ok]
    res = a.resolution
    ix = np.floor(x / res).astype(np.int64)
    iy = np.floor(y / res).astype(np.int64)
    terrain, header = read_asc(a.out / "lunar_dem" / "measured_terrain.asc")
    count, _ = read_asc(a.out / "lunar_dem" / "observation_count.asc")
    count = np.nan_to_num(count, nan=0.0)
    ox, oy = header["xllcorner"], header["yllcorner"]
    h, w = terrain.shape
    cx = ix - int(round(ox / res))
    cy = iy - int(round(oy / res))
    inside = (cx >= 0) & (cy >= 0) & (cx < w) & (cy < h)
    print(f"map.pcd {len(x)} points, {inside.sum()} inside the raster {w}x{h} at {res} m (origin {ox:.2f}, {oy:.2f}); "
          f"outside {int((~inside).sum())}")
    cx, cy, zz = cx[inside], cy[inside], z[inside]
    flat = cy * w + cx
    order = np.argsort(flat, kind="stable")
    flat, zz = flat[order], zz[order]
    starts = np.flatnonzero(np.r_[True, flat[1:] != flat[:-1]])
    ends = np.r_[starts[1:], len(flat)]
    my_terrain = np.full(h * w, np.nan, dtype=np.float32)
    my_count = np.zeros(h * w, dtype=np.int64)
    for s, e in zip(starts, ends):
        n = e - s
        my_count[flat[s]] = n
        if n < a.min_points:
            continue
        vals = np.sort(zz[s:e])
        q = int(np.floor(a.percentile * (n - 1)))
        my_terrain[flat[s]] = vals[q]
    my_terrain = my_terrain.reshape(h, w)
    my_count = my_count.reshape(h, w)
    cnt_diff = my_count - count.astype(np.int64)
    print(f"observation_count: cells with equal counts {np.mean(cnt_diff == 0) * 100:.3f} %, max |diff| {np.abs(cnt_diff).max()}")
    va, vb = np.isfinite(terrain), np.isfinite(my_terrain)
    both = va & vb
    d = terrain[both] - my_terrain[both]
    print(f"measured_terrain: backend valid {va.sum()}, here {vb.sum()}, both {both.sum()}, only backend {int((va & ~vb).sum())}, only here {int((vb & ~va).sum())}")
    if both.any():
        print(f"height difference on common cells: max |d| {np.abs(d).max():.5f} m, cells within 1 mm {np.mean(np.abs(d) < 1e-3) * 100:.3f} % "
              f"(ASCII grids carry 7 significant digits)")
    verdict = both.sum() == va.sum() == vb.sum() and np.abs(d).max() < 1e-3 and np.abs(cnt_diff).max() == 0
    print("RESULT", "exact" if verdict else "DIFFERS")


if __name__ == "__main__":
    main()
