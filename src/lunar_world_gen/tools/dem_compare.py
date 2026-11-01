#!/usr/bin/env python3
"""Compare two DEM outputs of graph_based_slam (lunar_dem folders with ESRI ASCII grids): extents, valid cells,
and on the common valid cells the height differences of measured_terrain.asc (and the other layers if present).
Used to check that the incremental DEM (2026-11-01) produces what the one-shot rasteriser produced.

usage: dem_compare.py DEM_DIR_A DEM_DIR_B [--layer measured_terrain]
"""
import argparse
from pathlib import Path

import numpy as np


def read_asc(path: Path):
    with open(path) as f:
        header = {}
        for _ in range(6):
            k, v = f.readline().split()
            header[k.lower()] = float(v)
        data = np.loadtxt(f, dtype=np.float32)
    w, h = int(header["ncols"]), int(header["nrows"])
    data = data.reshape(h, w)[::-1]                       # rows are north to south in the file; flip to y up
    nodata = header.get("nodata_value", -9999.0)
    data = np.where(data == nodata, np.nan, data)
    return data, header


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a", type=Path)
    ap.add_argument("b", type=Path)
    ap.add_argument("--layers", nargs="*", default=["measured_terrain", "measured_surface", "interpolated_terrain", "observation_count"])
    a = ap.parse_args()
    for layer in a.layers:
        pa, pb = a.a / f"{layer}.asc", a.b / f"{layer}.asc"
        if not pa.is_file() or not pb.is_file():
            print(f"{layer}: missing in one of the folders")
            continue
        da, ha = read_asc(pa)
        db, hb = read_asc(pb)
        res = ha["cellsize"]
        # align on world coordinates
        ox_a, oy_a, ox_b, oy_b = ha["xllcorner"], ha["yllcorner"], hb["xllcorner"], hb["yllcorner"]
        x0, y0 = max(ox_a, ox_b), max(oy_a, oy_b)
        x1 = min(ox_a + da.shape[1] * res, ox_b + db.shape[1] * res)
        y1 = min(oy_a + da.shape[0] * res, oy_b + db.shape[0] * res)
        if x1 <= x0 or y1 <= y0:
            print(f"{layer}: no overlap")
            continue
        ia = lambda d, ox, oy: d[int(round((y0 - oy) / res)):int(round((y1 - oy) / res)), int(round((x0 - ox) / res)):int(round((x1 - ox) / res))]
        sa, sb = ia(da, ox_a, oy_a), ia(db, ox_b, oy_b)
        n = min(sa.shape[0], sb.shape[0]), min(sa.shape[1], sb.shape[1])
        sa, sb = sa[:n[0], :n[1]], sb[:n[0], :n[1]]
        va, vb = np.isfinite(sa), np.isfinite(sb)
        both = va & vb
        d = (sa - sb)[both]
        print(f"{layer}: A {da.shape[1]}x{da.shape[0]} valid {va.sum()} | B {db.shape[1]}x{db.shape[0]} valid {vb.sum()} | "
              f"overlap {both.sum()} cells, only A {int((va & ~vb).sum())}, only B {int((vb & ~va).sum())} | "
              f"diff mean {d.mean():+.4f} rms {np.sqrt(np.mean(d ** 2)):.4f} p95|d| {np.percentile(np.abs(d), 95):.4f} max|d| {np.abs(d).max():.4f}"
              if both.any() else f"{layer}: no common valid cells")


if __name__ == "__main__":
    main()
