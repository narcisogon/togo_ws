#!/usr/bin/env python3
"""Picture of a coloured map PCD (graph_based_slam texture/enabled: map_colored.pcd, x y z rgb) without RViz: a
top-down view painted with the camera gray, and the gray histogram. Reads ascii, binary and binary_compressed PCDs
(numpy only; the LZF decoder is here).

usage: colored_map_view.py MAP_COLORED.pcd [--out PNG] [--max-points N]
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np


def lzf_decompress(src: bytes, out_len: int) -> bytes:
    """LibLZF decompression (the format PCL uses for binary_compressed)."""
    out = bytearray(out_len)
    i, o = 0, 0
    n = len(src)
    while i < n:
        ctrl = src[i]
        i += 1
        if ctrl < 32:                       # literal run
            length = ctrl + 1
            out[o:o + length] = src[i:i + length]
            i += length
            o += length
        else:                               # back reference
            length = ctrl >> 5
            if length == 7:
                length += src[i]
                i += 1
            ref = o - ((ctrl & 0x1F) << 8) - src[i] - 1
            i += 1
            length += 2
            for _ in range(length):         # byte by byte: the reference may overlap the output
                out[o] = out[ref]
                o += 1
                ref += 1
    return bytes(out[:o])


def read_pcd(path: Path):
    raw = path.read_bytes()
    header_end = raw.index(b"DATA ")
    nl = raw.index(b"\n", header_end)
    header = raw[:nl].decode("ascii", "replace")
    body = raw[nl + 1:]
    fields = re.search(r"^FIELDS (.+)$", header, re.M).group(1).split()
    sizes = [int(v) for v in re.search(r"^SIZE (.+)$", header, re.M).group(1).split()]
    types = re.search(r"^TYPE (.+)$", header, re.M).group(1).split()
    counts = [int(v) for v in re.search(r"^COUNT (.+)$", header, re.M).group(1).split()] if re.search(r"^COUNT", header, re.M) else [1] * len(fields)
    n = int(re.search(r"^POINTS (\d+)", header, re.M).group(1))
    data_kind = re.search(r"^DATA (\w+)", header, re.M).group(1)
    np_types = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
    dtype = np.dtype([(f, np_types[(t, s)], (c,) if c > 1 else ()) for f, s, t, c in zip(fields, sizes, types, counts)])
    if data_kind == "ascii":
        arr = np.loadtxt(body.decode().splitlines(), dtype=np.float64, max_rows=n)
        rec = np.zeros(n, dtype=dtype)
        col = 0
        for f, c in zip(fields, counts):
            rec[f] = arr[:, col:col + c].squeeze() if c > 1 else arr[:, col]
            col += c
        return rec
    if data_kind == "binary":
        return np.frombuffer(body, dtype=dtype, count=n)
    if data_kind == "binary_compressed":
        comp_len, uncomp_len = np.frombuffer(body[:8], dtype="<u4")
        flat = lzf_decompress(body[8:8 + comp_len], int(uncomp_len))
        rec = np.zeros(n, dtype=dtype)
        off = 0
        for f, s, c in zip(fields, sizes, counts):       # field-major layout
            width = s * c
            chunk = np.frombuffer(flat[off:off + width * n], dtype=dtype[f].base if c > 1 else dtype[f], count=n * c)
            rec[f] = chunk.reshape(n, c).squeeze() if c > 1 else chunk
            off += width * n
        return rec
    raise SystemExit(f"unknown DATA kind {data_kind}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcd", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--max-points", type=int, default=800000)
    a = ap.parse_args()
    rec = read_pcd(a.pcd)
    x, y, z = rec["x"].astype(np.float64), rec["y"].astype(np.float64), rec["z"].astype(np.float64)
    rgb = rec["rgb"].view(np.uint32) if rec["rgb"].dtype.kind == "f" else rec["rgb"].astype(np.uint32)
    gray = (rgb & 0xFF).astype(np.float64)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y, z, gray = x[ok], y[ok], z[ok], gray[ok]
    print(f"{a.pcd.name}: {len(x)} points, x {x.min():.1f}..{x.max():.1f} m, y {y.min():.1f}..{y.max():.1f} m, "
          f"gray mean {gray.mean():.1f}, p5 {np.percentile(gray, 5):.0f}, p50 {np.percentile(gray, 50):.0f}, p95 {np.percentile(gray, 95):.0f}")
    if len(x) > a.max_points:
        idx = np.random.RandomState(0).choice(len(x), a.max_points, replace=False)
        x, y, z, gray = x[idx], y[idx], z[idx], gray[idx]
    out = a.out or a.pcd.with_suffix(".png")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("matplotlib is needed for the picture")
    fig, axes = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw={"width_ratios": [3, 1]})
    order = np.argsort(z)                                    # higher points drawn last
    axes[0].scatter(x[order], y[order], c=gray[order], cmap="gray", vmin=0, vmax=255, s=0.6, marker=".", linewidths=0)
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].set_title(f"coloured map, top view: {len(x)} voxels painted from the left camera")
    axes[0].set_facecolor("#203040")
    axes[1].hist(gray, bins=64, range=(0, 255), color="gray")
    axes[1].set_title("camera gray per voxel")
    axes[1].set_xlabel("gray")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
