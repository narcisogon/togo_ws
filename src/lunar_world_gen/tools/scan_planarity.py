"""Planarity of 16-NN neighbourhoods on single voxel-filtered scans from the lap bag, as DLIO's source
covariances see them. If flat-ground points come out line-like (planarity well below 1), ADAPTIVE kept a
scan-line-shaped covariance for them, i.e. a fake across-ring constraint that PLANE discards."""
import sys, numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree
BAG = Path('/home/nargon/togo_assets/lunar/bags/apollo16_lap1_20261027')
TOPIC = '/husky/sensors/seyond/points'
TIMES = {'standstill': 15.0, 'rocks': 60.0, 'slope': 130.0, 'plain': 230.0, 'return': 320.0}
K, VOXEL = 16, 0.15

def points_of(msg):
    off = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height * msg.width, msg.point_step)
    xyz = np.stack([buf[:, off[c]:off[c] + 4].copy().view(np.float32)[:, 0] for c in 'xyz'], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]

def voxel(xyz, res):
    key = np.floor(xyz / res).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    n = np.bincount(inv)
    cent = np.stack([np.bincount(inv, xyz[:, i]) / n for i in range(3)], axis=1)
    return cent

def analyse(xyz, name):
    tree = cKDTree(xyz)
    _, nn = tree.query(xyz, k=K)
    nb = xyz[nn]                                  # (N, K, 3), includes the point itself like nanoflann
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nb, nb) / K
    s, v = np.linalg.eigh(cov)                    # ascending
    s = s[:, ::-1]; normal = v[:, :, 0]           # s1 >= s2 >= s3; normal = smallest eigenvector
    s1 = np.maximum(s[:, 0], 1e-12)
    planarity = np.clip((s[:, 1] - s[:, 2]) / s1, 0, 1)
    elong = s[:, 1] / s1                          # s2/s1: 1 = round patch, small = a line
    across = planarity * 1.0 + (1 - planarity) * np.maximum(elong, 1e-3)   # ADAPTIVE eigenvalue across the line (PLANE: 1)
    ground = (np.abs(normal[:, 2]) > 0.9) & (xyz[:, 2] < -0.2)
    r = np.linalg.norm(xyz[:, :2], axis=1)
    for label, m in (('ground', ground), ('non-ground', ~ground)):
        if m.sum() < 50: continue
        p = planarity[m]; e = elong[m]; a = across[m]
        print(f"{name:10s} {label:10s} n={m.sum():5d}  planarity p10/p50/p90 = {np.percentile(p,10):.2f}/{np.percentile(p,50):.2f}/{np.percentile(p,90):.2f}"
              f"  share p<0.5: {np.mean(p<0.5)*100:4.0f} %  s2/s1 median {np.median(e):.2f}"
              f"  ADAPTIVE across-line eigenvalue median {np.median(a):.2f} (PLANE 1.00)")
    g = ground & (r > 3)
    for lo, hi in ((3, 6), (6, 10), (10, 20)):
        m = g & (r >= lo) & (r < hi)
        if m.sum() >= 50:
            print(f"{'':10s} ground {lo:2d}-{hi:2d} m: n={m.sum():5d} planarity median {np.median(planarity[m]):.2f}  s2/s1 median {np.median(elong[m]):.2f}  across-line eigenvalue median {np.median(across[m]):.2f}")

with AnyReader([BAG]) as reader:
    conns = [c for c in reader.connections if c.topic == TOPIC]
    t0 = None; want = dict(TIMES); done = {}
    for conn, ts, raw in reader.messages(connections=conns):
        if t0 is None: t0 = ts
        t = (ts - t0) * 1e-9
        for name, at in list(want.items()):
            if t >= at:
                msg = reader.deserialize(raw, conn.msgtype)
                xyz = points_of(msg); vx = voxel(xyz, VOXEL)
                print(f"== {name} (t={t:.1f} s): raw {len(xyz)} points, {len(vx)} after the {VOXEL} m voxel filter")
                analyse(vx, name); del want[name]
        if not want: break
