"""Plan the SLAM benchmark loop on apollo16_core.

Usage (WSL, in the package folder): PYTHONPATH=. ~/.venvs/lunar_world/bin/python tools/bag_record/plan_route.py OUT
Writes route.json and route.png into OUT. To make a new plan the benchmark route, copy OUT/route.json over
tools/bag_record/route.json (record_bag.sh's default).

Start: the most level, rock-free spot near the 'flat' site, so the rover is truly still for IMU calibration
(the 'flat' site itself has a 4.85 deg footprint and the rover creeps 2-4 mm/s there).
Loop: start -> rock-rich area -> gentle slope (5-8.5 deg) -> smooth featureless plain -> start (it ends where it
began, so the drift over a lap can be measured), A* on a 0.2 m traversability grid: footprint slope <= MAX_SLOPE,
>= CLEAR m from rocks with >= 5 cm exposed.
"""
import glob
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

from lunar_world_gen.grid import Grid

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
HOME = Path.home()
HEIGHTS = HOME / "togo_assets/lunar/build/apollo16_core/heights"
CATALOG = HOME / "togo_assets/lunar/worlds/apollo16_core/rocks_catalog.npz"
FLAT = (-2.0, -4.5)
FLAT_Z = -0.4847                      # world_info.yaml: base_link spawn height at the flat site
LIM = 44.0                            # stay inside the core (fade/band start at 47 m)
RES = 0.1                             # height sampling step
PRES = 0.2                            # planning grid step
MAX_SLOPE = 10.5                      # deg, footprint plane fit (smooth 10 deg inclines: 95 % climb efficiency)
CLEAR = 0.50                          # m from the centreline to a rock edge (rover half width 0.38 + margin)
TARGET_LEN = (170.0, 260.0)           # m, about 8-11 min at 0.35-0.45 m/s with turns

# ------------------------------------------------------------------ heights at 0.1 m
xs = np.arange(-LIM - 2, LIM + 2 + 1e-9, RES)
XX, YY = np.meshgrid(xs, xs)
Z = np.full(XX.shape, np.nan)
for js in sorted(glob.glob(str(HEIGHTS / "tile_*.json"))):
    g = Grid.load(js[:-5], mmap=True)
    m = g.contains(XX, YY) & np.isnan(Z)
    if m.any():
        Z[m] = g.sample(XX[m], YY[m], order=1)
assert np.isfinite(Z).all(), "heights do not cover the planning window"


def ground(x, y):
    j = np.clip(np.round((np.asarray(x) - xs[0]) / RES).astype(int), 0, len(xs) - 1)
    i = np.clip(np.round((np.asarray(y) - xs[0]) / RES).astype(int), 0, len(xs) - 1)
    return Z[i, j]


# footprint slope: least-squares plane over a 0.5 m disc (rover body ~1.0 x 0.8 m)
rk = 5
ky, kx = np.mgrid[-rk:rk + 1, -rk:rk + 1]
disc = (kx ** 2 + ky ** 2) <= rk ** 2
kxm = np.where(disc, kx * RES, 0.0)
kym = np.where(disc, ky * RES, 0.0)
gx = ndimage.correlate(Z, kxm, mode="nearest") / (kxm ** 2).sum()
gy = ndimage.correlate(Z, kym, mode="nearest") / (kym ** 2).sum()
slope = np.degrees(np.arctan(np.hypot(gx, gy)))
# roughness: residual RMS around a 0.3 m-sigma smooth surface
rough = np.sqrt(ndimage.uniform_filter((Z - ndimage.gaussian_filter(Z, 3.0)) ** 2, 9))

# ------------------------------------------------------------------ rocks
cat = np.load(CATALOG)
rx, ry, rD, reh = cat["x"], cat["y"], cat["D"], cat["exposed_h"]
inwin = (np.abs(rx) < LIM + 2) & (np.abs(ry) < LIM + 2)
obst = inwin & (reh >= 0.07)                  # the rock test drove over an 8.7 cm rock (4.3 cm exposed)
print(f"rocks in window: {inwin.sum()}, obstacles (exposed >= 7 cm): {obst.sum()}")

# ------------------------------------------------------------------ planning grid (0.2 m)
f = int(round(PRES / RES))
n = len(xs) // f
px = xs[:n * f:f] + (f - 1) * RES / 2
sl = slope[:n * f, :n * f].reshape(n, f, n, f).max(axis=(1, 3))            # conservative: max inside the cell
ro = rough[:n * f, :n * f].reshape(n, f, n, f).mean(axis=(1, 3))
PX, PY = np.meshgrid(px, px)
blocked = (sl > MAX_SLOPE) | (np.abs(PX) > LIM) | (np.abs(PY) > LIM)
# rasterise rock clearance discs
for x, y, D in zip(rx[obst], ry[obst], rD[obst]):
    r = D / 2 + CLEAR
    i0, i1 = np.searchsorted(px, [y - r, y + r])
    j0, j1 = np.searchsorted(px, [x - r, x + r])
    if i1 <= i0 or j1 <= j0:
        continue
    sub = (PX[i0:i1, j0:j1] - x) ** 2 + (PY[i0:i1, j0:j1] - y) ** 2 <= r * r
    blocked[i0:i1, j0:j1] |= sub
cost = 1.0 + 2.0 * (sl / MAX_SLOPE) ** 2 + 40.0 * ro
print(f"planning grid {n}x{n} @ {PRES} m, blocked {blocked.mean() * 100:.1f} %")


def cell(x, y):
    return int(round((y - px[0]) / PRES)), int(round((x - px[0]) / PRES))


def xy(c):
    return float(px[c[1]]), float(px[c[0]])


# ------------------------------------------------------------------ start spot: level and rock-free, near 'flat'
small = inwin & (reh >= 0.03)
tree_pts = np.c_[rx[small], ry[small]]
best = None
for x in np.arange(FLAT[0] - 20, FLAT[0] + 20.01, 0.5):
    for y in np.arange(FLAT[1] - 20, FLAT[1] + 20.01, 0.5):
        i, j = int(round((y - xs[0]) / RES)), int(round((x - xs[0]) / RES))
        s_here = slope[i, j]                                               # plane over the 1 m footprint
        if slope[i - 3:i + 4, j - 3:j + 4].max() > 2.5:                    # and nothing steeper within 0.3 m
            continue
        if s_here > 1.5:
            continue
        d = np.hypot(tree_pts[:, 0] - x, tree_pts[:, 1] - y).min()
        if d < 1.0:
            continue
        c = cell(x, y)
        if blocked[c]:
            continue
        score = s_here + 0.05 * math.dist((x, y), FLAT)
        if best is None or score < best[0]:
            best = (score, x, y, s_here, d)
assert best, "no level start spot found"
_, SX, SY, s_start, d_rock = best
print(f"start ({SX:.2f}, {SY:.2f}): footprint slope <= {s_start:.2f} deg, nearest rock >= 3 cm exposed {d_rock:.2f} m away")

# reachable area from the start (flood fill on the free grid)
lab, _ = ndimage.label(~blocked)
reach = lab == lab[cell(SX, SY)]
print(f"reachable from start: {reach.sum() * PRES * PRES:.0f} m2")

# ------------------------------------------------------------------ waypoint candidates
dist = np.hypot(PX - SX, PY - SY)
ang = np.arctan2(PY - SY, PX - SX)
H2, _, _ = np.histogram2d(ry[inwin & (rD >= 0.05)], rx[inwin & (rD >= 0.05)], bins=[np.r_[px - PRES / 2, px[-1] + PRES / 2]] * 2)
dens = ndimage.uniform_filter(H2, 41) * 41 * 41                          # rocks >= 5 cm within ~4 m box
mslope = ndimage.uniform_filter(sl, 31)                                  # mean slope over ~6 m
relief = ndimage.maximum_filter(ndimage.zoom(Z, 1 / f, order=1)[:n, :n], 25) - \
    ndimage.minimum_filter(ndimage.zoom(Z, 1 / f, order=1)[:n, :n], 25)
smooth = ndimage.uniform_filter(ro, 21) + 0.002 * dens


def pick(mask, score, label):
    s = np.where(mask & reach, score, -np.inf)
    k = np.unravel_index(np.argmax(s), s.shape)
    assert np.isfinite(s[k]), f"no candidate for {label}"
    return k


def sector(a0):
    d = np.abs(np.arctan2(np.sin(ang - a0), np.cos(ang - a0)))
    return d < math.radians(60)


def top_k(mask, score, k=15, sep=4.0):
    """Best k cells of `score` inside `mask` and the reachable area, at least `sep` m apart."""
    s = np.where(mask & reach, score, -np.inf)
    order = np.argsort(s, axis=None)[::-1]
    out = []
    for idx in order:
        c = np.unravel_index(idx, s.shape)
        if not np.isfinite(s[c]):
            break
        if all(math.dist(xy(c), xy(o)) >= sep for o in out):
            out.append(c)
            if len(out) == k:
                break
    return out


ring = (dist > 8) & (dist < 28)
c_rocks = top_k(ring, dens)
c_slope = top_k(ring & (mslope > 4) & (mslope < 9), relief)
c_plain = top_k(ring, -smooth)
print(f"candidates: rocks {len(c_rocks)}, slope {len(c_slope)}, plain {len(c_plain)}")
S0 = cell(SX, SY)
best_route = None
for w1 in c_rocks:
    for w2 in c_slope:
        for w3 in c_plain:
            a = sorted(math.atan2(xy(w)[1] - SY, xy(w)[0] - SX) for w in (w1, w2, w3))
            gaps = [a[1] - a[0], a[2] - a[1], 2 * math.pi - (a[2] - a[0])]
            if min(gaps) < math.radians(70):                                 # spread around the start: a real loop
                continue
            est = sum(math.dist(xy(p), xy(q)) for p, q in [(S0, w1), (w1, w2), (w2, w3), (w3, S0)])
            if not (TARGET_LEN[0] / 1.35 <= est <= TARGET_LEN[1] / 1.1):
                continue
            key = (dens[w1] + 30 * relief[w2]) / (1 + smooth[w3] * 1000)
            if best_route is None or key > best_route[0]:
                best_route = (key, w1, w2, w3, est)
assert best_route, "no loop fits the target length"
_, W1, W2, W3, est = best_route
print(f"waypoints: rocks {xy(W1)} (dens {dens[W1]:.0f}), slope {xy(W2)} (mean {mslope[W2]:.1f} deg, relief {relief[W2]:.2f} m), "
      f"plain {xy(W3)}; straight-line loop {est:.0f} m")


# ------------------------------------------------------------------ A*
def astar(a, b):
    nb = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    g = {a: 0.0}
    prev = {}
    h = lambda c: math.hypot(c[0] - b[0], c[1] - b[1])
    pq = [(h(a), a)]
    while pq:
        _, c = heapq.heappop(pq)
        if c == b:
            break
        for di, dj, w in nb:
            q = (c[0] + di, c[1] + dj)
            if not (0 <= q[0] < n and 0 <= q[1] < n) or blocked[q]:
                continue
            ng = g[c] + w * 0.5 * (cost[c] + cost[q])
            if ng < g.get(q, 1e18):
                g[q] = ng
                prev[q] = c
                heapq.heappush(pq, (ng + h(q), q))
    assert b in prev or a == b, "A* found no path"
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1]


def los(a, b):
    """True if the straight segment a-b stays on free cells (0.1 m steps)."""
    (xa, ya), (xb, yb) = xy(a), xy(b)
    k = max(2, int(math.dist((xa, ya), (xb, yb)) / 0.1))
    for t in np.linspace(0, 1, k):
        if blocked[cell(xa + (xb - xa) * t, ya + (yb - ya) * t)]:
            return False
    return True


def prune(path):
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not los(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


legs = []
S = cell(SX, SY)
for a, b in [(S, W1), (W1, W2), (W2, W3), (W3, S)]:
    legs.append(prune(astar(a, b)))
pts = [xy(legs[0][0])]
for leg in legs:
    for c in leg[1:]:
        pts.append(xy(c))
# resample at 0.2 m
dense = [pts[0]]
for p, q in zip(pts[:-1], pts[1:]):
    k = max(1, int(math.dist(p, q) / 0.2))
    for t in range(1, k + 1):
        dense.append((p[0] + (q[0] - p[0]) * t / k, p[1] + (q[1] - p[1]) * t / k))
dense = np.array(dense)
L = float(np.sum(np.hypot(*np.diff(dense, axis=0).T)))

# ------------------------------------------------------------------ verify the final path
ps = slope[np.clip(np.round((dense[:, 1] - xs[0]) / RES).astype(int), 0, len(xs) - 1),
           np.clip(np.round((dense[:, 0] - xs[0]) / RES).astype(int), 0, len(xs) - 1)]
dmin = np.inf
for x, y, D in zip(rx[obst], ry[obst], rD[obst]):
    dmin = min(dmin, float(np.min(np.hypot(dense[:, 0] - x, dense[:, 1] - y)) - D / 2))
zs = ground(dense[:, 0], dense[:, 1])
yaw0 = math.atan2(dense[min(10, len(dense) - 1), 1] - SY, dense[min(10, len(dense) - 1), 0] - SX)
start_z = float(ground(SX, SY) - ground(*FLAT) + FLAT_Z)
info = {
    "start": {"x": round(SX, 3), "y": round(SY, 3), "z": round(start_z, 4), "yaw": round(yaw0, 4),
              "footprint_slope_deg_max": round(float(s_start), 2), "nearest_rock_3cm_m": round(float(d_rock), 2)},
    "waypoints": {"rocks": xy(W1), "slope": xy(W2), "plain": xy(W3)},
    "length_m": round(L, 1),
    "path_max_footprint_slope_deg": round(float(ps.max()), 2),
    "path_p95_footprint_slope_deg": round(float(np.percentile(ps, 95)), 2),
    "path_min_clearance_to_rock_7cm_m": round(dmin, 3),
    "path_elevation_range_m": round(float(zs.max() - zs.min()), 2),
    "params": {"max_slope_deg": MAX_SLOPE, "clearance_m": CLEAR, "grid_m": PRES},
    "path": [[round(float(x), 3), round(float(y), 3)] for x, y in dense],
}
(OUT / "route.json").write_text(json.dumps(info))
print(json.dumps({k: v for k, v in info.items() if k != "path"}, indent=1))

# ------------------------------------------------------------------ figure
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
pad = 6
x0, x1 = dense[:, 0].min() - pad, dense[:, 0].max() + pad
y0, y1 = dense[:, 1].min() - pad, dense[:, 1].max() + pad
i0, i1 = np.searchsorted(xs, [y0, y1]); j0, j1 = np.searchsorted(xs, [x0, x1])
fig, ax = plt.subplots(figsize=(9, 9))
ls = matplotlib.colors.LightSource(azdeg=300, altdeg=30)
ax.imshow(ls.hillshade(Z[i0:i1, j0:j1], vert_exag=3, dx=RES, dy=RES), cmap="gray", origin="lower",
          extent=(xs[j0], xs[j1 - 1], xs[i0], xs[i1 - 1]))
im = ax.imshow(np.where(slope[i0:i1, j0:j1] > 6, slope[i0:i1, j0:j1], np.nan), cmap="autumn_r", alpha=0.45, origin="lower",
               extent=(xs[j0], xs[j1 - 1], xs[i0], xs[i1 - 1]), vmin=6, vmax=20)
m = obst & (rx > x0) & (rx < x1) & (ry > y0) & (ry < y1)
ax.scatter(rx[m], ry[m], s=np.clip(rD[m] * 60, 2, 60), c="cyan", edgecolors="k", linewidths=0.3, label="rocks, >= 7 cm exposed")
ax.plot(dense[:, 0], dense[:, 1], "b-", lw=1.6, label=f"route {L:.0f} m")
ax.plot(SX, SY, "g^", ms=12, label="start (15 s still)")
for name, w, c in (("rocks", W1, "m"), ("slope", W2, "r"), ("plain", W3, "y")):
    ax.plot(*xy(w), "*", color=c, ms=15, markeredgecolor="k", label=f"waypoint: {name}")
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal")
ax.set_xlabel("world x (m)"); ax.set_ylabel("world y (m)")
ax.set_title("SLAM benchmark loop on apollo16_core (orange: footprint slope > 6 deg)")
ax.legend(loc="upper left", fontsize=8)
fig.colorbar(im, ax=ax, fraction=0.035, label="footprint slope (deg)")
fig.savefig(OUT / "route.png", dpi=110, bbox_inches="tight")
print("wrote", OUT / "route.json", OUT / "route.png")
