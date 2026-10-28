"""Analysis of static gpu_lidar captures (docs/DESIGN.md section 13, "Gazebo, world only").

Captures are written by ``tools/lidar_capture.py`` (one ``.npz`` per sensor pose):

=================  =============================  ==============================================
key                shape / type                   meaning
=================  =============================  ==============================================
``points``         (H, W, 3) float32              returns in the WORLD frame, NaN = no return
``points_sensor``  (H, W, 3) float32              raw sensor-frame points (inf = no return)
``ranges``         (H, W) float32                 range [m], inf = no return
``azimuth``        (W,) float64                   nominal beam azimuth [rad] (column order)
``elevation``      (H,) float64                   nominal beam elevation [rad] (row order)
``pose``           (6,) float64                   sensor x, y, z, roll, pitch, yaw (world)
``request``        (4,) float64                   requested x, y, height above ground, yaw
``ground_z``       ()                             ground height used to place the sensor
``meta_json``      str                            JSON: sdf, topic, lidar preset, stamps, ...
=================  =============================  ==============================================

Checks (``run_lidar_checks``):

* ``edge``: horizontal (Chebyshev) distance of every return from the core centre; returns are
  counted per zone; pass = no return beyond the ring from sensors inside the core.
* ``range_histogram``: returns per range bin (and per scan), return fraction.
* ``ground_residuals``: z - H(x, y) per return against the height grid(s) (or the visual
  meshes). Points more than ``thr(r) = max(min_thr, k * sigma_z(r)) + tin_error`` below
  the surface are "below ground"; points above it and outside every rock footprint are
  "floating". ``sigma_z(r) = 0.176 * beam_spacing * r`` is the range-quantisation height
  noise the gazebo agent measured (5 mm at 2-4 m, 24 mm at 8-16 m for 0.01 rad beams).
* ``missing_returns`` (the real leak detector: a ray that slips through a crack in a
  single-layer ground hits nothing): rays without a return are marched through the height
  field; a ray that clearly passes below the surface (by ``margin``) within range is a leak
  candidate.
* ``rock_detection``: per rock size class and range bin, the fraction of visible rocks with at
  least one return inside the rock's footprint cylinder above 0.3 x its exposed height, mean
  returns per rock, and an ideal ray-casting expectation (projected area / beam solid angle);
  compared with the gazebo small-object results (``GAZEBO_SMALL_OBJECTS``).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np

HeightFn = Callable[[np.ndarray, np.ndarray], np.ndarray]

RANGE_BINS_M = (0.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 150.0)
ROCK_D_BINS_M = (0.02, 0.03, 0.05, 0.10, 0.20, 0.50, 1.0, 3.0)
SIGMA_Z_COEF = 0.176  # height noise std per (metre of range x radian of beam spacing), gazebo t5f

# gazebo_capabilities.md test 5c (cubes, real sensor sampling, sensor 0.7 m high):
# mean returns per cube (ideal ray casting) and detection rate
GAZEBO_SMALL_OBJECTS = [
    {"size_m": 0.05, "range_m": 2.0, "mean_returns": 10.0, "ideal": 11.3, "p_detect": 1.0},
    {"size_m": 0.05, "range_m": 4.0, "mean_returns": 2.3, "ideal": 4.0, "p_detect": 1.0},
    {"size_m": 0.05, "range_m": 8.0, "mean_returns": 0.67, "ideal": 0.83, "p_detect": 4 / 6},
    {"size_m": 0.02, "range_m": 2.0, "mean_returns": 1.3, "ideal": 1.0, "p_detect": 5 / 6},
    {"size_m": 0.02, "range_m": 4.0, "mean_returns": 0.17, "ideal": 0.5, "p_detect": 1 / 6},
    {"size_m": 0.02, "range_m": 8.0, "mean_returns": 0.33, "ideal": 0.33, "p_detect": 2 / 6},
]


# ----------------------------------------------------------------------------------------------
# captures

@dataclass
class Capture:
    path: str
    points: np.ndarray        # (H, W, 3) float64 world, NaN = no return
    ranges: np.ndarray        # (H, W) float64, inf = no return
    azimuth: np.ndarray       # (W,)
    elevation: np.ndarray     # (H,)
    pose: np.ndarray          # (6,)
    meta: dict = field(default_factory=dict)

    @property
    def valid(self) -> np.ndarray:
        return np.isfinite(self.ranges) & np.isfinite(self.points).all(-1)

    @property
    def beam_spacing(self) -> float:
        dh = abs(self.azimuth[1] - self.azimuth[0]) if len(self.azimuth) > 1 else 0.01
        dv = abs(self.elevation[1] - self.elevation[0]) if len(self.elevation) > 1 else 0.01
        return float(0.5 * (dh + dv))

    def rotation(self) -> np.ndarray:
        return rpy_matrix(*self.pose[3:6])

    def ray_dirs_world(self) -> np.ndarray:
        """(H, W, 3) unit beam directions in the world frame (nominal angles)."""
        el = self.elevation[:, None]
        az = self.azimuth[None, :]
        d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el) * np.ones_like(az)], -1)
        return d @ self.rotation().T


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def load_capture(path: str | Path) -> Capture:
    with np.load(path, allow_pickle=False) as d:
        meta = json.loads(str(d["meta_json"])) if "meta_json" in d.files else {}
        pts = np.asarray(d["points"], dtype=np.float64)
        rng_ = np.asarray(d["ranges"], dtype=np.float64)
        return Capture(str(path), pts, rng_, np.asarray(d["azimuth"], np.float64),
                       np.asarray(d["elevation"], np.float64), np.asarray(d["pose"], np.float64), meta)


def load_captures(paths: Iterable[str | Path]) -> list[Capture]:
    out = []
    for p in paths:
        p = Path(p)
        files = sorted(p.glob("*.npz")) if p.is_dir() else [p]
        out.extend(load_capture(f) for f in files)
    return out


# ----------------------------------------------------------------------------------------------
# height sources

class GridHeights:
    """Evaluate one or more grids in priority order (first grid that contains the point wins).

    ``tin_error_m`` is the visual TIN tolerance of the zone the grid stands for (the mesh can
    deviate from its grid by up to this much); it widens the residual thresholds.
    """

    def __init__(self, grids: Sequence, tin_error_m: Sequence[float] | None = None):
        self.grids = list(grids)
        self.tin_error = list(tin_error_m) if tin_error_m is not None else [0.0] * len(self.grids)
        zs = [(float(np.nanmin(g.z[:: max(1, g.ny // 512), :: max(1, g.nx // 512)])),
               float(np.nanmax(g.z[:: max(1, g.ny // 512), :: max(1, g.nx // 512)]))) for g in self.grids]
        self.zmin = min(z[0] for z in zs) - 0.5
        self.zmax = max(z[1] for z in zs) + 0.5

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return self.height_and_tol(x, y)[0]

    def height_and_tol(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        out = np.full(x.shape, np.nan)
        tol = np.zeros(x.shape)
        todo = np.ones(x.shape, dtype=bool)
        for g, te in zip(self.grids, self.tin_error):
            m = todo & g.contains(x, y)
            if m.any():
                out[m] = g.sample(x[m], y[m], order=1)
                tol[m] = te
                todo &= ~m
        return out, tol


class FnHeights:
    """Wrap a plain callable h(x, y) (e.g. ``MeshHeightField.height``)."""

    def __init__(self, fn: HeightFn, zmin: float = -1e3, zmax: float = 1e3, tin_error_m: float = 0.0):
        self.fn, self.zmin, self.zmax, self.te = fn, zmin, zmax, tin_error_m

    def __call__(self, x, y):
        return self.fn(np.asarray(x, np.float64), np.asarray(y, np.float64))

    def height_and_tol(self, x, y):
        h = self(x, y)
        return h, np.full(np.shape(h), self.te)


def rasterize_meshes(paths: Sequence[str | Path], spacing: float = 0.02, rect: tuple | None = None):
    """Height grid sampled from 2.5-D GLB meshes (highest surface per node; NaN where none).

    Every file is read once; nodes inside its accessor bounds are interpolated on its TIN.
    ``rect`` = (x0, x1, y0, y1) limits the raster (default: union of the mesh bounds).
    """
    from ..grid import Grid
    from .static_checks import TinLocator, read_glb
    heads = []
    for p in paths:
        try:
            h = read_glb(p, header_only=True).info.get("bounds_accessor")
        except Exception:  # noqa: BLE001
            continue
        if h:
            heads.append((str(p), np.asarray(h[0]), np.asarray(h[1])))
    if not heads:
        raise ValueError("no readable mesh")
    if rect is None:
        rect = (min(h[1][0] for h in heads), max(h[2][0] for h in heads),
                min(h[1][1] for h in heads), max(h[2][1] for h in heads))
    x0 = math.floor(rect[0] / spacing) * spacing
    y0 = math.floor(rect[2] / spacing) * spacing
    nx = int(math.floor((rect[1] - x0) / spacing)) + 1
    ny = int(math.floor((rect[3] - y0) / spacing)) + 1
    z = np.full((ny, nx), np.nan, dtype=np.float32)
    for path, lo, hi in heads:
        i0 = max(0, int(math.ceil((lo[0] - x0) / spacing - 1e-9)))
        i1 = min(nx - 1, int(math.floor((hi[0] - x0) / spacing + 1e-9)))
        j0 = max(0, int(math.ceil((lo[1] - y0) / spacing - 1e-9)))
        j1 = min(ny - 1, int(math.floor((hi[1] - y0) / spacing + 1e-9)))
        if i1 < i0 or j1 < j0:
            continue
        g = read_glb(path)
        loc = TinLocator(g.positions[:, :2], g.triangles)
        X, Y = np.meshgrid(x0 + np.arange(i0, i1 + 1) * spacing, y0 + np.arange(j0, j1 + 1) * spacing)
        zz = loc.interpolate(g.positions[:, 2], X.ravel(), Y.ravel()).reshape(X.shape)
        cur = z[j0:j1 + 1, i0:i1 + 1]
        z[j0:j1 + 1, i0:i1 + 1] = np.where(np.isnan(cur), zz, np.fmax(cur, zz))
    return Grid(z, x0, y0, spacing, {"rasterized_from": len(heads)})


def final_core_tiles(build_dir: str | Path) -> dict[str, str]:
    """Per-tile final core heights of an integration build, ``build_dir/heights/tile_ii_jj``
    (H0 + fillets, 1 cm, tile + 0.5 m margin): the grids the visual ground TINs were made from.
    Returns {tile name: grid prefix}; {} when the build has none."""
    d = Path(build_dir) / "heights"
    if not d.is_dir():
        return {}
    return {js.stem: str(js.with_suffix("")) for js in sorted(d.glob("tile_*_*.json"))
            if js.with_suffix(".npy").exists()}


def build_height_sources(cfg, found: dict | None = None) -> tuple[list[tuple[str, list[str]]], dict]:
    """Height sources of a generator build, highest priority first, as [(name, [grid prefixes])].

    Core: a single core H grid if the build has one, else the final per-tile heights
    (``heights/``, what the visual TIN approximates), else the terrain stage's H0 tiles
    (``terrain/core_tiles``, no fillets). Then the ring H grid (25 cm, covers the whole ring
    square). ``found`` is ``static_checks.locate_inputs(cfg)`` if already computed.
    """
    if found is None:
        from .static_checks import locate_inputs
        found = locate_inputs(cfg)
    out: list[tuple[str, list[str]]] = []
    if found.get("core_grid"):
        out.append(("core_grid", [found["core_grid"]]))
    else:
        tiles = final_core_tiles(found.get("build_dir", ""))
        if tiles:
            out.append(("core_final_tiles", list(tiles.values())))
        elif found.get("core_tile_grids"):
            out.append(("core_h0_tiles", list(found["core_tile_grids"].values())))
    if found.get("ring_grid"):
        out.append(("ring_grid", [found["ring_grid"]]))
    return out, found


def point_height_fn(sources: list[tuple[str, list[str]]]):
    """h(x, y) -> float for a few points (sensor placement): the first source grid containing the
    point, bilinear; NaN outside all of them. Grids are memory-mapped, nothing is assembled."""
    from ..grid import Grid
    grids = [Grid.load(p, mmap=True) for _, ps in sources for p in ps]

    def h(x: float, y: float) -> float:
        xa, ya = np.array([float(x)]), np.array([float(y)])
        for g in grids:
            if bool(g.contains(xa, ya)[0]):
                return float(g.sample(xa, ya, order=1)[0])
        return float("nan")
    return h


def sigma_z(r: np.ndarray, beam_spacing: float) -> np.ndarray:
    """Range-quantisation height noise std of gpu_lidar returns on near-horizontal ground."""
    return SIGMA_Z_COEF * beam_spacing * np.asarray(r, dtype=np.float64)


# ----------------------------------------------------------------------------------------------
# checks

def edge_check(caps: Sequence[Capture], core_half: float = 50.0, ring_half: float = 250.0,
               far_half: float = 1100.0, horizon_half: float = 10000.0, centre=(0.0, 0.0), tol: float = 0.5) -> dict:
    zones = {"core": 0, "ring": 0, "far": 0, "horizon": 0, "beyond": 0}
    per = []
    worst = 0.0
    beyond_ring = 0
    for c in caps:
        v = c.valid
        P = c.points[v]
        if len(P) == 0:
            per.append({"capture": Path(c.path).name, "n_returns": 0})
            continue
        dx, dy = P[:, 0] - centre[0], P[:, 1] - centre[1]
        dcheb = np.maximum(np.abs(dx), np.abs(dy))
        deuc = np.hypot(dx, dy)
        z = np.digitize(dcheb, [core_half + tol, ring_half + tol, far_half + tol, horizon_half + tol])
        for k, name in enumerate(zones):
            zones[name] += int((z == k).sum())
        sensor_in_core = max(abs(c.pose[0] - centre[0]), abs(c.pose[1] - centre[1])) <= core_half
        nb = int((dcheb > ring_half + tol).sum())
        if sensor_in_core:
            beyond_ring += nb
        worst = max(worst, float(dcheb.max()))
        per.append({"capture": Path(c.path).name, "sensor_in_core": bool(sensor_in_core), "n_returns": int(len(P)),
                    "max_cheb_from_centre_m": float(dcheb.max()), "max_dist_from_centre_m": float(deuc.max()),
                    "max_range_m": float(np.nanmax(c.ranges[v])), "beyond_ring": nb})
    ok = beyond_ring == 0
    return {"status": "pass" if ok else "fail", "pass": ok, "returns_per_zone": zones,
            "beyond_ring_from_core_sensors": beyond_ring, "max_cheb_from_centre_m": worst,
            "ring_half_m": ring_half, "per_capture": per}


def range_histogram(caps: Sequence[Capture], edges: Sequence[float] = RANGE_BINS_M) -> dict:
    edges = np.asarray(edges, dtype=np.float64)
    counts = np.zeros(len(edges) - 1, dtype=np.int64)
    n_rays = 0
    n_ret = 0
    for c in caps:
        r = c.ranges[np.isfinite(c.ranges)]
        counts += np.histogram(r, bins=edges)[0]
        n_rays += c.ranges.size
        n_ret += r.size
    n = max(len(caps), 1)
    return {"edges_m": edges.tolist(), "counts": counts.tolist(), "per_scan": (counts / n).round(2).tolist(),
            "n_scans": len(caps), "n_rays": n_rays, "n_returns": n_ret,
            "return_fraction": n_ret / n_rays if n_rays else None}


def _rock_footprint_mask(P: np.ndarray, rocks: dict | None, pad: float = 0.02) -> np.ndarray:
    """True for points inside any rock's horizontal footprint (radius D/2 + pad)."""
    if rocks is None or len(P) == 0 or len(rocks["D"]) == 0:
        return np.zeros(len(P), dtype=bool)
    from scipy.spatial import cKDTree
    tree = cKDTree(np.stack([rocks["x"], rocks["y"]], 1))
    rmax = float(rocks["D"].max()) / 2 + pad
    hits = tree.query_ball_point(P[:, :2], r=rmax)
    out = np.zeros(len(P), dtype=bool)
    D, rx, ry = rocks["D"], rocks["x"], rocks["y"]
    for i, h in enumerate(hits):
        if not h:
            continue
        h = np.asarray(h)
        d = np.hypot(rx[h] - P[i, 0], ry[h] - P[i, 1])
        out[i] = bool((d <= D[h] / 2 + pad).any())
    return out


def ground_residuals(caps: Sequence[Capture], heights, rocks: dict | None = None, *, k_sigma: float = 5.0,
                     min_thr: float = 0.03, edges: Sequence[float] = RANGE_BINS_M,
                     max_frac_below: float = 1e-4, max_frac_floating: float = 1e-3) -> dict:
    """Residual z - H(x, y) of every return over the height source (see module docstring)."""
    edges = np.asarray(edges, dtype=np.float64)
    nb = len(edges) - 1
    acc = {"n": np.zeros(nb, np.int64), "below": np.zeros(nb, np.int64), "above": np.zeros(nb, np.int64),
           "floating": np.zeros(nb, np.int64)}
    res_all, r_all = [], []
    examples_below, examples_float = [], []
    n_outside = 0
    for c in caps:
        v = c.valid
        P = c.points[v]
        r = c.ranges[v]
        if len(P) == 0:
            continue
        h, tin = heights.height_and_tol(P[:, 0], P[:, 1])
        ok = np.isfinite(h)
        n_outside += int((~ok).sum())
        P, r, h, tin = P[ok], r[ok], h[ok], tin[ok]
        res = P[:, 2] - h
        thr = np.maximum(min_thr, k_sigma * sigma_z(r, c.beam_spacing)) + tin
        below = res < -thr
        above = res > thr
        rockm = _rock_footprint_mask(P, rocks) if (rocks is not None and above.any()) else np.zeros(len(P), bool)
        floating = above & ~rockm if rocks is not None else np.zeros(len(P), bool)
        b = np.clip(np.digitize(r, edges) - 1, 0, nb - 1)
        acc["n"] += np.bincount(b, minlength=nb)
        acc["below"] += np.bincount(b[below], minlength=nb)
        acc["above"] += np.bincount(b[above], minlength=nb)
        acc["floating"] += np.bincount(b[floating], minlength=nb)
        res_all.append(res)
        r_all.append(r)
        for i in np.flatnonzero(below)[:3]:
            if len(examples_below) < 10:
                examples_below.append({"capture": Path(c.path).name, "xyz": P[i].round(3).tolist(),
                                       "residual_m": float(res[i]), "range_m": float(r[i])})
        for i in np.flatnonzero(floating)[:3]:
            if len(examples_float) < 10:
                examples_float.append({"capture": Path(c.path).name, "xyz": P[i].round(3).tolist(),
                                       "residual_m": float(res[i]), "range_m": float(r[i])})
    if not res_all:
        return {"status": "skipped", "pass": None, "reason": "no returns over the height source"}
    res = np.concatenate(res_all)
    r = np.concatenate(r_all)
    bins = []
    b = np.clip(np.digitize(r, edges) - 1, 0, nb - 1)
    for k in range(nb):
        m = b == k
        row = {"range_m": [float(edges[k]), float(edges[k + 1])], "n": int(acc["n"][k]),
               "below": int(acc["below"][k]), "above": int(acc["above"][k]), "floating": int(acc["floating"][k])}
        if m.any():
            row.update({"median_m": float(np.median(res[m])), "p1_m": float(np.percentile(res[m], 1)),
                        "p99_m": float(np.percentile(res[m], 99)), "std_m": float(res[m].std())})
        bins.append(row)
    n = int(acc["n"].sum())
    fb = float(acc["below"].sum() / n) if n else 0.0
    ff = float(acc["floating"].sum() / n) if n else 0.0
    ok = fb <= max_frac_below and (rocks is None or ff <= max_frac_floating)
    return {"status": "pass" if ok else "fail", "pass": ok, "n": n, "n_outside_height_source": n_outside,
            "n_below": int(acc["below"].sum()), "n_above": int(acc["above"].sum()),
            "n_floating": int(acc["floating"].sum()) if rocks is not None else None,
            "frac_below": fb, "frac_floating": ff if rocks is not None else None,
            "floating_checked": rocks is not None,
            "thresholds": {"k_sigma": k_sigma, "min_thr_m": min_thr, "sigma_z_coef": SIGMA_Z_COEF,
                           "max_frac_below": max_frac_below, "max_frac_floating": max_frac_floating},
            "median_m": float(np.median(res)), "bins": bins,
            "examples_below": examples_below, "examples_floating": examples_float}


def missing_returns(caps: Sequence[Capture], heights, *, max_range: float | None = None, margin: float = 0.05,
                    k_ang: float = 0.5, step_rel: float = 0.003, step_min: float = 0.01,
                    min_elev_deg: float = -90.0, max_elev_deg: float = 10.0, max_frac: float = 1e-4) -> dict:
    """Rays without a return that clearly pass below the height field within range (leaks).

    "Clearly" = deeper than ``max(margin, k_ang * beam_spacing * t)`` below the surface at ray
    distance t: gpu_lidar ranges come from a depth texture whose angular quantisation is up to
    about half a beam spacing, so a grazing ray that only clips a low bump far away may read the
    background behind it (measured on the perf A_10M world: 22 rays of the -0.29 deg row clipping
    bumps by 5-6 cm at 38-43 m, none near a T-junction). A ray that slips through a crack keeps
    going down and exceeds the margin within a few metres.
    """
    tot_rays = 0
    tot_expected_hits = 0
    leaks = 0
    examples = []
    per = []
    for c in caps:
        rmax = float(max_range or c.meta.get("lidar", {}).get("range_max", 150.0))
        dirs = c.ray_dirs_world()
        sensor = c.pose[:3]
        el = np.broadcast_to(c.elevation[:, None], c.ranges.shape)
        cand = (~np.isfinite(c.ranges)) & (np.degrees(el) >= min_elev_deg) & (np.degrees(el) <= max_elev_deg)
        hit_rays = np.isfinite(c.ranges)
        D = dirs[cand]
        idx = np.argwhere(cand)
        n_leak = 0
        if len(D):
            # march: t grows geometrically (step = max(step_min, step_rel * t))
            ts = [0.2]
            while ts[-1] < rmax:
                ts.append(ts[-1] + max(step_min, step_rel * ts[-1]))
            ts = np.asarray(ts[:-1] + [rmax])
            first_below = np.full(len(D), np.inf)
            alive = np.ones(len(D), dtype=bool)
            for t in ts:
                if not alive.any():
                    break
                ii = np.flatnonzero(alive)
                p = sensor[None, :] + D[ii] * t
                # rays above the whole terrain and going up never come back
                gone = (p[:, 2] > heights.zmax) & (D[ii, 2] >= 0)
                alive[ii[gone]] = False
                below_floor = p[:, 2] < heights.zmin
                h = heights(p[:, 0], p[:, 1])
                hit = np.isfinite(h) & (p[:, 2] < h - max(margin, k_ang * c.beam_spacing * t))
                hit |= below_floor & np.isfinite(h)
                first_below[ii[hit]] = t
                alive[ii[hit]] = False
                alive[ii[~np.isfinite(h)]] = False   # left the height source: cannot judge
            leak = np.isfinite(first_below)
            n_leak = int(leak.sum())
            for k in np.flatnonzero(leak)[:5]:
                if len(examples) < 20:
                    j, i = idx[k]
                    examples.append({"capture": Path(c.path).name, "row": int(j), "col": int(i),
                                     "elev_deg": float(np.degrees(c.elevation[j])),
                                     "azim_deg": float(np.degrees(c.azimuth[i])),
                                     "expected_hit_xyz": (sensor + D[k] * first_below[k]).round(3).tolist()})
        n_hits = int(hit_rays.sum())
        per.append({"capture": Path(c.path).name, "rays": int(c.ranges.size), "returns": n_hits,
                    "no_return_marched": int(cand.sum()), "leak_candidates": n_leak})
        tot_rays += int(c.ranges.size)
        tot_expected_hits += n_hits + n_leak
        leaks += n_leak
    frac = leaks / tot_expected_hits if tot_expected_hits else 0.0
    ok = frac <= max_frac
    return {"status": "pass" if ok else "fail", "pass": ok, "leak_candidates": leaks,
            "expected_ground_hits": tot_expected_hits, "frac": frac, "max_frac": max_frac,
            "margin_m": margin, "k_ang": k_ang, "per_capture": per, "examples": examples,
            "note": "a ray that passes more than margin below the height field without a return; "
                    "single-layer ground means a crack leak produces no return at all"}


def _ideal_returns(D: np.ndarray, h_e: np.ndarray, rho: np.ndarray, dz: np.ndarray, dh: float, dv: float,
                   shape_factor: float = 0.75) -> np.ndarray:
    """Expected beams on an exposed rock by ideal ray casting.

    The exposed rock is a box of width/depth 0.85 D (mean of b/a = 0.7 and 1) and height h_e,
    seen from horizontal distance rho and height difference dz (sensor above the rock centre):
    projected area A = w h_e cos(alpha) + w d sin(alpha), alpha = atan2(dz, rho), rounded by
    ``shape_factor``; beams per area = 1 / (r^2 dh dv).
    """
    w = 0.85 * D
    alpha = np.arctan2(np.maximum(dz, 0.0), rho)
    A = shape_factor * (w * h_e * np.cos(alpha) + w * w * np.sin(alpha))
    r2 = rho ** 2 + dz ** 2
    return A / (r2 * dh * dv)


def rock_detection(caps: Sequence[Capture], rocks: dict, heights=None, *, d_bins: Sequence[float] = ROCK_D_BINS_M,
                   r_bins: Sequence[float] = RANGE_BINS_M, max_range: float = 32.0, min_returns: int = 1,
                   occlusion: bool = True, h_frac: float = 0.3) -> dict:
    """Detection probability of catalog rocks vs size and range (see module docstring)."""
    d_bins = np.asarray(d_bins, np.float64)
    r_bins = np.asarray(r_bins, np.float64)
    nd, nr = len(d_bins) - 1, len(r_bins) - 1
    n = np.zeros((nd, nr), np.int64)
    det = np.zeros((nd, nr), np.int64)
    ret = np.zeros((nd, nr), np.float64)
    ideal = np.zeros((nd, nr), np.float64)
    occl = np.zeros((nd, nr), np.int64)
    rx, ry, rD = (np.asarray(rocks[k], np.float64) for k in ("x", "y", "D"))
    he = np.asarray(rocks.get("exposed_h", 0.5 * rD), np.float64)
    rz = np.asarray(rocks.get("z", np.zeros_like(rx)), np.float64)
    from scipy.spatial import cKDTree
    for c in caps:
        s = c.pose[:3]
        R = c.rotation()
        rel = np.stack([rx - s[0], ry - s[1]], 1)
        rho = np.hypot(rel[:, 0], rel[:, 1])
        sel = np.flatnonzero((rho <= max_range) & (rho > 0.3) & (rD >= d_bins[0]) & (rD < d_bins[-1]))
        if len(sel) == 0:
            continue
        if heights is not None:
            g = heights(rx[sel], ry[sel])
            ground = np.where(np.isfinite(g), g, rz[sel] - 0.5 * he[sel])
        else:
            ground = rz[sel] - 0.5 * he[sel]
        top = ground + he[sel]
        # FOV test on the rock's top centre, in the sensor frame
        v = np.stack([rx[sel] - s[0], ry[sel] - s[1], top - s[2]], 1) @ R  # world -> sensor (R^T v)
        az = np.arctan2(v[:, 1], v[:, 0])
        el = np.arctan2(v[:, 2], np.hypot(v[:, 0], v[:, 1]))
        infov = ((az >= c.azimuth.min()) & (az <= c.azimuth.max()) & (el >= c.elevation.min() - 0.01)
                 & (el <= c.elevation.max()))
        sel, ground, top = sel[infov], ground[infov], top[infov]
        if len(sel) == 0:
            continue
        vis = np.ones(len(sel), dtype=bool)
        if occlusion and heights is not None:
            vis = _line_of_sight(s, rx[sel], ry[sel], top, heights)
        valid = c.valid
        P = c.points[valid]
        rr = c.ranges[valid]
        tree = cKDTree(P[:, :2]) if len(P) else None
        dh = abs(c.azimuth[1] - c.azimuth[0]) if len(c.azimuth) > 1 else 0.01
        dv = abs(c.elevation[1] - c.elevation[0]) if len(c.elevation) > 1 else 0.01
        for k, ri in enumerate(sel):
            rho_k = float(rho[ri])
            di = int(np.searchsorted(d_bins, rD[ri], side="right") - 1)
            bi = int(np.searchsorted(r_bins, rho_k, side="right") - 1)
            if not (0 <= di < nd and 0 <= bi < nr):
                continue
            if not vis[k]:
                occl[di, bi] += 1
                continue
            cnt = 0
            if tree is not None:
                cand = tree.query_ball_point([rx[ri], ry[ri]], r=rD[ri] / 2 * 1.05 + 0.005)
                if cand:
                    cand = np.asarray(cand)
                    zt = P[cand, 2]
                    sz = sigma_z(rr[cand], c.beam_spacing)
                    inside = (zt >= ground[k] + h_frac * he[ri]) & (zt <= top[k] + 0.02 + 2 * sz)
                    cnt = int(inside.sum())
            n[di, bi] += 1
            det[di, bi] += int(cnt >= min_returns)
            ret[di, bi] += cnt
            ideal[di, bi] += float(_ideal_returns(np.array([rD[ri]]), np.array([he[ri]]), np.array([rho_k]),
                                                  np.array([s[2] - (ground[k] + 0.5 * he[ri])]), dh, dv)[0])
    rows = []
    consistent = True
    judged = 0
    for di in range(nd):
        for bi in range(nr):
            if n[di, bi] == 0 and occl[di, bi] == 0:
                continue
            m = n[di, bi]
            row = {"D_m": [float(d_bins[di]), float(d_bins[di + 1])], "range_m": [float(r_bins[bi]), float(r_bins[bi + 1])],
                   "n_visible": int(m), "n_occluded": int(occl[di, bi]),
                   "p_detect": float(det[di, bi] / m) if m else None,
                   "mean_returns": float(ret[di, bi] / m) if m else None,
                   "mean_ideal_returns": float(ideal[di, bi] / m) if m else None}
            if m:
                row["ratio_to_ideal"] = float(ret[di, bi] / ideal[di, bi]) if ideal[di, bi] > 0 else None
            # judged bins: enough rocks and enough expected returns for a meaningful comparison
            if m >= 5 and row["mean_ideal_returns"] is not None and row["mean_ideal_returns"] >= 3.0:
                judged += 1
                ok = (row["ratio_to_ideal"] is not None and 0.4 <= row["ratio_to_ideal"] <= 1.6
                      and (row["mean_ideal_returns"] < 5.0 or row["p_detect"] >= 0.8))
                row["consistent"] = bool(ok)
                consistent &= ok
            rows.append(row)
    status = "pass" if (consistent and judged) else ("skipped" if not judged else "fail")
    return {"status": status, "pass": (consistent if judged else None), "n_judged_bins": judged, "rows": rows,
            "rule": "bins with >= 5 visible rocks and >= 3 ideal returns: returns/ideal in [0.4, 1.6] and, "
                    "where >= 5 ideal returns, p_detect >= 0.8 (gazebo cubes: ratio 0.34-1.3)",
            "gazebo_reference": GAZEBO_SMALL_OBJECTS, "max_range_m": max_range, "min_returns": min_returns,
            "h_frac": h_frac, "occlusion_checked": bool(occlusion and heights is not None)}


def _line_of_sight(s: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray, heights, step: float = 0.05,
                   clearance: float = 0.01) -> np.ndarray:
    """True where the segment sensor -> (x, y, z) stays above the terrain (last 10 % ignored)."""
    vis = np.ones(len(x), dtype=bool)
    for i in range(len(x)):
        L = math.hypot(x[i] - s[0], y[i] - s[1])
        nst = max(2, int(L / step))
        t = np.linspace(0.0, 0.9, nst)
        px = s[0] + t * (x[i] - s[0])
        py = s[1] + t * (y[i] - s[1])
        pz = s[2] + t * (z[i] - s[2])
        h = heights(px, py)
        ok = ~np.isfinite(h) | (pz >= h + clearance)
        vis[i] = bool(ok.all())
    return vis


# ----------------------------------------------------------------------------------------------
# driver

def run_lidar_checks(capture_paths: Sequence[str | Path], *, heights=None, rocks: dict | None = None,
                     zone_half: dict | None = None, out: str | Path | None = None, rock_max_range: float = 32.0,
                     do_missing: bool = True, inputs: dict | None = None) -> dict:
    """Run every LiDAR check on the captures; returns (and optionally writes) the JSON dict.

    ``inputs`` (optional) describes the height sources / catalog used; it is stored as is.
    """
    t0 = time.time()
    caps = load_captures(capture_paths)
    if not caps:
        raise FileNotFoundError(f"no capture npz in {capture_paths}")
    zh = {"core": 50.0, "ring": 250.0, "far": 1100.0, "horizon": 10000.0}
    zh.update(zone_half or {})
    res: dict = {"kind": "lidar_checks", "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                 "inputs": inputs or {},
                 "captures": [{"file": c.path, "pose": c.pose.round(4).tolist(),
                               "request": c.meta.get("request"), "n_returns": int(c.valid.sum()),
                               "shape": list(c.ranges.shape), "beam_spacing_rad": c.beam_spacing,
                               "sdf": c.meta.get("sdf"), "lidar": c.meta.get("lidar")} for c in caps]}
    checks: dict[str, dict] = {}
    checks["edge"] = edge_check(caps, zh["core"], zh["ring"], zh["far"], zh["horizon"])
    checks["range_histogram"] = {"status": "pass", "pass": True, **range_histogram(caps)}
    if heights is not None:
        checks["ground_residuals"] = ground_residuals(caps, heights, rocks)
        if do_missing:
            checks["missing_returns"] = missing_returns(caps, heights)
    else:
        checks["ground_residuals"] = {"status": "skipped", "pass": None, "reason": "no height source"}
    if rocks is not None:
        checks["rock_detection"] = rock_detection(caps, rocks, heights, max_range=rock_max_range)
    else:
        checks["rock_detection"] = {"status": "skipped", "pass": None, "reason": "no rock catalog"}
    # self-consistency: returned points lie on the nominal beams
    devs = []
    for c in caps:
        v = c.valid
        if v.any():
            d = c.points[v] - c.pose[:3]
            d /= np.linalg.norm(d, axis=1, keepdims=True)
            nom = c.ray_dirs_world()[v]
            devs.append(float(np.degrees(np.arccos(np.clip((d * nom).sum(1), -1, 1))).max()))
    res["beam_direction_max_dev_deg"] = max(devs) if devs else None
    res["checks"] = checks
    from .static_checks import summarize_results, write_json
    res["summary"] = summarize_results({k: v for k, v in checks.items()}, time.time() - t0)
    if out:
        write_json(out, res)
        res["written_to"] = str(out)
    return res


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Analyse gpu_lidar captures from tools/lidar_capture.py")
    ap.add_argument("captures", nargs="+", help="capture .npz files or directories")
    ap.add_argument("--config", help="generator config (zone sizes, default grid/catalog discovery)")
    ap.add_argument("--preset")
    ap.add_argument("--grid", action="append", default=[], help="height grid prefix (repeat: core first, then ring)")
    ap.add_argument("--auto-inputs", action="store_true",
                    help="find the core/ring grids and the rock catalog in the build/world dirs (needs --config)")
    ap.add_argument("--catalog", help="rocks_catalog.npz")
    ap.add_argument("--sdf-meshes", help="use the static visual GLB meshes of this SDF as the height source")
    ap.add_argument("--raster", type=float, default=0.02, help="raster spacing for --sdf-meshes [m]")
    ap.add_argument("--tin-error", type=float, nargs="*", default=None, help="per-grid TIN tolerance [m]")
    ap.add_argument("--out", help="output JSON (default: <first capture dir>/lidar_checks.json)")
    ap.add_argument("--no-missing", action="store_true", help="skip the ray-marching leak check")
    a = ap.parse_args(argv)
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from lunar_world_gen.grid import Grid
    heights = None
    rocks = None
    zone_half = None
    cfg = None
    if a.config or a.auto_inputs:
        from lunar_world_gen.config import load_config
        cfg = load_config(a.config, a.preset)
        zone_half = {z: float(cfg.zones[z].size_m) / 2 for z in ("core", "ring", "far", "horizon")}
    grids = [Grid.load(g, mmap=True) for g in a.grid]
    catalog = a.catalog
    tin = a.tin_error
    inputs: dict = {"grids": list(a.grid)} if a.grid else {}
    if a.auto_inputs and cfg is not None and not grids:
        # A build may hold no single core grid, only per-tile grids (the integration build writes
        # 100 tiles of 1101^2 nodes). Then the core is assembled from the FINAL tiles (heights/,
        # H0 + fillets: the source of the visual TIN), so the core is not judged against the 25 cm
        # ring grid (which also covers the core square but lacks all sub-0.5 m detail).
        from lunar_world_gen.validate.static_checks import load_core_grid
        t_in = time.time()
        sources, found = build_height_sources(cfg)
        tol = {"core": float(cfg.mesh.core_visual_max_error_m), "ring": float(cfg.mesh.ring_visual_max_error_m)}
        tin_auto = []
        for name, prefixes in sources:
            if name in ("core_final_tiles", "core_h0_tiles"):
                g, src = load_core_grid({"core_tile_grids": dict(enumerate(prefixes))}, cfg)
            else:
                g = Grid.load(prefixes[0], mmap=True)
            grids.append(g)
            tin_auto.append(tol["ring" if name == "ring_grid" else "core"])
        if tin is None:
            tin = tin_auto
        catalog = catalog or found.get("rock_catalog")
        inputs = {"auto": True, "height_sources": [{"name": n, "n_grids": len(p), "first": p[0]} for n, p in sources],
                  "tin_error_m": tin, "rock_catalog": catalog, "assembled_s": round(time.time() - t_in, 1)}
        print(f"height sources: {[(n, len(p)) for n, p in sources]} (TIN tolerance {tin}) in "
              f"{inputs['assembled_s']} s", flush=True)
    elif a.auto_inputs and cfg is not None:
        from lunar_world_gen.validate.static_checks import locate_inputs
        catalog = catalog or locate_inputs(cfg).get("rock_catalog")
    if grids:
        heights = GridHeights(grids, tin)
    elif a.sdf_meshes:
        t0 = time.time()
        g = rasterize_meshes(sdf_visual_meshes(a.sdf_meshes), a.raster)
        print(f"rasterized {g.meta['rasterized_from']} meshes to {g.nx} x {g.ny} at {a.raster} m "
              f"in {time.time() - t0:.1f} s", flush=True)
        heights = GridHeights([g], tin or [0.0])
    if catalog:
        from lunar_world_gen.validate.static_checks import load_catalog
        rocks = load_catalog(catalog)
        inputs["rock_catalog"] = str(catalog)
        inputs["n_rocks"] = int(len(rocks["D"]))
    first = Path(a.captures[0])
    out = a.out or ((first if first.is_dir() else first.parent) / "lidar_checks.json")
    res = run_lidar_checks(a.captures, heights=heights, rocks=rocks, zone_half=zone_half, out=out,
                           do_missing=not a.no_missing, inputs=inputs)
    print(json.dumps(res["summary"]))
    print(f"wrote {out}")
    return 0 if res["summary"]["pass"] else 1


def sdf_visual_meshes(sdf_path: str | Path) -> list[str]:
    """GLB files used as <visual> meshes by static models of an SDF (resolved paths)."""
    import xml.etree.ElementTree as ET
    sdf_path = Path(sdf_path)
    root = ET.parse(sdf_path).getroot()
    out = []
    for model in root.iter("model"):
        if (model.findtext("static") or "").strip().lower() not in ("true", "1"):
            continue
        for vis in model.iter("visual"):
            uri = vis.findtext("geometry/mesh/uri")
            if not uri or not uri.strip().lower().endswith(".glb"):
                continue
            uri = uri.strip()
            p = Path(uri[7:] if uri.startswith("file://") else uri)
            if not p.is_absolute():
                p = sdf_path.parent / p
            if p.exists():
                out.append(str(p))
    return out


if __name__ == "__main__":
    sys.exit(main())
