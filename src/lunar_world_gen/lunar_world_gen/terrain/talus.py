"""Angle-of-repose limiter of the composite height: the talus term T of H0 (FID-03).

Why
    Pure superposition can stack a synthetic fresh crater on a real DTM depression or slope. In the
    seed-16 world the 11.85 m crater at (-31.9, 26.0) landed in a 1.4 m deep DTM pit and made a 2.9 m
    deep compound pit with walls of 47.6 deg (0.5 m baseline; 24 m^2 above 35 deg) while B alone never
    exceeds 21.9 deg in the core. Loose regolith cannot hold slopes above its angle of repose (about
    35 deg; fresh small-crater walls are 26-31 deg): such walls slump.

What
    T is a correction field on a fixed global lattice grid (default: the 10 cm coarse-fBm lattice over
    the ring square + 2 m), computed once from the part of H that this grid resolves,
        Hs = B + w_ring * (F_coarse + K_big) + w_core * K_small,
    (everything but the fine fBm band, whose 2-50 cm wavelengths the 10 cm grid would only alias),
    by a mass-conserving talus relaxation (explicit nonlinear diffusion, Jacobi updates):
        z_p <- z_p + sum over the 4 edges (p, q) of  D_pq * (z_q - z_p)
        D_pq = c * clip((G_pq - S) / (ramp * G_pq), 0, 1),   S = tan(max_slope_deg - flux_margin_deg)
    where G_pq is the full gradient magnitude at the edge midpoint (along-edge difference plus the
    averaged cross-edge central difference). D_pq is computed from the same values in the same order by
    both nodes of an edge, so whatever leaves one node enters the other (mass is conserved to rounding),
    and only edges steeper than the flux threshold carry material, downhill. The relaxation runs until
    every edge gradient is <= tan(max_slope_deg): over-steep walls slump (upper wall and rim lose height,
    the floor gains it) and everything else is untouched (T = 0 exactly there).

    Work is restricted to windows: connected groups of nodes next to a steep edge, each bounding box
    grown by ``window_margin_m``, overlapping boxes merged. Each window is relaxed independently with
    its outermost node frame held fixed; a window whose change reaches the node ring next to its frame
    is grown and relaxed again. Updates are Jacobi (every node reads the previous iterate), so the
    result does not depend on the thread count or the window order.

    In H0 the field enters as ``w_ring(x, y) * T_s(x, y)``, where T_s is the Catmull-Rom interpolation
    of T (C1, interpolating: T_s = T at the lattice nodes; local 4x4 support: T_s = 0 exactly wherever T
    and its neighbours are 0, so H0 is bit-identical to the unlimited composition away from relaxed
    walls). Every zone samples the same cached field with the same kernel at the same lattice-exact
    coordinates, so shared zone boundary nodes stay bit-identical; F_fine (core only, < 0.5 m) rides on
    top of the relaxed surface (it adds about 1 deg at a 0.5 m baseline). K_small is part of Hs because
    a fresh 0.5-1 m crater on a relaxed wall would otherwise add 5-7 deg there.

Why 32 deg
    Walls that exceed the static angle of repose (about 35 deg) fail and come to rest a few degrees lower;
    fresh small-crater walls in the calibration are 26-31 deg (fresh_A, atan(4 d/D)). Relaxing to 32 deg
    keeps every wall within 1 deg of the steepest fresh wall of the calibration. Measured on the seed-16
    core (0.5 m baseline, all other settings equal; experiments/fix_terrain variant study):
    limit off / 35 / 34 / 33 / 32 / 30 deg -> max 49.3 / 35.6 / 35.7 / 35.5 / 34.0 / 33.4 deg,
    area > 30 deg 72.8 / 68.7 / 46.2 / 20.2 / 3.5 / 0.17 m^2, area > 35 deg 34.7 / 0.004 / 0.003 / 0.002 /
    0 / 0 m^2. 32 deg is the largest limit with max <= 35 deg and a > 30 deg area of the order of the
    calibration's fresh-wall share (about 2 m^2 per hectare); F_fine and sub-10 cm craters add about 1-2 deg
    on top of the relaxed walls.

Config ``terrain.talus`` (all optional): enabled (true), max_slope_deg (32), flux_margin_deg (1),
spacing_m (terrain.roughness.coarse_field_spacing_m), window_margin_m (3), max_iter (40000).
"""
from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import numba as nb
import numpy as np
from scipy import ndimage

from ..config import Config
from ..grid import Grid, from_lattice, to_lattice
from ..layout import Layout

TALUS_VERSION = "2"           # 2: K_small is part of the limiter input
INPUT_PARTS = ("B", "Fc", "Kb", "Ks")
COEF = 0.24          # explicit-diffusion edge coefficient (4 edges -> sum <= 0.96: maximum principle)
RAMP = 0.1           # the edge coefficient reaches COEF when G exceeds S by 10 %
CHECK_EVERY = 25     # iterations between convergence checks
GROW_TRIES = 4
DEFAULTS = {"enabled": True, "max_slope_deg": 32.0, "flux_margin_deg": 1.0, "window_margin_m": 3.0,
            "max_iter": 40000}


def talus_params(cfg: Config) -> dict:
    """Limiter parameters (config ``terrain.talus`` with defaults); also the cache-key input."""
    t = dict(cfg.terrain.get("talus") or {})
    unknown = set(t) - set(DEFAULTS) - {"spacing_m"}
    if unknown:
        raise ValueError(f"unknown terrain.talus keys {sorted(unknown)}")
    p = {k: t.get(k, v) for k, v in DEFAULTS.items()}
    p["enabled"] = bool(p["enabled"])
    for k in ("max_slope_deg", "flux_margin_deg", "window_margin_m"):
        p[k] = float(p[k])
    p["max_iter"] = int(p["max_iter"])
    p["spacing_m"] = float(t.get("spacing_m", cfg.terrain.roughness.coarse_field_spacing_m))
    if not 0.0 < p["max_slope_deg"] < 89.0 or not 0.0 <= p["flux_margin_deg"] < p["max_slope_deg"]:
        raise ValueError("terrain.talus: need 0 < max_slope_deg < 89 and 0 <= flux_margin_deg < max_slope_deg")
    to_lattice(p["spacing_m"])
    half = Layout(cfg).zone_rect("ring").x1 + 2.0
    n = int(round(2 * half / p["spacing_m"])) + 1
    to_lattice(half)
    p.update({"version": TALUS_VERSION, "half_m": half, "n": n})
    return p


# ---------------------------------------------------------------------------------------- numba kernels
@nb.njit(cache=False, inline="always")
def _coef_x(z, j, i, S, inv_h, ramp, c):
    """Edge coefficient between (j, i) and (j, i + 1)."""
    gx = (z[j, i + 1] - z[j, i]) * inv_h
    gy = (z[j + 1, i] + z[j + 1, i + 1] - z[j - 1, i] - z[j - 1, i + 1]) * (0.25 * inv_h)
    G = math.sqrt(gx * gx + gy * gy)
    if G <= S:
        return 0.0
    f = (G - S) / (ramp * G)
    return c if f >= 1.0 else c * f


@nb.njit(cache=False, inline="always")
def _coef_y(z, j, i, S, inv_h, ramp, c):
    """Edge coefficient between (j, i) and (j + 1, i)."""
    gy = (z[j + 1, i] - z[j, i]) * inv_h
    gx = (z[j, i + 1] + z[j + 1, i + 1] - z[j, i - 1] - z[j + 1, i - 1]) * (0.25 * inv_h)
    G = math.sqrt(gx * gx + gy * gy)
    if G <= S:
        return 0.0
    f = (G - S) / (ramp * G)
    return c if f >= 1.0 else c * f


@nb.njit(cache=False, parallel=True)
def _step(z, out, S, inv_h, ramp, c):
    """One Jacobi iteration on the interior nodes that have a full stencil (2-node frame kept)."""
    ny, nx = z.shape
    for j in nb.prange(2, ny - 2):
        for i in range(2, nx - 2):
            zp = z[j, i]
            acc = _coef_x(z, j, i, S, inv_h, ramp, c) * (z[j, i + 1] - zp)
            acc += _coef_x(z, j, i - 1, S, inv_h, ramp, c) * (z[j, i - 1] - zp)
            acc += _coef_y(z, j, i, S, inv_h, ramp, c) * (z[j + 1, i] - zp)
            acc += _coef_y(z, j - 1, i, S, inv_h, ramp, c) * (z[j - 1, i] - zp)
            out[j, i] = zp + acc


@nb.njit(cache=False, parallel=True)
def _max_edge_gradient_rows(z, inv_h):
    """Per row: max gradient magnitude over the x- and y-edges with a full stencil (rows 1..ny-2)."""
    ny, nx = z.shape
    out = np.zeros(ny)
    for j in nb.prange(1, ny - 1):
        m = 0.0
        for i in range(1, nx - 1):
            if i < nx - 2:
                gx = (z[j, i + 1] - z[j, i]) * inv_h
                gy = (z[j + 1, i] + z[j + 1, i + 1] - z[j - 1, i] - z[j - 1, i + 1]) * (0.25 * inv_h)
                g = math.sqrt(gx * gx + gy * gy)
                if g > m:
                    m = g
            if j < ny - 2:
                gy = (z[j + 1, i] - z[j, i]) * inv_h
                gx = (z[j, i + 1] + z[j + 1, i + 1] - z[j, i - 1] - z[j + 1, i - 1]) * (0.25 * inv_h)
                g = math.sqrt(gx * gx + gy * gy)
                if g > m:
                    m = g
        out[j] = m
    return out


@nb.njit(cache=False, parallel=True)
def _steep_nodes(z, inv_h, S, out):
    """out[j, i] = True for both nodes of every edge with gradient magnitude > S."""
    ny, nx = z.shape
    for j in nb.prange(1, ny - 1):
        for i in range(1, nx - 1):
            flag = False
            for di in (-1, 0):
                ii = i + di
                if ii >= 1 and ii + 1 <= nx - 2:
                    gx = (z[j, ii + 1] - z[j, ii]) * inv_h
                    gy = (z[j + 1, ii] + z[j + 1, ii + 1] - z[j - 1, ii] - z[j - 1, ii + 1]) * (0.25 * inv_h)
                    if gx * gx + gy * gy > S * S:
                        flag = True
            for dj in (-1, 0):
                jj = j + dj
                if jj >= 1 and jj + 1 <= ny - 2:
                    gy = (z[jj + 1, i] - z[jj, i]) * inv_h
                    gx = (z[jj, i + 1] + z[jj + 1, i + 1] - z[jj, i - 1] - z[jj + 1, i - 1]) * (0.25 * inv_h)
                    if gx * gx + gy * gy > S * S:
                        flag = True
            out[j, i] = flag


@nb.njit(cache=False, parallel=True)
def _catmull_rom_kernel(T, x0, y0, inv_h, px, py, out):
    ny, nx = T.shape
    for p in nb.prange(px.size):
        fx = (px[p] - x0) * inv_h
        fy = (py[p] - y0) * inv_h
        i = int(math.floor(fx))
        j = int(math.floor(fy))
        if i < 1 or j < 1 or i > nx - 3 or j > ny - 3:
            out[p] = 0.0
            continue
        tx = fx - i
        ty = fy - j
        wx0 = 0.5 * (-tx * tx * tx + 2.0 * tx * tx - tx)
        wx1 = 0.5 * (3.0 * tx * tx * tx - 5.0 * tx * tx + 2.0)
        wx2 = 0.5 * (-3.0 * tx * tx * tx + 4.0 * tx * tx + tx)
        wx3 = 0.5 * (tx * tx * tx - tx * tx)
        wy0 = 0.5 * (-ty * ty * ty + 2.0 * ty * ty - ty)
        wy1 = 0.5 * (3.0 * ty * ty * ty - 5.0 * ty * ty + 2.0)
        wy2 = 0.5 * (-3.0 * ty * ty * ty + 4.0 * ty * ty + ty)
        wy3 = 0.5 * (ty * ty * ty - ty * ty)
        acc = 0.0
        for a in range(4):
            wy = wy0 if a == 0 else (wy1 if a == 1 else (wy2 if a == 2 else wy3))
            r = j - 1 + a
            v = (wx0 * T[r, i - 1] + wx1 * T[r, i] + wx2 * T[r, i + 1] + wx3 * T[r, i + 2])
            acc += wy * v
        out[p] = acc


# ---------------------------------------------------------------------------------------- relaxation
def relax(z: np.ndarray, spacing: float, max_slope_deg: float, flux_margin_deg: float = 1.0,
          max_iter: int = 40000) -> tuple[np.ndarray, dict]:
    """Talus-relax a window (float64, 2-node frame held fixed); returns (relaxed, info)."""
    z = np.ascontiguousarray(z, dtype=np.float64)
    inv_h = 1.0 / spacing
    lim = math.tan(math.radians(max_slope_deg))
    S = math.tan(math.radians(max_slope_deg - flux_margin_deg))
    a = z.copy()
    b = z.copy()
    g0 = float(_max_edge_gradient_rows(a, inv_h).max())
    it = 0
    g = g0
    while g > lim and it < max_iter:
        for _ in range(CHECK_EVERY):
            _step(a, b, S, inv_h, RAMP, COEF)
            a, b = b, a
            it += 1
        g = float(_max_edge_gradient_rows(a, inv_h).max())
    info = {"iterations": it, "max_slope_before_deg": math.degrees(math.atan(g0)),
            "max_slope_after_deg": math.degrees(math.atan(g)), "converged": bool(g <= lim),
            "mass_balance_m3": float((a - z).sum() * spacing * spacing),
            "max_abs_change_m": float(np.abs(a - z).max())}
    return a, info


def _merge_boxes(boxes: list[list[int]]) -> list[list[int]]:
    """Merge overlapping or touching [j0, j1, i0, i1) boxes until they are disjoint (deterministic)."""
    boxes = sorted(boxes)
    changed = True
    while changed:
        changed = False
        out: list[list[int]] = []
        for b in boxes:
            for o in out:
                if b[0] <= o[1] and o[0] <= b[1] and b[2] <= o[3] and o[2] <= b[3]:
                    o[0], o[1], o[2], o[3] = min(o[0], b[0]), max(o[1], b[1]), min(o[2], b[2]), max(o[3], b[3])
                    changed = True
                    break
            else:
                out.append(list(b))
        boxes = sorted(out)
    return boxes


def compute_talus(model, log=print) -> Grid:
    """The talus correction field T of a TerrainModel (not cached; see ``talus_field``)."""
    cfg = model.cfg
    p = talus_params(cfg)
    sp, n, half = p["spacing_m"], p["n"], p["half_m"]
    l0, st = int(to_lattice(-half)), int(to_lattice(sp))
    xs = from_lattice(l0 + st * np.arange(n, dtype=np.int64))
    t0 = time.time()
    meta: dict = {"params": p, "grid": {"x0": -half, "spacing": sp, "n": n}}
    T = np.zeros((n, n), dtype=np.float32)
    if not p["enabled"]:
        meta["windows"] = []
        return Grid(T, -half, -half, sp, meta)
    # 1) the part of H the limiter grid resolves: B + w_ring (F_coarse + K_big) + w_core K_small
    Hs = np.empty((n, n), dtype=np.float64)
    rows = max(1, 2_000_000 // n)
    for j0 in range(0, n, rows):
        j1 = min(n, j0 + rows)
        X, Y = np.meshgrid(xs, xs[j0:j1])
        Hs[j0:j1] = model.evaluate(X, Y, "ring", parts=INPUT_PARTS)
    t_eval = time.time() - t0
    inv_h = 1.0 / sp
    lim = math.tan(math.radians(p["max_slope_deg"]))
    # 2) steep nodes -> windows
    steep = np.zeros((n, n), dtype=np.bool_)
    _steep_nodes(Hs, inv_h, lim, steep)
    lab, nlab = ndimage.label(steep, structure=np.ones((3, 3), bool))
    m = int(round(p["window_margin_m"] / sp))
    boxes = [[max(s[0].start - m, 0), min(s[0].stop + m, n), max(s[1].start - m, 0), min(s[1].stop + m, n)]
             for s in ndimage.find_objects(lab) if s is not None]
    boxes = _merge_boxes(boxes)
    meta["n_steep_nodes"] = int(steep.sum())
    del steep, lab
    results: list = []
    for rnd in range(GROW_TRIES):
        results, grow = [], []
        for b in boxes:
            j0, j1, i0, i1 = b
            sub = Hs[j0:j1, i0:i1]
            relaxed, info = relax(sub, sp, p["max_slope_deg"], p["flux_margin_deg"], p["max_iter"])
            d = relaxed - sub
            ring = np.zeros(d.shape, bool)
            ring[2:-2, 2:-2] = True
            ring[4:-4, 4:-4] = False                   # the two node rings next to the fixed frame
            touched = bool(np.abs(d[ring]).max() > 1e-9) if ring.any() else True
            at_edge = (j0 == 0 or i0 == 0 or j1 == n or i1 == n)
            results.append((b, d, info, touched))
            grow.append(touched and not at_edge)
        if not any(grow) or rnd == GROW_TRIES - 1:
            break
        # grow the windows whose change reached their frame, re-merge (disjoint again), relax all again
        boxes = _merge_boxes([[max(b[0] - m, 0), min(b[1] + m, n), max(b[2] - m, 0), min(b[3] + m, n)] if g
                              else list(b) for b, g in zip(boxes, grow)])
    wins = []
    for b, d, info, touched in results:
        T[b[0]:b[1], b[2]:b[3]] = d.astype(np.float32)
        info.update({"box_rows_cols": b, "x_range_m": [float(xs[b[2]]), float(xs[b[3] - 1])],
                     "y_range_m": [float(xs[b[0]]), float(xs[b[1] - 1])], "frame_touched": touched,
                     "grow_rounds": rnd, "n_changed_nodes": int((np.abs(d) > 1e-6).sum())})
        wins.append(info)
    meta["windows"] = wins
    meta["n_windows"] = len(wins)
    meta["elapsed_s"] = {"evaluate": round(t_eval, 2), "total": round(time.time() - t0, 2)}
    meta["max_abs_T_m"] = float(np.abs(T).max())
    meta["n_changed_nodes"] = int((np.abs(T) > 1e-6).sum())
    log(f"[terrain] talus: {len(wins)} window(s), {meta['n_changed_nodes']} nodes changed, max |T| "
        f"{meta['max_abs_T_m']:.3f} m, {meta['elapsed_s']['total']} s")
    return Grid(T, -half, -half, sp, meta)


# ---------------------------------------------------------------------------------------- cache + sampler
def talus_prefix(cfg: Config) -> Path:
    return Layout(cfg).stage_dir("terrain") / "talus" / "talus"


def load_or_build(model, key: str, rebuild: bool = False, log=print) -> Grid:
    """Cached T grid (build_dir/terrain/talus/talus.npy/.json, mmap); rebuilt when the key changed."""
    prefix = talus_prefix(model.cfg)
    j, npy = prefix.with_suffix(".json"), prefix.with_suffix(".npy")
    if not rebuild and j.exists() and npy.exists():
        try:
            if json.loads(j.read_text()).get("meta", {}).get("talus_key") == key:
                return Grid.load(prefix, mmap=True)
        except (OSError, json.JSONDecodeError):
            pass
    g = compute_talus(model, log=log)
    g.meta["talus_key"] = key
    prefix.parent.mkdir(parents=True, exist_ok=True)
    tmp = prefix.with_name(f"{prefix.name}_tmp{os.getpid()}")
    g.save(tmp)
    os.replace(tmp.with_suffix(".npy"), npy)
    os.replace(tmp.with_suffix(".json"), j)
    return Grid.load(prefix, mmap=True)


class TalusSampler:
    """Catmull-Rom interpolation of the T grid (exactly 0 where T and its neighbours are 0)."""

    def __init__(self, grid: Grid):
        self.T = np.ascontiguousarray(np.asarray(grid.z, dtype=np.float64))
        self.x0, self.y0, self.inv_h = grid.x0, grid.y0, 1.0 / grid.spacing
        self.empty = not bool(np.any(self.T))

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = np.broadcast_shapes(x.shape, y.shape)
        out = np.zeros(int(np.prod(shape)), dtype=np.float64)
        if out.size and not self.empty:
            _catmull_rom_kernel(self.T, self.x0, self.y0, self.inv_h,
                                np.ascontiguousarray(np.broadcast_to(x, shape).ravel()),
                                np.ascontiguousarray(np.broadcast_to(y, shape).ravel()), out)
        return out.reshape(shape)
