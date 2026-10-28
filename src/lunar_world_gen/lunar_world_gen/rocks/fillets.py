"""Regolith fillets around partly buried core rocks (docs/DESIGN.md sections 3 and 5).

``apply_fillets(cfg, grid, catalog)`` returns a copy of a core height grid (H0, 1 cm) with the
fillet field added. The field is a pure function of world position, of H0 at that node and of the
catalog, so any tile (with or without margin) gets identical values where tiles overlap.

Per rock (core rocks with ``D >= terrain.fillets.min_rock_D_m``, ``burial > min_burial`` and
``exposed_h > 0``; ``min_burial`` defaults to the upper bound of ``rocks.perched_burial``, so perched rocks bedded only
by the compliance get none; perched rocks bedded deeper, keels and boulders on undulating ground, get
one like buried rocks):

* fillet height ``h_f = u1 * exposed_h`` and width ``w_f = u2 * exposed_h`` with u1, u2 uniform in
  ``height_over_exposed_height`` (LM site: 0-0.15) and ``width_over_exposed_height`` (0.5-1.0).
  u1, u2 come from a hash of (world seed, rock id), so they do not depend on the order of rows;
* profile ``wedge`` (default, FID-11): regolith piled against the rock's real ground-level outline.
  Let d(z) be the horizontal distance from a node to the rock's convex hull cut by the local ground
  plane raised by z (the plane through the node with the H0 gradient over +-``GRAD_HALF_NODES``
  nodes; max of the signed distances to the hull's face planes, <= 0 inside). The fillet is
  ``h_f * (1 - smoothstep(d(0) / w_f))``: h_f where the rock body crosses the ground (hidden by the
  rock), decaying outward from the contact line to 0 at w_f, also on slopes and under overhangs.
  It is capped at the highest level z at which the rock is still within w_f (d(z) <= w_f), so it is
  never higher than the rock it leans on. Tiles agree bit for bit except within
  ``GRAD_HALF_NODES`` nodes of a grid's own edge (one-sided gradient there; the build's tiles carry
  a 50-node margin that is not meshed);
* profile ``ellipse`` (the pre-FID-11 shape): ``h_f`` inside the footprint ellipse
  ``0.45 * s * (a, b)`` turned by the rock's yaw, then a smoothstep ramp over ``w_f``;
* overlapping fillets combine with max, not sum.

H = H0 + fillets is used for the visual ground and the collision field; rocks stay placed on H0.
"""
from __future__ import annotations

import numpy as np
from numba import njit

from ..grid import LATTICE_M, Grid
from .placement import PERCHED_BURIAL, catalog_quats, quat_to_matrix, quat_yaw

FOOTPRINT_SCALE = 0.45          # ellipse semi-axis = 0.45 * extent (legacy 'ellipse' profile)
BISECT_ITERS = 16               # wedge profile: golden-section / bisection steps per node
GRAD_HALF_NODES = 8             # wedge profile: ground gradient by central differences over +-8 nodes


def hash_uniform(seed: int, ids: np.ndarray, salt: int) -> np.ndarray:
    """Deterministic uniform [0, 1) numbers from (seed, id, salt) (splitmix64)."""
    with np.errstate(over="ignore"):
        z = (np.asarray(ids, dtype=np.uint64) * np.uint64(0x9E3779B97F4A7C15)
             + np.uint64((int(seed) * 0x632BE59BD9B4E019 + salt * 0x85EBCA77C2B2AE63) & 0xFFFFFFFFFFFFFFFF))
        z = (z ^ (z >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        z = (z ^ (z >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        z = z ^ (z >> np.uint64(31))
    return (z >> np.uint64(11)).astype(np.float64) / float(1 << 53)


def fillet_settings(cfg) -> dict:
    fl = dict(cfg["terrain"]["fillets"])
    fl.setdefault("profile", "wedge")
    if fl.get("min_burial") is None:
        pb = cfg["rocks"].get("perched_burial", None)
        fl["min_burial"] = float((pb if pb is not None else PERCHED_BURIAL)[1])
    return fl


def fillet_mask(cfg, catalog: np.ndarray) -> np.ndarray:
    """Rows of ``catalog`` that get a fillet (the selection of ``fillet_params``; perched rocks, whose
    burial is only the compliance bedding, are excluded)."""
    fl = fillet_settings(cfg)
    cat = np.asarray(catalog)
    if not bool(fl.get("enabled", True)) or len(cat) == 0:
        return np.zeros(len(cat), dtype=bool)
    return ((cat["zone"] == 0) & (cat["D"] >= float(fl["min_rock_D_m"])) & (cat["burial"] > float(fl["min_burial"]))
            & (cat["exposed_h"] > 0))


def fillet_params(cfg, catalog: np.ndarray, library) -> dict[str, np.ndarray]:
    """Rows that get a fillet and their parameters: centre x, y; legacy ellipse A, B, cos, sin; height h,
    width w, id; and for the wedge profile the world-space hull planes of each rock (``pn`` (t,3),
    ``po`` (t,), offsets ``poff`` (n+1,)) and the horizontal reach ``reach`` (hull radius + w)."""
    fl = fillet_settings(cfg)
    cat = np.asarray(catalog)
    keys = ("x", "y", "A", "B", "cos", "sin", "h", "w", "id", "reach", "po")
    empty = {k: np.zeros(0) for k in keys}
    empty.update({"pn": np.zeros((0, 3)), "poff": np.zeros(1, dtype=np.int64)})
    if not bool(fl.get("enabled", True)) or len(cat) == 0:
        return empty
    rows = cat[fillet_mask(cfg, cat)]
    if not len(rows):
        return empty
    dims = library.dims_array()[rows["proto"]]
    A = FOOTPRINT_SCALE * rows["scale"] * dims[:, 0]
    B = FOOTPRINT_SCALE * rows["scale"] * dims[:, 1]
    yaw = quat_yaw(catalog_quats(rows))
    seed = int(cfg["world"]["seed"])
    h0, h1 = (float(v) for v in fl["height_over_exposed_height"])
    w0, w1 = (float(v) for v in fl["width_over_exposed_height"])
    u1 = hash_uniform(seed, rows["id"], 1)
    u2 = hash_uniform(seed, rows["id"], 2)
    h = (h0 + (h1 - h0) * u1) * rows["exposed_h"]
    w = np.maximum((w0 + (w1 - w0) * u2) * rows["exposed_h"], 1e-4)
    # world-space hull planes: n_w = R n, o_w = s o + n_w . t (vectorised per prototype, row order kept)
    R = quat_to_matrix(catalog_quats(rows))
    t = np.column_stack([rows["x"], rows["y"], rows["z"]]).astype(np.float64)
    sc = rows["scale"].astype(np.float64)
    nface = np.array([len(library.hull_planes(p)[1]) for p in range(len(library))], np.int64)
    rmax = np.array([float(np.linalg.norm(library.hull(p), axis=1).max()) for p in range(len(library))])
    proto = rows["proto"].astype(np.int64)
    poff = np.zeros(len(rows) + 1, np.int64)
    poff[1:] = np.cumsum(nface[proto])
    pn = np.empty((poff[-1], 3))
    po = np.empty(poff[-1])
    for pp in np.unique(proto):
        sel = np.nonzero(proto == pp)[0]
        n_l, o_l = library.hull_planes(int(pp))
        nw = np.einsum("kij,fj->kfi", R[sel], n_l)                       # (k, F, 3)
        ow = sc[sel, None] * o_l[None, :] + np.einsum("kfi,ki->kf", nw, t[sel])
        dst = poff[sel][:, None] + np.arange(len(o_l))[None, :]
        pn[dst] = nw
        po[dst] = ow
    return {"x": rows["x"].astype(np.float64), "y": rows["y"].astype(np.float64), "A": A, "B": B,
            "cos": np.cos(yaw), "sin": np.sin(yaw), "h": h, "w": w, "id": rows["id"],
            "reach": sc * rmax[proto] + w, "pn": pn, "po": po, "poff": poff}


@njit(cache=False)
def _fillet_field(px, py, A, B, c, s, h, w, x0, y0, step, lx0, ly0, lstep, unit, nx, ny, out):
    # legacy 'ellipse' profile.
    # node coordinates: from integer lattice indices when the grid is on the lattice (unit > 0), so
    # overlapping tiles evaluate bit-identical coordinates; else x0 + i * step
    for k in range(px.shape[0]):
        if h[k] <= 0.0:
            continue
        reach = max(A[k], B[k]) + w[k]
        i0 = max(0, int(np.ceil((px[k] - reach - x0) / step)))
        i1 = min(nx - 1, int(np.floor((px[k] + reach - x0) / step)))
        j0 = max(0, int(np.ceil((py[k] - reach - y0) / step)))
        j1 = min(ny - 1, int(np.floor((py[k] + reach - y0) / step)))
        for j in range(j0, j1 + 1):
            if unit > 0.0:
                dy = (ly0 + j * lstep) * unit - py[k]
            else:
                dy = y0 + j * step - py[k]
            for i in range(i0, i1 + 1):
                if unit > 0.0:
                    dx = (lx0 + i * lstep) * unit - px[k]
                else:
                    dx = x0 + i * step - px[k]
                u = c[k] * dx + s[k] * dy
                v = -s[k] * dx + c[k] * dy
                rho = np.sqrt((u / A[k]) ** 2 + (v / B[k]) ** 2)
                if rho <= 1.0:
                    val = h[k]
                else:
                    d = np.sqrt(u * u + v * v) * (1.0 - 1.0 / rho)     # radial distance beyond the ellipse
                    if d >= w[k]:
                        continue
                    t = d / w[k]
                    val = h[k] * (1.0 - t * t * (3.0 - 2.0 * t))
                if val > out[j, i]:
                    out[j, i] = val


@njit(cache=False)
def _cut_distance(PN, PO, f0, f1, X, Y, Z, gx, gy):
    """Horizontal distance from (X, Y) to the convex hull cut by the plane through (X, Y, Z) with slope
    (gx, gy) (max of signed half-plane distances: exact along the edges, a lower bound at corners;
    <= 0 inside; large if the cut is empty)."""
    d = -1e300
    for f in range(f0, f1):
        ax = PN[f, 0] + PN[f, 2] * gx
        ay = PN[f, 1] + PN[f, 2] * gy
        v = (PN[f, 0] * X + PN[f, 1] * Y + PN[f, 2] * Z - PO[f]) / max(np.sqrt(ax * ax + ay * ay), 1e-9)
        if v > d:
            d = v
    return d


@njit(cache=False)
def _fillet_wedge(px, py, reach, h, w, PN, PO, POFF, Z, GX, GY, x0, y0, step, lx0, ly0, lstep, unit, nx, ny, iters,
                  out):
    for k in range(px.shape[0]):
        if h[k] <= 0.0:
            continue
        i0 = max(0, int(np.ceil((px[k] - reach[k] - x0) / step)))
        i1 = min(nx - 1, int(np.floor((px[k] + reach[k] - x0) / step)))
        j0 = max(0, int(np.ceil((py[k] - reach[k] - y0) / step)))
        j1 = min(ny - 1, int(np.floor((py[k] + reach[k] - y0) / step)))
        f0 = POFF[k]
        f1 = POFF[k + 1]
        hk = h[k]
        wk = w[k]
        for j in range(j0, j1 + 1):
            if unit > 0.0:
                Y = (ly0 + j * lstep) * unit
            else:
                Y = y0 + j * step
            for i in range(i0, i1 + 1):
                if unit > 0.0:
                    X = (lx0 + i * lstep) * unit
                else:
                    X = x0 + i * step
                if (X - px[k]) ** 2 + (Y - py[k]) ** 2 > reach[k] * reach[k]:
                    continue
                G = Z[j, i]
                gx = GX[j, i]
                gy = GY[j, i]
                # distance to the rock's ground-level cross-section (cut by the local ground plane)
                d0 = _cut_distance(PN, PO, f0, f1, X, Y, G, gx, gy)
                if d0 >= wk:
                    continue
                t = d0 / wk
                if t < 0.0:
                    t = 0.0
                val = hk * (1.0 - t * t * (3.0 - 2.0 * t))
                # never higher than the rock it leans on: cap at the highest level above the local
                # ground at which the rock is still within reach (d is convex in that level)
                if _cut_distance(PN, PO, f0, f1, X, Y, G + val, gx, gy) > wk:
                    lo = 0.0
                    hi = val
                    for _ in range(iters):
                        mid = 0.5 * (lo + hi)
                        if _cut_distance(PN, PO, f0, f1, X, Y, G + mid, gx, gy) <= wk:
                            lo = mid
                        else:
                            hi = mid
                    val = lo
                if val > out[j, i]:
                    out[j, i] = val


def fillet_field(cfg, grid: Grid, catalog: np.ndarray, library, params: dict | None = None) -> np.ndarray:
    """The fillet height field (>= 0) on the nodes of ``grid`` (float64, same shape as grid.z).
    ``grid`` holds H0 (the wedge profile reads the ground height at every node); ``params`` may pass
    precomputed ``fillet_params``."""
    p = params if params is not None else fillet_params(cfg, catalog, library)
    out = np.zeros(grid.z.shape, dtype=np.float64)
    if not len(p["x"]):
        return out
    profile = str(fillet_settings(cfg)["profile"])
    try:
        lx0, ly0 = grid.lattice_origin()
        lstep, unit = grid.lattice_step(), LATTICE_M
    except ValueError:                       # grid not on the 1 cm lattice
        lx0 = ly0 = lstep = 0
        unit = 0.0
    x0, x1, y0, y1 = grid.extent
    if profile == "ellipse":
        reach = np.maximum(p["A"], p["B"]) + p["w"]
        near = (p["x"] + reach >= x0) & (p["x"] - reach <= x1) & (p["y"] + reach >= y0) & (p["y"] - reach <= y1)
        q = {k: np.ascontiguousarray(p[k][near], dtype=np.float64) for k in ("x", "y", "A", "B", "cos", "sin", "h", "w")}
        _fillet_field(q["x"], q["y"], q["A"], q["B"], q["cos"], q["sin"], q["h"], q["w"],
                      grid.x0, grid.y0, grid.spacing, lx0, ly0, lstep, unit, grid.nx, grid.ny, out)
        return out
    reach = p["reach"]
    near = np.nonzero((p["x"] + reach >= x0) & (p["x"] - reach <= x1) & (p["y"] + reach >= y0)
                      & (p["y"] - reach <= y1))[0]
    if not len(near):
        return out
    cnt = p["poff"][near + 1] - p["poff"][near]
    poff = np.zeros(len(near) + 1, np.int64)
    poff[1:] = np.cumsum(cnt)
    rows = np.concatenate([np.arange(p["poff"][k], p["poff"][k + 1]) for k in near])
    Z = np.ascontiguousarray(grid.z, dtype=np.float64)
    if bool(fillet_settings(cfg).get("slope_aware", True)):
        GX, GY = ground_gradient(Z, grid.spacing)
    else:
        GX = GY = np.zeros_like(Z)
    _fillet_wedge(np.ascontiguousarray(p["x"][near]), np.ascontiguousarray(p["y"][near]),
                  np.ascontiguousarray(reach[near]), np.ascontiguousarray(p["h"][near]),
                  np.ascontiguousarray(p["w"][near]), np.ascontiguousarray(p["pn"][rows]),
                  np.ascontiguousarray(p["po"][rows]), poff, Z, GX, GY, grid.x0, grid.y0, grid.spacing,
                  lx0, ly0, lstep, unit, grid.nx, grid.ny, BISECT_ITERS, out)
    return out


def ground_gradient(Z: np.ndarray, spacing: float, k: int = GRAD_HALF_NODES) -> tuple[np.ndarray, np.ndarray]:
    """(dz/dx, dz/dy) by central differences over +-k nodes (edge values repeated beyond the grid)."""
    k = max(1, min(int(k), max(1, min(Z.shape) // 2 - 1)))
    Zp = np.pad(np.asarray(Z, dtype=np.float64), k, mode="edge")
    gx = (Zp[k:-k, 2 * k:] - Zp[k:-k, :-2 * k]) / (2.0 * k * spacing)
    gy = (Zp[2 * k:, k:-k] - Zp[:-2 * k, k:-k]) / (2.0 * k * spacing)
    return np.ascontiguousarray(gx), np.ascontiguousarray(gy)


def apply_fillets(cfg, grid: Grid, catalog: np.ndarray, library=None) -> Grid:
    """Copy of ``grid`` with the fillets of the catalog's core rocks added (H = H0 + fillets)."""
    if library is None:
        from .library import RockLibrary

        library = RockLibrary(cfg["paths"]["rock_library"])
    p = fillet_params(cfg, catalog, library)
    f = fillet_field(cfg, grid, catalog, library, params=p)
    meta = dict(grid.meta)
    meta["fillets"] = {"n_rocks": int(np.count_nonzero(p["h"] > 0)),
                       "max_m": float(f.max()) if f.size else 0.0,
                       "profile": str(fillet_settings(cfg)["profile"])}
    return Grid((np.asarray(grid.z, dtype=np.float64) + f).astype(grid.z.dtype, copy=False),
                grid.x0, grid.y0, grid.spacing, meta)
