"""Collision height fields with rasterised rock hulls (docs/DESIGN.md section 7).

Core (2 cm grid)::

    C = lowpass(H, 0.25 m) + ground_offset_m          Gaussian, sigma = 0.25 * sqrt(ln 2 / (2 pi^2)) = 4.69 cm
    C = max(C, rock_top + rock_offset_m)               rocks with collide = true (core rocks >= 5 cm)
                                                       + ring rocks >= 30 cm reaching over the core edge

Ring (25 cm grid)::

    C = H_ring + ground_offset_m
    C = max(C, rock_top_dilated + rock_offset_m)       rocks with D >= rocks.D_min_m.ring (30 cm)

``rock_top(x, y)`` is the highest point of the transformed convex hull above (x, y), found by
rasterising every hull triangle with a max-z buffer (numba). For a convex hull the top faces
always win, so the result is exactly the hull's upper surface sampled at the grid nodes.

``rock_top_dilated`` (ring only, FID-08): a 25 cm node spacing is close to the size of the
smallest ring rocks (30-50 cm), so point samples often hit only a rock's flank or miss it (phase-3
world: 8.6 % of the ring rocks kept < 25 % of their rise in the ring collision, 15.6 % of the
30-50 cm rocks < 50 %). Each ring node therefore takes the maximum of the hull's upper surface
over the square ``node +- w`` (``w = mesh.collision.ring_rock_dilation`` x node spacing; exact:
every hull triangle is clipped to the square and the linear height evaluated at the clipped
polygon's vertices). With the default ``w`` = one node spacing, the four corner nodes of every grid
cell that a rock's footprint touches are at least as high as the rock anywhere in that cell, so
the piecewise linear surface through the nodes covers the whole footprint (conservative: the
collision rock is wider than the visual one by up to one node spacing on each side, plus the TIN
tolerance ``mesh.collision.ring_max_error_m``). ``ring_rock_dilation: 0`` restores point sampling.

Rocks of any zone are rasterised if their hull touches the grid, so a core rock that crosses the
core boundary also raises the ring collision and vice versa. The Gaussian uses mode 'nearest' at
the grid edge: pass a ground grid with a margin of >= 3 sigma (15 cm) if the edge values matter.
The core-edge band rocks (ring zone, D < 30 cm, FID-04) are visual only, like the core pebbles: the
25 cm ring grid cannot represent them and the rover operates inside the core.
"""
from __future__ import annotations

import math

import numpy as np
from numba import njit
from scipy import ndimage

from ..grid import Grid
from .placement import catalog_quats, quat_to_matrix

#: default ``mesh.collision.ring_rock_dilation`` (half-width of the max square in node spacings, FID-08)
RING_ROCK_DILATION = 1.0


def lowpass_sigma_m(wavelength_m: float) -> float:
    """Gaussian sigma whose transfer function is 0.5 at the given wavelength."""
    return float(wavelength_m) * math.sqrt(math.log(2.0) / (2.0 * math.pi ** 2))


def gaussian_lowpass(grid: Grid, wavelength_m: float) -> Grid:
    sigma_px = lowpass_sigma_m(wavelength_m) / grid.spacing
    z = ndimage.gaussian_filter(np.asarray(grid.z, dtype=np.float64), sigma_px, mode="nearest", truncate=4.0)
    return Grid(z, grid.x0, grid.y0, grid.spacing, dict(grid.meta))


def transformed_hulls(rows: np.ndarray, library) -> tuple[np.ndarray, np.ndarray]:
    """World-space hull triangles of the given catalog rows: (tris (m,3,3), owner row index (m,))."""
    rows = np.asarray(rows)
    if not len(rows):
        return np.zeros((0, 3, 3)), np.zeros(0, dtype=np.int64)
    R = quat_to_matrix(catalog_quats(rows))
    t = np.column_stack([rows["x"], rows["y"], rows["z"]])
    out, owner = [], []
    for p in np.unique(rows["proto"]):
        sel = np.nonzero(rows["proto"] == p)[0]
        V = library.hull(int(p))
        F = library.hull_faces(int(p))
        W = np.einsum("kij,mj->kmi", R[sel], V) * rows["scale"][sel, None, None] + t[sel, None, :]   # (k, m, 3)
        out.append(W[:, F, :].reshape(-1, 3, 3))
        owner.append(np.repeat(sel, len(F)))
    return np.concatenate(out), np.concatenate(owner)


@njit(cache=False)
def _raster_max(tris, x0, y0, step, nx, ny, out):
    eps = 1e-9
    for t in range(tris.shape[0]):
        ax, ay, az = tris[t, 0, 0], tris[t, 0, 1], tris[t, 0, 2]
        bx, by, bz = tris[t, 1, 0], tris[t, 1, 1], tris[t, 1, 2]
        cx, cy, cz = tris[t, 2, 0], tris[t, 2, 1], tris[t, 2, 2]
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if abs(area) < 1e-14:
            continue                                   # vertical in xy: covered by its neighbours
        i0 = max(0, int(np.ceil((min(ax, bx, cx) - x0) / step - 1e-9)))
        i1 = min(nx - 1, int(np.floor((max(ax, bx, cx) - x0) / step + 1e-9)))
        j0 = max(0, int(np.ceil((min(ay, by, cy) - y0) / step - 1e-9)))
        j1 = min(ny - 1, int(np.floor((max(ay, by, cy) - y0) / step + 1e-9)))
        inv = 1.0 / area
        for j in range(j0, j1 + 1):
            y = y0 + j * step
            for i in range(i0, i1 + 1):
                x = x0 + i * step
                w0 = ((bx - x) * (cy - y) - (cx - x) * (by - y)) * inv
                w1 = ((cx - x) * (ay - y) - (ax - x) * (cy - y)) * inv
                w2 = 1.0 - w0 - w1
                if w0 >= -eps and w1 >= -eps and w2 >= -eps:
                    z = w0 * az + w1 * bz + w2 * cz
                    if z > out[j, i]:
                        out[j, i] = z


@njit(cache=False)
def _clip_half_plane(px, py, pz, n, axis, bound, keep_ge, qx, qy, qz):
    """Sutherland-Hodgman step: clip the polygon (px, py, pz)[:n] to x (axis 0) or y (axis 1) >= bound
    (``keep_ge``) or <= bound, writing the result to q*; z is linear along the edges. Returns its size."""
    m = 0
    for k in range(n):
        k2 = k + 1 if k + 1 < n else 0
        va = px[k] if axis == 0 else py[k]
        vb = px[k2] if axis == 0 else py[k2]
        ina = va >= bound if keep_ge else va <= bound
        inb = vb >= bound if keep_ge else vb <= bound
        if ina:
            qx[m] = px[k]
            qy[m] = py[k]
            qz[m] = pz[k]
            m += 1
        if ina != inb:                                 # the edge crosses the bound (va != vb)
            t = (bound - va) / (vb - va)
            qx[m] = px[k] + t * (px[k2] - px[k])
            qy[m] = py[k] + t * (py[k2] - py[k])
            qz[m] = pz[k] + t * (pz[k2] - pz[k])
            m += 1
    return m


@njit(cache=False)
def _raster_max_square(tris, x0, y0, step, nx, ny, hw, out):
    """out[j, i] = max(out[j, i], max of z over (triangle within the square node +- hw)) for every
    triangle; exact for planar triangles (the max of a linear function over the clipped convex
    polygon is at one of its vertices)."""
    px = np.empty(16)
    py = np.empty(16)
    pz = np.empty(16)
    qx = np.empty(16)
    qy = np.empty(16)
    qz = np.empty(16)
    for t in range(tris.shape[0]):
        ax, ay, az = tris[t, 0, 0], tris[t, 0, 1], tris[t, 0, 2]
        bx, by, bz = tris[t, 1, 0], tris[t, 1, 1], tris[t, 1, 2]
        cx, cy, cz = tris[t, 2, 0], tris[t, 2, 1], tris[t, 2, 2]
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if abs(area) < 1e-14:
            continue                                   # vertical in xy: covered by its neighbours
        xmin, xmax = min(ax, bx, cx), max(ax, bx, cx)
        ymin, ymax = min(ay, by, cy), max(ay, by, cy)
        ztop = max(az, bz, cz)
        i0 = max(0, int(np.ceil((xmin - hw - x0) / step - 1e-9)))
        i1 = min(nx - 1, int(np.floor((xmax + hw - x0) / step + 1e-9)))
        j0 = max(0, int(np.ceil((ymin - hw - y0) / step - 1e-9)))
        j1 = min(ny - 1, int(np.floor((ymax + hw - y0) / step + 1e-9)))
        for j in range(j0, j1 + 1):
            ylo = y0 + j * step - hw
            yhi = y0 + j * step + hw
            for i in range(i0, i1 + 1):
                if ztop <= out[j, i]:
                    continue                           # this triangle cannot raise the node
                xlo = x0 + i * step - hw
                xhi = x0 + i * step + hw
                if xmin >= xlo and xmax <= xhi and ymin >= ylo and ymax <= yhi:
                    out[j, i] = ztop                   # the whole triangle lies in the square
                    continue
                px[0], py[0], pz[0] = ax, ay, az
                px[1], py[1], pz[1] = bx, by, bz
                px[2], py[2], pz[2] = cx, cy, cz
                n = _clip_half_plane(px, py, pz, 3, 0, xlo, True, qx, qy, qz)
                n = _clip_half_plane(qx, qy, qz, n, 0, xhi, False, px, py, pz)
                n = _clip_half_plane(px, py, pz, n, 1, ylo, True, qx, qy, qz)
                n = _clip_half_plane(qx, qy, qz, n, 1, yhi, False, px, py, pz)
                for k in range(n):
                    if pz[k] > out[j, i]:
                        out[j, i] = pz[k]


def rasterize_tops(tris: np.ndarray, x0: float, y0: float, spacing: float, nx: int, ny: int,
                   half_width: float = 0.0) -> np.ndarray:
    """Max-z raster of triangles on grid nodes; -inf where no triangle covers a node.

    ``half_width`` > 0: each node takes the max over the square ``node +- half_width`` instead of the
    point value (exact, see ``_raster_max_square``)."""
    out = np.full((ny, nx), -np.inf)
    if len(tris):
        t = np.ascontiguousarray(tris, dtype=np.float64)
        if half_width > 0.0:
            _raster_max_square(t, float(x0), float(y0), float(spacing), int(nx), int(ny), float(half_width), out)
        else:
            _raster_max(t, float(x0), float(y0), float(spacing), int(nx), int(ny), out)
    return out


def rows_near_grid(grid: Grid, rows: np.ndarray, library, margin: float = 0.0) -> np.ndarray:
    """Rows whose hull can reach the grid (bounding radius = D around the centre, plus ``margin``)."""
    rows = np.asarray(rows)
    if not len(rows):
        return rows
    a = library.a_array()[rows["proto"]] * rows["scale"] + float(margin)
    x0, x1, y0, y1 = grid.extent
    near = (rows["x"] + a >= x0) & (rows["x"] - a <= x1) & (rows["y"] + a >= y0) & (rows["y"] - a <= y1)
    return rows[near]


def rock_top_grid(grid: Grid, rows: np.ndarray, library, half_width: float = 0.0) -> np.ndarray:
    """rock_top on the nodes of ``grid`` for the given rows (rows that cannot reach the grid are skipped);
    ``half_width`` > 0: max over the square node +- half_width (see ``rasterize_tops``)."""
    tris, _ = transformed_hulls(rows_near_grid(grid, rows, library, margin=half_width), library)
    return rasterize_tops(tris, grid.x0, grid.y0, grid.spacing, grid.nx, grid.ny, half_width=half_width)


def _combine(base: np.ndarray, top: np.ndarray, rock_offset: float) -> tuple[np.ndarray, int]:
    has = np.isfinite(top)
    raised = has & (top + rock_offset > base)
    out = np.where(raised, top + rock_offset, base)
    return out, int(raised.sum())


def core_collision_grid(cfg, ground_2cm: Grid, catalog: np.ndarray, library) -> Grid:
    """Core collision height field (DESIGN 7) on the nodes of ``ground_2cm`` (H incl. fillets)."""
    col = cfg["mesh"]["collision"]
    lp = gaussian_lowpass(ground_2cm, float(col["lowpass_wavelength_m"]))
    base = lp.z + float(col["ground_offset_m"])
    cat = np.asarray(catalog)
    if len(cat):
        # core rocks flagged collide, plus ring rocks >= 30 cm whose hull may reach over the core edge
        ring_dmin = float(cfg["rocks"]["D_min_m"]["ring"])
        rows = cat[cat["collide"] | ((cat["zone"] == 1) & (cat["D"] >= ring_dmin))]
    else:
        rows = cat
    rows = rows_near_grid(ground_2cm, rows, library)
    top = rock_top_grid(ground_2cm, rows, library)
    z, n_raised = _combine(base, top, float(col["rock_offset_m"]))
    meta = {"kind": "core_collision", "lowpass_sigma_m": lowpass_sigma_m(float(col["lowpass_wavelength_m"])),
            "ground_offset_m": float(col["ground_offset_m"]), "rock_offset_m": float(col["rock_offset_m"]),
            "n_rocks": int(len(rows)), "n_nodes_raised": n_raised}
    return Grid(z.astype(np.float32 if ground_2cm.z.dtype == np.float32 else np.float64), ground_2cm.x0,
                ground_2cm.y0, ground_2cm.spacing, meta)


def ring_rock_dilation(cfg) -> float:
    """``mesh.collision.ring_rock_dilation`` (node spacings; default ``RING_ROCK_DILATION``)."""
    v = cfg["mesh"]["collision"].get("ring_rock_dilation", None)
    return RING_ROCK_DILATION if v is None else max(float(v), 0.0)


def ring_collision_grid(cfg, ring_ground: Grid, catalog: np.ndarray, library,
                        dilation: float | None = None) -> Grid:
    """Ring collision height field: visual ground + offset, rocks >= rocks.D_min_m.ring rasterised with
    the FID-08 footprint dilation (``dilation`` node spacings; default from the config, see the module
    docstring; 0 = point samples)."""
    col = cfg["mesh"]["collision"]
    base = np.asarray(ring_ground.z, dtype=np.float64) + float(col["ground_offset_m"])
    cat = np.asarray(catalog)
    dmin = float(cfg["rocks"]["D_min_m"]["ring"])
    dil = ring_rock_dilation(cfg) if dilation is None else max(float(dilation), 0.0)
    hw = dil * float(ring_ground.spacing)
    rows = cat[cat["D"] >= dmin] if len(cat) else cat           # any zone: core rocks may cross the edge
    rows = rows_near_grid(ring_ground, rows, library, margin=hw)
    top = rock_top_grid(ring_ground, rows, library, half_width=hw)
    z, n_raised = _combine(base, top, float(col["rock_offset_m"]))
    meta = {"kind": "ring_collision", "ground_offset_m": float(col["ground_offset_m"]),
            "rock_offset_m": float(col["rock_offset_m"]), "rock_min_D_m": dmin,
            "rock_dilation_nodes": dil, "rock_dilation_half_width_m": hw,
            "n_rocks": int(len(rows)), "n_nodes_raised": n_raised}
    return Grid(z.astype(np.float32 if ring_ground.z.dtype == np.float32 else np.float64), ring_ground.x0,
                ring_ground.y0, ring_ground.spacing, meta)
