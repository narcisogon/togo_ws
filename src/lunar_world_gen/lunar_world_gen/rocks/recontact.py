"""Re-seat placed rocks on the RENDERED ground (CR-05, FID-08, FID-10).

``rocks.placement`` puts every rock on the height grid H0 (core: 1 cm, ring: 25 cm, bilinear). What
Gazebo renders, however, is the TIN of that grid (core tiles: ``mesh.core_visual_max_error_m``, ring
strips: ``mesh.ring_visual_max_error_m``), which lies up to that tolerance above or below the grid.
Measured on the phase-3 world: ring rocks hovered up to 43 mm above the rendered ring TIN (5 cm
tolerance), perched core rocks 1-3.7 mm above the 5 mm core TIN, and 271 core rocks were entirely
below it (invisible).

``recontact(cfg, catalog, library, height_fn, rows=None)`` recomputes ``z`` (and ``exposed_h``) of
the selected rows against any surface ``height_fn(x, y)`` with the SAME contact rule as
``rocks.placement``: the rotated rock is lowered until its lowest visual-LOD vertex (the library's
``contact_points``) touches the surface under that vertex, then sunk by ``burial * (vertical
extent of the rotated rock)``; ``exposed_h`` = rock top above the surface at the rock centre.
Orientation, scale, burial fraction and everything else are kept. With the placement ground as
``height_fn`` it reproduces the placement heights exactly (tests/test_world_build_fixes.py).

The integration build (``build.py``, stage ``contact``) calls it with the stitched visual ground
TINs (``mesh.locate.PiecewiseTinHeight``) before the rock meshes are merged and before the rock tops
are rasterised into the collision fields, so the rendered rocks, the collision proxies and the
catalog agree. Rocks with a regolith fillet (``rocks.fillets``) keep their H0 placement: their
fillet was built around them on H0 and is part of the rendered ground, so seating them on it would
lift them onto their own fillet.

Rows are processed in spatial groups (10 m cells in the core, 50 m in the ring) so that a
piecewise height function only needs the pieces around one group at a time.
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from .placement import catalog_quats, quat_to_matrix

HeightFn = Callable[[np.ndarray, np.ndarray], np.ndarray]

__all__ = ["recontact", "contact_heights", "contact_clearance", "fillet_rows"]


def contact_heights(height_fn: HeightFn, library, x, y, scale, q, proto, lod, burial,
                    chunk: int = 400_000) -> tuple[np.ndarray, np.ndarray]:
    """Centre heights z and exposed heights for rocks resting on ``height_fn`` (placement rule)."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    n = len(x)
    z = np.empty(n)
    exposed = np.empty(n)
    if n == 0:
        return z, exposed
    R = quat_to_matrix(q)
    g_centre = np.asarray(height_fn(x, y), dtype=np.float64).ravel()
    key = np.asarray(proto, dtype=np.int64) * 16 + np.asarray(lod, dtype=np.int64)
    for k in np.unique(key):
        sel = np.nonzero(key == k)[0]
        pts = library.contact_points(int(k // 16), int(k % 16))                    # (m, 3)
        m = len(pts)
        step = max(1, chunk // max(m, 1))
        for s0 in range(0, len(sel), step):
            ss = sel[s0:s0 + step]
            V = np.einsum("kij,mj->kmi", R[ss], pts) * scale[ss, None, None]        # (k, m, 3) offsets
            gx = x[ss, None] + V[:, :, 0]
            gy = y[ss, None] + V[:, :, 1]
            g = np.asarray(height_fn(gx.ravel(), gy.ravel()), dtype=np.float64).reshape(gx.shape)
            zc = np.max(g - V[:, :, 2], axis=1)             # the lowest vertex touches the surface
            zmin = V[:, :, 2].min(axis=1)
            zmax = V[:, :, 2].max(axis=1)
            zc = zc - burial[ss] * (zmax - zmin)
            z[ss] = zc
            exposed[ss] = zc + zmax - g_centre[ss]
    return z, exposed


def contact_clearance(height_fn: HeightFn, library, rows: np.ndarray, chunk: int = 400_000, *,
                      core_cell_m: float = 10.0, ring_cell_m: float = 50.0) -> tuple[np.ndarray, np.ndarray]:
    """Per row: (min, max) over the rock's contact vertices of (vertex z - surface z under it).

    min > 0: the rock hovers above the surface; max < 0: the whole rock is below the surface.
    Rows are processed in the same spatial groups as ``recontact`` (a piecewise surface then only
    needs the pieces around one group at a time).
    """
    rows = np.asarray(rows)
    n = len(rows)
    lo = np.full(n, np.nan)
    hi = np.full(n, np.nan)
    if n == 0:
        return lo, hi
    R = quat_to_matrix(catalog_quats(rows))
    x, y, zr, s = (rows[f].astype(np.float64) for f in ("x", "y", "z", "scale"))
    key = rows["proto"].astype(np.int64) * 16 + rows["lod"].astype(np.int64)
    for grp in _groups(rows, np.arange(n), core_cell_m, ring_cell_m):
        for k in np.unique(key[grp]):
            sel = grp[key[grp] == k]
            pts = library.contact_points(int(k // 16), int(k % 16))
            step = max(1, chunk // max(len(pts), 1))
            for s0 in range(0, len(sel), step):
                ss = sel[s0:s0 + step]
                V = np.einsum("kij,mj->kmi", R[ss], pts) * s[ss, None, None]
                g = np.asarray(height_fn((x[ss, None] + V[:, :, 0]).ravel(), (y[ss, None] + V[:, :, 1]).ravel()),
                               dtype=np.float64).reshape(V.shape[:2])
                c = zr[ss, None] + V[:, :, 2] - g
                lo[ss] = c.min(axis=1)
                hi[ss] = c.max(axis=1)
    return lo, hi


def fillet_rows(cfg, catalog: np.ndarray) -> np.ndarray:
    """Mask of rows that carry a regolith fillet: exactly the selection ``rocks.fillets.apply_fillets``
    uses (``fillet_mask``: core rocks >= ``terrain.fillets.min_rock_D_m`` with burial above
    ``terrain.fillets.min_burial`` and ``exposed_h > 0``). Perched rocks carry a compliance burial
    (``rocks.perched_burial``, <= min_burial) but no fillet, so ``burial > 0`` would wrongly keep them
    on H0 instead of re-seating them on the rendered ground (fix_rocks.md 9.2)."""
    from .fillets import fillet_mask

    return fillet_mask(cfg, catalog)


def _groups(cat: np.ndarray, idx: np.ndarray, core_cell: float, ring_cell: float) -> list[np.ndarray]:
    zone = cat["zone"][idx].astype(np.int64)
    cell = np.where(zone == 0, core_cell, ring_cell)
    gx = np.floor(cat["x"][idx] / cell).astype(np.int64)
    gy = np.floor(cat["y"][idx] / cell).astype(np.int64)
    key = (zone * 4_000_000 + (gx + 1000) * 2000 + (gy + 1000))
    order = np.argsort(key, kind="stable")
    ks = key[order]
    cuts = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1]])
    return [idx[order[a:b]] for a, b in zip(cuts, list(cuts[1:]) + [len(ks)])]


def recontact(cfg, catalog: np.ndarray, library, height_fn: HeightFn, rows=None, *,
              core_cell_m: float | None = None, ring_cell_m: float = 50.0) -> np.ndarray:
    """Copy of ``catalog`` with ``z`` and ``exposed_h`` of ``rows`` re-seated on ``height_fn``.

    Args:
        cfg: generator config (for the core tile size used to group rows spatially).
        catalog: rock catalog (structured array, ``rocks.catalog.CATALOG_DTYPE``).
        library: ``RockLibrary`` (contact points of the LOD each row is merged with).
        height_fn: vectorised surface height ``f(x, y)`` (e.g. the rendered ground TIN).
        rows: boolean mask or index array of rows to re-seat (default: all rows).
    """
    cat = np.array(catalog, copy=True)
    n = len(cat)
    if rows is None:
        idx = np.arange(n)
    else:
        rows = np.asarray(rows)
        idx = np.flatnonzero(rows) if rows.dtype == bool else np.unique(rows.astype(np.int64))
    if idx.size == 0:
        return cat
    if core_cell_m is None:
        core_cell_m = float(cfg["zones"]["core"]["tile_size_m"])
    q_all = catalog_quats(cat)
    for g in _groups(cat, idx, core_cell_m, ring_cell_m):
        r = cat[g]
        z, exposed = contact_heights(height_fn, library, r["x"], r["y"], r["scale"].astype(np.float64), q_all[g],
                                     r["proto"], r["lod"], r["burial"].astype(np.float64))
        cat["z"][g] = z
        cat["exposed_h"][g] = exposed
    return cat
