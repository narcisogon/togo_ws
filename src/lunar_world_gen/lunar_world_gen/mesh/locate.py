"""Point location and height queries on 2.5-D TINs (the rendered ground surface).

* ``TinLocator`` - uniform bucket grid over a planar triangulation; ``locate`` returns the
  containing triangle and barycentric coordinates, ``interpolate`` the linearly interpolated
  per-vertex value (NaN outside). Moved here from ``validate.static_checks`` (which re-exports it)
  so that the build can seat rocks on the meshes it renders.
* ``PiecewiseTinHeight`` - the height of a set of non-overlapping ground pieces (core tiles, ring /
  far strips), each a rectangle plus a lazily loaded mesh; a small LRU cache keeps the locators of
  recently used pieces. Points outside every piece (or not located because of rounding on a piece
  edge) fall back to ``fallback(x, y)`` if given, else NaN.
"""
from __future__ import annotations

import math
from collections import OrderedDict
from typing import Callable, Sequence

import numpy as np

__all__ = ["TinLocator", "PiecewiseTinHeight"]


class TinLocator:
    """Locate points in a planar triangulation (x, y) with a uniform bucket grid.

    Works for any non-overlapping 2.5-D TIN (pydelatin tiles, collision meshes). ``interpolate``
    returns the linearly interpolated value of the containing triangle (NaN outside).
    """

    def __init__(self, xy: np.ndarray, triangles: np.ndarray, cell: float | None = None,
                 max_cells: int = 4_000_000):
        self.xy = np.asarray(xy, dtype=np.float64)[:, :2]
        tris = np.asarray(triangles, dtype=np.int64)
        p0, p1, p2 = self.xy[tris[:, 0]], self.xy[tris[:, 1]], self.xy[tris[:, 2]]
        v0, v1 = p1 - p0, p2 - p0
        det = v0[:, 0] * v1[:, 1] - v0[:, 1] * v1[:, 0]
        keep = np.abs(det) > 1e-18
        self.tris = tris[keep]
        self.p0, self.v0, self.v1, self.det = p0[keep], v0[keep], v1[keep], det[keep]
        lo = np.minimum(np.minimum(p0, p1), p2)[keep]
        hi = np.maximum(np.maximum(p0, p1), p2)[keep]
        m = len(self.tris)
        if m == 0:
            raise ValueError("TIN has no non-degenerate triangle")
        self.x0, self.y0 = lo.min(0)
        X1, Y1 = hi.max(0)
        span_x, span_y = max(X1 - self.x0, 1e-9), max(Y1 - self.y0, 1e-9)
        if cell is None:
            cell = 1.5 * math.sqrt(span_x * span_y / m)
        cell = max(cell, 1e-6)
        max_entries = max(8 * m, 1_000_000)
        while True:  # grow cells until the bucket index is bounded (big triangles span many cells)
            nxc = int(span_x / cell) + 1
            nyc = int(span_y / cell) + 1
            ix0 = np.clip(((lo[:, 0] - self.x0) / cell).astype(np.int64), 0, nxc - 1)
            ix1 = np.clip(((hi[:, 0] - self.x0) / cell).astype(np.int64), 0, nxc - 1)
            iy0 = np.clip(((lo[:, 1] - self.y0) / cell).astype(np.int64), 0, nyc - 1)
            iy1 = np.clip(((hi[:, 1] - self.y0) / cell).astype(np.int64), 0, nyc - 1)
            w = ix1 - ix0 + 1
            cnt = w * (iy1 - iy0 + 1)
            total = int(cnt.sum())
            if (nxc * nyc <= max_cells and total <= max_entries) or nxc * nyc == 1:
                break
            cell *= 1.5
        self.cell, self.nxc, self.nyc = cell, nxc, nyc
        tri_id = np.repeat(np.arange(m, dtype=np.int64), cnt)
        start = np.repeat(np.cumsum(cnt) - cnt, cnt)
        off = np.arange(total, dtype=np.int64) - start
        ww = np.repeat(w, cnt)
        cx = np.repeat(ix0, cnt) + off % ww
        cy = np.repeat(iy0, cnt) + off // ww
        cid = cy * nxc + cx
        order = np.argsort(cid, kind="stable")
        self.entries = tri_id[order]
        self.cell_start = np.searchsorted(cid[order], np.arange(nxc * nyc + 1, dtype=np.int64))

    def locate(self, qx: np.ndarray, qy: np.ndarray, eps: float = 1e-9,
               max_pairs: int = 4_000_000) -> tuple[np.ndarray, np.ndarray]:
        """Return (triangle index into ``self.tris`` or -1, barycentric (n, 3))."""
        qx = np.asarray(qx, dtype=np.float64).ravel()
        qy = np.asarray(qy, dtype=np.float64).ravel()
        n = len(qx)
        tri = np.full(n, -1, dtype=np.int64)
        bary = np.full((n, 3), np.nan)
        cxf_all = np.floor((qx - self.x0) / self.cell)
        cyf_all = np.floor((qy - self.y0) / self.cell)
        ok_all = (cxf_all >= 0) & (cxf_all < self.nxc) & (cyf_all >= 0) & (cyf_all < self.nyc)
        cid_all = np.where(ok_all, cyf_all * self.nxc + cxf_all, 0).astype(np.int64)
        cnt_all = np.where(ok_all, self.cell_start[cid_all + 1] - self.cell_start[cid_all], 0)
        csum = np.cumsum(cnt_all)
        bounds = [0]
        while bounds[-1] < n:  # chunks of at most max_pairs (query, triangle) candidate pairs
            base = csum[bounds[-1] - 1] if bounds[-1] > 0 else 0
            nxt = int(np.searchsorted(csum, base + max_pairs, side="right"))
            bounds.append(min(n, max(nxt, bounds[-1] + 1)))
        for c0, c1 in zip(bounds[:-1], bounds[1:]):
            sl = slice(c0, c1)
            x, y = qx[sl], qy[sl]
            s = self.cell_start[cid_all[sl]]
            cnt = cnt_all[sl]
            total = int(cnt.sum())
            if total == 0:
                continue
            qi = np.repeat(np.arange(len(x), dtype=np.int64), cnt)
            off = np.arange(total, dtype=np.int64) - np.repeat(np.cumsum(cnt) - cnt, cnt)
            ti = self.entries[np.repeat(s, cnt) + off]
            dx = x[qi] - self.p0[ti, 0]
            dy = y[qi] - self.p0[ti, 1]
            l1 = (dx * self.v1[ti, 1] - dy * self.v1[ti, 0]) / self.det[ti]
            l2 = (self.v0[ti, 0] * dy - self.v0[ti, 1] * dx) / self.det[ti]
            l0 = 1.0 - l1 - l2
            inside = (l0 >= -eps) & (l1 >= -eps) & (l2 >= -eps)
            if not inside.any():
                continue
            qin = qi[inside]
            uq, first = np.unique(qin, return_index=True)
            sel = np.flatnonzero(inside)[first]
            tri[c0 + uq] = ti[sel]
            bary[c0 + uq] = np.stack([l0[sel], l1[sel], l2[sel]], axis=1)
        return tri, bary

    def interpolate(self, values: np.ndarray, qx: np.ndarray, qy: np.ndarray) -> np.ndarray:
        """Linear interpolation of per-vertex ``values`` (NaN outside the TIN)."""
        tri, bary = self.locate(qx, qy)
        out = np.full(len(tri), np.nan)
        ok = tri >= 0
        vi = self.tris[tri[ok]]
        vals = np.asarray(values, dtype=np.float64)
        out[ok] = (bary[ok] * vals[vi]).sum(1)
        return out


class PiecewiseTinHeight:
    """Height of the union of non-overlapping ground pieces at (x, y).

    ``pieces``: sequence of (rect (x0, x1, y0, y1), loader) where ``loader()`` returns
    (vertices (n, 3), triangles (m, 3)); loaded lazily, at most ``cache_size`` locators are kept.
    ``fallback(x, y)``: heights for points no piece covers (None: NaN).
    """

    def __init__(self, pieces: Sequence[tuple[tuple[float, float, float, float], Callable]],
                 fallback: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None, cache_size: int = 24):
        self.rects = np.array([r for r, _ in pieces], dtype=np.float64).reshape(-1, 4)
        self.loaders = [ld for _, ld in pieces]
        self.fallback = fallback
        self.cache_size = int(cache_size)
        self._cache: OrderedDict[int, tuple[TinLocator, np.ndarray]] = OrderedDict()
        self.n_loads = 0

    def _locator(self, k: int) -> tuple[TinLocator, np.ndarray]:
        if k in self._cache:
            self._cache.move_to_end(k)
            return self._cache[k]
        V, T = self.loaders[k]()
        V = np.asarray(V, dtype=np.float64)
        item = (TinLocator(V[:, :2], T), V[:, 2].copy())
        self.n_loads += 1
        self._cache[k] = item
        while len(self._cache) > self.cache_size:
            self._cache.popitem(last=False)
        return item

    def __call__(self, x, y) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = np.broadcast_shapes(x.shape, y.shape)
        xf = np.broadcast_to(x, shape).ravel()
        yf = np.broadcast_to(y, shape).ravel()
        out = np.full(xf.size, np.nan)
        eps = 1e-9
        todo = np.ones(xf.size, dtype=bool)
        for k, (x0, x1, y0, y1) in enumerate(self.rects):
            if not todo.any():
                break
            m = todo & (xf >= x0 - eps) & (xf <= x1 + eps) & (yf >= y0 - eps) & (yf <= y1 + eps)
            if not m.any():
                continue
            loc, zv = self._locator(k)
            idx = np.flatnonzero(m)
            z = loc.interpolate(zv, xf[idx], yf[idx])
            good = np.isfinite(z)
            out[idx[good]] = z[good]
            todo[idx[good]] = False
        if todo.any() and self.fallback is not None:
            out[todo] = np.asarray(self.fallback(xf[todo], yf[todo]), dtype=np.float64).ravel()
        return out.reshape(shape)
