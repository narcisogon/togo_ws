"""Conforming maximum-edge-length refinement of grid TINs (longest-edge bisection + flips).

Why (docs/impl/gz_validation.md 5.1, docs/impl/fix_build.md): classic Bullet treats every mesh as
GImpact. The rover's CAD collision meshes (4 motors with 46.6 k triangles each about 10 cm above the
ground, a 174 k-triangle LiDAR body) are tested against the bounding box of every terrain triangle
they come near. The adaptive collision TIN gives smooth slopes large tilted triangles whose boxes
reach up to the motors, and GImpact then visits tens of thousands of motor triangles per 1 ms step
(RTF 0.05-0.2 on the slope site). Bounding the terrain triangles' edge length bounds the height of
their boxes on a slope (about ``max_edge * tan(slope)``).

Algorithm (``refine_max_edge``), vectorised in rounds:

1. Edges of the current TIN are numbered; an edge is *marked* when it is longer than ``max_edge_m``
   (and accepted by an optional ``select`` filter, e.g. a band along the piece boundary).
2. Closure (Rivara): every triangle with a marked edge also gets its LONGEST edge marked, until
   stable. Refined triangles are therefore bisected through their longest edge first, which keeps
   the angles bounded away from 0.
3. Every marked edge gets one new vertex at its midpoint SNAPPED TO A GRID NODE (the exact midpoint
   when it is a node; else one of the up to four surrounding nodes, the one closest to the edge line
   first). Both triangles sharing an edge use the same vertex, so the result is conforming (no
   T-junctions) within the piece. A boundary edge lies on one rectangle side, so its new vertex stays
   on that side: corners are untouched and boundary vertices stay on the boundary, as
   ``mesh.stitch`` requires.
4. Each triangle is split by the template of its marked sides (AB = longest side, M/P/Q = new
   vertices on AB/BC/CA)::

       AB          (A, M, C) (M, B, C)
       AB, BC      (A, M, C) (M, B, P) (M, P, C)
       AB, CA      (A, M, Q) (Q, M, C) (M, B, C)
       AB, BC, CA  (A, M, Q) (Q, M, C) (M, B, P) (M, P, C)
       BC          (A, B, P) (A, P, C)                       (AB could not be split)
       CA          (A, B, Q) (Q, B, C)
       BC, CA      (A, B, P) (A, P, Q) (Q, P, C)  or  (A, B, Q) (Q, B, P) (Q, P, C) (shorter diagonal)

   Every sub-triangle is checked with exact integer orientation tests in node space. A sub-triangle
   can only be inverted by a vertex snapped off its edge, so such a vertex (or one that would land on
   an existing vertex or on another new vertex) moves to its next snap candidate; an edge without a
   valid candidate is not split in this round.
5. Long edges that could not be split (thin slivers, e.g. one-node-wide strips along the rectangle
   side where no grid node lies strictly inside) are flipped when their quad is strictly convex and
   the other diagonal is shorter (Lawson flip; the change of the surface inside such a thin quad is
   negligible), and the next round splits the new, better-shaped triangles.
6. Repeat until no edge is marked (or no progress is possible, or ``max_rounds``).

All vertices are grid nodes and take the node height, so the refined mesh keeps the TIN contract
(vertices on nodes, exact heights, CCW, ``lattice`` filled, rectangle tiled). The interpolation
error can only change near new vertices or flipped edges; ``tin.tin_max_error`` reports it.
"""
from __future__ import annotations

import time
from typing import Callable

import numpy as np

from ..grid import Grid, from_lattice
from .types import Mesh

__all__ = ["refine_max_edge", "edge_lengths", "triangle_box_heights", "boundary_band_select"]

#: select(edge_a_ij, edge_b_ij, length2_nodes) -> bool mask of edges that may be refined
EdgeSelect = Callable[[np.ndarray, np.ndarray, np.ndarray], np.ndarray]

# split templates: (mAB, mBC, mCA, option) -> sub-triangles over the symbols A B C (old) M P Q (new)
_TEMPLATES: dict[tuple[int, int, int, int], tuple[tuple[str, str, str], ...]] = {
    (1, 0, 0, 0): (("A", "M", "C"), ("M", "B", "C")),
    (1, 1, 0, 0): (("A", "M", "C"), ("M", "B", "P"), ("M", "P", "C")),
    (1, 0, 1, 0): (("A", "M", "Q"), ("Q", "M", "C"), ("M", "B", "C")),
    (1, 1, 1, 0): (("A", "M", "Q"), ("Q", "M", "C"), ("M", "B", "P"), ("M", "P", "C")),
    (0, 1, 0, 0): (("A", "B", "P"), ("A", "P", "C")),
    (0, 0, 1, 0): (("A", "B", "Q"), ("Q", "B", "C")),
    (0, 1, 1, 0): (("A", "B", "P"), ("A", "P", "Q"), ("Q", "P", "C")),
    (0, 1, 1, 1): (("A", "B", "Q"), ("Q", "B", "P"), ("Q", "P", "C")),
}


def _node_coords(mesh: Mesh, grid: Grid) -> np.ndarray:
    if mesh.lattice is None:
        raise ValueError(f"{mesh.name}: refine_max_edge needs mesh.lattice")
    ox, oy = grid.lattice_origin()
    step = grid.lattice_step()
    lat = np.asarray(mesh.lattice, dtype=np.int64)
    di, ri = np.divmod(lat[:, 0] - ox, step)
    dj, rj = np.divmod(lat[:, 1] - oy, step)
    if np.any(ri) or np.any(rj):
        raise ValueError(f"{mesh.name}: vertices are not nodes of the grid")
    if di.min() < 0 or dj.min() < 0 or di.max() > grid.nx - 1 or dj.max() > grid.ny - 1:
        raise ValueError(f"{mesh.name}: vertices outside the grid")
    return np.column_stack([di, dj])


def _edges(T: np.ndarray, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(eid (m,3) edge id of side k = (v_k, v_k+1), ea, eb) over the unique undirected edges."""
    b = T[:, [1, 2, 0]]
    lo = np.minimum(T, b)
    hi = np.maximum(T, b)
    key = lo * np.int64(n) + hi
    uk, inv = np.unique(key.ravel(), return_inverse=True)
    return np.asarray(inv).reshape(-1, 3), uk // n, uk % n


def _cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """z of (a - o) x (b - o) for integer points (k, 2)."""
    return (a[:, 0] - o[:, 0]) * (b[:, 1] - o[:, 1]) - (a[:, 1] - o[:, 1]) * (b[:, 0] - o[:, 0])


def _candidates(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """(k, 4, 2) grid-node snap candidates of the midpoints of edges AB, best first; -1 = none.

    Candidates are the nodes around the exact midpoint (1 if it is a node, 2 if it is exact in one
    axis, else 4), ordered by distance from the edge line, then by position (deterministic).
    """
    S = A + B
    lo = np.floor_divide(S, 2)
    hi = lo + (S % 2)
    cand = np.stack([np.column_stack([lo[:, 0], lo[:, 1]]), np.column_stack([hi[:, 0], lo[:, 1]]),
                     np.column_stack([lo[:, 0], hi[:, 1]]), np.column_stack([hi[:, 0], hi[:, 1]])], axis=1)
    d = B - A
    off = np.abs(d[:, None, 0] * (cand[:, :, 1] - A[:, None, 1]) - d[:, None, 1] * (cand[:, :, 0] - A[:, None, 0]))
    dup = np.zeros(off.shape, dtype=bool)
    for k in range(1, 4):
        for j in range(k):
            dup[:, k] |= np.all(cand[:, k] == cand[:, j], axis=1)
    big = np.iinfo(np.int64).max // 4
    order = np.argsort(np.where(dup, big, off), axis=1, kind="stable")
    cand = np.take_along_axis(cand, order[:, :, None], axis=1)
    valid = np.take_along_axis(~dup, order, axis=1)
    cand[~valid] = -1
    return cand


class _Round:
    """Per-round topology: edges, lengths, the longest side of every triangle (AB, BC, CA)."""

    def __init__(self, P: np.ndarray, T: np.ndarray):
        self.P, self.T = P, T
        self.m = T.shape[0]
        self.eid, self.ea, self.eb = _edges(T, P.shape[0])
        d = P[self.ea] - P[self.eb]
        self.len2 = d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1]
        ne = self.len2.size
        rank = self.len2[self.eid] * np.int64(ne + 1) + (np.int64(ne) - self.eid)
        self.kmax = np.argmax(rank, axis=1)
        rows = np.arange(self.m)
        self.sides = (self.kmax[:, None] + np.arange(3)[None, :]) % 3
        self.e = [self.eid[rows, self.sides[:, k]] for k in range(3)]     # AB, BC, CA edge ids
        self.v = [T[rows, self.sides[:, k]] for k in range(3)]            # A, B, C vertex ids

    def closure(self, marked: np.ndarray, forbidden: np.ndarray) -> None:
        """Mark the longest side of every triangle that has a marked side (unless forbidden)."""
        eab = self.e[0]
        while True:
            need = (marked[self.e[0]] | marked[self.e[1]] | marked[self.e[2]]) & ~marked[eab] & ~forbidden[eab]
            if not need.any():
                return
            marked[eab[need]] = True

    def cases(self, marked: np.ndarray, Q: np.ndarray | None, pos_of: np.ndarray) -> np.ndarray:
        """Template code per triangle: mAB*8 + mBC*4 + mCA*2 + option (-1 = not split)."""
        mab, mbc, mca = (marked[self.e[k]] for k in range(3))
        code = mab.astype(np.int64) * 8 + mbc * 4 + mca * 2
        both = ~mab & mbc & mca
        if both.any():
            t = np.flatnonzero(both)
            A = self.P[self.v[0][t]]
            B = self.P[self.v[1][t]]
            Pn = Q[pos_of[self.e[1][t]]]
            Qn = Q[pos_of[self.e[2][t]]]
            dAP = np.sum((Pn - A) ** 2, axis=1)
            dBQ = np.sum((Qn - B) ** 2, axis=1)
            code[t] += (dBQ < dAP).astype(np.int64)
        code[code == 0] = -1
        return code

    def subtriangles(self, marked, Q, pos_of, new_id=None, X2=None):
        """Yield (triangle indices, list of sub-triangles as (sym, sym, sym)) per template.

        Each symbol resolves to (doubled positions (k, 2), vertex ids (k,) or None, index into the
        marked-edge list or None, doubled exact midpoints (k, 2) or None). Doubled coordinates keep
        the exact midpoints of new vertices integral."""
        code = self.cases(marked, Q, pos_of)
        for (mab, mbc, mca, opt), subs in _TEMPLATES.items():
            c = mab * 8 + mbc * 4 + mca * 2 + opt
            t = np.flatnonzero(code == c)
            if t.size == 0:
                continue
            sym = {"A": (2 * self.P[self.v[0][t]], self.v[0][t], None, None),
                   "B": (2 * self.P[self.v[1][t]], self.v[1][t], None, None),
                   "C": (2 * self.P[self.v[2][t]], self.v[2][t], None, None)}
            for s, k in (("M", 0), ("P", 1), ("Q", 2)):
                if (s == "M" and mab) or (s == "P" and mbc) or (s == "Q" and mca):
                    pos = pos_of[self.e[k][t]]
                    ids = new_id[self.e[k][t]] if new_id is not None else None
                    sym[s] = (2 * Q[pos], ids, pos, X2[pos] if X2 is not None else None)
            yield t, [(sym[a], sym[b], sym[c]) for a, b, c in subs]


def _validity(rd: _Round, marked, Q, pos_of, X2) -> np.ndarray:
    """Positions (into the marked-edge list) of new vertices that make a sub-triangle invalid.

    Exact midpoints always give valid sub-triangles. A new vertex is blamed for an inverted
    sub-triangle when the sub-triangle is still inverted with that vertex snapped and the other new
    vertices at their exact midpoints; if no single vertex is to blame, all snapped ones are."""
    blame = []
    for _t, subs in rd.subtriangles(marked, Q, pos_of, X2=X2):
        for tri in subs:
            bad = _cross(tri[0][0], tri[1][0], tri[2][0]) <= 0
            if not bad.any():
                continue
            news = [k for k in range(3) if tri[k][2] is not None]
            if not news:
                continue
            found = np.zeros(bad.shape, dtype=bool)
            for k in news:
                pts = [tri[j][0] if (j == k or tri[j][2] is None) else tri[j][3] for j in range(3)]
                culprit = bad & (_cross(pts[0], pts[1], pts[2]) <= 0)
                if culprit.any():
                    blame.append(tri[k][2][culprit])
                    found |= culprit
            rest = bad & ~found
            if rest.any():
                for k in news:
                    snapped = np.any(tri[k][0] != tri[k][3], axis=1)
                    blame.append(tri[k][2][rest & snapped])
    if not blame:
        return np.zeros(0, dtype=np.int64)
    return np.unique(np.concatenate(blame))


def _split(rd: _Round, marked, Q, pos_of, new_id) -> np.ndarray:
    parts = []
    split = np.zeros(rd.m, dtype=bool)
    for t, subs in rd.subtriangles(marked, Q, pos_of, new_id=new_id):
        split[t] = True
        for a, b, c in subs:
            parts.append(np.column_stack([a[1], b[1], c[1]]))
    return np.vstack([rd.T[~split]] + parts).astype(np.int64)


def _flip(P: np.ndarray, T: np.ndarray, pairs: np.ndarray) -> tuple[np.ndarray, int]:
    """Flip the edges given as vertex pairs (k, 2) where their quad is strictly convex and the other
    diagonal is shorter; one flip per triangle per call (longest edges first)."""
    if pairs.size == 0:
        return T, 0
    n = P.shape[0]
    eid, ea, eb = _edges(T, n)
    want_key = np.minimum(pairs[:, 0], pairs[:, 1]) * np.int64(n) + np.maximum(pairs[:, 0], pairs[:, 1])
    ekey = ea * np.int64(n) + eb
    sel = np.flatnonzero(np.isin(ekey, want_key))
    if sel.size == 0:
        return T, 0
    flat_e = eid.ravel()
    flat_t = np.repeat(np.arange(T.shape[0]), 3)
    flat_k = np.tile(np.arange(3), T.shape[0])
    keep = np.isin(flat_e, sel)
    fe, ft, fk = flat_e[keep], flat_t[keep], flat_k[keep]
    o = np.argsort(fe, kind="stable")
    fe, ft, fk = fe[o], ft[o], fk[o]
    first = np.r_[True, fe[1:] != fe[:-1]]
    starts = np.flatnonzero(first)
    counts = np.diff(np.r_[starts, fe.size])
    two = starts[counts == 2]                         # interior edges only
    if two.size == 0:
        return T, 0
    e = fe[two]
    t1, k1 = ft[two], fk[two]
    t2, k2 = ft[two + 1], fk[two + 1]
    a = T[t1, k1]
    b = T[t1, (k1 + 1) % 3]
    c1 = T[t1, (k1 + 2) % 3]
    c2 = T[t2, (k2 + 2) % 3]
    Pa, Pb, P1, P2 = P[a], P[b], P[c1], P[c2]
    ok = (_cross(Pa, P2, P1) > 0) & (_cross(P2, Pb, P1) > 0)
    d_old = np.sum((Pa - Pb) ** 2, axis=1)
    d_new = np.sum((P1 - P2) ** 2, axis=1)
    ok &= d_new < d_old
    idx = np.flatnonzero(ok)
    if idx.size == 0:
        return T, 0
    idx = idx[np.argsort(-d_old[idx], kind="stable")]
    used = np.zeros(T.shape[0], dtype=bool)
    T = T.copy()
    nf = 0
    for i in idx.tolist():                            # greedy independent set (usually small)
        if used[t1[i]] or used[t2[i]]:
            continue
        used[t1[i]] = used[t2[i]] = True
        T[t1[i]] = (a[i], c2[i], c1[i])
        T[t2[i]] = (c2[i], b[i], c1[i])
        nf += 1
    del e
    return T, nf


def refine_max_edge(mesh: Mesh, grid: Grid, max_edge_m: float, *, select: EdgeSelect | None = None,
                    max_rounds: int = 64, name: str | None = None) -> tuple[Mesh, dict]:
    """Refine a grid TIN until no (selected) edge is longer than ``max_edge_m``.

    Args:
        mesh: TIN whose vertices are nodes of ``grid`` (``mesh.lattice`` filled), e.g. from
            ``tin_from_grid(grid, ...)``; it must tile the grid rectangle.
        grid: the height grid the TIN was made from (new vertices take its node heights).
        max_edge_m: maximum horizontal edge length in metres (> 2 grid spacings).
        select: optional filter ``select(a_ij, b_ij, len2_nodes) -> bool`` restricting which long
            edges are refined (closure may still split the longest edge of an affected triangle).
        max_rounds: safety bound on the number of rounds.

    Returns:
        (refined Mesh with vertices / triangles / lattice, report dict). Normals and UVs are not
        carried over (recompute them, e.g. ``mesh.normals.assign_normals``); ``meta`` is copied and
        gets the report under ``refine``.
    """
    t0 = time.perf_counter()
    name = name or mesh.name
    sp = float(grid.spacing)
    if not max_edge_m > 2.0 * sp:
        raise ValueError(f"{name}: max_edge_m {max_edge_m} must be > 2 grid spacings ({2 * sp})")
    P = _node_coords(mesh, grid)
    T = np.asarray(mesh.triangles, dtype=np.int64).copy()
    L2 = (float(max_edge_m) / sp) ** 2
    n_tri0, n_ver0 = int(T.shape[0]), int(P.shape[0])
    stats = {"rounds": 0, "split_edges": 0, "unsplit_events": 0, "flips": 0, "snapped_off_midpoint": 0}
    key_mult = np.int64(max(grid.nx, grid.ny) + 1)

    def long_mask(rd: _Round) -> np.ndarray:
        mk = rd.len2 > L2
        if select is not None and mk.any():
            idx = np.flatnonzero(mk)
            ok = np.asarray(select(P[rd.ea[idx]], P[rd.eb[idx]], rd.len2[idx]), dtype=bool)
            mk[idx[~ok]] = False
        return mk

    for _ in range(int(max_rounds)):
        rd = _Round(P, T)
        marked = long_mask(rd)
        if not marked.any():
            break
        ne = rd.len2.size
        forbidden = np.zeros(ne, dtype=bool)
        pkey = P[:, 0] * key_mult + P[:, 1]
        Q = pos_of = None
        while True:
            rd.closure(marked, forbidden)
            me = np.flatnonzero(marked)
            if me.size == 0:
                break
            pos_of = np.full(ne, -1, dtype=np.int64)
            pos_of[me] = np.arange(me.size)
            Ae, Be = P[rd.ea[me]], P[rd.eb[me]]
            X2 = Ae + Be                                       # exact midpoints, doubled
            C = _candidates(Ae, Be)
            ch = np.zeros(me.size, dtype=np.int64)
            rows = np.arange(me.size)
            for _attempt in range(8):
                Q = C[rows, np.minimum(ch, 3)]
                exhausted = (ch > 3) | (Q[:, 0] < 0)
                snapped = ~np.all(2 * Q == X2, axis=1)
                qkey = np.where(exhausted, -1 - rows, Q[:, 0] * key_mult + Q[:, 1])
                bad = exhausted | np.isin(qkey, pkey)
                o = np.lexsort((rows, snapped, qkey))          # one new vertex per node, exact first
                same = np.r_[False, qkey[o][1:] == qkey[o][:-1]]
                bad[o[same]] = True
                bad[_validity(rd, marked, np.where(exhausted[:, None], Ae, Q), pos_of, X2)] = True
                if not bad.any():
                    break
                ch[bad & ~exhausted] += 1
            if not bad.any():
                break
            marked[me[bad]] = False                            # no valid vertex: not split this round
            forbidden[me[bad]] = True
            stats["unsplit_events"] += int(bad.sum())
        me = np.flatnonzero(marked)
        n_split = int(me.size)
        if n_split:
            stats["snapped_off_midpoint"] += int((~np.all(2 * Q == P[rd.ea[me]] + P[rd.eb[me]], axis=1)).sum())
            new_id = np.full(ne, -1, dtype=np.int64)
            new_id[me] = P.shape[0] + np.arange(me.size)
            P = np.vstack([P, Q])
            T = _split(rd, marked, Q, pos_of, new_id)
            stats["split_edges"] += n_split
        n_flip = 0
        if forbidden.any():
            # long edges that could not be split: flip them where that shortens them
            fb = np.flatnonzero(forbidden & (rd.len2 > L2))
            if fb.size:
                T, n_flip = _flip(P, T, np.column_stack([rd.ea[fb], rd.eb[fb]]))
                stats["flips"] += n_flip
        stats["rounds"] += 1
        if n_split == 0 and n_flip == 0:
            break
    rd = _Round(P, T)
    over = long_mask(rd)
    n = P.shape[0]
    ox, oy = grid.lattice_origin()
    step = grid.lattice_step()
    lat = np.column_stack([ox + P[:, 0] * step, oy + P[:, 1] * step]).astype(np.int64)
    V = np.empty((n, 3), dtype=np.float64)
    V[:, :2] = from_lattice(lat)
    V[:, 2] = np.asarray(grid.z, dtype=np.float64)[P[:, 1], P[:, 0]]
    V[:n_ver0, 2] = mesh.vertices[:, 2]                 # input heights stay bit-identical
    out = Mesh(V, T, lattice=lat, name=name, meta=dict(mesh.meta))
    _check_refined(out, P, grid, name)
    rep = dict(stats, max_edge_m=float(max_edge_m), triangles_before=n_tri0, triangles_after=int(T.shape[0]),
               vertices_before=n_ver0, vertices_after=int(n),
               max_edge_after_m=float(np.sqrt(rd.len2.max()) * sp) if rd.len2.size else 0.0,
               edges_over_limit_after=int(over.sum()), time_s=round(time.perf_counter() - t0, 4))
    out.meta["refine"] = rep
    return out, rep


def _check_refined(mesh: Mesh, P: np.ndarray, grid: Grid, name: str) -> None:
    T = mesh.triangles
    key = P[:, 1] * np.int64(grid.nx) + P[:, 0]
    if np.unique(key).size != key.size:
        raise RuntimeError(f"{name}: refinement produced duplicate vertices")
    area2 = _cross(P[T[:, 0]], P[T[:, 1]], P[T[:, 2]])
    if np.any(area2 <= 0):
        raise RuntimeError(f"{name}: refinement produced {int((area2 <= 0).sum())} non-CCW triangles")
    if int(area2.sum()) != 2 * (grid.nx - 1) * (grid.ny - 1):
        raise RuntimeError(f"{name}: refined TIN does not tile the grid rectangle")
    used = np.zeros(P.shape[0], dtype=bool)
    used[T.ravel()] = True
    if not used.all():
        raise RuntimeError(f"{name}: refinement left {int((~used).sum())} unused vertices")


def boundary_band_select(grid: Grid, band_m: float) -> EdgeSelect:
    """``select`` for :func:`refine_max_edge`: edges with an endpoint within ``band_m`` of the grid
    rectangle's boundary (e.g. to refine a visual tile only along its seams)."""
    b = float(band_m) / float(grid.spacing)
    nx1, ny1 = grid.nx - 1, grid.ny - 1

    def near(p: np.ndarray) -> np.ndarray:
        return np.minimum(np.minimum(p[:, 0], nx1 - p[:, 0]), np.minimum(p[:, 1], ny1 - p[:, 1])) <= b

    return lambda a, c, _l2: near(a) | near(c)


def edge_lengths(mesh: Mesh) -> np.ndarray:
    """Horizontal length of every undirected edge [m]."""
    _, ea, eb = _edges(np.asarray(mesh.triangles, dtype=np.int64), mesh.n_vertices)
    d = mesh.vertices[ea, :2] - mesh.vertices[eb, :2]
    return np.hypot(d[:, 0], d[:, 1])


def triangle_box_heights(mesh: Mesh) -> np.ndarray:
    """Vertical extent of every triangle's axis-aligned bounding box [m] (GImpact box height)."""
    z = mesh.vertices[:, 2][mesh.triangles]
    return z.max(axis=1) - z.min(axis=1)
