"""Crack-free stitching of adjacent ground pieces (docs/DESIGN.md section 6).

Invariant: for every pair of adjacent pieces (tile-tile, tile-ring strip, strip-strip, ring-far,
far-horizon, also collision pieces) the set of vertices on the shared boundary segment is
identical in both pieces - compared through integer 1 cm lattice coordinates - and so are their
heights (bit-identical float64, hence also float32) and their normals. In Gazebo, coincident
shared vertices leaked 0 LiDAR rays while T-junctions leaked 0.02-0.2 % (gazebo report 5b).

Algorithm (``stitch_pieces``):

1. ``find_contacts``: every pair of pieces whose closed lattice rectangles touch along a segment
   of positive length (``Contact.kind == 'segment'``) or in a single corner point (``'point'``).
   Overlapping rectangles are an error.
2. For each segment contact, the union of both pieces' boundary vertices on the segment is
   computed; every point a piece lacks is inserted into it by fan-splitting the boundary
   triangle whose edge contains the point: triangle (a, b, c) with boundary edge a->b and
   points q1..qm in order becomes (a, q1, c), (q1, q2, c), ..., (qm, b, c), which keeps the CCW
   winding. Segment endpoints are always rectangle corners of one of the two pieces, and
   pydelatin TINs always contain their corners, so after this pass every piece whose closed
   rectangle contains a shared boundary point has a vertex there (3- and 4-piece corners
   included).
3. Canonical values: for every shared lattice point the canonical piece is the one with the
   LOWEST ``priority`` among the pieces containing it (``tile_priority`` / ``strip_priority``:
   inner zone first, then the lower core-tile index j * n + i, then the strip side order
   south, north, west, east). Its height (and normal) is copied into every other piece's vertex
   at that point, overwriting the outer pieces' own boundary heights. If the canonical piece
   itself lacked the point, its new vertex gets ``height_fn(x, y)`` (``normal_fn(x, y)``) when the
   piece has one, else the linear interpolation along the split edge (the surface is unchanged).
   Inserted vertices of non-canonical pieces receive the canonical height and normal; their UVs
   come from ``uv_tile_m`` (planar) when set, else from the linear interpolation along the edge.

``check_stitched`` proves the invariant independently (0 mismatches, 0 T-junctions) and runs a
topology check of the union: every interior edge shared by exactly two triangles with opposite
orientation, no crack along any shared segment, and the Euler characteristic of the union.

The pieces' meshes are modified in place. Stitch BEFORE writing the GLBs; compute normals and
UVs of each piece from its own grid BEFORE stitching (then shared vertices take the canonical
piece's normals), or recompute UVs afterwards - they only depend on x, y.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from ..grid import Grid, from_lattice, to_lattice
from ..layout import RING_SIDES, Rect, TileId
from .types import Mesh

__all__ = [
    "Piece",
    "Contact",
    "ZONE_RANK",
    "tile_priority",
    "strip_priority",
    "make_piece",
    "find_contacts",
    "stitch_pieces",
    "check_stitched",
    "union_topology",
]

HeightFn = Callable[[np.ndarray, np.ndarray], np.ndarray]
NormalFn = Callable[[np.ndarray, np.ndarray], np.ndarray]

_W, _E, _S, _N = 1, 2, 4, 8
ZONE_RANK = {"core": 0, "ring": 1, "far": 2, "horizon": 3}
_ZONE_STRIDE = 1_000_000
_KEY_OFFSET = 1 << 30  # lattice coordinates are within +-1e6 (20 km at 1 cm)


def tile_priority(t: TileId, n_tiles: int) -> int:
    """Core tiles: lower tile index (j * n + i, the ``Layout.tiles()`` order) wins."""
    return int(t.j) * int(n_tiles) + int(t.i)


def strip_priority(zone: str, side: str) -> int:
    """Outer-zone strips: inner zone wins; within a zone south, north, west, east."""
    return ZONE_RANK[zone] * _ZONE_STRIDE + RING_SIDES.index(side)


@dataclass
class Piece:
    """One ground mesh and the rectangle it covers.

    Attributes:
        mesh: TIN covering ``rect`` exactly, with ``lattice`` filled (``tin_from_grid``).
        rect: covered rectangle (node-inclusive); its corners must be mesh vertices.
        priority: lower wins (see ``tile_priority`` / ``strip_priority``).
        height_fn: H(x, y) in metres for vertices inserted into this piece when it is canonical
            (e.g. ``grid.sample`` of its own height grid; exact at nodes).
        normal_fn: unit normals (n, 3) at (x, y) for such vertices (e.g. a ``GridNormalField``).
        uv_tile_m: planar UV repeat length used for inserted vertices when the mesh has UVs.
        name: for reports (defaults to ``mesh.name``).
    """

    mesh: Mesh
    rect: Rect
    priority: int
    height_fn: HeightFn | None = None
    normal_fn: NormalFn | None = None
    uv_tile_m: float | None = None
    name: str = ""

    def __post_init__(self) -> None:
        if self.mesh.lattice is None:
            raise ValueError(f"piece {self.name or self.mesh.name}: mesh.lattice is required")
        if not self.name:
            self.name = self.mesh.name

    @property
    def lrect(self) -> tuple[int, int, int, int]:
        """(x0, x1, y0, y1) in integer lattice units."""
        r = self.rect
        return (int(to_lattice(r.x0)), int(to_lattice(r.x1)), int(to_lattice(r.y0)), int(to_lattice(r.y1)))


class _GridHeight:
    """height_fn of a zone grid: bilinear sample (exact at nodes). Holds only a reference."""

    def __init__(self, grid: Grid):
        self.grid = grid

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return self.grid.sample(np.asarray(x, np.float64), np.asarray(y, np.float64), order=1)


class _LazyNormals:
    """normal_fn of a zone grid window: builds a one-level normal field only when called."""

    def __init__(self, grid: Grid, rect: Rect, margin: float, sigma_nodes: float):
        self.grid, self.rect, self.margin, self.sigma = grid, rect, margin, sigma_nodes

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        from .normals import GridNormalField

        r, m = self.rect, self.margin
        win = self.grid.crop(r.x0 - m, r.x1 + m, r.y0 - m, r.y1 + m)
        return GridNormalField(win, (self.sigma,)).normals(x, y)


def make_piece(zone_grid: Grid, rect: Rect, max_error: float, priority: int, *, name: str,
               uv_tile_m: float | None = None, normals: bool = True, normal_margin_m: float = 0.0,
               adaptive_normals: bool = True, max_triangles: int | None = None,
               max_edge_m: float | None = None, max_edge_band_m: float | None = None) -> Piece:
    """Crop ``rect`` from a zone grid, TIN it and wrap it as a stitchable ``Piece``.

    The rectangle edges must be grid node lines (the TIN must cover the rectangle exactly).
    Normals come from the zone grid over ``rect`` grown by ``normal_margin_m`` (so boundary
    vertices get true central differences; ``cfg.zones.core.margin_m`` for core tiles), UVs are
    world-planar with ``uv_tile_m``. ``height_fn`` / ``normal_fn`` reference the zone grid
    lazily (no per-piece copies), so ``zone_grid`` may be a memory map. Collision pieces use
    ``normals=False, uv_tile_m=None``. ``max_triangles`` caps the TIN (pydelatin stops refining
    there; the error bound then no longer holds and the achieved error is in ``mesh.meta['tin_error']``),
    used by the integration build to enforce the per-tile triangle budget (DESIGN 13a).

    ``max_edge_m`` refines the TIN until no edge is longer (``mesh.refine.refine_max_edge``:
    conforming longest-edge bisection with grid-node vertices; the collision tiles use it to keep
    GImpact bounding boxes thin on slopes). ``max_edge_band_m`` restricts that refinement to edges
    within this distance of the rectangle boundary (e.g. against crease lines along tile seams).
    The report is in ``mesh.meta['refine']``.
    """
    from .normals import DEFAULT_LEVELS, GridNormalField, assign_normals
    from .tin import tin_from_grid
    from .uv import assign_uvs

    g = zone_grid.crop(rect.x0, rect.x1, rect.y0, rect.y1)
    if (abs(g.x0 - rect.x0) > 1e-9 or abs(g.x1 - rect.x1) > 1e-9 or abs(g.y0 - rect.y0) > 1e-9
            or abs(g.y1 - rect.y1) > 1e-9):
        raise ValueError(f"{name}: rectangle {rect} is not aligned with the grid nodes "
                         f"(crop gives {g.extent})")
    mesh = tin_from_grid(g, max_error, name, max_triangles=max_triangles)
    if max_edge_m:
        from .refine import boundary_band_select, refine_max_edge

        sel = boundary_band_select(g, max_edge_band_m) if max_edge_band_m else None
        mesh, _rep = refine_max_edge(mesh, g, float(max_edge_m), select=sel, name=name)
    del g
    normal_fn = None
    if normals:
        m = float(normal_margin_m)
        win = zone_grid.crop(rect.x0 - m, rect.x1 + m, rect.y0 - m, rect.y1 + m)
        field = GridNormalField(win, DEFAULT_LEVELS if adaptive_normals else (1.0,))
        assign_normals(mesh, field, adaptive=adaptive_normals)
        del field, win
        normal_fn = _LazyNormals(zone_grid, rect, m, 1.0)
    if uv_tile_m:
        assign_uvs(mesh, uv_tile_m)
    return Piece(mesh, rect, priority, height_fn=_GridHeight(zone_grid), normal_fn=normal_fn,
                 uv_tile_m=uv_tile_m, name=name)


@dataclass(frozen=True)
class Contact:
    """Two pieces touching along a segment or in one point (lattice units)."""

    a: int
    b: int
    kind: str  # "segment" | "point"
    axis: int  # segment direction: 0 = along x (y = const), 1 = along y (x = const); -1 for points
    const: int  # the fixed coordinate of the segment line (point: x)
    lo: int  # segment start along its direction (point: y)
    hi: int


@dataclass
class _Boundary:
    idx: np.ndarray  # vertex indices of boundary vertices
    lat: np.ndarray  # their lattice coordinates (k, 2)
    mask: np.ndarray  # side bits per boundary vertex


# ---------------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------------
def _side_mask(lat: np.ndarray, lrect: tuple[int, int, int, int]) -> np.ndarray:
    x0, x1, y0, y1 = lrect
    m = np.zeros(lat.shape[0], dtype=np.uint8)
    m |= np.where(lat[:, 0] == x0, _W, 0).astype(np.uint8)
    m |= np.where(lat[:, 0] == x1, _E, 0).astype(np.uint8)
    m |= np.where(lat[:, 1] == y0, _S, 0).astype(np.uint8)
    m |= np.where(lat[:, 1] == y1, _N, 0).astype(np.uint8)
    return m


def _boundary(p: Piece) -> _Boundary:
    lat = p.mesh.lattice
    m = _side_mask(lat, p.lrect)
    idx = np.nonzero(m)[0]
    return _Boundary(idx, lat[idx], m[idx])


def _keys(lat: np.ndarray) -> np.ndarray:
    lat = np.asarray(lat, dtype=np.int64).reshape(-1, 2)
    return (lat[:, 0] + _KEY_OFFSET) * (2 * _KEY_OFFSET) + (lat[:, 1] + _KEY_OFFSET)


def _on_segment(b: _Boundary, c: Contact) -> tuple[np.ndarray, np.ndarray]:
    """(along-coordinates, vertex indices) of boundary vertices on a segment contact, sorted."""
    if c.axis == 0:  # y = const, along x
        sel = (b.lat[:, 1] == c.const) & (b.lat[:, 0] >= c.lo) & (b.lat[:, 0] <= c.hi)
        t = b.lat[sel, 0]
    else:
        sel = (b.lat[:, 0] == c.const) & (b.lat[:, 1] >= c.lo) & (b.lat[:, 1] <= c.hi)
        t = b.lat[sel, 1]
    v = b.idx[sel]
    order = np.argsort(t, kind="stable")
    return t[order], v[order]


def _validate_piece(p: Piece) -> None:
    m = p.mesh
    lat = m.lattice
    if lat.shape != (m.n_vertices, 2):
        raise ValueError(f"{p.name}: lattice shape {lat.shape} != ({m.n_vertices}, 2)")
    x0, x1, y0, y1 = p.lrect
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f"{p.name}: degenerate rectangle {p.rect}")
    if lat[:, 0].min() < x0 or lat[:, 0].max() > x1 or lat[:, 1].min() < y0 or lat[:, 1].max() > y1:
        raise ValueError(f"{p.name}: mesh vertices outside its rectangle")
    keys = _keys(lat)
    corners = _keys(np.array([[x0, y0], [x1, y0], [x0, y1], [x1, y1]]))
    if not np.isin(corners, keys).all():
        raise ValueError(f"{p.name}: a rectangle corner is not a mesh vertex")


def find_contacts(pieces: Sequence[Piece]) -> list[Contact]:
    """All touching pairs (segment or corner point). Raises if two rectangles overlap."""
    rects = np.array([p.lrect for p in pieces], dtype=np.int64).reshape(-1, 4)
    out: list[Contact] = []
    n = len(pieces)
    for a in range(n):
        ax0, ax1, ay0, ay1 = rects[a]
        bx0, bx1, by0, by1 = rects[a + 1:].T
        ix0 = np.maximum(ax0, bx0)
        ix1 = np.minimum(ax1, bx1)
        iy0 = np.maximum(ay0, by0)
        iy1 = np.minimum(ay1, by1)
        touch = (ix0 <= ix1) & (iy0 <= iy1)
        for k in np.nonzero(touch)[0]:
            b = a + 1 + int(k)
            wx = int(ix1[k] - ix0[k])
            wy = int(iy1[k] - iy0[k])
            if wx > 0 and wy > 0:
                raise ValueError(f"pieces {pieces[a].name} and {pieces[b].name} overlap")
            if wx > 0:
                out.append(Contact(a, b, "segment", 0, int(iy0[k]), int(ix0[k]), int(ix1[k])))
            elif wy > 0:
                out.append(Contact(a, b, "segment", 1, int(ix0[k]), int(iy0[k]), int(iy1[k])))
            else:
                out.append(Contact(a, b, "point", -1, int(ix0[k]), int(iy0[k]), int(iy0[k])))
    return out


def _canonical_mask(points: np.ndarray, own: int, pieces: Sequence[Piece]) -> np.ndarray:
    """True for points where piece ``own`` has the lowest priority of all containing pieces."""
    if points.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    pr = pieces[own].priority
    canon = np.ones(points.shape[0], dtype=bool)
    for q, p in enumerate(pieces):
        if q == own or (p.priority, q) > (pr, own):
            continue
        x0, x1, y0, y1 = p.lrect
        inside = (points[:, 0] >= x0) & (points[:, 0] <= x1) & (points[:, 1] >= y0) & (points[:, 1] <= y1)
        canon &= ~inside
    return canon


# ---------------------------------------------------------------------------------------------
# insertion (fan split of boundary triangles)
# ---------------------------------------------------------------------------------------------
def _boundary_edges(mesh: Mesh, lrect: tuple[int, int, int, int]):
    """Directed boundary edges (edge id = 3 * triangle + local index) with their side and coords."""
    lat = mesh.lattice
    mask = _side_mask(lat, lrect)
    tri = mesh.triangles
    ea = tri.reshape(-1)
    eb = tri[:, [1, 2, 0]].reshape(-1)
    common = mask[ea] & mask[eb]
    eid = np.nonzero(common)[0]
    side = common[eid]
    along_axis = np.where((side == _W) | (side == _E), 1, 0)
    ta = lat[ea[eid], along_axis]
    tb = lat[eb[eid], along_axis]
    return eid, side, ta, tb


def _insert_points(p: Piece, pts: np.ndarray, canonical: np.ndarray) -> dict:
    """Insert boundary lattice points ``pts`` (k, 2) into piece ``p`` by fan splits."""
    mesh = p.mesh
    lrect = p.lrect
    pts = np.asarray(pts, dtype=np.int64).reshape(-1, 2)
    if pts.shape[0] == 0:
        return {"inserted": 0, "rounds": 0}
    pmask = _side_mask(pts, lrect)
    if np.any(pmask == 0):
        raise ValueError(f"{p.name}: point to insert is not on the rectangle boundary")
    if np.any((pmask & (pmask - 1)) != 0):
        raise ValueError(f"{p.name}: point to insert is a rectangle corner (corners must exist)")
    existing = np.isin(_keys(pts), _keys(mesh.lattice))
    if np.any(existing):
        raise ValueError(f"{p.name}: point to insert already exists")

    n0 = mesh.n_vertices
    k = pts.shape[0]
    new_ids = n0 + np.arange(k, dtype=np.int64)
    # append the new vertices (z / normals / uvs filled in below, per split edge)
    xy = from_lattice(pts)
    mesh.vertices = np.vstack([mesh.vertices, np.column_stack([xy, np.zeros(k)])])
    mesh.lattice = np.vstack([mesh.lattice, pts])
    has_n = mesh.normals is not None
    has_uv = mesh.uvs is not None
    if has_n:
        mesh.normals = np.vstack([mesh.normals, np.zeros((k, 3), dtype=np.float32)])
    if has_uv:
        mesh.uvs = np.vstack([mesh.uvs, np.zeros((k, 2), dtype=np.float32)])

    # canonical heights / normals of all points at once (height_fn / normal_fn may be expensive)
    canonical = np.asarray(canonical, dtype=bool)
    use_h = p.height_fn is not None and bool(np.any(canonical))
    use_n = has_n and p.normal_fn is not None and bool(np.any(canonical))
    z_fn = np.full(k, np.nan)
    n_fn = np.full((k, 3), np.nan)
    if use_h:
        z_fn[canonical] = np.asarray(p.height_fn(xy[canonical, 0], xy[canonical, 1]), dtype=np.float64).ravel()
    if use_n:
        n_fn[canonical] = np.asarray(p.normal_fn(xy[canonical, 0], xy[canonical, 1]),
                                     dtype=np.float64).reshape(-1, 3)

    along = np.where((pmask == _W) | (pmask == _E), pts[:, 1], pts[:, 0])
    todo = np.arange(k)
    rounds = 0
    while todo.size:
        rounds += 1
        if rounds > 4:
            raise RuntimeError(f"{p.name}: boundary insertion did not converge")
        # the new vertices are not yet referenced by triangles, so their lattice does not
        # create spurious boundary edges
        eid, side, ta, tb = _boundary_edges(mesh, lrect)
        lo = np.minimum(ta, tb)
        hi = np.maximum(ta, tb)
        match = np.full(todo.size, -1, dtype=np.int64)
        for s in (_W, _E, _S, _N):
            es = np.nonzero(side == s)[0]
            ps = np.nonzero(pmask[todo] == s)[0]
            if ps.size == 0:
                continue
            if es.size == 0:
                raise RuntimeError(f"{p.name}: no boundary edges on side {s}")
            order = es[np.argsort(lo[es], kind="stable")]
            pos = np.searchsorted(lo[order], along[todo[ps]], side="right") - 1
            pos = np.clip(pos, 0, order.size - 1)
            cand = order[pos]
            ok = (lo[cand] < along[todo[ps]]) & (along[todo[ps]] < hi[cand])
            if not np.all(ok):
                raise RuntimeError(f"{p.name}: {int((~ok).sum())} boundary points fall on no boundary edge "
                                   "(mesh does not cover its rectangle?)")
            match[ps] = cand
        edge = eid[match]
        tri_of = edge // 3
        # one split edge per triangle per round (corner triangles may have two boundary edges)
        order = np.lexsort((edge, tri_of))
        first_edge = np.empty_like(edge)
        tri_sorted = tri_of[order]
        start = np.r_[True, tri_sorted[1:] != tri_sorted[:-1]]
        grp = np.cumsum(start) - 1
        first_edge[order] = edge[order][np.nonzero(start)[0]][grp]
        now = edge == first_edge
        sel = todo[now]
        e_sel = edge[now]
        a_loc = e_sel % 3
        t_sel = e_sel // 3
        tri = mesh.triangles
        va = tri[t_sel, a_loc]
        vb = tri[t_sel, (a_loc + 1) % 3]
        vc = tri[t_sel, (a_loc + 2) % 3]
        ta_s = mesh.lattice[va]
        tb_s = mesh.lattice[vb]
        axis = np.where((pmask[sel] == _W) | (pmask[sel] == _E), 1, 0)
        t_a = ta_s[np.arange(sel.size), axis]
        t_b = tb_s[np.arange(sel.size), axis]
        frac = (along[sel] - t_a) / (t_b - t_a)  # 0..1 from a to b
        # attributes of the new vertices
        za = mesh.vertices[va, 2]
        zb = mesh.vertices[vb, 2]
        z_lerp = za + frac * (zb - za)
        z = z_lerp.copy()
        canon = canonical[sel]
        if use_h:
            z[canon] = z_fn[sel[canon]]
        mesh.vertices[new_ids[sel], 2] = z
        if has_n:
            na = mesh.normals[va].astype(np.float64)
            nb = mesh.normals[vb].astype(np.float64)
            nl = na + frac[:, None] * (nb - na)
            nl /= np.maximum(np.linalg.norm(nl, axis=1, keepdims=True), 1e-300)
            if use_n:
                nl[canon] = n_fn[sel[canon]]
            mesh.normals[new_ids[sel]] = nl.astype(np.float32)
        if has_uv:
            if p.uv_tile_m:
                uv = np.column_stack([xy[sel, 0] / p.uv_tile_m, -xy[sel, 1] / p.uv_tile_m])
            else:
                ua = mesh.uvs[va].astype(np.float64)
                ub = mesh.uvs[vb].astype(np.float64)
                uv = ua + frac[:, None] * (ub - ua)
            mesh.uvs[new_ids[sel]] = uv.astype(np.float32)
        # fan triangles: sort by edge, then by fraction along the edge
        o = np.lexsort((frac, e_sel))
        e_o = e_sel[o]
        q_o = new_ids[sel[o]]
        first = np.r_[True, e_o[1:] != e_o[:-1]]
        last = np.r_[e_o[1:] != e_o[:-1], True]
        prev = np.where(first, va[o], np.r_[-1, q_o[:-1]])
        fan = np.column_stack([prev, q_o, vc[o]])
        close = np.column_stack([q_o[last], vb[o][last], vc[o][last]])
        split_tris = np.unique(t_sel)
        keep = np.ones(mesh.n_triangles, dtype=bool)
        keep[split_tris] = False
        mesh.triangles = np.vstack([tri[keep], fan, close])
        todo = todo[~now]
    return {"inserted": int(k), "rounds": rounds}


# ---------------------------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------------------------
def stitch_pieces(pieces: Sequence[Piece], *, copy_normals: bool = True) -> dict:
    """Make all shared boundaries identical (in place). Returns a report dict.

    Report keys: n_pieces, n_contacts (segment / point), inserted (per piece name), n_inserted,
    n_overwritten (existing non-canonical boundary vertices whose height was replaced),
    max_abs_dz_overwritten_m / rms_dz_overwritten_m (how far the zones disagreed before),
    max_normal_change_deg, time_s.
    """
    t0 = time.perf_counter()
    for p in pieces:
        _validate_piece(p)
    contacts = find_contacts(pieces)
    bnd = [_boundary(p) for p in pieces]

    # --- 1. union of boundary vertices on every shared segment -> insertion requests
    requests: list[list[np.ndarray]] = [[] for _ in pieces]
    for c in contacts:
        if c.kind != "segment":
            continue
        ta, _ = _on_segment(bnd[c.a], c)
        tb, _ = _on_segment(bnd[c.b], c)
        union = np.union1d(ta, tb)
        for who, own in ((c.a, ta), (c.b, tb)):
            miss = np.setdiff1d(union, own, assume_unique=True)
            if miss.size:
                pts = (np.column_stack([miss, np.full(miss.size, c.const)]) if c.axis == 0
                       else np.column_stack([np.full(miss.size, c.const), miss]))
                requests[who].append(pts.astype(np.int64))

    # --- 2. insert (with canonical heights / normals where this piece is canonical)
    inserted: dict[str, int] = {}
    n_new_by_piece = []
    for k, p in enumerate(pieces):
        if requests[k]:
            pts = np.unique(np.vstack(requests[k]), axis=0)
            canon = _canonical_mask(pts, k, pieces)
            n_before = p.mesh.n_vertices
            _insert_points(p, pts, canon)
            inserted[p.name] = int(pts.shape[0])
            n_new_by_piece.append((n_before, p.mesh.n_vertices))
        else:
            n_new_by_piece.append((p.mesh.n_vertices, p.mesh.n_vertices))

    # --- 3. canonical values for every shared boundary lattice point
    bnd = [_boundary(p) for p in pieces]
    keys = np.concatenate([_keys(b.lat) for b in bnd]) if bnd else np.zeros(0, np.int64)
    pidx = np.concatenate([np.full(b.idx.size, k, dtype=np.int64) for k, b in enumerate(bnd)])
    vidx = np.concatenate([b.idx for b in bnd])
    prio = np.array([p.priority for p in pieces], dtype=np.int64)[pidx]
    order = np.lexsort((pidx, prio, keys))
    ks = keys[order]
    start = np.r_[True, ks[1:] != ks[:-1]] if ks.size else np.zeros(0, bool)
    grp = np.cumsum(start) - 1
    canon_row = np.nonzero(start)[0][grp]
    rows = np.nonzero(canon_row != np.arange(ks.size))[0]  # rows that copy from their canonical row
    dz_existing: list[np.ndarray] = []
    dn_existing: list[np.ndarray] = []
    n_overwritten = 0
    if rows.size:
        src_p = pidx[order][canon_row[rows]]
        src_v = vidx[order][canon_row[rows]]
        dst_p = pidx[order][rows]
        dst_v = vidx[order][rows]
        z_src = np.empty(rows.size)
        n_src = np.empty((rows.size, 3), dtype=np.float32)
        for k in np.unique(src_p):
            s = src_p == k
            z_src[s] = pieces[k].mesh.vertices[src_v[s], 2]
            if pieces[k].mesh.normals is not None:
                n_src[s] = pieces[k].mesh.normals[src_v[s]]
            else:
                n_src[s] = np.nan
        for k in np.unique(dst_p):
            s = dst_p == k
            m = pieces[k].mesh
            v = dst_v[s]
            pre_existing = v < n_new_by_piece[k][0]
            dz_existing.append((m.vertices[v, 2] - z_src[s])[pre_existing])
            m.vertices[v, 2] = z_src[s]
            n_overwritten += int(pre_existing.sum())
            if copy_normals and m.normals is not None:
                ns = n_src[s]
                good = np.all(np.isfinite(ns), axis=1)
                if np.any(good & pre_existing):
                    n_old = m.normals[v][good & pre_existing].astype(np.float64)
                    n_new = ns[good & pre_existing].astype(np.float64)
                    sin_ = np.linalg.norm(np.cross(n_old, n_new), axis=1)
                    cos_ = np.sum(n_old * n_new, axis=1)
                    dn_existing.append(np.degrees(np.arctan2(sin_, cos_)))
                m.normals[v[good]] = ns[good]
    dz = np.concatenate(dz_existing) if dz_existing else np.zeros(0)
    dn = np.concatenate(dn_existing) if dn_existing else np.zeros(0)
    return {
        "n_pieces": len(pieces),
        "n_contacts": len(contacts),
        "n_segment_contacts": sum(c.kind == "segment" for c in contacts),
        "n_point_contacts": sum(c.kind == "point" for c in contacts),
        "inserted": inserted,
        "n_inserted": int(sum(inserted.values())),
        "n_shared_points_copied": int(rows.size),
        "n_overwritten": n_overwritten,
        "max_abs_dz_overwritten_m": float(np.max(np.abs(dz))) if dz.size else 0.0,
        "rms_dz_overwritten_m": float(np.sqrt(np.mean(dz ** 2))) if dz.size else 0.0,
        "max_normal_change_deg": float(np.max(dn)) if dn.size else 0.0,
        "time_s": round(time.perf_counter() - t0, 4),
    }


def check_stitched(pieces: Sequence[Piece], *, topology: bool = True, compare_normals: bool = True,
                   max_examples: int = 10) -> dict:
    """Independently verify the stitching invariant over all contacts.

    Returns dict(ok, n_pieces, n_contacts, n_shared_points, mismatches (heights not
    bit-identical), t_junctions (boundary points present in one piece but not the other,
    including missing corners), normal_mismatches, max_abs_dz_m, examples, topology ...).
    """
    for p in pieces:
        _validate_piece(p)
    contacts = find_contacts(pieces)
    bnd = [_boundary(p) for p in pieces]
    n_shared = 0
    mismatches = 0
    tj = 0
    nmis = 0
    max_dz = 0.0
    examples: list[dict] = []
    for c in contacts:
        pa, pb = pieces[c.a], pieces[c.b]
        if c.kind == "segment":
            ta, va = _on_segment(bnd[c.a], c)
            tb, vb = _on_segment(bnd[c.b], c)
            common, ia, ib = np.intersect1d(ta, tb, assume_unique=True, return_indices=True)
            only = np.setxor1d(ta, tb, assume_unique=True)
            va, vb = va[ia], vb[ib]
        else:
            key = _keys(np.array([[c.const, c.lo]]))[0]
            ka = np.nonzero(_keys(bnd[c.a].lat) == key)[0]
            kb = np.nonzero(_keys(bnd[c.b].lat) == key)[0]
            only = np.zeros(int(ka.size == 0) + int(kb.size == 0))
            if ka.size and kb.size:
                va, vb = bnd[c.a].idx[ka[:1]], bnd[c.b].idx[kb[:1]]
            else:
                va = vb = np.zeros(0, dtype=np.int64)
        tj += int(only.size)
        n_shared += int(va.size)
        za = pa.mesh.vertices[va, 2]
        zb = pb.mesh.vertices[vb, 2]
        bad = za != zb
        mismatches += int(bad.sum())
        if va.size:
            max_dz = max(max_dz, float(np.max(np.abs(za - zb))))
        if compare_normals and pa.mesh.normals is not None and pb.mesh.normals is not None and va.size:
            nbad = np.any(pa.mesh.normals[va] != pb.mesh.normals[vb], axis=1)
            nmis += int(nbad.sum())
        if (only.size or bad.any()) and len(examples) < max_examples:
            examples.append({"a": pa.name, "b": pb.name, "kind": c.kind, "axis": c.axis, "const": c.const,
                             "lo": c.lo, "hi": c.hi, "t_junctions": int(only.size),
                             "mismatches": int(bad.sum())})
    res = {
        "n_pieces": len(pieces),
        "n_contacts": len(contacts),
        "n_shared_points": n_shared,
        "mismatches": mismatches,
        "t_junctions": tj,
        "normal_mismatches": nmis,
        "max_abs_dz_m": max_dz,
        "examples": examples,
    }
    ok = mismatches == 0 and tj == 0 and (nmis == 0 or not compare_normals)
    if topology:
        topo = union_topology(pieces)
        res["topology"] = topo
        ok = ok and topo["ok"]
    res["ok"] = bool(ok)
    return res


def union_topology(pieces: Sequence[Piece]) -> dict:
    """Topology of the union of all pieces, vertices merged through lattice coordinates.

    Per piece: every non-boundary edge is shared by exactly two triangles with opposite
    orientation, and every edge used once lies on the piece's rectangle boundary. Globally: the
    boundary edges of all pieces are matched by lattice keys; an edge used once must lie on the
    outer boundary of the union (its midpoint inside exactly one rectangle), else it is a crack.
    Also reports the Euler characteristic V - E + F of the union (1 for a disc).
    """
    cracks = 0
    nonmanifold = 0
    orient = 0
    piece_bad_boundary = 0
    n_v = 0
    n_e = 0
    n_f = 0
    gb_a: list[np.ndarray] = []
    gb_b: list[np.ndarray] = []
    for p in pieces:
        m = p.mesh
        tri = m.triangles
        n_f += m.n_triangles
        a = tri.reshape(-1)
        b = tri[:, [1, 2, 0]].reshape(-1)
        lo = np.minimum(a, b)
        hi = np.maximum(a, b)
        key = lo * np.int64(m.n_vertices) + hi
        sign = np.where(a < b, 1, -1)
        uk, first, inv, cnt = np.unique(key, return_index=True, return_inverse=True, return_counts=True)
        inv = np.asarray(inv).ravel()
        ssum = np.bincount(inv, weights=sign, minlength=uk.size)
        nonmanifold += int(np.sum(cnt > 2))
        orient += int(np.sum((cnt == 2) & (ssum != 0)))
        once = np.nonzero(cnt == 1)[0]
        n_e += int(np.sum(cnt >= 2))  # interior edges of the piece
        # boundary edges (used once): must lie on the rectangle boundary
        e_once = first[once]
        mask = _side_mask(m.lattice, p.lrect)
        on_side = (mask[a[e_once]] & mask[b[e_once]]) != 0
        piece_bad_boundary += int(np.sum(~on_side))
        gb_a.append(_keys(m.lattice[a[e_once]]))
        gb_b.append(_keys(m.lattice[b[e_once]]))
        used = np.zeros(m.n_vertices, dtype=bool)
        used[tri.reshape(-1)] = True
        n_v += int(np.sum(used & (mask == 0)))  # interior vertices are unique to the piece
    ga = np.concatenate(gb_a) if gb_a else np.zeros(0, np.int64)
    gb = np.concatenate(gb_b) if gb_b else np.zeros(0, np.int64)
    bkeys = np.unique(np.concatenate([ga, gb]))
    n_v += int(bkeys.size)
    # global boundary-edge matching on lattice keys
    lo = np.minimum(ga, gb)
    hi = np.maximum(ga, gb)
    pair = np.column_stack([lo, hi])
    upair, inv, cnt = np.unique(pair, axis=0, return_inverse=True, return_counts=True)
    inv = np.asarray(inv).ravel()
    sign = np.where(ga < gb, 1, -1)
    ssum = np.bincount(inv, weights=sign, minlength=upair.shape[0])
    nonmanifold += int(np.sum(cnt > 2))
    orient += int(np.sum((cnt == 2) & (ssum != 0)))
    n_e += int(upair.shape[0])
    single = upair[cnt == 1]
    n_outer = 0
    if single.size:
        # decode keys -> lattice, midpoint in doubled coordinates, count containing rectangles
        def dec(k):
            return np.column_stack([k // (2 * _KEY_OFFSET) - _KEY_OFFSET, k % (2 * _KEY_OFFSET) - _KEY_OFFSET])

        pa = dec(single[:, 0])
        pb = dec(single[:, 1])
        mid2 = pa + pb
        inside = np.zeros(single.shape[0], dtype=np.int64)
        for p in pieces:
            x0, x1, y0, y1 = p.lrect
            inside += ((mid2[:, 0] >= 2 * x0) & (mid2[:, 0] <= 2 * x1) &
                       (mid2[:, 1] >= 2 * y0) & (mid2[:, 1] <= 2 * y1))
        cracks = int(np.sum(inside >= 2))
        n_outer = int(np.sum(inside == 1))
    euler = n_v - n_e + n_f
    return {
        "ok": bool(cracks == 0 and nonmanifold == 0 and orient == 0 and piece_bad_boundary == 0),
        "cracks": cracks,
        "nonmanifold_edges": nonmanifold,
        "orientation_conflicts": orient,
        "piece_boundary_edges_off_rect": piece_bad_boundary,
        "outer_boundary_edges": n_outer,
        "V": n_v,
        "E": n_e,
        "F": n_f,
        "euler": int(euler),
    }
