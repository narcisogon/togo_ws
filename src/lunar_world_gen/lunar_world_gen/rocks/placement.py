"""Rock placement: size class, prototype, orientation, burial and height (docs/DESIGN.md section 5).

Given rock centres (x, y) and diameters D from ``population.py``, ``place_rocks`` fills the catalog
fields ``cls, lod, proto, scale, q*, burial, z, exposed_h, collide``:

* **Class and LOD.** ``cfg.rocks.classes`` maps D to a size class (pebble / small / medium / large),
  its library class and the LOD used in the core. Ring rocks use ``cfg.rocks.ring_lod``; callers
  may pass explicit LODs (the core-edge band of ``population``).
* **Prototype.** Uniform choice within the library class, restricted to prototypes whose longest
  axis ``a`` is within a factor ``rocks.proto_size_window`` (default 3) of D. That bounds the
  scale ``s = D / a`` to 1/3..3, which keeps the baked texel size and the LOD error close to
  their design values. If no prototype is in the window, the nearest one in log size is used.
  (DESIGN says "uniformly within the class library"; this is the same rule with a size window.)
* **Burial.** A ``perched_fraction`` of rocks is perched: not buried, only bedded into the regolith
  by a compliance burial ``U(rocks.perched_burial)`` (default 2-6 % of the rock height; FID-01:
  perched means "not buried", not "not in contact"). A perched rock whose rest facet is not flat
  (a keel) is bedded deeper, by ``(base_gap / a - rocks.bedding.target_gap_over_D) * D`` (at most
  ``max_frac`` = 18 % of its height, inside the calibration's 0-20 % for young debris): the keel
  presses into the loose top layer. After the contact
  rule, perched rocks >= ``adaptive_min_D_m`` (5 cm) settle further until the gap between their hull
  underside and the real ground over the central 60 % of the footprint is <= 3 % of D (same cap):
  on undulating ground a big rock is supported over an area, not on three grains. The others get
  ``f = lo + (hi - lo) * Beta(burial_beta)`` with the class range ``[lo, hi]`` (``[0, 1]`` gives
  the calibration's young-debris Beta(1.5, 6) directly, FID-12).
* **Orientation (FID-01).** Every rock first rests on a stable face of its convex hull
  (``RockLibrary.rest_faces``: facets the centre of mass projects into). Candidates have a tipping
  margin >= ``rocks.rest_face.min_tip_deg`` (3 deg; the single-facet margin underestimates gently
  curved bases) and a pose height <= ``max_extent_over_a`` (0.8) x the long axis; the facet is drawn
  with probability ~ solid angle (landing probability) x exp(-base_gap / (flat_scale * a)), so
  rocks come to rest on flat faces rather than on keels (``base_gap``: underside height over the
  central 60 % of the footprint). Then a random yaw, then the rotation that takes +z to the normal
  of the ground plane fitted under the footprint (central differences of 4 samples at radius
  0.4 D). Then the rock is **settled** on the real ground
  (``settle_rocks``): rotated about its centre of mass until three vertices touch the ground with the
  centre of mass over their triangle (quasi-static rest on rigid ground; a 2-D LP solved per rock,
  iterated because the ground under the vertices changes as they move). Finally buried rocks get a
  random tilt of up to ``tilt_max_deg`` (uniform angle, random horizontal axis) and perched rocks of
  up to ``rocks.perched_tilt_max_deg`` (default 0).
  ``rocks.settle.enabled: false``, ``rocks.rest_face.mode: authored``, ``perched_burial: [0, 0]``
  and ``perched_tilt_max_deg = tilt_max_deg`` give the pre-fix scheme (authored base down, random
  tilt, lowest vertex touching; random streams differ).
* **Height (contact rule).** The rock is lowered until its lowest visual vertex touches the ground
  under that vertex, then sunk by ``f * c_rot * s`` (``c_rot`` = vertical extent of the rotated
  rock). On a plane without extra tilt this is exactly the DESIGN rule
  ``z = ground + s * (bottom_offset - f * c)``; on rough or tilted ground it guarantees that no rock
  floats and that the deepest point is buried by the sampled fraction (the DESIGN rule with
  "lowest of 5 footprint samples" only approximates that). Ground = H0, without fillets.
* ``exposed_h`` = top of the rotated rock above the ground at its centre;
  ``collide = D >= rocks.collision_min_D_m`` for core rocks, False for ring rocks.

Quaternions are (w, x, y, z) and rotate the rock-local frame (Z up) into the world frame:
``p_world = t + s * R(q) @ p_local``.
"""
from __future__ import annotations

from typing import Callable

import numpy as np
from numba import njit, prange

from ..grid import Grid
from .library import RockLibrary

#: defaults of the optional config keys read here (the config may omit them)
PERCHED_BURIAL = (0.02, 0.06)          # rocks.perched_burial: compliance burial of perched rocks
REST_FACE = {"mode": "stable", "min_tip_deg": 3.0, "max_extent_over_a": 0.8, "flat_scale": 0.02}  # rocks.rest_face
BEDDING = {"target_gap_over_D": 0.03, "max_frac": 0.18, "apply_to": "perched", "adaptive_min_D_m": 0.05}
#   rocks.bedding: perched keels press into the loose top layer; perched rocks >= adaptive_min_D_m settle
#   until the central gap on the real ground is <= target * D; 'apply_to: all' also floors buried rocks
SETTLE = {"enabled": True, "apply_to": "all", "iterations": 4, "max_step_deg": 20.0}   # rocks.settle
PERCHED_TILT_MAX_DEG = 0.0             # rocks.perched_tilt_max_deg


def _opt(cfg, key: str, default):
    """cfg.rocks.<key> with a default (dict defaults are merged key by key)."""
    v = cfg["rocks"].get(key, None)
    if isinstance(default, dict):
        out = dict(default)
        out.update(dict(v or {}))
        return out
    return default if v is None else v


# ---------------------------------------------------------------------------------------------
# Quaternions (vectorised, (n, 4) arrays in w, x, y, z order)
# ---------------------------------------------------------------------------------------------
def quat_from_axis_angle(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64).reshape(-1, 3)
    angle = np.asarray(angle, dtype=np.float64).reshape(-1)
    n = np.linalg.norm(axis, axis=1, keepdims=True)
    axis = axis / np.maximum(n, 1e-300)
    h = 0.5 * angle
    s = np.sin(h)
    return np.column_stack([np.cos(h), axis * s[:, None]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a * b (rotation b first, then a)."""
    aw, ax, ay, az = np.moveaxis(np.asarray(a, dtype=np.float64).reshape(-1, 4), 1, 0)
    bw, bx, by, bz = np.moveaxis(np.asarray(b, dtype=np.float64).reshape(-1, 4), 1, 0)
    return np.column_stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(-1, 4)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    return np.where(q[:, :1] < 0, -q, q)          # canonical sign (w >= 0)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(n,4) unit quaternions -> (n,3,3) rotation matrices."""
    q = np.asarray(q, dtype=np.float64).reshape(-1, 4)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def quat_z_to(n: np.ndarray) -> np.ndarray:
    """Minimal rotation taking +z onto the unit vectors n (n,3)."""
    n = np.asarray(n, dtype=np.float64).reshape(-1, 3)
    n = n / np.linalg.norm(n, axis=1, keepdims=True)
    # q = (1 + z.n, z x n) normalised; z x n = (-n_y, n_x, 0)
    w = 1.0 + n[:, 2]
    q = np.column_stack([w, -n[:, 1], n[:, 0], np.zeros(len(n))])
    bad = w < 1e-12                                # n = -z: rotate pi about x
    q[bad] = (0.0, 1.0, 0.0, 0.0)
    return quat_normalize(q)


def quat_from_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minimal rotations taking the unit vectors a onto b ((n,3) each, or one of them (3,))."""
    a = np.asarray(a, dtype=np.float64).reshape(-1, 3)
    b = np.asarray(b, dtype=np.float64).reshape(-1, 3)
    a, b = np.broadcast_arrays(a, b)
    a = a / np.linalg.norm(a, axis=1, keepdims=True)
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    w = 1.0 + np.einsum("ij,ij->i", a, b)
    q = np.column_stack([w, np.cross(a, b)])
    bad = w < 1e-12                                # antiparallel: half turn about any axis normal to a
    if bad.any():
        ax = np.cross(a[bad], np.array([1.0, 0.0, 0.0]))
        small = np.linalg.norm(ax, axis=1) < 1e-6
        ax[small] = np.cross(a[bad][small], np.array([0.0, 1.0, 0.0]))
        q[bad] = np.column_stack([np.zeros(int(bad.sum())), ax / np.linalg.norm(ax, axis=1, keepdims=True)])
    return quat_normalize(q)


def quat_yaw(q: np.ndarray) -> np.ndarray:
    """Heading [rad] of the rotated local +x axis projected onto the xy plane."""
    R = quat_to_matrix(q)
    return np.arctan2(R[:, 1, 0], R[:, 0, 0])


def catalog_quats(rows: np.ndarray) -> np.ndarray:
    return np.column_stack([rows["qw"], rows["qx"], rows["qy"], rows["qz"]]).astype(np.float64)


# ---------------------------------------------------------------------------------------------
# Ground interface: any object with height(x, y) and normal(x, y)
# ---------------------------------------------------------------------------------------------
class GroundGrid:
    """Ground from a ``Grid`` (bilinear heights, normals from central differences of the grid)."""

    def __init__(self, grid: Grid, order: int = 1):
        self.grid = grid
        self.order = order

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return self.grid.sample(np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64), order=self.order)

    def normal(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        h = self.grid.spacing
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        gx = (self.height(x + h, y) - self.height(x - h, y)) / (2 * h)
        gy = (self.height(x, y + h) - self.height(x, y - h)) / (2 * h)
        return _normal_from_gradient(gx, gy)


class GroundFunction:
    """Ground from a vectorised height function f(x, y) (and optionally a normal function)."""

    def __init__(self, height_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
                 normal_fn: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None, eps: float = 0.005):
        self._h = height_fn
        self._n = normal_fn
        self.eps = eps

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        return np.asarray(self._h(x, np.asarray(y, dtype=np.float64)), dtype=np.float64) + 0.0 * x

    def normal(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        if self._n is not None:
            return np.asarray(self._n(x, y), dtype=np.float64)
        e = self.eps
        gx = (self.height(x + e, y) - self.height(x - e, y)) / (2 * e)
        gy = (self.height(x, y + e) - self.height(x, y - e)) / (2 * e)
        return _normal_from_gradient(gx, gy)


def flat_ground(z0: float = 0.0) -> GroundFunction:
    return GroundFunction(lambda x, y: np.full(np.shape(x), float(z0)))


def as_ground(obj) -> GroundGrid | GroundFunction:
    """Wrap a Grid or a callable; objects that already have .height are returned unchanged."""
    if hasattr(obj, "height"):
        return obj
    if isinstance(obj, Grid):
        return GroundGrid(obj)
    if callable(obj):
        return GroundFunction(obj)
    raise TypeError(f"cannot use {type(obj)} as ground (need a Grid, a callable or .height/.normal)")


def _normal_from_gradient(gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    n = np.stack([-np.asarray(gx), -np.asarray(gy), np.ones(np.shape(gx))], axis=-1)
    return n / np.linalg.norm(n, axis=-1, keepdims=True)


def footprint_plane(ground, x: np.ndarray, y: np.ndarray, r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares plane through 5 samples (centre + 4 at radius r): (normal (n,3), centre height)."""
    xs = np.concatenate([x, x + r, x - r, x, x])
    ys = np.concatenate([y, y, y, y + r, y - r])
    h = ground.height(xs, ys).reshape(5, -1)
    gx = (h[1] - h[2]) / (2 * r)
    gy = (h[3] - h[4]) / (2 * r)
    return _normal_from_gradient(gx, gy), h[0]


# ---------------------------------------------------------------------------------------------
# Class / prototype / orientation / burial
# ---------------------------------------------------------------------------------------------
def class_table(cfg) -> list[dict]:
    """cfg.rocks.classes as plain dicts, in config order (the catalog's ``cls`` indexes this list)."""
    return [dict(c) for c in cfg["rocks"]["classes"]]


def size_class(cfg, D: np.ndarray) -> np.ndarray:
    """Config index of the size class of each D: the class with the smallest D_max_m > D
    (the largest class for D beyond every D_max_m)."""
    tab = class_table(cfg)
    dmax = np.array([float(c["D_max_m"]) for c in tab])
    order = np.argsort(dmax, kind="stable")
    k = np.minimum(np.searchsorted(dmax[order], np.asarray(D, dtype=np.float64), side="right"), len(dmax) - 1)
    return order[k]


def choose_prototypes(rng: np.random.Generator, library: RockLibrary, lib_cls: np.ndarray, D: np.ndarray,
                      window: float = 3.0) -> np.ndarray:
    """Prototype index per rock: uniform within the library class among sizes in [D/w, D*w]."""
    D = np.asarray(D, dtype=np.float64)
    out = np.empty(len(D), dtype=np.int64)
    u = rng.random(len(D))
    for name in np.unique(lib_cls):
        sel = np.nonzero(lib_cls == name)[0]
        protos = library.by_class(str(name))
        if not protos:
            raise ValueError(f"rock library {library.path} has no '{name}' prototypes")
        idx = np.array([p.index for p in protos])
        la = np.log(np.array([p.a for p in protos]))
        lD = np.log(D[sel])
        ok = np.abs(lD[:, None] - la[None, :]) <= np.log(window) + 1e-12      # (n_sel, n_proto)
        none = ~ok.any(axis=1)
        if none.any():                                                       # nearest in log size
            ok[none, np.argmin(np.abs(lD[none, None] - la[None, :]), axis=1)] = True
        cnt = ok.sum(axis=1)
        k = np.minimum((u[sel] * cnt).astype(np.int64), cnt - 1)             # k-th allowed prototype
        rank = np.cumsum(ok, axis=1) - 1
        pick = np.argmax(ok & (rank == k[:, None]), axis=1)
        out[sel] = idx[pick]
    return out


def random_orientation(rng: np.random.Generator, normals: np.ndarray, tilt_max_deg: float) -> np.ndarray:
    """q = q_align(normal) * q_tilt * q_yaw (yaw uniform, tilt uniform in [0, tilt_max])."""
    n = len(normals)
    yaw = rng.uniform(0.0, 2.0 * np.pi, n)
    tilt = np.radians(tilt_max_deg) * rng.random(n)
    phi = rng.uniform(0.0, 2.0 * np.pi, n)
    z = np.tile([0.0, 0.0, 1.0], (n, 1))
    q_yaw = quat_from_axis_angle(z, yaw)
    q_tilt = quat_from_axis_angle(np.column_stack([np.cos(phi), np.sin(phi), np.zeros(n)]), tilt)
    q_align = quat_z_to(normals)
    return quat_normalize(quat_mul(q_align, quat_mul(q_tilt, q_yaw)))


def burial_fractions(cfg, rng: np.random.Generator, cls_idx: np.ndarray, return_perched: bool = False):
    """Burial fraction of the rotated rock height per rock.

    Perched with probability ``perched_fraction``: compliance burial ``U(rocks.perched_burial)``
    (default 2-6 %; ``[0, 0]`` gives the old burial 0). Else ``lo + (hi - lo) * Beta(a, b)`` with the
    class range ``[lo, hi]`` (default ``[0, 1]`` when a class has no ``burial`` key).
    With ``return_perched`` also the perched mask.
    """
    rk = cfg["rocks"]
    a, b = (float(v) for v in rk["burial_beta"])
    n = len(cls_idx)
    beta = rng.beta(a, b, n)
    perched = rng.random(n) < float(rk["perched_fraction"])
    tab = class_table(cfg)
    lo = np.array([float(c.get("burial", (0.0, 1.0))[0]) for c in tab])[cls_idx]
    hi = np.array([float(c.get("burial", (0.0, 1.0))[1]) for c in tab])[cls_idx]
    f = lo + (hi - lo) * beta
    p0, p1 = (float(v) for v in _opt(cfg, "perched_burial", PERCHED_BURIAL))
    comp = p0 + (p1 - p0) * rng.random(n)
    f[perched] = comp[perched]
    return (f, perched) if return_perched else f


def rest_face_normals(cfg, rng: np.random.Generator, library: RockLibrary, proto: np.ndarray,
                      return_gap: bool = False):
    """Outward normal (rock frame) of the hull facet each rock rests on (see the module docstring).

    ``rocks.rest_face.mode``: ``stable`` (default) or ``authored`` (the library's authored base, -z, as
    before FID-01). With ``return_gap`` also the facet's ``base_gap`` relative to the prototype's
    longest axis (underside flatness, ``RockLibrary.rest_faces``; 0 for the authored base)."""
    rf = _opt(cfg, "rest_face", REST_FACE)
    n = len(proto)
    u = rng.random(n)
    out = np.tile([0.0, 0.0, -1.0], (n, 1))
    gap = np.zeros(n)
    if str(rf["mode"]) == "authored" or n == 0:
        return (out, gap) if return_gap else out
    for p in np.unique(proto):
        sel = np.nonzero(proto == p)[0]
        normals, w, g = rest_face_candidates(library, int(p), float(rf["min_tip_deg"]),
                                             float(rf["max_extent_over_a"]), float(rf["flat_scale"]),
                                             return_gap=True)
        cw = np.cumsum(w) / w.sum()
        k = np.minimum(np.searchsorted(cw, u[sel], side="right"), len(w) - 1)
        out[sel] = normals[k]
        gap[sel] = g[k]
    return (out, gap) if return_gap else out


def rest_face_candidates(library: RockLibrary, proto: int, min_tip_deg: float = 3.0, max_extent_over_a: float = 0.8,
                         flat_scale: float = 0.02, return_gap: bool = False):
    """(normals (k,3), weights (k,)) of the facets a prototype may rest on (+ base_gap / a with
    ``return_gap``).

    Candidates: stable facets with a tipping margin >= ``min_tip_deg`` whose pose height is at most
    ``max_extent_over_a`` x the longest axis (no rocks standing on end); if none qualifies, the
    stable ones with enough margin, else the most stable facet. Weight = solid angle (landing
    probability) x exp(-base_gap / (flat_scale * a)): rocks come to rest on flat faces, not on keels.
    (The facet margin underestimates the stability of gently curved bases, whose neighbouring
    vertices also touch, hence the small default margin.)"""
    f = library.rest_faces(proto)
    a = library.proto(proto).a
    ok = (f["tip_deg"] >= min_tip_deg) & (f["extent"] <= max_extent_over_a * a)
    if not ok.any():
        ok = f["tip_deg"] >= min_tip_deg
    if not ok.any():
        ok = f["tip_deg"] >= f["tip_deg"].max() - 1e-12
    w = np.maximum(f["solid_angle"][ok], 1e-12) * np.exp(-f["base_gap"][ok] / (max(flat_scale, 1e-9) * a))
    w = np.maximum(w, 1e-300)
    if return_gap:
        return f["normal"][ok], w, f["base_gap"][ok] / a
    return f["normal"][ok], w


# ---------------------------------------------------------------------------------------------
# Settling on the ground (FID-01)
# ---------------------------------------------------------------------------------------------
@njit(cache=False)
def _min_affine(e, u, v, a, b):
    m = np.inf
    for k in range(e.shape[0]):
        val = e[k] - a * u[k] - b * v[k]
        if val < m:
            m = val
    return m


@njit(cache=False)
def _best_b(e, u, v, a, s, iters):
    gr = 0.6180339887498949
    lo, hi = -s, s
    x1 = hi - gr * (hi - lo)
    x2 = lo + gr * (hi - lo)
    f1 = _min_affine(e, u, v, a, x1)
    f2 = _min_affine(e, u, v, a, x2)
    for _ in range(iters):
        if f1 < f2:
            lo = x1
            x1 = x2
            f1 = f2
            x2 = lo + gr * (hi - lo)
            f2 = _min_affine(e, u, v, a, x2)
        else:
            hi = x2
            x2 = x1
            f2 = f1
            x1 = hi - gr * (hi - lo)
            f1 = _min_affine(e, u, v, a, x1)
    b = 0.5 * (lo + hi)
    return _min_affine(e, u, v, a, b), b


@njit(cache=False, parallel=True)
def _support_slopes(U, V, E, s, iters, out):
    """Per row r: the plane e = t + a u + b v below all points (U[r,k], V[r,k], E[r,k]) with the
    largest t (value above the origin = the centre of mass), |a|, |b| <= s; writes (a, b, t).

    That plane is the lower convex hull's facet above the origin (the 2-D LP of a rigid body resting
    on rigid ground); nested golden-section search on the concave min-of-affine function. Only points
    within 2 s r of the lowest one can touch such a plane, the others are skipped."""
    gr = 0.6180339887498949
    m = U.shape[1]
    for r in prange(U.shape[0]):                     # rows are independent: deterministic in parallel
        emin = np.inf
        rmax = 0.0
        for k in range(m):
            emin = min(emin, E[r, k])
            rmax = max(rmax, np.sqrt(U[r, k] ** 2 + V[r, k] ** 2))
        lim = emin + 2.0 * s * rmax + 1e-12
        cnt = 0
        for k in range(m):
            if E[r, k] <= lim:
                cnt += 1
        e = np.empty(cnt)
        u = np.empty(cnt)
        v = np.empty(cnt)
        c = 0
        for k in range(m):
            if E[r, k] <= lim:
                e[c] = E[r, k]
                u[c] = U[r, k]
                v[c] = V[r, k]
                c += 1
        lo, hi = -s, s
        x1 = hi - gr * (hi - lo)
        x2 = lo + gr * (hi - lo)
        f1, b1 = _best_b(e, u, v, x1, s, iters)
        f2, b2 = _best_b(e, u, v, x2, s, iters)
        for _ in range(iters):
            if f1 < f2:
                lo = x1
                x1 = x2
                f1 = f2
                b1 = b2
                x2 = lo + gr * (hi - lo)
                f2, b2 = _best_b(e, u, v, x2, s, iters)
            else:
                hi = x2
                x2 = x1
                f2 = f1
                b2 = b1
                x1 = hi - gr * (hi - lo)
                f1, b1 = _best_b(e, u, v, x1, s, iters)
        a = 0.5 * (lo + hi)
        t, b = _best_b(e, u, v, a, s, iters)
        out[r, 0] = a
        out[r, 1] = b
        out[r, 2] = t


def settle_rocks(ground, library: RockLibrary, x, y, scale, q, proto, lod, mask=None, iterations: int = 4,
                 max_step_deg: float = 20.0, tol_slope: float = 1e-4, golden_iters: int = 26,
                 chunk: int = 400_000) -> tuple[np.ndarray, dict]:
    """Rest rocks on the ground (FID-01): rotate each selected rock about its centre of mass until its
    support plane is level, i.e. three visual-LOD vertices touch the ground with the centre of mass
    over their triangle (rigid rock, rigid ground; the vertical position is set afterwards by the
    contact rule). Returns (new quaternions, stats)."""
    q = np.array(q, dtype=np.float64, copy=True)
    n = len(q)
    active = np.arange(n) if mask is None else np.nonzero(mask)[0]
    s = float(np.tan(np.radians(max_step_deg)))
    stats = {"n": int(len(active)), "iterations": 0, "not_converged": 0, "max_total_deg": 0.0}
    total = np.zeros(n)
    for it in range(iterations):
        if not len(active):
            break
        stats["iterations"] = it + 1
        R = quat_to_matrix(q[active])
        res = np.zeros((len(active), 3))
        key = proto[active].astype(np.int64) * 16 + lod[active].astype(np.int64)
        for k in np.unique(key):
            sel = np.nonzero(key == k)[0]
            pts = library.contact_points(int(k // 16), int(k % 16))
            m = len(pts)
            step = max(1, chunk // max(m, 1))
            for s0 in range(0, len(sel), step):
                ss = sel[s0:s0 + step]
                ia = active[ss]
                Vw = np.einsum("kij,mj->kmi", R[ss], pts) * scale[ia, None, None]
                g = ground.height((x[ia, None] + Vw[:, :, 0]).ravel(),
                                  (y[ia, None] + Vw[:, :, 1]).ravel()).reshape(len(ss), m)
                o = np.zeros((len(ss), 3))
                _support_slopes(np.ascontiguousarray(Vw[:, :, 0]), np.ascontiguousarray(Vw[:, :, 1]),
                                np.ascontiguousarray(Vw[:, :, 2] - g), s, golden_iters, o)
                res[ss] = o
        a, b = res[:, 0], res[:, 1]
        # rotation taking the support plane's normal (-a, -b, 1) to +z levels the three contacts
        q_corr = quat_from_to(np.column_stack([-a, -b, np.ones(len(a))]), np.array([0.0, 0.0, 1.0]))
        q[active] = quat_normalize(quat_mul(q_corr, q[active]))
        ang = np.degrees(np.arctan(np.hypot(a, b)))
        total[active] += ang
        done = np.hypot(a, b) < tol_slope
        active = active[~done]
    stats["not_converged"] = int(len(active))
    stats["max_total_deg"] = float(total.max()) if n else 0.0
    stats["median_total_deg"] = float(np.median(total[mask] if mask is not None and mask.any() else total)) if n else 0.0
    return q, stats


def inner_underside_gap(ground, library: RockLibrary, x, y, z, scale, q, proto, inner: float = 0.6,
                        n_side: int = 11) -> np.ndarray:
    """Per rock: the largest gap between the convex hull's underside and the ground over the central
    ``inner`` part of the footprint (the footprint polygon shrunk to ``inner`` about its centroid),
    sampled on an ``n_side`` x ``n_side`` lattice. The hull contains the visual LOD up to a few mm,
    so this is a slightly conservative estimate of the visible gap."""
    from scipy.spatial import ConvexHull

    x = np.asarray(x, dtype=np.float64)
    out = np.zeros(len(x))
    if not len(x):
        return out
    R = quat_to_matrix(q)
    t = np.linspace(0.0, 1.0, n_side)
    for k in range(len(x)):
        p = int(proto[k])
        nl, ol = library.hull_planes(p)
        Vw = (library.hull(p) @ R[k].T) * scale[k]
        nw = nl @ R[k].T
        ow = scale[k] * ol
        try:
            P = Vw[ConvexHull(Vw[:, :2]).vertices, :2]
        except Exception:  # noqa: BLE001  (degenerate footprint)
            continue
        cr = P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]
        A = 0.5 * cr.sum()
        if abs(A) < 1e-300:
            continue
        c = np.array([((P[:, 0] + np.roll(P[:, 0], -1)) * cr).sum(), ((P[:, 1] + np.roll(P[:, 1], -1)) * cr).sum()]) / (6 * A)
        Q = c + inner * (P - c)
        lo, hi = Q.min(axis=0), Q.max(axis=0)
        U, V = np.meshgrid(lo[0] + t * (hi[0] - lo[0]), lo[1] + t * (hi[1] - lo[1]))
        U, V = U.ravel(), V.ravel()
        E = np.roll(Q, -1, axis=0) - Q                                  # inside a CCW convex polygon
        ins = np.all(E[:, 0][None, :] * (V[:, None] - Q[:, 1][None, :])
                     - E[:, 1][None, :] * (U[:, None] - Q[:, 0][None, :]) >= -1e-12, axis=1)
        if not ins.any():
            continue
        U, V = U[ins], V[ins]
        down = nw[:, 2] < -1e-9
        zb = ((ow[down][None, :] - nw[down, 0][None, :] * U[:, None] - nw[down, 1][None, :] * V[:, None])
              / nw[down, 2][None, :]).max(axis=1)
        g = ground.height(x[k] + U, y[k] + V)
        out[k] = float(np.max(z[k] + zb - g))
    return out


def orient_rocks(cfg, rng: np.random.Generator, library: RockLibrary, ground, x, y, D, scale, proto, lod,
                 perched) -> tuple[np.ndarray, dict]:
    """Orientation of every rock (see the module docstring): rest face, yaw, ground alignment,
    settling, then the random tilt (buried: tilt_max_deg, perched: perched_tilt_max_deg).
    Returns (q, stats); stats['base_gap_over_D'] is the rest facet's underside flatness per rock."""
    n = len(D)
    rk = cfg["rocks"]
    normal, _ = footprint_plane(ground, x, y, np.maximum(0.4 * D, 0.005))
    nf, base_gap = rest_face_normals(cfg, rng, library, proto, return_gap=True)
    yaw = rng.uniform(0.0, 2.0 * np.pi, n)
    tilt = rng.random(n)
    phi = rng.uniform(0.0, 2.0 * np.pi, n)
    zax = np.tile([0.0, 0.0, 1.0], (n, 1))
    q_rest = quat_from_to(nf, np.array([0.0, 0.0, -1.0]))
    q = quat_mul(quat_z_to(normal), quat_mul(quat_from_axis_angle(zax, yaw), q_rest))
    st = _opt(cfg, "settle", SETTLE)
    stats: dict = {"settle": None}
    if bool(st["enabled"]) and n:
        mask = np.ones(n, dtype=bool) if str(st["apply_to"]) == "all" else np.asarray(perched, dtype=bool)
        q, stats["settle"] = settle_rocks(ground, library, x, y, scale, q, proto, lod, mask,
                                          iterations=int(st["iterations"]), max_step_deg=float(st["max_step_deg"]))
    tmax = np.where(perched, float(_opt(cfg, "perched_tilt_max_deg", PERCHED_TILT_MAX_DEG)),
                    float(rk["tilt_max_deg"]))
    ang = np.radians(tmax) * tilt
    q_tilt = quat_from_axis_angle(np.column_stack([np.cos(phi), np.sin(phi), np.zeros(n)]), ang)
    stats["base_gap_over_D"] = base_gap
    return quat_normalize(quat_mul(q_tilt, q)), stats


# ---------------------------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------------------------
def place_rocks(cfg, rng: np.random.Generator, x: np.ndarray, y: np.ndarray, D: np.ndarray, zone: str,
                ground, library: RockLibrary, lod: np.ndarray | None = None,
                stats: dict | None = None) -> dict[str, np.ndarray]:
    """Fill the placement fields of the catalog for rocks at (x, y) with diameters D.

    lod    optional explicit LOD per rock (default: the class LOD in the core, ``ring_lod`` in the ring)
    stats  optional dict that receives placement statistics (settling)
    Returns a dict of arrays: cls, lod, proto, scale, qw, qx, qy, qz, burial, z, exposed_h, collide.
    """
    ground = as_ground(ground)
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    n = len(D)
    rk = cfg["rocks"]
    tab = class_table(cfg)
    cls_idx = size_class(cfg, D)
    lib_cls = np.array([tab[k]["library_class"] for k in range(len(tab))], dtype=object)[cls_idx]
    if lod is not None:
        lod = np.asarray(lod, dtype=np.int64).reshape(-1) * np.ones(n, dtype=np.int64)
    elif zone == "core":
        lod = np.array([int(c["lod"]) for c in tab])[cls_idx]
    else:
        lod = np.full(n, int(rk["ring_lod"]))
    window = float(rk.get("proto_size_window", 3.0))
    proto = choose_prototypes(rng, library, lib_cls, D, window)
    # clamp LODs to what the prototype has
    nl = np.array([p.n_lods for p in library.prototypes])[proto]
    lod = np.minimum(lod, nl - 1)
    a_proto = library.a_array()[proto]
    scale = D / a_proto
    burial, perched = burial_fractions(cfg, rng, cls_idx, return_perched=True)
    q, ost = orient_rocks(cfg, rng, library, ground, x, y, D, scale, proto, lod, perched)
    bed = _opt(cfg, "bedding", BEDDING)
    bed_depth = np.clip(ost["base_gap_over_D"] - float(bed["target_gap_over_D"]), 0.0, None) * D
    if str(bed["apply_to"]) != "all":
        bed_depth[~perched] = 0.0             # buried rocks keep the calibrated burial distribution (FID-12)
    z, exposed, burial = _contact_heights(ground, library, x, y, scale, q, proto, lod, burial,
                                          min_sink=bed_depth, max_sink_frac=float(bed["max_frac"]),
                                          return_burial=True)
    # perched rocks >= adaptive_min_D_m settle into the loose top layer until the gap under the central
    # part of their footprint (on the real ground) is <= target_gap_over_D * D (same cap)
    ad = perched & (D >= float(bed["adaptive_min_D_m"])) & (float(bed["max_frac"]) > 0)
    n_ad = 0
    if ad.any():
        idx = np.nonzero(ad)[0]
        gap = inner_underside_gap(ground, library, x[idx], y[idx], z[idx], scale[idx], q[idx], proto[idx])
        hrot = _rotated_heights(library, scale[idx], q[idx], proto[idx], lod[idx])
        extra = np.clip(gap - float(bed["target_gap_over_D"]) * D[idx], 0.0, None)
        extra = np.minimum(extra, np.maximum(float(bed["max_frac"]) * hrot - burial[idx] * hrot, 0.0))
        z[idx] -= extra
        exposed[idx] -= extra
        burial[idx] += extra / np.maximum(hrot, 1e-300)
        n_ad = int(np.count_nonzero(extra > 0))
    if stats is not None:
        stats.update({"n": int(n), "n_perched": int(perched.sum()), "settle": ost.get("settle"),
                      "n_bedded": int(np.count_nonzero(bed_depth > 0)), "n_bedded_adaptive": n_ad,
                      "perched_mask": perched})
    # DESIGN 5: collide = D >= collision_min_D_m, core only (ring rocks >= 30 cm are raised into the
    # ring collision field by size, DESIGN 7)
    collide = (D >= float(rk["collision_min_D_m"])) & (zone == "core")
    return dict(cls=cls_idx, lod=lod, proto=proto, scale=scale, qw=q[:, 0], qx=q[:, 1], qy=q[:, 2], qz=q[:, 3],
                burial=burial, z=z, exposed_h=exposed, collide=collide)


def _rotated_heights(library: RockLibrary, scale, q, proto, lod) -> np.ndarray:
    """Vertical extent of each rotated, scaled visual LOD."""
    out = np.empty(len(scale))
    R = quat_to_matrix(q)
    key = np.asarray(proto, dtype=np.int64) * 16 + np.asarray(lod, dtype=np.int64)
    for k in np.unique(key):
        sel = np.nonzero(key == k)[0]
        pts = library.contact_points(int(k // 16), int(k % 16))
        Vz = np.einsum("kj,mj->km", R[sel, 2, :], pts) * np.asarray(scale)[sel, None]
        out[sel] = Vz.max(axis=1) - Vz.min(axis=1)
    return out


def _contact_heights(ground, library: RockLibrary, x, y, scale, q, proto, lod, burial,
                     chunk: int = 400_000, min_sink=None, max_sink_frac: float = 1.0,
                     return_burial: bool = False):
    """Centre heights from the contact rule, and exposed heights (see module docstring).

    The rock sinks by ``burial * c_rot`` below first contact, at least by ``min_sink`` [m] (bedding;
    capped at ``max_sink_frac * c_rot``). With ``return_burial`` also the effective burial fraction."""
    n = len(x)
    z = np.empty(n)
    exposed = np.empty(n)
    burial = np.asarray(burial, dtype=np.float64)
    burial_eff = burial.copy()
    R = quat_to_matrix(q)
    g_centre = ground.height(x, y)
    key = proto.astype(np.int64) * 16 + lod
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
            g = ground.height(gx.ravel(), gy.ravel()).reshape(gx.shape)
            # clearance of vertex v is z + Vz_v - g_v; the lowest one touches: min_v clearance = 0
            zc = np.max(g - V[:, :, 2], axis=1)
            zmin = V[:, :, 2].min(axis=1)
            zmax = V[:, :, 2].max(axis=1)
            sink = burial[ss] * (zmax - zmin)
            if min_sink is not None:
                sink = np.maximum(sink, np.minimum(np.asarray(min_sink)[ss], max_sink_frac * (zmax - zmin)))
                burial_eff[ss] = sink / np.maximum(zmax - zmin, 1e-300)
            zc = zc - sink
            z[ss] = zc
            exposed[ss] = zc + zmax - g_centre[ss]
    return (z, exposed, burial_eff) if return_burial else (z, exposed)
