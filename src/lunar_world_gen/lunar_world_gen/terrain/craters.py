"""Synthetic crater population and crater relief K(x, y) (docs/DESIGN.md section 4).

Population
    Cumulative size-frequency distribution ``N(>=D) = C D^-b`` per m^2 (config
    ``terrain.craters.sfd_C_per_m2`` / ``sfd_b``). Diameters are drawn by inverse transform between
    ``D_lo`` and ``D_hi``; centres are uniform over the zone rectangle plus a margin; craters with
    ``fade_D_m[0] < D < fade_D_m[1]`` (8-16 m) are kept with a probability falling linearly from 1 to 0
    because the band-limited DTM already resolves them.

    * ``big``   : ``D_min_m.ring <= D <= D_max_m`` over the ring square (500 m) + margin  -> K_big
    * ``small`` : ``D_min_m.core <= D <  D_min_m.ring`` over the core square (100 m) + margin -> K_small

    Margin: DESIGN says ``2.5 * D_max / 2``. Degraded craters are widened (``R_eff`` below), so the
    margin used here is ``ejecta_extent_R * (1 + DEGRADED_WIDENING) * D_hi / 2``. It is a superset of the
    DESIGN margin; the density per m^2 is unchanged. (Elliptical craters reach ``R / sqrt(axis_ratio)``
    along their major axis; the few centred just outside the margin could touch only the zone's
    edge-fade band, where the crater term is weighted to 0, so the margin is kept and the population
    draws stay identical to version 1.)

Freshness and shape (per crater)
    ``rho = exp(-u)``, ``u ~ U(0, u_max)``; depth ``d = rho * d_over_D_fresh * D * v_d``; rim crest height
    ``h = rho * rim_height_over_D_fresh * D * v_h``; effective (equal-area) radius
    ``R = D/2 * (1 + 0.15 * (1 - rho))`` (degraded craters are slightly wider); interior exponent
    ``n = 1 + rho * v_n``.

    Shape variation (FID-16, config ``terrain.craters.shape``; drawn from the separate stream
    ``terrain.craters.<which>.shape`` so the population draws - positions, sizes, freshness - are
    unchanged): ``v_d ~ U(1 -+ d_spread)`` (0.15: fresh d/D 0.11-0.15, calibration fresh_A 0.11-0.17),
    ``v_h ~ U(1 -+ rim_spread)`` (0.2: fresh rim 0.024-0.036 D, calibration 0.022-0.04),
    ``v_n ~ U(1 -+ n_spread)`` (0.1), an axis ratio ``e ~ U(axis_ratio_min, 1)`` (0.85) with a uniform
    orientation ``psi`` (semi-axes ``R / sqrt(e)`` and ``R sqrt(e)``, area preserved), and a low-order
    azimuthal rim modulation ``m(az) = 1 + a2 cos 2(az - p2) + a3 cos 3(az - p3)`` with
    ``a2 ~ U(0, rim_mod_a2_max)`` (0.2) and ``a3 ~ U(0, rim_mod_a3_max)`` (0.1): the rim crest varies by up
    to +-30 % around the crater (asymmetric rim and ejecta). All of it is deterministic per seed.

Profile (normalised elliptical radius ``t = |A (p - c)| / R``, azimuth ``az`` in the crater frame)
    * interior  ``z_in(t)  = -d + (d + hm) * t^n``,     ``hm = hp * m(az)``
    * exterior  ``z_out(t) = hm * (t^e - q) / (1 - q)``, ``e = ejecta_exponent (-3)``,
      ``q = ejecta_extent_R^e``, for ``t < ejecta_extent_R`` and 0 beyond
    * ``z = smin(z_in, z_out, k)`` with the quadratic polynomial smooth minimum
      ``smin(a, b, k) = min(a, b) - k/4 * max(k - |a - b|, 0)^2 / k^2`` and
      ``k = rim_rounding_k_over_h * hm + 1e-4`` (0.5 by default).

    FID-05: the smooth minimum lowers the crest below the corner height; with DESIGN's ``k = 2 h`` the
    crest was only 0.56-0.69 h (0.021 D for the freshest craters instead of 0.03 D). Now the rounding is
    narrower (``k = 0.5 hp``: the crest is rounded over about +-4 % of R) and the rim parameter ``hp`` is
    solved per crater (``solve_rim_parameter``) so that the rounded crest of the unmodulated profile is
    exactly ``h`` (``hp`` = 1.10-1.13 h; the ejecta flank scales with it). The azimuthal mean of the
    modulated crest stays at h within a few per cent.

    The profile is continuous everywhere (C1 except at the centre of non-paraboloid craters and a tiny
    slope kink of the ejecta at ``2.5 R``). The config keys ``rim_width_over_R``,
    ``ejecta_fraction_of_rim`` and ``fresh_profile`` are not used (DESIGN section 4 fixes the shape).

Relief
    ``K(x, y) = sum_i z_i(p)`` (pure superposition, so every zone and every resolution sees exactly the
    same function). Evaluation is numba-parallel with a uniform-cell bucket index: every crater is
    registered in all cells its influence disc's bounding box overlaps (radius ``ejecta_extent_R *
    R / sqrt(e)``), and a point only visits the craters of its own cell. Grid and point evaluation share
    the same per-point routine, so identical coordinates give bit-identical results.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numba as nb
import numpy as np

from ..config import Config
from ..config import rng as cfg_rng
from ..grid import Grid
from ..layout import Layout

CRATERS_VERSION = "2"      # 2: FID-05 crest-exact rim (hp solve, k = 0.5 hp) + FID-16 shape variation
DEGRADED_WIDENING = 0.15   # R_eff = R * (1 + 0.15 * (1 - rho))           (DESIGN 4)
SMIN_K_FLOOR_M = 1e-4
ID_OFFSET = {"big": 0, "small": 1_000_000}   # ids are unique across the two sets
WHICH = ("big", "small")

#: defaults of ``terrain.craters.shape`` (FID-16) and ``terrain.craters.rim_rounding_k_over_h`` (FID-05)
SHAPE_DEFAULTS = {"d_spread": 0.15, "rim_spread": 0.2, "n_spread": 0.1, "axis_ratio_min": 0.85,
                  "rim_mod_a2_max": 0.2, "rim_mod_a3_max": 0.1}
RIM_ROUNDING_K_OVER_H = 0.5

_FIELDS = ("x", "y", "D", "rho", "d", "h", "R", "id")
#: per-crater shape fields (defaults reproduce a circular crater with hp = h, n = 1 + rho)
_SHAPE_FIELDS = ("hp", "n", "axis", "theta", "a2", "p2", "a3", "p3")


# ---------------------------------------------------------------------------------------- numba core
@nb.njit(cache=False, inline="always")
def _smin_profile(t: float, d: float, hm: float, nexp: float, k: float, e: float, q: float) -> float:
    """Radial profile (normalised radius t > 0, t < ejecta extent) for rim parameter hm and rounding k."""
    z_in = -d + (d + hm) * t ** nexp
    if hm <= 0.0:
        z_out = 0.0
    else:
        z_out = hm * (t ** e - q) / (1.0 - q)
    diff = z_in - z_out
    m = z_in if diff < 0.0 else z_out
    s = k - abs(diff)
    if s <= 0.0:
        return m
    s = s / k
    return m - 0.25 * k * s * s


@nb.njit(cache=False, inline="always")
def _crater_z(dx: float, dy: float, d: float, hp: float, nexp: float, kf: float, cps: float, sps: float,
              iRa: float, iRb: float, a2c: float, a2s: float, a3c: float, a3s: float,
              ext: float, e: float, q: float) -> float:
    """Height of one crater at offset (dx, dy) from its centre (profile in the module docstring)."""
    u = dx * cps + dy * sps
    v = -dx * sps + dy * cps
    un = u * iRa
    vn = v * iRb
    t2 = un * un + vn * vn
    if t2 >= ext * ext:
        return 0.0
    t = math.sqrt(t2)
    if t < 1e-12:
        return -d             # z_out -> +inf at the centre, smin = z_in = -d
    c = un / t
    s = vn / t
    c2 = c * c - s * s
    s2 = 2.0 * c * s
    c3 = c * (4.0 * c * c - 3.0)
    s3 = s * (3.0 - 4.0 * s * s)
    m = 1.0 + a2c * c2 + a2s * s2 + a3c * c3 + a3s * s3
    hm = hp * m
    return _smin_profile(t, d, hm, nexp, kf * m + SMIN_K_FLOOR_M, e, q)


@nb.njit(cache=False, inline="always")
def _relief_at(px: float, py: float, cx, cy, d, hp, nexp, kf, cps, sps, iRa, iRb, a2c, a2s, a3c, a3s, rI2,
               ext: float, e: float, q: float, gx0: float, gy0: float, cs: float, ncx: int, ncy: int,
               start, items) -> float:
    fx = math.floor((px - gx0) / cs)
    fy = math.floor((py - gy0) / cs)
    if fx < 0.0 or fy < 0.0 or fx >= ncx or fy >= ncy:
        return 0.0
    c = int(fy) * ncx + int(fx)
    acc = 0.0
    for t in range(start[c], start[c + 1]):
        j = items[t]
        dx = px - cx[j]
        dy = py - cy[j]
        if dx * dx + dy * dy < rI2[j]:
            acc += _crater_z(dx, dy, d[j], hp[j], nexp[j], kf[j], cps[j], sps[j], iRa[j], iRb[j],
                             a2c[j], a2s[j], a3c[j], a3s[j], ext, e, q)
    return acc


@nb.njit(cache=False, parallel=True)
def _relief_points_kernel(px, py, out, cx, cy, d, hp, nexp, kf, cps, sps, iRa, iRb, a2c, a2s, a3c, a3s, rI2,
                          ext, e, q, gx0, gy0, cs, ncx, ncy, start, items):
    for p in nb.prange(px.size):
        out[p] = _relief_at(px[p], py[p], cx, cy, d, hp, nexp, kf, cps, sps, iRa, iRb, a2c, a2s, a3c, a3s,
                            rI2, ext, e, q, gx0, gy0, cs, ncx, ncy, start, items)


@nb.njit(cache=False, parallel=True)
def _relief_grid_kernel(xs, ys, out, cx, cy, d, hp, nexp, kf, cps, sps, iRa, iRb, a2c, a2s, a3c, a3s, rI2,
                        ext, e, q, gx0, gy0, cs, ncx, ncy, start, items):
    for jj in nb.prange(ys.size):
        y = ys[jj]
        for ii in range(xs.size):
            out[jj, ii] = _relief_at(xs[ii], y, cx, cy, d, hp, nexp, kf, cps, sps, iRa, iRb, a2c, a2s, a3c,
                                     a3s, rI2, ext, e, q, gx0, gy0, cs, ncx, ncy, start, items)


@nb.njit(cache=False)
def _build_cells(cx, cy, rI, gx0, gy0, cs, ncx, ncy):
    n = cx.size
    counts = np.zeros(ncx * ncy + 1, dtype=np.int64)
    for j in range(n):
        ix0 = max(int(math.floor((cx[j] - rI[j] - gx0) / cs)), 0)
        ix1 = min(int(math.floor((cx[j] + rI[j] - gx0) / cs)), ncx - 1)
        iy0 = max(int(math.floor((cy[j] - rI[j] - gy0) / cs)), 0)
        iy1 = min(int(math.floor((cy[j] + rI[j] - gy0) / cs)), ncy - 1)
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                counts[iy * ncx + ix + 1] += 1
    start = np.cumsum(counts)
    items = np.empty(start[-1], dtype=np.int32)
    cursor = start[:-1].copy()
    for j in range(n):            # increasing j -> deterministic order inside every cell
        ix0 = max(int(math.floor((cx[j] - rI[j] - gx0) / cs)), 0)
        ix1 = min(int(math.floor((cx[j] + rI[j] - gx0) / cs)), ncx - 1)
        iy0 = max(int(math.floor((cy[j] - rI[j] - gy0) / cs)), 0)
        iy1 = min(int(math.floor((cy[j] + rI[j] - gy0) / cs)), ncy - 1)
        for iy in range(iy0, iy1 + 1):
            for ix in range(ix0, ix1 + 1):
                c = iy * ncx + ix
                items[cursor[c]] = j
                cursor[c] += 1
    return start, items


# ---------------------------------------------------------------------------------------- rim solve (FID-05)
@nb.njit(cache=False)
def _crest(d: float, hp: float, nexp: float, k: float, ext: float, e: float, q: float) -> float:
    """Crest (maximum) of the circular profile with rim parameter hp: it lies in the outer half of the
    smooth-min blend, t in [1, t_b] with z_in(t_b) - z_out(t_b) = k (z_in rises, z_out falls there).
    Found by bisection on the sign of the derivative (unimodal on that interval)."""
    if hp <= 0.0:
        return 0.0
    # t_b: end of the blend (difference z_in - z_out increases monotonically for t > 1)
    lo, hi = 1.0, ext
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        zin = -d + (d + hp) * mid ** nexp
        zout = hp * (mid ** e - q) / (1.0 - q)
        if zin - zout < k:
            lo = mid
        else:
            hi = mid
    tb = hi
    a, b = 1.0, tb
    for _ in range(80):
        m1 = a + (b - a) * 0.5 - (b - a) * 1e-6
        m2 = a + (b - a) * 0.5 + (b - a) * 1e-6
        if _smin_profile(m1, d, hp, nexp, k, e, q) < _smin_profile(m2, d, hp, nexp, k, e, q):
            a = m1
        else:
            b = m2
    t = 0.5 * (a + b)
    best = _smin_profile(t, d, hp, nexp, k, e, q)
    z1 = _smin_profile(1.0, d, hp, nexp, k, e, q)
    return best if best > z1 else z1


@nb.njit(cache=False, parallel=True)
def _solve_hp_kernel(d, h, nexp, kfac, ext, e, q, out):
    for i in nb.prange(d.size):
        if h[i] <= 0.0:
            out[i] = h[i]
            continue
        lo = h[i]
        hi = 4.0 * h[i]
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if _crest(d[i], mid, nexp[i], kfac * mid + SMIN_K_FLOOR_M, ext, e, q) < h[i]:
                lo = mid
            else:
                hi = mid
        out[i] = 0.5 * (lo + hi)


def solve_rim_parameter(d, h, nexp, k_over_h: float = RIM_ROUNDING_K_OVER_H, ejecta_extent_R: float = 2.5,
                        ejecta_exponent: float = -3.0) -> np.ndarray:
    """Rim parameter hp per crater such that the rounded crest of the circular profile equals h (FID-05)."""
    d = np.ascontiguousarray(np.broadcast_to(np.asarray(d, np.float64), np.shape(h)), dtype=np.float64)
    h = np.ascontiguousarray(h, dtype=np.float64)
    nexp = np.ascontiguousarray(np.broadcast_to(np.asarray(nexp, np.float64), h.shape), dtype=np.float64)
    out = np.empty_like(h)
    if h.size:
        _solve_hp_kernel(d.ravel(), h.ravel(), nexp.ravel(), float(k_over_h), float(ejecta_extent_R),
                         float(ejecta_exponent), float(ejecta_extent_R) ** float(ejecta_exponent), out.ravel())
    return out


def crest_height(d, hp, nexp, k_over_h: float = RIM_ROUNDING_K_OVER_H, ejecta_extent_R: float = 2.5,
                 ejecta_exponent: float = -3.0) -> float:
    """Crest of one circular profile (for tests and reports)."""
    q = ejecta_extent_R ** ejecta_exponent
    return float(_crest(float(d), float(hp), float(nexp), float(k_over_h) * float(hp) + SMIN_K_FLOOR_M,
                        float(ejecta_extent_R), float(ejecta_exponent), q))


# ---------------------------------------------------------------------------------------- data types
@dataclass
class CraterIndex:
    """Uniform-cell bucket index (cell -> craters whose influence-disc bounding box overlaps it)."""

    gx0: float
    gy0: float
    cell: float
    ncx: int
    ncy: int
    start: np.ndarray   # int64 (ncx*ncy + 1)
    items: np.ndarray   # int32 crater indices

    @property
    def n_entries(self) -> int:
        return int(self.items.size)


@dataclass
class CraterSet:
    """A crater population; arrays share one length. ``R`` is the effective (equal-area) radius [m],
    ``h`` the rim crest height, ``hp`` the profile's rim parameter (solved so the rounded crest is h),
    ``n`` the interior exponent, ``axis`` the axis ratio (<= 1) with major-axis orientation ``theta``
    [rad], ``a2, p2, a3, p3`` the azimuthal rim modulation (see the module docstring)."""

    x: np.ndarray
    y: np.ndarray
    D: np.ndarray
    rho: np.ndarray
    d: np.ndarray
    h: np.ndarray
    R: np.ndarray
    id: np.ndarray
    which: str = ""
    ejecta_extent_R: float = 2.5
    ejecta_exponent: float = -3.0
    meta: dict = field(default_factory=dict)
    hp: np.ndarray | None = None
    n: np.ndarray | None = None
    axis: np.ndarray | None = None
    theta: np.ndarray | None = None
    a2: np.ndarray | None = None
    p2: np.ndarray | None = None
    a3: np.ndarray | None = None
    p3: np.ndarray | None = None
    k_over_h: float = RIM_ROUNDING_K_OVER_H
    _index: CraterIndex | None = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        for name in ("x", "y", "D", "rho", "d", "h", "R"):
            setattr(self, name, np.ascontiguousarray(getattr(self, name), dtype=np.float64))
        self.id = np.ascontiguousarray(self.id, dtype=np.int64)
        n = self.x.size
        defaults = {"hp": self.h, "n": 1.0 + self.rho, "axis": np.ones(n), "theta": np.zeros(n),
                    "a2": np.zeros(n), "p2": np.zeros(n), "a3": np.zeros(n), "p3": np.zeros(n)}
        for name in _SHAPE_FIELDS:
            v = getattr(self, name)
            setattr(self, name, np.ascontiguousarray(defaults[name] if v is None else v, dtype=np.float64))
        for name in _FIELDS + _SHAPE_FIELDS:
            if getattr(self, name).shape != (n,):
                raise ValueError(f"CraterSet.{name} has shape {getattr(self, name).shape}, expected ({n},)")
        if n and (np.any(self.axis <= 0.0) or np.any(self.axis > 1.0)):
            raise ValueError("CraterSet.axis must be in (0, 1]")
        self.k_over_h = float(self.k_over_h)

    def __len__(self) -> int:
        return int(self.x.size)

    # ------------------------------------------------------------------ derived quantities
    @property
    def influence_radius(self) -> np.ndarray:
        """Radius beyond which a crater contributes exactly 0 [m] (major semi-axis x ejecta extent)."""
        return self.ejecta_extent_R * self.R / np.sqrt(self.axis)

    @property
    def n_exponent(self) -> np.ndarray:
        return self.n

    @property
    def k_smooth(self) -> np.ndarray:
        """Smooth-min width of the unmodulated profile, k = k_over_h * hp + 1e-4."""
        return self.k_over_h * self.hp + SMIN_K_FLOOR_M

    def subset(self, mask: np.ndarray) -> "CraterSet":
        mask = np.asarray(mask)
        kw = {f: getattr(self, f)[mask] for f in _SHAPE_FIELDS}
        return CraterSet(*(getattr(self, f)[mask] for f in _FIELDS), which=self.which,
                         ejecta_extent_R=self.ejecta_extent_R, ejecta_exponent=self.ejecta_exponent,
                         meta=dict(self.meta), k_over_h=self.k_over_h, **kw)

    def count_ge(self, D_values, rect: tuple[float, float, float, float] | None = None) -> np.ndarray:
        """Number of craters with D >= each value (centres inside rect=(x0, x1, y0, y1) if given)."""
        D = self.D
        if rect is not None:
            x0, x1, y0, y1 = rect
            D = D[(self.x >= x0) & (self.x <= x1) & (self.y >= y0) & (self.y <= y1)]
        Ds = np.sort(D)
        return Ds.size - np.searchsorted(Ds, np.asarray(D_values, dtype=np.float64), side="left")

    # ------------------------------------------------------------------ index
    def index(self, cell: float | None = None) -> CraterIndex:
        """Bucket index (built once and cached on the object)."""
        if self._index is not None and (cell is None or abs(cell - self._index.cell) < 1e-12):
            return self._index
        rI = self.influence_radius
        if cell is None:
            cell = float(np.clip(2.0 * np.median(rI), 0.25, 4.0)) if len(self) else 1.0
        if len(self):
            gx0 = float(np.floor(np.min(self.x - rI))) - cell
            gy0 = float(np.floor(np.min(self.y - rI))) - cell
            ncx = int(np.ceil((np.max(self.x + rI) - gx0) / cell)) + 2
            ncy = int(np.ceil((np.max(self.y + rI) - gy0) / cell)) + 2
        else:
            gx0 = gy0 = 0.0
            ncx = ncy = 1
        start, items = _build_cells(self.x, self.y, rI, gx0, gy0, float(cell), ncx, ncy)
        self._index = CraterIndex(gx0, gy0, float(cell), ncx, ncy, start, items)
        return self._index

    def _kernel_args(self) -> tuple:
        ix = self.index()
        ext = float(self.ejecta_extent_R)
        e = float(self.ejecta_exponent)
        q = ext ** e
        rI = self.influence_radius
        sq = np.sqrt(self.axis)
        iRa = sq / self.R                 # 1 / (R / sqrt(e)): major semi-axis along the crater's u axis
        iRb = 1.0 / (self.R * sq)         # 1 / (R sqrt(e)):  minor semi-axis along v
        c = np.ascontiguousarray
        return (self.x, self.y, self.d, self.hp, self.n, c(self.k_over_h * self.hp), c(np.cos(self.theta)),
                c(np.sin(self.theta)), c(iRa), c(iRb), c(self.a2 * np.cos(2 * self.p2)),
                c(self.a2 * np.sin(2 * self.p2)), c(self.a3 * np.cos(3 * self.p3)), c(self.a3 * np.sin(3 * self.p3)),
                c(rI * rI), ext, e, q, ix.gx0, ix.gy0, ix.cell, ix.ncx, ix.ncy, ix.start, ix.items)

    # ------------------------------------------------------------------ io
    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_name(path.name + ".tmp.npz")
        np.savez(tmp, **{f: getattr(self, f) for f in _FIELDS + _SHAPE_FIELDS}, which=np.array(self.which),
                 ejecta_extent_R=np.array(self.ejecta_extent_R), ejecta_exponent=np.array(self.ejecta_exponent),
                 k_over_h=np.array(self.k_over_h),
                 meta_json=np.array(json.dumps(self.meta, default=_json_default)))
        tmp.replace(path)

    @classmethod
    def load(cls, path: str | Path) -> "CraterSet":
        with np.load(Path(path), allow_pickle=False) as f:
            meta = json.loads(str(f["meta_json"])) if "meta_json" in f else {}
            kw = {k: f[k] for k in _SHAPE_FIELDS if k in f.files}
            if "k_over_h" in f.files:
                kw["k_over_h"] = float(f["k_over_h"])
            else:                      # version-1 file: DESIGN profile k = 2 h, circular
                kw["k_over_h"] = 2.0
            return cls(*(f[k] for k in _FIELDS), which=str(f["which"]),
                       ejecta_extent_R=float(f["ejecta_extent_R"]), ejecta_exponent=float(f["ejecta_exponent"]),
                       meta=meta, **kw)


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serialisable: {type(o)}")


# ---------------------------------------------------------------------------------------- sampling
def keep_probability(D: np.ndarray, fade_D: tuple[float, float]) -> np.ndarray:
    """Probability that a crater of diameter D is kept: 1 below fade_D[0], linear to 0 at fade_D[1]."""
    f0, f1 = float(fade_D[0]), float(fade_D[1])
    D = np.asarray(D, dtype=np.float64)
    if f1 <= f0:
        return (D < f0).astype(np.float64)
    return np.clip((f1 - D) / (f1 - f0), 0.0, 1.0)


def expected_count_ge(D: float, C: float, b: float, D_lo: float, D_hi: float,
                      fade_D: tuple[float, float], area_m2: float) -> float:
    """Expected number of sampled craters with diameter >= D (truncation and fade included)."""
    lo = max(D, D_lo)
    if lo >= D_hi:
        return 0.0
    # integrate p(D') * dN over [lo, D_hi]; dN = C b D'^(-b-1) dD'
    grid = np.geomspace(lo, D_hi, 4001)
    dens = C * b * grid ** (-b - 1.0) * keep_probability(grid, fade_D)
    return float(np.trapezoid(dens, grid) * area_m2)


def sample_population(rng: np.random.Generator, C: float, b: float, D_lo: float, D_hi: float,
                      rect: tuple[float, float, float, float], fade_D: tuple[float, float],
                      u_max: float) -> dict[str, np.ndarray]:
    """Draw (x, y, D, rho) of a Poisson crater population; see the module docstring.

    The draw order is fixed (count, D, x, y, keep, freshness) so a seed always gives the same set.
    """
    x0, x1, y0, y1 = rect
    area = (x1 - x0) * (y1 - y0)
    a_lo, a_hi = D_lo ** -b, D_hi ** -b
    n = int(rng.poisson(C * (a_lo - a_hi) * area))
    u = rng.random(n)
    D = (a_lo - u * (a_lo - a_hi)) ** (-1.0 / b)
    x = rng.uniform(x0, x1, n)
    y = rng.uniform(y0, y1, n)
    keep = rng.random(n) < keep_probability(D, fade_D)
    rho = np.exp(-rng.uniform(0.0, u_max, n))
    return {"x": x[keep], "y": y[keep], "D": D[keep], "rho": rho[keep], "n_drawn": n}


def shape_params(cfg: Config) -> dict:
    """FID-16 shape variation and FID-05 rim rounding parameters (config with defaults)."""
    cc = cfg.terrain.craters
    s = dict(SHAPE_DEFAULTS)
    s.update({k: float(v) for k, v in dict(cc.get("shape") or {}).items()})
    unknown = set(s) - set(SHAPE_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown terrain.craters.shape keys {sorted(unknown)}")
    if not 0.0 < s["axis_ratio_min"] <= 1.0:
        raise ValueError("terrain.craters.shape.axis_ratio_min must be in (0, 1]")
    if s["rim_mod_a2_max"] + s["rim_mod_a3_max"] >= 1.0:
        raise ValueError("rim_mod_a2_max + rim_mod_a3_max must be < 1")
    for k in ("d_spread", "rim_spread", "n_spread"):
        if not 0.0 <= s[k] < 1.0:
            raise ValueError(f"terrain.craters.shape.{k} must be in [0, 1)")
    s["rim_rounding_k_over_h"] = float(cc.get("rim_rounding_k_over_h", RIM_ROUNDING_K_OVER_H))
    if s["rim_rounding_k_over_h"] <= 0:
        raise ValueError("terrain.craters.rim_rounding_k_over_h must be > 0")
    return s


def crater_params(cfg: Config, which: str) -> dict:
    """The parameters that define one crater set (also used as its cache key)."""
    if which not in WHICH:
        raise ValueError(f"which must be one of {WHICH}, got {which!r}")
    cc = cfg.terrain.craters
    lay = Layout(cfg)
    if which == "big":
        D_lo, D_hi = float(cc.D_min_m.ring), float(cc.D_max_m)
        r = lay.zone_rect("ring")
    else:
        D_lo, D_hi = float(cc.D_min_m.core), float(cc.D_min_m.ring)
        r = lay.zone_rect("core")
    ext = float(cc.get("ejecta_extent_R", 2.5))
    shape = shape_params(cfg)
    # the sampling margin is kept at its version-1 value so the population draws (positions, sizes,
    # freshness) are unchanged by the shape variation; an elliptical crater centred just outside it can
    # reach at most (1/sqrt(axis_ratio_min) - 1) * 2.875 R (2 m for D = 16 m, 12 cm for D = 1 m) into the
    # zone, i.e. into the zone's edge-fade band (w_ring / w_core = 0 on the zone boundary)
    margin = ext * (1.0 + DEGRADED_WIDENING) * D_hi / 2.0
    return {
        "version": CRATERS_VERSION, "which": which, "seed": int(cfg.world.seed),
        "C_per_m2": float(cc.sfd_C_per_m2), "b": float(cc.sfd_b), "D_lo": D_lo, "D_hi": D_hi,
        "fade_D": [float(v) for v in cc.fade_D_m], "u_max": float(cc.freshness.u_max),
        "d_over_D_fresh": float(cc.d_over_D_fresh), "rim_height_over_D_fresh": float(cc.rim_height_over_D_fresh),
        "ejecta_extent_R": ext, "ejecta_exponent": float(cc.get("ejecta_exponent", -3.0)),
        "shape": shape,
        "zone_rect": [r.x0, r.x1, r.y0, r.y1], "margin_m": margin,
        "rect": [r.x0 - margin, r.x1 + margin, r.y0 - margin, r.y1 + margin],
    }


def sample_craters(cfg: Config, which: str) -> CraterSet:
    """Sample the 'big' (K_big) or 'small' (K_small) crater set of a config (deterministic)."""
    p = crater_params(cfg, which)
    rng = cfg_rng(cfg, f"terrain.craters.{which}")
    pop = sample_population(rng, p["C_per_m2"], p["b"], p["D_lo"], p["D_hi"], tuple(p["rect"]),
                            tuple(p["fade_D"]), p["u_max"])
    D, rho = pop["D"], pop["rho"]
    n = D.size
    s = p["shape"]
    srng = cfg_rng(cfg, f"terrain.craters.{which}.shape")     # separate stream: population draws unchanged
    v_d = srng.uniform(1.0 - s["d_spread"], 1.0 + s["d_spread"], n)
    v_h = srng.uniform(1.0 - s["rim_spread"], 1.0 + s["rim_spread"], n)
    v_n = srng.uniform(1.0 - s["n_spread"], 1.0 + s["n_spread"], n)
    axis = srng.uniform(s["axis_ratio_min"], 1.0, n)
    theta = srng.uniform(0.0, math.pi, n)
    a2 = srng.uniform(0.0, s["rim_mod_a2_max"], n)
    p2 = srng.uniform(0.0, math.pi, n)
    a3 = srng.uniform(0.0, s["rim_mod_a3_max"], n)
    p3 = srng.uniform(0.0, 2.0 * math.pi / 3.0, n)
    d = rho * p["d_over_D_fresh"] * D * v_d
    h = rho * p["rim_height_over_D_fresh"] * D * v_h
    nexp = 1.0 + rho * v_n
    kfac = s["rim_rounding_k_over_h"]
    hp = solve_rim_parameter(d, h, nexp, kfac, p["ejecta_extent_R"], p["ejecta_exponent"])
    meta = dict(p)
    meta["n_drawn_before_fade"] = int(pop["n_drawn"])
    meta["hp_over_h"] = {"min": float(np.min(hp / h)) if n else None, "max": float(np.max(hp / h)) if n else None}
    return CraterSet(
        x=pop["x"], y=pop["y"], D=D, rho=rho, d=d, h=h,
        R=0.5 * D * (1.0 + DEGRADED_WIDENING * (1.0 - rho)),
        id=ID_OFFSET[which] + np.arange(n, dtype=np.int64),
        which=which, ejecta_extent_R=p["ejecta_extent_R"], ejecta_exponent=p["ejecta_exponent"], meta=meta,
        hp=hp, n=nexp, axis=axis, theta=theta, a2=a2, p2=p2, a3=a3, p3=p3, k_over_h=kfac)


# ---------------------------------------------------------------------------------------- relief
def crater_profile(r: np.ndarray, R: float, d: float, h: float, rho: float,
                   ejecta_extent_R: float = 2.5, ejecta_exponent: float = -3.0, hp: float | None = None,
                   n: float | None = None, k_over_h: float = RIM_ROUNDING_K_OVER_H) -> np.ndarray:
    """Radial profile of a single circular, unmodulated crater (numpy twin of the kernel, for tests and
    plots). ``hp`` defaults to the solved rim parameter (crest = h), ``n`` to 1 + rho."""
    nexp = 1.0 + rho if n is None else float(n)
    if hp is None:
        hp = float(solve_rim_parameter(np.array([d]), np.array([h]), np.array([nexp]), k_over_h,
                                       ejecta_extent_R, ejecta_exponent)[0])
    cs = CraterSet(x=[0.0], y=[0.0], D=[2 * R], rho=[rho], d=[d], h=[h], R=[R], id=[0],
                   ejecta_extent_R=ejecta_extent_R, ejecta_exponent=ejecta_exponent, hp=[hp], n=[nexp],
                   k_over_h=k_over_h)
    r = np.asarray(r, dtype=np.float64)
    return crater_relief(cs, r, np.zeros_like(r))


def crater_relief(craters: CraterSet, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """K(x, y) at arbitrary points (float64, same shape as x)."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    shape = np.broadcast_shapes(x.shape, y.shape)
    px = np.ascontiguousarray(np.broadcast_to(x, shape).ravel())
    py = np.ascontiguousarray(np.broadcast_to(y, shape).ravel())
    out = np.zeros(px.size, dtype=np.float64)
    if len(craters) and px.size:
        _relief_points_kernel(px, py, out, *craters._kernel_args())
    return out.reshape(shape)


def crater_relief_grid(craters: CraterSet, grid: Grid, lattice_exact: bool = True) -> np.ndarray:
    """K at every node of ``grid`` (float64 array shaped like grid.z).

    With ``lattice_exact`` (default) node coordinates are computed from integer 1 cm lattice
    coordinates (``(l0 + i * step) * 0.01``) when the grid is on the lattice, which is exactly how
    compose and stitching form coordinates, so values are bit-identical to ``crater_relief`` there.
    """
    xs, ys = node_coordinates(grid, lattice_exact)
    out = np.zeros((ys.size, xs.size), dtype=np.float64)
    if len(craters) and out.size:
        _relief_grid_kernel(xs, ys, out, *craters._kernel_args())
    return out


def node_coordinates(grid: Grid, lattice_exact: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """Node x and y coordinates of a grid (lattice-exact when the grid is on the 1 cm lattice)."""
    if lattice_exact:
        try:
            lx0, ly0 = grid.lattice_origin()
            step = grid.lattice_step()
        except ValueError:
            return grid.xs(), grid.ys()
        from ..grid import from_lattice
        return (from_lattice(lx0 + step * np.arange(grid.nx, dtype=np.int64)),
                from_lattice(ly0 + step * np.arange(grid.ny, dtype=np.int64)))
    return grid.xs(), grid.ys()


def craters_path(cfg: Config, which: str) -> Path:
    return Layout(cfg).stage_dir("terrain") / "craters" / f"{which}.npz"


def get_craters(cfg: Config, which: str, rebuild: bool = False) -> CraterSet:
    """Cached crater set (build_dir/terrain/craters/<which>.npz); resampled if the parameters changed."""
    path = craters_path(cfg, which)
    params = crater_params(cfg, which)
    if path.exists() and not rebuild:
        cs = CraterSet.load(path)
        stored = {k: cs.meta.get(k) for k in params}
        if json.dumps(stored, sort_keys=True) == json.dumps(params, sort_keys=True):
            return cs
    cs = sample_craters(cfg, which)
    cs.save(path)
    return cs
