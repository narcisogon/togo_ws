"""Rock population: sizes and positions (docs/DESIGN.md section 5).

``sample_rocks(cfg, zone, ground, craters=None)`` returns the rock catalog of a zone
(``catalog.CATALOG_DTYPE``). Steps:

1. **Sizes** follow the broken power law ``rocks.sfd`` (cumulative N(>=D) per m^2, continuous at
   the breaks, anchored at N(>=1 m)), truncated to [D_min(zone), D_max]: core 2 cm - 3 m, ring
   >= 30 cm. Inverse-transform sampling. The range is split into two size bands at
   ``crater_enhancement.small_max_D_m`` (10 cm), "debris" below and "blocks" above, because the crater
   enhancement differs between them; each band has its own Poisson count and intensity.
2. **Intensity.** Poisson positions with intensity ``N_sfd * m(x, y) * e_band(x, y)``:

   * ``m`` is a log-normal density field ``exp(sigma_ln * G - sigma_ln^2 / 2)`` where G is a unit
     Gaussian random field with covariance ``exp(-r^2 / L^2)`` (L = ``corr_length_m``, i.e. the
     correlation falls to 1/e at L). One field is synthesised (FFT, 2.5 m grid) over the whole
     ring square, so core and ring see the same field.
   * ``e`` is the fresh-crater enhancement (FID-02), ``1 + max_c A_c p(r_c / R_c)`` over the fresh
     craters (rho >= ``min_rho``, default 0.73 = the calibration's "fresh" class) with
     D >= ``min_crater_D_m``. ``p`` is a smooth radial profile, continuous everywhere: it rises from
     ``floor_fraction`` at the centre to 1 at the rim (smoothstep over the wall and floor), then
     falls with the ejecta-thickness law ``(r/R)^-3`` (calibration 2.3) to 0 at ``outer_R`` x R.
     The amplitude is size gated as the calibration (2.4) says:

     - debris (D < 10 cm): ``A = (debris_factor - 1) * rho`` for every fresh crater, a mild boost
       (default <= 1.5x) of the cm fragments of small fresh craters in regolith;
     - blocks (D >= 10 cm): ``A = (block_factor - 1) * w_blocky(D_c) * rho`` with ``w_blocky``
       rising linearly from 0 at ``blocky_D_m[0]`` (20 m) to 1 at ``blocky_D_m[1]`` (80 m): only
       craters that dig through the ~7 m regolith carry block fields (``block_factor`` defaults to
       the old ``factor`` key). The 5-16 m craters of this world get no block rings.

     ``profile: annulus`` restores the old step function (``factor`` on ``annulus_R`` x R, all sizes).
     Crater sources: the ``craters`` argument (any object or dict with x, y, D, rho arrays, e.g.
     ``terrain.craters.CraterSet``; pass the same set for core and ring) plus the fresh DTM craters
     listed in the site report (``site_selection.yaml``: the 22.6 m crater at (32, -60)).
   * **Normalisation.** ``m * e_band`` is scaled by one constant per band so that its mean over the
     core is exactly 1. The core then holds the nominal LM-site totals on average (the validation
     compares core counts with ``N_sfd * core area`` at +-3 sigma) and the field and the craters only
     redistribute rocks. Without it the 100 m core, only ~3 correlation lengths wide, would shift its
     totals by +-20 % per realisation (seed 16: raw core mean 0.80). The same constants are used
     in the ring, so the intensity stays continuous.
   * The count of a band is Poisson with mean ``N_sfd(band) * Lambda``, ``Lambda`` = integral of the
     scaled intensity over the zone (stored in the metadata: exact expectations for validation);
     positions are drawn by thinning.
3. **Separation.** Rocks >= 10 cm are placed largest first; a rock whose centre is closer than
   ``min_separation_factor * (r1 + r2)`` to an already placed one is re-drawn from the intensity
   (up to 200 tries; failures are dropped and counted). Rocks < 10 cm are re-drawn if their centre
   falls inside ``min_separation_factor * r`` of a rock >= 10 cm.
4. **Placement** (class, prototype, orientation, burial, height): ``placement.place_rocks``.
5. **Interpenetration (FID-09).** After placement every pair of rocks whose bounding discs overlap is
   tested on the real geometry: a rock is re-drawn (new position from its band's intensity, placed
   again) if its centre lies inside the footprint of a larger rock (the vertical line through it hits
   the larger rock's convex hull), or, when the larger rock is >= ``rocks.overlap.pierce_min_D_m``
   (5 cm), if any of its visual-LOD vertices is inside the larger rock's hull or a hull vertex of the
   larger rock is inside its own hull. Repeated until no conflict is left (``max_rounds``; the rest
   is dropped and counted). Pebbles may still touch each other. ``avoid`` (e.g. the core catalog when
   sampling the ring) adds fixed rocks of another zone to the test.
6. **Core-edge band (FID-04, ring only).** The core population of rocks smaller than the ring's
   ``D_min`` (2-30 cm) continues outside the core with its density multiplied by a raised cosine
   of the distance to the core edge, 1 at the edge and 0 at ``rocks.edge_band.width_m``. These rows
   are ring rocks (zone 1, ``collide`` false, merged into the ring strips) with ids from
   ``BAND_ID_OFFSET``; their LOD is the class LOD of the core with ``rocks.edge_band.lod``
   overrides (pebbles LOD1 by default). With the default 6 m the band holds ~34 k rocks and ~0.53 M
   triangles on seed 16 (the density field outside the core averages ~1.26x the core mean; 8 m gave
   0.72 M).
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numba import njit

from ..config import config_hash, rng as cfg_rng
from ..grid import Grid
from ..layout import RING_SIDES, Layout, Rect
from .catalog import SFD_CHECK_D_M, ZONE_CODES, empty_catalog
from .library import RockLibrary
from .placement import as_ground, class_table, place_rocks, quat_to_matrix, size_class

SEPARATION_MIN_D_M = 0.10       # rocks this size and larger keep their distance (DESIGN 5)
FRESH_RHO_MIN = 0.73            # calibration generator_rule: ~10 % of craters have rho > 0.73
ID_ZONE_OFFSET = 10_000_000     # ring ids start here, so core + ring ids stay unique
BAND_ID_OFFSET = 12_000_000     # core-edge band rocks (ring zone, D < D_min ring; FID-04)
FIELD_SPACING_M = 2.5
RING_CHECK_D_M = (0.30, 0.50, 1.0, 2.0)

#: defaults of the optional config keys read here (the config may omit any of them). ``annulus_R`` is
#: read only by the legacy ``profile: annulus`` (DESIGN 5: factor 2 in 1-2 R); without this default the
#: legacy profile raised KeyError once the key was dropped from the config (final integration).
CRATER_ENHANCEMENT = {"profile": "smooth", "min_crater_D_m": 5.0, "min_rho": FRESH_RHO_MIN,
                      "small_max_D_m": 0.10, "debris_factor": 1.5, "blocky_D_m": [20.0, 80.0],
                      "outer_R": 2.5, "floor_fraction": 0.5, "annulus_R": [1.0, 2.0]}
OVERLAP = {"enabled": True, "pierce_min_D_m": 0.05, "max_rounds": 8, "tol_m": 1e-5}     # rocks.overlap
EDGE_BAND = {"enabled": True, "width_m": 6.0, "lod": {"pebble": 1}}                      # rocks.edge_band


def _section(cfg, key: str, default: dict) -> dict:
    out = dict(default)
    out.update(dict(cfg["rocks"].get(key) or {}))
    return out


def crater_settings(cfg) -> dict:
    """``rocks.crater_enhancement`` merged over the defaults; ``block_factor`` defaults to ``factor``."""
    ce = _section(cfg, "crater_enhancement", CRATER_ENHANCEMENT)
    if "block_factor" not in ce:
        ce["block_factor"] = float(ce.get("factor", 3.0))
    return ce


# ---------------------------------------------------------------------------------------------
# Size-frequency distribution
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class BrokenPowerLaw:
    """Cumulative N(>=D) = A_i * D^-b_i per m^2 on segment i = [lo_i, hi_i), continuous at breaks."""

    lo: tuple[float, ...]
    hi: tuple[float, ...]
    b: tuple[float, ...]
    A: tuple[float, ...]

    @classmethod
    def from_config(cls, cfg) -> "BrokenPowerLaw":
        sfd = cfg["rocks"]["sfd"]
        segs = sorted(([float(s[0]), float(s[1]), float(s[2])] for s in sfd["segments"]), key=lambda s: s[0])
        for s0, s1 in zip(segs[:-1], segs[1:]):
            if abs(s0[1] - s1[0]) > 1e-12:
                raise ValueError("rocks.sfd.segments must be contiguous")
        d_ref, n_ref = 1.0, float(sfd["A_ge_1m_per_m2"])
        r = next((i for i, s in enumerate(segs) if s[0] <= d_ref <= s[1]), None)
        if r is None:
            raise ValueError("rocks.sfd.segments must contain D = 1 m (the anchor)")
        A = [0.0] * len(segs)
        A[r] = n_ref * d_ref ** segs[r][2]
        for i in range(r + 1, len(segs)):
            d = segs[i][0]
            A[i] = A[i - 1] * d ** (-segs[i - 1][2]) * d ** segs[i][2]
        for i in range(r - 1, -1, -1):
            d = segs[i][1]
            A[i] = A[i + 1] * d ** (-segs[i + 1][2]) * d ** segs[i][2]
        return cls(tuple(s[0] for s in segs), tuple(s[1] for s in segs), tuple(s[2] for s in segs), tuple(A))

    def _seg(self, D: np.ndarray) -> np.ndarray:
        return np.clip(np.searchsorted(np.array(self.hi), D, side="right"), 0, len(self.b) - 1)

    def N(self, D) -> np.ndarray:
        """Cumulative number per m^2 of rocks with diameter >= D."""
        D = np.asarray(D, dtype=np.float64)
        i = self._seg(D)
        return np.array(self.A)[i] * D ** (-np.array(self.b)[i])

    def inverse(self, n) -> np.ndarray:
        """D such that N(>=D) = n."""
        n = np.asarray(n, dtype=np.float64)
        n_lo = self.N(np.array(self.lo))                  # N at segment lower bounds (decreasing)
        # segment i holds n in (N(hi_i), N(lo_i)]
        i = np.clip(np.searchsorted(-n_lo, -n, side="right") - 1, 0, len(self.b) - 1)
        return (n / np.array(self.A)[i]) ** (-1.0 / np.array(self.b)[i])

    def count_density(self, D_min: float, D_max: float) -> float:
        """Rocks per m^2 with D_min <= D < D_max."""
        return float(self.N(D_min) - self.N(D_max))

    def sample(self, rng: np.random.Generator, n: int, D_min: float, D_max: float) -> np.ndarray:
        n0, n1 = float(self.N(D_min)), float(self.N(D_max))
        u = 1.0 - rng.random(n)                                   # (0, 1]
        D = self.inverse(n1 + u * (n0 - n1))
        return np.clip(D, D_min, D_max)


def size_bands(cfg, D_min: float, D_max: float) -> list[tuple[float, float, str]]:
    """[(lo, hi, band)] covering [D_min, D_max]; band 'small' (debris) below
    ``crater_enhancement.small_max_D_m``, 'big' (blocks) above."""
    split = float(crater_settings(cfg)["small_max_D_m"])
    if D_min < split < D_max:
        return [(D_min, split, "small"), (split, D_max, "big")]
    return [(D_min, D_max, "small" if D_min < split else "big")]


# ---------------------------------------------------------------------------------------------
# Density field and crater enhancement
# ---------------------------------------------------------------------------------------------
def density_field(cfg) -> Grid:
    """Log-normal density multiplier m(x, y) on a 2.5 m grid covering the ring square (+10 m).

    Deterministic per config seed (stream 'rocks.density_field'); mean over the core = 1.
    """
    lay = Layout(cfg)
    df = cfg["rocks"]["density_field"]
    sigma = float(df["sigma_ln"])
    L = float(df["corr_length_m"])
    h = FIELD_SPACING_M
    ring = lay.zone_rect("ring")
    x0 = ring.x0 - 10.0
    n_out = int(round((ring.width + 20.0) / h)) + 1
    pad = int(np.ceil(3.0 * L / h))
    n_fft = int(2 ** np.ceil(np.log2(n_out + 2 * pad)))
    rng = cfg_rng(cfg, "rocks.density_field")
    white = rng.standard_normal((n_fft, n_fft))
    # white noise smoothed by a Gaussian kernel exp(-r^2 / (2 s^2)) has covariance exp(-r^2 / (4 s^2));
    # s = L / 2 gives exp(-r^2 / L^2). Transfer function of the kernel (f in cycles per cell):
    s = 0.5 * L / h
    k = np.fft.fftfreq(n_fft)
    K = np.exp(-2.0 * np.pi ** 2 * s ** 2 * (k[:, None] ** 2 + k[None, :] ** 2))
    G = np.fft.ifft2(np.fft.fft2(white) * K).real
    G /= np.sqrt(np.mean(K ** 2))          # Parseval: var(G) = mean(|K|^2) for unit white noise
    G = G[pad:pad + n_out, pad:pad + n_out]
    m = np.exp(sigma * G - 0.5 * sigma ** 2)
    grid = Grid(m, x0, x0, h, {"kind": "rock_density_multiplier", "sigma_ln": sigma, "corr_length_m": L})
    core = lay.zone_rect("core")
    mean_core = _rect_mean(grid, core, 0.5)
    grid.z = grid.z / mean_core
    grid.meta["core_mean_before_normalisation"] = float(mean_core)
    return grid


def _rect_mean(grid: Grid, r: Rect, step: float) -> float:
    xs = np.arange(r.x0 + step / 2, r.x1, step)
    ys = np.arange(r.y0 + step / 2, r.y1, step)
    X, Y = np.meshgrid(xs, ys)
    return float(grid.sample(X.ravel(), Y.ravel()).mean())


def _crater_arrays(craters) -> dict[str, np.ndarray] | None:
    if craters is None:
        return None
    if isinstance(craters, (str, Path)):
        with np.load(craters) as d:
            return {k: np.asarray(d[k], dtype=np.float64) for k in ("x", "y", "D", "rho")}
    out = {}
    for k in ("x", "y", "D", "rho"):
        if isinstance(craters, dict) or (isinstance(craters, np.ndarray) and craters.dtype.names):
            v = craters[k]
        else:
            v = getattr(craters, k)
        out[k] = np.asarray(v, dtype=np.float64).reshape(-1)
    return out


def dtm_fresh_craters(cfg) -> dict[str, np.ndarray]:
    """Fresh craters resolved by the DTM, from the site report (site_selection.yaml).

    Entries of ``<site>.metrics.craters_intersecting`` whose note says "fresh"; rho is taken as
    min(1, (depth / D) / d_over_D_fresh).
    """
    empty = {k: np.zeros(0) for k in ("x", "y", "D", "rho")}
    try:
        import yaml

        path = Path(cfg["paths"]["site_yaml"])
        site = yaml.safe_load(path.read_text())
        entries = site[cfg["site"]["key"]]["metrics"]["craters_intersecting"]
    except Exception:
        return empty
    d_fresh = float(cfg["terrain"]["craters"].get("d_over_D_fresh", 0.13))
    rows = [e for e in entries if "fresh" in str(e.get("note", "")).lower() and "degraded" not in str(e.get("note", "")).lower()]
    if not rows:
        return empty
    return {
        "x": np.array([float(e["e"]) for e in rows]),
        "y": np.array([float(e["n"]) for e in rows]),
        "D": np.array([float(e["D"]) for e in rows]),
        "rho": np.array([min(1.0, float(e.get("depth", d_fresh * e["D"])) / float(e["D"]) / d_fresh) for e in rows]),
    }


def fresh_craters(cfg, craters=None, include_dtm: bool = True) -> dict[str, np.ndarray]:
    """Craters that enhance the rock density: D >= min_crater_D_m and rho >= min_rho."""
    ce = crater_settings(cfg)
    dmin = float(ce["min_crater_D_m"])
    rho_min = float(ce.get("min_rho", FRESH_RHO_MIN))
    parts = []
    c = _crater_arrays(craters)
    if c is not None:
        parts.append(c)
    if include_dtm:
        parts.append(dtm_fresh_craters(cfg))
    if not parts:
        return {k: np.zeros(0) for k in ("x", "y", "D", "rho")}
    allc = {k: np.concatenate([p[k] for p in parts]) for k in ("x", "y", "D", "rho")}
    keep = (allc["D"] >= dmin) & (allc["rho"] >= rho_min)
    return {k: v[keep] for k, v in allc.items()}


def crater_amplitudes(cfg, fc: dict[str, np.ndarray], band: str = "small") -> np.ndarray:
    """Peak excess A_c = (factor - 1) at the rim of every fresh crater for a size band (see module
    docstring); all ``factor - 1`` for the legacy ``annulus`` profile."""
    ce = crater_settings(cfg)
    D = np.asarray(fc["D"], dtype=np.float64)
    rho = np.clip(np.asarray(fc["rho"], dtype=np.float64), 0.0, 1.0)
    if str(ce["profile"]) == "annulus":
        return np.full(len(D), float(ce["factor"]) - 1.0)
    if band == "small":
        return (float(ce["debris_factor"]) - 1.0) * rho
    d0, d1 = (float(v) for v in ce["blocky_D_m"])
    w = np.clip((D - d0) / max(d1 - d0, 1e-9), 0.0, 1.0)
    return (float(ce["block_factor"]) - 1.0) * w * rho


def crater_profile(t: np.ndarray, outer_R: float = 2.5, floor_fraction: float = 0.5) -> np.ndarray:
    """Radial weight p(r / R) of the crater enhancement: ``floor + (1 - floor) * smoothstep(t)`` inside
    (t <= 1), ``(t^-3 - q) / (1 - q)`` with ``q = outer_R^-3`` outside, 0 beyond ``outer_R``.
    Continuous everywhere, peak 1 at the rim."""
    t = np.asarray(t, dtype=np.float64)
    q = outer_R ** -3.0
    tin = np.clip(t, 0.0, 1.0)
    inside = floor_fraction + (1.0 - floor_fraction) * tin * tin * (3.0 - 2.0 * tin)
    outside = (np.maximum(t, 1.0) ** -3.0 - q) / (1.0 - q)
    return np.where(t <= 1.0, inside, np.where(t < outer_R, outside, 0.0))


def crater_factor(cfg, x: np.ndarray, y: np.ndarray, fc: dict[str, np.ndarray], band: str = "small") -> np.ndarray:
    """Enhancement factor e(x, y) of a size band ('small' < 10 cm, 'big' >= 10 cm); 1 far from craters.
    Overlapping craters do not stack (the largest excess wins)."""
    ce = crater_settings(cfg)
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    e = np.ones(x.shape)
    if not len(fc["x"]):
        return e
    if str(ce["profile"]) == "annulus":                       # legacy step function (all sizes)
        factor = float(ce["factor"])
        r0, r1 = (float(v) for v in ce["annulus_R"])
        for cx, cy, D in zip(fc["x"], fc["y"], fc["D"]):
            R = 0.5 * D
            ro = r1 * R
            sel = (np.abs(x - cx) <= ro) & (np.abs(y - cy) <= ro)
            if not sel.any():
                continue
            d = np.hypot(x[sel] - cx, y[sel] - cy)
            inside = (d >= r0 * R) & (d <= ro)
            e[np.nonzero(sel)[0][inside]] = factor
        return e
    amp = crater_amplitudes(cfg, fc, band)
    outer = float(ce["outer_R"])
    floor = float(ce["floor_fraction"])
    excess = np.zeros(x.shape)
    for cx, cy, D, A in zip(fc["x"], fc["y"], fc["D"], amp):
        if A <= 0.0:
            continue
        R = 0.5 * D
        ro = outer * R
        sel = (np.abs(x - cx) <= ro) & (np.abs(y - cy) <= ro)
        if not sel.any():
            continue
        idx = np.nonzero(sel)[0]
        v = A * crater_profile(np.hypot(x[idx] - cx, y[idx] - cy) / R, outer, floor)
        excess[idx] = np.maximum(excess[idx], v)
    return e + excess


def max_crater_factor(cfg, fc: dict[str, np.ndarray], band: str = "small") -> float:
    """Upper bound of crater_factor (thinning)."""
    if not len(fc["x"]):
        return 1.0
    amp = crater_amplitudes(cfg, fc, band)
    return 1.0 + float(max(amp.max(), 0.0))


# ---------------------------------------------------------------------------------------------
# Zone geometry and intensity sampling
# ---------------------------------------------------------------------------------------------
def zone_region(cfg, zone: str, rect: tuple[float, float, float, float] | None = None) -> tuple[Rect, Rect | None]:
    """(bounding rectangle, hole) of the sampling region; ring = ring square minus the core square."""
    lay = Layout(cfg)
    outer = lay.zone_rect(zone)
    hole = lay.zone_rect("core") if zone == "ring" else None
    if rect is not None:
        x0, x1, y0, y1 = rect
        outer = Rect(max(outer.x0, x0), min(outer.x1, x1), max(outer.y0, y0), min(outer.y1, y1))
        if outer.x1 <= outer.x0 or outer.y1 <= outer.y0:
            raise ValueError("rect does not overlap the zone")
    return outer, hole


def edge_band_weight(cfg, x: np.ndarray, y: np.ndarray, width_m: float) -> np.ndarray:
    """Raised cosine of the distance d outside the core square: 1 at the edge, 0 at d >= width_m
    (and 1 inside the core)."""
    core = Layout(cfg).zone_rect("core")
    dx = np.maximum(np.maximum(core.x0 - np.asarray(x, dtype=np.float64), np.asarray(x) - core.x1), 0.0)
    dy = np.maximum(np.maximum(core.y0 - np.asarray(y, dtype=np.float64), np.asarray(y) - core.y1), 0.0)
    t = np.clip(np.hypot(dx, dy) / max(width_m, 1e-9), 0.0, 1.0)
    return 0.5 * (1.0 + np.cos(np.pi * t))


class IntensitySampler:
    """Draws positions from the intensity scale * m * e_band (* weight) over a rectangle minus an
    optional hole (thinning)."""

    def __init__(self, cfg, outer: Rect, hole: Rect | None, field: Grid, fc: dict, rng: np.random.Generator,
                 scale: float = 1.0, band: str = "small", weight=None):
        self.cfg, self.outer, self.hole, self.field, self.fc, self.rng = cfg, outer, hole, field, fc, rng
        self.scale = float(scale)
        self.band = band
        self.weight = weight
        sub = field.crop(outer.x0 - field.spacing, outer.x1 + field.spacing,
                         outer.y0 - field.spacing, outer.y1 + field.spacing)
        self.lmax = self.scale * float(sub.z.max()) * max_crater_factor(cfg, fc, band)
        self._acc = 1.0 / max(self.lmax, 1e-9)          # running estimate of the thinning acceptance rate

    def intensity(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        lam = self.scale * self.field.sample(x, y) * crater_factor(self.cfg, x, y, self.fc, self.band)
        if self.weight is not None:
            lam = lam * self.weight(x, y)
        if self.hole is not None:
            lam = np.where(self.hole.contains(x, y, eps=-1e-12), 0.0, lam)
        return lam

    def draw(self, n: int) -> tuple[np.ndarray, np.ndarray]:
        """Exactly n positions distributed with density proportional to the intensity."""
        if n <= 0:
            return np.zeros(0), np.zeros(0)
        xs, ys = [], []
        got = 0
        while got < n:
            m = min(int(1.2 * (n - got) / max(self._acc, 1e-4)) + 64, 4_000_000)
            x = self.rng.uniform(self.outer.x0, self.outer.x1, m)
            y = self.rng.uniform(self.outer.y0, self.outer.y1, m)
            keep = self.rng.random(m) * self.lmax < self.intensity(x, y)
            k = int(keep.sum())
            self._acc = max(k / m, 1e-4)
            xs.append(x[keep])
            ys.append(y[keep])
            got += k
        return np.concatenate(xs)[:n], np.concatenate(ys)[:n]

    def integral(self, step: float = 0.25) -> float:
        """Integral of the intensity over the region (midpoint rule, chunked)."""
        r = self.outer
        xs = np.arange(r.x0 + step / 2, r.x1, step)
        ys = np.arange(r.y0 + step / 2, r.y1, step)
        total = 0.0
        rows = max(1, 2_000_000 // max(len(xs), 1))
        for j0 in range(0, len(ys), rows):
            X, Y = np.meshgrid(xs, ys[j0:j0 + rows])
            total += float(self.intensity(X.ravel(), Y.ravel()).sum())
        return total * step * step


def core_intensity_scale(cfg, field: Grid, fc: dict, band: str = "small") -> float:
    """Factor that makes the mean of m * e_band over the whole core exactly 1.

    The crater enhancement then redistributes rocks inside the core instead of adding to them, so the
    core keeps the calibration's nominal LM-site totals (which are site averages that already include
    fresh-crater ejecta). The same factor is used for the ring, so the intensity stays continuous.
    """
    core = Layout(cfg).zone_rect("core")
    s = IntensitySampler(cfg, core, None, field, fc, np.random.default_rng(0), band=band).integral()
    return core.width * core.height / s


def _sample_bands(cfg, sfd: BrokenPowerLaw, bands, outer: Rect, hole: Rect | None, field: Grid, fc: dict,
                  stream: str, weight=None) -> dict:
    """Sizes and positions of every size band. Streams: '<stream>.count|sizes|positions' for the first
    band (the pre-FID-02 names), '<stream>.<band>.*' for the second."""
    xs, ys, Ds, bs, samplers, lam, scales, ndraw = [], [], [], [], [], [], [], []
    for b, (lo, hi, name) in enumerate(bands):
        pre = stream if b == 0 else f"{stream}.{name}"
        scale = core_intensity_scale(cfg, field, fc, name)
        smp = IntensitySampler(cfg, outer, hole, field, fc, cfg_rng(cfg, f"{pre}.positions"), scale, name, weight)
        L = smp.integral()
        n = int(cfg_rng(cfg, f"{pre}.count").poisson(sfd.count_density(lo, hi) * L))
        D = sfd.sample(cfg_rng(cfg, f"{pre}.sizes"), n, lo, hi)
        x, y = smp.draw(n)
        xs.append(x), ys.append(y), Ds.append(D), bs.append(np.full(n, b, dtype=np.int64))
        samplers.append(smp), lam.append(L), scales.append(scale), ndraw.append(n)
    return {"x": np.concatenate(xs), "y": np.concatenate(ys), "D": np.concatenate(Ds), "band": np.concatenate(bs),
            "samplers": samplers, "Lambda": lam, "scale": scales, "n_draw": ndraw}


def _expected(sfd: BrokenPowerLaw, bands, Lambda, D_list) -> dict[float, float]:
    out = {}
    for d in D_list:
        out[float(d)] = float(sum(sfd.count_density(max(float(d), lo), hi) * L
                                  for (lo, hi, _), L in zip(bands, Lambda) if hi > d))
    return out


# ---------------------------------------------------------------------------------------------
# Separation (before placement)
# ---------------------------------------------------------------------------------------------
def separate(cfg, x: np.ndarray, y: np.ndarray, D: np.ndarray, sampler: IntensitySampler,
             max_tries: int = 200, band: np.ndarray | None = None,
             samplers: list | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Apply the separation rule; returns (x, y, keep mask, stats). Inputs are not modified.

    Re-drawn rocks come from ``samplers[band[i]]`` when given, else from ``sampler``."""
    x = x.copy()
    y = y.copy()
    keep = np.ones(len(D), dtype=bool)
    if samplers is None or band is None:
        samplers, band = [sampler], np.zeros(len(D), dtype=np.int64)
    fac = float(cfg["rocks"]["min_separation_factor"])
    dsep = float(cfg["rocks"].get("separation_min_D_m", SEPARATION_MIN_D_M))
    big = np.nonzero(D >= dsep)[0]
    big = big[np.argsort(-D[big], kind="stable")]
    stats = {"n_big": int(len(big)), "redraws_big": 0, "dropped_big": 0, "redraws_small": 0, "dropped_small": 0}
    if len(big):
        cell = fac * float(D[big].max())
        grid: dict[tuple[int, int], list[int]] = {}
        pools: dict[int, list] = {}
        r_max = 0.5 * float(D[big].max())
        for i in big:
            ri = 0.5 * D[i]
            ok = False
            for t in range(max_tries):
                if t > 0:
                    b = int(band[i])
                    pool = pools.get(b)
                    if pool is None or pool[2] >= len(pool[0]):
                        px, py = samplers[b].draw(max(64, len(big)))
                        pool = [px, py, 0]
                        pools[b] = pool
                    x[i], y[i] = pool[0][pool[2]], pool[1][pool[2]]
                    pool[2] += 1
                    stats["redraws_big"] += 1
                cx, cy = int(np.floor(x[i] / cell)), int(np.floor(y[i] / cell))
                reach = int(np.ceil(fac * (ri + r_max) / cell))
                clash = False
                for gx in range(cx - reach, cx + reach + 1):
                    for gy in range(cy - reach, cy + reach + 1):
                        for j in grid.get((gx, gy), ()):
                            if (x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2 < (fac * (ri + 0.5 * D[j])) ** 2:
                                clash = True
                                break
                        if clash:
                            break
                    if clash:
                        break
                if not clash:
                    ok = True
                    break
            if ok:
                grid.setdefault((int(np.floor(x[i] / cell)), int(np.floor(y[i] / cell))), []).append(int(i))
            else:
                keep[i] = False
                stats["dropped_big"] += 1
        # small rocks whose centre falls inside a big rock's exclusion disc are re-drawn
        small = np.nonzero(D < dsep)[0]
        placed = big[keep[big]]
        if len(small) and len(placed):
            from scipy.spatial import cKDTree

            bx, by, br = x[placed], y[placed], fac * 0.5 * D[placed]
            btree = cKDTree(np.column_stack([bx, by]))
            todo = small
            for _round in range(10):
                kq = min(8, len(placed))
                d, j = btree.query(np.column_stack([x[todo], y[todo]]), k=kq, distance_upper_bound=float(br.max()))
                d, j = d.reshape(len(todo), kq), j.reshape(len(todo), kq)
                j = np.minimum(j, len(placed) - 1)          # missing neighbours come back as n
                inside = np.zeros(len(todo), dtype=bool)
                for col in range(d.shape[1]):
                    valid = np.isfinite(d[:, col])
                    jj = np.where(valid, j[:, col], 0)
                    inside |= valid & (d[:, col] < br[jj])
                bad = todo[inside]
                if not len(bad):
                    break
                for b in np.unique(band[bad]):
                    sel = bad[band[bad] == b]
                    nx_, ny_ = samplers[int(b)].draw(len(sel))
                    x[sel], y[sel] = nx_, ny_
                stats["redraws_small"] += int(len(bad))
                todo = bad
            else:
                keep[todo] = False
                stats["dropped_small"] += int(len(todo))
    return x, y, keep, stats


# ---------------------------------------------------------------------------------------------
# Interpenetration (after placement, FID-09)
# ---------------------------------------------------------------------------------------------
class _Geom:
    """Packed per-prototype hull planes / hull vertices and per-(prototype, LOD) LOD vertices."""

    def __init__(self, library: RockLibrary):
        npr = len(library)
        PN, PO, HV, poff, hoff = [], [], [], [0], [0]
        rb = np.zeros(npr)
        for p in library.prototypes:
            n, o = library.hull_planes(p.index)
            V = library.hull(p.index)
            PN.append(n), PO.append(o), HV.append(V)
            poff.append(poff[-1] + len(n))
            hoff.append(hoff[-1] + len(V))
            r = float(np.linalg.norm(V, axis=1).max())
            for lod in range(p.n_lods):
                r = max(r, float(np.linalg.norm(library.contact_points(p.index, lod), axis=1).max()))
            rb[p.index] = r
        self.PN, self.PO, self.HV = np.concatenate(PN), np.concatenate(PO), np.concatenate(HV)
        self.poff, self.hoff = np.array(poff, np.int64), np.array(hoff, np.int64)
        self.rb = rb                                      # bounding radius of every LOD and the hull (rock frame)
        self.library = library
        self._cp: dict[int, int] = {}
        self._CP: list[np.ndarray] = []
        self._coff = [0]

    def cp_index(self, proto: np.ndarray, lod: np.ndarray) -> np.ndarray:
        key = proto.astype(np.int64) * 16 + lod.astype(np.int64)
        out = np.empty(len(key), np.int64)
        for k in np.unique(key):
            if int(k) not in self._cp:
                pts = self.library.contact_points(int(k // 16), int(k % 16))
                self._cp[int(k)] = len(self._CP)
                self._CP.append(pts)
                self._coff.append(self._coff[-1] + len(pts))
            out[key == k] = self._cp[int(k)]
        return out

    def cp_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        return (np.concatenate(self._CP) if self._CP else np.zeros((0, 3))), np.array(self._coff, np.int64)


@njit(cache=False)
def _pair_conflicts(I, J, R, T, S, P, K, PN, PO, POFF, HV, HOFF, CP, COFF, pierce, tol, out):
    """For pairs (I[k] larger, J[k] smaller): out[k, 0] = centre of J inside I's footprint; out[k, 1] =
    J's LOD vertices inside I's hull or I's hull vertices inside J's hull (only where pierce[k])."""
    for k in range(I.shape[0]):
        i = I[k]
        j = J[k]
        pi = P[i]
        pj = P[j]
        # -- centre line of j (vertical) against i's hull, in i's frame
        dx = T[j, 0] - T[i, 0]
        dy = T[j, 1] - T[i, 1]
        c0 = np.empty(3)
        d = np.empty(3)
        for a in range(3):
            c0[a] = (R[i, 0, a] * dx + R[i, 1, a] * dy) / S[i]
            d[a] = R[i, 2, a]
        lo = -1e300
        hi = 1e300
        ok = True
        for f in range(POFF[pi], POFF[pi + 1]):
            nd = PN[f, 0] * d[0] + PN[f, 1] * d[1] + PN[f, 2] * d[2]
            rhs = PO[f] - (PN[f, 0] * c0[0] + PN[f, 1] * c0[1] + PN[f, 2] * c0[2])
            if abs(nd) < 1e-12:
                if rhs < 0.0:
                    ok = False
                    break
            elif nd > 0.0:
                hi = min(hi, rhs / nd)
            else:
                lo = max(lo, rhs / nd)
        out[k, 0] = 1 if (ok and lo <= hi) else 0
        out[k, 1] = 0
        if not pierce[k]:
            continue
        # -- j's LOD vertices inside i's hull
        ti = tol / S[i]
        hit = False
        kj = K[j]
        for v in range(COFF[kj], COFF[kj + 1]):
            w = np.empty(3)
            for a in range(3):
                w[a] = T[j, a] + S[j] * (R[j, a, 0] * CP[v, 0] + R[j, a, 1] * CP[v, 1] + R[j, a, 2] * CP[v, 2]) - T[i, a]
            pl = np.empty(3)
            for a in range(3):
                pl[a] = (R[i, 0, a] * w[0] + R[i, 1, a] * w[1] + R[i, 2, a] * w[2]) / S[i]
            inside = True
            for f in range(POFF[pi], POFF[pi + 1]):
                if PN[f, 0] * pl[0] + PN[f, 1] * pl[1] + PN[f, 2] * pl[2] > PO[f] - ti:
                    inside = False
                    break
            if inside:
                hit = True
                break
        if not hit:
            tj = tol / S[j]
            for v in range(HOFF[pi], HOFF[pi + 1]):
                w = np.empty(3)
                for a in range(3):
                    w[a] = T[i, a] + S[i] * (R[i, a, 0] * HV[v, 0] + R[i, a, 1] * HV[v, 1] + R[i, a, 2] * HV[v, 2]) - T[j, a]
                pl = np.empty(3)
                for a in range(3):
                    pl[a] = (R[j, 0, a] * w[0] + R[j, 1, a] * w[1] + R[j, 2, a] * w[2]) / S[j]
                inside = True
                for f in range(POFF[pj], POFF[pj + 1]):
                    if PN[f, 0] * pl[0] + PN[f, 1] * pl[1] + PN[f, 2] * pl[2] > PO[f] - tj:
                        inside = False
                        break
                if inside:
                    hit = True
                    break
        out[k, 1] = 1 if hit else 0


def find_conflicts(cfg, rocks: dict, library: RockLibrary, fixed: np.ndarray | None = None,
                   geom: _Geom | None = None) -> tuple[np.ndarray, dict]:
    """Rocks that must move (bool mask over ``rocks``) and conflict counts.

    ``rocks``: dict of arrays x, y, z, D, scale, proto, lod, qw, qx, qy, qz (movable rocks);
    ``fixed``: optional catalog rows that never move (another zone). In a conflicting pair the smaller
    rock moves; if it is fixed, the larger one moves (if movable)."""
    from scipy.spatial import cKDTree

    ov = _section(cfg, "overlap", OVERLAP)
    geom = geom or _Geom(library)
    n = len(rocks["D"])
    cols = ("x", "y", "z", "D", "scale", "proto", "lod", "qw", "qx", "qy", "qz")
    if fixed is not None and len(fixed):
        A = {c: np.concatenate([np.asarray(rocks[c], dtype=np.float64), np.asarray(fixed[c], dtype=np.float64)])
             for c in cols}
    else:
        A = {c: np.asarray(rocks[c], dtype=np.float64) for c in cols}
    m = len(A["D"])
    movable = np.zeros(m, dtype=bool)
    movable[:n] = True
    stats = {"pairs_tested": 0, "centre_inside": 0, "pierce": 0}
    if m < 2:
        return np.zeros(n, dtype=bool), stats
    P = A["proto"].astype(np.int64)
    L = A["lod"].astype(np.int64)
    S = A["scale"]
    D = A["D"]
    rb = geom.rb[P] * S
    pts = np.column_stack([A["x"], A["y"]])
    tree = cKDTree(pts)
    small = D < SEPARATION_MIN_D_M
    pairs = []
    if small.any():
        rmax_s = float(rb[small].max())
        pr = tree.query_pairs(r=2.0 * rmax_s, output_type="ndarray")
        if len(pr):
            pr = pr[small[pr[:, 0]] & small[pr[:, 1]]]
            pairs.append(pr)
    bigi = np.nonzero(~small)[0]
    if len(bigi):
        rmax = float(rb.max())
        lists = tree.query_ball_point(pts[bigi], rb[bigi] + rmax)
        for i, js in zip(bigi, lists):
            js = np.asarray(js, dtype=np.int64)
            js = js[(js != i) & (small[js] | (js > i))]
            if len(js):
                pairs.append(np.column_stack([np.full(len(js), i, np.int64), js]))
    if not pairs:
        return np.zeros(n, dtype=bool), stats
    pr = np.concatenate(pairs)
    pr = pr[movable[pr[:, 0]] | movable[pr[:, 1]]]
    dd = np.hypot(pts[pr[:, 0], 0] - pts[pr[:, 1], 0], pts[pr[:, 0], 1] - pts[pr[:, 1], 1])
    pr = pr[dd < rb[pr[:, 0]] + rb[pr[:, 1]]]
    if not len(pr):
        return np.zeros(n, dtype=bool), stats
    # order: I = larger (ties by index), J = smaller
    swap = (D[pr[:, 1]] > D[pr[:, 0]]) | ((D[pr[:, 1]] == D[pr[:, 0]]) & (pr[:, 1] < pr[:, 0]))
    I = np.where(swap, pr[:, 1], pr[:, 0]).astype(np.int64)
    J = np.where(swap, pr[:, 0], pr[:, 1]).astype(np.int64)
    R = quat_to_matrix(np.column_stack([A["qw"], A["qx"], A["qy"], A["qz"]]))
    T = np.column_stack([A["x"], A["y"], A["z"]])
    K = geom.cp_index(P, L)
    CP, COFF = geom.cp_arrays()
    pierce = D[I] >= float(ov["pierce_min_D_m"])
    out = np.zeros((len(I), 2), np.int64)
    _pair_conflicts(I, J, np.ascontiguousarray(R), np.ascontiguousarray(T), np.ascontiguousarray(S), P, K,
                    geom.PN, geom.PO, geom.poff, geom.HV, geom.hoff, CP, COFF, pierce, float(ov["tol_m"]), out)
    bad = (out[:, 0] > 0) | (out[:, 1] > 0)
    stats.update({"pairs_tested": int(len(I)), "centre_inside": int((out[:, 0] > 0).sum()),
                  "pierce": int((out[:, 1] > 0).sum()), "conflicts": int(bad.sum())})
    mover = np.where(movable[J], J, np.where(movable[I], I, -1))[bad]
    move = np.zeros(m, dtype=bool)
    move[mover[mover >= 0]] = True
    return move[:n], stats


def resolve_overlaps(cfg, zone: str, x, y, D, fields: dict, library: RockLibrary, ground, band_idx, samplers,
                     stream: str, fixed: np.ndarray | None = None, lod=None) -> tuple[np.ndarray, np.ndarray, dict, np.ndarray, dict]:
    """Re-draw and re-place interpenetrating rocks until none is left (see the module docstring).
    Returns (x, y, fields, keep mask, stats)."""
    ov = _section(cfg, "overlap", OVERLAP)
    x = np.array(x, dtype=np.float64, copy=True)
    y = np.array(y, dtype=np.float64, copy=True)
    fields = {k: np.array(v, copy=True) for k, v in fields.items()}
    keep = np.ones(len(D), dtype=bool)
    stats: dict = {"enabled": bool(ov["enabled"]), "rounds": 0, "moved": 0, "dropped": 0}
    if not bool(ov["enabled"]) or not len(D):
        return x, y, fields, keep, stats
    geom = _Geom(library)
    for rnd in range(int(ov["max_rounds"]) + 1):
        rocks = {"x": x, "y": y, "D": D, **fields}
        move, st = find_conflicts(cfg, rocks, library, fixed, geom)
        if rnd == 0:
            stats["initial"] = st
        stats["final"] = st
        idx = np.nonzero(move & keep)[0]
        if not len(idx):
            break
        if rnd == int(ov["max_rounds"]):
            keep[idx] = False
            stats["dropped"] = int(len(idx))
            break
        stats["rounds"] = rnd + 1
        stats["moved"] += int(len(idx))
        for b in np.unique(band_idx[idx]):
            sel = idx[band_idx[idx] == b]
            nx_, ny_ = samplers[int(b)].draw(len(sel))
            x[sel], y[sel] = nx_, ny_
        sub = place_rocks(cfg, cfg_rng(cfg, f"{stream}.{rnd}"), x[idx], y[idx], D[idx], zone, ground, library,
                          lod=None if lod is None else np.asarray(lod)[idx])
        for k, v in sub.items():
            fields[k][idx] = v
    return x, y, fields, keep, stats


# ---------------------------------------------------------------------------------------------
# Core-edge band (FID-04)
# ---------------------------------------------------------------------------------------------
def band_lods(cfg, D: np.ndarray) -> np.ndarray:
    """LOD of core-edge band rocks: the core class LOD, overridden per class name by edge_band.lod."""
    eb = _section(cfg, "edge_band", EDGE_BAND)
    tab = class_table(cfg)
    over = dict(eb.get("lod") or {})
    lods = np.array([int(over.get(c["name"], c["lod"])) for c in tab])
    return lods[size_class(cfg, D)]


def sample_edge_band(cfg, ground, library: RockLibrary, field: Grid, fc: dict, sfd: BrokenPowerLaw,
                     rect: tuple[float, float, float, float] | None = None, fixed: np.ndarray | None = None):
    """Rocks smaller than the ring's D_min continued outside the core (see the module docstring).
    Returns (x, y, D, fields, meta) or None when disabled."""
    eb = _section(cfg, "edge_band", EDGE_BAND)
    rk = cfg["rocks"]
    W = float(eb["width_m"])
    lo, hi = float(rk["D_min_m"]["core"]), float(rk["D_min_m"]["ring"])
    if not bool(eb["enabled"]) or W <= 0.0 or hi <= lo:
        return None
    lay = Layout(cfg)
    core, ring = lay.zone_rect("core"), lay.zone_rect("ring")
    outer = Rect(max(core.x0 - W, ring.x0), min(core.x1 + W, ring.x1), max(core.y0 - W, ring.y0), min(core.y1 + W, ring.y1))
    if rect is not None:
        x0, x1, y0, y1 = rect
        outer = Rect(max(outer.x0, x0), min(outer.x1, x1), max(outer.y0, y0), min(outer.y1, y1))
        if outer.x1 <= outer.x0 or outer.y1 <= outer.y0:
            return None
    bands = size_bands(cfg, lo, hi)

    def weight(x, y):
        return edge_band_weight(cfg, x, y, W)

    t0 = time.perf_counter()
    pop = _sample_bands(cfg, sfd, bands, outer, core, field, fc, "rocks.band", weight)
    x, y, D, bidx = pop["x"], pop["y"], pop["D"], pop["band"]
    x, y, keep, sep = separate(cfg, x, y, D, pop["samplers"][0], band=bidx, samplers=pop["samplers"])
    x, y, D, bidx = x[keep], y[keep], D[keep], bidx[keep]
    lod = band_lods(cfg, D)
    pst: dict = {}
    fields = place_rocks(cfg, cfg_rng(cfg, "rocks.band.placement"), x, y, D, "ring", ground, library, lod=lod, stats=pst)
    x, y, fields, keep, ovs = resolve_overlaps(cfg, "ring", x, y, D, fields, library, ground, bidx, pop["samplers"],
                                               "rocks.band.repair", fixed=fixed, lod=lod)
    x, y, D = x[keep], y[keep], D[keep]
    fields = {k: v[keep] for k, v in fields.items()}
    tri = np.zeros((len(library), 3), np.int64)
    for p in library.prototypes:
        tri[p.index, :p.n_lods] = p.lod_tris
    meta = {"width_m": W, "D_range_m": [lo, hi], "region": [outer.x0, outer.x1, outer.y0, outer.y1],
            "n": int(len(D)), "n_poisson_draw": pop["n_draw"], "intensity_integral_m2_by_band": pop["Lambda"],
            "expected_total": float(sum(sfd.count_density(b0, b1) * L for (b0, b1, _), L in zip(bands, pop["Lambda"]))),
            "expected_counts": _expected(sfd, bands, pop["Lambda"], SFD_CHECK_D_M),
            "separation": sep, "overlap": ovs, "settle": pst.get("settle"),
            "lod_by_class": {c["name"]: int(dict(eb.get("lod") or {}).get(c["name"], c["lod"]))
                             for c in class_table(cfg)},
            "triangles": int(tri[fields["proto"], fields["lod"]].sum()),
            "timing_s": time.perf_counter() - t0}
    return x, y, D, fields, meta


# ---------------------------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------------------------
def sample_rocks(cfg, zone: str, ground, craters=None, library: RockLibrary | None = None,
                 rect: tuple[float, float, float, float] | None = None, include_dtm_craters: bool = True,
                 field: Grid | None = None, return_meta: bool = False, avoid: np.ndarray | None = None):
    """Sample and place the rocks of a zone ('core' or 'ring').

    ground   object with .height(x, y) / .normal(x, y), a Grid or a callable (H0, without fillets)
    craters  crater set for the enhancement (object/dict with x, y, D, rho), optional
    library  RockLibrary (default: cfg.paths.rock_library)
    rect     optional (x0, x1, y0, y1) restricting the sampling region (tests, trials)
    avoid    optional catalog of already placed rocks of another zone (e.g. the core when sampling the
             ring): the interpenetration test keeps clear of them (they never move)
    Returns the catalog (structured array), or (catalog, meta) with return_meta=True.
    """
    if zone not in ("core", "ring"):
        raise ValueError("zone must be 'core' or 'ring'")
    t0 = time.perf_counter()
    library = library if library is not None else RockLibrary(cfg["paths"]["rock_library"])
    ground = as_ground(ground)
    rk = cfg["rocks"]
    sfd = BrokenPowerLaw.from_config(cfg)
    D_min = float(rk["D_min_m"][zone])
    D_max = float(rk["D_max_m"])
    outer, hole = zone_region(cfg, zone, rect)
    field = field if field is not None else density_field(cfg)
    fc = fresh_craters(cfg, craters, include_dtm=include_dtm_craters)
    bands = size_bands(cfg, D_min, D_max)
    pop = _sample_bands(cfg, sfd, bands, outer, hole, field, fc, f"rocks.{zone}")
    x, y, D, bidx = pop["x"], pop["y"], pop["D"], pop["band"]
    t1 = time.perf_counter()
    x, y, keep, sep_stats = separate(cfg, x, y, D, pop["samplers"][0], band=bidx, samplers=pop["samplers"])
    x, y, D, bidx = x[keep], y[keep], D[keep], bidx[keep]
    t2 = time.perf_counter()
    pst: dict = {}
    fields = place_rocks(cfg, cfg_rng(cfg, f"rocks.{zone}.placement"), x, y, D, zone, ground, library, stats=pst)
    t3 = time.perf_counter()
    fixed = np.asarray(avoid) if avoid is not None and len(avoid) else None
    x, y, fields, keep, ov_stats = resolve_overlaps(cfg, zone, x, y, D, fields, library, ground, bidx,
                                                    pop["samplers"], f"rocks.{zone}.repair", fixed=fixed)
    x, y, D = x[keep], y[keep], D[keep]
    fields = {k: v[keep] for k, v in fields.items()}
    t4 = time.perf_counter()
    band = None
    if zone == "ring":
        main = empty_catalog(len(D))
        main["x"], main["y"], main["D"] = x, y, D
        for k, v in fields.items():
            main[k] = v
        fx = main if fixed is None else np.concatenate([main, np.asarray(fixed, dtype=main.dtype)])
        band = sample_edge_band(cfg, ground, library, field, fc, sfd, rect=rect, fixed=fx)
    t5 = time.perf_counter()
    n_main = len(D)
    n_band = 0 if band is None else len(band[2])
    cat = empty_catalog(n_main + n_band)
    cat["id"][:n_main] = np.arange(n_main) + (ID_ZONE_OFFSET if zone == "ring" else 0)
    cat["x"][:n_main], cat["y"][:n_main], cat["D"][:n_main] = x, y, D
    for k, v in fields.items():
        cat[k][:n_main] = v
    if band is not None:
        bx, by, bD, bf, _ = band
        cat["id"][n_main:] = BAND_ID_OFFSET + np.arange(n_band)
        cat["x"][n_main:], cat["y"][n_main:], cat["D"][n_main:] = bx, by, bD
        for k, v in bf.items():
            cat[k][n_main:] = v
    cat["zone"] = ZONE_CODES[zone]
    ti, tj = assign_tiles(cfg, zone, cat["x"], cat["y"])
    cat["tile_i"], cat["tile_j"] = ti, tj
    checks = SFD_CHECK_D_M if zone == "core" else RING_CHECK_D_M
    expected = _expected(sfd, bands, pop["Lambda"], checks)
    meta = {
        "zone": zone, "n": int(len(cat)), "n_main": int(n_main), "n_poisson_draw": int(sum(pop["n_draw"])),
        "region": [outer.x0, outer.x1, outer.y0, outer.y1],
        "hole": [hole.x0, hole.x1, hole.y0, hole.y1] if hole is not None else None,
        "area_m2": float(outer.width * outer.height - (hole.width * hole.height if hole is not None else 0.0)),
        "D_range_m": [D_min, D_max], "size_bands": [[lo, hi, nm] for lo, hi, nm in bands],
        "intensity_integral_m2": float(pop["Lambda"][0]), "intensity_integral_m2_by_band": pop["Lambda"],
        "expected_total": float(sum(sfd.count_density(lo, hi) * L for (lo, hi, _), L in zip(bands, pop["Lambda"]))),
        "expected_counts": expected, "separation": sep_stats, "overlap": ov_stats, "settle": pst.get("settle"),
        "n_perched": pst.get("n_perched"), "n_bedded": pst.get("n_bedded"),
        "fresh_craters": {k: v.tolist() for k, v in fc.items()},
        "crater_enhancement": {k: (list(v) if isinstance(v, (list, tuple)) else v)
                               for k, v in crater_settings(cfg).items()},
        "crater_amplitudes": {b: crater_amplitudes(cfg, fc, b).tolist() for b in ("small", "big")},
        "density_field_core_mean_before_norm": field.meta.get("core_mean_before_normalisation"),
        "intensity_scale": float(pop["scale"][0]), "intensity_scale_by_band": pop["scale"],
        "library": {"path": str(library.path), "version": library.version, "digest": library.digest},
        "config_hash_rocks": config_hash(cfg, ["rocks", "world"]),
        "timing_s": {"sizes_positions": t1 - t0, "separation": t2 - t1, "placement": t3 - t2, "overlap": t4 - t3,
                     "edge_band": t5 - t4, "total": t5 - t0},
    }
    if band is not None:
        meta["edge_band"] = band[4]
    return (cat, meta) if return_meta else cat


def assign_tiles(cfg, zone: str, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Core: tile (i, j) of the centre. Ring: (strip index in RING_SIDES, -1)."""
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    if zone == "core":
        T = lay.tile_size
        ti = np.clip(np.floor((x - core.x0) / T), 0, lay.n_tiles - 1).astype(np.int16)
        tj = np.clip(np.floor((y - core.y0) / T), 0, lay.n_tiles - 1).astype(np.int16)
        return ti, tj
    side = np.where(y < core.y0, RING_SIDES.index("south"),
                    np.where(y > core.y1, RING_SIDES.index("north"),
                             np.where(x < core.x0, RING_SIDES.index("west"), RING_SIDES.index("east"))))
    return side.astype(np.int16), np.full(len(x), -1, dtype=np.int16)


def expected_counts(cfg, zone: str, D_list, intensity_integral_m2: float) -> dict[float, float]:
    """Expected N(>=D) for a zone whose intensity integral is known (see sample_rocks meta; with two
    size bands pass ``meta['intensity_integral_m2_by_band']`` as a list for the exact value)."""
    sfd = BrokenPowerLaw.from_config(cfg)
    D_min = float(cfg["rocks"]["D_min_m"][zone])
    D_max = float(cfg["rocks"]["D_max_m"])
    bands = size_bands(cfg, D_min, D_max)
    if isinstance(intensity_integral_m2, (list, tuple, np.ndarray)) and len(intensity_integral_m2) == len(bands):
        return _expected(sfd, bands, list(intensity_integral_m2), D_list)
    L = float(np.atleast_1d(intensity_integral_m2)[0])
    return {float(d): float(sfd.count_density(max(d, D_min), D_max) * L) for d in D_list}
