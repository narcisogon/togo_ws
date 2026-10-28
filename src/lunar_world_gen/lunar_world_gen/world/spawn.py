"""Spawn height and test sites (docs/DESIGN.md section 11).

Inputs: the core collision height field ``C`` (a :class:`~lunar_world_gen.grid.Grid`, 2 cm),
the rock catalog (``rocks_catalog.npz``; only x, y, D and optionally zone/collide/exposed_h/id
are used) and optionally the visual height function ``H(x, y)`` (reported per site).

Spawn height
    ``z = max(C over a disc of radius r_fp) + base_link_height_m + clearance_m`` with
    ``r_fp = hypot(*spawn.footprint_half_m)`` (0.68 m: the +-0.55 x +-0.40 m footprint at any
    yaw). Wheels rest ~10 mm above a Bullet mesh (contact margin), so the 0.04 m clearance
    leaves the robot ~3 cm above the margin band, as required (>= 15 mm).

Footprint statistics (:class:`DiscSampler`)
    For a disc of radius ``r_fp`` around a grid node: the maximum of C, the least-squares
    plane's slope and downhill azimuth, and the RMS of the residual about that plane
    ("roughness"). The plane fit is one fixed linear map of the disc samples, so thousands
    of candidates are evaluated with a single matrix product per chunk.

Sites (all inside ``region``, default the core minus ``EDGE_MARGIN_M``)
    flat   nearest suitable site to the core centre: footprint slope <= ``spawn.max_slope_deg``,
           no rock with D >= ``spawn.rock_free_D_m`` whose edge is within r_fp +
           ``spawn.rock_free_margin_m`` of the centre, roughness <= the roughness limit.
           The yaw is the heading whose ``flat_path_m`` (4 m) corridor is the most benign
           (fewest rocks >= 5 cm, then lowest slope), so the standard drive test (3 m forward,
           then an in-place turn) runs on that ground.
    turn   like flat, but no rock with D >= ``spawn.turn_rock_free_D_m`` (default 0.10 m) within
           ``TURN_CLEAR_RADIUS_M`` (1.5 m) of the centre. (DESIGN 13 decision after phase 2: at the
           Apollo 16 density, 1.66 rocks/m^2 >= 5 cm, no 1.5 m disc is free of 5 cm rocks; the
           threshold falls back to ``rock_free_D_m`` if the key is absent.)
    slope  start ("bottom") of a straight run of ``SLOPE_LENGTH_M`` (5 m) whose line-fit slope
           is within ``SLOPE_RANGE_DEG`` (10-15 deg), facing uphill; the 1 m sub-slopes stay
           within ``SLOPE_LOCAL_RANGE_DEG`` and the cross slope <= ``SLOPE_MAX_CROSS_DEG``.
           Among valid runs: fewest rocks >= 10 cm in the corridor, then slope closest to
           12.5 deg, then nearest to the centre.
    rock   2 m before a lone 6-10 cm rock (catalog ``collide`` rocks) on a clear straight path:
           the robot centre line is offset so the LEFT wheels (body y = +``ROCK_WHEEL_OFFSET_M``)
           roll over the rock; no other rock >= rock_free_D_m in the corridor
           [-0.6 m, rock + 1.0 m] x (half-width ``footprint_half_m[1]`` + 0.1 m), none within
           ``ROCK_LONE_RADIUS_M`` of the target, and the path slope <= max_slope_deg.

When no candidate meets a criterion the search relaxes it (documented in the site's
``relaxed`` field) rather than failing; a site is only omitted (``missing`` with a reason) if
no candidate exists at all (e.g. no 6-10 cm rock, or no 10-15 deg slope anywhere).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np
from scipy.spatial import cKDTree

from ..config import Config
from ..grid import Grid
from ..layout import Layout, Rect

HeightFn = Callable[[np.ndarray, np.ndarray], np.ndarray]

EDGE_MARGIN_M = 3.0             # keep sites this far inside the search region's edge
CANDIDATE_STEP_M = 0.5          # candidate lattice spacing (snapped to grid nodes)
FLAT_PATH_M = 4.0               # forward corridor scored for the flat site's yaw
N_HEADINGS = 16                 # headings tried for yaw / slope / rock searches
TURN_CLEAR_RADIUS_M = 1.5
SLOPE_LENGTH_M = 5.0
SLOPE_RANGE_DEG = (10.0, 15.0)
SLOPE_LOCAL_RANGE_DEG = (5.0, 20.0)
SLOPE_MAX_CROSS_DEG = 6.0
SLOPE_MAX_FIT_RMS_M = 0.10
SLOPE_ROCK_D_M = 0.10
ROCK_D_RANGE_M = (0.06, 0.10)
ROCK_DISTANCE_M = 2.0
ROCK_WHEEL_OFFSET_M = 0.2829    # left wheel line in base_link (rover_baseline.md section 1.1)
ROCK_LONE_RADIUS_M = 0.6
ROCK_PATH_BEHIND_M = 0.6
ROCK_PATH_PAST_M = 1.0


# ---------------------------------------------------------------------------------- rocks
@dataclass
class RockTable:
    """The catalog columns the site search needs (core rocks only)."""

    x: np.ndarray
    y: np.ndarray
    D: np.ndarray
    id: np.ndarray
    collide: np.ndarray
    exposed_h: np.ndarray

    def __len__(self) -> int:
        return int(self.x.shape[0])

    @classmethod
    def empty(cls) -> "RockTable":
        z = np.zeros(0)
        return cls(z, z, z, np.zeros(0, dtype=np.int64), np.zeros(0, dtype=bool), z)

    @classmethod
    def from_any(cls, catalog: Any, core_only: bool = True) -> "RockTable":
        """Build from a mapping of arrays (dict / ``np.load`` NpzFile), a structured array,
        a path to ``.npz``/``.csv``, an existing RockTable, or None (no rocks)."""
        if catalog is None:
            return cls.empty()
        if isinstance(catalog, RockTable):
            return catalog
        if isinstance(catalog, (str, Path)):
            p = Path(catalog)
            if p.suffix == ".csv":
                arr = np.genfromtxt(p, delimiter=",", names=True)
                return cls.from_any(arr, core_only)
            with np.load(p, allow_pickle=False) as npz:
                return cls.from_any({k: npz[k] for k in npz.files}, core_only)
        if isinstance(catalog, np.ndarray) and catalog.dtype.names:
            catalog = {k: catalog[k] for k in catalog.dtype.names}
        if not isinstance(catalog, Mapping):
            raise TypeError(f"unsupported rock catalog type {type(catalog)}")
        for key in ("x", "y", "D"):
            if key not in catalog:
                raise KeyError(f"rock catalog lacks column '{key}'")
        x = np.asarray(catalog["x"], dtype=np.float64).ravel()
        n = x.shape[0]
        y = np.asarray(catalog["y"], dtype=np.float64).ravel()
        D = np.asarray(catalog["D"], dtype=np.float64).ravel()
        ids = np.asarray(catalog["id"], dtype=np.int64).ravel() if "id" in catalog else np.arange(n)
        collide = (np.asarray(catalog["collide"]).astype(bool).ravel() if "collide" in catalog
                   else np.ones(n, dtype=bool))
        exposed = (np.asarray(catalog["exposed_h"], dtype=np.float64).ravel() if "exposed_h" in catalog
                   else np.full(n, np.nan))
        keep = np.ones(n, dtype=bool)
        if core_only and "zone" in catalog:
            keep &= np.asarray(catalog["zone"]).ravel() == 0
        return cls(x[keep], y[keep], D[keep], ids[keep], collide[keep], exposed[keep])

    def subset(self, mask: np.ndarray) -> "RockTable":
        return RockTable(self.x[mask], self.y[mask], self.D[mask], self.id[mask], self.collide[mask],
                         self.exposed_h[mask])


class RockIndex:
    """KD-tree over rocks with D >= ``min_D``; queries account for the rock radius."""

    def __init__(self, rocks: RockTable, min_D: float):
        m = rocks.D >= min_D
        self.rocks = rocks.subset(m)
        self.min_D = float(min_D)
        self.r_max = float(self.rocks.D.max() / 2) if len(self.rocks) else 0.0
        self.tree = cKDTree(np.column_stack([self.rocks.x, self.rocks.y])) if len(self.rocks) else None

    def within(self, x: float, y: float, radius: float) -> np.ndarray:
        """Indices of rocks whose edge is within ``radius`` of (x, y)."""
        if self.tree is None:
            return np.zeros(0, dtype=np.int64)
        idx = np.asarray(self.tree.query_ball_point([x, y], radius + self.r_max), dtype=np.int64)
        if idx.size == 0:
            return idx
        d = np.hypot(self.rocks.x[idx] - x, self.rocks.y[idx] - y) - self.rocks.D[idx] / 2
        return idx[d < radius]

    def count_within(self, xs: np.ndarray, ys: np.ndarray, radius: float) -> np.ndarray:
        if self.tree is None:
            return np.zeros(len(xs), dtype=np.int64)
        lists = self.tree.query_ball_point(np.column_stack([xs, ys]), radius + self.r_max)
        out = np.zeros(len(xs), dtype=np.int64)
        for k, idx in enumerate(lists):
            if idx:
                idx = np.asarray(idx)
                d = np.hypot(self.rocks.x[idx] - xs[k], self.rocks.y[idx] - ys[k]) - self.rocks.D[idx] / 2
                out[k] = int(np.count_nonzero(d < radius))
        return out

    def in_corridor(self, x0: float, y0: float, yaw: float, s0: float, s1: float,
                    half_width: float) -> np.ndarray:
        """Indices of rocks overlapping the rectangle s in [s0, s1], |l| <= half_width
        (s along the heading ``yaw`` from (x0, y0), l to the left)."""
        if self.tree is None:
            return np.zeros(0, dtype=np.int64)
        c, s = math.cos(yaw), math.sin(yaw)
        sm = 0.5 * (s0 + s1)
        cx, cy = x0 + c * sm, y0 + s * sm
        r = math.hypot(0.5 * (s1 - s0), half_width) + self.r_max
        idx = np.asarray(self.tree.query_ball_point([cx, cy], r), dtype=np.int64)
        if idx.size == 0:
            return idx
        dx = self.rocks.x[idx] - x0
        dy = self.rocks.y[idx] - y0
        along = dx * c + dy * s
        left = -dx * s + dy * c
        rad = self.rocks.D[idx] / 2
        m = (along >= s0 - rad) & (along <= s1 + rad) & (np.abs(left) <= half_width + rad)
        return idx[m]


# ---------------------------------------------------------------------------------- footprint
def footprint_radius(cfg: Config) -> float:
    hx, hy = (float(v) for v in cfg.spawn.footprint_half_m)
    return math.hypot(hx, hy)


def spawn_height_offset(cfg: Config) -> float:
    return float(cfg.spawn.base_link_height_m) + float(cfg.spawn.clearance_m)


@dataclass
class FootprintStats:
    z_max: np.ndarray
    z_mean: np.ndarray
    slope_deg: np.ndarray
    downhill_az: np.ndarray   # radians, ENU atan2 convention (0 = +x/east, CCW)
    roughness: np.ndarray     # RMS residual about the LS plane [m]


class DiscSampler:
    """Exact statistics of a grid over a disc of fixed radius centred on grid nodes."""

    def __init__(self, grid: Grid, radius: float, chunk: int = 512):
        self.grid = grid
        self.radius = float(radius)
        h = grid.spacing
        n = int(math.ceil(self.radius / h))
        dj, di = np.mgrid[-n:n + 1, -n:n + 1]
        m = (di * h) ** 2 + (dj * h) ** 2 <= self.radius ** 2 + 1e-12
        self.di = di[m].astype(np.int64)
        self.dj = dj[m].astype(np.int64)
        self.n_half = n
        X = np.column_stack([self.di * h, self.dj * h, np.ones(self.di.size)])
        self.pinv = np.linalg.pinv(X)          # (3, n): plane = pinv @ z
        self.X = X
        self.chunk = int(chunk)

    def valid(self, i: np.ndarray, j: np.ndarray) -> np.ndarray:
        n = self.n_half
        return (i >= n) & (j >= n) & (i < self.grid.nx - n) & (j < self.grid.ny - n)

    def stats(self, i: np.ndarray, j: np.ndarray) -> FootprintStats:
        """Statistics at nodes (i, j) (column, row); all must satisfy :meth:`valid`."""
        i = np.asarray(i, dtype=np.int64).ravel()
        j = np.asarray(j, dtype=np.int64).ravel()
        if not np.all(self.valid(i, j)):
            raise ValueError("disc centre too close to the grid edge")
        k = i.size
        out = {name: np.empty(k) for name in ("z_max", "z_mean", "slope", "az", "rough")}
        z = self.grid.z
        for a in range(0, k, self.chunk):
            b = min(k, a + self.chunk)
            rows = j[a:b, None] + self.dj[None, :]
            cols = i[a:b, None] + self.di[None, :]
            vals = np.asarray(z[rows, cols], dtype=np.float64)        # (m, n)
            coef = vals @ self.pinv.T                                 # (m, 3): gx, gy, c
            resid = vals - coef @ self.X.T
            gx, gy = coef[:, 0], coef[:, 1]
            out["z_max"][a:b] = vals.max(axis=1)
            out["z_mean"][a:b] = vals.mean(axis=1)
            out["slope"][a:b] = np.degrees(np.arctan(np.hypot(gx, gy)))
            out["az"][a:b] = np.arctan2(-gy, -gx)
            out["rough"][a:b] = np.sqrt(np.mean(resid ** 2, axis=1))
        return FootprintStats(out["z_max"], out["z_mean"], out["slope"], out["az"], out["rough"])

    def node_of(self, x: float, y: float) -> tuple[int, int]:
        col, row = self.grid.fractional_index(np.array([x]), np.array([y]))
        return int(np.rint(col[0])), int(np.rint(row[0]))


def spawn_z(collision: Grid, x: float, y: float, cfg: Config, sampler: DiscSampler | None = None) -> float:
    """Spawn height (base_link z) at an arbitrary (x, y): max C over the footprint disc
    (radius enlarged by one node spacing because the centre is snapped to a node) + offsets."""
    r = footprint_radius(cfg) + collision.spacing
    if sampler is None or abs(sampler.radius - r) > 1e-9:
        sampler = DiscSampler(collision, r)
    i, j = sampler.node_of(x, y)
    st = sampler.stats(np.array([i]), np.array([j]))
    return float(st.z_max[0]) + spawn_height_offset(cfg)


# ---------------------------------------------------------------------------------- sites
@dataclass
class Site:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    description: str
    ground_z_max: float          # max collision height under the footprint disc
    ground_z_center: float       # collision height at (x, y)
    ground_z_visual: float | None
    slope_deg: float
    roughness_m: float
    relaxed: list[str] = field(default_factory=list)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = {
            "x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4),
            "yaw": round(self.yaw, 6), "yaw_deg": round(math.degrees(self.yaw), 3),
            "description": self.description,
            "ground_z_max": round(self.ground_z_max, 4),
            "ground_z_center": round(self.ground_z_center, 4),
            "ground_z_visual": None if self.ground_z_visual is None else round(self.ground_z_visual, 4),
            "footprint_slope_deg": round(self.slope_deg, 3),
            "footprint_roughness_m": round(self.roughness_m, 5),
        }
        if self.relaxed:
            d["relaxed"] = list(self.relaxed)
        if self.extra:
            d.update(_plain(self.extra))
        return d


@dataclass
class SiteSearchResult:
    sites: dict[str, Site]
    missing: dict[str, str]
    params: dict[str, Any]


def _plain(o: Any) -> Any:
    if isinstance(o, dict):
        return {str(k): _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return round(float(o), 6)
    if isinstance(o, float):
        return round(o, 6)
    if isinstance(o, np.ndarray):
        return _plain(o.tolist())
    return o


def default_region(layout: Layout) -> Rect:
    return layout.zone_rect("core")


class SiteFinder:
    """Shared state for the site searches (see the module docstring)."""

    def __init__(self, cfg: Config, collision: Grid, rocks: Any = None,
                 height_fn: HeightFn | None = None, region: Rect | None = None):
        self.cfg = cfg
        self.C = collision
        self.rocks = RockTable.from_any(rocks)
        self.H = height_fn
        sp = cfg.spawn
        self.r_fp = footprint_radius(cfg)
        self.half_w = float(sp.footprint_half_m[1])
        self.max_slope = float(sp.max_slope_deg)
        self.rock_free_D = float(sp.rock_free_D_m)
        self.turn_rock_free_D = float(sp.get("turn_rock_free_D_m", self.rock_free_D))
        self.rock_free_r = self.r_fp + float(sp.rock_free_margin_m)
        self.flat_path = float(sp.get("flat_path_m", FLAT_PATH_M))
        self.offset = spawn_height_offset(cfg)
        if region is None:
            region = Layout(cfg).zone_rect("core")
        gx0, gx1, gy0, gy1 = collision.extent
        m = EDGE_MARGIN_M
        self.region = Rect(max(region.x0, gx0) + m, min(region.x1, gx1) - m,
                           max(region.y0, gy0) + m, min(region.y1, gy1) - m)
        if self.region.width <= 0 or self.region.height <= 0:
            raise ValueError(f"search region {region} does not overlap the collision grid {collision.extent}")
        self.sampler = DiscSampler(collision, self.r_fp)
        self.idx5 = RockIndex(self.rocks.subset(self.rocks.collide) if len(self.rocks) else self.rocks,
                              self.rock_free_D)
        self.idx10 = RockIndex(self.rocks, SLOPE_ROCK_D_M)
        # turn site: every core rock of the threshold size counts (collide flag or not)
        self.idx_turn = RockIndex(self.rocks, self.turn_rock_free_D)
        self.headings = np.arange(N_HEADINGS) * (2 * math.pi / N_HEADINGS)
        self._cand: dict[str, np.ndarray] | None = None

    # ----------------------------------------------------------------- candidates
    def candidates(self) -> dict[str, np.ndarray]:
        """Footprint statistics on the candidate lattice (computed once)."""
        if self._cand is not None:
            return self._cand
        C = self.C
        step = max(1, int(round(CANDIDATE_STEP_M / C.spacing)))
        i0 = int(math.ceil((self.region.x0 - C.x0) / C.spacing))
        i1 = int(math.floor((self.region.x1 - C.x0) / C.spacing))
        j0 = int(math.ceil((self.region.y0 - C.y0) / C.spacing))
        j1 = int(math.floor((self.region.y1 - C.y0) / C.spacing))
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1, step), np.arange(j0, j1 + 1, step))
        ii, jj = ii.ravel(), jj.ravel()
        ok = self.sampler.valid(ii, jj)
        ii, jj = ii[ok], jj[ok]
        st = self.sampler.stats(ii, jj)
        x = C.x0 + ii * C.spacing
        y = C.y0 + jj * C.spacing
        self._cand = {"i": ii, "j": jj, "x": x, "y": y, "dist": np.hypot(x, y), "z_max": st.z_max,
                      "z_mean": st.z_mean, "slope": st.slope_deg, "az": st.downhill_az, "rough": st.roughness}
        return self._cand

    def _make_site(self, name: str, i: int, j: int, yaw: float, description: str,
                   relaxed: list[str], extra: dict[str, Any], x: float | None = None,
                   y: float | None = None) -> Site:
        C = self.C
        st = self.sampler.stats(np.array([i]), np.array([j]))
        if x is None or y is None:
            # site on a grid node: the exact disc of radius r_fp
            x = C.x0 + i * C.spacing
            y = C.y0 + j * C.spacing
            z = float(st.z_max[0]) + self.offset
        else:
            # arbitrary point: disc enlarged by one node spacing (centre snapped to a node)
            z = spawn_z(C, x, y, self.cfg)
        zc = float(C.sample(np.array([x]), np.array([y]))[0])
        zv = None if self.H is None else float(np.asarray(self.H(np.array([x]), np.array([y]))).ravel()[0])
        return Site(name=name, x=float(x), y=float(y), z=z, yaw=float(_wrap(yaw)), description=description,
                    ground_z_max=z - self.offset, ground_z_center=zc, ground_z_visual=zv,
                    slope_deg=float(st.slope_deg[0]), roughness_m=float(st.roughness[0]),
                    relaxed=relaxed, extra=extra)

    def _path_stats(self, x0: float, y0: float, yaw: float, s_values: np.ndarray) -> FootprintStats:
        c, s = math.cos(yaw), math.sin(yaw)
        xs = x0 + c * s_values
        ys = y0 + s * s_values
        col, row = self.C.fractional_index(xs, ys)
        i = np.rint(col).astype(np.int64)
        j = np.rint(row).astype(np.int64)
        ok = self.sampler.valid(i, j)
        if not np.all(ok):
            nan = np.full(s_values.shape, np.nan)
            return FootprintStats(nan, nan, np.full(s_values.shape, 90.0), nan, np.full(s_values.shape, np.inf))
        return self.sampler.stats(i, j)

    def _roughness_limit(self, mask: np.ndarray) -> float:
        cfg_lim = self.cfg.spawn.get("max_roughness_m")
        if cfg_lim is not None:
            return float(cfg_lim)
        r = self.candidates()["rough"][mask]
        return float(np.median(r)) if r.size else float("inf")

    # ----------------------------------------------------------------- flat / turn
    def _flat_like(self, name: str, clear_radius: float, description: str,
                   index: RockIndex | None = None) -> Site | None:
        """Nearest slope-, rock- and roughness-qualified candidate; ``index`` holds the rocks that
        must stay outside ``clear_radius`` (default: collide rocks >= ``rock_free_D_m``)."""
        index = self.idx5 if index is None else index
        rock_D = index.min_D
        cand = self.candidates()
        if cand["x"].size == 0:
            return None
        slope_ok = cand["slope"] <= self.max_slope
        nrock = index.count_within(cand["x"], cand["y"], clear_radius)
        relaxed: list[str] = []
        base = slope_ok & (nrock == 0)
        if not np.any(base):
            if not np.any(slope_ok):
                relaxed.append(f"no footprint with slope <= {self.max_slope} deg; using the least sloped")
                base = cand["slope"] <= np.min(cand["slope"]) + 1e-9
            else:
                mn = int(nrock[slope_ok].min())
                relaxed.append(f"no slope-ok footprint free of rocks >= {rock_D} m within "
                               f"{clear_radius:.2f} m; using the minimum count {mn}")
                base = slope_ok & (nrock == mn)
        rlim = self._roughness_limit(base)
        ok = base & (cand["rough"] <= rlim)
        if not np.any(ok):
            ok = base
        k = int(np.flatnonzero(ok)[np.argmin(cand["dist"][ok])])
        x, y = float(cand["x"][k]), float(cand["y"][k])
        yaw, path = self._best_heading(x, y)
        extra = {"criteria": {"max_slope_deg": self.max_slope, "rock_free_D_m": rock_D,
                              "rock_free_radius_m": round(clear_radius, 3), "roughness_limit_m": round(rlim, 5)},
                 "rocks_in_clear_radius": int(nrock[k]), "path": path}
        return self._make_site(name, int(cand["i"][k]), int(cand["j"][k]), yaw, description, relaxed, extra)

    def _best_heading(self, x: float, y: float) -> tuple[float, dict[str, Any]]:
        """Heading whose forward corridor (flat_path_m) is the most benign."""
        best = None
        s_vals = np.arange(0.0, self.flat_path + 1e-9, 1.0)
        for yaw in self.headings:
            n = len(self.idx5.in_corridor(x, y, yaw, 0.0, self.flat_path + self.r_fp, self.r_fp))
            st = self._path_stats(x, y, yaw, s_vals)
            smax = float(np.nanmax(st.slope_deg))
            rmax = float(np.nanmax(st.roughness))
            key = (n, smax > self.max_slope, round(smax, 1), rmax)
            if best is None or key < best[0]:
                best = (key, yaw, {"length_m": self.flat_path, "rocks_ge_D_in_corridor": n,
                                   "max_slope_deg": round(smax, 2), "max_roughness_m": round(rmax, 4)})
        assert best is not None
        return float(best[1]), best[2]

    def flat(self) -> Site | None:
        return self._flat_like("flat", self.rock_free_r,
                               "nearest flat site to the core centre (spawn, standard drive test)")

    def turn(self) -> Site | None:
        return self._flat_like("turn", TURN_CLEAR_RADIUS_M,
                               f"flat site with no rocks >= {self.turn_rock_free_D} m within {TURN_CLEAR_RADIUS_M} m "
                               "(in-place turn test)", index=self.idx_turn)

    # ----------------------------------------------------------------- slope
    def slope(self) -> tuple[Site | None, str]:
        cand = self.candidates()
        if cand["x"].size == 0:
            return None, "no candidates"
        L = SLOPE_LENGTH_M
        s = np.linspace(0.0, L, int(round(L / 0.1)) + 1)
        best = None
        n_valid = 0
        xs_all, ys_all = cand["x"], cand["y"]
        for yaw in self.headings:
            c, sn = math.cos(yaw), math.sin(yaw)
            for a in range(0, xs_all.size, 2048):
                xs = xs_all[a:a + 2048]
                ys = ys_all[a:a + 2048]
                px = xs[:, None] + c * s[None, :]
                py = ys[:, None] + sn * s[None, :]
                inside = self.C.contains(px, py).all(axis=1)
                z = self.C.sample(px, py)                                 # (m, ns)
                A = np.column_stack([s, np.ones_like(s)])
                coef, *_ = np.linalg.lstsq(A, z.T, rcond=None)            # (2, m)
                g = coef[0]
                ang = np.degrees(np.arctan(g))
                fit_rms = np.sqrt(np.mean((z - (coef.T @ A.T)) ** 2, axis=1))
                ok = inside & (ang >= SLOPE_RANGE_DEG[0]) & (ang <= SLOPE_RANGE_DEG[1]) & (fit_rms <= SLOPE_MAX_FIT_RMS_M)
                if not np.any(ok):
                    continue
                # 1 m sub-slopes
                step = int(round(1.0 / 0.1))
                dz = z[:, step::step] - z[:, :-step:step]
                loc = np.degrees(np.arctan(dz / 1.0))
                ok &= (loc.min(axis=1) >= SLOPE_LOCAL_RANGE_DEG[0]) & (loc.max(axis=1) <= SLOPE_LOCAL_RANGE_DEG[1])
                # cross slope at 3 stations (+-0.5 m)
                cross_ok = np.ones_like(ok)
                cross_max = np.zeros(ok.shape)
                for st in (0.2 * L, 0.5 * L, 0.8 * L):
                    lx = xs + c * st
                    ly = ys + sn * st
                    zl = self.C.sample(lx - sn * 0.5, ly + c * 0.5)
                    zr = self.C.sample(lx + sn * 0.5, ly - c * 0.5)
                    cr = np.degrees(np.arctan(np.abs(zl - zr) / 1.0))
                    cross_max = np.maximum(cross_max, cr)
                    cross_ok &= cr <= SLOPE_MAX_CROSS_DEG
                ok &= cross_ok
                for k in np.flatnonzero(ok):
                    n_valid += 1
                    x0, y0 = float(xs[k]), float(ys[k])
                    nr = len(self.idx10.in_corridor(x0, y0, float(yaw), -0.6, L + 0.6, self.r_fp))
                    score = (nr, abs(float(ang[k]) - 12.5) / 2.5 + float(np.hypot(x0, y0)) / 50.0)
                    if best is None or score < best[0]:
                        best = (score, a + int(k), float(yaw), float(ang[k]), float(fit_rms[k]),
                                float(loc[k].min()), float(loc[k].max()), float(cross_max[k]), nr)
        if best is None:
            return None, (f"no straight {L} m run with a {SLOPE_RANGE_DEG[0]}-{SLOPE_RANGE_DEG[1]} deg line-fit slope "
                          f"(1 m sub-slopes {SLOPE_LOCAL_RANGE_DEG}, cross slope <= {SLOPE_MAX_CROSS_DEG} deg)")
        _, k, yaw, ang, rms, lmin, lmax, cmax, nr = best
        extra = {"slope_run": {"length_m": L, "mean_slope_deg": round(ang, 2), "fit_rms_m": round(rms, 4),
                               "sub_slope_range_deg": [round(lmin, 2), round(lmax, 2)],
                               "max_cross_slope_deg": round(cmax, 2),
                               "rise_m": round(L * math.tan(math.radians(ang)), 3),
                               "rocks_ge_0.10m_in_corridor": nr, "valid_runs_found": n_valid}}
        site = self._make_site("slope", int(cand["i"][k]), int(cand["j"][k]), yaw,
                               f"bottom of a {L:g} m, {ang:.1f} deg slope, facing uphill (climb test)", [], extra)
        return site, ""

    # ----------------------------------------------------------------- rock
    def rock(self) -> tuple[Site | None, str]:
        r = self.rocks
        if len(r) == 0:
            return None, "empty rock catalog"
        sel = (r.D >= ROCK_D_RANGE_M[0]) & (r.D <= ROCK_D_RANGE_M[1]) & r.collide
        sel &= self.region.contains(r.x, r.y)
        if not np.any(sel):
            return None, f"no collide rock with {ROCK_D_RANGE_M[0]} <= D <= {ROCK_D_RANGE_M[1]} m in the region"
        targets = np.flatnonzero(sel)
        targets = targets[np.argsort(np.hypot(r.x[targets], r.y[targets]))]
        hw = self.half_w + 0.1
        s_path = np.arange(0.0, ROCK_DISTANCE_M + ROCK_PATH_PAST_M + 1e-9, 1.0)
        tried = 0
        for t in targets:
            tx, ty, tD = float(r.x[t]), float(r.y[t]), float(r.D[t])
            # lone: no other >= rock_free_D rock within ROCK_LONE_RADIUS_M
            near = self.idx5.within(tx, ty, ROCK_LONE_RADIUS_M)
            near = [k for k in near if self.idx5.rocks.id[k] != r.id[t]]
            if near:
                continue
            for yaw in self.headings:
                tried += 1
                c, s = math.cos(yaw), math.sin(yaw)
                # spawn so that the target is ROCK_DISTANCE_M ahead on the left wheel line
                x0 = tx - ROCK_DISTANCE_M * c + ROCK_WHEEL_OFFSET_M * s
                y0 = ty - ROCK_DISTANCE_M * s - ROCK_WHEEL_OFFSET_M * c
                if not self.region.contains(x0, y0):
                    continue
                inc = self.idx5.in_corridor(x0, y0, yaw, -ROCK_PATH_BEHIND_M,
                                            ROCK_DISTANCE_M + ROCK_PATH_PAST_M, hw)
                inc = [k for k in inc if self.idx5.rocks.id[k] != r.id[t]]
                if inc:
                    continue
                st = self._path_stats(x0, y0, yaw, s_path)
                if not np.all(st.slope_deg <= self.max_slope):
                    continue
                i, j = self.sampler.node_of(x0, y0)
                extra = {"rock": {"id": int(r.id[t]), "x": round(tx, 4), "y": round(ty, 4), "D": round(tD, 4),
                                  "exposed_h": None if not np.isfinite(r.exposed_h[t]) else round(float(r.exposed_h[t]), 4),
                                  "distance_ahead_m": ROCK_DISTANCE_M, "wheel": "left",
                                  "lateral_offset_m": ROCK_WHEEL_OFFSET_M},
                         "path": {"from_m": -ROCK_PATH_BEHIND_M, "to_m": ROCK_DISTANCE_M + ROCK_PATH_PAST_M,
                                  "half_width_m": hw, "max_slope_deg": round(float(np.max(st.slope_deg)), 2),
                                  "other_rocks_ge_D": 0, "targets_tried": tried}}
                site = self._make_site("rock", i, j, float(yaw),
                                       f"2 m before a lone {tD * 100:.1f} cm rock on a clear straight path; "
                                       "the left wheels roll over it (traverse test)", [], extra, x=x0, y=y0)
                return site, ""
        return None, (f"no lone {ROCK_D_RANGE_M[0]}-{ROCK_D_RANGE_M[1]} m rock with a clear "
                      f"{ROCK_PATH_BEHIND_M + ROCK_DISTANCE_M + ROCK_PATH_PAST_M:g} m approach ({tried} poses tried)")


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def find_sites(cfg: Config, collision: Grid, rocks: Any = None, height_fn: HeightFn | None = None,
               region: Rect | None = None, names: list[str] | None = None) -> SiteSearchResult:
    """Find the test sites (``spawn.test_sites`` by default: flat, slope, rock, turn)."""
    names = list(names or cfg.spawn.get("test_sites", ["flat", "slope", "rock", "turn"]))
    finder = SiteFinder(cfg, collision, rocks, height_fn, region)
    sites: dict[str, Site] = {}
    missing: dict[str, str] = {}
    for name in names:
        if name == "flat":
            s = finder.flat()
            reason = "no candidate positions"
        elif name == "turn":
            s = finder.turn()
            reason = "no candidate positions"
        elif name == "slope":
            s, reason = finder.slope()
        elif name == "rock":
            s, reason = finder.rock()
        else:
            raise ValueError(f"unknown test site '{name}'")
        if s is None:
            missing[name] = reason
        else:
            sites[name] = s
    params = {"footprint_radius_m": round(finder.r_fp, 4), "spawn_height_offset_m": round(finder.offset, 4),
              "candidate_step_m": CANDIDATE_STEP_M, "candidates": int(finder.candidates()["x"].size),
              "region": [finder.region.x0, finder.region.x1, finder.region.y0, finder.region.y1],
              "rocks_considered": len(finder.rocks)}
    return SiteSearchResult(sites=sites, missing=missing, params=params)
