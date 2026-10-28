"""Composition of the visual height function H and the terrain build (docs/DESIGN.md sections 2-4).

    H(x, y) = B(x, y) + w_ring(x, y) * [F_coarse + K_big + T] + w_core(x, y) * [F_fine + K_small]

(fillets are added later by the rocks stage; everything here is H0 = H without fillets).

T is the angle-of-repose (talus) correction of ``terrain.talus`` (FID-03): a cached global 10 cm field,
computed once from B + w_ring (F_coarse + K_big) + w_core K_small by a mass-conserving relaxation of walls
steeper than ``max_slope_deg`` and sampled with a local Catmull-Rom kernel (exactly 0 away from relaxed
walls).
F_coarse and F_fine have separate gains (``fbm_gain`` / ``fbm_gain_fine``, FID-06).

One global function, one evaluation path
    ``TerrainModel.evaluate(x, y, zone)`` is the single implementation and H0 does not depend on the
    zone: every zone sees the same value at the same point (``zone`` is validated and kept for the API).
    Every grid (``core_tile_height``, ``ring_height``, ``far_height``, ``horizon_height``) is that
    function evaluated at lattice-exact node coordinates ``(l0 + i * step) * 0.01``, and ``height_at``
    is the same function at arbitrary points, rounded to float32 like the grids. Points within 1e-8 m of
    the 1 cm lattice are snapped to it first, so ``height_at`` at grid nodes returns exactly the stored
    grid values and shared boundary nodes of different zones are bit-identical.

Terms
    * ``B``: the band-limited DTM (terrain.base), nested from the four extractions. Each extraction is
      wider than its zone square; with ``extent_weight(zone)`` = 1 inside the zone square and a raised
      cosine to 0 over the extraction's extra border (core.margin_m, base.RING_MARGIN_M,
      base.FAR_SKIRT_M)::

          B = B_horizon;  then for zone in (far, ring, core):  B = B_zone        where v_zone == 1
                                                               B + v (B_zone - B) where 0 < v_zone < 1
      So inside the core B is exactly the 1 cm core extraction, inside the ring the 25 cm one, etc., and
      outer zones reproduce inner values exactly at their shared boundaries. At nodes of an extraction
      its stored node value is used; elsewhere cubic B-spline interpolation (bilinear for the 1 cm core
      grid). The blends absorb the small differences between extractions (dtm_extract's vertical
      sampling iteration in the 2.6 km far extraction: ~1 mm at 250 m; the block-averaged horizon:
      up to 2.4 m at the far boundary).
    * ``F_coarse`` (unit field x sqrt(fbm_gain)) via ``fbm.CubicSampler`` (= Grid.sample(order=3)),
      ``K_big`` via ``craters.crater_relief``, ``T`` via ``talus.TalusSampler``; only where w_ring > 0.
    * ``F_fine`` (unit field x sqrt(fbm_gain_fine)) by node index (bilinear off the lattice),
      ``K_small``; only where w_core > 0.

Fades (raised cosine, product of the two axes so the weight is C1 everywhere)
    ``edge_fade(x, y, half, width) = s((half - |x|) / width) * s((half - |y|) / width)``,
    ``s(t) = 0 (t <= 0), (1 - cos(pi t)) / 2 (0 < t < 1), 1 (t >= 1)``.
    w_core: half = 50 m, width = core.edge_fade_m (config; 3 m proposed by FID-04, was 1 m);
    w_ring: half = 250 m, width = ring.edge_fade_m (20 m). Both are exactly 0 on their zone boundary
    and exactly 1 inside the fade band.

CLI
    python -m lunar_world_gen.terrain.compose --all [--preset fast] [--set key=value ...] [--rebuild]
    builds and caches (build_dir/terrain/): base/*, fbm/*, craters/*, talus/*, core_tiles/tile_ii_jj.*,
    the ring/far/horizon height grids and manifest.json (inputs, hashes, timings, peak memory, disk use,
    cross-zone consistency checks, crater counts, talus windows).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import resource
import sys
import time
from pathlib import Path

import numpy as np

from ..config import Config, config_hash, load_config
from ..grid import LATTICE_M, Grid, from_lattice, to_lattice
from ..layout import Layout, TileId
from . import base as tbase
from . import craters as tcraters
from . import fbm as tfbm
from . import talus as ttalus

# 2: global nested B (extent blends) instead of a per-zone B
# 3: talus term T (FID-03), separate fine fBm gain (FID-06), crater sets v2 (FID-05/16)
TERRAIN_VERSION = "3"
PARTS = ("B", "Fc", "Kb", "Ff", "Ks", "T")
ZONES = ("core", "ring", "far", "horizon")
_SNAP_TOL = 1e-6   # lattice units (1e-8 m)


# ---------------------------------------------------------------------------------------- fades
def raised_cosine(t: np.ndarray) -> np.ndarray:
    """0 for t <= 0, (1 - cos(pi t)) / 2 on (0, 1), 1 for t >= 1 (C1)."""
    t = np.clip(np.asarray(t, dtype=np.float64), 0.0, 1.0)
    return 0.5 * (1.0 - np.cos(np.pi * t))


def edge_fade(x: np.ndarray, y: np.ndarray, half: float, width: float) -> np.ndarray:
    """1 inside the square |x|,|y| <= half - width, 0 on/outside its boundary, raised cosine between."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if width <= 0:
        return ((np.abs(x) < half) & (np.abs(y) < half)).astype(np.float64)
    return raised_cosine((half - np.abs(x)) / width) * raised_cosine((half - np.abs(y)) / width)


def w_core(cfg: Config, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return edge_fade(x, y, cfg.zones.core.size_m / 2.0, float(cfg.zones.core.edge_fade_m))


def w_ring(cfg: Config, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return edge_fade(x, y, cfg.zones.ring.size_m / 2.0, float(cfg.zones.ring.edge_fade_m))


def extent_weight(cfg: Config, zone: str, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Weight of a zone's own B extraction in the nested B: 1 inside the zone square, raised cosine to 0
    across the extraction's extra border (core: core.margin_m, ring: RING_MARGIN_M, far: FAR_SKIRT_M)."""
    spec = tbase.base_spec(cfg, zone)
    half = spec.zone_half_m if zone != "core" else cfg.zones.core.size_m / 2.0
    w = spec.half_m - half
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    if w <= 0:
        return ((np.abs(x) <= half) & (np.abs(y) <= half)).astype(np.float64)
    return (1.0 - raised_cosine((np.abs(x) - half) / w)) * (1.0 - raised_cosine((np.abs(y) - half) / w))


def horizon_blend_weight(cfg: Config, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Weight of B_far over the horizon's own B: 1 inside the far square, 0 beyond FAR_SKIRT_M outside it."""
    return extent_weight(cfg, "far", x, y)


# ---------------------------------------------------------------------------------------- helpers
def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


def _lattice(v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(nearest integer lattice coordinate, whether v is within 1e-8 m of it)."""
    q = v / LATTICE_M
    r = np.rint(q)
    return r.astype(np.int64), np.abs(q - r) <= _SNAP_TOL


def _peak_rss_mb() -> dict:
    s = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    c = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / 1024.0
    return {"self_mb": round(s, 1), "children_mb": round(c, 1)}


def _du(paths) -> int:
    total = 0
    for p in paths:
        p = Path(p)
        if p.is_file():
            total += p.stat().st_size
        elif p.is_dir():
            total += sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
    return total


# process-level caches of the (gain-independent) components, keyed by their cache keys
_BASE: dict[tuple, Grid] = {}
_BASE_SAMPLER: dict[tuple, tfbm.CubicSampler] = {}
_UNIT: dict[tuple, Grid] = {}
_COARSE_SAMPLER: dict[tuple, tfbm.CubicSampler] = {}
_CRATERS: dict[tuple, tcraters.CraterSet] = {}
_TALUS: dict[tuple, ttalus.TalusSampler] = {}


def clear_caches() -> None:
    for c in (_BASE, _BASE_SAMPLER, _UNIT, _COARSE_SAMPLER, _CRATERS, _TALUS):
        c.clear()


def _scales(fbm_scale) -> tuple[float, float] | None:
    """fbm_scale argument -> (coarse, fine) amplitude factors (None: the config gains)."""
    if fbm_scale is None:
        return None
    if isinstance(fbm_scale, (tuple, list)):
        return float(fbm_scale[0]), float(fbm_scale[1])
    return float(fbm_scale), float(fbm_scale)


class TerrainModel:
    """Evaluates H for a config; components are loaded lazily from (or built into) build_dir/terrain."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.layout = Layout(cfg)
        z = cfg.zones
        self.core_half = z.core.size_m / 2.0
        self.ring_half = z.ring.size_m / 2.0
        self.far_half = z.far.size_m / 2.0
        self.fade_core = float(z.core.edge_fade_m)
        self.fade_ring = float(z.ring.edge_fade_m)
        self.gains = tfbm.fbm_gains(cfg)
        self.scale_c = tfbm.fbm_scale(cfg, "coarse")
        self.scale_f = tfbm.fbm_scale(cfg, "fine")
        self.scale = self.scale_c          # backward compatible name (coarse band)
        self.talus_params = ttalus.talus_params(cfg)
        self._build_dir = str(self.layout.build_dir)
        self._keys: dict | None = None

    # ------------------------------------------------------------------ components
    def base(self, zone: str) -> Grid:
        k = (self._build_dir, zone, self.component_keys()["base"][zone])
        if k not in _BASE:
            _BASE[k] = tbase.extract_base(self.cfg, zone)
        return _BASE[k]

    def base_sampler(self, zone: str) -> tfbm.CubicSampler:
        k = (self._build_dir, zone, self.component_keys()["base"][zone])
        if k not in _BASE_SAMPLER:
            _BASE_SAMPLER[k] = tfbm.CubicSampler(self.base(zone))
        return _BASE_SAMPLER[k]

    def unit(self, band: str) -> Grid:
        k = (self._build_dir, band, self.component_keys()["fbm"][band])
        if k not in _UNIT:
            _UNIT[k] = tfbm.unit_field(self.cfg, band)
        return _UNIT[k]

    def coarse_sampler(self) -> tfbm.CubicSampler:
        k = (self._build_dir, self.component_keys()["fbm"]["coarse"])
        if k not in _COARSE_SAMPLER:
            _COARSE_SAMPLER[k] = tfbm.CubicSampler(self.unit("coarse"))
        return _COARSE_SAMPLER[k]

    def craters(self, which: str) -> tcraters.CraterSet:
        k = (self._build_dir, which, self.component_keys()["craters"][which])
        if k not in _CRATERS:
            _CRATERS[k] = tcraters.get_craters(self.cfg, which)
            _CRATERS[k].index()
        return _CRATERS[k]

    def talus_field(self, rebuild: bool = False, log=print) -> Grid:
        """The cached talus correction grid T (computed on first use; see terrain.talus)."""
        return ttalus.load_or_build(self, self.talus_key, rebuild=rebuild, log=log)

    def talus_sampler(self) -> ttalus.TalusSampler:
        k = (self._build_dir, self.talus_key)
        if k not in _TALUS:
            _TALUS[k] = ttalus.TalusSampler(self.talus_field())
        return _TALUS[k]

    def component_keys(self) -> dict:
        """Cache keys of the gain-independent components (base grids, unit fBm fields, crater sets)."""
        if self._keys is None:
            self._keys = {
                "base": {z: _hash(tbase.base_inputs(self.cfg, z)) for z in tbase.BASE_ZONES},
                "fbm": {b: _hash(tfbm.band_params(self.cfg, b)) for b in tfbm.BANDS},
                "craters": {w: _hash(tcraters.crater_params(self.cfg, w)) for w in tcraters.WHICH},
            }
        return self._keys

    @property
    def talus_key(self) -> str:
        """Key of the talus field: its input Hs = B + w_ring (F_coarse + K_big) + w_core K_small and the
        limiter parameters."""
        z = self.cfg.zones
        return _hash({"talus": self.talus_params, "base": self.component_keys()["base"],
                      "fbm_coarse": self.component_keys()["fbm"]["coarse"],
                      "craters": self.component_keys()["craters"], "gain_coarse": self.gains["coarse"],
                      "input_parts": list(ttalus.INPUT_PARTS),
                      "zones": {k: dict(z[k]) for k in ZONES}, "far_skirt_m": tbase.FAR_SKIRT_M,
                      "terrain_version": TERRAIN_VERSION})

    @property
    def key(self) -> str:
        """Cache key of every composed height grid (components + gains + talus + fades + geometry)."""
        z = self.cfg.zones
        return _hash({"version": TERRAIN_VERSION, "components": self.component_keys(),
                      "fbm_gain": self.gains["coarse"], "fbm_gain_fine": self.gains["fine"],
                      "talus": self.talus_key,
                      "zones": {k: dict(z[k]) for k in ZONES}, "far_skirt_m": tbase.FAR_SKIRT_M})

    # ------------------------------------------------------------------ lookups
    def _grid_lookup(self, g: Grid, x, y, lx, ly, onlat, interp) -> np.ndarray:
        """Node values where (x, y) is a node of g (exact), else interp(x, y)."""
        lx0, ly0 = g.lattice_origin()
        st = g.lattice_step()
        di = lx - lx0
        dj = ly - ly0
        node = onlat & (di % st == 0) & (dj % st == 0)
        ii = di // st
        jj = dj // st
        node &= (ii >= 0) & (ii < g.nx) & (jj >= 0) & (jj < g.ny)
        out = np.empty(x.shape, dtype=np.float64)
        if node.any():
            out[node] = g.z[jj[node], ii[node]]
        off = ~node
        if off.any():
            out[off] = interp(x[off], y[off])
        return out

    def _extraction_values(self, zone: str, x, y, lx, ly, onlat) -> np.ndarray:
        """Values of one zone's own extraction (node value at its nodes, else interpolation)."""
        g = self.base(zone)
        interp = (lambda a, b, g=g: g.sample(a, b, order=1)) if zone == "core" else self.base_sampler(zone)
        return self._grid_lookup(g, x, y, lx, ly, onlat, interp)

    _NEST = ("core", "ring", "far", "horizon")

    def _base_values(self, x, y, lx, ly, onlat, level: int = 0) -> np.ndarray:
        """Nested global B (see the module docstring), recursing from the core outward."""
        zone = self._NEST[level]
        if zone == "horizon":
            return self._extraction_values("horizon", x, y, lx, ly, onlat)
        v = extent_weight(self.cfg, zone, x, y)
        out = np.empty(x.shape, dtype=np.float64)
        full = v >= 1.0
        if full.any():
            s = np.nonzero(full)[0]
            out[s] = self._extraction_values(zone, x[s], y[s], lx[s], ly[s], onlat[s])
        rest = np.nonzero(~full)[0]
        if rest.size:
            outer = self._base_values(x[rest], y[rest], lx[rest], ly[rest], onlat[rest], level + 1)
            vr = v[rest]
            part = vr > 0.0
            if part.any():
                p = rest[part]
                inner = self._extraction_values(zone, x[p], y[p], lx[p], ly[p], onlat[p])
                outer[part] = outer[part] + vr[part] * (inner - outer[part])
            out[rest] = outer
        return out

    # ------------------------------------------------------------------ evaluation
    def evaluate(self, x: np.ndarray, y: np.ndarray, zone: str, parts=PARTS,
                 fbm_scale: float | tuple[float, float] | None = None) -> np.ndarray:
        """H0 (or the selected parts of it) at points, float64, shaped like x.

        ``fbm_scale`` overrides the amplitude factors of the fBm bands: one float for both bands or
        (coarse, fine); None uses sqrt(fbm_gain) / sqrt(fbm_gain_fine) of the config. T is always the
        cached field of the config (it does not follow an fbm_scale override)."""
        if zone not in ZONES:
            raise ValueError(f"zone must be one of {ZONES}")
        bad = set(parts) - set(PARTS)
        if bad:
            raise ValueError(f"unknown parts {bad}")
        sc = _scales(fbm_scale)
        scale, scale_f = (self.scale_c, self.scale_f) if sc is None else sc
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = np.broadcast_shapes(x.shape, y.shape)
        x = np.broadcast_to(x, shape).ravel()
        y = np.broadcast_to(y, shape).ravel()
        lx, onx = _lattice(x)
        ly, ony = _lattice(y)
        xs = np.where(onx, lx * LATTICE_M, x)
        ys = np.where(ony, ly * LATTICE_M, y)
        onlat = onx & ony
        out = np.zeros(xs.shape, dtype=np.float64)
        if "B" in parts:
            out += self._base_values(xs, ys, lx, ly, onlat)
        use_t = "T" in parts and self.talus_params["enabled"]
        if ("Fc" in parts or "Kb" in parts or use_t):
            sel = np.nonzero((np.abs(xs) < self.ring_half) & (np.abs(ys) < self.ring_half))[0]
            if sel.size:
                px, py = xs[sel], ys[sel]
                s = np.zeros(sel.size)
                if "Fc" in parts and scale != 0.0:
                    s += scale * self.coarse_sampler()(px, py)
                if "Kb" in parts:
                    s += tcraters.crater_relief(self.craters("big"), px, py)
                if use_t:
                    s += self.talus_sampler()(px, py)
                out[sel] += edge_fade(px, py, self.ring_half, self.fade_ring) * s
        if ("Ff" in parts or "Ks" in parts):
            sel = np.nonzero((np.abs(xs) < self.core_half) & (np.abs(ys) < self.core_half))[0]
            if sel.size:
                px, py = xs[sel], ys[sel]
                s = np.zeros(sel.size)
                if "Ff" in parts and scale_f != 0.0:
                    gf = self.unit("fine")
                    s += scale_f * self._grid_lookup(gf, px, py, lx[sel], ly[sel], onlat[sel],
                                                     lambda a, b, g=gf: g.sample(a, b, order=1))
                if "Ks" in parts:
                    s += tcraters.crater_relief(self.craters("small"), px, py)
                out[sel] += edge_fade(px, py, self.core_half, self.fade_core) * s
        return out.reshape(shape)

    def grid(self, zone: str, x0: float, y0: float, nx: int, ny: int, spacing: float, parts=PARTS,
             fbm_scale: float | tuple[float, float] | None = None, chunk_nodes: int = 2_000_000,
             dtype=np.float32) -> Grid:
        """H0 (or parts) on a lattice-aligned node grid; node coordinates are lattice-exact."""
        lx0, ly0, st = int(to_lattice(x0)), int(to_lattice(y0)), int(to_lattice(spacing))
        xs = from_lattice(lx0 + st * np.arange(nx, dtype=np.int64))
        z = np.empty((ny, nx), dtype=dtype)
        rows = max(1, chunk_nodes // max(nx, 1))
        for j0 in range(0, ny, rows):
            j1 = min(ny, j0 + rows)
            ys = from_lattice(ly0 + st * np.arange(j0, j1, dtype=np.int64))
            X, Y = np.meshgrid(xs, ys)
            z[j0:j1] = self.evaluate(X, Y, zone, parts, fbm_scale).astype(dtype)
        sc = _scales(fbm_scale)
        gains = dict(self.gains) if sc is None else {"coarse": sc[0] ** 2, "fine": sc[1] ** 2}
        return Grid(z, from_lattice(lx0), from_lattice(ly0), from_lattice(st),
                    {"zone": zone, "parts": list(parts), "terrain_key": self.key,
                     "fbm_gain": gains["coarse"], "fbm_gain_fine": gains["fine"]})


_MODELS: dict[str, TerrainModel] = {}


def get_model(cfg: Config) -> TerrainModel:
    """Memoised TerrainModel per config content (components are shared through module caches)."""
    k = config_hash(cfg)
    if k not in _MODELS:
        if len(_MODELS) > 4:
            _MODELS.clear()
        _MODELS[k] = TerrainModel(cfg)
    return _MODELS[k]


# ---------------------------------------------------------------------------------------- public API
def core_tile_height(cfg: Config, tile: TileId, margin_m: float | None = None) -> Grid:
    """H0 of one core tile at the core spacing (1 cm) over the tile rectangle +- margin_m.

    margin_m defaults to zones.core.margin_m (0.5 m): 1101 x 1101 nodes for a 10 m tile.
    """
    lay = Layout(cfg)
    m = float(cfg.zones.core.margin_m) if margin_m is None else float(margin_m)
    r = lay.tile_rect(tile)
    sp = float(cfg.zones.core.spacing_m)
    n = int(round((r.width + 2 * m) / sp)) + 1
    g = get_model(cfg).grid("core", r.x0 - m, r.y0 - m, n, n, sp)
    g.meta.update({"tile": [tile.i, tile.j], "name": tile.name, "margin_m": m})
    return g


def core_height(cfg: Config, x0: float, x1: float, y0: float, y1: float, parts=PARTS,
                fbm_scale: float | tuple[float, float] | None = None, dtype=np.float32) -> Grid:
    """H0 (or parts) on the core lattice (1 cm) over [x0, x1] x [y0, y1] (inclusive, lattice-aligned)."""
    sp = float(cfg.zones.core.spacing_m)
    nx = int(round((x1 - x0) / sp)) + 1
    ny = int(round((y1 - y0) / sp)) + 1
    return get_model(cfg).grid("core", x0, y0, nx, ny, sp, parts, fbm_scale, dtype=dtype)


def ring_height(cfg: Config, margin_m: float = 0.0) -> Grid:
    """H0 on the ring grid (25 cm) over the 500 m square (+ margin_m <= base.RING_MARGIN_M)."""
    half = cfg.zones.ring.size_m / 2.0 + margin_m
    sp = float(cfg.zones.ring.spacing_m)
    n = int(round(2 * half / sp)) + 1
    return get_model(cfg).grid("ring", -half, -half, n, n, sp)


def far_height(cfg: Config) -> Grid:
    """H0 on the far grid (2 m) over the 2.2 km square (H = B outside the ring: w_ring = 0)."""
    half = cfg.zones.far.size_m / 2.0
    sp = float(cfg.zones.far.spacing_m)
    n = int(round(2 * half / sp)) + 1
    return get_model(cfg).grid("far", -half, -half, n, n, sp)


def horizon_height(cfg: Config) -> Grid:
    """H0 on the horizon grid (20 m) over 20 km; equals the far field at the far boundary."""
    half = cfg.zones.horizon.size_m / 2.0
    sp = float(cfg.zones.horizon.spacing_m)
    n = int(round(2 * half / sp)) + 1
    return get_model(cfg).grid("horizon", -half, -half, n, n, sp)


def height_at(cfg: Config, x: np.ndarray, y: np.ndarray, zone: str) -> np.ndarray:
    """H0 at arbitrary points of a zone, float32 (equals that zone's grids exactly at their nodes)."""
    return get_model(cfg).evaluate(x, y, zone).astype(np.float32)


# ---------------------------------------------------------------------------------------- caching
def tiles_dir(cfg: Config) -> Path:
    return Layout(cfg).stage_dir("terrain") / "core_tiles"


def tile_prefix(cfg: Config, tile: TileId) -> Path:
    return tiles_dir(cfg) / tile.name


def zone_height_prefix(cfg: Config, zone: str) -> Path:
    return Layout(cfg).stage_dir("terrain") / f"{zone}_height"


def _cached(prefix: Path, key: str) -> bool:
    j, n = prefix.with_suffix(".json"), prefix.with_suffix(".npy")
    if not (j.exists() and n.exists()):
        return False
    try:
        return json.loads(j.read_text()).get("meta", {}).get("terrain_key") == key
    except (OSError, json.JSONDecodeError):
        return False


def _save(g: Grid, prefix: Path) -> None:
    tmp = prefix.with_name(prefix.name + "_tmp")
    g.save(tmp)
    os.replace(tmp.with_suffix(".npy"), prefix.with_suffix(".npy"))
    os.replace(tmp.with_suffix(".json"), prefix.with_suffix(".json"))


def load_core_tile(cfg: Config, tile: TileId, mmap: bool = False, build: bool = True) -> Grid:
    """Cached H0 tile grid (default margin); computed and saved if missing or stale (build=True)."""
    prefix = tile_prefix(cfg, tile)
    key = get_model(cfg).key
    if not _cached(prefix, key):
        if not build:
            raise FileNotFoundError(f"{prefix}.npy missing or stale (terrain key {key})")
        g = core_tile_height(cfg, tile)
        g.meta.update(_zstats(g.z))
        _save(g, prefix)
    return Grid.load(prefix, mmap=mmap)


def load_zone_height(cfg: Config, zone: str, mmap: bool = False, build: bool = True) -> Grid:
    """Cached ring/far/horizon H0 grid; computed and saved if missing or stale (build=True)."""
    fn = {"ring": ring_height, "far": far_height, "horizon": horizon_height}[zone]
    prefix = zone_height_prefix(cfg, zone)
    key = get_model(cfg).key
    if not _cached(prefix, key):
        if not build:
            raise FileNotFoundError(f"{prefix}.npy missing or stale (terrain key {key})")
        g = fn(cfg)
        g.meta.update(_zstats(g.z))
        _save(g, prefix)
    return Grid.load(prefix, mmap=mmap)


def _zstats(z: np.ndarray) -> dict:
    z = np.asarray(z)
    return {"z_min": float(np.min(z)), "z_max": float(np.max(z)), "z_mean": float(np.mean(z, dtype=np.float64))}


def assemble_core(cfg: Config, dtype=np.float32) -> Grid:
    """The core H0 (nodes |x|, |y| <= core half) assembled from the cached tiles (no margins)."""
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    sp = float(cfg.zones.core.spacing_m)
    n = int(round(core.width / sp)) + 1
    per = int(round(lay.tile_size / sp))
    z = np.empty((n, n), dtype=dtype)
    for t in lay.tiles():
        g = load_core_tile(cfg, t, mmap=True)
        m = int(round(g.meta.get("margin_m", cfg.zones.core.margin_m) / sp))
        z[t.j * per:(t.j + 1) * per + 1, t.i * per:(t.i + 1) * per + 1] = g.z[m:m + per + 1, m:m + per + 1]
    return Grid(z, core.x0, core.y0, sp, {"assembled_from": "core_tiles", "terrain_key": get_model(cfg).key})


# ---------------------------------------------------------------------------------------- checks
def consistency_checks(cfg: Config) -> dict:
    """Cross-zone consistency of the cached grids at shared lattice points (max |difference| in m)."""
    lay = Layout(cfg)
    model = get_model(cfg)
    res: dict = {}
    sp_c = float(cfg.zones.core.spacing_m)
    core = lay.zone_rect("core")
    per = int(round(lay.tile_size / sp_c))
    m = int(round(float(cfg.zones.core.margin_m) / sp_c))
    nt = lay.n_tiles
    # tile-tile: shared boundary lines and overlapping margins
    tmax = 0.0
    tiles = {(t.i, t.j): load_core_tile(cfg, t, mmap=True) for t in lay.tiles()}
    for (i, j), g in tiles.items():
        if i + 1 < nt:
            h = tiles[(i + 1, j)]
            a = np.asarray(g.z[:, m + per - m: m + per + m + 1], dtype=np.float64)   # g's east margin band
            b = np.asarray(h.z[:, 0:2 * m + 1], dtype=np.float64)
            tmax = max(tmax, float(np.abs(a - b).max()))
        if j + 1 < nt:
            h = tiles[(i, j + 1)]
            a = np.asarray(g.z[m + per - m: m + per + m + 1, :], dtype=np.float64)
            b = np.asarray(h.z[0:2 * m + 1, :], dtype=np.float64)
            tmax = max(tmax, float(np.abs(a - b).max()))
    res["tile_tile_overlap_max_abs_m"] = tmax
    # core tiles vs ring grid on the core boundary (ring nodes)
    ring = load_zone_height(cfg, "ring", mmap=True)
    sp_r = ring.spacing
    s = np.arange(core.x0, core.x1 + 1e-9, sp_r)
    bx = np.concatenate([s, s, np.full_like(s, core.x0), np.full_like(s, core.x1)])
    by = np.concatenate([np.full_like(s, core.y0), np.full_like(s, core.y1), s, s])
    lx, _ = _lattice(bx)
    ly, _ = _lattice(by)
    bx, by = lx * LATTICE_M, ly * LATTICE_M
    zc = _tile_values(cfg, tiles, bx, by)
    zr = ring.sample(bx, by, order=1)
    res["core_vs_ring_on_core_boundary"] = _diffstats(zc, zr)
    # B_core vs B_ring at all common nodes (25 cm lattice inside the core extraction)
    bcore, bring = model.base("core"), model.base("ring")
    half = min(bcore.x1, -bcore.x0)
    q = np.arange(-half, half + 1e-9, sp_r)
    q = from_lattice(_lattice(q)[0])
    Xq, Yq = np.meshgrid(q, q)
    res["B_core_vs_B_ring_common_nodes"] = _diffstats(bcore.sample(Xq, Yq, order=1), bring.sample(Xq, Yq, order=1))
    # ring vs far at the ring boundary (far nodes)
    far = load_zone_height(cfg, "far", mmap=True)
    rr = lay.zone_rect("ring")
    s = np.arange(rr.x0, rr.x1 + 1e-9, far.spacing)
    bx = np.concatenate([s, s, np.full_like(s, rr.x0), np.full_like(s, rr.x1)])
    by = np.concatenate([np.full_like(s, rr.y0), np.full_like(s, rr.y1), s, s])
    res["ring_vs_far_on_ring_boundary"] = _diffstats(ring.sample(bx, by, order=1), far.sample(bx, by, order=1))
    # B_ring vs B_far at the 2 m nodes inside the ring extraction
    bfar = model.base("far")
    half = min(bring.x1, -bring.x0)
    q = np.arange(-half, half + 1e-9, far.spacing)
    Xq, Yq = np.meshgrid(q, q)
    res["B_ring_vs_B_far_common_nodes"] = _diffstats(bring.sample(Xq, Yq, order=1), bfar.sample(Xq, Yq, order=1))
    # far vs horizon at the far boundary (horizon nodes)
    hor = load_zone_height(cfg, "horizon", mmap=True)
    fr = lay.zone_rect("far")
    s = np.arange(fr.x0, fr.x1 + 1e-9, hor.spacing)
    bx = np.concatenate([s, s, np.full_like(s, fr.x0), np.full_like(s, fr.x1)])
    by = np.concatenate([np.full_like(s, fr.y0), np.full_like(s, fr.y1), s, s])
    res["far_vs_horizon_on_far_boundary"] = _diffstats(far.sample(bx, by, order=1), hor.sample(bx, by, order=1))
    # horizon raw block-averaged B vs far B at the far boundary (what the blend removes)
    bh = model.base("horizon")
    res["horizon_raw_B_vs_far_B_on_far_boundary"] = _diffstats(bh.sample(bx, by, order=1), bfar.sample(bx, by, order=1))
    # height_at == grids at nodes (spot check)
    rng = np.random.default_rng(0)
    t = tiles[(nt // 2, nt // 2)]
    jj, ii = rng.integers(0, t.ny, 500), rng.integers(0, t.nx, 500)
    lx0, ly0 = t.lattice_origin()
    st = t.lattice_step()
    px, py = from_lattice(lx0 + st * ii), from_lattice(ly0 + st * jj)
    res["height_at_vs_tile_nodes_max_abs_m"] = float(np.abs(height_at(cfg, px, py, "core").astype(np.float64)
                                                            - np.asarray(t.z[jj, ii], dtype=np.float64)).max())
    jj, ii = rng.integers(0, ring.ny, 500), rng.integers(0, ring.nx, 500)
    res["height_at_vs_ring_nodes_max_abs_m"] = float(np.abs(height_at(cfg, ring.xs()[ii], ring.ys()[jj], "ring")
                                                            .astype(np.float64) - np.asarray(ring.z[jj, ii])).max())
    return res


def _tile_values(cfg: Config, tiles: dict, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    T = lay.tile_size
    i = np.clip(np.floor((x - core.x0) / T).astype(int), 0, lay.n_tiles - 1)
    j = np.clip(np.floor((y - core.y0) / T).astype(int), 0, lay.n_tiles - 1)
    out = np.empty(x.shape)
    for (ti, tj) in set(zip(i.tolist(), j.tolist())):
        sel = (i == ti) & (j == tj)
        out[sel] = tiles[(ti, tj)].sample(x[sel], y[sel], order=1)
    return out


def _diffstats(a: np.ndarray, b: np.ndarray) -> dict:
    d = np.asarray(a, dtype=np.float64) - np.asarray(b, dtype=np.float64)
    return {"n": int(d.size), "max_abs_m": float(np.abs(d).max()), "rms_m": float(np.sqrt(np.mean(d * d))),
            "mean_m": float(d.mean())}


def crater_counts(cfg: Config, D_values=(0.2, 0.5, 1.0, 2.0, 5.0, 10.0)) -> dict:
    """N(>=D) of the sampled craters with centres in the core (both sets) and in the ring (big set)."""
    model = get_model(cfg)
    big, small = model.craters("big"), model.craters("small")
    lay = Layout(cfg)
    cc = cfg.terrain.craters
    C, b = float(cc.sfd_C_per_m2), float(cc.sfd_b)
    fade = tuple(float(v) for v in cc.fade_D_m)
    out = {}
    for name, rect in (("core", lay.zone_rect("core")), ("ring", lay.zone_rect("ring"))):
        area = rect.width * rect.height
        r = (rect.x0, rect.x1, rect.y0, rect.y1)
        n_big = big.count_ge(D_values, r)
        n_small = small.count_ge(D_values, r) if name == "core" else np.zeros(len(D_values), dtype=int)
        rows = []
        for k, D in enumerate(D_values):
            exp = tcraters.expected_count_ge(D, C, b, float(cc.D_min_m.ring), float(cc.D_max_m), fade, area)
            if name == "core":
                exp += tcraters.expected_count_ge(D, C, b, float(cc.D_min_m.core), float(cc.D_min_m.ring), (1e9, 2e9), area)
            n = int(n_big[k] + n_small[k])
            include = name == "core" or D >= float(cc.D_min_m.ring)
            rows.append({"D_m": D, "count": n if include else None, "expected": round(exp, 2) if include else None,
                         "power_law_C_D^-b": round(C * D ** -b * area, 2),
                         "ratio_to_expected": round(n / exp, 3) if include and exp > 0 else None})
        out[name] = {"area_m2": area, "rows": rows}
    out["n_big"], out["n_small"] = len(big), len(small)
    return out


# ---------------------------------------------------------------------------------------- build
ALL_STAGES = ("base", "fbm", "craters", "talus", "tiles", "zones", "checks")


def build_all(cfg: Config, rebuild: bool = False, stages=ALL_STAGES,
              tiles: list[TileId] | None = None, log=print) -> dict:
    """Build and cache all terrain products; writes build_dir/terrain/manifest.json."""
    lay = Layout(cfg)
    tdir = lay.stage_dir("terrain")
    model = get_model(cfg)
    mpath = tdir / "manifest.json"
    manifest = {"stage": "terrain", "terrain_version": TERRAIN_VERSION, "generator": "lunar_world_gen.terrain",
                "inputs": {"config": cfg.get("_source"), "config_hash": config_hash(cfg),
                           "section_hash": config_hash(cfg, ["world", "site", "zones", "dtm", "terrain", "paths"]),
                           "fbm_gain": model.gains["coarse"], "fbm_gain_fine": model.gains["fine"],
                           "talus": model.talus_params, "seed": int(cfg.world.seed),
                           "dtm": str(cfg.paths.dtm), "dtm_filled": str(cfg.paths.dtm_filled)},
                "component_keys": model.component_keys(), "talus_key": model.talus_key, "terrain_key": model.key,
                "stages": {}, "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    if mpath.exists():
        try:
            old = json.loads(mpath.read_text())
            if old.get("component_keys") == manifest["component_keys"]:
                keep = old.get("stages", {})
                if old.get("terrain_key") != model.key:      # gains, talus or fades changed: grids are stale
                    keep = {k: v for k, v in keep.items() if k == "base" or k.startswith("fbm") or k == "craters"
                            or (k == "talus" and old.get("talus_key") == model.talus_key)}
                elif "checks" in old:
                    manifest["checks"] = old["checks"]
                if "crater_counts" in old:
                    manifest["crater_counts"] = old["crater_counts"]
                manifest["stages"] = keep
        except (OSError, json.JSONDecodeError):
            pass

    def done(name: str, t0: float, **extra) -> None:
        manifest["stages"][name] = {"elapsed_s": round(time.time() - t0, 2), "peak_rss_after": _peak_rss_mb(),
                                    **extra}
        mpath.write_text(json.dumps(manifest, indent=1, default=str))
        log(f"[terrain] {name}: {manifest['stages'][name]['elapsed_s']} s, peak RSS {_peak_rss_mb()}")

    def rel(paths) -> list[str]:
        return [str(Path(p).relative_to(tdir)) for p in paths]

    if "base" in stages:
        t0 = time.time()
        info = tbase.build_all_bases(cfg, rebuild=rebuild)
        outs = [tbase.base_prefix(cfg, z).with_suffix(s) for z in tbase.BASE_ZONES for s in (".npy", ".json")]
        done("base", t0, zones=info, bytes=_du([tdir / "base"]), outputs=rel(outs))
    if "fbm" in stages:
        for band in tfbm.BANDS:
            t0 = time.time()
            g = tfbm.unit_field(cfg, band, rebuild=rebuild)
            pfx = tfbm.field_prefix(cfg, band)
            done(f"fbm_{band}", t0, n=[g.ny, g.nx], spacing_m=g.spacing, synth_elapsed_s=g.meta.get("elapsed_s"),
                 rms_unit_m=g.meta.get("rms_m"), n_fft=g.meta["params"]["n_fft"],
                 bytes=_du([pfx.with_suffix(".npy")]), outputs=rel([pfx.with_suffix(".npy"), pfx.with_suffix(".json")]))
    if "craters" in stages:
        t0 = time.time()
        for which in tcraters.WHICH:
            tcraters.get_craters(cfg, which, rebuild=rebuild)
        done("craters", t0, counts=crater_counts(cfg), bytes=_du([tdir / "craters"]),
             outputs=rel([tcraters.craters_path(cfg, w) for w in tcraters.WHICH]))
    if "talus" in stages:
        t0 = time.time()
        g = model.talus_field(rebuild=rebuild, log=log)
        pfx = ttalus.talus_prefix(cfg)
        tm = g.meta
        done("talus", t0, enabled=model.talus_params["enabled"], params=model.talus_params,
             n_windows=tm.get("n_windows", 0), n_changed_nodes=tm.get("n_changed_nodes", 0),
             max_abs_T_m=tm.get("max_abs_T_m", 0.0), n_steep_nodes=tm.get("n_steep_nodes", 0),
             compute_elapsed_s=tm.get("elapsed_s"),
             windows=[{k: w.get(k) for k in ("x_range_m", "y_range_m", "iterations", "converged",
                                              "max_slope_before_deg", "max_slope_after_deg", "max_abs_change_m",
                                              "mass_balance_m3", "frame_touched", "n_changed_nodes")}
                      for w in tm.get("windows", [])],
             bytes=_du([pfx.with_suffix(".npy")]), outputs=rel([pfx.with_suffix(".npy"), pfx.with_suffix(".json")]))
    if "tiles" in stages:
        t0 = time.time()
        todo = tiles or lay.tiles()
        zmin, zmax, built = np.inf, -np.inf, 0
        for t in todo:
            prefix = tile_prefix(cfg, t)
            if rebuild or not _cached(prefix, model.key):
                g = core_tile_height(cfg, t)
                g.meta.update(_zstats(g.z))
                _save(g, prefix)
                built += 1
            info = json.loads(prefix.with_suffix(".json").read_text())["meta"]
            zmin, zmax = min(zmin, info["z_min"]), max(zmax, info["z_max"])
        done("tiles", t0, n_tiles=len(todo), built=built, z_min_with_margins=zmin, z_max_with_margins=zmax,
             bytes=_du([tiles_dir(cfg)]), outputs=[f"core_tiles/tile_ii_jj.npy/.json x {len(todo)}"],
             grid=f"1 cm, tile + {float(cfg.zones.core.margin_m)} m margin, float32, row 0 south")
    if "zones" in stages:
        for zone in ("ring", "far", "horizon"):
            t0 = time.time()
            prefix = zone_height_prefix(cfg, zone)
            if rebuild and prefix.with_suffix(".json").exists():
                prefix.with_suffix(".json").unlink()
            g = load_zone_height(cfg, zone, mmap=True)
            done(f"zone_{zone}", t0, n=[g.ny, g.nx], spacing_m=g.spacing, z_min=g.meta.get("z_min"),
                 z_max=g.meta.get("z_max"), bytes=_du([prefix.with_suffix(".npy")]),
                 outputs=rel([prefix.with_suffix(".npy"), prefix.with_suffix(".json")]))
    if "checks" in stages:
        t0 = time.time()
        checks = consistency_checks(cfg)
        core = assemble_core(cfg)
        checks["core_H0_z_range_m"] = [float(core.z.min()), float(core.z.max())]
        del core
        done("checks", t0, results=checks)
        manifest["checks"] = checks
        manifest["crater_counts"] = crater_counts(cfg)
    manifest["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    manifest["peak_rss"] = _peak_rss_mb()
    manifest["disk_bytes"] = {"total": _du([tdir]), "base": _du([tdir / "base"]), "fbm": _du([tdir / "fbm"]),
                              "craters": _du([tdir / "craters"]), "talus": _du([tdir / "talus"]),
                              "core_tiles": _du([tiles_dir(cfg)]),
                              "zone_heights": _du([zone_height_prefix(cfg, z).with_suffix(".npy") for z in ("ring", "far", "horizon")])}
    mpath.write_text(json.dumps(manifest, indent=1, default=str))
    return manifest


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build the terrain stage (DTM base, fBm, craters, H0 grids).")
    ap.add_argument("--config", default=None, help="config YAML (default: config/apollo16_core.yaml)")
    ap.add_argument("--preset", default=None)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="config override")
    ap.add_argument("--all", action="store_true", help="run every stage")
    ap.add_argument("--stages", default=None, help=f"comma list of {','.join(ALL_STAGES)}")
    ap.add_argument("--rebuild", action="store_true", help="ignore caches")
    ap.add_argument("--tiles", default=None, help="only these tiles, e.g. 0_0,4_5")
    args = ap.parse_args(argv)
    cfg = load_config(args.config, preset=args.preset, overrides=args.set)
    if args.all:
        stages = ALL_STAGES
    elif args.stages:
        stages = tuple(s.strip() for s in args.stages.split(","))
    else:
        ap.error("give --all or --stages")
    tiles = None
    if args.tiles:
        tiles = [TileId(*map(int, s.split("_"))) for s in args.tiles.split(",")]
    t0 = time.time()
    man = build_all(cfg, rebuild=args.rebuild, stages=stages, tiles=tiles)
    print(json.dumps({"elapsed_s": round(time.time() - t0, 1), "peak_rss": man.get("peak_rss"),
                      "disk_bytes": man.get("disk_bytes"), "checks": man.get("checks")}, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
