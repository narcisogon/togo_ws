"""terrain package of the lunar world generator (docs/DESIGN.md sections 2-4 and the terrain part of 13).

Modules
    base       band-limited DTM base field B per zone (tools/dtm_extract.py, cached; textured nodata fill)
    fbm        synthetic fBm roughness bands F_coarse / F_fine (cached at unit gain; one gain per band)
    craters    crater population (CraterSet: crest-exact rims, shape variation) and crater relief K
               (numba, bucket index)
    talus      angle-of-repose limiter: the talus correction T (cached 10 cm field, mass-conserving)
    compose    the height function H0 = B + w_ring (F_coarse + K_big + T) + w_core (F_fine + K_small),
               zone grids, height_at, caches and the build CLI
    calibrate  plane-detrended directional deviogram and the two-gain site-aware calibration

Main entry points (also reachable as attributes of this package, imported lazily)::

    from lunar_world_gen.terrain import compose
    compose.core_tile_height(cfg, TileId(i, j))      # 1 cm H0 of a core tile (+ margin)
    compose.load_core_tile(cfg, TileId(i, j))        # the same from build_dir/terrain/core_tiles
    compose.ring_height(cfg); compose.far_height(cfg); compose.horizon_height(cfg)
    compose.height_at(cfg, x, y, zone)               # equals the grids at their nodes

CLI: ``python -m lunar_world_gen.terrain.compose --all`` and
``python -m lunar_world_gen.terrain.calibrate [--apply]``.
"""
from __future__ import annotations

import importlib

_API = {
    "base_grid": "base", "extract_base": "base",
    "coarse_field": "fbm", "fine_field": "fbm", "CubicSampler": "fbm",
    "CraterSet": "craters", "sample_craters": "craters", "get_craters": "craters",
    "crater_relief": "craters", "crater_relief_grid": "craters",
    "core_tile_height": "compose", "core_height": "compose", "ring_height": "compose", "far_height": "compose",
    "horizon_height": "compose", "height_at": "compose", "w_core": "compose", "w_ring": "compose",
    "load_core_tile": "compose", "load_zone_height": "compose", "assemble_core": "compose",
    "TerrainModel": "compose", "get_model": "compose", "build_all": "compose",
    "deviogram": "calibrate",
}

__all__ = sorted(_API)


def __getattr__(name: str):
    if name in _API:
        return getattr(importlib.import_module(f".{_API[name]}", __name__), name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
