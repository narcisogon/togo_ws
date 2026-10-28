"""world package of the lunar world generator (see docs/DESIGN.md sections 8, 11, 12).

* :mod:`.sdf`        - world SDF writer (``write_world``, ``VisualSpec``, ``CollisionSpec``)
* :mod:`.lighting`   - sun / earthshine / fill lights from ``cfg.lighting``
* :mod:`.spawn`      - spawn height and the flat / slope / rock / turn test sites
* :mod:`.world_info` - ``world_info.yaml`` writer and reader

The launch side lives outside the package: ``launch/apollo16_sim.launch.py`` and
``scripts/run_apollo16_sim.sh``; the patched physics plugin in ``tools/gz_physics_bullet_lunar/``.
"""
from .lighting import LightingSetup, LightSpec, direction_from_el_az, lighting_setup, sun_light
from .sdf import (CollisionSpec, MaterialDef, VisualSpec, default_materials, find_missing,
                  physics_settings, read_engine_filename, validate_sdf, write_all_variants, write_world)
from .spawn import RockTable, Site, SiteSearchResult, find_sites, footprint_radius, spawn_z
from .world_info import build_world_info, read_world_info, site_pose, write_world_info

__all__ = [
    "LightingSetup", "LightSpec", "direction_from_el_az", "lighting_setup", "sun_light",
    "CollisionSpec", "MaterialDef", "VisualSpec", "default_materials", "find_missing",
    "physics_settings", "read_engine_filename", "validate_sdf", "write_all_variants", "write_world",
    "RockTable", "Site", "SiteSearchResult", "find_sites", "footprint_radius", "spawn_z",
    "build_world_info", "read_world_info", "site_pose", "write_world_info",
]
