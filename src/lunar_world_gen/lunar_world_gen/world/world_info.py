"""``world_info.yaml``: what a built world contains and where to spawn the robot.

Written next to the SDF files (``layout.world_info_path``) and read by
``launch/apollo16_sim.launch.py`` (``site:=flat``) and the test tools. The launch file parses
it with plain PyYAML, so the schema is kept flat and explicit (DESIGN section 11)::

    schema_version: 1
    world_name: apollo16_core
    generator_version: 0.1.0
    config_hash: <16 hex>            # config.config_hash(cfg)
    created_utc: 2026-10-25T12:00:00Z
    sdf_variants: {default: apollo16_core.sdf, inspect: ..._inspect.sdf, stock: ..._stock.sdf,
                   inspect_stock: ..._inspect_stock.sdf}
    physics: {engine, plugin, plugin_present, step_s: {patched, stock}, gravity_m_s2, ground_mu, friction_note}
    origin: {site_key, lat_deg, lon_deg, href_m, sphere_radius_m, surface_model, frame}
    gravity_m_s2: 1.62
    lighting: {sun_event, sun_elevation_deg, sun_azimuth_deg}
    zones: {core: {size_m, spacing_m, tile_size_m}, ring: {...}, far: {...}, horizon: {...}}
    robot: {footprint_half_m, footprint_radius_m, base_link_height_m, clearance_m}
    sites:
      flat: {x, y, z, yaw, yaw_deg, description, ground_z_max, ...}   # yaw in radians (ENU)
      slope: ...
      rock: ...
      turn: ...
    missing_sites: {<name>: <reason>}
    site_search: {...}
    triangle_counts: {...}            # as given by the integration stage
    rock_counts: {...}

``z`` of a site is the base_link spawn height (collision max under the footprint disc +
base_link height + clearance), directly usable as ``ros_gz_sim create -z``.
"""
from __future__ import annotations

import datetime as _dt
import math
import os
import uuid
from pathlib import Path
from typing import Any, Mapping

import yaml

from .. import __version__
from ..config import Config, config_hash
from ..layout import Layout
from .lighting import sun_light
from .sdf import VARIANTS, physics_settings
from .spawn import Site, SiteSearchResult, footprint_radius

SCHEMA_VERSION = 1
REQUIRED_SITE_KEYS = ("x", "y", "z", "yaw")


def _plain(o: Any) -> Any:
    """Convert numpy scalars/arrays and Paths to YAML-friendly Python types."""
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return [_plain(v) for v in o.tolist()]
    except ImportError:  # pragma: no cover
        pass
    if isinstance(o, Mapping):
        return {str(k): _plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_plain(v) for v in o]
    if isinstance(o, Path):
        return str(o)
    if isinstance(o, float) and not math.isfinite(o):
        return None
    return o


def build_world_info(cfg: Config, layout: Layout,
                     sites: SiteSearchResult | Mapping[str, Site | Mapping[str, Any]] | None = None, *,
                     triangle_counts: Mapping[str, Any] | None = None,
                     rock_counts: Mapping[str, Any] | None = None,
                     extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Assemble the world_info dictionary (see the module docstring for the schema)."""
    missing: dict[str, str] = {}
    search: dict[str, Any] = {}
    if isinstance(sites, SiteSearchResult):
        missing = dict(sites.missing)
        search = dict(sites.params)
        site_map: Mapping[str, Any] = sites.sites
    else:
        site_map = sites or {}
    site_dicts: dict[str, Any] = {}
    for name, s in site_map.items():
        d = s.to_dict() if isinstance(s, Site) else dict(s)
        for key in REQUIRED_SITE_KEYS:
            if key not in d:
                raise KeyError(f"site '{name}' lacks '{key}'")
        site_dicts[name] = d

    patched = physics_settings(cfg, "")
    stock = physics_settings(cfg, "stock")
    plugin = Path(str(cfg.paths.physics_plugin)).expanduser()
    sun = sun_light(cfg)
    zones = {}
    for zone in ("core", "ring", "far", "horizon"):
        z = cfg.zones[zone]
        zones[zone] = {k: z[k] for k in ("size_m", "spacing_m", "tile_size_m") if k in z}
    info: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "world_name": layout.world_name,
        "generator_version": __version__,
        "config_hash": config_hash(cfg),
        "config_file": cfg.get("_source", {}).get("config_file"),
        "preset": cfg.world.get("preset"),
        "created_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "sdf_variants": {(v or "default"): layout.sdf_path(v).name for v in VARIANTS},
        "physics": {
            "engine": patched.engine,
            "engine_filename": patched.engine_filename,
            "plugin": str(plugin),
            "plugin_present": plugin.is_file(),
            "step_s": {"patched": float(cfg.physics.step_s.patched), "stock": stock.step_s},
            "gravity_m_s2": float(cfg.physics.gravity_m_s2),
            "ground_mu": float(cfg.physics.ground_mu),
            # CR-09: rock collision proxies are baked into the ground collision meshes (DESIGN 7), and
            # Bullet stores friction per body, so ground_mu applies to rocks too (physics.rock_mu unused)
            "friction_note": "rocks are part of the ground collision meshes: ground_mu applies to them",
        },
        "origin": {
            "site_key": cfg.site.get("key"),
            "lat_deg": float(cfg.site.lat_deg),
            "lon_deg": float(cfg.site.lon_deg),
            "href_m": float(cfg.site.href_m),
            "sphere_radius_m": float(cfg.site.sphere_radius_m),
            "surface_model": cfg.spherical_coordinates.get("surface_model", "CUSTOM_SURFACE"),
            "frame": "local ENU tangent plane at the core centre (x east, y north, z up); z = 0 at href",
        },
        "gravity_m_s2": float(cfg.physics.gravity_m_s2),
        "lighting": {"sun_event": cfg.lighting.sun_event, "sun_elevation_deg": sun.elevation_deg,
                     "sun_azimuth_deg": sun.azimuth_deg},
        "zones": zones,
        "robot": {
            "footprint_half_m": [float(v) for v in cfg.spawn.footprint_half_m],
            "footprint_radius_m": round(footprint_radius(cfg), 4),
            "base_link_height_m": float(cfg.spawn.base_link_height_m),
            "clearance_m": float(cfg.spawn.clearance_m),
            "spawn_rule": "z = max(collision height over the footprint disc) + base_link_height_m + clearance_m",
        },
        "sites": site_dicts,
        "missing_sites": missing,
        "site_search": search,
        "triangle_counts": dict(triangle_counts or {}),
        "rock_counts": dict(rock_counts or {}),
    }
    if extra:
        info.update(dict(extra))
    return _plain(info)


def write_world_info(cfg: Config, layout: Layout,
                     sites: SiteSearchResult | Mapping[str, Site | Mapping[str, Any]] | None = None, *,
                     path: str | Path | None = None, **kwargs: Any) -> Path:
    """Write world_info.yaml (default ``layout.world_info_path``) and return its path."""
    info = build_world_info(cfg, layout, sites, **kwargs)
    path = Path(path) if path is not None else layout.world_info_path
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.tmp{os.getpid()}_{uuid.uuid4().hex[:8]}")   # unique (CR-06)
    try:
        tmp.write_text("# generated by lunar_world_gen world/world_info.py - do not edit by hand\n"
                       + yaml.safe_dump(info, sort_keys=False, default_flow_style=False, width=110))
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)
    return path


def read_world_info(path: str | Path) -> dict[str, Any]:
    """Load and sanity-check a world_info.yaml."""
    info = yaml.safe_load(Path(path).read_text())
    if not isinstance(info, dict):
        raise ValueError(f"{path}: not a mapping")
    ver = info.get("schema_version")
    if ver != SCHEMA_VERSION:
        raise ValueError(f"{path}: schema_version {ver!r} != {SCHEMA_VERSION}")
    for name, site in (info.get("sites") or {}).items():
        for key in REQUIRED_SITE_KEYS:
            if key not in site or not isinstance(site[key], (int, float)):
                raise ValueError(f"{path}: site '{name}' has no numeric '{key}'")
    return info


def site_pose(info: Mapping[str, Any], name: str) -> tuple[float, float, float, float]:
    """(x, y, z, yaw) of a site; KeyError lists the available sites."""
    sites = info.get("sites") or {}
    if name not in sites:
        missing = (info.get("missing_sites") or {}).get(name)
        why = f" (not found by the generator: {missing})" if missing else ""
        raise KeyError(f"site '{name}' not in world_info{why}; available: {sorted(sites)}")
    s = sites[name]
    return float(s["x"]), float(s["y"]), float(s["z"]), float(s["yaw"])
