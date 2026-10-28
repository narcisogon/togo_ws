"""World-planar texture coordinates (docs/DESIGN.md section 6).

``u = x / tile_m``, ``v = -y / tile_m`` with ``tile_m = cfg.mesh.uv_tile_m[zone]``. The same
position always gets the same UV, so UVs are continuous across pieces of one zone. With the
glTF / Ogre convention (v = 0 at the TOP row of the image, v growing downwards) the image then
lies on the ground unmirrored when seen from above: image right = east (+x), image up = north
(+y). ``materials.regolith`` builds its normal map for exactly this orientation.

UVs are computed in float64 and stored as float32: at |x| = 10 km and tile_m = 200 m the
float32 step is 4e-6 (0.01 texel of a 2048 px map), so precision is not a concern.
"""
from __future__ import annotations

import numpy as np

from .types import Mesh

__all__ = ["planar_uv", "assign_uvs", "zone_uv_tile_m"]


def planar_uv(x: np.ndarray, y: np.ndarray, tile_m: float) -> np.ndarray:
    """(n, 2) float32 world-planar UVs: u = x / tile_m, v = -y / tile_m."""
    if tile_m <= 0:
        raise ValueError("tile_m must be > 0")
    x = np.asarray(x, dtype=np.float64).ravel()
    y = np.asarray(y, dtype=np.float64).ravel()
    uv = np.empty((x.size, 2), dtype=np.float64)
    uv[:, 0] = x / tile_m
    uv[:, 1] = -y / tile_m
    return uv.astype(np.float32)


def assign_uvs(mesh: Mesh, tile_m: float) -> Mesh:
    """Set ``mesh.uvs`` to world-planar UVs (in place) and return the mesh."""
    mesh.uvs = planar_uv(mesh.vertices[:, 0], mesh.vertices[:, 1], tile_m)
    return mesh


def zone_uv_tile_m(cfg, zone: str) -> float:
    """UV repeat length of a zone (``cfg.mesh.uv_tile_m[zone]``)."""
    return float(cfg["mesh"]["uv_tile_m"][zone])
