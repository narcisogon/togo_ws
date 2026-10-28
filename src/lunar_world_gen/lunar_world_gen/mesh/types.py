"""Mesh and material containers shared by the terrain, rock, mesh and world modules.

Conventions (docs/DESIGN.md section 6):
* ``vertices`` are world ENU metres, Z-up, float64 while processing (written as float32). Vertices
  given as float32 stay float32 (merged rock meshes: they are written as float32 anyway, and
  float64 doubled their memory, CR-12); everything else is converted to float64.
* ``triangles`` are CCW when seen from +z for ground meshes (outward for closed rocks).
* ``lattice`` holds the integer 1 cm lattice coordinates (``grid.to_lattice``) of vertices
  that sit on height-grid nodes; vertices that do not (e.g. rock vertices) carry -1. Ground
  meshes must fill it: stitching compares boundary vertices through it, never via floats.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class MaterialSpec:
    """Material written into a GLB (factors only; textures come from the SDF PBR material)."""

    name: str
    base_color: tuple[float, float, float, float] = (0.12, 0.107, 0.088, 1.0)  # linear RGBA
    metallic: float = 0.0
    roughness: float = 0.95


@dataclass
class Mesh:
    vertices: np.ndarray
    triangles: np.ndarray
    normals: np.ndarray | None = None
    uvs: np.ndarray | None = None
    lattice: np.ndarray | None = None
    name: str = ""
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        v = np.asarray(self.vertices)
        self.vertices = (v if v.dtype == np.float32 else v.astype(np.float64, copy=False)).reshape(-1, 3)
        self.triangles = np.asarray(self.triangles, dtype=np.int64).reshape(-1, 3)
        if self.normals is not None:
            self.normals = np.asarray(self.normals, dtype=np.float32).reshape(-1, 3)
        if self.uvs is not None:
            self.uvs = np.asarray(self.uvs, dtype=np.float32).reshape(-1, 2)
        if self.lattice is not None:
            self.lattice = np.asarray(self.lattice, dtype=np.int64).reshape(-1, 2)

    @property
    def n_vertices(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def n_triangles(self) -> int:
        return int(self.triangles.shape[0])

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def face_normals(self) -> np.ndarray:
        v = self.vertices
        t = self.triangles
        n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
        length = np.linalg.norm(n, axis=1, keepdims=True)
        return n / np.maximum(length, 1e-30)

    def validate(self, require_up: bool = False) -> None:
        """Raise ValueError on structural problems (NaN, bad indices, attribute sizes)."""
        n = self.n_vertices
        if not np.all(np.isfinite(self.vertices)):
            raise ValueError(f"{self.name}: non-finite vertex coordinates")
        if self.n_triangles and (self.triangles.min() < 0 or self.triangles.max() >= n):
            raise ValueError(f"{self.name}: triangle index out of range")
        for attr, width in (("normals", 3), ("uvs", 2), ("lattice", 2)):
            arr = getattr(self, attr)
            if arr is not None and arr.shape != (n, width):
                raise ValueError(f"{self.name}: {attr} shape {arr.shape} != ({n}, {width})")
        if require_up and self.n_triangles:
            up = self.face_normals()[:, 2]
            bad = int(np.sum(up <= 0))
            if bad:
                raise ValueError(f"{self.name}: {bad} triangles face down (expected CCW from +z)")

    @staticmethod
    def concatenate(meshes: list["Mesh"], name: str = "") -> "Mesh":
        """Concatenate meshes; attributes are kept only if every input has them."""
        meshes = [m for m in meshes if m.n_vertices]
        if not meshes:
            return Mesh(np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), name=name)
        offsets = np.cumsum([0] + [m.n_vertices for m in meshes[:-1]])
        verts = np.vstack([m.vertices for m in meshes])
        tris = np.vstack([m.triangles + o for m, o in zip(meshes, offsets)])

        def cat(attr):
            arrs = [getattr(m, attr) for m in meshes]
            return None if any(a is None for a in arrs) else np.vstack(arrs)

        return Mesh(verts, tris, cat("normals"), cat("uvs"), cat("lattice"), name=name)
