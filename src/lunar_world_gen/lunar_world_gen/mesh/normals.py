"""Ground vertex normals from the height-grid gradient (docs/DESIGN.md section 6).

The visual ground is a TIN decimated to a few millimetres (core) up to metres (horizon), so
normals averaged from its triangles would show the facets and lose the fine relief. Instead
every vertex takes the normal of the full-resolution height field:

* the gradient is computed with central differences on the full grid (``np.gradient``:
  second-order central differences inside, first-order one-sided at the outermost nodes - pass
  a grid with a margin around the piece so its boundary vertices get true central differences),
* then lightly smoothed with a Gaussian of ``sigma`` nodes (``mode='nearest'``).

Adaptive smoothing (default): a vertex of a decimated TIN represents the surface over its
neighbourhood, so point-sampling a 1 cm gradient at vertices 5 cm apart aliases fine slopes into
blotchy shading. ``vertex_normals`` therefore picks, per vertex, ``sigma = 0.375 * L / spacing``
nodes (L = mean length of the vertex's incident edges; a Gaussian of that sigma halves the
amplitude at wavelength 2 L, the vertex Nyquist), clipped to the precomputed levels
``(1, 2, 4, 8)`` nodes and blended linearly in log2(sigma). With ``adaptive=False`` all vertices
use ``sigma_nodes`` (the plain "central differences + light smoothing" rule).

Shared boundary vertices of adjacent pieces get identical normals in ``mesh.stitch`` by copying
the canonical piece's values; ``GridNormalField`` can be passed as ``Piece.normal_fn``.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np
from scipy import ndimage

from ..grid import Grid
from .types import Mesh

__all__ = [
    "DEFAULT_LEVELS",
    "GridNormalField",
    "gradient_to_normals",
    "vertex_spacing",
    "vertex_normals",
    "assign_normals",
    "triangle_normals",
]

DEFAULT_LEVELS: tuple[float, ...] = (1.0, 2.0, 4.0, 8.0)  # Gaussian sigmas in nodes
ADAPTIVE_ALPHA = 0.375  # sigma = alpha * local vertex spacing (half amplitude at wavelength 2 L)


def gradient_to_normals(gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    """Unit normals (-gx, -gy, 1) / norm as float32 (n, 3)."""
    gx = np.asarray(gx, dtype=np.float64).ravel()
    gy = np.asarray(gy, dtype=np.float64).ravel()
    inv = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    n = np.empty((gx.size, 3), dtype=np.float64)
    n[:, 0] = -gx * inv
    n[:, 1] = -gy * inv
    n[:, 2] = inv
    return n.astype(np.float32)


class GridNormalField:
    """Smoothed central-difference gradient of a height grid at one or more smoothing levels.

    Callable as ``field(x, y)`` -> (n, 3) float32 unit normals (finest level), which is the
    ``normal_fn`` signature used by ``mesh.stitch.Piece``.
    """

    def __init__(self, grid: Grid, levels: Sequence[float] = DEFAULT_LEVELS):
        if len(levels) == 0:
            raise ValueError("need at least one smoothing level")
        self.grid = grid
        self.levels = np.asarray(sorted(float(s) for s in levels), dtype=np.float64)
        z = np.asarray(grid.z, dtype=np.float64)
        if min(z.shape) < 2:
            raise ValueError("grid must have at least 2 x 2 nodes")
        gy, gx = np.gradient(z, grid.spacing)  # axis 0 = rows (y), axis 1 = columns (x)
        self._gx: list[np.ndarray] = []
        self._gy: list[np.ndarray] = []
        for s in self.levels:
            if s > 0:
                gxs = ndimage.gaussian_filter(gx, s, mode="nearest", truncate=4.0)
                gys = ndimage.gaussian_filter(gy, s, mode="nearest", truncate=4.0)
            else:
                gxs, gys = gx, gy
            self._gx.append(gxs.astype(np.float32))
            self._gy.append(gys.astype(np.float32))

    # ------------------------------------------------------------------ sampling
    def _sample_level(self, k: int, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        g = self.grid
        col = np.clip((np.asarray(x, dtype=np.float64) - g.x0) / g.spacing, 0.0, g.nx - 1)
        row = np.clip((np.asarray(y, dtype=np.float64) - g.y0) / g.spacing, 0.0, g.ny - 1)
        c0 = np.minimum(np.floor(col).astype(np.int64), g.nx - 2)
        r0 = np.minimum(np.floor(row).astype(np.int64), g.ny - 2)
        fc = col - c0
        fr = row - r0
        out = []
        for a in (self._gx[k], self._gy[k]):
            v00 = a[r0, c0].astype(np.float64)
            v01 = a[r0, c0 + 1]
            v10 = a[r0 + 1, c0]
            v11 = a[r0 + 1, c0 + 1]
            out.append(v00 * (1 - fr) * (1 - fc) + v01 * (1 - fr) * fc + v10 * fr * (1 - fc) + v11 * fr * fc)
        return out[0], out[1]

    def gradient(self, x: np.ndarray, y: np.ndarray, sigma_nodes: np.ndarray | float | None = None
                 ) -> tuple[np.ndarray, np.ndarray]:
        """(dz/dx, dz/dy) at world positions (bilinear; exact at nodes).

        ``sigma_nodes`` per point (or scalar) selects the smoothing: values between levels are
        blended linearly in log2(sigma); values outside the level range are clipped. ``None`` =
        the finest level.
        """
        x = np.asarray(x, dtype=np.float64).ravel()
        y = np.asarray(y, dtype=np.float64).ravel()
        if sigma_nodes is None or self.levels.size == 1:
            return self._sample_level(0, x, y)
        s = np.broadcast_to(np.asarray(sigma_nodes, dtype=np.float64), x.shape)
        lv = np.log2(np.maximum(self.levels, 1e-6))
        f = np.interp(np.log2(np.maximum(s, 1e-6)), lv, np.arange(lv.size, dtype=np.float64))
        k0 = np.clip(np.floor(f).astype(np.int64), 0, lv.size - 1)
        w = f - k0
        gx = np.zeros_like(x)
        gy = np.zeros_like(x)
        for k in range(lv.size):
            w_k = np.where(k0 == k, 1.0 - w, 0.0) + np.where(k0 + 1 == k, w, 0.0)
            sel = w_k > 0
            if not np.any(sel):
                continue
            ax, ay = self._sample_level(k, x[sel], y[sel])
            gx[sel] += w_k[sel] * ax
            gy[sel] += w_k[sel] * ay
        return gx, gy

    def normals(self, x: np.ndarray, y: np.ndarray, sigma_nodes: np.ndarray | float | None = None) -> np.ndarray:
        gx, gy = self.gradient(x, y, sigma_nodes)
        return gradient_to_normals(gx, gy)

    __call__ = normals


def vertex_spacing(mesh: Mesh) -> np.ndarray:
    """Mean horizontal length of the edges incident to each vertex (metres; 0 if isolated)."""
    t = mesh.triangles
    v = mesh.vertices
    a = np.concatenate([t[:, 0], t[:, 1], t[:, 2]])
    b = np.concatenate([t[:, 1], t[:, 2], t[:, 0]])
    length = np.hypot(v[a, 0] - v[b, 0], v[a, 1] - v[b, 1])
    n = mesh.n_vertices
    # every interior edge appears twice (once per triangle): counting it from both ends of
    # both triangles weights all edges equally up to the boundary, which is fine for a scale
    s = np.bincount(a, length, minlength=n) + np.bincount(b, length, minlength=n)
    c = np.bincount(a, minlength=n) + np.bincount(b, minlength=n)
    return np.where(c > 0, s / np.maximum(c, 1), 0.0)


def vertex_normals(mesh: Mesh, field: GridNormalField | Grid, *, adaptive: bool = True,
                   sigma_nodes: float = 1.0, alpha: float = ADAPTIVE_ALPHA) -> np.ndarray:
    """Per-vertex unit normals (float32) from the grid gradient, see the module docstring."""
    if isinstance(field, Grid):
        field = GridNormalField(field, DEFAULT_LEVELS if adaptive else (sigma_nodes,))
    x = mesh.vertices[:, 0]
    y = mesh.vertices[:, 1]
    if adaptive:
        sig = alpha * vertex_spacing(mesh) / field.grid.spacing
        sig = np.maximum(sig, field.levels[0])
        return field.normals(x, y, sig)
    if field.levels.size == 1 or np.isclose(field.levels, sigma_nodes).any():
        return field.normals(x, y, sigma_nodes)
    return field.normals(x, y, np.full(x.shape, sigma_nodes))


def assign_normals(mesh: Mesh, field: GridNormalField | Grid, **kwargs) -> Mesh:
    """Set ``mesh.normals`` from the grid gradient (in place) and return the mesh."""
    mesh.normals = vertex_normals(mesh, field, **kwargs)
    return mesh


def triangle_normals(mesh: Mesh) -> np.ndarray:
    """Area-weighted vertex normals from the triangles (fallback / comparison), float32."""
    v = mesh.vertices
    t = mesh.triangles
    fn = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])  # length = 2 * area
    n = np.zeros((mesh.n_vertices, 3), dtype=np.float64)
    for k in range(3):
        np.add.at(n, t[:, k], fn)
    length = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(length > 0, n / np.maximum(length, 1e-300), np.array([0.0, 0.0, 1.0]))
    return n.astype(np.float32)
