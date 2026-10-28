"""Regular height grids in the local east-north-up (ENU) world frame.

Every height field in the generator is a ``Grid``: a node-centred regular grid with
both edges included, row 0 at the SOUTH edge (the same layout tools/dtm_extract.py
writes with its default ``--row0 south``)::

    z[j, i] is the height at x = x0 + i * spacing, y = y0 + j * spacing

All zone grids sit on one global integer lattice of ``LATTICE_M`` (1 cm), so node
positions of different zones can be compared exactly through integer lattice
coordinates (see ``lattice_index``). This is what lets meshes of different zones share
boundary vertices without floating-point mismatches.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage

LATTICE_M = 0.01  # global lattice; every zone spacing and origin is an integer multiple of it


def to_lattice(value_m: float | np.ndarray) -> np.ndarray:
    """Convert metres to integer lattice units (raises if not on the lattice)."""
    q = np.asarray(value_m, dtype=np.float64) / LATTICE_M
    r = np.rint(q)
    if np.any(np.abs(q - r) > 1e-6):
        raise ValueError(f"value {value_m!r} is not a multiple of the {LATTICE_M} m lattice")
    return r.astype(np.int64)


def from_lattice(index: int | np.ndarray) -> np.ndarray:
    """Convert integer lattice units back to metres."""
    return np.asarray(index, dtype=np.float64) * LATTICE_M


@dataclass
class Grid:
    """Node-centred height grid; z[j, i] at (x0 + i*spacing, y0 + j*spacing), row 0 = south."""

    z: np.ndarray
    x0: float
    y0: float
    spacing: float
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.z.ndim != 2:
            raise ValueError("Grid.z must be 2-D (ny, nx)")
        self.x0 = float(self.x0)
        self.y0 = float(self.y0)
        self.spacing = float(self.spacing)

    # ------------------------------------------------------------------ geometry
    @property
    def ny(self) -> int:
        return int(self.z.shape[0])

    @property
    def nx(self) -> int:
        return int(self.z.shape[1])

    @property
    def x1(self) -> float:
        return self.x0 + (self.nx - 1) * self.spacing

    @property
    def y1(self) -> float:
        return self.y0 + (self.ny - 1) * self.spacing

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(x_min, x_max, y_min, y_max) of the node positions."""
        return self.x0, self.x1, self.y0, self.y1

    def xs(self) -> np.ndarray:
        return self.x0 + np.arange(self.nx) * self.spacing

    def ys(self) -> np.ndarray:
        return self.y0 + np.arange(self.ny) * self.spacing

    def mesh_xy(self) -> tuple[np.ndarray, np.ndarray]:
        """(X, Y) arrays shaped like z."""
        return np.meshgrid(self.xs(), self.ys())

    def lattice_origin(self) -> tuple[int, int]:
        """Integer lattice coordinates of node (0, 0)."""
        return int(to_lattice(self.x0)), int(to_lattice(self.y0))

    def lattice_step(self) -> int:
        """Node spacing in lattice units."""
        return int(to_lattice(self.spacing))

    def contains(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x = np.asarray(x)
        y = np.asarray(y)
        eps = 1e-9
        return (x >= self.x0 - eps) & (x <= self.x1 + eps) & (y >= self.y0 - eps) & (y <= self.y1 + eps)

    # ------------------------------------------------------------------ sampling
    def fractional_index(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Fractional (col, row) indices of world positions."""
        return (np.asarray(x, dtype=np.float64) - self.x0) / self.spacing, (
            np.asarray(y, dtype=np.float64) - self.y0
        ) / self.spacing

    def sample(self, x: np.ndarray, y: np.ndarray, order: int = 1) -> np.ndarray:
        """Interpolate heights at world positions.

        order=1 is bilinear (exact at nodes, matches the linear triangles of a full-resolution
        mesh along grid edges); order=3 is a cubic spline (C2, for smooth coarse fields). Points
        outside the grid are clamped to the nearest edge value.
        """
        col, row = self.fractional_index(x, y)
        col = np.clip(col, 0.0, self.nx - 1)
        row = np.clip(row, 0.0, self.ny - 1)
        shape = np.shape(col)
        coords = np.vstack([np.ravel(row), np.ravel(col)])
        if order == 1:
            out = _bilinear(self.z, coords[0], coords[1])
        else:
            out = ndimage.map_coordinates(
                np.asarray(self.z, dtype=np.float64), coords, order=order, mode="nearest", prefilter=True
            )
        return out.reshape(shape)

    def crop(self, x_min: float, x_max: float, y_min: float, y_max: float) -> "Grid":
        """Node-aligned sub-grid covering [x_min, x_max] x [y_min, y_max] (inclusive nodes)."""
        i0 = int(np.ceil((x_min - self.x0) / self.spacing - 1e-9))
        i1 = int(np.floor((x_max - self.x0) / self.spacing + 1e-9))
        j0 = int(np.ceil((y_min - self.y0) / self.spacing - 1e-9))
        j1 = int(np.floor((y_max - self.y0) / self.spacing + 1e-9))
        i0, j0 = max(i0, 0), max(j0, 0)
        i1, j1 = min(i1, self.nx - 1), min(j1, self.ny - 1)
        if i1 < i0 or j1 < j0:
            raise ValueError("crop window does not overlap the grid")
        return Grid(
            self.z[j0 : j1 + 1, i0 : i1 + 1].copy(),
            self.x0 + i0 * self.spacing,
            self.y0 + j0 * self.spacing,
            self.spacing,
            dict(self.meta),
        )

    # ------------------------------------------------------------------ io
    def save(self, prefix: str | Path) -> None:
        """Write PREFIX.npy (heights) and PREFIX.json (frame + meta)."""
        prefix = Path(prefix)
        prefix.parent.mkdir(parents=True, exist_ok=True)
        np.save(prefix.with_suffix(".npy"), self.z)
        info = {"x0": self.x0, "y0": self.y0, "spacing": self.spacing, "nx": self.nx, "ny": self.ny,
                "row0": "south", "meta": self.meta}
        prefix.with_suffix(".json").write_text(json.dumps(info, indent=2, default=_json_default))

    @classmethod
    def load(cls, prefix: str | Path, mmap: bool = False) -> "Grid":
        prefix = Path(prefix)
        info = json.loads(prefix.with_suffix(".json").read_text())
        z = np.load(prefix.with_suffix(".npy"), mmap_mode="r" if mmap else None)
        return cls(z, info["x0"], info["y0"], info["spacing"], info.get("meta", {}))

    @classmethod
    def from_dtm_extract(cls, prefix: str | Path, mmap: bool = False) -> "Grid":
        """Load a tools/dtm_extract.py output (PREFIX.npy + PREFIX.json, row0 south)."""
        prefix = Path(prefix)
        meta = json.loads(prefix.with_suffix(".json").read_text())
        z = np.load(prefix.with_suffix(".npy"), mmap_mode="r" if mmap else None)
        grid = meta.get("grid", {})
        if grid.get("row0", "south") != "south":
            raise ValueError("expected a dtm_extract grid written with --row0 south")
        x0 = grid.get("x_min", grid.get("x0"))
        y0 = grid.get("y_min", grid.get("y0"))
        spacing = grid.get("spacing", grid.get("spacing_m"))
        if x0 is None or y0 is None or spacing is None:
            raise KeyError(f"cannot find grid origin/spacing in {prefix.with_suffix('.json')}: keys {sorted(grid)}")
        return cls(z, x0, y0, spacing, {"dtm_extract": meta})


def _bilinear(z: np.ndarray, row: np.ndarray, col: np.ndarray) -> np.ndarray:
    ny, nx = z.shape
    r0 = np.clip(np.floor(row).astype(np.int64), 0, max(ny - 2, 0))
    c0 = np.clip(np.floor(col).astype(np.int64), 0, max(nx - 2, 0))
    fr = row - r0
    fc = col - c0
    r1 = np.minimum(r0 + 1, ny - 1)
    c1 = np.minimum(c0 + 1, nx - 1)
    z00 = z[r0, c0].astype(np.float64)
    z01 = z[r0, c1].astype(np.float64)
    z10 = z[r1, c0].astype(np.float64)
    z11 = z[r1, c1].astype(np.float64)
    return (z00 * (1 - fr) * (1 - fc) + z01 * (1 - fr) * fc + z10 * fr * (1 - fc) + z11 * fr * fc)


def _json_default(o):
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")
