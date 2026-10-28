"""TIN (triangulated irregular network) meshing of height grids with pydelatin.

docs/DESIGN.md section 6: every ground piece (core tile, ring / far / horizon strip, collision
field) is one ``Grid`` meshed with pydelatin at a vertical tolerance ``max_error``. All TIN
vertices sit on grid nodes and carry the node height, the four rectangle corners are always
vertices, and ``Mesh.lattice`` holds the integer 1 cm lattice coordinates of every vertex so
that ``mesh.stitch`` can compare boundary vertices exactly.

pydelatin conventions (pydelatin 0.4 = Fogleman's ``hmm``; verified on an asymmetric probe,
see ``tests/test_world_mesh.py::test_pydelatin_convention_probe``):

* the input ``arr[r, c]`` (r = image row, c = column) is flattened in C order and converted to
  float32; ``max_error`` is in the units of ``arr``;
* an output vertex is ``(c, (H - 1) - r, arr[r, c])`` in float32, i.e. the row axis is
  MIRRORED, and the output triangles are counter-clockwise in that mirrored (x, y) frame;
* the triangulation starts from the 4 corners (always present) and refines the triangle with
  the largest error until every node of every closed triangle is within ``max_error`` of the
  linear interpolant (rasterised in integer node space, float32 arithmetic).

Our grids have row 0 at the SOUTH edge (``Grid.z[j, i]`` at ``y0 + j * s``), so the
row-reversed array is fed in: the output (x, y) is then exactly the node index (i, j) and the
triangles are CCW seen from +z in the world frame. Heights are taken back from the original
grid in float64 (pydelatin's float32 copy is only used for the error test), with a reference
height subtracted first so the float32 copy keeps sub-micrometre precision.
"""
from __future__ import annotations

import time

import numpy as np

from ..grid import Grid, from_lattice
from .types import Mesh

try:  # numba is in the project venv; the pure-numpy fallback is only for exotic environments
    import numba

    _njit = numba.njit(cache=False, nogil=True)
except Exception:  # pragma: no cover - fallback path
    numba = None

    def _njit(f):  # type: ignore[no-redef]
        return f


__all__ = ["tin_from_grid", "tin_surface_on_grid", "tin_max_error", "grid_node_index"]


def tin_from_grid(grid: Grid, max_error: float, name: str = "", *, max_triangles: int | None = None,
                  check: bool = True) -> Mesh:
    """Triangulate a height grid into a TIN whose vertices are grid nodes.

    Args:
        grid: node-centred height grid on the global lattice (``grid.to_lattice`` must hold for
            its origin and spacing). The whole grid is meshed: pass a tile's own node rectangle
            including both boundary node lines (e.g. 1001 x 1001 nodes for a 10 m tile at 1 cm).
        max_error: vertical tolerance in metres: every grid node lies within ``max_error`` of
            the TIN surface (up to float32 rounding, ~1e-6 m for |z| < 10 m).
        name: mesh name (also used in error messages).
        max_triangles: optional cap passed to pydelatin (the error bound then no longer holds).
        check: verify the output (integer node coordinates, unique vertices, corners present,
            every triangle CCW with positive area). Costs a few ms per million triangles.

    Returns:
        ``Mesh`` with float64 ``vertices`` (x, y from the lattice, z = the node height),
        int64 ``triangles`` (CCW seen from +z), ``lattice`` (int64 1 cm lattice coordinates) and
        ``meta`` = {max_error, tin_error, grid_shape, spacing_m, n_nodes, time_s}. Normals and UVs
        are not set (see ``mesh.normals`` / ``mesh.uv``).
    """
    from pydelatin import Delatin  # local import keeps module import cheap

    t0 = time.perf_counter()
    z = np.asarray(grid.z)
    if z.ndim != 2 or min(z.shape) < 2:
        raise ValueError(f"{name}: grid must be 2-D with at least 2 x 2 nodes, got {z.shape}")
    if not np.all(np.isfinite(z)):
        raise ValueError(f"{name}: grid contains non-finite heights")
    if max_error <= 0:
        raise ValueError(f"{name}: max_error must be > 0")
    ny, nx = z.shape
    ox, oy = grid.lattice_origin()
    step = grid.lattice_step()
    if step <= 0:
        raise ValueError(f"{name}: grid spacing must be positive")

    # float32 copy for pydelatin, centred so the float32 rounding stays tiny; rows reversed so
    # pydelatin's mirrored output y equals our south-origin row index j
    zmin = float(np.min(z))
    zmax = float(np.max(z))
    zref = 0.5 * (zmin + zmax)
    data = np.ascontiguousarray((z[::-1] - zref).astype(np.float32))
    kwargs = {"max_error": float(max_error)}
    if max_triangles:
        kwargs["max_triangles"] = int(max_triangles)
    tin = Delatin(data, **kwargs)
    pts = np.asarray(tin.vertices)
    tris = np.asarray(tin.triangles).astype(np.int64)
    tin_error = float(tin.error)
    del data

    ii = np.rint(pts[:, 0]).astype(np.int64)
    jj = np.rint(pts[:, 1]).astype(np.int64)
    if check:
        if np.any(np.abs(pts[:, 0] - ii) > 1e-3) or np.any(np.abs(pts[:, 1] - jj) > 1e-3):
            raise RuntimeError(f"{name}: pydelatin returned non-integer node coordinates")
        if ii.min() < 0 or jj.min() < 0 or ii.max() > nx - 1 or jj.max() > ny - 1:
            raise RuntimeError(f"{name}: pydelatin vertex outside the grid")

    lattice = np.empty((ii.size, 2), dtype=np.int64)
    lattice[:, 0] = ox + ii * step
    lattice[:, 1] = oy + jj * step
    vertices = np.empty((ii.size, 3), dtype=np.float64)
    vertices[:, :2] = from_lattice(lattice)
    vertices[:, 2] = z[jj, ii].astype(np.float64)

    mesh = Mesh(vertices, tris, lattice=lattice, name=name)
    if check:
        _check_tin(mesh, ii, jj, nx, ny, name)
    mesh.meta.update({
        "max_error": float(max_error),
        "tin_error": tin_error,
        "grid_shape": [int(ny), int(nx)],
        "spacing_m": float(grid.spacing),
        "n_nodes": int(nx * ny),
        "time_s": round(time.perf_counter() - t0, 4),
    })
    return mesh


def _check_tin(mesh: Mesh, ii: np.ndarray, jj: np.ndarray, nx: int, ny: int, name: str) -> None:
    key = jj * nx + ii
    if np.unique(key).size != key.size:
        raise RuntimeError(f"{name}: duplicate TIN vertices")
    corners = np.array([0, nx - 1, (ny - 1) * nx, (ny - 1) * nx + nx - 1], dtype=np.int64)
    if not np.isin(corners, key).all():
        raise RuntimeError(f"{name}: a grid corner is missing from the TIN")
    t = mesh.triangles
    if t.size == 0:
        raise RuntimeError(f"{name}: empty TIN")
    # exact integer signed areas in node-index space
    xi = ii[t]
    yj = jj[t]
    area2 = (xi[:, 1] - xi[:, 0]) * (yj[:, 2] - yj[:, 0]) - (yj[:, 1] - yj[:, 0]) * (xi[:, 2] - xi[:, 0])
    if np.any(area2 <= 0):
        raise RuntimeError(f"{name}: {int(np.sum(area2 <= 0))} TIN triangles are not CCW / degenerate")
    total = int(area2.sum())
    if total != 2 * (nx - 1) * (ny - 1):
        raise RuntimeError(f"{name}: TIN does not tile the grid rectangle (2*area {total} != "
                           f"{2 * (nx - 1) * (ny - 1)})")


def grid_node_index(grid: Grid, lattice: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(i, j, on_grid) node indices of lattice points in ``grid`` (on_grid False if not a node)."""
    lattice = np.asarray(lattice, dtype=np.int64).reshape(-1, 2)
    ox, oy = grid.lattice_origin()
    step = grid.lattice_step()
    dx = lattice[:, 0] - ox
    dy = lattice[:, 1] - oy
    i, ri = np.divmod(dx, step)
    j, rj = np.divmod(dy, step)
    on = (ri == 0) & (rj == 0) & (i >= 0) & (j >= 0) & (i < grid.nx) & (j < grid.ny)
    return i, j, on


# ---------------------------------------------------------------------------------------------
# verification: evaluate the piecewise-linear TIN surface at every grid node
# ---------------------------------------------------------------------------------------------
@_njit
def _raster_tin(lx, ly, z, tris, ox, oy, step, nx, ny, out):  # pragma: no cover - numba
    """Barycentric interpolation of a TIN (lattice coordinates) at the grid nodes it covers."""
    for t in range(tris.shape[0]):
        a = tris[t, 0]
        b = tris[t, 1]
        c = tris[t, 2]
        ax, ay, bx, by, cx, cy = lx[a], ly[a], lx[b], ly[b], lx[c], ly[c]
        area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        if area == 0:
            continue
        sgn = 1 if area > 0 else -1
        xmin = min(ax, min(bx, cx))
        xmax = max(ax, max(bx, cx))
        ymin = min(ay, min(by, cy))
        ymax = max(ay, max(by, cy))
        i0 = max(0, -((ox - xmin) // step))  # ceil((xmin - ox) / step)
        i1 = min(nx - 1, (xmax - ox) // step)
        j0 = max(0, -((oy - ymin) // step))
        j1 = min(ny - 1, (ymax - oy) // step)
        inv = 1.0 / area
        for j in range(j0, j1 + 1):
            py = oy + j * step
            for i in range(i0, i1 + 1):
                px = ox + i * step
                w0 = (bx - px) * (cy - py) - (by - py) * (cx - px)
                w1 = (cx - px) * (ay - py) - (cy - py) * (ax - px)
                w2 = (ax - px) * (by - py) - (ay - py) * (bx - px)
                if sgn * w0 >= 0 and sgn * w1 >= 0 and sgn * w2 >= 0:
                    out[j, i] = (w0 * z[a] + w1 * z[b] + w2 * z[c]) * inv


def tin_surface_on_grid(mesh: Mesh, grid: Grid) -> np.ndarray:
    """Height of the piecewise-linear mesh surface at every node of ``grid`` (NaN where uncovered).

    Works for any mesh whose vertices are lattice points (TINs, stitched pieces with inserted
    vertices, collision meshes); the test is exact integer arithmetic in lattice units.
    """
    lat = mesh.lattice if mesh.lattice is not None else np.rint(mesh.vertices[:, :2] / 0.01).astype(np.int64)
    ox, oy = grid.lattice_origin()
    out = np.full((grid.ny, grid.nx), np.nan, dtype=np.float64)
    if numba is None:  # pragma: no cover
        raise RuntimeError("tin_surface_on_grid needs numba")
    _raster_tin(np.ascontiguousarray(lat[:, 0]), np.ascontiguousarray(lat[:, 1]),
                np.ascontiguousarray(mesh.vertices[:, 2]), np.ascontiguousarray(mesh.triangles),
                np.int64(ox), np.int64(oy), np.int64(grid.lattice_step()), np.int64(grid.nx),
                np.int64(grid.ny), out)
    return out


def tin_max_error(mesh: Mesh, grid: Grid) -> dict:
    """Max / RMS |z_grid - z_mesh| over all grid nodes, plus the count of uncovered nodes."""
    surf = tin_surface_on_grid(mesh, grid)
    covered = np.isfinite(surf)
    dz = np.abs(surf[covered] - np.asarray(grid.z, dtype=np.float64)[covered])
    return {
        "max_abs_m": float(dz.max()) if dz.size else float("nan"),
        "rms_m": float(np.sqrt(np.mean(dz ** 2))) if dz.size else float("nan"),
        "uncovered_nodes": int((~covered).sum()),
        "n_nodes": int(covered.size),
    }
