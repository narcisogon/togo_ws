"""Merge placed rocks into one mesh (docs/DESIGN.md section 5, "Merging into tiles").

``rocks_mesh(rows, library)`` transforms the LOD geometry of every catalog row
(``p = t + s * R p_proto``), rotates the normals, keeps the atlas UVs (glTF convention) and returns a
single ``mesh.types.Mesh`` (one primitive, one material: the shared rock atlas). Work is vectorised
per (prototype, LOD) group, so ~200 k rocks take well under a second per million output vertices.

* Rotations are proper (det +1) and scales positive, so the outward CCW winding is preserved.
* ``lattice`` is -1 for every vertex (rock vertices are not on the height-grid lattice).
* Vertex order: groups sorted by (prototype, LOD), rows in catalog order within a group.
"""
from __future__ import annotations

import numpy as np

from ..mesh.types import Mesh
from .placement import catalog_quats, quat_to_matrix


def rocks_mesh(catalog_rows: np.ndarray, library, lod_override: int | None = None, name: str = "",
               dtype=np.float64) -> Mesh:
    """One mesh with the transformed LOD geometry of every row (positions, normals, atlas UVs)."""
    rows = np.asarray(catalog_rows)
    n = len(rows)
    if n == 0:
        return Mesh(np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64), np.zeros((0, 3), np.float32),
                    np.zeros((0, 2), np.float32), np.zeros((0, 2), np.int64), name=name,
                    meta={"n_rocks": 0, "triangles_by_lod": {}})
    lod = rows["lod"].astype(np.int64) if lod_override is None else np.full(n, int(lod_override), dtype=np.int64)
    n_lods = np.array([p.n_lods for p in library.prototypes])[rows["proto"]]
    lod = np.minimum(lod, n_lods - 1)
    key = rows["proto"].astype(np.int64) * 16 + lod
    order = np.argsort(key, kind="stable")
    uk, start, counts = np.unique(key[order], return_index=True, return_counts=True)
    groups = []
    nv = nt = 0
    for k, s0, c in zip(uk, start, counts):
        pos, nrm, uv, idx = library.lod_mesh(int(k // 16), int(k % 16))
        groups.append((order[s0:s0 + c], pos, nrm, uv, idx, nv, nt))
        nv += c * len(pos)
        nt += c * len(idx)
    V = np.empty((nv, 3), dtype=dtype)
    N = np.empty((nv, 3), dtype=np.float32)
    UV = np.empty((nv, 2), dtype=np.float32)
    T = np.empty((nt, 3), dtype=np.int64)
    R_all = quat_to_matrix(catalog_quats(rows))
    t_all = np.column_stack([rows["x"], rows["y"], rows["z"]])
    s_all = rows["scale"].astype(np.float64)
    tri_by_lod: dict[int, int] = {}
    for sel, pos, nrm, uv, idx, v0, t0 in groups:
        k = len(sel)
        m = len(pos)
        R = R_all[sel]
        P = np.einsum("kij,mj->kmi", R, pos.astype(np.float64)) * s_all[sel, None, None] + t_all[sel, None, :]
        V[v0:v0 + k * m] = P.reshape(-1, 3)
        Nr = np.einsum("kij,mj->kmi", R, nrm.astype(np.float64))
        Nr /= np.linalg.norm(Nr, axis=2, keepdims=True)
        N[v0:v0 + k * m] = Nr.reshape(-1, 3)
        UV[v0:v0 + k * m] = np.broadcast_to(uv, (k, m, 2)).reshape(-1, 2)
        offs = v0 + np.arange(k, dtype=np.int64)[:, None, None] * m
        T[t0:t0 + k * len(idx)] = (idx.astype(np.int64)[None, :, :] + offs).reshape(-1, 3)
        lv = int(lod[sel[0]])
        tri_by_lod[lv] = tri_by_lod.get(lv, 0) + k * len(idx)
    mesh = Mesh(V, T, N, UV, np.full((nv, 2), -1, dtype=np.int64), name=name,
                meta={"n_rocks": int(n), "triangles_by_lod": tri_by_lod, "kind": "rocks"})
    return mesh


def rocks_mesh_by_tile(catalog: np.ndarray, library, lod_override: int | None = None) -> dict[tuple[int, int, int], Mesh]:
    """{(zone, tile_i, tile_j): Mesh} for every tile / ring strip that holds rocks."""
    from ..layout import RING_SIDES
    from .catalog import split_by_tile

    cat = np.asarray(catalog)
    out = {}
    for key, idx in split_by_tile(cat).items():
        zone, ti, tj = key
        name = f"tile_{ti:02d}_{tj:02d}_rocks" if zone == 0 else f"ring_{RING_SIDES[ti]}_rocks"
        out[key] = rocks_mesh(cat[idx], library, lod_override, name=name)
    return out
