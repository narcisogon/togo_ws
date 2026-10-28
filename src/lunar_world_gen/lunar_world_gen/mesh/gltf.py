"""Minimal, fast binary glTF (GLB) writer and reader for ground and rock meshes.

GLB rules (docs/DESIGN.md section 6, verified in Gazebo by the gazebo agent):

* positions are raw Z-up world metres - no Y-up conversion, no node transform;
* float32 POSITION / NORMAL / TEXCOORD_0, uint32 indices, triangles CCW seen from outside
  (ground: from +z) because Gazebo culls back faces for the LiDAR and the camera;
* exactly one primitive and one explicit material with ``metallicFactor`` 0 (a GLB without a
  material renders almost black in ogre2), ``roughnessFactor`` and ``baseColorFactor`` from the
  ``MaterialSpec``; no textures (they come from the SDF PBR material, shared by all tiles).

No TANGENT attribute is needed: ogre2 derives tangents itself, and an SDF normal map shaded
identically with and without a GLB TANGENT in the Gazebo check (docs/impl/mesh.md); the
``tangents=`` option exists for other renderers. An SDF ``<material>`` on the visual overrides
the GLB material (verified), so the GLB material only matters when the SDF gives none.

The writer streams the numpy buffers straight into the file (no intermediate copies of the
binary chunk), so millions of vertices are written at disk speed (2 M triangles / 56 MB in
0.21 s on the WSL ext4 disk). ``read_glb`` parses the files
back into a ``Mesh`` (any accessor layout of plain glTF 2.0 triangles), and ``verify_glb``
checks a written file both with this reader and with trimesh.
"""
from __future__ import annotations

import json
import os
import struct
import uuid
from pathlib import Path
from typing import Any

import numpy as np

from .types import MaterialSpec, Mesh

__all__ = ["write_glb", "read_glb", "read_glb_json", "verify_glb", "planar_tangents"]

_MAGIC = 0x46546C67  # b"glTF"
_CHUNK_JSON = 0x4E4F534A  # b"JSON"
_CHUNK_BIN = 0x004E4942  # b"BIN\0"
_FLOAT = 5126
_UINT32 = 5125
_ARRAY_BUFFER = 34962
_ELEMENT_ARRAY_BUFFER = 34963
_COMPONENT_DTYPE = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_TYPE_WIDTH = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
GENERATOR = "lunar_world_gen.mesh.gltf"


def planar_tangents(normals: np.ndarray, handedness: float = 1.0) -> np.ndarray:
    """glTF TANGENT (x, y, z, w) for world-planar UVs (u along +x): +x projected on the tangent plane.

    ``w`` is the bitangent sign: bitangent = cross(normal, tangent) * w. With u = x / tile and
    v = -y / tile the image "up" (+Y of an OpenGL normal map) is world +y = cross(+z, +x), i.e.
    w = +1 in the glTF sense. Only needed if the renderer does not derive tangents itself.
    """
    n = np.asarray(normals, dtype=np.float64).reshape(-1, 3)
    t = np.zeros_like(n)
    t[:, 0] = 1.0
    t -= n * n[:, :1]  # t = x - (x.n) n
    length = np.linalg.norm(t, axis=1, keepdims=True)
    bad = length[:, 0] < 1e-8
    t = t / np.maximum(length, 1e-300)
    if np.any(bad):  # normal parallel to x: use y instead
        alt = np.zeros((int(bad.sum()), 3))
        alt[:, 1] = 1.0
        alt -= n[bad] * n[bad][:, 1:2]
        t[bad] = alt / np.linalg.norm(alt, axis=1, keepdims=True)
    out = np.empty((n.shape[0], 4), dtype=np.float32)
    out[:, :3] = t
    out[:, 3] = 1.0 if handedness >= 0 else -1.0
    return out


def _as_le(a: np.ndarray, dtype) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.dtype(dtype).newbyteorder("<"))


def write_glb(path: str | Path, mesh: Mesh, material: MaterialSpec, *, node_name: str | None = None,
              tangents: np.ndarray | None = None, extras: dict | None = None) -> dict:
    """Write ``mesh`` as a single-primitive GLB with one explicit material.

    Args:
        path: output file (parent folders are created).
        mesh: vertices (world ENU metres, Z-up), CCW triangles; ``normals`` and ``uvs`` are
            written when present (ground meshes must have both).
        material: base colour (linear RGBA), metallic (must be 0 for Gazebo), roughness.
        node_name: glTF node / mesh name (default ``mesh.name`` or the file stem).
        tangents: optional (n, 4) glTF TANGENT attribute (see ``planar_tangents``).
        extras: optional JSON-serialisable dict stored in the asset ``extras``.

    Returns:
        dict(path, bytes, n_vertices, n_triangles, bounds_min, bounds_max, attributes).
    """
    path = Path(path)
    n = mesh.n_vertices
    m = mesh.n_triangles
    if n == 0 or m == 0:
        raise ValueError(f"{path.name}: refusing to write an empty mesh (Gazebo needs real geometry)")
    if m and (mesh.triangles.min() < 0 or mesh.triangles.max() >= n):
        raise ValueError(f"{path.name}: triangle index out of range")
    if not np.all(np.isfinite(mesh.vertices)):
        raise ValueError(f"{path.name}: non-finite vertex coordinates")
    if n >= 2 ** 32 - 1:
        raise ValueError(f"{path.name}: too many vertices for uint32 indices")
    name = node_name or mesh.name or path.stem

    pos = _as_le(mesh.vertices, np.float32)
    arrays: list[tuple[str, np.ndarray, int, str, int]] = [("POSITION", pos, _FLOAT, "VEC3", _ARRAY_BUFFER)]
    if mesh.normals is not None:
        nrm = np.asarray(mesh.normals, dtype=np.float64)
        length = np.linalg.norm(nrm, axis=1, keepdims=True)
        if np.any(~np.isfinite(length)) or np.any(length < 1e-6):
            raise ValueError(f"{path.name}: zero or non-finite normals")
        arrays.append(("NORMAL", _as_le(nrm / length, np.float32), _FLOAT, "VEC3", _ARRAY_BUFFER))
    if mesh.uvs is not None:
        arrays.append(("TEXCOORD_0", _as_le(mesh.uvs, np.float32), _FLOAT, "VEC2", _ARRAY_BUFFER))
    if tangents is not None:
        tg = _as_le(np.asarray(tangents).reshape(-1, 4), np.float32)
        if tg.shape[0] != n:
            raise ValueError(f"{path.name}: tangents shape {tg.shape} != ({n}, 4)")
        arrays.append(("TANGENT", tg, _FLOAT, "VEC4", _ARRAY_BUFFER))
    idx = _as_le(mesh.triangles, np.uint32).reshape(-1)
    arrays.append(("indices", idx, _UINT32, "SCALAR", _ELEMENT_ARRAY_BUFFER))

    accessors: list[dict[str, Any]] = []
    views: list[dict[str, Any]] = []
    offset = 0
    attributes: dict[str, int] = {}
    indices_acc = -1
    for key, arr, ctype, atype, target in arrays:
        nbytes = arr.nbytes
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": nbytes, "target": target})
        acc: dict[str, Any] = {"bufferView": len(views) - 1, "componentType": ctype,
                               "count": int(arr.shape[0] if arr.ndim > 1 else arr.size), "type": atype}
        if key == "POSITION":
            acc["min"] = [float(v) for v in pos.min(axis=0)]
            acc["max"] = [float(v) for v in pos.max(axis=0)]
        accessors.append(acc)
        if key == "indices":
            indices_acc = len(accessors) - 1
        else:
            attributes[key] = len(accessors) - 1
        offset += nbytes + (-nbytes) % 4
    bin_len = offset

    r, g, b, a = (float(c) for c in material.base_color)
    doc = {
        "asset": {"version": "2.0", "generator": GENERATOR},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": name}],
        "meshes": [{"name": name, "primitives": [
            {"attributes": attributes, "indices": indices_acc, "material": 0, "mode": 4}]}],
        "materials": [{
            "name": material.name,
            "pbrMetallicRoughness": {
                "baseColorFactor": [r, g, b, a],
                "metallicFactor": float(material.metallic),
                "roughnessFactor": float(material.roughness),
            },
            "doubleSided": False,
        }],
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": bin_len}],
    }
    if extras:
        doc["asset"]["extras"] = extras
    js = json.dumps(doc, separators=(",", ":")).encode("utf-8")
    js += b" " * ((-len(js)) % 4)
    total = 12 + 8 + len(js) + 8 + bin_len

    path.parent.mkdir(parents=True, exist_ok=True)
    # unique temporary name (pid + random): two writers never share a temp file (CR-06)
    tmp = path.with_name(f"{path.name}.tmp{os.getpid()}_{uuid.uuid4().hex[:8]}")
    try:
        with open(tmp, "wb") as f:
            f.write(struct.pack("<III", _MAGIC, 2, total))
            f.write(struct.pack("<II", len(js), _CHUNK_JSON))
            f.write(js)
            f.write(struct.pack("<II", bin_len, _CHUNK_BIN))
            for _key, arr, *_ in arrays:
                f.write(memoryview(arr).cast("B"))
                pad = (-arr.nbytes) % 4
                if pad:
                    f.write(b"\0" * pad)
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)
    return {
        "path": str(path),
        "bytes": int(total),
        "n_vertices": int(n),
        "n_triangles": int(m),
        "bounds_min": accessors[0]["min"],
        "bounds_max": accessors[0]["max"],
        "attributes": sorted(attributes),
    }


def _read_chunks(path: str | Path) -> tuple[dict, bytes]:
    data = Path(path).read_bytes()
    if len(data) < 20:
        raise ValueError(f"{path}: too short for a GLB")
    magic, version, total = struct.unpack_from("<III", data, 0)
    if magic != _MAGIC or version != 2:
        raise ValueError(f"{path}: not a glTF 2.0 binary file")
    if total != len(data):
        raise ValueError(f"{path}: header length {total} != file size {len(data)}")
    pos = 12
    doc = None
    binary = b""
    while pos < total:
        clen, ctype = struct.unpack_from("<II", data, pos)
        pos += 8
        chunk = data[pos:pos + clen]
        if ctype == _CHUNK_JSON:
            doc = json.loads(chunk.decode("utf-8"))
        elif ctype == _CHUNK_BIN:
            binary = chunk
        pos += clen
    if doc is None:
        raise ValueError(f"{path}: no JSON chunk")
    return doc, binary


def read_glb_json(path: str | Path) -> dict:
    """The glTF JSON document of a GLB file."""
    return _read_chunks(path)[0]


def _accessor(doc: dict, binary: bytes, k: int) -> np.ndarray:
    acc = doc["accessors"][k]
    view = doc["bufferViews"][acc["bufferView"]]
    dtype = np.dtype(_COMPONENT_DTYPE[acc["componentType"]]).newbyteorder("<")
    width = _TYPE_WIDTH[acc["type"]]
    count = acc["count"]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", 0) or dtype.itemsize * width
    if stride == dtype.itemsize * width:
        arr = np.frombuffer(binary, dtype=dtype, count=count * width, offset=start)
    else:
        rows = np.frombuffer(binary, dtype=np.uint8, count=stride * (count - 1) + dtype.itemsize * width, offset=start)
        arr = np.lib.stride_tricks.as_strided(rows, shape=(count, dtype.itemsize * width), strides=(stride, 1))
        arr = np.ascontiguousarray(arr).view(dtype)
    return arr.reshape(count, width) if width > 1 else arr.reshape(count)


def read_glb(path: str | Path) -> Mesh:
    """Read a GLB written by ``write_glb`` (or any plain triangle glTF) back into a ``Mesh``.

    All primitives of all meshes are concatenated (node transforms are NOT applied - our files
    have none; ``meta['node_transforms']`` reports whether any exist). ``meta`` also holds the
    materials, the number of primitives and the raw glTF JSON.
    """
    doc, binary = _read_chunks(path)
    parts = []
    n_prims = 0
    for mdef in doc.get("meshes", []):
        for prim in mdef["primitives"]:
            n_prims += 1
            if prim.get("mode", 4) != 4:
                raise ValueError(f"{path}: only triangle primitives are supported")
            att = prim["attributes"]
            pos = _accessor(doc, binary, att["POSITION"]).astype(np.float64)
            tris = (_accessor(doc, binary, prim["indices"]).astype(np.int64).reshape(-1, 3)
                    if "indices" in prim else np.arange(pos.shape[0], dtype=np.int64).reshape(-1, 3))
            nrm = _accessor(doc, binary, att["NORMAL"]) if "NORMAL" in att else None
            uv = _accessor(doc, binary, att["TEXCOORD_0"]) if "TEXCOORD_0" in att else None
            parts.append(Mesh(pos, tris, nrm, uv, name=mdef.get("name", "")))
    mesh = Mesh.concatenate(parts, name=Path(path).stem) if len(parts) != 1 else parts[0]
    mesh.meta.update({
        "materials": doc.get("materials", []),
        "n_primitives": n_prims,
        "node_transforms": any(k in nd for nd in doc.get("nodes", []) for k in ("matrix", "translation", "rotation", "scale")),
        "n_images": len(doc.get("images", [])),
        "n_textures": len(doc.get("textures", [])),
        "gltf": doc,
    })
    return mesh


def verify_glb(path: str | Path, expected: Mesh | None = None, *, require_up: bool = False,
               use_trimesh: bool = True) -> dict:
    """Check a written GLB against the DESIGN rules and (optionally) the source mesh.

    Checks: one primitive, one material with metallicFactor 0, no textures/images, no node
    transforms, float32 positions; if ``expected`` is given, identical vertex/triangle counts,
    float32-exact positions and identical indices; trimesh loads the file with the same vertex
    and face counts and bounds; with ``require_up`` the fraction of +z vertex normals (and face
    normals) is reported. Returns a dict with ``ok`` and the individual results.
    """
    res: dict[str, Any] = {"path": str(path), "errors": []}
    mesh = read_glb(path)
    mats = mesh.meta["materials"]
    res.update(n_vertices=mesh.n_vertices, n_triangles=mesh.n_triangles,
               n_primitives=mesh.meta["n_primitives"], n_materials=len(mats),
               bounds_min=mesh.vertices.min(axis=0).tolist(), bounds_max=mesh.vertices.max(axis=0).tolist())
    if mesh.meta["n_primitives"] != 1:
        res["errors"].append("not exactly one primitive")
    if len(mats) != 1:
        res["errors"].append("not exactly one material")
    elif mats[0].get("pbrMetallicRoughness", {}).get("metallicFactor", 1.0) != 0.0:
        res["errors"].append("metallicFactor is not 0")
    if mesh.meta["n_images"] or mesh.meta["n_textures"]:
        res["errors"].append("embedded textures present")
    if mesh.meta["node_transforms"]:
        res["errors"].append("node transform present")
    if expected is not None:
        if mesh.n_vertices != expected.n_vertices or mesh.n_triangles != expected.n_triangles:
            res["errors"].append("vertex/triangle count differs from the source mesh")
        else:
            if not np.array_equal(mesh.vertices, expected.vertices.astype(np.float32).astype(np.float64)):
                res["errors"].append("positions differ from the float32 source positions")
            if not np.array_equal(mesh.triangles, expected.triangles):
                res["errors"].append("indices differ")
            if expected.normals is not None and (mesh.normals is None or
                                                 np.max(np.abs(mesh.normals - expected.normals)) > 1e-6):
                res["errors"].append("normals differ")
            if expected.uvs is not None and (mesh.uvs is None or not np.array_equal(mesh.uvs, expected.uvs)):
                res["errors"].append("uvs differ")
    if require_up:
        up_faces = float(np.mean(mesh.face_normals()[:, 2] > 0))
        res["up_face_fraction"] = up_faces
        if mesh.normals is not None:
            res["up_normal_fraction"] = float(np.mean(mesh.normals[:, 2] > 0))
        if up_faces < 1.0:
            res["errors"].append(f"{(1 - up_faces) * 100:.4f} % of faces point down")
    if use_trimesh:
        try:
            import trimesh

            loaded = trimesh.load(str(path), process=False)
            geoms = list(loaded.geometry.values()) if hasattr(loaded, "geometry") else [loaded]
            tv = sum(len(g.vertices) for g in geoms)
            tf = sum(len(g.faces) for g in geoms)
            tb = np.asarray(loaded.bounds, dtype=np.float64)
            res["trimesh"] = {"geometries": len(geoms), "vertices": int(tv), "faces": int(tf),
                              "bounds": tb.tolist()}
            if tv != mesh.n_vertices or tf != mesh.n_triangles:
                res["errors"].append("trimesh counts differ")
            ref = np.array([mesh.vertices.min(axis=0), mesh.vertices.max(axis=0)])
            if not np.allclose(tb, ref, rtol=0, atol=1e-6 * max(1.0, float(np.abs(ref).max()))):
                res["errors"].append("trimesh bounds differ")
        except Exception as exc:  # pragma: no cover - reported, not raised
            res["errors"].append(f"trimesh failed: {exc!r}")
    res["ok"] = not res["errors"]
    return res
