"""Static acceptance checks of a built world (docs/DESIGN.md section 13, "Static").

``run_static_checks(cfg)`` reads the products of a build from ``cfg.paths.build_dir`` and
``cfg.paths.world_dir`` and writes ``world_dir/reports/static_checks.json``:

==========================  ===========================================================  =====================
check                       what is measured                                             DESIGN threshold
==========================  ===========================================================  =====================
``roughness``               plane-detrended directional deviogram of core H0 (no rocks)  site-aware (DESIGN 13):
                            at 0.01..10 m, of the DTM base B and of H0 - B, RMS slopes,  H0 +-25 % at 0.01-0.3 m;
                            radial PSD fit, slope stats                                  H0-B +-25 % at 1, 3 m;
                                                                                         H0 <= 1.15 B at >= 10 m
``craters``                 N(>=D) of craters centred in the core vs the population       +-20 % at 0.2, 0.5,
                            model (C D^-b, D_max, 8-16 m fade)                           1, 2 m
``rocks``                   N(>=D) of core rocks vs the broken power law                  Poisson +-3 sigma at
                            (+ ring rocks >= 30 cm, informational)                        2, 5, 10, 20 cm
``visual_vs_collision``     dz = collision - visual on rock-free core ground              median -20 +-3 mm,
                            (meshes; grids as fallback)                                  RMS(dz-med) <= 10 mm,
                                                                                         p99 |dz-med| <= 30 mm
``glb``                     every GLB: loads in trimesh, Z-up bounds vs its rectangle,     +z normals >= 99.9 %
                            +z vertex normals / CCW faces, material, dtypes, sizes        of ground vertices
``stitching``               shared boundary vertex sets of adjacent pieces from the GLB    0 mismatches,
                            files (positions rounded to 1e-5 m), T-junctions, heights,    0 T-junctions
                            interior cracks
``sdf``                     every URI in every SDF resolves (meshes, textures, engine)     all resolve
``world_info``              world_info.yaml present, sites inside the core                informational
==========================  ===========================================================  =====================

Each check returns ``{"status": "pass"|"fail"|"skipped"|"error", "pass": bool|None, ...}``.
A producer that has not run yet makes its check ``skipped`` (never a crash).

Inputs are discovered in the build/world dirs (see ``locate_inputs``); explicit paths can
be given through ``inputs=`` or the config key ``validation.inputs`` (same names).

The module only needs numpy/scipy/yaml at import time (it also runs under the system python
for the ROS-side tools); trimesh is imported lazily for the "loads in trimesh" check.

Run:  python -m lunar_world_gen.validate.static_checks [--config CFG] [--preset P] [--only a,b]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from ..config import Config, config_hash, load_config, rng
from ..grid import Grid
from ..layout import RING_SIDES, Layout, Rect, TileId
from . import stats as st

STATUS_ORDER = ("pass", "fail", "skipped", "error")

# ==============================================================================================
# GLB reading / writing (numpy only)

GLB_MAGIC = 0x46546C67
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN = 0x004E4942
_COMP = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT2": 4, "MAT3": 9, "MAT4": 16}
COMP_NAMES = {5120: "int8", 5121: "uint8", 5122: "int16", 5123: "uint16", 5125: "uint32", 5126: "float32"}


@dataclass
class GlbData:
    """Geometry and metadata of one GLB file (all primitives concatenated)."""

    path: str
    file_size: int
    positions: np.ndarray            # (n, 3) float64, node transforms applied
    positions_f32: np.ndarray        # (n, 3) float32 as stored (before node transforms)
    triangles: np.ndarray            # (m, 3) int64
    normals: np.ndarray | None
    uvs: np.ndarray | None
    info: dict = field(default_factory=dict)

    @property
    def n_vertices(self) -> int:
        return int(self.positions.shape[0])

    @property
    def n_triangles(self) -> int:
        return int(self.triangles.shape[0])


def read_glb_json(path: str | Path) -> tuple[dict, bytes | None]:
    """Parse the GLB container; returns (gltf json, BIN chunk bytes or None)."""
    data = Path(path).read_bytes()
    if len(data) < 20:
        raise ValueError(f"{path}: too short for a GLB")
    magic, version, length = struct.unpack_from("<III", data, 0)
    if magic != GLB_MAGIC:
        raise ValueError(f"{path}: not a GLB (bad magic)")
    if version != 2:
        raise ValueError(f"{path}: GLB version {version} != 2")
    off = 12
    gltf, binary = None, None
    while off + 8 <= min(length, len(data)):
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == CHUNK_JSON:
            gltf = json.loads(chunk.decode("utf-8"))
        elif ctype == CHUNK_BIN and binary is None:
            binary = chunk
        off += 8 + clen
    if gltf is None:
        raise ValueError(f"{path}: no JSON chunk")
    return gltf, binary


def _accessor_array(gltf: dict, binary: bytes | None, idx: int) -> np.ndarray:
    acc = gltf["accessors"][idx]
    dtype = np.dtype(_COMP[acc["componentType"]]).newbyteorder("<")
    ncomp = _NCOMP[acc["type"]]
    count = int(acc["count"])
    if "bufferView" not in acc:
        return np.zeros((count, ncomp), dtype=dtype)
    bv = gltf["bufferViews"][acc["bufferView"]]
    if bv.get("buffer", 0) != 0 or binary is None:
        raise ValueError("only GLB-embedded buffer 0 is supported")
    start = int(bv.get("byteOffset", 0)) + int(acc.get("byteOffset", 0))
    elem = dtype.itemsize * ncomp
    stride = int(bv.get("byteStride", 0)) or elem
    if stride == elem:
        arr = np.frombuffer(binary, dtype=dtype, count=count * ncomp, offset=start).reshape(count, ncomp)
    else:
        raw = np.frombuffer(binary, dtype=np.uint8, count=stride * (count - 1) + elem, offset=start)
        arr = np.lib.stride_tricks.as_strided(raw, shape=(count, elem), strides=(stride, 1))
        arr = np.ascontiguousarray(arr).view(dtype).reshape(count, ncomp)
    if "sparse" in acc:
        raise ValueError("sparse accessors are not supported")
    return arr


def _node_matrix(node: dict) -> np.ndarray:
    if "matrix" in node:
        return np.asarray(node["matrix"], dtype=np.float64).reshape(4, 4).T  # column-major
    M = np.eye(4)
    if "scale" in node:
        M = np.diag(list(node["scale"]) + [1.0]) @ M
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        T = np.eye(4)
        T[:3, :3] = R
        M = T @ M
    if "translation" in node:
        T = np.eye(4)
        T[:3, 3] = node["translation"]
        M = T @ M
    return M


def read_glb(path: str | Path, header_only: bool = False) -> GlbData:
    """Read every triangle primitive of a GLB (node transforms of the default scene applied).

    ``info`` records what the validation needs: primitive/mesh/node counts, component types,
    whether all node transforms are identity, materials (with whether ``metallicFactor`` is
    explicitly present), image/texture counts and the accessor bounds.
    """
    path = Path(path)
    gltf, binary = read_glb_json(path)
    info: dict[str, Any] = {"n_meshes": len(gltf.get("meshes", [])), "n_nodes": len(gltf.get("nodes", [])),
                            "n_images": len(gltf.get("images", [])), "n_textures": len(gltf.get("textures", [])),
                            "n_materials": len(gltf.get("materials", [])), "primitives": []}
    mats = []
    for m in gltf.get("materials", []):
        pbr = m.get("pbrMetallicRoughness", {})
        mats.append({"name": m.get("name"), "metallicFactor": pbr.get("metallicFactor"),
                     "metallic_explicit": "metallicFactor" in pbr,
                     "roughnessFactor": pbr.get("roughnessFactor"),
                     "baseColorFactor": pbr.get("baseColorFactor"),
                     "has_textures": any(k in pbr for k in ("baseColorTexture", "metallicRoughnessTexture"))
                     or any(k in m for k in ("normalTexture", "occlusionTexture", "emissiveTexture"))})
    info["materials"] = mats
    # scene graph: collect (mesh index, world matrix)
    nodes = gltf.get("nodes", [])
    scenes = gltf.get("scenes", [])
    if scenes:
        roots = scenes[gltf.get("scene", 0)].get("nodes", [])
    else:
        children = {c for n in nodes for c in n.get("children", [])}
        roots = [i for i in range(len(nodes)) if i not in children]
    inst: list[tuple[int, np.ndarray]] = []
    identity = True

    def walk(i: int, parent: np.ndarray, depth: int = 0) -> None:
        nonlocal identity
        if depth > 64:
            raise ValueError("node hierarchy too deep / cyclic")
        n = nodes[i]
        M = parent @ _node_matrix(n)
        if not np.allclose(_node_matrix(n), np.eye(4), atol=1e-12):
            identity = False
        if "mesh" in n:
            inst.append((int(n["mesh"]), M))
        for c in n.get("children", []):
            walk(c, M, depth + 1)

    for r in roots:
        walk(r, np.eye(4))
    if not inst and gltf.get("meshes"):
        inst = [(k, np.eye(4)) for k in range(len(gltf["meshes"]))]
    info["identity_transforms"] = identity
    info["n_mesh_instances"] = len(inst)
    pos_list, pos32_list, tri_list, nrm_list, uv_list = [], [], [], [], []
    have_n, have_uv = True, True
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    nv = 0
    for mesh_idx, M in inst:
        for prim in gltf["meshes"][mesh_idx].get("primitives", []):
            attrs = prim.get("attributes", {})
            mode = prim.get("mode", 4)
            pinfo = {"mode": mode, "material": prim.get("material"),
                     "attributes": sorted(attrs)}
            pacc = gltf["accessors"][attrs["POSITION"]]
            pinfo["position_type"] = COMP_NAMES.get(pacc["componentType"], str(pacc["componentType"]))
            pinfo["n_vertices"] = int(pacc["count"])
            if "indices" in prim:
                iacc = gltf["accessors"][prim["indices"]]
                pinfo["index_type"] = COMP_NAMES.get(iacc["componentType"], str(iacc["componentType"]))
                pinfo["n_indices"] = int(iacc["count"])
            else:
                pinfo["index_type"] = None
                pinfo["n_indices"] = int(pacc["count"])
            for key in ("NORMAL", "TEXCOORD_0"):
                if key in attrs:
                    pinfo[key.lower() + "_type"] = COMP_NAMES.get(gltf["accessors"][attrs[key]]["componentType"])
            if "min" in pacc and "max" in pacc:
                pmin = np.asarray(pacc["min"], dtype=np.float64)
                pmax = np.asarray(pacc["max"], dtype=np.float64)
                corners = np.array([[a, b, c] for a in (pmin[0], pmax[0]) for b in (pmin[1], pmax[1])
                                    for c in (pmin[2], pmax[2])])
                cw = corners @ M[:3, :3].T + M[:3, 3]
                lo = np.minimum(lo, cw.min(0))
                hi = np.maximum(hi, cw.max(0))
            info["primitives"].append(pinfo)
            if header_only:
                continue
            if mode != 4:
                continue  # only triangle lists are geometry for us
            p32 = _accessor_array(gltf, binary, attrs["POSITION"]).astype(np.float32, copy=False)
            P = p32.astype(np.float64)
            if not np.allclose(M, np.eye(4), atol=1e-12):
                P = P @ M[:3, :3].T + M[:3, 3]
            if "indices" in prim:
                idx = _accessor_array(gltf, binary, prim["indices"]).astype(np.int64).reshape(-1)
            else:
                idx = np.arange(len(P), dtype=np.int64)
            if len(idx) % 3:
                raise ValueError(f"{path}: index count {len(idx)} not a multiple of 3")
            pos_list.append(P)
            pos32_list.append(p32)
            tri_list.append(idx.reshape(-1, 3) + nv)
            nv += len(P)
            if "NORMAL" in attrs:
                N = _accessor_array(gltf, binary, attrs["NORMAL"]).astype(np.float64)
                if not np.allclose(M[:3, :3], np.eye(3), atol=1e-12):
                    N = N @ np.linalg.inv(M[:3, :3]).T
                nrm_list.append(N.astype(np.float32))
            else:
                have_n = False
            if "TEXCOORD_0" in attrs:
                uv_list.append(_accessor_array(gltf, binary, attrs["TEXCOORD_0"]).astype(np.float32))
            else:
                have_uv = False
    info["bounds_accessor"] = [lo.tolist(), hi.tolist()] if np.all(np.isfinite(lo)) else None
    info["n_primitives"] = len(info["primitives"])
    if pos_list:
        positions = np.vstack(pos_list)
        positions_f32 = np.vstack(pos32_list)
        triangles = np.vstack(tri_list)
    else:
        positions = np.zeros((0, 3))
        positions_f32 = np.zeros((0, 3), np.float32)
        triangles = np.zeros((0, 3), np.int64)
    normals = np.vstack(nrm_list) if (pos_list and have_n and nrm_list) else None
    uvs = np.vstack(uv_list) if (pos_list and have_uv and uv_list) else None
    return GlbData(str(path), path.stat().st_size, positions, positions_f32, triangles, normals, uvs, info)


def write_glb(path: str | Path, positions: np.ndarray, triangles: np.ndarray, normals: np.ndarray | None = None,
              uvs: np.ndarray | None = None, *, material: bool = True, metallic: float = 0.0,
              roughness: float = 0.95, base_color: Sequence[float] = (0.12, 0.107, 0.088, 1.0),
              node_translation: Sequence[float] | None = None) -> Path:
    """Minimal GLB writer following the DESIGN section 6 rules (used for tests / synthetic data;
    the production writer is ``mesh/gltf.py``)."""
    path = Path(path)
    P = np.ascontiguousarray(positions, dtype=np.float32).reshape(-1, 3)
    T = np.ascontiguousarray(triangles, dtype=np.uint32).reshape(-1)
    blobs, views, accessors = [], [], []
    offset = 0

    def add(arr: np.ndarray, comp: int, typ: str, target: int, minmax: bool = False) -> int:
        nonlocal offset
        b = arr.tobytes()
        pad = (-len(b)) % 4
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(b), "target": target})
        acc = {"bufferView": len(views) - 1, "componentType": comp, "count": int(arr.shape[0]), "type": typ}
        if minmax:
            acc["min"] = arr.min(0).astype(float).tolist()
            acc["max"] = arr.max(0).astype(float).tolist()
        accessors.append(acc)
        blobs.append(b + b"\0" * pad)
        offset += len(b) + pad
        return len(accessors) - 1

    attrs = {"POSITION": add(P, 5126, "VEC3", 34962, True)}
    if normals is not None:
        attrs["NORMAL"] = add(np.ascontiguousarray(normals, dtype=np.float32).reshape(-1, 3), 5126, "VEC3", 34962)
    if uvs is not None:
        attrs["TEXCOORD_0"] = add(np.ascontiguousarray(uvs, dtype=np.float32).reshape(-1, 2), 5126, "VEC2", 34962)
    ind = add(T.reshape(-1, 1), 5125, "SCALAR", 34963)
    accessors[ind]["type"] = "SCALAR"
    prim: dict[str, Any] = {"attributes": attrs, "indices": ind, "mode": 4}
    gltf: dict[str, Any] = {"asset": {"version": "2.0", "generator": "lunar_world_gen.validate"},
                            "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
                            "meshes": [{"primitives": [prim]}], "accessors": accessors,
                            "bufferViews": views, "buffers": [{"byteLength": offset}]}
    if node_translation is not None:
        gltf["nodes"][0]["translation"] = [float(v) for v in node_translation]
    if material:
        prim["material"] = 0
        gltf["materials"] = [{"name": "regolith", "pbrMetallicRoughness": {
            "baseColorFactor": [float(c) for c in base_color], "metallicFactor": float(metallic),
            "roughnessFactor": float(roughness)}}]
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    binary = b"".join(blobs)
    total = 12 + 8 + len(js) + 8 + len(binary)
    out = struct.pack("<III", GLB_MAGIC, 2, total) + struct.pack("<II", len(js), CHUNK_JSON) + js
    out += struct.pack("<II", len(binary), CHUNK_BIN) + binary
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(out)
    return path


def trimesh_load_check(path: str | Path) -> dict:
    """Load with trimesh (process=False) exactly as a consumer would; returns ok/error and counts."""
    try:
        import trimesh
    except ImportError as e:  # system python has no trimesh
        return {"ok": None, "error": f"trimesh not available: {e}"}
    try:
        sc = trimesh.load(str(path), force="scene", process=False)
        nv = sum(len(g.vertices) for g in sc.geometry.values())
        nf = sum(len(g.faces) for g in sc.geometry.values() if hasattr(g, "faces"))
        return {"ok": True, "n_geometries": len(sc.geometry), "n_vertices": int(nv), "n_faces": int(nf)}
    except Exception as e:  # noqa: BLE001 - any failure is the result
        return {"ok": False, "error": f"{type(e).__name__}: {e}"[:300]}


# ==============================================================================================
# point location in a TIN (2.5-D height-field meshes)

# TinLocator moved to mesh/locate.py (the build seats rocks on the rendered TIN with it); re-exported here
from ..mesh.locate import TinLocator  # noqa: E402,F401


class MeshHeightField:
    """Height of a set of 2.5-D GLB meshes at (x, y): the highest surface over the point.

    Only the header (accessor bounds) of each file is read up front; a file's geometry and
    locator are built lazily when a query falls inside its bounds.
    """

    def __init__(self, paths: Iterable[str | Path], cache_size: int = 8):
        self.items = []
        for p in paths:
            try:
                g = read_glb(p, header_only=True)
            except Exception:  # noqa: BLE001 - unreadable files are skipped
                continue
            b = g.info.get("bounds_accessor")
            if b is None:
                continue
            self.items.append((str(p), np.asarray(b[0]), np.asarray(b[1])))
        self._cache: dict[str, tuple[TinLocator, np.ndarray]] = {}
        self._order: list[str] = []
        self.cache_size = cache_size

    def _loc(self, path: str) -> tuple[TinLocator, np.ndarray]:
        if path in self._cache:
            return self._cache[path]
        g = read_glb(path)
        loc = TinLocator(g.positions[:, :2], g.triangles)
        self._cache[path] = (loc, g.positions[:, 2].copy())
        self._order.append(path)
        while len(self._order) > self.cache_size:
            self._cache.pop(self._order.pop(0), None)
        return self._cache[path]

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64).ravel()
        y = np.asarray(y, dtype=np.float64).ravel()
        out = np.full(len(x), np.nan)
        for path, lo, hi in self.items:
            m = (x >= lo[0]) & (x <= hi[0]) & (y >= lo[1]) & (y <= hi[1])
            if not m.any():
                continue
            loc, zv = self._loc(path)
            z = loc.interpolate(zv, x[m], y[m])
            cur = out[m]
            out[m] = np.where(np.isnan(cur), z, np.fmax(cur, z))
        return out


# ==============================================================================================
# helpers

def _json_default(o: Any) -> Any:
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return None if not np.isfinite(o) else float(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    if isinstance(o, set):
        return sorted(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")


def _clean_nan(o: Any) -> Any:
    """Replace non-finite floats by None so the JSON is strict."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _clean_nan(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean_nan(v) for v in o]
    return o


def write_json(path: str | Path, obj: Any) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_clean_nan(json.loads(json.dumps(obj, default=_json_default))), indent=1))
    return path


def _result(status: str, **kw: Any) -> dict:
    if status not in STATUS_ORDER:
        raise ValueError(status)
    passed = True if status == "pass" else (False if status == "fail" else None)
    return {"status": status, "pass": passed, **kw}


def _cfg_get(cfg: dict, dotted: str, default: Any = None) -> Any:
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = dict.__getitem__(node, part) if isinstance(node, dict) else node[part]
    return node


def _tokens(name: str) -> list[str]:
    return [t for t in re.split(r"[^a-z0-9]+", name.lower()) if t]


# ==============================================================================================
# input discovery

GRID_EXCLUDE = {"b", "base", "dtm", "fbm", "f", "fine", "coarse", "k", "kbig", "ksmall", "crater", "craters",
                "fillet", "fillets", "mask", "weight", "w", "normal", "normals", "lowpass", "rocks", "rock",
                "slope", "uv", "albedo", "rough", "roughness", "density", "field"}
GRID_GOOD = {"h", "h0", "height", "heights", "ground", "terrain", "visual", "composite", "final", "hf"}


def read_grid_header(json_path: str | Path) -> dict | None:
    """Return {x0, y0, spacing, nx, ny, x1, y1} for a Grid/dtm_extract json, else None."""
    try:
        d = json.loads(Path(json_path).read_text())
    except Exception:  # noqa: BLE001
        return None
    if not isinstance(d, dict):
        return None
    g = d.get("grid", d) if isinstance(d.get("grid"), dict) else d
    x0 = g.get("x0", g.get("x_min"))
    y0 = g.get("y0", g.get("y_min"))
    s = g.get("spacing", g.get("spacing_m"))
    nx, ny = g.get("nx"), g.get("ny")
    if None in (x0, y0, s) or not isinstance(s, (int, float)):
        return None
    npy = Path(json_path).with_suffix(".npy")
    if not npy.exists():
        return None
    if nx is None or ny is None:
        try:
            shp = np.load(npy, mmap_mode="r").shape
            ny, nx = shp
        except Exception:  # noqa: BLE001
            return None
    return {"x0": float(x0), "y0": float(y0), "spacing": float(s), "nx": int(nx), "ny": int(ny),
            "x1": float(x0) + (int(nx) - 1) * float(s), "y1": float(y0) + (int(ny) - 1) * float(s),
            "row0": g.get("row0", "south")}


def _covers(h: dict, r: Rect, tol: float = 1e-6) -> bool:
    return h["x0"] <= r.x0 + tol and h["x1"] >= r.x1 - tol and h["y0"] <= r.y0 + tol and h["y1"] >= r.y1 - tol


def _grid_candidates(build_dir: Path) -> list[tuple[Path, dict]]:
    out = []
    if not build_dir.exists():
        return out
    for js in build_dir.rglob("*.json"):
        if js.name == "manifest.json":
            continue
        h = read_grid_header(js)
        if h:
            out.append((js.with_suffix(""), h))
    return out


def _score_grid(prefix: Path, h: dict, role: str, spacing: float, rect: Rect, root: Path | None = None) -> float | None:
    """Score a grid file for a role; None = not eligible. Tokens come from the path relative to
    the build dir, so component folders (``base/``, ``fbm/``, ``craters/``) are recognised."""
    try:
        rel = prefix.relative_to(root) if root else Path(prefix.name)
    except ValueError:
        rel = Path(prefix.name)
    toks = set(_tokens(str(rel)))
    if abs(h["spacing"] - spacing) > 1e-9 or not _covers(h, rect) or h.get("row0", "south") != "south":
        return None
    score = 0.0
    if role == "collision":
        if not ({"collision", "col", "c"} & toks):
            return None
        score += 10
    else:
        if {"collision", "col", "c"} & toks or GRID_EXCLUDE & toks or not (GRID_GOOD & toks):
            return None          # a component (B, fBm, craters, ...) or no height-like name: not H
        score += 5
        if role == "core_H":
            score += 2 * ("core" in toks) + 1 * ("h0" in toks or "nofillet" in toks)
    return score


def locate_inputs(cfg: Config, build_dir: str | Path | None = None, world_dir: str | Path | None = None,
                  overrides: dict | None = None) -> dict:
    """Find the build products the checks need. Missing ones are reported as None.

    Keys: core_grid, collision_grid, ring_grid, core_tile_grids, core_base_grid, crater_catalogs,
    rock_catalog, rock_library, meshes_dir, sdf_files, world_info, candidates (the alternatives
    considered).
    """
    lay = Layout(cfg)
    build = Path(build_dir) if build_dir else lay.build_dir
    world = Path(world_dir) if world_dir else lay.world_dir
    ov = dict(_cfg_get(cfg, "validation.inputs", {}) or {})
    ov.update(overrides or {})
    core = lay.zone_rect("core")
    ring = lay.zone_rect("ring")
    found: dict[str, Any] = {"build_dir": str(build), "world_dir": str(world), "candidates": {}}
    cands = _grid_candidates(build)
    roles = {"core_grid": ("core_H", float(cfg.zones.core.spacing_m), core),
             "collision_grid": ("collision", float(_cfg_get(cfg, "mesh.collision.spacing_m", 0.02)), core),
             "ring_grid": ("ring_H", float(cfg.zones.ring.spacing_m), ring)}
    for key, (role, spacing, rect) in roles.items():
        scored = []
        for prefix, h in cands:
            s = _score_grid(prefix, h, role, spacing, rect, build)
            if s is not None:
                scored.append((s, prefix.with_suffix(".npy").stat().st_mtime, prefix))
        scored.sort(key=lambda t: (t[0], t[1]), reverse=True)
        found["candidates"][key] = [str(p) for _, _, p in scored[:8]]
        found[key] = str(scored[0][2]) if scored and scored[0][0] >= 0 else None
    # per-tile core grids (used when no single core grid exists). The terrain stage's H0 tiles
    # (``terrain/core_tiles``, no rocks, no fillets: what the roughness acceptance measures) win over
    # other per-tile grids such as the integration build's final tiles (``heights/``, with fillets).
    tiles, h0_tiles = {}, {}
    for prefix, h in cands:
        m = re.search(r"tile_(\d+)_(\d+)", prefix.name)
        if not m or abs(h["spacing"] - float(cfg.zones.core.spacing_m)) > 1e-9:
            continue
        toks = set(_tokens(prefix.name))
        if {"collision", "col", "c"} & toks or GRID_EXCLUDE & toks:
            continue
        try:
            rel = str(prefix.relative_to(build))
        except ValueError:
            rel = prefix.name
        rel_toks = set(_tokens(rel))
        if {"collision", "col", "c"} & rel_toks or GRID_EXCLUDE & rel_toks:
            continue   # e.g. fbm/tile_ii_jj (component fields cached per tile)
        t = TileId(int(m.group(1)), int(m.group(2)))
        if _covers(h, lay.tile_rect(t)):
            (h0_tiles if "core_tiles" in Path(rel).parts else tiles)[t.name] = str(prefix)
    found["core_tile_grids"] = h0_tiles or tiles or None
    # the band-limited DTM B of the core (terrain/base/core, 1 cm over the core + margin): the
    # site-aware roughness acceptance compares H0 with B (DESIGN 13 decision after phase 2)
    found["core_base_grid"] = None
    for prefix, h in cands:
        try:
            rel_parts = prefix.relative_to(build).parts
        except ValueError:
            continue
        if (rel_parts[-2:] == ("base", "core") and abs(h["spacing"] - float(cfg.zones.core.spacing_m)) <= 1e-9
                and _covers(h, core)):
            found["core_base_grid"] = str(prefix)
            break
    # catalogs
    craters, rocks = [], []
    for root in (build, world):
        if not root.exists():
            continue
        for npz in root.rglob("*.npz"):
            try:
                with np.load(npz, allow_pickle=False) as d:
                    keys = set(d.files)
            except Exception:  # noqa: BLE001
                continue
            if {"x", "y", "D"} <= keys and ({"cls", "proto"} & keys or "rock" in npz.name.lower()):
                rocks.append(npz)
            elif {"x", "y", "D"} <= keys and ({"rho", "d", "h"} & keys or "crater" in npz.name.lower()):
                craters.append(npz)
    found["crater_catalogs"] = sorted({str(p) for p in craters}) or None
    rock_named = [p for p in rocks if p.name == "rocks_catalog.npz"] or rocks
    rock_named.sort(key=lambda p: (str(p).startswith(str(world)), p.stat().st_mtime), reverse=True)
    found["rock_catalog"] = str(rock_named[0]) if rock_named else None
    found["candidates"]["rock_catalog"] = [str(p) for p in rock_named[:5]]
    lib = Path(cfg.paths.rock_library) / "library.json" if "rock_library" in cfg.paths else None
    found["rock_library"] = str(lib) if lib and lib.exists() else None
    meshes = world / "meshes"
    found["meshes_dir"] = str(meshes) if meshes.exists() else None
    found["sdf_files"] = sorted(str(p) for p in world.glob("*.sdf")) if world.exists() else []
    wi = world / "world_info.yaml"
    found["world_info"] = str(wi) if wi.exists() else None
    for k, v in ov.items():
        found[k] = v
        found.setdefault("overridden", []).append(k)
    return found


def load_core_grid(inputs: dict, cfg: Config) -> tuple[Grid | None, str]:
    """Core height grid cropped to the core rectangle (memmap where possible)."""
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    if inputs.get("core_grid"):
        g = Grid.load(inputs["core_grid"], mmap=True)
        return g.crop(core.x0, core.x1, core.y0, core.y1) if not _same_rect(g, core) else g, inputs["core_grid"]
    tiles = inputs.get("core_tile_grids")
    if tiles:
        s = float(cfg.zones.core.spacing_m)
        n = int(round(core.width / s)) + 1
        z = np.full((n, n), np.nan, dtype=np.float32)
        for name, prefix in tiles.items():
            g = Grid.load(prefix, mmap=True)
            i0 = int(round((max(g.x0, core.x0) - core.x0) / s))
            j0 = int(round((max(g.y0, core.y0) - core.y0) / s))
            sub = g.crop(max(g.x0, core.x0), min(g.x1, core.x1), max(g.y0, core.y0), min(g.y1, core.y1))
            z[j0:j0 + sub.ny, i0:i0 + sub.nx] = sub.z
        return Grid(z, core.x0, core.y0, s, {"assembled_from_tiles": len(tiles)}), f"{len(tiles)} tile grids"
    return None, "no core height grid found"


def _same_rect(g: Grid, r: Rect, tol: float = 1e-6) -> bool:
    return abs(g.x0 - r.x0) < tol and abs(g.x1 - r.x1) < tol and abs(g.y0 - r.y0) < tol and abs(g.y1 - r.y1) < tol


def load_catalog(path: str | Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as d:
        return {k: np.asarray(d[k]) for k in d.files}


def load_crater_catalogs(paths: Sequence[str | Path]) -> dict[str, np.ndarray]:
    """Union of crater catalogs, de-duplicated on (x, y, D) rounded to 0.1 mm."""
    xs, ys, Ds, rhos = [], [], [], []
    for p in paths:
        c = load_catalog(p)
        xs.append(c["x"].astype(np.float64).ravel())
        ys.append(c["y"].astype(np.float64).ravel())
        Ds.append(c["D"].astype(np.float64).ravel())
        rhos.append(c["rho"].astype(np.float64).ravel() if "rho" in c else np.full(len(c["x"]), np.nan))
    if not xs:
        return {"x": np.zeros(0), "y": np.zeros(0), "D": np.zeros(0), "rho": np.zeros(0)}
    x, y, D, rho = map(np.concatenate, (xs, ys, Ds, rhos))
    key = np.stack([np.round(x * 1e4), np.round(y * 1e4), np.round(D * 1e4)], 1).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    idx = np.sort(idx)
    return {"x": x[idx], "y": y[idx], "D": D[idx], "rho": rho[idx], "n_raw": len(x)}


# ==============================================================================================
# check: roughness

#: site-aware roughness acceptance (DESIGN 13, decision after phase 2): the composite H0 is judged
#: against the target at the small lags, the synthetic part H0 - B (craters + fBm) at 1-3 m, and at
#: >= 10 m the composite may exceed the real band-limited DTM B alone by at most 15 %.
ROUGH_COMPOSITE_LAGS_M = (0.01, 0.03, 0.1, 0.3)
ROUGH_SYNTHETIC_LAGS_M = (1.0, 3.0)
ROUGH_DTM_MIN_LAG_M = 10.0
ROUGH_DTM_RATIO_MAX = 1.15


def load_core_base_grid(inputs: dict, cfg: Config) -> tuple[Grid | None, str]:
    """The core DTM base B on the core rectangle (same nodes as the core H grid), or None."""
    p = inputs.get("core_base_grid")
    if not p:
        return None, "no core DTM base grid (terrain/base/core) found"
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    g = Grid.load(p, mmap=True)
    return (g.crop(core.x0, core.x1, core.y0, core.y1) if not _same_rect(g, core) else g), str(p)


def check_roughness(cfg: Config, inputs: dict, *, stride: int = 1, psd: bool = True,
                    slope_baselines: Sequence[float] = (0.1, 1.0, 10.0)) -> dict:
    """Site-aware deviogram acceptance (DESIGN 13, decision after phase 2) on H0 without rocks:

    * composite H0 within +-tolerance of nu1 L^H at 0.01-0.3 m (``rows``, checked lags);
    * synthetic part H0 - B (craters + fBm) within +-tolerance at 1 m and 3 m (``rows_synthetic``);
    * at L >= 10 m the composite at most ``ROUGH_DTM_RATIO_MAX`` x the DTM base B alone (``rows``,
      ``ratio_to_B``); the real DTM governs there.

    Without the core B grid only the composite rule is evaluated (``site_aware`` false).
    """
    t0 = time.time()
    rough = cfg.terrain.roughness
    try:
        grid, src = load_core_grid(inputs, cfg)
    except Exception as e:  # noqa: BLE001
        return _result("error", reason=f"cannot load core grid: {e}")
    if grid is None:
        return _result("skipped", reason=src)
    nu1, H, tol = float(rough.nu1_m), float(rough.hurst), float(rough.tolerance_frac)
    nfin = int(np.isfinite(np.asarray(grid.z[:: max(1, grid.ny // 200), :: max(1, grid.nx // 200)])).mean() * 100)
    res = st.deviogram(grid, st.DEFAULT_LAGS_M, detrend="plane", stride=stride)
    rows = st.compare_deviogram(res, nu1, H, tol, ROUGH_COMPOSITE_LAGS_M)
    targets = {float(k): float(v) for k, v in (rough.get("targets_rms_slope_deg") or {}).items()}
    for r in rows:
        if r["L_m"] in targets:
            r["config_target_rms_slope_deg"] = targets[r["L_m"]]
        if r["checked"]:
            r["rule"] = f"composite within +-{tol:.0%} of the target"
    H_fit, nu1_fit = st.fit_hurst(res.lag_used_m, res.nu_m)
    site: dict[str, Any] = {"site_aware": False}
    rows_syn: list[dict] = []
    try:
        B, bsrc = load_core_base_grid(inputs, cfg)
    except Exception as e:  # noqa: BLE001
        B, bsrc = None, f"cannot load the core DTM base grid: {e}"
    if B is not None and B.z.shape == grid.z.shape:
        resB = st.deviogram(B, st.DEFAULT_LAGS_M, detrend="plane", stride=stride)
        syn = np.asarray(grid.z, dtype=np.float32) - np.asarray(B.z, dtype=np.float32)
        del B
        resS = st.deviogram(Grid(syn, grid.x0, grid.y0, grid.spacing), st.DEFAULT_LAGS_M, detrend="plane",
                            stride=stride)
        del syn
        rows_syn = st.compare_deviogram(resS, nu1, H, tol, ROUGH_SYNTHETIC_LAGS_M)
        for r in rows_syn:
            if r["checked"]:
                r["rule"] = f"synthetic part H0 - B within +-{tol:.0%} of the target"
        for r, nb in zip(rows, resB.nu_m):
            r["nu_B_m"] = nb
            r["ratio_to_B"] = r["nu_m"] / nb if nb and np.isfinite(nb) and nb > 0 else float("nan")
            if r["L_m"] >= ROUGH_DTM_MIN_LAG_M - 1e-9:
                r["checked"] = True
                r["pass"] = bool(np.isfinite(r["ratio_to_B"]) and r["ratio_to_B"] <= ROUGH_DTM_RATIO_MAX)
                r["rule"] = f"composite <= {ROUGH_DTM_RATIO_MAX} x the DTM base B alone"
        site = {"site_aware": True, "base_source": bsrc, "deviogram_B": resB.to_dict(),
                "deviogram_synthetic": resS.to_dict()}
    else:
        site["reason"] = bsrc if B is None else f"base grid shape {B.z.shape} != core grid {grid.z.shape}"
    checked = [r for r in rows + rows_syn if r.get("checked")]
    out = _result("pass" if checked and all(r.get("pass") for r in checked) else "fail",
                  source=src, grid={"x0": grid.x0, "y0": grid.y0, "spacing": grid.spacing, "nx": grid.nx,
                                    "ny": grid.ny, "finite_pct_sampled": nfin},
                  definition="plane-detrended (whole core) directional deviogram, pooled x+y forward "
                             "differences; nu(L) = sqrt(mean(d^2)); RMS slope = atan(nu/L) "
                             "(matches science fbm_check.py / detrended_compare.py)",
                  acceptance=(f"site-aware (DESIGN 13): composite H0 within +-{tol:.0%} at "
                              f"{', '.join(f'{L:g}' for L in ROUGH_COMPOSITE_LAGS_M)} m; H0 - B within "
                              f"+-{tol:.0%} at {', '.join(f'{L:g}' for L in ROUGH_SYNTHETIC_LAGS_M)} m; composite "
                              f"<= {ROUGH_DTM_RATIO_MAX} x B at >= {ROUGH_DTM_MIN_LAG_M:g} m"),
                  target={"nu1_m": nu1, "hurst": H, "tolerance_frac": tol,
                          "composite_lags_m": list(ROUGH_COMPOSITE_LAGS_M),
                          "synthetic_lags_m": list(ROUGH_SYNTHETIC_LAGS_M),
                          "dtm_min_lag_m": ROUGH_DTM_MIN_LAG_M, "dtm_ratio_max": ROUGH_DTM_RATIO_MAX},
                  rows=rows, rows_synthetic=rows_syn, fit={"H": H_fit, "nu1_m": nu1_fit},
                  deviogram=res.to_dict(), **site)
    if psd:
        try:
            p = st.radial_psd(grid, segment=min(4096, grid.nx, grid.ny), max_segments=9)
            out["psd"] = {**p, "fit_0p1_20": st.fit_power_law(p["f"], p["S"], 0.1, 20.0),
                          "target_S0_m4": float(rough.psd_S0_m4) * float(rough.get("fbm_gain", 1.0)),
                          "target_beta": -(2 + 2 * H)}
        except Exception as e:  # noqa: BLE001
            out["psd"] = {"error": str(e)}
    out["slope_distribution"] = []
    for L in slope_baselines:
        try:
            out["slope_distribution"].append(st.slope_distribution(grid, L))
        except Exception as e:  # noqa: BLE001
            out["slope_distribution"].append({"baseline_m": L, "error": str(e)})
    out["seconds"] = round(time.time() - t0, 2)
    return out


# ==============================================================================================
# check: crater counts

def check_craters(cfg: Config, inputs: dict, D_values: Sequence[float] = (0.2, 0.5, 1.0, 2.0),
                  frac_tol: float = 0.20) -> dict:
    paths = inputs.get("crater_catalogs")
    if not paths:
        return _result("skipped", reason="no crater catalog (*.npz with x, y, D) in the build dir")
    try:
        cat = load_crater_catalogs(paths)
    except Exception as e:  # noqa: BLE001
        return _result("error", reason=f"cannot read crater catalogs: {e}")
    cr = cfg.terrain.craters
    core = Layout(cfg).zone_rect("core")
    inside = core.contains(cat["x"], cat["y"])
    sizes = cat["D"][inside]
    C, b = float(cr.sfd_C_per_m2), float(cr.sfd_b)
    D_min = float(_cfg_get(cfg, "terrain.craters.D_min_m.core", 0.2))
    D_max = float(cr.D_max_m)
    fade = tuple(float(v) for v in cr.get("fade_D_m", [8.0, 16.0])) if cr.get("fade_D_m") else None
    area = core.width * core.height

    def model(D):
        return st.crater_expected_N(D, C, b, D_min, D_max, fade)

    rows = st.sfd_rows(sizes, area, D_values, model, frac_tol=frac_tol)
    allD = np.array([0.2, 0.3, 0.5, 0.7, 1, 1.5, 2, 3, 5, 8, 10, 16])
    curve = {"D_m": allD.tolist(), "observed": st.cumulative_counts(sizes, allD).tolist(),
             "expected": (model(allD) * area).tolist()}
    fresh = None
    if np.isfinite(cat["rho"]).any():
        rho = cat["rho"][inside]
        fresh = {"rho_gt_0p73": float(np.mean(rho > 0.73)), "rho_0p4_0p73": float(np.mean((rho >= 0.4) & (rho <= 0.73))),
                 "rho_lt_0p4": float(np.mean(rho < 0.4)), "target": [0.10, 0.20, 0.70]}
    return _result("pass" if all(r["pass"] for r in rows) else "fail", sources=paths, n_catalog=int(len(cat["D"])),
                   n_raw=int(cat.get("n_raw", len(cat["D"]))), n_in_core=int(inside.sum()), area_m2=area,
                   model={"C_per_m2": C, "b": b, "D_min_m": D_min, "D_max_m": D_max, "fade_D_m": fade},
                   rows=rows, curve=curve, slope_fit=st.fit_cumulative_slope(sizes, D_min, min(D_max, fade[0]) if fade else D_max),
                   freshness_mix=fresh)


# ==============================================================================================
# check: rock counts

def _class_index(cat: dict, cfg: Config) -> np.ndarray | None:
    cls = cat.get("cls")
    if cls is None:
        return None
    if cls.dtype.kind in "iu":
        return cls.astype(np.int64)
    names = [c["name"] for c in cfg.rocks.classes]
    return np.array([names.index(str(c)) if str(c) in names else -1 for c in cls])


def check_rocks(cfg: Config, inputs: dict, D_core: Sequence[float] = (0.02, 0.05, 0.10, 0.20),
                D_ring: Sequence[float] = (0.30, 0.50, 1.0), sigma: float = 3.0) -> dict:
    path = inputs.get("rock_catalog")
    if not path:
        return _result("skipped", reason="no rocks_catalog.npz in the build/world dir")
    try:
        cat = load_catalog(path)
    except Exception as e:  # noqa: BLE001
        return _result("error", reason=f"cannot read {path}: {e}")
    lay = Layout(cfg)
    sfd = cfg.rocks.sfd
    A, segs = float(sfd.A_ge_1m_per_m2), [list(s) for s in sfd.segments]
    D_max = float(cfg.rocks.D_max_m)

    def model(D):
        return np.maximum(st.broken_power_law_N(D, A, segs) - st.broken_power_law_N(D_max, A, segs), 0.0)

    required = ["id", "zone", "tile_i", "tile_j", "proto", "cls", "lod", "x", "y", "z", "qw", "qx", "qy", "qz",
                "scale", "D", "exposed_h", "burial", "collide"]
    missing = [k for k in required if k not in cat]
    zone = cat["zone"].astype(np.int64) if "zone" in cat else np.zeros(len(cat["D"]), np.int64)
    D = cat["D"].astype(np.float64)
    x, y = cat["x"].astype(np.float64), cat["y"].astype(np.float64)
    core = lay.zone_rect("core")
    ring = lay.zone_rect("ring")
    a_core = core.width * core.height
    a_ring = ring.width * ring.height - a_core
    in_core = zone == 0
    d_min_core = float(_cfg_get(cfg, "rocks.D_min_m.core", 0.02))
    d_min_ring = float(_cfg_get(cfg, "rocks.D_min_m.ring", 0.30))
    # check sizes below the zone's sampled minimum (e.g. the fast preset's 3 cm) are moved up to it
    D_core = sorted({max(float(d), d_min_core) for d in D_core})
    D_ring = sorted({max(float(d), d_min_ring) for d in D_ring})
    rows_core = st.sfd_rows(D[in_core], a_core, D_core, model, sigma=sigma)
    rows_ring = st.sfd_rows(D[zone == 1], a_ring, D_ring, model, sigma=sigma) if (zone == 1).any() else []
    # sanity of the catalog itself
    sanity: dict[str, Any] = {"missing_fields": missing, "n_total": int(len(D)), "n_core": int(in_core.sum()),
                              "n_ring": int((zone == 1).sum())}
    sanity["nonfinite_xyzD"] = int((~np.isfinite(np.stack([x, y, cat["z"].astype(np.float64) if "z" in cat else x, D]))).sum())
    sanity["core_outside_rect"] = int((in_core & ~core.contains(x, y)).sum())
    sanity["core_below_D_min"] = int((in_core & (D < d_min_core - 1e-9)).sum())
    ring_small = (zone == 1) & (D < d_min_ring - 1e-9)
    # core-edge band (rocks.population, FID-04): rocks below the ring's D_min continue the core
    # population a few metres into the ring on purpose; only ring rocks outside that band are wrong
    band: dict = {"enabled": False, "width_m": 0.0}
    try:
        from ..rocks import population as _pop

        band.update(dict(getattr(_pop, "EDGE_BAND", {})))
    except Exception:  # noqa: BLE001 - the band is optional
        pass
    cfg_band = _cfg_get(cfg, "rocks.edge_band", None)
    if isinstance(cfg_band, dict):
        band.update(cfg_band)
    if band.get("enabled") and float(band.get("width_m") or 0.0) > 0:
        half = 0.5 * core.width
        dist = np.maximum(np.abs(x), np.abs(y)) - half              # distance outside the core square
        in_band = dist <= float(band["width_m"]) + 1e-6
        sanity["ring_edge_band_below_D_min"] = int((ring_small & in_band).sum())
        ring_small &= ~in_band
    sanity["ring_below_D_min"] = int(ring_small.sum())
    sanity["above_D_max"] = int((D > D_max + 1e-9).sum())
    if {"qw", "qx", "qy", "qz"} <= set(cat):
        qn = np.sqrt(sum(cat[k].astype(np.float64) ** 2 for k in ("qw", "qx", "qy", "qz")))
        sanity["quat_norm_max_dev"] = float(np.abs(qn - 1).max()) if len(qn) else 0.0
    if {"tile_i", "tile_j"} <= set(cat) and in_core.any():
        T = lay.tile_size
        ti = np.clip(np.floor((x - core.x0) / T), 0, lay.n_tiles - 1).astype(np.int64)
        tj = np.clip(np.floor((y - core.y0) / T), 0, lay.n_tiles - 1).astype(np.int64)
        bad = in_core & ((ti != cat["tile_i"].astype(np.int64)) | (tj != cat["tile_j"].astype(np.int64)))
        # rocks exactly on a tile line may legitimately go either way
        on_line = (np.abs((x - core.x0) / T - np.round((x - core.x0) / T)) < 1e-9) | (
            np.abs((y - core.y0) / T - np.round((y - core.y0) / T)) < 1e-9)
        sanity["tile_assignment_mismatch"] = int((bad & ~on_line).sum())
    if "collide" in cat:
        cmin = float(cfg.rocks.collision_min_D_m)
        expect = in_core & (D >= cmin)
        sanity["collide_flag_mismatch"] = int((cat["collide"].astype(bool) != expect).sum())
    if "id" in cat:
        sanity["duplicate_ids"] = int(len(cat["id"]) - len(np.unique(cat["id"])))
    if "zone" in cat:
        sanity["bad_zone"] = int((~np.isin(zone, [0, 1])).sum())
    lib_json = inputs.get("rock_library")
    if lib_json and Path(lib_json).is_file() and {"proto", "lod"} <= set(cat):
        try:
            protos = json.loads(Path(lib_json).read_text())["prototypes"]
            n_lods = np.array([len(p.get("lods", p.get("lod_tris", []))) for p in protos])
            pr = cat["proto"].astype(np.int64)
            bad_p = (pr < 0) | (pr >= len(protos))
            sanity["proto_out_of_range"] = int(bad_p.sum())
            lod = cat["lod"].astype(np.int64)
            sanity["lod_out_of_range"] = int(((lod < 0) | (~bad_p & (lod >= n_lods[np.clip(pr, 0, len(protos) - 1)]))).sum())
        except (OSError, ValueError, KeyError) as e:
            sanity["library_read_error"] = str(e)[:200]
    # CR-03: every computed sanity figure is judged (before: only missing / non-finite / rect / tile / D_max)
    judged = {"nonfinite_xyzD": 0, "core_outside_rect": 0, "tile_assignment_mismatch": 0, "above_D_max": 0,
              "collide_flag_mismatch": 0, "core_below_D_min": 0, "ring_below_D_min": 0, "duplicate_ids": 0,
              "bad_zone": 0, "proto_out_of_range": 0, "lod_out_of_range": 0}
    problems = [k for k, lim in judged.items() if sanity.get(k, 0) > lim]
    if sanity.get("quat_norm_max_dev", 0.0) > 1e-6:
        problems.append("quat_norm_max_dev")
    if missing:
        problems.append("missing_fields")
    sanity["problems"] = problems
    sanity_ok = not problems
    counts_ok = all(r["pass"] for r in rows_core)
    allD = np.array([0.02, 0.03, 0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0])
    curve = {"D_m": allD.tolist(), "observed_core": st.cumulative_counts(D[in_core], allD).tolist(),
             "expected_core": (model(allD) * a_core).tolist()}
    cls_idx = _class_index(cat, cfg)
    per_class = None
    if cls_idx is not None:
        per_class = {c["name"]: int(((cls_idx == k) & in_core).sum()) for k, c in enumerate(cfg.rocks.classes)}
    return _result("pass" if (counts_ok and sanity_ok) else "fail", source=path,
                   model={"A_ge_1m_per_m2": A, "segments": segs, "D_max_m": D_max,
                          "note": "expected = (N(>=D) - N(>=D_max)) * area; Poisson +-3 sigma acceptance"},
                   area_core_m2=a_core, area_ring_m2=a_ring, rows_core=rows_core, rows_ring=rows_ring,
                   ring_rows_informational=True, sanity=sanity, curve=curve, per_class_core=per_class,
                   slope_fit_core=st.fit_cumulative_slope(D[in_core], d_min_core, 1.0))


# ==============================================================================================
# check: visual vs collision

def _rock_free_mask(px: np.ndarray, py: np.ndarray, cat: dict | None, D_min: float, margin: float) -> np.ndarray:
    """True where (px, py) is farther than D + margin from every rock with D >= D_min."""
    keep = np.ones(len(px), dtype=bool)
    if cat is None or len(px) == 0:
        return keep
    D = cat["D"].astype(np.float64)
    sel = D >= D_min
    if "zone" in cat:
        sel &= cat["zone"].astype(np.int64) == 0
    if not sel.any():
        return keep
    from scipy.spatial import cKDTree
    tree = cKDTree(np.stack([px, py], 1))
    rx, ry, rD = cat["x"][sel].astype(np.float64), cat["y"][sel].astype(np.float64), D[sel]
    hits = tree.query_ball_point(np.stack([rx, ry], 1), r=rD + margin)
    for h in hits:
        if h:
            keep[h] = False
    return keep


def dz_stats(dz: np.ndarray, target_median: float, median_tol: float = 0.003, rms_max: float = 0.010,
             p99_max: float = 0.030) -> dict:
    dz = np.asarray(dz, dtype=np.float64)
    dz = dz[np.isfinite(dz)]
    if len(dz) == 0:
        return {"n": 0}
    med = float(np.median(dz))
    dev = dz - med
    rms = float(np.sqrt(np.mean(dev ** 2)))
    p99 = float(np.percentile(np.abs(dev), 99))
    edges = np.arange(-0.100, 0.0605, 0.001)
    hist = np.histogram(np.clip(dz, edges[0], edges[-1] - 1e-9), bins=edges)[0]
    out = {"n": int(len(dz)), "median_m": med, "mean_m": float(dz.mean()), "std_m": float(dz.std()),
           "rms_about_median_m": rms, "p99_abs_dev_m": p99, "max_abs_dev_m": float(np.abs(dev).max()),
           "p1_m": float(np.percentile(dz, 1)), "p99_m": float(np.percentile(dz, 99)),
           "min_m": float(dz.min()), "max_m": float(dz.max()),
           "thresholds": {"median_target_m": target_median, "median_tol_m": median_tol,
                          "rms_about_median_max_m": rms_max, "p99_abs_dev_max_m": p99_max},
           "hist": {"edges_m": edges.round(4).tolist(), "counts": hist.tolist()}}
    out["pass_median"] = abs(med - target_median) <= median_tol + 1e-12
    out["pass_rms"] = rms <= rms_max
    out["pass_p99"] = p99 <= p99_max
    out["pass"] = bool(out["pass_median"] and out["pass_rms"] and out["pass_p99"])
    return out


def check_visual_vs_collision(cfg: Config, inputs: dict, n_samples: int = 200_000,
                              edge_margin_m: float = 0.05, rock_margin_m: float = 0.15) -> dict:
    """dz = collision height - visual height at random rock-free core points.

    Source preference: the GLB meshes (tile ground TINs vs the per-tile collision TINs
    ``tile_ii_jj_collision.glb`` of DESIGN 7, or a single ``core_collision.glb``), which is what
    Gazebo uses; fallback: the core visual and collision height grids.
    """
    t0 = time.time()
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    gen = rng(cfg, "validation/visual_vs_collision")
    px = gen.uniform(core.x0 + edge_margin_m, core.x1 - edge_margin_m, n_samples)
    py = gen.uniform(core.y0 + edge_margin_m, core.y1 - edge_margin_m, n_samples)
    cat = None
    if inputs.get("rock_catalog"):
        try:
            cat = load_catalog(inputs["rock_catalog"])
        except Exception:  # noqa: BLE001
            cat = None
    dmin = float(_cfg_get(cfg, "rocks.collision_min_D_m", 0.05))
    keep = _rock_free_mask(px, py, cat, dmin, rock_margin_m)
    px, py = px[keep], py[keep]
    target = float(_cfg_get(cfg, "mesh.collision.ground_offset_m", -0.02))
    meshes = Path(inputs["meshes_dir"]) if inputs.get("meshes_dir") else None
    col_glb = meshes / "core_collision.glb" if meshes else None
    per_tile_col = bool(meshes) and any(meshes.glob("tile_*_collision.glb"))
    source, zv, zc = None, None, None
    if meshes and (per_tile_col or (col_glb and col_glb.exists())) and any(meshes.glob("tile_*_ground.glb")):
        try:
            zv = np.full(len(px), np.nan)
            zc = np.full(len(px), np.nan)
            T = lay.tile_size
            ti = np.clip(np.floor((px - core.x0) / T), 0, lay.n_tiles - 1).astype(np.int64)
            tj = np.clip(np.floor((py - core.y0) / T), 0, lay.n_tiles - 1).astype(np.int64)
            missing_tiles = []
            for t in lay.tiles():
                m = (ti == t.i) & (tj == t.j)
                if not m.any():
                    continue
                kinds = ("ground", "collision") if per_tile_col else ("ground",)
                for kind, dest in zip(kinds, (zv, zc)):
                    f = meshes / lay.tile_mesh_name(t, kind)
                    if not f.exists():
                        missing_tiles.append(f.name)
                        continue
                    g = read_glb(f)
                    dest[m] = TinLocator(g.positions[:, :2], g.triangles).interpolate(g.positions[:, 2],
                                                                                       px[m], py[m])
            if per_tile_col:
                col_src = "tile_*_collision.glb"
            else:
                g = read_glb(col_glb)
                zc = TinLocator(g.positions[:, :2], g.triangles).interpolate(g.positions[:, 2], px, py)
                col_src = str(col_glb)
            source = {"kind": "meshes", "visual": "tile_*_ground.glb", "collision": col_src,
                      "missing_tiles": missing_tiles}
        except Exception as e:  # noqa: BLE001
            source = {"kind": "meshes_failed", "error": str(e)}
            zv = zc = None
    if zv is None:
        if inputs.get("core_grid") and inputs.get("collision_grid"):
            Hg = Grid.load(inputs["core_grid"], mmap=True)
            Cg = Grid.load(inputs["collision_grid"], mmap=True)
            zv = Hg.sample(px, py, order=1)
            zc = Cg.sample(px, py, order=1)
            source = {"kind": "grids", "visual": inputs["core_grid"], "collision": inputs["collision_grid"],
                      "note": "mesh files not available; grid-vs-grid (excludes TIN errors)"}
        else:
            return _result("skipped", reason="need tile ground GLBs + tile collision GLBs (or core_collision.glb), "
                                             "or the core visual and collision height grids", previous=source)
    dz = zc - zv
    s = dz_stats(dz, target)
    if s.get("n", 0) == 0:
        return _result("error", reason="no valid dz samples", source=source)
    return _result("pass" if s["pass"] else "fail", source=source, n_requested=n_samples,
                   n_rock_free=int(keep.sum()), definition="dz = z_collision - z_visual at uniform random core "
                   f"points farther than D + {rock_margin_m} m from rocks with D >= {dmin} m", **s,
                   seconds=round(time.time() - t0, 2))


# ==============================================================================================
# checks: GLB files, SDF URIs

_TILE_RE = re.compile(r"^tile_(\d+)_(\d+)_(ground|rocks|collision)\.glb$")
_STRIP_RE = re.compile(r"^(ring|far|horizon)_(south|north|west|east)_(ground|rocks|collision)\.glb$")


def classify_mesh(name: str, lay: Layout) -> tuple[str, Rect | None]:
    """(kind, expected rectangle) of a mesh file name; kind e.g. 'core_ground', 'ring_rocks'."""
    m = _TILE_RE.match(name)
    if m:
        return f"core_{m.group(3)}", lay.tile_rect(TileId(int(m.group(1)), int(m.group(2))))
    m = _STRIP_RE.match(name)
    if m:
        try:
            return f"{m.group(1)}_{m.group(3)}", lay.strip_rects(m.group(1))[m.group(2)]
        except KeyError:
            return f"{m.group(1)}_{m.group(3)}", None
    if name == "core_collision.glb":
        return "core_collision", lay.zone_rect("core")
    return "other", None


def _edge_list(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Unique undirected edges (k, 2) and how many triangles use each."""
    tris = np.asarray(tris, dtype=np.int64)
    if len(tris) == 0:
        return np.zeros((0, 2), np.int64), np.zeros(0, np.int64)
    e = np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]])
    e.sort(axis=1)
    nv = int(e.max()) + 1
    key = e[:, 0] * nv + e[:, 1]
    uk, counts = np.unique(key, return_counts=True)
    return np.stack([uk // nv, uk % nv], axis=1), counts


def weld_positions(pos32: np.ndarray, tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Merge bit-identical vertex positions (seam-split vertices) -> (unique positions, remapped tris)."""
    if len(pos32) == 0:
        return pos32.astype(np.float64), tris
    v = np.ascontiguousarray(pos32, dtype=np.float32)
    key = v.view(np.dtype((np.void, v.dtype.itemsize * 3))).ravel()
    _, first, inv = np.unique(key, return_index=True, return_inverse=True)
    return v[first].astype(np.float64), np.asarray(inv).reshape(-1)[tris]


def _perimeter_tol(r: Rect) -> float:
    return 1e-5 + 2.4e-7 * max(abs(r.x0), abs(r.x1), abs(r.y0), abs(r.y1))


def mesh_checks(g: GlbData, kind: str, rect: Rect | None, roughness_expected: float | None = None) -> dict:
    """Per-file geometric/format checks (see ``check_glbs``)."""
    info = g.info
    rec: dict[str, Any] = {"n_vertices": g.n_vertices, "n_triangles": g.n_triangles,
                           "n_primitives": info["n_primitives"], "n_meshes": info["n_meshes"],
                           "identity_transforms": info["identity_transforms"], "n_images": info["n_images"],
                           "n_textures": info["n_textures"], "materials": info["materials"]}
    prims = info["primitives"]
    rec["position_float32"] = all(p.get("position_type") == "float32" for p in prims)
    rec["indices_uint32"] = all(p.get("index_type") == "uint32" for p in prims)
    rec["has_normals"] = g.normals is not None
    rec["has_uvs"] = g.uvs is not None
    mats = info["materials"]
    rec["material_ok"] = (len(mats) == 1 and mats[0]["metallic_explicit"] and mats[0]["metallicFactor"] == 0)
    if roughness_expected is not None and len(mats) == 1:
        rf = mats[0]["roughnessFactor"]
        rec["roughness_matches_config"] = rf is not None and abs(float(rf) - roughness_expected) < 1e-6
    if g.n_vertices:
        lo, hi = g.positions.min(0), g.positions.max(0)
        rec["bounds"] = [lo.tolist(), hi.tolist()]
        rec["finite"] = bool(np.isfinite(g.positions).all())
        zr = float(hi[2] - lo[2])
        rec["z_range_m"] = zr
        ok, why = True, []
        if rect is not None:
            tol = _perimeter_tol(rect)
            if kind.endswith(("ground", "collision")):
                d = [abs(lo[0] - rect.x0), abs(hi[0] - rect.x1), abs(lo[1] - rect.y0), abs(hi[1] - rect.y1)]
                if max(d) > tol:
                    ok = False
                    why.append(f"xy bounds differ from the piece rectangle by up to {max(d):.6g} m")
            else:  # rocks overhang their tile by up to D_max/2
                pad = 1.6
                if (lo[0] < rect.x0 - pad or hi[0] > rect.x1 + pad or lo[1] < rect.y0 - pad or hi[1] > rect.y1 + pad):
                    ok = False
                    why.append("rocks extend more than 1.6 m outside their tile")
            span = min(rect.width, rect.height)
            if kind.endswith(("ground", "collision")) and zr > 0.5 * span + 5.0:
                ok = False
                why.append(f"z range {zr:.2f} m implausible for a {span:.0f} m piece (Y-up export?)")
        if abs(lo[2]) > 5000 or abs(hi[2]) > 5000:
            ok = False
            why.append("|z| > 5 km")
        rec["bounds_ok"] = ok and rec["finite"]
        rec["bounds_problems"] = why
    if kind.endswith(("ground", "collision")) and g.n_triangles:
        v = g.positions
        t = g.triangles
        n = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
        area2 = np.linalg.norm(n, axis=1)
        nondeg = area2 > 1e-12
        rec["degenerate_faces"] = int((~nondeg).sum())
        rec["up_faces_frac"] = float((n[nondeg, 2] > 0).mean()) if nondeg.any() else None
        if g.normals is not None:
            nz = g.normals[:, 2]
            rec["up_normals_frac"] = float((nz > 0).mean())
            rec["normal_len_max_dev"] = float(np.abs(np.linalg.norm(g.normals, axis=1) - 1).max())
        # cracks: boundary edges that are not on the piece perimeter; non-manifold edges
        wp, wt = weld_positions(g.positions_f32, t)
        edges, counts = _edge_list(wt)
        rec["nonmanifold_edges"] = int((counts > 2).sum())
        if rect is not None:
            b = edges[counts == 1]
            tol = _perimeter_tol(rect)
            a, c = wp[b[:, 0]], wp[b[:, 1]]

            def on(side_val, coord, p):
                return np.abs(p[:, coord] - side_val) <= tol

            same_side = ((on(rect.x0, 0, a) & on(rect.x0, 0, c)) | (on(rect.x1, 0, a) & on(rect.x1, 0, c))
                         | (on(rect.y0, 1, a) & on(rect.y0, 1, c)) | (on(rect.y1, 1, a) & on(rect.y1, 1, c)))
            rec["boundary_edges"] = int(len(b))
            rec["interior_boundary_edges"] = int((~same_side).sum())
            if (~same_side).any():
                k = np.flatnonzero(~same_side)[:5]
                rec["interior_boundary_examples"] = [[a[i].round(4).tolist(), c[i].round(4).tolist()] for i in k]
    return rec


def sdf_uris(sdf_path: str | Path) -> dict:
    """Mesh/texture/engine references of an SDF and whether each resolves."""
    sdf_path = Path(sdf_path)
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    refs = []
    tags = {"uri", "albedo_map", "normal_map", "roughness_map", "metalness_map", "emissive_map", "light_map",
            "ambient_occlusion_map", "environment_map"}
    for el in root.iter():
        if el.tag in tags and el.text and el.text.strip():
            refs.append((el.tag, el.text.strip()))
    for eng in root.iter("engine"):
        fn = eng.findtext("filename")
        if fn and "/" in fn:
            refs.append(("engine", fn.strip()))
    rows = []
    for tag, uri in refs:
        if uri.startswith(("http://", "https://", "model://")):
            rows.append({"tag": tag, "uri": uri, "resolved": None, "exists": None})
            continue
        p = uri[7:] if uri.startswith("file://") else uri
        pp = Path(p) if Path(p).is_absolute() else (sdf_path.parent / p)
        rows.append({"tag": tag, "uri": uri, "resolved": str(pp.resolve()), "exists": pp.exists()})
    info: dict[str, Any] = {"world_name": world.get("name") if world is not None else None}
    if world is not None:
        info["plugins"] = [p.get("filename") for p in world.findall("plugin")]
        info["gravity"] = world.findtext("gravity")
        ph = world.find("physics")
        if ph is not None:
            info["max_step_size"] = ph.findtext("max_step_size")
            info["real_time_update_rate"] = ph.findtext("real_time_update_rate")
        sc = world.find("spherical_coordinates")
        if sc is not None:
            info["surface_model"] = sc.findtext("surface_model")
    return {"refs": rows, "info": info}


def stock_siblings(name: str) -> list[str]:
    """Stock-physics fallbacks of a world file name, preferred first (the launch file's rule):
    ``W.sdf`` -> ``W_stock.sdf``; ``W_inspect.sdf`` -> ``W_inspect_stock.sdf``, then ``W_stock.sdf``."""
    stem = name[:-4] if name.endswith(".sdf") else name
    if stem.endswith("_stock"):
        return []
    out = [f"{stem}_stock.sdf"]
    if stem.endswith("_inspect"):
        out.append(f"{stem[:-len('_inspect')]}_stock.sdf")
    return out


def check_sdf(cfg: Config, inputs: dict) -> dict:
    """Every mesh / texture / engine reference of every SDF must resolve.

    A missing PATCHED physics plugin (absolute engine path) is only a warning when a stock sibling
    variant of that world exists and is complete: the launch file falls back to it (CR-08a). Any
    other missing file fails the check."""
    sdfs = inputs.get("sdf_files") or []
    if not sdfs:
        return _result("skipped", reason="no *.sdf in the world dir")
    per: dict[str, dict] = {}
    for s in sdfs:
        try:
            r = sdf_uris(s)
        except Exception as e:  # noqa: BLE001
            per[Path(s).name] = {"error": str(e), "n_missing": None}
            continue
        missing = [x for x in r["refs"] if x["exists"] is False]
        unknown = [x for x in r["refs"] if x["exists"] is None]
        sensors = any(p and "sensors" in p for p in r["info"].get("plugins", []))
        per[Path(s).name] = {"n_refs": len(r["refs"]), "n_missing": len(missing), "missing": missing[:20],
                             "n_unresolvable_scheme": len(unknown), "info": r["info"],
                             "world_has_sensors_system": sensors,
                             "_missing_engine": [x for x in missing if x["tag"] == "engine"],
                             "_missing_other": [x for x in missing if x["tag"] != "engine"]}
    all_ok, warnings = True, []
    for name, rec in per.items():
        if "error" in rec:
            all_ok = False
            continue
        eng, other = rec.pop("_missing_engine"), rec.pop("_missing_other")
        if other:
            all_ok = False
        elif eng:
            fallback = next((sib for sib in stock_siblings(name) if per.get(sib, {}).get("n_missing") == 0), None)
            if fallback is None:
                all_ok = False
                rec["engine_missing_no_fallback"] = True
            else:
                rec["engine_missing_fallback"] = fallback
                warnings.append(f"{name}: physics plugin {eng[0]['uri']} missing; the launch file falls back to "
                                f"{fallback}")
    return _result("pass" if all_ok else "fail", sdfs=per, warnings=warnings,
                   note="world SDFs must not include Sensors/Imu/NavSat systems (the robot brings them); a missing "
                        "patched plugin is a warning when a complete stock sibling exists")


def check_glbs(cfg: Config, inputs: dict, *, with_trimesh: bool = True, up_min: float = 0.999) -> dict:
    t0 = time.time()
    meshes = inputs.get("meshes_dir")
    if not meshes:
        return _result("skipped", reason="no meshes dir in the world dir")
    lay = Layout(cfg)
    files = sorted(Path(meshes).glob("*.glb"))
    if not files:
        return _result("skipped", reason="meshes dir has no GLB")
    referenced = set()
    for s in inputs.get("sdf_files") or []:
        try:
            for r in sdf_uris(s)["refs"]:
                if r["resolved"] and r["resolved"].endswith(".glb"):
                    referenced.add(Path(r["resolved"]).name)
        except Exception:  # noqa: BLE001
            pass
    rough_ground = float(_cfg_get(cfg, "materials.regolith.roughness", 0.95))
    rough_rocks = float(_cfg_get(cfg, "materials.rocks.roughness", 0.85))
    per: dict[str, dict] = {}
    by_kind: dict[str, dict] = {}
    problems: list[str] = []
    for f in files:
        kind, rect = classify_mesh(f.name, lay)
        rec: dict[str, Any] = {"kind": kind, "size_bytes": f.stat().st_size, "referenced": f.name in referenced}
        try:
            g = read_glb(f)
            rexp = rough_ground if kind.endswith("ground") else (rough_rocks if kind.endswith("rocks") else None)
            rec.update(mesh_checks(g, kind, rect, rexp))
        except Exception as e:  # noqa: BLE001
            rec["read_error"] = f"{type(e).__name__}: {e}"[:300]
        if with_trimesh:
            rec["trimesh"] = trimesh_load_check(f)
        per[f.name] = rec
        k = by_kind.setdefault(kind, {"files": 0, "triangles": 0, "vertices": 0, "bytes": 0})
        k["files"] += 1
        k["triangles"] += rec.get("n_triangles", 0)
        k["vertices"] += rec.get("n_vertices", 0)
        k["bytes"] += rec["size_bytes"]
        # judgement
        visual = kind.endswith(("ground", "rocks"))
        groundish = kind.endswith(("ground", "collision"))
        if "read_error" in rec:
            problems.append(f"{f.name}: unreadable ({rec['read_error']})")
            continue
        if with_trimesh and rec["trimesh"].get("ok") is False:
            problems.append(f"{f.name}: trimesh load failed")
        if not rec.get("bounds_ok", True):
            problems.append(f"{f.name}: bounds {rec.get('bounds_problems')}")
        if not rec["position_float32"] or not rec["indices_uint32"]:
            problems.append(f"{f.name}: positions must be float32 and indices uint32")
        if not rec["identity_transforms"]:
            problems.append(f"{f.name}: non-identity node transform")
        if rec["n_primitives"] != 1:
            problems.append(f"{f.name}: {rec['n_primitives']} primitives (expected 1)")
        if visual and not rec["material_ok"]:
            problems.append(f"{f.name}: needs exactly one material with explicit metallicFactor 0")
        if visual and rec["n_images"]:
            problems.append(f"{f.name}: embedded images (textures belong in the SDF material)")
        if groundish:
            if rec.get("up_faces_frac") is not None and rec["up_faces_frac"] < up_min:
                problems.append(f"{f.name}: only {rec['up_faces_frac']:.5f} of faces CCW from +z")
            if kind.endswith("ground") and (rec.get("up_normals_frac") is None or rec["up_normals_frac"] < up_min):
                problems.append(f"{f.name}: +z vertex normals fraction {rec.get('up_normals_frac')}")
            if rec.get("interior_boundary_edges"):
                problems.append(f"{f.name}: {rec['interior_boundary_edges']} boundary edges inside the piece (cracks)")
    unref = sorted(n for n in per if referenced and not per[n]["referenced"])
    up_all = [r.get("up_normals_frac") for r in per.values() if r["kind"].endswith("ground") and r.get("up_normals_frac") is not None]
    nv_ground = sum(r.get("n_vertices", 0) for r in per.values() if r["kind"].endswith("ground"))
    up_vertices = sum(r.get("n_vertices", 0) * (r.get("up_normals_frac") or 0) for r in per.values()
                      if r["kind"].endswith("ground"))
    summary = {"files": len(per), "by_kind": by_kind,
               "total_triangles": sum(k["triangles"] for k in by_kind.values()),
               "total_bytes": sum(k["bytes"] for k in by_kind.values()),
               "ground_up_normals_frac_all": (up_vertices / nv_ground) if nv_ground else None,
               "ground_up_normals_frac_min_file": min(up_all) if up_all else None,
               "unreferenced_files": unref, "n_referenced": len(referenced),
               "trimesh_failures": sum(1 for r in per.values() if r.get("trimesh", {}).get("ok") is False)}
    return _result("fail" if problems else "pass", summary=summary, problems=problems[:200],
                   n_problems=len(problems), files=per, seconds=round(time.time() - t0, 2))


# ==============================================================================================
# check: stitching

@dataclass
class Piece:
    name: str
    group: str
    rect: Rect
    path: Path


@dataclass
class Segment:
    orient: str   # 'h' (constant y) or 'v' (constant x)
    c: float      # the constant coordinate
    t0: float
    t1: float


def shared_segment(a: Rect, b: Rect, tol: float = 1e-6) -> Segment | None:
    """Boundary segment two touching rectangles share (None for no contact or a point)."""
    x0, x1 = max(a.x0, b.x0), min(a.x1, b.x1)
    y0, y1 = max(a.y0, b.y0), min(a.y1, b.y1)
    if x1 < x0 - tol or y1 < y0 - tol:
        return None
    w, h = x1 - x0, y1 - y0
    if abs(h) <= tol and w > tol:
        return Segment("h", 0.5 * (y0 + y1), x0, x1)
    if abs(w) <= tol and h > tol:
        return Segment("v", 0.5 * (x0 + x1), y0, y1)
    return None


def stitch_pieces(cfg: Config, meshes_dir: str | Path) -> list[Piece]:
    """All ground/collision pieces present in ``meshes_dir`` with their nominal rectangles."""
    lay = Layout(cfg)
    md = Path(meshes_dir)
    pieces = []
    for t in lay.tiles():
        f = md / lay.tile_mesh_name(t, "ground")
        if f.exists():
            pieces.append(Piece(f.stem, "visual", lay.tile_rect(t), f))
    for zone in ("ring", "far", "horizon"):
        for side, r in lay.strip_rects(zone).items():
            f = md / lay.strip_mesh_name(zone, side, "ground")
            if f.exists():
                pieces.append(Piece(f.stem, "visual", r, f))
    f = md / "core_collision.glb"
    if f.exists():
        pieces.append(Piece(f.stem, "collision", lay.zone_rect("core"), f))
    for t in lay.tiles():                      # per-tile core collision (DESIGN 7)
        f = md / lay.tile_mesh_name(t, "collision")
        if f.exists():
            pieces.append(Piece(f.stem, "collision", lay.tile_rect(t), f))
    for zone in ("ring", "far"):
        for side, r in lay.strip_rects(zone).items():
            f = md / lay.strip_mesh_name(zone, side, "collision")
            if f.exists():
                pieces.append(Piece(f.stem, "collision", r, f))
    return pieces


@dataclass
class _Border:
    pos: np.ndarray        # (k, 3) float64 of vertices within tol of the rect perimeter
    edges: set             # boundary edges (pairs of indices into pos) along the perimeter


def _border_of(path: Path, rect: Rect) -> _Border:
    g = read_glb(path)
    wp, wt = weld_positions(g.positions_f32, g.triangles)
    tol = _perimeter_tol(rect)
    near = ((np.abs(wp[:, 0] - rect.x0) <= tol) | (np.abs(wp[:, 0] - rect.x1) <= tol)
            | (np.abs(wp[:, 1] - rect.y0) <= tol) | (np.abs(wp[:, 1] - rect.y1) <= tol))
    idx = np.flatnonzero(near)
    remap = -np.ones(len(wp), dtype=np.int64)
    remap[idx] = np.arange(len(idx))
    edges, counts = _edge_list(wt)
    b = edges[counts == 1]
    b = b[(remap[b[:, 0]] >= 0) & (remap[b[:, 1]] >= 0)]
    es = {(int(min(remap[i], remap[j])), int(max(remap[i], remap[j]))) for i, j in b}
    return _Border(wp[idx], es)


def compare_segment(A: _Border, B: _Border, seg: Segment, key_m: float = 1e-5, z_tol: float = 1e-5,
                    collinear_tol: float = 1e-4) -> dict:
    """Compare the boundary vertices two pieces have on a shared segment."""
    tol = 1e-5 + 2.4e-7 * max(abs(seg.c), abs(seg.t0), abs(seg.t1))
    ci, ti = (1, 0) if seg.orient == "h" else (0, 1)

    def on_seg(P):
        if len(P) == 0:
            return np.zeros(0, dtype=np.int64)
        m = (np.abs(P[:, ci] - seg.c) <= tol) & (P[:, ti] >= seg.t0 - tol) & (P[:, ti] <= seg.t1 + tol)
        return np.flatnonzero(m)

    ia, ib = on_seg(A.pos), on_seg(B.pos)
    PA, PB = A.pos[ia], B.pos[ib]
    ka = {(int(round(p[0] / key_m)), int(round(p[1] / key_m))): k for k, p in zip(ia, PA)}
    kb = {(int(round(p[0] / key_m)), int(round(p[1] / key_m))): k for k, p in zip(ib, PB)}
    common = ka.keys() & kb.keys()
    only_a = ka.keys() - kb.keys()
    only_b = kb.keys() - ka.keys()
    dz = np.array([A.pos[ka[k], 2] - B.pos[kb[k], 2] for k in common]) if common else np.zeros(0)
    res: dict[str, Any] = {"n_a": len(ka), "n_b": len(kb), "n_common": len(common), "only_a": len(only_a),
                           "only_b": len(only_b), "height_mismatch": int((np.abs(dz) > z_tol).sum()),
                           "max_abs_dz_m": float(np.abs(dz).max()) if len(dz) else 0.0}

    def classify(only, own: dict, own_border: _Border, other: dict, other_border: _Border) -> dict:
        out = {"t_junction": 0, "t_junction_noncollinear": 0, "uncovered": 0, "examples": []}
        if not only:
            return out
        oi = np.array(list(other.values()), dtype=np.int64)
        if len(oi) == 0:
            out["uncovered"] = len(only)
            return out
        ot = other_border.pos[oi, ti]
        order = np.argsort(ot)
        ot, oi = ot[order], oi[order]
        for k in only:
            p = own_border.pos[own[k]]
            t = p[ti]
            j = int(np.searchsorted(ot, t))
            if 0 < j < len(ot) and ot[j - 1] < t < ot[j]:
                a_, b_ = oi[j - 1], oi[j]
                f = (t - ot[j - 1]) / (ot[j] - ot[j - 1])
                zi = other_border.pos[a_, 2] + f * (other_border.pos[b_, 2] - other_border.pos[a_, 2])
                out["t_junction"] += 1
                if abs(p[2] - zi) > collinear_tol:
                    out["t_junction_noncollinear"] += 1
                if len(out["examples"]) < 3:
                    out["examples"].append({"xyz": p.round(5).tolist(), "dz_to_edge_m": float(p[2] - zi)})
            else:
                out["uncovered"] += 1
                if len(out["examples"]) < 3:
                    out["examples"].append({"xyz": p.round(5).tolist(), "uncovered": True})
        return out

    ca = classify(only_a, ka, A, kb, B)
    cb = classify(only_b, kb, B, ka, A)
    res["t_junctions"] = ca["t_junction"] + cb["t_junction"]
    res["t_junctions_noncollinear"] = ca["t_junction_noncollinear"] + cb["t_junction_noncollinear"]
    res["uncovered"] = ca["uncovered"] + cb["uncovered"]
    res["examples"] = (ca["examples"] + cb["examples"])[:4]
    # coverage: both pieces must reach both segment ends
    for name, P in (("a", PA), ("b", PB)):
        if len(P) == 0:
            res[f"coverage_{name}"] = [None, None]
            res["uncovered"] += 1
        else:
            res[f"coverage_{name}"] = [float(P[:, ti].min()), float(P[:, ti].max())]
            if P[:, ti].min() > seg.t0 + tol or P[:, ti].max() < seg.t1 - tol:
                res["uncovered"] += 1
    # consecutive boundary vertices of each piece along the segment must be joined by a boundary edge
    for name, border, ii in (("a", A, ia), ("b", B, ib)):
        if len(ii) < 2:
            continue
        o = ii[np.argsort(border.pos[ii, ti])]
        gaps = sum(1 for u, v in zip(o[:-1], o[1:]) if (min(u, v), max(u, v)) not in border.edges)
        res[f"edge_gaps_{name}"] = gaps
    res["mismatches"] = res["only_a"] + res["only_b"] + res["height_mismatch"]
    res["ok"] = (res["mismatches"] == 0 and res["uncovered"] == 0 and res.get("edge_gaps_a", 0) == 0
                 and res.get("edge_gaps_b", 0) == 0)
    return res


def check_stitched_pieces(pieces: Sequence[Piece], max_report: int = 50) -> dict:
    """Stitching invariant over all adjacent pairs of pieces of the same group."""
    borders: dict[str, _Border] = {}
    errors = {}
    for p in pieces:
        try:
            borders[p.name] = _border_of(p.path, p.rect)
        except Exception as e:  # noqa: BLE001
            errors[p.name] = f"{type(e).__name__}: {e}"[:200]
    pairs, bad = [], []
    tot = {"pairs": 0, "mismatches": 0, "t_junctions": 0, "t_junctions_noncollinear": 0, "height_mismatch": 0,
           "uncovered": 0, "edge_gaps": 0}
    by_type: dict[str, int] = {}
    for i, a in enumerate(pieces):
        for b in pieces[i + 1:]:
            if a.group != b.group or a.name not in borders or b.name not in borders:
                continue
            seg = shared_segment(a.rect, b.rect)
            if seg is None:
                continue
            r = compare_segment(borders[a.name], borders[b.name], seg)
            r.update({"a": a.name, "b": b.name, "segment": [seg.orient, seg.c, seg.t0, seg.t1]})
            typ = "-".join(sorted({a.name.split("_")[0], b.name.split("_")[0]}))
            by_type[f"{a.group}:{typ}"] = by_type.get(f"{a.group}:{typ}", 0) + 1
            tot["pairs"] += 1
            for k in ("mismatches", "t_junctions", "t_junctions_noncollinear", "height_mismatch", "uncovered"):
                tot[k] += r[k]
            tot["edge_gaps"] += r.get("edge_gaps_a", 0) + r.get("edge_gaps_b", 0)
            pairs.append(r)
            if not r["ok"]:
                bad.append(r)
    ok = not errors and all(tot[k] == 0 for k in tot if k != "pairs")
    return {"ok": ok, "totals": tot, "pairs_by_type": by_type, "failing_pairs": bad[:max_report],
            "n_failing_pairs": len(bad), "read_errors": errors}


def check_stitching(cfg: Config, inputs: dict) -> dict:
    t0 = time.time()
    meshes = inputs.get("meshes_dir")
    if not meshes:
        return _result("skipped", reason="no meshes dir")
    pieces = stitch_pieces(cfg, meshes)
    if len(pieces) < 2:
        return _result("skipped", reason=f"{len(pieces)} stitchable piece(s) found")
    r = check_stitched_pieces(pieces)
    lay = Layout(cfg)
    expected_visual = lay.n_tiles ** 2 + 12
    expected_collision = lay.n_tiles ** 2 + 8
    n_vis = sum(p.group == "visual" for p in pieces)
    n_col = sum(p.group == "collision" for p in pieces)
    # every expected piece must be there: a missing tile also has no seams to compare (CR-03)
    complete = n_vis == expected_visual and n_col in (expected_collision, 1 + 8)
    return _result("pass" if (r["ok"] and complete) else "fail", n_pieces_visual=n_vis, n_pieces_collision=n_col,
                   expected_visual_pieces=expected_visual, expected_collision_pieces=expected_collision,
                   complete=complete, **r,
                   method="boundary vertices on each shared segment, positions rounded to 1e-5 m (keys), "
                          "heights compared to 1e-5 m; vertices of one piece inside an edge of the other "
                          "= T-junctions", seconds=round(time.time() - t0, 2))


# ==============================================================================================
# check: world_info

def check_world_info(cfg: Config, inputs: dict) -> dict:
    wi = inputs.get("world_info")
    if not wi:
        return _result("skipped", reason="world_info.yaml not written yet")
    import yaml
    try:
        info = yaml.safe_load(Path(wi).read_text())
    except Exception as e:  # noqa: BLE001
        return _result("error", reason=f"cannot parse {wi}: {e}")
    sites = parse_sites(info)
    core = Layout(cfg).zone_rect("core")
    probs = []
    for name in cfg.spawn.get("test_sites", []):
        if name not in sites:
            probs.append(f"site {name} missing")
    for name, s in sites.items():
        if not core.contains(s["x"], s["y"]):
            probs.append(f"site {name} outside the core")
    return _result("fail" if probs else "pass", path=wi, sites=sites, problems=probs,
                   keys=sorted(info) if isinstance(info, dict) else None)


def parse_sites(info: Any) -> dict[str, dict]:
    """Sites from world_info.yaml: a mapping name -> {x, y, z, yaw, ...} or a list of such
    entries with a 'name'. Positions may also be given as 'pose: [x, y, z, yaw]'."""
    raw = info.get("sites") if isinstance(info, dict) else None
    out: dict[str, dict] = {}
    if raw is None:
        return out
    items = raw.items() if isinstance(raw, dict) else ((r.get("name"), r) for r in raw if isinstance(r, dict))
    for name, s in items:
        if not isinstance(s, dict) or name is None:
            continue
        pose = s.get("pose") or s.get("xyzyaw")
        if pose is not None and len(pose) >= 3:
            x, y, z = (float(v) for v in pose[:3])
            yaw = float(pose[3]) if len(pose) > 3 else 0.0
        else:
            try:
                x, y, z = float(s["x"]), float(s["y"]), float(s.get("z", 0.0))
            except (KeyError, TypeError, ValueError):
                continue
            yaw = float(s.get("yaw", s.get("yaw_rad", 0.0)))
            if "yaw_deg" in s and "yaw" not in s:
                yaw = math.radians(float(s["yaw_deg"]))
        out[str(name)] = {"x": x, "y": y, "z": z, "yaw": yaw,
                          "description": s.get("description", s.get("desc", ""))}
    return out


# ==============================================================================================
# driver

def summarize_results(results: dict[str, dict], seconds: float | None = None) -> dict:
    """Overall verdict: pass = no check failed or errored and at least one passed."""
    n = {s: sum(1 for r in results.values() if r.get("status") == s) for s in STATUS_ORDER}
    out = {"pass": n["fail"] == 0 and n["error"] == 0 and n["pass"] > 0,
           "all_ran": n["skipped"] == 0 and n["error"] == 0,
           "n_pass": n["pass"], "n_fail": n["fail"], "n_skipped": n["skipped"], "n_error": n["error"],
           "failed": [k for k, r in results.items() if r.get("status") == "fail"],
           "skipped": [k for k, r in results.items() if r.get("status") == "skipped"],
           "errors": [k for k, r in results.items() if r.get("status") == "error"]}
    if seconds is not None:
        out["seconds"] = round(seconds, 2)
    return out


CHECKS =("roughness", "craters", "rocks", "visual_vs_collision", "glb", "stitching", "sdf", "world_info")


def run_static_checks(cfg: Config, *, build_dir: str | Path | None = None, world_dir: str | Path | None = None,
                      inputs: dict | None = None, only: Iterable[str] | None = None, write: bool = True,
                      out_path: str | Path | None = None, n_dz_samples: int = 200_000, stride: int = 1,
                      with_trimesh: bool = True, verbose: bool = True) -> dict:
    """Run the static acceptance checks and (by default) write reports/static_checks.json."""
    t0 = time.time()
    found = locate_inputs(cfg, build_dir, world_dir, inputs)
    sel = list(only) if only else list(CHECKS)
    unknown = [c for c in sel if c not in CHECKS]
    if unknown:
        raise ValueError(f"unknown checks {unknown}; available {CHECKS}")
    funcs = {
        "roughness": lambda: check_roughness(cfg, found, stride=stride),
        "craters": lambda: check_craters(cfg, found),
        "rocks": lambda: check_rocks(cfg, found),
        "visual_vs_collision": lambda: check_visual_vs_collision(cfg, found, n_samples=n_dz_samples),
        "glb": lambda: check_glbs(cfg, found, with_trimesh=with_trimesh),
        "stitching": lambda: check_stitching(cfg, found),
        "sdf": lambda: check_sdf(cfg, found),
        "world_info": lambda: check_world_info(cfg, found),
    }
    results: dict[str, dict] = {}
    for name in sel:
        ts = time.time()
        try:
            results[name] = funcs[name]()
        except Exception as e:  # noqa: BLE001 - a crashing check must not hide the others
            import traceback
            results[name] = _result("error", reason=f"{type(e).__name__}: {e}",
                                    traceback=traceback.format_exc()[-2000:])
        results[name].setdefault("seconds", round(time.time() - ts, 2))
        if verbose:
            print(f"[static_checks] {name:22s} {results[name]['status']:8s} {results[name]['seconds']:7.1f} s",
                  flush=True)
    report = {"kind": "static_checks", "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
              "world": cfg.world.name, "config_hash": config_hash(cfg),
              "config_source": cfg.get("_source"), "inputs": found, "checks": results,
              "summary": summarize_results(results, time.time() - t0)}
    if write:
        path = Path(out_path) if out_path else Path(found["world_dir"]) / "reports" / "static_checks.json"
        write_json(path, report)
        report["written_to"] = str(path)
        if verbose:
            print(f"[static_checks] wrote {path}", flush=True)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default=None)
    ap.add_argument("--preset", default=None)
    ap.add_argument("--set", action="append", default=[], help="config override key=value")
    ap.add_argument("--only", default=None, help=f"comma list of {','.join(CHECKS)}")
    ap.add_argument("--build-dir")
    ap.add_argument("--world-dir")
    ap.add_argument("--out", help="output JSON (default world_dir/reports/static_checks.json)")
    ap.add_argument("--input", action="append", default=[], help="explicit input key=path (e.g. core_grid=PREFIX)")
    ap.add_argument("--dz-samples", type=int, default=200_000)
    ap.add_argument("--stride", type=int, default=1, help="deviogram row/column stride (speed)")
    ap.add_argument("--no-trimesh", action="store_true")
    a = ap.parse_args(argv)
    cfg = load_config(a.config, a.preset, a.set)
    ov = {}
    for kv in a.input:
        k, _, v = kv.partition("=")
        ov[k] = [s for s in v.split(",")] if k == "crater_catalogs" else v
    rep = run_static_checks(cfg, build_dir=a.build_dir, world_dir=a.world_dir, inputs=ov,
                            only=a.only.split(",") if a.only else None, out_path=a.out,
                            n_dz_samples=a.dz_samples, stride=a.stride, with_trimesh=not a.no_trimesh)
    s = rep["summary"]
    print(json.dumps(s, default=str))
    return 0 if s["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
