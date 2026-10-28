"""Reader for the rock prototype library (docs/DESIGN.md section 5).

The library is produced by ``blender/rock_library.py`` (headless Blender) into
``cfg.paths.rock_library`` (``~/togo_assets/lunar/library/rocks_v1``)::

    library.json                    prototype list, atlas, conventions, provenance
    npz/<id>_lod<k>.npz             positions (n,3) f4, normals (n,3) f4, uv (n,2) f4, indices (m,3) u4
    hulls/<id>.npz                  vertices (k,3) f4, faces (t,3) u4   (convex, outward CCW, <= 64 vertices)
    atlas/rock_atlas_{basecolor,normal,orm,roughness}.png

Conventions (identical for every file):

* Rock-local frame, Z up, metres, origin = volume centroid of the high-poly rock. The rock rests on
  its flattened base with its shortest axis vertical; ``dims_m = [a, b, c]`` are the x, y, z extents
  of LOD0 with ``a`` the longest axis. ``bottom_offset_m`` is the distance from the origin down to
  the lowest LOD0 vertex.
* ``uv`` are atlas coordinates in the glTF convention (origin at the top-left of the PNG, v down).
* LOD vertex arrays are glTF-style corner arrays (positions duplicated where normals or UVs split);
  the merged geometry is closed (watertight) after welding equal positions.

``RockLibrary.rest_faces`` lists the quasi-static rest poses of each hull (facets the centre of mass
projects into, with their stability margin); ``placement`` rests rocks on them (FID-01).
``RockLibrary.hull_planes`` gives the hull as half-spaces (fillets, interpenetration tests).

``write_synthetic_library`` writes a small library of random convex polyhedra in exactly this
format. It needs no Blender and is meant for unit tests and dry runs of the world generator.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

LIBRARY_CLASSES = ("pebble", "medium", "large")
LOD_LEVELS = (0, 1, 2)
ATLAS_KEYS = ("basecolor", "normal", "orm", "roughness")


@dataclass
class Prototype:
    """One rock prototype (record of ``library.json``)."""

    index: int
    id: str
    cls: str
    dims: tuple[float, float, float]          # (a, b, c) extents [m]; a = longest axis
    bottom_offset: float                      # origin -> lowest LOD0 vertex [m]
    lod_tris: tuple[int, ...]                 # triangle count per LOD
    lod_files: tuple[str, ...]                # npz path per LOD (relative to the library root)
    hull_file: str                            # hull npz (relative)
    meta: dict = field(default_factory=dict, repr=False, compare=False)

    @property
    def a(self) -> float:
        """Longest axis [m] (the size the SFD diameter D refers to)."""
        return float(max(self.dims))

    @property
    def c(self) -> float:
        """Vertical extent [m]."""
        return float(self.dims[2])

    @property
    def n_lods(self) -> int:
        return len(self.lod_tris)


class RockLibrary:
    """Read-only access to a rock library folder (lazy, cached loading of the npz files)."""

    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser()
        jpath = self.path / "library.json"
        if not jpath.exists():
            raise FileNotFoundError(f"rock library not found: {jpath} (build it with blender/rock_library.py)")
        raw = jpath.read_bytes()
        self.meta: dict = json.loads(raw)
        self.digest: str = hashlib.sha256(raw).hexdigest()[:16]
        self.version: str = str(self.meta.get("version", self.path.name))
        self.seed = self.meta.get("seed")
        self.prototypes: list[Prototype] = []
        for k, rec in enumerate(self.meta["prototypes"]):
            lods = sorted(rec["lods"], key=lambda r: int(r["lod"]))
            self.prototypes.append(Prototype(
                index=int(rec.get("index", k)), id=str(rec["id"]), cls=str(rec["cls"]),
                dims=tuple(float(v) for v in rec["dims_m"]),
                bottom_offset=float(rec["bottom_offset_m"]),
                lod_tris=tuple(int(r["tris"]) for r in lods),
                lod_files=tuple(str(r["file"]) for r in lods),
                hull_file=str(rec["hull"]["file"]),
                meta=rec,
            ))
        if [p.index for p in self.prototypes] != list(range(len(self.prototypes))):
            raise ValueError(f"{jpath}: prototype indices must be 0..n-1 in file order")
        self._by_id = {p.id: p for p in self.prototypes}
        self._mesh_cache: dict[tuple[int, int], tuple[np.ndarray, ...]] = {}
        self._hull_cache: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._contact_cache: dict[tuple[int, int], np.ndarray] = {}
        self._plane_cache: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self._rest_cache: dict[int, dict[str, np.ndarray]] = {}

    # ------------------------------------------------------------------ lookup
    def __len__(self) -> int:
        return len(self.prototypes)

    def proto(self, proto_id: int | str | np.integer) -> Prototype:
        """Prototype by index or id string."""
        if isinstance(proto_id, (int, np.integer)):
            return self.prototypes[int(proto_id)]
        return self._by_id[str(proto_id)]

    def by_class(self, cls: str) -> list[Prototype]:
        """Prototypes of a library class ('pebble' | 'medium' | 'large'), in index order."""
        return [p for p in self.prototypes if p.cls == cls]

    @property
    def class_names(self) -> list[str]:
        return sorted({p.cls for p in self.prototypes}, key=lambda c: (LIBRARY_CLASSES + (c,)).index(c))

    def dims_array(self) -> np.ndarray:
        """(n_proto, 3) array of [a, b, c]."""
        return np.array([p.dims for p in self.prototypes], dtype=np.float64)

    def a_array(self) -> np.ndarray:
        return np.array([p.a for p in self.prototypes], dtype=np.float64)

    @property
    def atlas_paths(self) -> dict[str, Path]:
        """Absolute paths of the shared atlas maps (basecolor sRGB, normal, orm, roughness)."""
        files = self.meta.get("atlas", {}).get("files", {})
        if isinstance(files, list):           # prototype-era layout: list of paths
            files = {Path(f).stem.replace("rock_atlas_", ""): f for f in files}
        return {k: self.path / v for k, v in files.items()}

    @property
    def atlas_size(self) -> int:
        return int(self.meta.get("atlas", {}).get("size_px", 0))

    # ------------------------------------------------------------------ geometry
    def lod_mesh(self, proto_id: int | str, lod: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(positions (n,3) f4, normals (n,3) f4, uv (n,2) f4, indices (m,3) u4) of one LOD (cached)."""
        p = self.proto(proto_id)
        lod = int(lod)
        if not 0 <= lod < p.n_lods:
            raise ValueError(f"{p.id}: no LOD {lod} (has {p.n_lods})")
        key = (p.index, lod)
        if key not in self._mesh_cache:
            with np.load(self.path / p.lod_files[lod]) as d:
                uv = d["uv"] if "uv" in d else d["uv_atlas_gltf"]
                arrs = (np.ascontiguousarray(d["positions"], dtype=np.float32),
                        np.ascontiguousarray(d["normals"], dtype=np.float32),
                        np.ascontiguousarray(uv, dtype=np.float32),
                        np.ascontiguousarray(d["indices"], dtype=np.uint32))
            for a in arrs:
                a.setflags(write=False)
            self._mesh_cache[key] = arrs
        return self._mesh_cache[key]

    def hull(self, proto_id: int | str) -> np.ndarray:
        """Convex hull vertices (k,3) float64, Z-up rock frame."""
        return self._load_hull(proto_id)[0]

    def hull_faces(self, proto_id: int | str) -> np.ndarray:
        """Convex hull triangles (t,3) int64 (outward, CCW)."""
        return self._load_hull(proto_id)[1]

    def _load_hull(self, proto_id: int | str) -> tuple[np.ndarray, np.ndarray]:
        p = self.proto(proto_id)
        if p.index not in self._hull_cache:
            with np.load(self.path / p.hull_file) as d:
                v = np.asarray(d["vertices"], dtype=np.float64)
                f = np.asarray(d["faces"], dtype=np.int64)
            v.setflags(write=False)
            f.setflags(write=False)
            self._hull_cache[p.index] = (v, f)
        return self._hull_cache[p.index]

    def contact_points(self, proto_id: int | str, lod: int) -> np.ndarray:
        """Unique vertex positions (k,3) float64 of a LOD; used to rest rocks on the ground."""
        p = self.proto(proto_id)
        key = (p.index, int(lod))
        if key not in self._contact_cache:
            pos = self.lod_mesh(p.index, lod)[0].astype(np.float64)
            u = np.unique(np.round(pos, 7), axis=0)
            u.setflags(write=False)
            self._contact_cache[key] = u
        return self._contact_cache[key]

    def lod_extent_z(self, proto_id: int | str, lod: int) -> tuple[float, float]:
        """(min z, max z) of a LOD in the rock frame."""
        pts = self.contact_points(proto_id, lod)
        return float(pts[:, 2].min()), float(pts[:, 2].max())

    def hull_planes(self, proto_id: int | str) -> tuple[np.ndarray, np.ndarray]:
        """Face planes of the convex hull in the rock frame: outward unit normals (t,3) and offsets (t,),
        so that a point p is inside the hull iff ``normals @ p <= offsets`` for every face (cached)."""
        p = self.proto(proto_id)
        if p.index not in self._plane_cache:
            V, F = self._load_hull(p.index)
            a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
            n = np.cross(b - a, c - a)
            n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-300)
            o = np.einsum("ij,ij->i", n, a)
            n.setflags(write=False)
            o.setflags(write=False)
            self._plane_cache[p.index] = (n, o)
        return self._plane_cache[p.index]

    def rest_faces(self, proto_id: int | str) -> dict[str, np.ndarray]:
        """Quasi-static rest poses of a prototype on a plane (rock frame; origin = centre of mass).

        Coplanar hull triangles are merged into facets. A facet is a stable rest face when the centre
        of mass, projected along the facet normal, falls inside it. Returns arrays over the stable
        facets, sorted by ``h`` (lowest centre of mass first):

        * ``normal`` (k,3) outward unit normal (it points down when the rock rests on the facet);
        * ``h`` (k,) height of the centre of mass above the facet plane;
        * ``tip_deg`` (k,) tilt that topples the rock over the nearest facet edge (stability margin);
        * ``solid_angle`` (k,) solid angle of the facet seen from the centre of mass [sr] (a proxy for
          the probability of landing on it);
        * ``extent`` (k,) height of the hull above the facet plane (the rock's height in that pose);
        * ``area`` (k,) facet area [m^2];
        * ``base_gap`` (k,) flatness of the underside in that pose: the largest height of the hull's
          underside above the facet plane over the central 60 % of the footprint (the footprint shrunk
          to 0.6 about its centroid) [m]; 0 for a flat base.
        """
        p = self.proto(proto_id)
        if p.index in self._rest_cache:
            return self._rest_cache[p.index]
        V, F = self._load_hull(p.index)
        n, o = self.hull_planes(p.index)
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        q = o[:, None] * n                                       # centre of mass projected on each plane
        e0 = np.einsum("ij,ij->i", np.cross(b - a, q - a), n)
        e1 = np.einsum("ij,ij->i", np.cross(c - b, q - b), n)
        e2 = np.einsum("ij,ij->i", np.cross(a - c, q - c), n)
        inside = np.minimum(np.minimum(e0, e1), e2) >= -1e-12 * max(p.a, 1e-9) ** 2
        cos_tol = np.cos(np.radians(1.0))
        group = -np.ones(len(F), dtype=np.int64)
        ng = 0
        for i in range(len(F)):
            if group[i] < 0:
                same = (group < 0) & (n @ n[i] > cos_tol) & (np.abs(o - o[i]) < 1e-4 * max(p.a, 1e-9))
                group[same] = ng
                ng += 1
        keys = ("normal", "h", "tip_deg", "solid_angle", "extent", "area", "base_gap")
        out: dict[str, list] = {k: [] for k in keys}
        for g in range(ng):
            idx = np.nonzero(group == g)[0]
            if not inside[idx].any():
                continue
            k0 = idx[np.argmax(area[idx])]
            nrm, h = n[k0], float(o[k0])
            E = np.sort(np.concatenate([F[idx][:, [0, 1]], F[idx][:, [1, 2]], F[idx][:, [2, 0]]]), axis=1)
            uniq, cnt = np.unique(E, axis=0, return_counts=True)
            dmin = np.inf
            for e0_, e1_ in uniq[cnt == 1]:
                A_, B_ = V[e0_], V[e1_]
                AB = B_ - A_
                t = np.clip(np.dot(q[k0] - A_, AB) / max(np.dot(AB, AB), 1e-300), 0.0, 1.0)
                dmin = min(dmin, float(np.linalg.norm(q[k0] - (A_ + t * AB))))
            out["normal"].append(nrm)
            out["h"].append(h)
            out["tip_deg"].append(float(np.degrees(np.arctan2(dmin, max(h, 1e-12)))))
            out["solid_angle"].append(float(_solid_angle(a[idx], b[idx], c[idx]).sum()))
            out["extent"].append(float(h - (V @ nrm).min()))
            out["area"].append(float(area[idx].sum()))
            out["base_gap"].append(_base_gap(V, n, o, nrm, h))
        order = np.argsort(np.array(out["h"]), kind="stable")
        res = {k: np.asarray(v, dtype=np.float64)[order] for k, v in out.items()}
        if not len(order):                                       # degenerate hull: rest on the authored base
            res = {"normal": np.array([[0.0, 0.0, -1.0]]), "h": np.array([p.bottom_offset]),
                   "tip_deg": np.array([0.0]), "solid_angle": np.array([4 * np.pi]), "extent": np.array([p.c]),
                   "area": np.array([0.0]), "base_gap": np.array([0.0])}
        for v in res.values():
            v.setflags(write=False)
        self._rest_cache[p.index] = res
        return res

    def summary(self) -> dict:
        """Counts and triangle ranges per class and LOD (for reports)."""
        out: dict = {"version": self.version, "n_prototypes": len(self), "digest": self.digest, "classes": {}}
        for cls in self.class_names:
            ps = self.by_class(cls)
            tri = np.array([p.lod_tris for p in ps])
            out["classes"][cls] = {
                "n": len(ps),
                "a_m": [float(min(p.a for p in ps)), float(max(p.a for p in ps))],
                "lod_tris_min": tri.min(axis=0).tolist(), "lod_tris_max": tri.max(axis=0).tolist(),
                "lod_tris_mean": np.round(tri.mean(axis=0), 1).tolist(),
            }
        return out


def _base_gap(V: np.ndarray, n: np.ndarray, o: np.ndarray, nrm: np.ndarray, h: float,
              inner: float = 0.6) -> float:
    """Largest height of a convex hull's underside above the plane ``nrm . p = h`` (the rest facet,
    ``nrm`` its outward normal) over the footprint shrunk to ``inner`` about its centroid.

    The underside of a convex body is a convex function over its footprint, so the maximum over the
    (convex) inner polygon is reached at one of its vertices."""
    from scipy.spatial import ConvexHull

    t = np.array([1.0, 0.0, 0.0]) if abs(nrm[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(nrm, t)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(nrm, e1)
    uv = np.column_stack([V @ e1, V @ e2])
    try:
        P = uv[ConvexHull(uv).vertices]
    except Exception:  # noqa: BLE001  (degenerate projection)
        return 0.0
    x, y = P[:, 0], P[:, 1]
    cr = x * np.roll(y, -1) - np.roll(x, -1) * y
    A = 0.5 * cr.sum()
    if abs(A) < 1e-300:
        return 0.0
    c = np.array([((x + np.roll(x, -1)) * cr).sum(), ((y + np.roll(y, -1)) * cr).sum()]) / (6.0 * A)
    Q = c + inner * (P - c)
    down = (n @ nrm) > 1e-6
    nd, od, dd = n[down], o[down], (n @ nrm)[down]
    pts = h * nrm[None, :] + Q[:, 0:1] * e1[None, :] + Q[:, 1:2] * e2[None, :]
    z = ((pts @ nd.T) - od[None, :]) / dd[None, :]
    return float(max(0.0, z.max(axis=1).max()))


def _solid_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """Solid angles of triangles (a, b, c) (n,3 each) seen from the origin (Van Oosterom & Strackee)."""
    num = np.abs(np.einsum("ij,ij->i", a, np.cross(b, c)))
    la, lb, lc = (np.linalg.norm(v, axis=1) for v in (a, b, c))
    den = (la * lb * lc + np.einsum("ij,ij->i", a, b) * lc + np.einsum("ij,ij->i", a, c) * lb
           + np.einsum("ij,ij->i", b, c) * la)
    return 2.0 * np.arctan2(num, den)


# ---------------------------------------------------------------------------------------------
# Synthetic library (tests / dry runs; no Blender needed)
# ---------------------------------------------------------------------------------------------
def _convex_polyhedron(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convex hull of points -> (vertices, outward CCW triangles), vertices compacted."""
    from scipy.spatial import ConvexHull

    hull = ConvexHull(points)
    used = np.unique(hull.simplices)
    remap = -np.ones(len(points), dtype=np.int64)
    remap[used] = np.arange(len(used))
    v = points[used]
    f = remap[hull.simplices]
    c = v.mean(axis=0)
    a, b, cc = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, cc - a)
    flip = np.einsum("ij,ij->i", n, a - c) < 0
    f[flip] = f[flip][:, [0, 2, 1]]
    return v, f


def _flat_corner_arrays(v: np.ndarray, f: np.ndarray, uv_rect: tuple[float, float, float, float]):
    """Flat-shaded glTF corner arrays (one vertex per triangle corner) with planar atlas UVs."""
    pos = v[f].reshape(-1, 3)
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    nrm = np.repeat(n, 3, axis=0)
    lo, hi = v.min(axis=0), v.max(axis=0)
    t = (pos[:, :2] - lo[:2]) / np.maximum(hi[:2] - lo[:2], 1e-9)
    u0, v0, u1, v1 = uv_rect
    uv = np.stack([u0 + t[:, 0] * (u1 - u0), v0 + (1.0 - t[:, 1]) * (v1 - v0)], axis=1)
    idx = np.arange(len(pos), dtype=np.uint32).reshape(-1, 3)
    return pos.astype(np.float32), nrm.astype(np.float32), uv.astype(np.float32), idx


def write_synthetic_library(path: str | Path, counts: dict[str, int] | None = None, seed: int = 0,
                            atlas_px: int = 64, flat_base: bool = True) -> Path:
    """Write a small library of random convex rocks in the production file format.

    Each prototype is the convex hull of points on an ellipsoid with axes a : 0.7a : 0.5a (optionally
    with a flattened base); LOD0/1/2 are hulls of farthest-point subsets (pebble 14/8/8, medium
    60/20/10, large 160/50/16 points). Sizes per class: pebble 2-5 cm, medium 5-30 cm, large
    0.3-3 m (log-stratified). Returns the library path.
    """
    counts = counts or {"pebble": 4, "medium": 4, "large": 2}
    path = Path(path)
    for d in ("npz", "hulls", "atlas"):
        (path / d).mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    sizes = {"pebble": (0.02, 0.05), "medium": (0.05, 0.30), "large": (0.30, 3.0)}
    # hull points per LOD (a convex hull of k points has 2k - 4 triangles)
    lod_points = {"pebble": (14, 8, 8), "medium": (60, 20, 10), "large": (160, 50, 16)}
    protos = []
    total = sum(counts.values())
    k = 0
    for cls in LIBRARY_CLASSES:
        n = counts.get(cls, 0)
        lo, hi = sizes[cls]
        for i in range(n):
            u = (i + 0.5) / max(n, 1)
            a = float(np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo))))
            axes = np.array([a, 0.7 * a, 0.5 * a]) / 2.0
            pts = rng.normal(size=(400, 3))
            pts /= np.linalg.norm(pts, axis=1, keepdims=True)
            pts *= axes
            if flat_base:
                pts[:, 2] = np.maximum(pts[:, 2], -0.6 * axes[2])
            rid = f"rock_{cls}_{i:02d}"
            uv_rect = (k / total, 0.0, (k + 1) / total, 1.0)
            lods = []
            v0 = None
            for lod, m in enumerate(lod_points[cls]):
                sel = _farthest_point_subset(pts, m)
                v, f = _convex_polyhedron(pts[sel])
                if lod == 0:
                    v0 = v
                    # centre on the centroid of the LOD0 hull vertices, x = longest axis
                    shift = v.mean(axis=0)
                pos, nrm, uv, idx = _flat_corner_arrays(v - shift, f, uv_rect)
                rel = f"npz/{rid}_lod{lod}.npz"
                np.savez_compressed(path / rel, positions=pos, normals=nrm, uv=uv, indices=idx)
                lods.append({"lod": lod, "tris": int(len(idx)), "vertices": int(len(pos)), "file": rel,
                             "uv_rect": list(uv_rect)})
            hv, hf = _convex_polyhedron(v0 - shift)
            hrel = f"hulls/{rid}.npz"
            np.savez_compressed(path / hrel, vertices=hv.astype(np.float32), faces=hf.astype(np.uint32))
            vv = v0 - shift
            dims = vv.max(axis=0) - vv.min(axis=0)
            protos.append({
                "index": k, "id": rid, "cls": cls, "dims_m": [float(x) for x in dims],
                "bottom_offset_m": float(-vv[:, 2].min()), "top_m": float(vv[:, 2].max()),
                "lod_tris": [lv["tris"] for lv in lods], "lods": lods,
                "hull": {"file": hrel, "vertices": int(len(hv)), "triangles": int(len(hf))},
            })
            k += 1
    atlas = {}
    try:
        from PIL import Image

        for key, col in (("basecolor", (120, 116, 110)), ("normal", (128, 128, 255)),
                         ("orm", (255, 230, 0)), ("roughness", (230, 230, 230))):
            rel = f"atlas/rock_atlas_{key}.png"
            Image.new("RGB", (atlas_px, atlas_px), col).save(path / rel)
            atlas[key] = rel
    except ImportError:  # pragma: no cover
        pass
    lib = {"version": "synthetic", "generator": "lunar_world_gen.rocks.library.write_synthetic_library",
           "generator_version": "1", "seed": seed,
           "atlas": {"size_px": atlas_px, "files": atlas}, "prototypes": protos}
    (path / "library.json").write_text(json.dumps(lib, indent=1))
    return path


def _farthest_point_subset(pts: np.ndarray, m: int) -> np.ndarray:
    """Greedy farthest-point subset of m indices (deterministic)."""
    m = min(m, len(pts))
    sel = [int(np.argmax(np.abs(pts[:, 0])))]
    d = np.linalg.norm(pts - pts[sel[0]], axis=1)
    while len(sel) < m:
        j = int(np.argmax(d))
        sel.append(j)
        d = np.minimum(d, np.linalg.norm(pts - pts[j], axis=1))
    return np.array(sel)


def load_library(path: str | Path | None = None, cfg=None) -> RockLibrary:
    """Open the library at ``path`` or ``cfg.paths.rock_library``."""
    if path is None:
        if cfg is None:
            raise ValueError("give a path or a config")
        path = cfg["paths"]["rock_library"]
    return RockLibrary(path)


def iter_lod_meshes(lib: RockLibrary) -> Iterable[tuple[Prototype, int, tuple[np.ndarray, ...]]]:
    """Yield (prototype, lod, (positions, normals, uv, indices)) for every LOD of every prototype."""
    for p in lib.prototypes:
        for lod in range(p.n_lods):
            yield p, lod, lib.lod_mesh(p.index, lod)
