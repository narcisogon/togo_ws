"""
rock_proto.py -- procedural Apollo-16-style lunar rock library prototype (headless Blender + Cycles).

Run (Git Bash on Windows; Blender 5.2.1 LTS tested, needs >= 4.2 for the APIs used):

    "/c/Program Files/Blender Foundation/Blender 5.2/blender.exe" -b --factory-startup \
        -P rock_proto.py -- --out //wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/experiments/blender/run1 \
        --seed 1 --n-large 2 --n-medium 3 --n-pebble 4 --atlas 2048 --preview

Everything after "--" is parsed by argparse (`... -P rock_proto.py -- --help` lists all options).
Modes (--mode):
    full     (default) generate rocks, LODs, UVs, bakes, atlas, GLB/NPZ/JSON exports [+ --preview renders]
    shapes   generate high-poly + LOD only and render clay previews (fast shape iteration, no bake/export)
    preview  only render previews of an existing output dir (re-imports rocks_library.glb)

Pipeline per rock (units: metres, Blender frame: right-handed, Z up):
  1. shape    lumpy superellipsoid with impact-fragment axial ratios a:b:c ~ 2:sqrt(2):1 -> exact planar
              "fracture" cuts (bmesh.bisect_plane + hole fill) incl. a flattened base and small edge chips
              -> affine rescale back to the sampled a, b, c -> voxel remesh (uniform topology, edges slightly
              rounded) -> height-weighted Taubin smoothing (rounded, micrometeorite-eroded top / angular base)
              -> Catmull-Clark subdivision -> numpy displacement (fBm roughness, cellular fracture cracks,
              pits on the exposed top) -> origin at the volume centroid, uniform rescale so max extent == a.
  2. LOD      medium/large: quadric (collapse) decimation to the class triangle budget.
              pebbles: greedy convex hull with k vertices (2k-4 triangles) of the high-poly.
  3. UV       smart UV project (it packs the islands into one 0..1 square per rock). The atlas region per
              rock is the smallest power of two reaching the class texel target (--texel-mm-<class>).
  4. bake     Cycles (GPU/OptiX), high -> low, from a procedural breccia material (angular clasts in a
              matrix, dark glass coatings and veins, dust): tangent-space normal map (OpenGL/glTF +Y),
              base colour (DIFFUSE/COLOR pass, specular disabled so it equals the albedo), roughness.
  5. hull     greedy convex hull (<= --hull-max-verts vertices) of the LOD vertices for collision.
  6. export   rocks/<id>.glb (standalone, own textures), atlas packing (quadtree, power-of-two regions),
              rocks_library.glb (all LODs, ONE material + the shared atlas embedded once),
              hulls/<id>_hull.glb + rocks_hulls.glb, npz/<id>.npz, meta/<id>.json, library.json,
              plus two test assets for the Gazebo side: axis_marker.glb and normalmap_test.glb.

Coordinate conventions (read this before using the files in a world generator):
  * Rock-local frame = Blender frame: Z up, metres, origin = volume centroid of the high-poly rock;
    the rock rests on its flattened base with its smallest axis (c) vertical. bottom_offset_m in the
    metadata is the distance from the origin down to the lowest LOD vertex.
  * glTF files are written with export_yup=True (glTF spec +Y up). The Blender exporter rewrites every
    position / normal / tangent as (x, y, z)_blender -> (x, z, -y)_gltf; node transforms stay identity.
    Back to Z-up: rotate +90 deg about X, (x, y, z)_gltf -> (x, -z, y). If the consumer does NOT do
    the Y-up -> Z-up conversion itself, add roll = +pi/2 to the SDF <visual>/<collision> pose.
  * npz/*.npz and the hull vertices in meta/*.json are Z-up (rock-local frame). npz UVs use the glTF
    convention (origin top-left of the PNG, v down).
"""

import argparse
import json
import math
import os
import sys
import time
import warnings

import bmesh
import bpy
import mathutils
import numpy as np
from mathutils.bvhtree import BVHTree

LOG_PREFIX = "[rock_proto]"


def log(*a):
    print(LOG_PREFIX, *a, flush=True)


# ----------------------------------------------------------------------------------------------
# Class configuration (design targets). Region = atlas square (px) per rock.
# ----------------------------------------------------------------------------------------------
CLASS_IDS = {"large": 1, "medium": 2, "pebble": 3}
CLASS_CFG = {
    # size: longest axis a [m] (log-stratified); tris: LOD triangle budget (pebble: random in range,
    # medium/large: grows with size); n_vox: voxel remesh resolution across `a` (fresh -> rounded);
    # hi_faces: target high-poly triangles; cuts / chips: number of fracture planes / edge chips.
    "large": dict(size=(0.30, 3.0), tris=(2000, 5000), region_min=256, gap_px=8, margin_px=8,
                  n_vox=(120, 60), hi_faces=700000, cuts=(8, 13), chips=(10, 24), sharp_deg=40.0,
                  burial=(0.10, 0.35), hull=True, ico=5),
    "medium": dict(size=(0.05, 0.30), tris=(200, 1000), region_min=64, gap_px=4, margin_px=4,
                   n_vox=(90, 45), hi_faces=90000, cuts=(7, 11), chips=(6, 16), sharp_deg=40.0,
                   burial=(0.10, 0.40), hull=True, ico=5),
    "pebble": dict(size=(0.02, 0.05), tris=(16, 36), region_min=32, gap_px=2, margin_px=2,
                   n_vox=(45, 28), hi_faces=12000, cuts=(5, 9), chips=(2, 6), sharp_deg=30.0,
                   burial=(0.20, 0.50), hull=False, ico=4),
}

# Linear albedo palettes. Apollo 16 rocks: light-grey friable breccias with white (anorthositic) and
# dark (impact-melt) clasts, "dark-matrix" breccias that are medium grey, partial dark glass coatings
# and glass veins, dust coating. Highland soil normal albedo ~0.12-0.18 -> regolith ~0.14 here.
LITHOLOGIES = {
    "light_breccia": dict(matrix=(0.19, 0.27), clast_frac=(0.8, 1.1),
                          ramp=[(0.0, 0.06), (0.12, 0.13), (0.32, 0.27), (0.62, 0.42), (0.86, 0.55)],
                          glass=(0.0, 0.06), veins=0.30, dust=(0.25, 0.55), rough=(0.86, 0.94)),
    "dark_matrix_breccia": dict(matrix=(0.10, 0.14), clast_frac=(0.9, 1.2),
                                ramp=[(0.0, 0.05), (0.20, 0.18), (0.50, 0.33), (0.78, 0.50)],
                                glass=(0.0, 0.12), veins=0.50, dust=(0.20, 0.45), rough=(0.85, 0.93)),
    "glass_coated_breccia": dict(matrix=(0.18, 0.25), clast_frac=(0.7, 1.0),
                                 ramp=[(0.0, 0.06), (0.15, 0.15), (0.40, 0.29), (0.72, 0.46)],
                                 glass=(0.30, 0.60), veins=0.70, dust=(0.10, 0.35), rough=(0.85, 0.93)),
    "anorthositic": dict(matrix=(0.33, 0.43), clast_frac=(0.25, 0.45),
                         ramp=[(0.0, 0.07), (0.30, 0.22), (0.60, 0.50)],
                         glass=(0.0, 0.05), veins=0.15, dust=(0.30, 0.60), rough=(0.88, 0.96)),
}
# Deterministic per-class cycle so even a small test set shows the variety (weights ~ 4:2:2:1).
LITHO_CYCLE = ["light_breccia", "dark_matrix_breccia", "glass_coated_breccia", "light_breccia",
               "anorthositic", "light_breccia", "dark_matrix_breccia", "glass_coated_breccia",
               "light_breccia"]
MATRIX_TINT = np.array([1.0, 0.965, 0.925])   # very slightly warm grey (soil-like)
CLAST_TINT = np.array([1.0, 0.99, 0.975])
GLASS_COLOR = 0.035 * np.array([1.0, 0.93, 0.86])
DUST_ALBEDO = 0.15
REGOLITH_ALBEDO = 0.14

# Clast layers: (cell size [m], selection fraction). Layers finer than ~3 texels are skipped.
CLAST_LAYERS = [(0.004, 0.40), (0.015, 0.32), (0.055, 0.26), (0.18, 0.16)]


# ----------------------------------------------------------------------------------------------
# Small numpy noise library (deterministic, Blender-version independent)
# ----------------------------------------------------------------------------------------------
class Noise3:
    """Seeded 3D gradient (Perlin-style) noise, vectorised. Normalised so std ~ 1/3 (|v| <~ 1)."""

    def __init__(self, seed):
        rng = np.random.default_rng(int(seed) & 0xFFFFFFFF)
        p = rng.permutation(256).astype(np.int64)
        self.perm = np.concatenate([p, p])
        g = rng.normal(size=(256, 3))
        self.grad = g / np.linalg.norm(g, axis=1, keepdims=True)
        self.k = 1.0
        s = self(rng.uniform(-50, 50, size=(20000, 3)))
        self.k = (1.0 / 3.0) / max(float(s.std()), 1e-6)

    def __call__(self, P):
        P = np.asarray(P, np.float64)
        Pi = np.floor(P)
        f = P - Pi
        Pi = Pi.astype(np.int64) & 255
        u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0)
        perm, grad = self.perm, self.grad
        X, Y, Z = Pi[:, 0], Pi[:, 1], Pi[:, 2]
        out = np.zeros(len(P))
        for dx in (0, 1):
            wx = u[:, 0] if dx else 1.0 - u[:, 0]
            hx = perm[X + dx]
            for dy in (0, 1):
                wy = u[:, 1] if dy else 1.0 - u[:, 1]
                hy = perm[hx + Y + dy]
                for dz in (0, 1):
                    wz = u[:, 2] if dz else 1.0 - u[:, 2]
                    g = grad[perm[hy + Z + dz]]
                    out += wx * wy * wz * (g[:, 0] * (f[:, 0] - dx) + g[:, 1] * (f[:, 1] - dy)
                                           + g[:, 2] * (f[:, 2] - dz))
        return out * self.k


def fbm(noise, P, octaves, lacunarity=2.0, gain=0.5):
    """Fractal sum; normalised so the std stays ~1/3 regardless of octave count."""
    s = np.zeros(len(P))
    amp, freq, tot = 1.0, 1.0, 0.0
    for o in range(int(octaves)):
        s += amp * noise(P * freq + 17.31 * o)
        tot += amp * amp
        amp *= gain
        freq *= lacunarity
    return s / math.sqrt(tot)


def _hash3(ix, iy, iz, seed):
    """splitmix-style 64-bit hash of integer lattice coordinates (vectorised, wraps silently)."""
    with np.errstate(over="ignore"):
        h = (np.ascontiguousarray(ix).view(np.uint64) * np.uint64(0x9E3779B185EBCA87)
             ^ np.ascontiguousarray(iy).view(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F)
             ^ np.ascontiguousarray(iz).view(np.uint64) * np.uint64(0x165667B19E3779F9)
             ^ np.uint64(int(seed) & 0xFFFFFFFFFFFF))
        h ^= h >> np.uint64(33)
        h *= np.uint64(0xFF51AFD7ED558CCD)
        h ^= h >> np.uint64(33)
        h *= np.uint64(0xC4CEB9FE1A85EC53)
        h ^= h >> np.uint64(33)
    return h


def worley(P, seed):
    """Cellular (Worley) noise F1, F2 in cell units for points P (N,3) given in cell coordinates."""
    P = np.asarray(P, np.float64)
    C = np.floor(P).astype(np.int64)
    f1 = np.full(len(P), np.inf)
    f2 = np.full(len(P), np.inf)
    inv = 1.0 / 65536.0
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                cx, cy, cz = C[:, 0] + dx, C[:, 1] + dy, C[:, 2] + dz
                h = _hash3(cx, cy, cz, seed)
                jx = (h & np.uint64(0xFFFF)).astype(np.float64) * inv
                jy = ((h >> np.uint64(16)) & np.uint64(0xFFFF)).astype(np.float64) * inv
                jz = ((h >> np.uint64(32)) & np.uint64(0xFFFF)).astype(np.float64) * inv
                d = ((P[:, 0] - cx - jx) ** 2 + (P[:, 1] - cy - jy) ** 2 + (P[:, 2] - cz - jz) ** 2)
                m = d < f1
                f2 = np.where(m, f1, np.minimum(f2, d))
                f1 = np.where(m, d, f1)
    return np.sqrt(f1), np.sqrt(f2)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# ----------------------------------------------------------------------------------------------
# Blender helpers
# ----------------------------------------------------------------------------------------------
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def link(obj):
    bpy.context.scene.collection.objects.link(obj)
    return obj


def set_active(obj, others=()):
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    for o in others:
        o.select_set(True)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def apply_modifiers(obj):
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    eo = obj.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(eo, preserve_all_data_layers=True, depsgraph=dg)
    old = obj.data
    name = old.name
    obj.modifiers.clear()
    obj.data = me
    bpy.data.meshes.remove(old)
    me.name = name


def get_verts(me):
    a = np.empty(len(me.vertices) * 3, np.float32)
    me.vertices.foreach_get("co", a)
    return a.reshape(-1, 3).astype(np.float64)


def set_verts(me, V):
    me.vertices.foreach_set("co", np.ascontiguousarray(V, np.float32).ravel())
    me.update()


def get_edges(me):
    a = np.empty(len(me.edges) * 2, np.int32)
    me.edges.foreach_get("vertices", a)
    return a.reshape(-1, 2).astype(np.int64)


def get_tris(me):
    me.calc_loop_triangles()
    a = np.empty(len(me.loop_triangles) * 3, np.int32)
    me.loop_triangles.foreach_get("vertices", a)
    return a.reshape(-1, 3).astype(np.int64)


def get_vertex_normals(me):
    a = np.empty(len(me.vertices) * 3, np.float32)
    me.vertex_normals.foreach_get("vector", a)
    return a.reshape(-1, 3).astype(np.float64)


def mesh_from_np(name, V, F):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(map(float, v)) for v in V], [], [tuple(map(int, f)) for f in F])
    me.update()
    me.validate()
    return me


def triangulate_mesh(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method="BEAUTY", ngon_method="BEAUTY")
    bm.to_mesh(me)
    bm.free()
    me.update()


def mass_props(V, F):
    """Volume and volume centroid of a closed, consistently oriented triangle mesh."""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = v6.sum() / 6.0
    cen = ((a + b + c) * v6[:, None]).sum(0) / (4.0 * v6.sum())
    return float(vol), cen


def surface_area(V, F):
    return float(0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1).sum())


def taubin_smooth(V, E, w, iters, lam=0.5, mu=-0.53):
    """Taubin (lambda|mu) smoothing with per-vertex weights w in [0,1] (umbrella operator)."""
    n = len(V)
    deg = np.maximum(np.bincount(E.ravel(), minlength=n).astype(np.float64), 1.0)
    for _ in range(int(iters)):
        for f in (lam, mu):
            S = np.empty_like(V)
            for k in range(3):
                S[:, k] = (np.bincount(E[:, 0], weights=V[E[:, 1], k], minlength=n)
                           + np.bincount(E[:, 1], weights=V[E[:, 0], k], minlength=n))
            V = V + (f * w)[:, None] * (S / deg[:, None] - V)
    return V


def new_material(name):
    m = bpy.data.materials.new(name)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            m.use_nodes = True   # deprecated in 5.x (always node based) but required in 4.x
        except Exception:
            pass
    return m


def sock(coll, ident):
    for s in coll:
        if s.identifier == ident:
            return s
    raise KeyError(ident)


class NodeBuilder:
    """Tiny helper to build shader node graphs with Python values or sockets as inputs."""

    def __init__(self, mat):
        self.nt = mat.node_tree
        self.nt.nodes.clear()

    def node(self, typ, **props):
        n = self.nt.nodes.new(typ)
        for k, v in props.items():
            setattr(n, k, v)
        return n

    def put(self, socket, val):
        if isinstance(val, bpy.types.NodeSocket):
            self.nt.links.new(val, socket)
        elif val is not None:
            socket.default_value = val

    def math(self, op, a, b=None, clamp=False):
        n = self.node("ShaderNodeMath", operation=op, use_clamp=clamp)
        self.put(n.inputs[0], a)
        if b is not None:
            self.put(n.inputs[1], b)
        return n.outputs[0]

    def maprange(self, v, f0, f1, t0=0.0, t1=1.0, smooth=False, clamp=True):
        n = self.node("ShaderNodeMapRange", clamp=clamp,
                      interpolation_type="SMOOTHSTEP" if smooth else "LINEAR")
        self.put(n.inputs["Value"], v)
        self.put(n.inputs["From Min"], f0)
        self.put(n.inputs["From Max"], f1)
        self.put(n.inputs["To Min"], t0)
        self.put(n.inputs["To Max"], t1)
        return n.outputs["Result"]

    def mix(self, fac, a, b):
        n = self.node("ShaderNodeMix", data_type="RGBA", blend_type="MIX")
        self.put(sock(n.inputs, "Factor_Float"), fac)
        self.put(sock(n.inputs, "A_Color"), a)
        self.put(sock(n.inputs, "B_Color"), b)
        return sock(n.outputs, "Result_Color")

    def mixf(self, fac, a, b):
        n = self.node("ShaderNodeMix", data_type="FLOAT", blend_type="MIX")
        self.put(sock(n.inputs, "Factor_Float"), fac)
        self.put(sock(n.inputs, "A_Float"), a)
        self.put(sock(n.inputs, "B_Float"), b)
        return sock(n.outputs, "Result_Float")

    def noise(self, vec, scale, detail=2.0, rough=0.5, distortion=0.0):
        n = self.node("ShaderNodeTexNoise")
        self.put(n.inputs["Vector"], vec)
        self.put(n.inputs["Scale"], scale)
        self.put(n.inputs["Detail"], detail)
        self.put(n.inputs["Roughness"], rough)
        self.put(n.inputs["Distortion"], distortion)
        return n

    def voronoi(self, vec, scale, feature="F1", metric="EUCLIDEAN", randomness=1.0):
        n = self.node("ShaderNodeTexVoronoi", feature=feature, distance=metric)
        self.put(n.inputs["Vector"], vec)
        self.put(n.inputs["Scale"], scale)
        self.put(n.inputs["Randomness"], randomness)
        return n

    def rgb(self, c):
        return (float(c[0]), float(c[1]), float(c[2]), 1.0)


# ----------------------------------------------------------------------------------------------
# Rock parameter sampling
# ----------------------------------------------------------------------------------------------
def sample_specs(args, class_cfg=None):
    """Per-rock parameter sets. class_cfg overrides CLASS_CFG (used by rock_library.py)."""
    specs = []
    class_cfg = class_cfg or CLASS_CFG
    for cls, n in (("large", args.n_large), ("medium", args.n_medium), ("pebble", args.n_pebble)):
        cfg = class_cfg[cls]
        cyc_rng = np.random.default_rng([args.seed, CLASS_IDS[cls], 999])
        offset = int(cyc_rng.integers(0, len(LITHO_CYCLE)))
        for i in range(n):
            seed = int(np.random.default_rng([args.seed, CLASS_IDS[cls], i]).integers(1, 2 ** 31 - 1))
            rng = np.random.default_rng(seed)
            lo, hi = cfg["size"]
            u = (i + rng.uniform(0.15, 0.85)) / max(n, 1)
            a = math.exp(math.log(lo) + u * (math.log(hi) - math.log(lo)))
            ba = float(np.clip(rng.normal(0.72, 0.09), 0.52, 0.95))
            ca = float(np.clip(rng.normal(0.52, 0.08), 0.32, 0.92 * ba))
            axes = (a, a * ba, a * ca)
            if cls == "pebble":
                tris = int(rng.integers(cfg["tris"][0], cfg["tris"][1] + 1))
            else:
                tris = int(round(cfg["tris"][0] + u * (cfg["tris"][1] - cfg["tris"][0])))
            roundness = float(rng.beta(2.0, 3.0))
            litho = LITHO_CYCLE[(offset + i) % len(LITHO_CYCLE)]
            # --- fracture planes: flattened base first, then big fractures, then small chips
            cuts = []
            nb = np.array([rng.normal(0, 0.12), rng.normal(0, 0.12), -1.0])
            cuts.append((tuple(nb / np.linalg.norm(nb)), float(rng.uniform(0.06, 0.16))))
            normals = [cuts[0][0]]
            n_big = int(rng.integers(cfg["cuts"][0], cfg["cuts"][1] + 1))
            tries = 0
            while len(normals) < n_big + 1 and tries < 500:
                tries += 1
                v = rng.normal(size=3)
                v /= np.linalg.norm(v)
                if max(float(np.dot(v, np.array(o))) for o in normals) > math.cos(math.radians(32)):
                    continue
                frac = float(rng.uniform(0.10, 0.38))
                if v[2] > 0.6:          # avoid big flat tops: the exposed top is eroded/rounded
                    frac *= 0.4
                normals.append(tuple(v))
                cuts.append((tuple(v), frac))
            for _ in range(int(rng.integers(cfg["chips"][0], cfg["chips"][1] + 1))):
                v = rng.normal(size=3)
                v /= np.linalg.norm(v)
                cuts.append((tuple(v), float(rng.uniform(0.01, 0.055))))
            n_vox = cfg["n_vox"][0] + roundness * (cfg["n_vox"][1] - cfg["n_vox"][0])
            L = LITHOLOGIES[litho]
            P = dict(
                seed=seed, axes=axes, tris=tris, roundness=roundness, litho=litho,
                # lump_amp = 1-sigma relative radius perturbation of the base shape (low frequency)
                lump_amp=float(rng.uniform(0.015, 0.04)), lump_freq=float(rng.uniform(0.9, 1.5)),
                super_p=float(rng.uniform(2.0, 2.8)),
                cuts=cuts, ico=cfg["ico"], voxel=a / n_vox, n_vox=n_vox,
                top_round=0.35 + 0.65 * roundness, smooth_iters=int(round(2 + 6 * roundness)),
                hi_faces=cfg["hi_faces"],
                # displacement std as a fraction of a (peaks ~3x); kept small so fracture facets stay planar
                rough_amp=float(rng.uniform(0.0011, 0.0022)) * (0.7 if cls == "pebble" else 1.0),
                rough_freq=float(rng.uniform(6.0, 10.0)),
                crack_cells=float(rng.uniform(1.6, 3.0)) if (cls != "pebble" and rng.uniform() < 0.4) else 0.0,
                crack_width=float(rng.uniform(0.02, 0.04)), crack_depth=float(rng.uniform(0.001, 0.0025)),
                n_pits=int(rng.integers(1, 5)) if (cls != "pebble" and rng.uniform() < 0.35) else 0,
                # appearance
                matrix=float(rng.uniform(*L["matrix"])), clast_frac=float(rng.uniform(*L["clast_frac"])),
                glass=float(rng.uniform(*L["glass"])), veins=bool(rng.uniform() < L["veins"]),
                dust=float(rng.uniform(*L["dust"])), rough=float(rng.uniform(*L["rough"])),
                glass_dir=tuple(float(x) for x in rng.normal(size=3)),
                tex_offset=tuple(float(x) for x in rng.uniform(-100, 100, 3)),
                burial=float(rng.uniform(*cfg["burial"])),
            )
            specs.append(dict(id=f"rock_{cls}_{i:02d}", cls=cls, idx=i, params=P))
    if args.only:
        keep = set(args.only.split(","))
        specs = [s for s in specs if s["id"] in keep]
    return specs


# ----------------------------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------------------------
def _cut(bm, n, frac):
    """Remove the cap of the shape beyond a plane with normal n; `frac` = cut depth / extent along n."""
    n = np.asarray(n, np.float64)
    V = np.array([v.co[:] for v in bm.verts])
    proj = V @ n
    h, lo = float(proj.max()), float(proj.min())
    d = h - frac * (h - lo)
    geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
    bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-7, plane_co=tuple(n * d), plane_no=tuple(n),
                           clear_outer=True, clear_inner=False)
    bnd = [e for e in bm.edges if e.is_boundary]
    if bnd:
        bmesh.ops.holes_fill(bm, edges=bnd, sides=0)


def surface_detail(V, N, P, edge_len):
    """Normal displacement field [m]: fBm roughness, cellular fracture cracks, pits on the top."""
    a = P["axes"][0]
    seed = P["seed"]
    f0 = P["rough_freq"] / a
    octv = int(np.clip(1 + math.floor(math.log2(max(1.0, 1.0 / (3.0 * edge_len * f0)))), 1, 7))
    disp = P["rough_amp"] * a * 3.0 * fbm(Noise3(seed + 21), V * f0, octv)
    if P["crack_cells"] > 0:
        # cellular fracture network, mildly domain-warped (irregular traces), gated (cracks fade in/out),
        # narrow V-shaped profile; half-width ~0.4 % of a but >= 2.5 high-poly edges
        Pc = V * (P["crack_cells"] / a) + 7.7
        warp = np.stack([fbm(Noise3(seed + 32 + k), V * (2.5 / a), 2) for k in range(3)], axis=1)
        F1, F2 = worley(Pc + 0.15 * warp, seed + 31)
        w = max(P["crack_width"] * 0.004 / 0.03, 2.5 * edge_len / a) * P["crack_cells"]
        g = np.clip(1.0 - (F2 - F1) / w, 0.0, 1.0) ** 1.5
        gate = smoothstep(-0.05, 0.25, fbm(Noise3(seed + 36), V * (1.7 / a), 2))
        disp -= P["crack_depth"] * a * g * gate
    if P["n_pits"] > 0:
        rng = np.random.default_rng(seed + 41)
        z = V[:, 2]
        zr = (z - z.min()) / max(np.ptp(z), 1e-9)
        cand = np.nonzero((zr > 0.45) & (N[:, 2] > 0.25))[0]
        if len(cand):
            for c in rng.choice(cand, size=min(P["n_pits"], len(cand)), replace=False):
                r = a * rng.uniform(0.006, 0.018)
                r = max(r, 3.0 * edge_len)
                dep = r * rng.uniform(0.15, 0.30)
                d2 = ((V - V[c]) ** 2).sum(1) / (r * r)
                m = d2 < 1.0
                disp[m] -= dep * (1.0 - d2[m]) ** 2
    return disp, octv


def build_high_poly(rid, P):
    a, b, c = P["axes"]
    st = {}
    bm = bmesh.new()
    bmesh.ops.create_icosphere(bm, subdivisions=P["ico"], radius=1.0)
    V = np.array([v.co[:] for v in bm.verts], np.float64)
    D = V / np.linalg.norm(V, axis=1, keepdims=True)
    # superellipsoid radius along each direction (exponent p > 2 -> boxier, blockier base)
    S = 0.5 * np.array([a, b, c])
    p = P["super_p"]
    rad = ((np.abs(D / S) ** p).sum(1)) ** (-1.0 / p)
    R = rad * (1.0 + P["lump_amp"] * 3.0 * fbm(Noise3(P["seed"] + 11), D * P["lump_freq"], 3))
    V = D * R[:, None]
    for v, p in zip(bm.verts, V):
        v.co = p
    for n, frac in P["cuts"]:
        _cut(bm, n, frac)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    # cuts shrink the shape unevenly; an affine rescale (planes stay planes) restores the sampled
    # axial ratios a:b:c as bounding-box extents
    Vc = np.array([v.co[:] for v in bm.verts])
    sc3 = np.array([a, b, c]) / np.maximum(Vc.max(0) - Vc.min(0), 1e-9)
    for v in bm.verts:
        v.co = (v.co.x * sc3[0], v.co.y * sc3[1], v.co.z * sc3[2])
    st["cut_rescale"] = [round(float(x), 3) for x in sc3]
    me = bpy.data.meshes.new(rid + "_hi")
    bm.to_mesh(me)
    bm.free()
    obj = link(bpy.data.objects.new(rid + "_hi", me))
    # voxel remesh -> uniform quads, edges rounded at ~voxel scale
    mod = obj.modifiers.new("remesh", "REMESH")
    mod.mode = "VOXEL"
    mod.voxel_size = P["voxel"]
    mod.adaptivity = 0.0
    apply_modifiers(obj)
    st["remesh_quads"] = len(obj.data.polygons)
    # height-weighted Taubin smoothing: rounded exposed top, angular base
    V = get_verts(obj.data)
    E = get_edges(obj.data)
    z = V[:, 2]
    zr = (z - z.min()) / max(np.ptp(z), 1e-9)
    w = np.clip(0.15 + P["top_round"] * smoothstep(0.30, 0.85, zr), 0.0, 1.0)
    V = taubin_smooth(V, E, w, P["smooth_iters"])
    set_verts(obj.data, V)
    # subdivide to the bake resolution
    nq = max(len(obj.data.polygons), 1)
    # level whose triangle count (2 * quads * 4^L) is closest (in log) to the target, capped at 1.6x target
    L = int(np.clip(round(math.log(max(P["hi_faces"] / (2.0 * nq), 1.0), 4)), 0, 4))
    while L > 0 and 2.0 * nq * 4 ** L > 1.6 * P["hi_faces"]:
        L -= 1
    if L > 0:
        mod = obj.modifiers.new("subsurf", "SUBSURF")
        mod.levels = L
        mod.render_levels = L
        apply_modifiers(obj)
    st["subsurf_levels"] = L
    me = obj.data
    V = get_verts(me)
    N = get_vertex_normals(me)
    edge_len = P["voxel"] / (2 ** L)
    disp, octv = surface_detail(V, N, P, edge_len)
    V = V + N * disp[:, None]
    F = get_tris(me)
    vol, cen = mass_props(V, F)
    V = V - cen
    s_final = a / float((V.max(0) - V.min(0)).max())       # longest extent == a exactly
    V = V * s_final
    vol *= s_final ** 3
    st["final_rescale"] = round(s_final, 4)
    set_verts(me, V)
    me.shade_smooth()
    st.update(hi_tris=int(len(F)), hi_verts=int(len(V)), hi_volume_m3=vol, hi_edge_m=edge_len * s_final,
              fbm_octaves=octv, disp_rms_m=float(np.sqrt((disp ** 2).mean())))
    return obj, st


def convex_hull_np(P, fix_convex=False):
    """Exact convex hull (bmesh quickhull) -> (V, F) with outward triangles.
    fix_convex: flip slightly reflex edges (only needed for final output hulls; O(iter * edges))."""
    bm = bmesh.new()
    vs = [bm.verts.new(tuple(map(float, p))) for p in P]
    res = bmesh.ops.convex_hull(bm, input=vs, use_existing_faces=False)
    rem = list({g for g in res["geom_interior"] + res["geom_unused"] if isinstance(g, bmesh.types.BMVert)})
    if rem:
        bmesh.ops.delete(bm, geom=rem, context="VERTS")
    loose = [v for v in bm.verts if not v.link_faces]
    if loose:
        bmesh.ops.delete(bm, geom=loose, context="VERTS")
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    # bmesh's hull can leave nearly coplanar n-gons whose triangulation is very slightly reflex
    # (measured ~2e-5 m on a 0.5 m rock); flip such edges until the triangulation is strictly convex
    for _ in range(500 if fix_convex else 0):
        worst, wd = None, 1e-12
        for e in bm.edges:
            if len(e.link_faces) != 2:
                continue
            f1, f2 = e.link_faces
            vo = next(v for v in f2.verts if v not in e.verts)
            d = (vo.co - f1.verts[0].co).dot(f1.normal)
            if d > wd:
                worst, wd = e, d
        if worst is None:
            break
        bmesh.ops.rotate_edges(bm, edges=[worst], use_ccw=False)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        for f in bm.faces:
            f.normal_update()
    bm.verts.index_update()
    V = np.array([v.co[:] for v in bm.verts], np.float64)
    F = np.array([[v.index for v in f.verts] for f in bm.faces], np.int64)
    bm.free()
    return V, F


def _planes(V, F):
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(B - A, C - A)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-15)
    off = (n * A).sum(1)
    cen = V.mean(0)
    s = np.where(n @ cen - off > 0, -1.0, 1.0)     # make normals point away from the centroid
    return n * s[:, None], off * s


def greedy_hull(P, kmax, tol=0.0):
    """Greedy farthest-point convex hull approximation with <= kmax vertices (inside the exact hull)."""
    P = np.asarray(P, np.float64)
    i0 = int(np.argmin(P[:, 0]))
    i1 = int(np.argmax(((P - P[i0]) ** 2).sum(1)))
    d = P[i1] - P[i0]
    d /= np.linalg.norm(d)
    X = P - P[i0]
    i2 = int(np.argmax(np.linalg.norm(X - np.outer(X @ d, d), axis=1)))
    nrm = np.cross(P[i1] - P[i0], P[i2] - P[i0])
    nrm /= np.linalg.norm(nrm)
    i3 = int(np.argmax(np.abs(X @ nrm)))
    sel = list(dict.fromkeys([i0, i1, i2, i3]))
    while True:
        V, F = convex_hull_np(P[sel])
        n, off = _planes(V, F)
        dist = (P @ n.T - off).max(1)
        j = int(np.argmax(dist))
        dmax = float(dist[j])
        if len(sel) >= kmax or dmax <= tol or j in sel:
            V, F = convex_hull_np(P[sel], fix_convex=True)
            return V, F, dmax
        sel.append(j)


def make_lod(hi, rid, cls, P, cfg):
    st = {}
    if cls == "pebble":
        Vh = get_verts(hi.data)
        cand, _ = convex_hull_np(Vh)
        k = (P["tris"] + 4) // 2
        Vl, Fl, dmax = greedy_hull(cand, k)
        lo = link(bpy.data.objects.new(rid, mesh_from_np(rid, Vl, Fl)))
        st["lod_method"] = f"greedy convex hull k={k}"
    else:
        me = hi.data.copy()
        me.name = rid
        lo = link(bpy.data.objects.new(rid, me))
        hi_tris = len(get_tris(me))
        mod = lo.modifiers.new("decimate", "DECIMATE")
        mod.decimate_type = "COLLAPSE"
        mod.ratio = min(1.0, P["tris"] / hi_tris)
        mod.use_collapse_triangulate = True
        apply_modifiers(lo)
        triangulate_mesh(lo.data)
        st["lod_method"] = f"decimate COLLAPSE ratio={P['tris'] / hi_tris:.5f}"
    me = lo.data
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(cfg["sharp_deg"]))
    st["lod_tris"] = len(get_tris(me))
    st["lod_verts"] = len(me.vertices)
    return lo, st


def lod_error(hi, lo, max_samples=150000):
    """Signed distance of high-poly vertices to the LOD surface (+ = high-poly outside the LOD)."""
    Vl = get_verts(lo.data)
    Fl = get_tris(lo.data)
    bvh = BVHTree.FromPolygons([tuple(v) for v in Vl], [tuple(f) for f in Fl])
    Vh = get_verts(hi.data)
    step = max(1, len(Vh) // max_samples)
    sd = []
    for p in Vh[::step]:
        q, n, _, dist = bvh.find_nearest(mathutils.Vector(p))
        if q is None:
            continue
        s = 1.0 if (mathutils.Vector(p) - q).dot(n) >= 0 else -1.0
        sd.append(s * dist)
    sd = np.array(sd)
    ad = np.abs(sd)
    return dict(mean_abs_mm=1e3 * float(ad.mean()), rms_mm=1e3 * float(np.sqrt((sd ** 2).mean())),
                p99_mm=1e3 * float(np.percentile(ad, 99)), max_mm=1e3 * float(ad.max()),
                max_out_mm=1e3 * float(max(sd.max(), 0.0)), max_in_mm=1e3 * float(max(-sd.min(), 0.0)),
                samples=int(len(sd)))


def outside_distance(P, Vh, Fh):
    """Max/mean distance of points P outside the closed convex mesh (Vh, Fh)."""
    n, off = _planes(Vh, Fh)
    outside = (P @ n.T - off).max(1) > 1e-9
    if not outside.any():
        return 0.0, 0.0
    bvh = BVHTree.FromPolygons([tuple(v) for v in Vh], [tuple(f) for f in Fh])
    ds = [bvh.find_nearest(mathutils.Vector(p))[3] for p in P[outside]]
    return float(max(ds)), float(np.mean(ds))


# ----------------------------------------------------------------------------------------------
# UV + bake
# ----------------------------------------------------------------------------------------------
def uv_unwrap(lo, gap_frac):
    set_active(lo)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    # smart project packs the islands itself; an extra uv.pack_islands (CONVEX/CONCAVE) took 7-140 s
    # per rock in Blender 5.2 without better coverage, so it is not used.
    bpy.ops.uv.smart_project(angle_limit=math.radians(66.0), margin_method="FRACTION", island_margin=gap_frac,
                             area_weight=0.0, correct_aspect=True, scale_to_bounds=False)
    bpy.ops.object.mode_set(mode="OBJECT")
    me = lo.data
    uv = np.empty(len(me.loops) * 2, np.float32)
    me.uv_layers.active.data.foreach_get("uv", uv)
    uv = uv.reshape(-1, 2).astype(np.float64)
    me.calc_loop_triangles()
    tl = np.empty(len(me.loop_triangles) * 3, np.int32)
    me.loop_triangles.foreach_get("loops", tl)
    tl = tl.reshape(-1, 3)
    a, b, c = uv[tl[:, 0]], uv[tl[:, 1]], uv[tl[:, 2]]
    uv_area = float(0.5 * np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])).sum())
    return uv_area


def new_image(name, size, colorspace, fill):
    img = bpy.data.images.new(name, size, size, alpha=False, float_buffer=False)
    img.colorspace_settings.name = colorspace
    img.generated_color = (float(fill[0]), float(fill[1]), float(fill[2]), 1.0)
    return img


def img_to_u8(img):
    w, h = img.size
    a = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(a)
    return np.round(a.reshape(h, w, 4) * 255.0).astype(np.uint8)


def u8_to_img(img, arr):
    img.pixels.foreach_set((arr.astype(np.float32) / 255.0).ravel())
    img.update()


def save_png(img, path):
    img.filepath_raw = path
    img.file_format = "PNG"
    img.save()


def build_breccia_material(rid, P, texel_m):
    """Procedural Apollo-16 breccia (bake source only; evaluated on the high-poly surface)."""
    mat = new_material(rid + "_breccia")
    nb = NodeBuilder(mat)
    out = nb.node("ShaderNodeOutputMaterial")
    bsdf = nb.node("ShaderNodeBsdfPrincipled")
    nb.put(bsdf.inputs["Specular IOR Level"], 0.0)   # DIFFUSE/COLOR bake == albedo exactly
    nb.nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    tc = nb.node("ShaderNodeTexCoord")
    mp = nb.node("ShaderNodeMapping")
    mp.inputs["Location"].default_value = P["tex_offset"]
    nb.put(mp.inputs["Vector"], tc.outputs["Object"])
    Pv = mp.outputs["Vector"]
    geo = nb.node("ShaderNodeNewGeometry")
    sep_n = nb.node("ShaderNodeSeparateXYZ")
    nb.put(sep_n.inputs["Vector"], geo.outputs["Normal"])
    up = sep_n.outputs["Z"]
    L = LITHOLOGIES[P["litho"]]
    grain = max(0.0015, 2.5 * texel_m)       # smallest resolvable feature size [m]

    def warp(vec, size, amp):
        """vec + (noise_color - 0.5) * amp: domain warp at feature size `size` [m]."""
        dn = nb.noise(vec, 1.0 / size, detail=2.0)
        dvec = nb.node("ShaderNodeVectorMath", operation="SUBTRACT")
        nb.put(dvec.inputs[0], dn.outputs["Color"])
        nb.put(dvec.inputs[1], (0.5, 0.5, 0.5))
        dsc = nb.node("ShaderNodeVectorMath", operation="SCALE")
        nb.put(dsc.inputs[0], dvec.outputs["Vector"])
        nb.put(dsc.inputs["Scale"], amp)
        padd = nb.node("ShaderNodeVectorMath", operation="ADD")
        nb.put(padd.inputs[0], vec)
        nb.put(padd.inputs[1], dsc.outputs["Vector"])
        return padd.outputs["Vector"]

    def gray(val):
        cmb = nb.node("ShaderNodeCombineColor")
        for ch in ("Red", "Green", "Blue"):
            nb.put(cmb.inputs[ch], val)
        return cmb.outputs["Color"]

    def multiply(c, val):
        mm = nb.node("ShaderNodeMix", data_type="RGBA", blend_type="MULTIPLY")
        nb.put(sock(mm.inputs, "Factor_Float"), 1.0)
        nb.put(sock(mm.inputs, "A_Color"), c)
        nb.put(sock(mm.inputs, "B_Color"), gray(val))
        return sock(mm.outputs, "Result_Color")

    # matrix: fine grains + cm-scale mottling
    m = P["matrix"]
    gn = nb.noise(Pv, 1.0 / grain, detail=3.0, rough=0.6)
    gfac = nb.maprange(gn.outputs["Fac"], 0.3, 0.7)
    col = nb.mix(gfac, nb.rgb(MATRIX_TINT * m * 0.78), nb.rgb(MATRIX_TINT * m * 1.22))
    mot = nb.noise(Pv, 1.0 / 0.05, detail=3.0)
    col = multiply(col, nb.maprange(mot.outputs["Fac"], 0.35, 0.65, 0.88, 1.12))
    height = nb.math("MULTIPLY", gn.outputs["Fac"], 0.5)

    # clast layers (small -> large; bigger clasts painted on top). Each layer: Voronoi cells on a
    # domain-warped coordinate (irregular, angular outlines); a random subset of cells becomes clasts,
    # shrunk away from the cell edges so matrix shows between them; clast lithology from a palette,
    # per-clast brightness jitter, internal grain; contrast vs. matrix damped (dust-coated look).
    for li, (cell, frac) in enumerate(CLAST_LAYERS):
        if cell < 3.0 * texel_m:
            continue
        f = float(np.clip(frac * P["clast_frac"], 0.0, 0.9))
        Pw = warp(Pv, 0.9 * cell, 0.30 * cell)
        v1 = nb.voronoi(Pw, 1.0 / cell, "F1")
        v2 = nb.voronoi(Pw, 1.0 / cell, "DISTANCE_TO_EDGE")
        sc = nb.node("ShaderNodeSeparateColor")
        nb.put(sc.inputs["Color"], v1.outputs["Color"])
        sel = nb.maprange(sc.outputs["Red"], 1.0 - f - 0.001, 1.0 - f + 0.001)
        gap = 0.05 + 0.10 * (li == 0)
        inner = nb.maprange(v2.outputs["Distance"], gap, gap + 0.025, smooth=True)
        mask = nb.math("MULTIPLY", nb.math("MULTIPLY", sel, inner), 0.85)
        ramp = nb.node("ShaderNodeValToRGB")
        cr = ramp.color_ramp
        cr.interpolation = "CONSTANT"
        stops = L["ramp"]
        while len(cr.elements) < len(stops):
            cr.elements.new(0.5)
        for el, (pos, alb) in zip(cr.elements, stops):
            el.position = pos
            el.color = nb.rgb(CLAST_TINT * alb)
        nb.put(ramp.inputs["Fac"], sc.outputs["Green"])
        ccol = multiply(ramp.outputs["Color"], nb.maprange(sc.outputs["Blue"], 0.0, 1.0, 0.82, 1.15))
        ccol = multiply(ccol, nb.maprange(gn.outputs["Fac"], 0.3, 0.7, 0.88, 1.12))
        col = nb.mix(mask, col, ccol)
        height = nb.math("ADD", height, nb.math("MULTIPLY", mask, 0.5))

    # dark glass veinlets: thin, wandering zero-crossing lines of a warped noise, present only in
    # some regions (gated) -> sparse, irregular veins instead of a regular network
    if P["veins"]:
        Pvw = warp(Pv, 0.08, 0.02)
        vn = nb.noise(Pvw, 1.0 / 0.12, detail=3.0, rough=0.55)
        dist = nb.math("ABSOLUTE", nb.math("SUBTRACT", vn.outputs["Fac"], 0.5))
        vw = float(np.clip(2.0 * texel_m, 0.003, 0.012))
        vein = nb.maprange(dist, vw, 0.0, smooth=True)
        vreg = nb.noise(Pv, 1.0 / 0.35, detail=2.0)
        vein = nb.math("MULTIPLY", vein, nb.maprange(vreg.outputs["Fac"], 0.54, 0.60))
        col = nb.mix(vein, col, nb.rgb(GLASS_COLOR))
        height = nb.math("SUBTRACT", height, nb.math("MULTIPLY", vein, 0.6))
    else:
        vein = 0.0

    # glass coating / splash (biased to one side)
    glass = 0.0
    if P["glass"] > 0.01:
        gd = np.array(P["glass_dir"])
        gd = gd / np.linalg.norm(gd)
        dot = nb.node("ShaderNodeVectorMath", operation="DOT_PRODUCT")
        nb.put(dot.inputs[0], geo.outputs["Normal"])
        nb.put(dot.inputs[1], tuple(gd))
        gnz = nb.noise(Pv, 1.0 / 0.20, detail=6.0, rough=0.6)
        gsum = nb.math("ADD", gnz.outputs["Fac"], nb.math("MULTIPLY", dot.outputs["Value"], 0.12))
        thr = 0.5 + 0.25 * (0.5 - P["glass"])
        glass = nb.maprange(gsum, thr, thr + 0.03, smooth=True)
        col = nb.mix(glass, col, nb.rgb(GLASS_COLOR))

    # dust: more on upward faces, patchy
    dz = nb.noise(Pv, 1.0 / 0.07, detail=4.0)
    dpatch = nb.maprange(dz.outputs["Fac"], 0.35, 0.65)
    dup = nb.maprange(up, -0.3, 0.9, 0.35, 1.0)
    dust = nb.math("MULTIPLY", nb.math("MULTIPLY", dpatch, dup), P["dust"], clamp=True)
    col = nb.mix(dust, col, nb.rgb(MATRIX_TINT * DUST_ALBEDO))
    nb.put(bsdf.inputs["Base Color"], col)

    # roughness
    rn = nb.maprange(mot.outputs["Fac"], 0.3, 0.7, P["rough"] - 0.04, P["rough"] + 0.04)
    if not isinstance(glass, float):
        rn = nb.mixf(glass, rn, 0.32)
    if not isinstance(vein, float):
        rn = nb.mixf(vein, rn, 0.40)
    rn = nb.mixf(dust, rn, 0.97)
    nb.put(bsdf.inputs["Roughness"], rn)

    # micro relief for the normal map
    if not isinstance(glass, float):
        height = nb.math("MULTIPLY", height, nb.maprange(glass, 0.0, 1.0, 1.0, 0.3))
    bump = nb.node("ShaderNodeBump")
    nb.put(bump.inputs["Strength"], 1.0)
    nb.put(bump.inputs["Distance"], 0.08 * grain)
    nb.put(bump.inputs["Height"], height)
    nb.put(bsdf.inputs["Normal"], bump.outputs["Normal"])
    mean_alb = (m * (1 - 0.3 * P["clast_frac"]) + 0.3 * P["clast_frac"] * np.mean([s[1] for s in L["ramp"]]))
    mean_alb = (1 - 0.5 * P["dust"]) * mean_alb + 0.5 * P["dust"] * DUST_ALBEDO
    return mat, float(mean_alb)


def lin2srgb(x):
    x = np.asarray(x, np.float64)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(np.maximum(x, 0.0), 1.0 / 2.4) - 0.055)


def bake_maps(hi, lo, rid, R, margin, ext, ray, P, mean_alb):
    # NB: Image.generated_color is written to the byte buffer as-is (no linear->sRGB conversion),
    # so the base-colour fill (seen only through mip-map bleeding) must be given sRGB-encoded.
    imgs = {
        "basecolor": new_image(rid + "_basecolor", R, "sRGB", lin2srgb(MATRIX_TINT * mean_alb)),
        "normal": new_image(rid + "_normal", R, "Non-Color", (0.5, 0.5, 1.0)),
        "rough": new_image(rid + "_rough", R, "Non-Color", (P["rough"],) * 3),
    }
    mat = new_material(rid + "_bake")
    nb = NodeBuilder(mat)
    nb.node("ShaderNodeOutputMaterial")
    node = nb.node("ShaderNodeTexImage")
    lo.data.materials.clear()
    lo.data.materials.append(mat)
    T = {}
    for key, kind, kw in (("normal", "NORMAL", dict(normal_space="TANGENT")),
                          ("basecolor", "DIFFUSE", dict(pass_filter={"COLOR"})),
                          ("rough", "ROUGHNESS", {})):
        node.image = imgs[key]
        mat.node_tree.nodes.active = node
        set_active(lo, others=[hi])
        t = time.perf_counter()
        bpy.ops.object.bake(type=kind, use_selected_to_active=True, cage_extrusion=ext, max_ray_distance=ray,
                            margin=margin, margin_type="EXTEND", use_clear=False, target="IMAGE_TEXTURES", **kw)
        T[key] = time.perf_counter() - t
    lo.data.materials.clear()
    bpy.data.materials.remove(mat)
    return imgs, T


def make_pbr_material(name, img_base, img_nrm, img_orm):
    """glTF-friendly Principled setup: baseColor, tangent normal map, ORM (G=roughness, B=metallic)."""
    mat = new_material(name)
    mat.use_backface_culling = True     # closed meshes -> glTF doubleSided = false
    nb = NodeBuilder(mat)
    out = nb.node("ShaderNodeOutputMaterial")
    bsdf = nb.node("ShaderNodeBsdfPrincipled")
    nb.nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    tb = nb.node("ShaderNodeTexImage")
    tb.image = img_base
    nb.put(bsdf.inputs["Base Color"], tb.outputs["Color"])
    tn = nb.node("ShaderNodeTexImage")
    tn.image = img_nrm
    nm = nb.node("ShaderNodeNormalMap", space="TANGENT")
    nb.put(nm.inputs["Color"], tn.outputs["Color"])
    nb.put(bsdf.inputs["Normal"], nm.outputs["Normal"])
    to = nb.node("ShaderNodeTexImage")
    to.image = img_orm
    sp = nb.node("ShaderNodeSeparateColor")
    nb.put(sp.inputs["Color"], to.outputs["Color"])
    nb.put(bsdf.inputs["Roughness"], sp.outputs["Green"])
    nb.put(bsdf.inputs["Metallic"], sp.outputs["Blue"])
    return mat


# ----------------------------------------------------------------------------------------------
# Export helpers
# ----------------------------------------------------------------------------------------------
def export_glb(objs, path, materials=True, tangents=True, normals=True):
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.export_scene.gltf(
        filepath=path, export_format="GLB", use_selection=True, export_apply=True, export_yup=True,
        export_texcoords=materials, export_normals=normals, export_tangents=bool(tangents and materials),
        export_materials="EXPORT" if materials else "NONE", export_image_format="AUTO",
        export_animations=False, export_extras=False, export_cameras=False, export_lights=False,
        export_draco_mesh_compression_enable=False)
    return os.path.getsize(path)


def export_axis_marker(path):
    """Orientation test asset, exported exactly like the rocks (export_yup=True, identity node):
    one mesh, arms along Blender +X (0.30 m, red), +Y (0.20 m, green), +Z (0.10 m, blue), 2 cm thick,
    plus a 4 cm grey cube at the origin. A consumer that converts glTF +Y-up to Z-up correctly shows the
    blue arm pointing up and the red arm along its +X."""
    bm = bmesh.new()
    t = 0.02
    boxes = [((0.15, 0, 0), (0.30, t, t), 1), ((0, 0.10, 0), (t, 0.20, t), 2), ((0, 0, 0.05), (t, t, 0.10), 3),
             ((0, 0, 0), (0.04, 0.04, 0.04), 0)]
    for centre, size, mi in boxes:
        r = bmesh.ops.create_cube(bm, size=1.0)
        for v in r["verts"]:
            v.co = mathutils.Vector((centre[0] + v.co.x * size[0], centre[1] + v.co.y * size[1], centre[2] + v.co.z * size[2]))
        faces = {f for v in r["verts"] for f in v.link_faces}
        for f in faces:
            f.material_index = mi
    me = bpy.data.meshes.new("axis_marker")
    bm.to_mesh(me)
    bm.free()
    for name, rgb in (("grey", (0.5, 0.5, 0.5)), ("x_red", (0.9, 0.05, 0.05)), ("y_green", (0.05, 0.9, 0.05)),
                      ("z_blue", (0.05, 0.05, 0.9))):
        m = new_material("axis_" + name)
        nb = NodeBuilder(m)
        out_n = nb.node("ShaderNodeOutputMaterial")
        bsdf = nb.node("ShaderNodeBsdfPrincipled")
        nb.put(bsdf.inputs["Base Color"], (*rgb, 1.0))
        nb.nt.links.new(bsdf.outputs["BSDF"], out_n.inputs["Surface"])
        me.materials.append(m)
    obj = link(bpy.data.objects.new("axis_marker", me))
    size = export_glb([obj], path, tangents=False)
    bpy.data.objects.remove(obj)
    return size


def export_normalmap_test(path, tex_dir):
    """Normal-map convention test asset (same bake + export path as the rocks). Two 0.4 x 0.4 m tiles:
    x < 0: real geometry (dome + bar pointing +X + small bump at +Y), x > 0: flat 2-triangle tile with the
    baked tangent-space normal map of that relief. Under any light both tiles must shade alike; if the
    renderer expects the other green-channel convention, the right tile's +Y bump looks inverted (lit from
    the wrong side along Y) while the +X bar still looks right."""
    s, n = 0.4, 160
    xs = np.linspace(-s / 2, s / 2, n + 1)
    X, Y = np.meshgrid(xs, xs, indexing="xy")
    r = np.sqrt(X ** 2 + Y ** 2)
    Z = 0.03 * np.clip(1 - (r / 0.07) ** 2, 0, None) ** 0.5                                 # dome
    Z = np.maximum(Z, 0.015 * (np.abs(Y) < 0.012) * (X > 0) * (X < 0.16) * np.clip((0.16 - X) / 0.03, 0, 1))  # bar +X
    Z = np.maximum(Z, 0.02 * np.clip(1 - ((X ** 2 + (Y - 0.13) ** 2) / 0.025 ** 2), 0, None))  # bump at +Y
    V = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    F = []
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i
            F += [(a, a + 1, a + n + 2), (a, a + n + 2, a + n + 1)]
    hi = link(bpy.data.objects.new("nm_relief", mesh_from_np("nm_relief", V, np.array(F))))
    hi.data.shade_smooth()
    lo_me = mesh_from_np("nm_flat", np.array([(-s / 2, -s / 2, 0), (s / 2, -s / 2, 0), (s / 2, s / 2, 0), (-s / 2, s / 2, 0)]),
                         np.array([(0, 1, 2), (0, 2, 3)]))
    uvl = lo_me.uv_layers.new(name="UVMap")
    loops_v = np.empty(len(lo_me.loops), np.int32)
    lo_me.loops.foreach_get("vertex_index", loops_v)
    uv = np.array([(0, 0), (1, 0), (1, 1), (0, 1)], np.float32)[loops_v]
    uvl.data.foreach_set("uv", uv.ravel())
    lo = link(bpy.data.objects.new("nm_flat", lo_me))
    img = new_image("normalmap_test_normal", 512, "Non-Color", (0.5, 0.5, 1.0))
    mat = new_material("nm_bake")
    nb = NodeBuilder(mat)
    nb.node("ShaderNodeOutputMaterial")
    node = nb.node("ShaderNodeTexImage")
    node.image = img
    mat.node_tree.nodes.active = node
    lo.data.materials.append(mat)
    set_active(lo, others=[hi])
    bpy.ops.object.bake(type="NORMAL", normal_space="TANGENT", use_selected_to_active=True, cage_extrusion=0.05,
                        max_ray_distance=0.1, margin=4, use_clear=False, target="IMAGE_TEXTURES")
    save_png(img, os.path.join(tex_dir, "normalmap_test_normal.png"))
    grey = new_image("normalmap_test_base", 8, "sRGB", (0.6, 0.6, 0.6))
    orm = new_image("normalmap_test_orm", 8, "Non-Color", (1.0, 0.8, 0.0))
    lo.data.materials.clear()
    lo.data.materials.append(make_pbr_material("nm_flat_mat", grey, img, orm))
    plain = new_material("nm_relief_mat")
    nb = NodeBuilder(plain)
    o_ = nb.node("ShaderNodeOutputMaterial")
    b_ = nb.node("ShaderNodeBsdfPrincipled")
    nb.put(b_.inputs["Base Color"], (0.6 ** 2.2, 0.6 ** 2.2, 0.6 ** 2.2, 1.0))
    nb.put(b_.inputs["Roughness"], 0.8)
    nb.nt.links.new(b_.outputs["BSDF"], o_.inputs["Surface"])
    hi.data.materials.append(plain)
    # bake the side-by-side offsets into the vertices (identity node transforms, like the rocks)
    set_verts(hi.data, get_verts(hi.data) + np.array([-0.25, 0, 0]))
    set_verts(lo.data, get_verts(lo.data) + np.array([0.25, 0, 0]))
    size = export_glb([hi, lo], path)
    for o in (hi, lo):
        bpy.data.objects.remove(o)
    return size


def corner_arrays(me):
    """glTF-style vertex arrays (unique position/normal/uv corners) of a triangulated mesh, Z-up."""
    me.calc_loop_triangles()
    nl = len(me.loops)
    vi = np.empty(nl, np.int32)
    me.loops.foreach_get("vertex_index", vi)
    V = get_verts(me)
    cn = np.empty(nl * 3, np.float32)
    me.corner_normals.foreach_get("vector", cn)
    cn = cn.reshape(-1, 3)
    uv = np.empty(nl * 2, np.float32)
    me.uv_layers.active.data.foreach_get("uv", uv)
    uv = uv.reshape(-1, 2)
    tl = np.empty(len(me.loop_triangles) * 3, np.int32)
    me.loop_triangles.foreach_get("loops", tl)
    key = np.concatenate([vi[:, None].astype(np.float64), np.round(cn, 5), np.round(uv, 6)], axis=1)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.ravel()
    pos = V[vi[first]].astype(np.float32)
    nrm = cn[first].astype(np.float32)
    uvg = uv[first].astype(np.float32).copy()
    uvg[:, 1] = 1.0 - uvg[:, 1]                     # glTF UV convention (v down)
    idx = inv[tl].reshape(-1, 3).astype(np.uint32)
    return pos, nrm, uvg, idx


# ----------------------------------------------------------------------------------------------
# Atlas
# ----------------------------------------------------------------------------------------------
def allocate_atlas(sizes, S):
    """Quadtree (buddy) allocation of power-of-two squares; returns (x, y_bottom, size) per request."""
    free = [(0, 0, S)]
    out = [None] * len(sizes)
    for i in sorted(range(len(sizes)), key=lambda i: -sizes[i]):
        s = sizes[i]
        if s & (s - 1) or s > S:
            raise ValueError(f"region size {s} must be a power of two <= atlas {S}")
        cands = [f for f in free if f[2] >= s]
        if not cands:
            raise RuntimeError(f"atlas {S}^2 is full (request {s}^2); increase --atlas")
        f = min(cands, key=lambda f: (f[2], -f[1], f[0]))
        free.remove(f)
        x, y, fs = f
        while fs > s:
            fs //= 2
            free += [(x + fs, y, fs), (x, y + fs, fs), (x + fs, y + fs, fs)]
        out[i] = (x, y, s)
    return out


# ----------------------------------------------------------------------------------------------
# Main generation
# ----------------------------------------------------------------------------------------------
def setup_cycles(scene, device, samples):
    scene.render.engine = "CYCLES"
    used = []
    try:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        if device in ("OPTIX", "CUDA"):
            prefs.compute_device_type = device
            prefs.get_devices()
            for d in prefs.devices:
                d.use = (d.type == device)
                if d.use:
                    used.append(f"{d.name} ({d.type})")
        scene.cycles.device = "GPU" if used else "CPU"
    except Exception as e:  # pragma: no cover
        log("WARN cycles device setup failed:", e)
        scene.cycles.device = "CPU"
    if not used:
        used = ["CPU"]
    scene.cycles.samples = samples
    scene.cycles.use_denoising = False
    return used


def build_rock(spec, args, out):
    rid, cls, P = spec["id"], spec["cls"], spec["params"]
    cfg = CLASS_CFG[cls]
    T = {}
    t0 = time.perf_counter()
    hi, hst = build_high_poly(rid, P)
    T["high_poly_s"] = time.perf_counter() - t0
    t = time.perf_counter()
    lo, lst = make_lod(hi, rid, cls, P, cfg)
    T["lod_s"] = time.perf_counter() - t
    t = time.perf_counter()
    err = lod_error(hi, lo)
    T["lod_error_s"] = time.perf_counter() - t
    a = P["axes"][0]
    log(f"{rid}: {P['litho']} a={a:.3f} m round={P['roundness']:.2f} hi_tris={hst['hi_tris']} "
        f"lod_tris={lst['lod_tris']}/{P['tris']} "
        f"err rms={err['rms_mm']:.2f} mm max={err['max_mm']:.2f} mm ({T['high_poly_s']:.1f}+{T['lod_s']:.1f} s)")
    return hi, lo, dict(T=T, hst=hst, lst=lst, err=err)


def finish_rock(spec, hi, lo, info, args, out, devices):
    """UV, bake, standalone export, hull. Returns the metadata dict (atlas fields filled later)."""
    rid, cls, P = spec["id"], spec["cls"], spec["params"]
    cfg = CLASS_CFG[cls]
    T = info["T"]
    Vl = get_verts(lo.data)
    Fl = get_tris(lo.data)
    area = surface_area(Vl, Fl)
    # atlas region: smallest power of two that reaches the class texel-density target (assuming ~55 %
    # UV coverage), clamped to [region_min, --region-<cls>]
    R_max = getattr(args, f"region_{cls}")
    tex_mm = getattr(args, f"texel_mm_{cls}")
    if tex_mm > 0:
        R_est = math.sqrt(area / (0.55 * (tex_mm * 1e-3) ** 2))
        R = int(min(max(2 ** math.ceil(math.log2(max(R_est, 1.0))), cfg["region_min"]), R_max))
    else:
        R = R_max
    t = time.perf_counter()
    while True:
        # island gap scales with the region (R/128 px, 2..8 px) so small regions are not wasted on margins
        gap_px = int(min(max(R // 128, 2), cfg["gap_px"]))
        uv_area = uv_unwrap(lo, gap_px / R)
        texel = math.sqrt(area / max(uv_area * R * R, 1e-9))
        # real UV coverage can be far below the 55 % assumed above (many small islands): grow if needed
        if tex_mm > 0 and texel > 1.25 * tex_mm * 1e-3 and R < R_max:
            R *= 2
            continue
        break
    T["uv_s"] = time.perf_counter() - t
    mat_hi, mean_alb = build_breccia_material(rid, P, texel)
    hi.data.materials.clear()
    hi.data.materials.append(mat_hi)
    err = info["err"]
    ext = 1.3 * err["max_out_mm"] * 1e-3 + 0.002 * P["axes"][0]
    ray = ext + 1.3 * err["max_in_mm"] * 1e-3 + 0.002 * P["axes"][0]
    bpy.context.scene.cycles.samples = args.bake_samples
    t = time.perf_counter()
    imgs, TB = bake_maps(hi, lo, rid, R, max(gap_px, 2), ext, ray, P, mean_alb)
    T["bake_s"] = time.perf_counter() - t
    T["bake_detail_s"] = TB
    px = {k: img_to_u8(v) for k, v in imgs.items()}
    orm = np.empty_like(px["rough"])
    orm[..., 0] = 255
    orm[..., 1] = px["rough"][..., 0]
    orm[..., 2] = 0
    orm[..., 3] = 255
    px["orm"] = orm
    img_orm = new_image(rid + "_orm", R, "Non-Color", (1.0, P["rough"], 0.0))
    u8_to_img(img_orm, orm)
    texdir = os.path.join(out, "rocks", "tex")
    os.makedirs(texdir, exist_ok=True)
    for key, img in (("basecolor", imgs["basecolor"]), ("normal", imgs["normal"]), ("orm", img_orm)):
        save_png(img, os.path.join(texdir, f"{rid}_{key}.png"))
    mat = make_pbr_material(rid + "_mat", imgs["basecolor"], imgs["normal"], img_orm)
    lo.data.materials.clear()
    lo.data.materials.append(mat)
    t = time.perf_counter()
    glb = os.path.join(out, "rocks", rid + ".glb")
    size_glb = export_glb([lo], glb)
    T["export_s"] = time.perf_counter() - t
    if not args.keep_high:
        # free per-rock Blender data (pixels are kept in numpy for the atlas) so the session does not grow
        lo.data.materials.clear()
        hi.data.materials.clear()
        bpy.data.materials.remove(mat)
        bpy.data.materials.remove(mat_hi)
        for im in (imgs["basecolor"], imgs["normal"], imgs["rough"], img_orm):
            bpy.data.images.remove(im)
    # convex hull for collision
    hull = None
    t = time.perf_counter()
    if cfg["hull"] or args.hull_all:
        cand, _ = convex_hull_np(Vl)
        Vh, Fh, _ = greedy_hull(cand, args.hull_max_verts)
        vol_h, _ = mass_props(Vh, Fh)
        vol_l, _ = mass_props(Vl, Fl)
        dmax, dmean = outside_distance(Vl, Vh, Fh)
        ho = link(bpy.data.objects.new(rid + "_hull", mesh_from_np(rid + "_hull", Vh, Fh)))
        hpath = os.path.join(out, "hulls", rid + "_hull.glb")
        os.makedirs(os.path.dirname(hpath), exist_ok=True)
        hsize = export_glb([ho], hpath, materials=False, normals=False)
        hull = dict(file=os.path.relpath(hpath, out).replace("\\", "/"), vertices=int(len(Vh)),
                    triangles=int(len(Fh)), volume_m3=vol_h, volume_ratio_vs_lod=vol_h / vol_l,
                    lod_max_outside_mm=1e3 * dmax, lod_mean_outside_mm=1e3 * dmean,
                    file_bytes=hsize, verts_zup=np.round(Vh, 6).tolist(), faces=Fh.tolist(), _obj=ho)
    T["hull_s"] = time.perf_counter() - t
    vol_l, _ = mass_props(Vl, Fl)
    bmin, bmax = Vl.min(0), Vl.max(0)
    meta = dict(
        id=rid, cls=cls, seed=P["seed"], lithology=P["litho"], roundness=round(P["roundness"], 3),
        units="m", frame="rock-local, Z up, origin = volume centroid; glTF files are +Y up (x, z, -y)",
        axes_target_m=[round(x, 4) for x in P["axes"]],
        dims_m=[round(float(x), 4) for x in (bmax - bmin)],
        bbox_min_m=[round(float(x), 4) for x in bmin], bbox_max_m=[round(float(x), 4) for x in bmax],
        bottom_offset_m=round(float(-bmin[2]), 4), top_m=round(float(bmax[2]), 4),
        burial_fraction_range=list(cfg["burial"]), preview_burial_fraction=round(P["burial"], 3),
        volume_m3=vol_l, surface_area_m2=area,
        high_poly=info["hst"], lod=dict(info["lst"], error=info["err"], sharp_edge_deg=cfg["sharp_deg"]),
        texture=dict(region_px=R, island_gap_px=gap_px, uv_coverage=round(uv_area, 4), texel_mm=round(1e3 * texel, 3),
                     bake_cage_extrusion_m=ext, bake_max_ray_m=ray, bake_samples=args.bake_samples,
                     mean_albedo_linear=round(mean_alb, 4)),
        hull=hull, files=dict(standalone_glb=f"rocks/{rid}.glb", standalone_glb_bytes=size_glb,
                              textures=[f"rocks/tex/{rid}_{k}.png" for k in ("basecolor", "normal", "orm")]),
        timing_s=T,
    )
    meta["_px"] = px
    return meta


def disable_undo():
    """Operators called from a script push (memfile) undo steps; nothing here needs undo."""
    try:
        bpy.context.preferences.edit.use_global_undo = False
        bpy.context.preferences.edit.undo_steps = 0
    except Exception as e:  # pragma: no cover
        log("WARN could not disable undo:", e)


def generate(args, out):
    t_start = time.perf_counter()
    reset_scene()
    if not args.keep_undo:
        disable_undo()
    sc = bpy.context.scene
    devices = setup_cycles(sc, args.device, args.bake_samples)
    log("Cycles devices:", devices)
    for d in ("rocks", "hulls", "npz", "meta", "atlas", "preview"):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    specs = sample_specs(args)
    metas, los = [], []
    for spec in specs:
        t = time.perf_counter()
        hi, lo, info = build_rock(spec, args, out)
        meta = finish_rock(spec, hi, lo, info, args, out, devices)
        meta["timing_s"]["total_s"] = time.perf_counter() - t
        metas.append(meta)
        los.append(lo)
        # Cycles syncs every renderable object for each bake: hide finished rocks from rendering, otherwise
        # bake time grows with the library size (measured: pebble bake 2.0 s -> 6.7 s after 49 rocks)
        lo.hide_render = True
        if meta["hull"]:
            meta["hull"]["_obj"].hide_render = True
        if not args.keep_high:
            hm = hi.data
            bpy.data.objects.remove(hi)
            bpy.data.meshes.remove(hm)
        else:
            hi.hide_render = True
        tx = meta["texture"]
        log(f"{spec['id']}: done in {meta['timing_s']['total_s']:.1f} s (bake {meta['timing_s']['bake_s']:.1f} s), "
            f"region {tx['region_px']} px, texel {tx['texel_mm']:.2f} mm, uv coverage {tx['uv_coverage']:.2f}")
    # every rock was baked into its own images, so the atlas layout can be decided now
    regions = allocate_atlas([m["texture"]["region_px"] for m in metas], args.atlas)
    for m, reg in zip(metas, regions):
        m["_region"] = reg
    used = sum(r[2] ** 2 for r in regions) / float(args.atlas ** 2)
    log(f"atlas {args.atlas}^2: {used * 100:.1f} % allocated")

    # ---- atlas
    t = time.perf_counter()
    S = args.atlas
    fills = {"basecolor": None, "normal": (128, 128, 255, 255), "orm": (255, 230, 0, 255)}
    mean_bc = np.mean([m["_px"]["basecolor"][..., :3].reshape(-1, 3).mean(0) for m in metas], axis=0)
    fills["basecolor"] = (int(mean_bc[0]), int(mean_bc[1]), int(mean_bc[2]), 255)
    atlas_imgs = {}
    for key, cs in (("basecolor", "sRGB"), ("normal", "Non-Color"), ("orm", "Non-Color")):
        arr = np.empty((S, S, 4), np.uint8)
        arr[...] = np.array(fills[key], np.uint8)
        for m in metas:
            x, y, s = m["_region"]
            arr[y:y + s, x:x + s] = m["_px"][key]
        img = new_image(f"rock_atlas_{key}", S, cs, (0.5, 0.5, 0.5))
        u8_to_img(img, arr)
        save_png(img, os.path.join(out, "atlas", f"rock_atlas_{key}.png"))
        atlas_imgs[key] = img
    atlas_mat = make_pbr_material("lunar_rock_atlas", atlas_imgs["basecolor"], atlas_imgs["normal"], atlas_imgs["orm"])
    for m, lo in zip(metas, los):
        x, y, s = m["_region"]
        me = lo.data
        uv = np.empty(len(me.loops) * 2, np.float32)
        me.uv_layers.active.data.foreach_get("uv", uv)
        uv = uv.reshape(-1, 2)
        uv[:, 0] = (x + uv[:, 0] * s) / S
        uv[:, 1] = (y + uv[:, 1] * s) / S
        me.uv_layers.active.data.foreach_set("uv", uv.ravel())
        me.materials.clear()
        me.materials.append(atlas_mat)
        y_top = S - (y + s)
        m["atlas"] = dict(atlas_px=S, region_px_xywh_topleft=[x, y_top, s, s],
                          uv_rect_gltf=[x / S, y_top / S, (x + s) / S, (y_top + s) / S])
    T_atlas = time.perf_counter() - t

    # ---- library exports + npz + json
    t = time.perf_counter()
    lib_bytes = export_glb(los, os.path.join(out, "rocks_library.glb"))
    hull_objs = [m["hull"]["_obj"] for m in metas if m["hull"]]
    hull_bytes = export_glb(hull_objs, os.path.join(out, "rocks_hulls.glb"), materials=False, normals=False) if hull_objs else 0
    export_axis_marker(os.path.join(out, "axis_marker.glb"))
    export_normalmap_test(os.path.join(out, "normalmap_test.glb"), os.path.join(out, "atlas"))
    T_lib = time.perf_counter() - t
    for m, lo in zip(metas, los):
        pos, nrm, uvg, idx = corner_arrays(lo.data)
        np.savez_compressed(os.path.join(out, "npz", m["id"] + ".npz"), positions=pos, normals=nrm,
                            uv_atlas_gltf=uvg, indices=idx)
        m["files"]["npz"] = f"npz/{m['id']}.npz"
        m["lod"]["gltf_vertices"] = int(len(pos))
        m.pop("_px")
        m.pop("_region")
        if m["hull"]:
            m["hull"].pop("_obj")
        with open(os.path.join(out, "meta", m["id"] + ".json"), "w") as f:
            json.dump(m, f, indent=1)
    files = {}
    for rel in ["rocks_library.glb", "rocks_hulls.glb", "axis_marker.glb", "normalmap_test.glb"] + \
            [f"atlas/rock_atlas_{k}.png" for k in ("basecolor", "normal", "orm")]:
        p = os.path.join(out, rel)
        if os.path.exists(p):
            files[rel] = os.path.getsize(p)
    lib = dict(
        generator="lunar_world_gen/blender/rock_proto.py", blender=bpy.app.version_string,
        args=vars(args), cycles_devices=devices,
        conventions=dict(
            units="m", rock_frame="Z up (Blender), origin = volume centroid, rests on flattened base",
            gltf_axis="export_yup=True: (x, y, z)_blender -> (x, z, -y)_gltf; to restore Z-up rotate +90 deg about X",
            axis_marker="axis_marker.glb: +X 0.30 m red, +Y 0.20 m green, +Z (up) 0.10 m blue, same export settings",
            normalmap_test="normalmap_test.glb: x<0 real relief (dome, bar to +X, bump at +Y), x>0 flat tile with "
                           "its baked normal map; both must shade alike in the target renderer",
            normal_map="tangent space, OpenGL/glTF convention (+Y green up), MikkTSpace tangents exported",
            orm="R=1 (no occlusion), G=roughness, B=metallic(0) -> glTF metallicRoughnessTexture",
            npz="positions/normals Z-up rock frame; uv_atlas_gltf in glTF UV convention (origin top-left)"),
        atlas=dict(size_px=S, files=[f"atlas/rock_atlas_{k}.png" for k in ("basecolor", "normal", "orm")],
                   material="lunar_rock_atlas", regions={m["id"]: m["atlas"] for m in metas}),
        files_bytes=files,
        timing_s=dict(atlas_s=T_atlas, library_export_s=T_lib, total_s=time.perf_counter() - t_start),
        rocks=metas,
    )
    with open(os.path.join(out, "library.json"), "w") as f:
        json.dump(lib, f, indent=1)
    log(f"library: {len(metas)} rocks, rocks_library.glb {lib_bytes / 1e6:.2f} MB, hulls {hull_bytes / 1e3:.1f} kB, "
        f"total {time.perf_counter() - t_start:.1f} s")
    if args.save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, "rocks_library.blend"), compress=True)
    return lib


# ----------------------------------------------------------------------------------------------
# Preview rendering
# ----------------------------------------------------------------------------------------------
def look_at(obj, target):
    d = mathutils.Vector(target) - obj.location
    obj.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def setup_stage(args, ground_size=80.0):
    sc = bpy.context.scene
    world = bpy.data.worlds.new("black_sky")
    sc.world = world
    world.color = (0.0, 0.0, 0.0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            world.use_nodes = True
        except Exception:
            pass
    if world.node_tree:
        for n in world.node_tree.nodes:
            if n.type == "BACKGROUND":
                n.inputs["Color"].default_value = (0, 0, 0, 1)
                n.inputs["Strength"].default_value = 0.0
    # regolith ground
    me = bpy.data.meshes.new("ground")
    h = ground_size / 2
    me.from_pydata([(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)], [], [(0, 1, 2, 3)])
    g = link(bpy.data.objects.new("ground", me))
    mat = new_material("regolith")
    nb = NodeBuilder(mat)
    out = nb.node("ShaderNodeOutputMaterial")
    bsdf = nb.node("ShaderNodeBsdfPrincipled")
    nb.nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    nb.put(bsdf.inputs["Specular IOR Level"], 0.2)
    tc = nb.node("ShaderNodeTexCoord")
    n1 = nb.noise(tc.outputs["Object"], 0.8, detail=6.0, rough=0.55)
    n2 = nb.noise(tc.outputs["Object"], 25.0, detail=4.0, rough=0.6)
    n3 = nb.noise(tc.outputs["Object"], 300.0, detail=2.0, rough=0.5)
    a = REGOLITH_ALBEDO
    cfac = nb.maprange(n1.outputs["Fac"], 0.35, 0.65)
    col = nb.mix(cfac, nb.rgb(MATRIX_TINT * a * 0.9), nb.rgb(MATRIX_TINT * a * 1.1))
    nb.put(bsdf.inputs["Base Color"], col)
    nb.put(bsdf.inputs["Roughness"], 1.0)
    hsum = nb.math("ADD", nb.math("MULTIPLY", n1.outputs["Fac"], 4.0),
                   nb.math("ADD", nb.math("MULTIPLY", n2.outputs["Fac"], 0.25), nb.math("MULTIPLY", n3.outputs["Fac"], 0.03)))
    bump = nb.node("ShaderNodeBump")
    nb.put(bump.inputs["Strength"], 1.0)
    nb.put(bump.inputs["Distance"], 0.004)
    nb.put(bump.inputs["Height"], hsum)
    nb.put(bsdf.inputs["Normal"], bump.outputs["Normal"])
    g.data.materials.append(mat)
    # sun: low elevation, sharp shadows (solar disc 0.53 deg)
    ld = bpy.data.lights.new("sun", "SUN")
    ld.energy = args.sun_strength
    ld.angle = math.radians(0.53)
    sun = link(bpy.data.objects.new("sun", ld))
    el, az = math.radians(args.sun_elev), math.radians(args.sun_az)
    to_sun = mathutils.Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    sun.rotation_euler = to_sun.to_track_quat("Z", "Y").to_euler()
    for vt in ("AgX", "Filmic", "Standard"):
        try:
            sc.view_settings.view_transform = vt
            break
        except TypeError:
            continue
    try:
        sc.view_settings.look = "None"
    except TypeError:
        pass
    sc.view_settings.exposure = args.exposure
    sc.cycles.use_denoising = True
    for dn in ("OPTIX", "OPENIMAGEDENOISE"):
        try:
            sc.cycles.denoiser = dn
            break
        except TypeError:
            continue
    sc.cycles.max_bounces = 4
    return g


def render(cam_loc, target, lens, res, path, samples):
    sc = bpy.context.scene
    cd = bpy.data.cameras.new("cam")
    cd.lens = lens
    cd.clip_start = 0.01
    cam = link(bpy.data.objects.new("cam", cd))
    cam.location = cam_loc
    look_at(cam, target)
    sc.camera = cam
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    sc.cycles.samples = samples
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = path
    t = time.perf_counter()
    bpy.ops.render.render(write_still=True)
    dt = time.perf_counter() - t
    bpy.data.objects.remove(cam)
    log(f"rendered {os.path.basename(path)} in {dt:.1f} s")
    return dt


def place_rows(rows, burial):
    """rows: list of (y, gap, [(obj, meta)]); objects are centred in x, sunk by burial*height."""
    centres = {}
    for y, gap, items in rows:
        widths = [o.dimensions.x for o, _ in items]
        x = -(sum(widths) + gap * (len(items) - 1)) / 2.0
        for (o, m), w in zip(items, widths):
            bo = m["bottom_offset_m"]
            hgt = m["dims_m"][2]
            o.location = (x + w / 2.0, y + 0.0, bo - burial(m) * hgt)
            x += w + gap
        centres[y] = 0.0
    return centres


def scatter_instances(objs, metas, centre, half, seed=7):
    """Linked-duplicate scatter of the library (random yaw, class-dependent scale jitter and burial),
    i.e. roughly how a world generator would instance the prototypes. Returns the instance count."""
    rng = np.random.default_rng(seed)
    per_class = {"large": (1, 0.6, 1.3), "medium": (8, 0.6, 1.4), "pebble": (45, 0.7, 1.3)}
    placed = []      # (x, y, r) of large/medium footprints for crude overlap rejection
    n = 0
    for cls in ("large", "medium", "pebble"):
        cnt, s0, s1 = per_class[cls]
        for rid, o in objs.items():
            m = metas[rid]
            if m["cls"] != cls:
                continue
            for _ in range(cnt):
                s = float(rng.uniform(s0, s1))
                r = 0.5 * s * max(m["dims_m"][:2])
                for _try in range(30):
                    x = centre[0] + float(rng.uniform(-half[0], half[0]))
                    y = centre[1] + float(rng.uniform(-half[1], half[1]))
                    if all((x - px) ** 2 + (y - py) ** 2 > (r + pr) ** 2 for px, py, pr in placed):
                        break
                inst = o.copy()            # linked duplicate (shares mesh + material)
                bpy.context.scene.collection.objects.link(inst)
                bur = float(rng.uniform(*m["burial_fraction_range"]))
                inst.scale = (s, s, s)
                inst.rotation_euler = (0.0, 0.0, float(rng.uniform(0, 2 * math.pi)))
                inst.location = (x, y, s * (m["bottom_offset_m"] - bur * m["dims_m"][2]))
                if cls != "pebble":
                    placed.append((x, y, r))
                n += 1
    return n


def preview(args, out, lib=None):
    reset_scene()
    sc = bpy.context.scene
    setup_cycles(sc, args.device, args.preview_samples)
    if lib is None:
        with open(os.path.join(out, "library.json")) as f:
            lib = json.load(f)
    bpy.ops.import_scene.gltf(filepath=os.path.join(out, "rocks_library.glb"))
    metas = {m["id"]: m for m in lib["rocks"]}
    objs = {o.name: o for o in bpy.data.objects if o.type == "MESH" and o.name in metas}
    # import check: the importer converts +Y up back to Z up -> dimensions must match the metadata
    chk = []
    for rid, o in objs.items():
        o.rotation_mode = "XYZ"
        d = [round(x, 4) for x in o.dimensions]
        chk.append((rid, d, metas[rid]["dims_m"]))
    log("re-import dims (blender XYZ vs meta):", chk)
    setup_stage(args)
    # ascending size from left to right: the sun comes from the left, so shadows fall on empty ground
    large = sorted([(objs[k], metas[k]) for k in objs if metas[k]["cls"] == "large"], key=lambda t: t[1]["dims_m"][0])
    medium = sorted([(objs[k], metas[k]) for k in objs if metas[k]["cls"] == "medium"], key=lambda t: t[1]["dims_m"][0])
    pebble = [(objs[k], metas[k]) for k in objs if metas[k]["cls"] == "pebble"]
    yl = 1.6 + (max([m["dims_m"][1] for _, m in large]) / 2 if large else 0.0)
    place_rows([(yl, 0.5, large), (0.75, 0.08, medium), (0.42, 0.035, pebble)],
               burial=lambda m: m["preview_burial_fraction"])
    pv = os.path.join(out, "preview")
    os.makedirs(pv, exist_ok=True)
    times = {}
    times["overview"] = render((0.0, -1.6, 1.15), (0.0, yl * 0.75, 0.15), 30.0, (1600, 900),
                               os.path.join(pv, "preview_overview.png"), args.preview_samples)
    times["closeup"] = render((0.05, 0.02, 0.30), (0.0, 0.62, 0.02), 35.0, (1600, 900),
                              os.path.join(pv, "preview_closeup.png"), args.preview_samples)
    if large:   # texture detail of the biggest boulder, seen from ~1.6 x its length, lit side
        o, m = large[-1]
        L0 = max(m["dims_m"])
        tgt = mathutils.Vector(o.location) + mathutils.Vector((0.0, 0.0, 0.15 * m["dims_m"][2]))
        cam = tgt + mathutils.Vector((-0.9 * L0, -1.3 * L0, 0.55 * L0))
        times["large_detail"] = render(tuple(cam), tuple(tgt), 50.0, (1600, 900),
                                       os.path.join(pv, "preview_large_detail.png"), args.preview_samples)
    # scatter field (away from the rows), seen from rover camera height with a wide lens
    n_inst = scatter_instances(objs, metas, centre=(20.0, 8.0), half=(3.5, 4.0))
    times["scatter"] = render((20.0, 3.2, 0.85), (20.0, 8.5, 0.0), 24.0, (1600, 900),
                              os.path.join(pv, "preview_scatter.png"), args.preview_samples)
    log(f"scatter preview: {n_inst} instances")
    # normal-map round trip: re-imported normalmap_test.glb, real relief (left) vs. normal-mapped tile (right)
    nmt = os.path.join(out, "normalmap_test.glb")
    if os.path.exists(nmt):
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=nmt)
        for o in set(bpy.data.objects) - before:
            if o.type == "MESH":
                o.location = (o.location.x - 20.0, o.location.y + 8.0, o.location.z + 0.001)
        times["normalmap_test"] = render((-20.0, 7.35, 0.55), (-20.0, 8.0, 0.0), 50.0, (1200, 700),
                                         os.path.join(pv, "preview_normalmap_test.png"), args.preview_samples)
    clay = new_material("clay")
    nb = NodeBuilder(clay)
    o_ = nb.node("ShaderNodeOutputMaterial")
    b_ = nb.node("ShaderNodeBsdfPrincipled")
    nb.put(b_.inputs["Base Color"], (0.3, 0.3, 0.3, 1.0))
    nb.put(b_.inputs["Roughness"], 0.9)
    nb.nt.links.new(b_.outputs["BSDF"], o_.inputs["Surface"])
    bpy.context.view_layer.material_override = clay
    times["closeup_clay"] = render((0.05, 0.02, 0.30), (0.0, 0.62, 0.02), 35.0, (1600, 900),
                                   os.path.join(pv, "preview_closeup_clay.png"), max(16, args.preview_samples // 2))
    times["overview_clay"] = render((0.0, -1.6, 1.15), (0.0, yl * 0.75, 0.15), 30.0, (1600, 900),
                                    os.path.join(pv, "preview_overview_clay.png"), max(16, args.preview_samples // 2))
    bpy.context.view_layer.material_override = None
    with open(os.path.join(pv, "preview_info.json"), "w") as f:
        json.dump(dict(render_s=times, sun_elev_deg=args.sun_elev, sun_az_deg=args.sun_az,
                       sun_strength=args.sun_strength, exposure=args.exposure,
                       view_transform=sc.view_settings.view_transform, reimport_dims=chk), f, indent=1)
    return times


def shapes_mode(args, out):
    """Fast shape iteration: high-poly + LOD side by side, clay renders only."""
    reset_scene()
    sc = bpy.context.scene
    setup_cycles(sc, args.device, args.preview_samples)
    specs = sample_specs(args)
    items = {"large": [], "medium": [], "pebble": []}
    for spec in specs:
        hi, lo, info = build_rock(spec, args, out)
        Vl = get_verts(lo.data)
        mh = dict(bottom_offset_m=float(-get_verts(hi.data)[:, 2].min()), dims_m=list(hi.dimensions),
                  preview_burial_fraction=0.0)
        ml = dict(bottom_offset_m=float(-Vl[:, 2].min()), dims_m=list(lo.dimensions), preview_burial_fraction=0.0)
        items[spec["cls"]].append((hi, mh))
        items[spec["cls"]].append((lo, ml))
    setup_stage(args)
    yl = 1.6 + (max([m["dims_m"][1] for _, m in items["large"]]) / 2 if items["large"] else 0.0)
    place_rows([(yl, 0.3, items["large"]), (0.75, 0.05, items["medium"]), (0.42, 0.02, items["pebble"])],
               burial=lambda m: 0.0)
    clay = new_material("clay")
    nb = NodeBuilder(clay)
    o_ = nb.node("ShaderNodeOutputMaterial")
    b_ = nb.node("ShaderNodeBsdfPrincipled")
    nb.put(b_.inputs["Base Color"], (0.3, 0.3, 0.3, 1.0))
    nb.put(b_.inputs["Roughness"], 0.9)
    nb.nt.links.new(b_.outputs["BSDF"], o_.inputs["Surface"])
    bpy.context.view_layer.material_override = clay
    pv = os.path.join(out, "preview")
    os.makedirs(pv, exist_ok=True)
    render((0.0, -1.6, 1.15), (0.0, yl * 0.75, 0.15), 30.0, (1600, 900), os.path.join(pv, "shapes_overview.png"),
           args.preview_samples)
    render((0.05, 0.02, 0.30), (0.0, 0.62, 0.02), 35.0, (1600, 900), os.path.join(pv, "shapes_closeup.png"),
           args.preview_samples)


# ----------------------------------------------------------------------------------------------
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="rock_proto.py", description=__doc__.split("\n")[1],
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--out", required=True, help="output directory (UNC //wsl.localhost/... works)")
    ap.add_argument("--mode", choices=["full", "shapes", "preview"], default="full")
    ap.add_argument("--seed", type=int, default=1, help="library seed (each rock derives its own seed)")
    ap.add_argument("--n-large", type=int, default=2)
    ap.add_argument("--n-medium", type=int, default=3)
    ap.add_argument("--n-pebble", type=int, default=4)
    ap.add_argument("--only", default="", help="comma separated rock ids to build (debug)")
    ap.add_argument("--atlas", type=int, default=2048, help="atlas size (px, power of two)")
    ap.add_argument("--region-large", type=int, default=1024, help="max atlas region per large rock (px)")
    ap.add_argument("--region-medium", type=int, default=512, help="max atlas region per medium rock (px)")
    ap.add_argument("--region-pebble", type=int, default=64, help="max atlas region per pebble (px)")
    ap.add_argument("--texel-mm-large", type=float, default=3.0,
                    help="target texel size [mm]; region = smallest power of two reaching it (<=0: always max)")
    ap.add_argument("--texel-mm-medium", type=float, default=1.0)
    ap.add_argument("--texel-mm-pebble", type=float, default=0.5)
    ap.add_argument("--device", choices=["OPTIX", "CUDA", "CPU"], default="OPTIX")
    ap.add_argument("--bake-samples", type=int, default=8, help="Cycles AA samples per bake")
    ap.add_argument("--hull-max-verts", type=int, default=64)
    ap.add_argument("--hull-all", action="store_true", help="also make hulls for pebbles")
    ap.add_argument("--keep-high", action="store_true", help="keep high-poly objects (with --save-blend)")
    ap.add_argument("--save-blend", action="store_true")
    ap.add_argument("--keep-undo", action="store_true",
                    help="diagnostic: keep Blender's global undo enabled (default: disabled). The bake-time growth "
                         "seen in long runs came from per-rock images/materials, which are now freed; with them "
                         "freed, undo on/off measured no difference")
    ap.add_argument("--preview", action="store_true", help="render previews after generation")
    ap.add_argument("--preview-samples", type=int, default=96)
    ap.add_argument("--sun-elev", type=float, default=12.0, help="preview sun elevation [deg]")
    ap.add_argument("--sun-az", type=float, default=195.0,
                    help="preview sun azimuth [deg from +X, CCW]; camera looks along +Y, 180 = from the left")
    ap.add_argument("--sun-strength", type=float, default=6.0, help="preview sun irradiance (Blender W/m^2)")
    ap.add_argument("--exposure", type=float, default=0.0)
    args = ap.parse_args(argv)
    for k in ("atlas", "region_large", "region_medium", "region_pebble"):
        v = getattr(args, k)
        if v & (v - 1):
            ap.error(f"--{k.replace('_', '-')} must be a power of two")
    return args


def main():
    args = parse_args()
    out = os.path.normpath(args.out)          # '//wsl.localhost/..' -> '\\\\wsl.localhost\\..' on Windows
    os.makedirs(out, exist_ok=True)
    t = time.perf_counter()
    if args.mode == "shapes":
        shapes_mode(args, out)
    elif args.mode == "preview":
        preview(args, out)
    else:
        lib = generate(args, out)
        if args.preview:
            preview(args, out)
    log(f"finished mode={args.mode} in {time.perf_counter() - t:.1f} s -> {out}")


if __name__ == "__main__":
    main()
