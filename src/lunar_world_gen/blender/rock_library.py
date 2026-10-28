"""
rock_library.py -- production rock library for the Apollo 16 lunar world (headless Blender + Cycles).

Builds the prototype library that lunar_world_gen.rocks reads (docs/DESIGN.md section 5):
20 pebbles + 20 medium + 10 large rocks, each with LOD0/LOD1/LOD2, a convex hull (<= 64 vertices)
and one shared texture atlas (basecolor sRGB, tangent-space normal, ORM, roughness).

Run (Git Bash on Windows; Blender 5.2.1 tested):

    B="/c/Program Files/Blender Foundation/Blender 5.2/blender.exe"
    "$B" -b --factory-startup -P rock_library.py -- \
        --out //wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/library/rocks_v1 --seed 16 --preview

Options: ``-- --help``. ``--mode preview`` re-renders the previews of an existing library.

Pipeline per prototype (shape and material code is shared with rock_proto.py, the research prototype):
  1. high-poly shape: superellipsoid with impact-fragment axial ratios, planar fracture cuts, voxel
     remesh, height-weighted smoothing, subdivision, fBm / crack / pit displacement (rock_proto).
  2. LOD0 at the class budget: pebbles 16-40 tris (greedy convex hull of the high-poly), medium
     200-1000 and large 2000-5000 (quadric collapse decimation, grows with size).
     LOD1 = 30 % and LOD2 = 10 % (>= 12 tris) of LOD0: collapse decimation of LOD0, or a greedy convex
     hull of the LOD0 vertices when the target is below 48 triangles (robustly closed). Pebble LOD1
     and LOD2 are both 12-triangle hulls, so LOD2 reuses LOD1 (same geometry, same atlas region).
  3. every distinct LOD gets its own smart-UV unwrap and its own bake (normal, base colour, roughness)
     from the high-poly breccia material, so each LOD's normal map encodes the full high-poly detail.
     Atlas region per LOD = smallest power of two reaching the class texel target (pebble 0.5 mm,
     medium 1 mm, large 3 mm) times a LOD factor (large 1/2/2, medium 1/1/2, pebble 1/2/2).
  4. hull: greedy farthest-point convex hull of the LOD0 vertices (<= 64 vertices, strictly convex).
  5. atlas: buddy (quadtree) allocation of power-of-two regions; if they do not fit, the region whose
     texel is finest relative to its target is halved (2x2 box filter: linear-space colour,
     renormalised normals) until they fit.
  6. export (Z-up rock frame, metres): npz/<id>_lod<k>.npz (positions f4, normals f4, uv f4 in glTF
     convention = atlas, v down, indices u4), hulls/<id>.npz (vertices f4, faces u4),
     atlas/rock_atlas_{basecolor,normal,orm,roughness}.png, library.json.
  7. --preview: rebuilds the rocks from the exported npz files (round-trip check) and renders them under
     a 15 deg sun and a black sky: overview, close-up, LOD comparison, large-rock detail, and a scatter
     field drawn from the LM-site size-frequency distribution with the world's class -> LOD rules.

Rock frame: Z up, origin at the volume centroid of the high-poly rock, the rock rests on its flattened
base, x is the longest horizontal axis. dims_m = LOD0 extents [x, y, z]; bottom_offset_m = -min z (LOD0).
Deterministic per --seed (each rock derives its seed from (seed, class, index)).
"""

import argparse
import json
import math
import os
import sys
import time

import bpy
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rock_proto as rp  # noqa: E402  (Blender adds no script dir to sys.path)

GENERATOR_VERSION = "1.0.0"
LIBRARY_VERSION = "rocks_v1"
LOG_PREFIX = "[rock_library]"

LOD_FRACTIONS = (1.0, 0.30, 0.10)
LOD_MIN_TRIS = 12
HULL_LOD_BELOW_TRIS = 48             # LOD1/LOD2 targets below this use a greedy convex hull
# LOD0 budgets per DESIGN (pebble 12-40; sampled 16-40 so LOD0 always has more detail than LOD1/2)
PROD_CLASS_CFG = {k: dict(v) for k, v in rp.CLASS_CFG.items()}
PROD_CLASS_CFG["pebble"]["tris"] = (16, 40)
PROD_CLASS_CFG["medium"]["tris"] = (200, 1000)
PROD_CLASS_CFG["large"]["tris"] = (2000, 5000)
TEXEL_MM = {"pebble": 0.5, "medium": 1.0, "large": 3.0}          # LOD0 texel targets
REGION_MAX = {"pebble": 64, "medium": 512, "large": 1024}          # LOD0 region caps [px]
LOD_TEXEL_FACTOR = {"pebble": (1, 2, 2), "medium": (1, 1, 2), "large": (1, 2, 2)}
MIN_REGION = 16
CLASS_ORDER = ("pebble", "medium", "large")                        # prototype index order in library.json


def log(*a):
    print(LOG_PREFIX, *a, flush=True)


# ----------------------------------------------------------------------------------------------
# geometry helpers
# ----------------------------------------------------------------------------------------------
def weld_closed(V, F):
    """True if the triangle mesh is closed (every edge in exactly 2 triangles) after welding positions."""
    _, inv = np.unique(np.round(np.asarray(V, np.float64), 7), axis=0, return_inverse=True)
    W = inv.ravel()[np.asarray(F)]
    e = np.sort(np.concatenate([W[:, [0, 1]], W[:, [1, 2]], W[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return bool(np.all(c == 2))


def hull_lod(name, points, target_tris, sharp_deg):
    """Greedy convex hull with k = (tris + 4) / 2 vertices (2k - 4 triangles) as a Blender object."""
    k = max(4, (int(target_tris) + 4) // 2)
    V, F, dmax = rp.greedy_hull(points, k)
    obj = rp.link(bpy.data.objects.new(name, rp.mesh_from_np(name, V, F)))
    me = obj.data
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(sharp_deg))
    return obj, f"greedy convex hull k={k}"


def decimate_lod(name, src, target_tris, sharp_deg):
    """Collapse-decimated copy of src with about target_tris triangles."""
    me = src.data.copy()
    me.name = name
    obj = rp.link(bpy.data.objects.new(name, me))
    n0 = len(rp.get_tris(me))
    if target_tris < n0:
        mod = obj.modifiers.new("decimate", "DECIMATE")
        mod.decimate_type = "COLLAPSE"
        mod.ratio = float(target_tris) / n0
        mod.use_collapse_triangulate = True
        rp.apply_modifiers(obj)
        rp.triangulate_mesh(obj.data)
    me = obj.data
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(sharp_deg))
    return obj, f"decimate COLLAPSE {n0}->{target_tris}"


def remove_object(obj):
    me = obj.data
    bpy.data.objects.remove(obj)
    if me is not None and me.users == 0:
        bpy.data.meshes.remove(me)


# ----------------------------------------------------------------------------------------------
# image helpers
# ----------------------------------------------------------------------------------------------
def srgb2lin(c):
    c = np.asarray(c, np.float64)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def box2(a):
    return 0.25 * (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2])


def downsample2(px, kind):
    """2x2 box downsample of an RGBA uint8 map (colour in linear space, normals renormalised)."""
    a = px[..., :3].astype(np.float64) / 255.0
    if kind == "basecolor":
        rgb = rp.lin2srgb(box2(srgb2lin(a)))
    elif kind == "normal":
        n = box2(a * 2.0 - 1.0)
        n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
        rgb = 0.5 * (n + 1.0)
    else:
        rgb = box2(a)
    out = np.empty((rgb.shape[0], rgb.shape[1], 4), np.uint8)
    out[..., :3] = np.clip(np.round(rgb * 255.0), 0, 255).astype(np.uint8)
    out[..., 3] = 255
    return out


def write_png(name, arr, colorspace, path):
    img = rp.new_image(name, arr.shape[0], colorspace, (0.5, 0.5, 0.5))
    rp.u8_to_img(img, arr)
    rp.save_png(img, path)
    bpy.data.images.remove(img)
    return os.path.getsize(path)


# ----------------------------------------------------------------------------------------------
# per-rock build
# ----------------------------------------------------------------------------------------------
def lod_targets(cls, t0):
    t1 = max(LOD_MIN_TRIS, int(round(LOD_FRACTIONS[1] * t0)))
    t2 = max(LOD_MIN_TRIS, int(round(LOD_FRACTIONS[2] * t0)))
    return [t0, t1, t2]


def choose_region(lo, area, cls, lod, args):
    """Unwrap lo and pick its atlas region size (smallest power of two reaching the texel target)."""
    cfg = PROD_CLASS_CFG[cls]
    f = LOD_TEXEL_FACTOR[cls][lod]
    tex_m = TEXEL_MM[cls] * f * 1e-3 * args.texel_scale
    r_max = max(MIN_REGION, REGION_MAX[cls] // f)
    r_min = max(MIN_REGION, cfg["region_min"] // f)
    R_est = math.sqrt(area / (0.55 * tex_m ** 2))
    R = int(min(max(2 ** math.ceil(math.log2(max(R_est, 1.0))), r_min), r_max))
    while True:
        gap_px = int(min(max(R // 128, 2), cfg["gap_px"]))
        uv_area = rp.uv_unwrap(lo, gap_px / R)
        texel = math.sqrt(area / max(uv_area * R * R, 1e-12))
        if texel > 1.25 * tex_m and R < r_max:
            R *= 2
            continue
        return R, gap_px, uv_area, texel, tex_m


def build_rock(spec, args):
    rid, cls, P = spec["id"], spec["cls"], spec["params"]
    cfg = PROD_CLASS_CFG[cls]
    T = {}
    t0 = time.perf_counter()
    hi, hst = rp.build_high_poly(rid, P)
    T["high_poly_s"] = time.perf_counter() - t0
    # ---- LODs
    t = time.perf_counter()
    objs, methods = [], []
    if cls == "pebble":
        cand, _ = rp.convex_hull_np(rp.get_verts(hi.data))
        targets = lod_targets(cls, P["tris"])
        o0, m0 = hull_lod(f"{rid}_lod0", cand, targets[0], cfg["sharp_deg"])
        objs.append(o0)
        methods.append(m0)
    else:
        o0, st = rp.make_lod(hi, f"{rid}_lod0", cls, P, cfg)
        objs.append(o0)
        methods.append(st["lod_method"])
    t0_tris = len(rp.get_tris(objs[0].data))
    targets = lod_targets(cls, t0_tris)
    V0 = rp.get_verts(objs[0].data)
    shared = [None, None, None]                     # LOD k reuses LOD shared[k] (identical geometry)
    for lod in (1, 2):
        name = f"{rid}_lod{lod}"
        tgt = targets[lod]
        if lod == 2 and tgt == targets[1] and targets[1] < HULL_LOD_BELOW_TRIS:
            objs.append(objs[1])                    # same greedy hull as LOD1
            methods.append(methods[1])
            shared[2] = 1
            continue
        if tgt < HULL_LOD_BELOW_TRIS:
            cand0, _ = rp.convex_hull_np(V0)
            o, m = hull_lod(name, cand0, tgt, cfg["sharp_deg"])
        else:
            o, m = decimate_lod(name, objs[0], tgt, cfg["sharp_deg"])
            if not weld_closed(rp.get_verts(o.data), rp.get_tris(o.data)):
                log(f"WARN {name}: decimated LOD not closed, using a convex hull instead")
                remove_object(o)
                cand0, _ = rp.convex_hull_np(V0)
                o, m = hull_lod(name, cand0, tgt, cfg["sharp_deg"])
        objs.append(o)
        methods.append(m)
    T["lod_s"] = time.perf_counter() - t
    # ---- LOD errors (signed distance of high-poly vertices to each LOD)
    t = time.perf_counter()
    errs = []
    for lod, o in enumerate(objs):
        errs.append(errs[shared[lod]] if shared[lod] is not None else rp.lod_error(hi, o))
    T["lod_error_s"] = time.perf_counter() - t
    tris = [len(rp.get_tris(o.data)) for o in objs]
    log(f"{rid}: {P['litho']} a={P['axes'][0]:.3f} m hi_tris={hst['hi_tris']} lod_tris={tris} "
        f"err_rms_mm={[round(e['rms_mm'], 2) for e in errs]} ({T['high_poly_s']:.1f}+{T['lod_s']:.1f} s)")
    # ---- UV + bake per distinct LOD
    t = time.perf_counter()
    infos = []
    texel_min = None
    for lod, o in enumerate(objs):
        if shared[lod] is not None:
            infos.append(None)
            continue
        Vl, Fl = rp.get_verts(o.data), rp.get_tris(o.data)
        area = rp.surface_area(Vl, Fl)
        R, gap_px, uv_area, texel, tex_m = choose_region(o, area, cls, lod, args)
        texel_min = texel if texel_min is None else min(texel_min, texel)
        infos.append(dict(R=R, gap_px=gap_px, uv_area=uv_area, texel=texel, target=tex_m, area=area))
    T["uv_s"] = time.perf_counter() - t
    mat_hi, mean_alb = rp.build_breccia_material(rid, P, texel_min)
    hi.data.materials.clear()
    hi.data.materials.append(mat_hi)
    bpy.context.scene.cycles.samples = args.bake_samples
    t = time.perf_counter()
    T["bake_detail_s"] = {}
    for lod, o in enumerate(objs):
        if shared[lod] is not None:
            continue
        for other in objs:
            other.hide_render = other is not o
        hi.hide_render = False
        info = infos[lod]
        e = errs[lod]
        a = P["axes"][0]
        ext = 1.3 * e["max_out_mm"] * 1e-3 + 0.002 * a
        ray = ext + 1.3 * e["max_in_mm"] * 1e-3 + 0.002 * a
        margin = max(info["gap_px"], min(info["R"] // 16, 16))
        imgs, TB = rp.bake_maps(hi, o, f"{rid}_lod{lod}", info["R"], margin, ext, ray, P, mean_alb)
        T["bake_detail_s"][f"lod{lod}"] = TB
        px = {k: rp.img_to_u8(v) for k, v in imgs.items()}
        orm = np.empty_like(px["rough"])
        orm[..., 0] = 255
        orm[..., 1] = px["rough"][..., 0]
        orm[..., 2] = 0
        orm[..., 3] = 255
        rough = px["rough"].copy()
        rough[..., 1] = rough[..., 0]
        rough[..., 2] = rough[..., 0]
        rough[..., 3] = 255
        info["px"] = {"basecolor": px["basecolor"], "normal": px["normal"], "orm": orm, "roughness": rough}
        info.update(bake_cage_extrusion_m=ext, bake_max_ray_m=ray, bake_margin_px=margin)
        for im in imgs.values():
            bpy.data.images.remove(im)
    T["bake_s"] = time.perf_counter() - t
    hi.data.materials.clear()
    bpy.data.materials.remove(mat_hi)
    # ---- export arrays (Z-up, region-local glTF UVs; atlas remap happens later)
    arrays = []
    for lod, o in enumerate(objs):
        if shared[lod] is not None:
            arrays.append(arrays[shared[lod]])
            continue
        arrays.append(rp.corner_arrays(o.data))
    # ---- hull (from LOD0 vertices)
    t = time.perf_counter()
    if cls == "pebble":
        Vh, Fh = rp.convex_hull_np(V0, fix_convex=True)
    else:
        cand, _ = rp.convex_hull_np(V0)
        Vh, Fh, _ = rp.greedy_hull(cand, args.hull_max_verts)
    F0 = rp.get_tris(objs[0].data)
    vol_h, _ = rp.mass_props(Vh, Fh)
    vol_l, _ = rp.mass_props(V0, F0)
    out_max, out_mean = rp.outside_distance(V0, Vh, Fh)
    T["hull_s"] = time.perf_counter() - t
    # ---- frame check: x must be the longest horizontal axis
    ext0 = V0.max(0) - V0.min(0)
    rot90 = bool(ext0[1] > ext0[0])
    if rot90:
        log(f"{rid}: y extent > x extent, rotating by 90 deg about z")
    closed = [weld_closed(a[0], a[3].astype(np.int64)) for a in arrays]
    rec = dict(
        id=rid, cls=cls, seed=P["seed"], lithology=P["litho"], roundness=round(P["roundness"], 3),
        axes_target_m=[round(x, 5) for x in P["axes"]], volume_m3=vol_l, surface_area_m2=infos[0]["area"],
        mean_albedo_linear=round(mean_alb, 4), high_poly=hst,
        lods=[dict(lod=lod, method=methods[lod], tris=int(tris[lod]), closed=closed[lod],
                   shared_with_lod=shared[lod], error_mm=errs[lod]) for lod in range(3)],
        hull=dict(vertices=int(len(Vh)), triangles=int(len(Fh)), volume_ratio_vs_lod0=vol_h / vol_l,
                  lod0_max_outside_mm=1e3 * out_max, lod0_mean_outside_mm=1e3 * out_mean),
        timing_s=T,
    )
    for o in {id(o): o for o in objs}.values():
        remove_object(o)
    remove_object(hi)
    return rec, infos, arrays, (Vh, Fh), rot90


def rot90z(a):
    a = np.asarray(a).copy()
    x = a[:, 0].copy()
    a[:, 0] = -a[:, 1]
    a[:, 1] = x
    return a


# ----------------------------------------------------------------------------------------------
# atlas
# ----------------------------------------------------------------------------------------------
def fit_atlas(regions, S):
    """Halve the finest (relative to target) regions until sum(R^2) <= S^2. Returns #halvings."""
    n_half = 0
    while sum(r["R"] ** 2 for r in regions) > S * S:
        cand = [r for r in regions if r["R"] > MIN_REGION]
        if not cand:
            raise RuntimeError("atlas too small even with minimum regions")
        r = min(cand, key=lambda r: (r["texel"] / r["target"], -r["R"]))
        r["px"] = {k: downsample2(v, k) for k, v in r["px"].items()}
        r["R"] //= 2
        r["texel"] *= 2.0
        r["halved"] = r.get("halved", 0) + 1
        n_half += 1
    return n_half


def generate(args, out):
    t_start = time.perf_counter()
    rp.reset_scene()
    rp.disable_undo()
    sc = bpy.context.scene
    devices = rp.setup_cycles(sc, args.device, args.bake_samples)
    if args.threads > 0:
        sc.render.threads_mode = "FIXED"
        sc.render.threads = args.threads
    log("Cycles devices:", devices)
    for d in ("npz", "hulls", "atlas", "preview"):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    specs = rp.sample_specs(args, PROD_CLASS_CFG)
    specs.sort(key=lambda s: (CLASS_ORDER.index(s["cls"]), s["idx"]))
    recs, regions, arrays_all, hulls, rots = [], [], [], [], []
    for spec in specs:
        t = time.perf_counter()
        rec, infos, arrays, hull, rot90 = build_rock(spec, args)
        rec["timing_s"]["total_s"] = time.perf_counter() - t
        recs.append(rec)
        arrays_all.append(arrays)
        hulls.append(hull)
        rots.append(rot90)
        for lod, info in enumerate(infos):
            if info is not None:
                info.update(rock=len(recs) - 1, lod=lod, cls=spec["cls"])
                regions.append(info)
        log(f"{spec['id']}: done in {rec['timing_s']['total_s']:.1f} s (bake {rec['timing_s']['bake_s']:.1f} s), "
            f"regions {[i['R'] for i in infos if i]} px, texel {[round(1e3 * i['texel'], 2) for i in infos if i]} mm")
    # ---- atlas
    t = time.perf_counter()
    S = args.atlas
    n_half = fit_atlas(regions, S)
    alloc = rp.allocate_atlas([r["R"] for r in regions], S)
    used = sum(r["R"] ** 2 for r in regions) / float(S * S)
    log(f"atlas {S}^2: {used * 100:.1f} % allocated, {n_half} region halvings")
    fills = {"normal": (128, 128, 255, 255), "orm": (255, 230, 0, 255), "roughness": (230, 230, 230, 255)}
    mean_bc = np.mean([r["px"]["basecolor"][..., :3].reshape(-1, 3).mean(0) for r in regions], axis=0)
    fills["basecolor"] = (int(mean_bc[0]), int(mean_bc[1]), int(mean_bc[2]), 255)
    files_bytes = {}
    atlas_files = {}
    for key, cs in (("basecolor", "sRGB"), ("normal", "Non-Color"), ("orm", "Non-Color"), ("roughness", "Non-Color")):
        arr = np.empty((S, S, 4), np.uint8)
        arr[...] = np.array(fills[key], np.uint8)
        for r, (x, y, s) in zip(regions, alloc):
            arr[y:y + s, x:x + s] = r["px"][key]
        rel = f"atlas/rock_atlas_{key}.png"
        files_bytes[rel] = write_png(f"rock_atlas_{key}", arr, cs, os.path.join(out, rel))
        atlas_files[key] = rel
    for r, (x, y, s) in zip(regions, alloc):
        r["xy"] = (x, y, s)
        r.pop("px")
    T_atlas = time.perf_counter() - t
    # ---- per-LOD npz with atlas UVs, hulls, records
    t = time.perf_counter()
    reg_of = {(r["rock"], r["lod"]): r for r in regions}
    protos = []
    for k, (rec, arrays, (Vh, Fh), rot90) in enumerate(zip(recs, arrays_all, hulls, rots)):
        rid = rec["id"]
        lod_recs = []
        for lod in range(3):
            src = rec["lods"][lod]["shared_with_lod"]
            r = reg_of[(k, src if src is not None else lod)]
            x, y, s = r["xy"]
            y_top = S - (y + s)
            pos, nrm, uvg, idx = arrays[lod]
            if rot90:
                pos, nrm = rot90z(pos), rot90z(nrm)
            uv = np.empty_like(uvg)
            uv[:, 0] = (x + uvg[:, 0] * s) / S
            uv[:, 1] = (y_top + uvg[:, 1] * s) / S
            rel = f"npz/{rid}_lod{lod}.npz"
            np.savez_compressed(os.path.join(out, rel), positions=pos.astype(np.float32),
                                normals=nrm.astype(np.float32), uv=uv.astype(np.float32),
                                indices=idx.astype(np.uint32))
            files_bytes[rel] = os.path.getsize(os.path.join(out, rel))
            lr = dict(rec["lods"][lod])
            lr.update(file=rel, vertices=int(len(pos)),
                      atlas_region_px_xywh_topleft=[int(x), int(y_top), int(s), int(s)],
                      uv_rect_gltf=[x / S, y_top / S, (x + s) / S, (y_top + s) / S],
                      region_px=int(s), texel_mm=round(1e3 * r["texel"], 3),
                      texel_target_mm=round(1e3 * r["target"], 3), uv_coverage=round(r["uv_area"], 4),
                      region_halvings=int(r.get("halved", 0)),
                      bake_cage_extrusion_m=r.get("bake_cage_extrusion_m"), bake_max_ray_m=r.get("bake_max_ray_m"))
            lod_recs.append(lr)
        hv = rot90z(Vh) if rot90 else Vh
        hrel = f"hulls/{rid}.npz"
        np.savez_compressed(os.path.join(out, hrel), vertices=hv.astype(np.float32), faces=Fh.astype(np.uint32))
        files_bytes[hrel] = os.path.getsize(os.path.join(out, hrel))
        P0 = arrays[0][0] if not rot90 else rot90z(arrays[0][0])
        bmin, bmax = P0.min(0), P0.max(0)
        dims = bmax - bmin
        if dims[2] > dims[0]:
            log(f"WARN {rid}: vertical extent exceeds x extent; a = max(dims)")
        proto = dict(index=k, id=rid, cls=rec["cls"], dims_m=[round(float(v), 6) for v in dims],
                     bbox_min_m=[round(float(v), 6) for v in bmin], bbox_max_m=[round(float(v), 6) for v in bmax],
                     bottom_offset_m=round(float(-bmin[2]), 6), top_m=round(float(bmax[2]), 6),
                     lod_tris=[lr["tris"] for lr in lod_recs], lods=lod_recs,
                     hull=dict(rec["hull"], file=hrel), rotated_90_about_z=rot90)
        for key in ("seed", "lithology", "roundness", "axes_target_m", "volume_m3", "surface_area_m2",
                    "mean_albedo_linear", "high_poly", "timing_s"):
            proto[key] = rec[key]
        protos.append(proto)
    T_export = time.perf_counter() - t
    classes = {}
    for cls in CLASS_ORDER:
        ps = [p for p in protos if p["cls"] == cls]
        if not ps:
            continue
        tri = np.array([p["lod_tris"] for p in ps])
        classes[cls] = dict(n=len(ps), size_range_m=list(PROD_CLASS_CFG[cls]["size"]),
                            lod0_budget_tris=list(PROD_CLASS_CFG[cls]["tris"]),
                            lod_tris_min=tri.min(0).tolist(), lod_tris_max=tri.max(0).tolist(),
                            texel_target_mm=[TEXEL_MM[cls] * f * args.texel_scale for f in LOD_TEXEL_FACTOR[cls]],
                            burial_fraction_range=list(PROD_CLASS_CFG[cls]["burial"]))
    lib = dict(
        version=LIBRARY_VERSION, generator="lunar_world_gen/blender/rock_library.py",
        generator_version=GENERATOR_VERSION, seed=args.seed, blender=bpy.app.version_string,
        created_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), args=vars(args), cycles_devices=devices,
        conventions=dict(
            units="m", frame="Z up; origin = volume centroid of the high-poly rock; rests on its flattened base; "
                           "x = longest horizontal axis",
            dims="dims_m = LOD0 extents [x, y, z]; a = max(dims_m) is the size the SFD diameter refers to",
            bottom_offset="bottom_offset_m = distance from the origin down to the lowest LOD0 vertex",
            npz="positions (n,3) f4, normals (n,3) f4, uv (n,2) f4 = atlas UV in glTF convention (v down, "
                "origin top-left of the PNG), indices (m,3) u4; glTF-style corner arrays, CCW outward",
            hull="hulls/<id>.npz: vertices (k,3) f4, faces (t,3) u4, strictly convex, outward CCW, <= 64 vertices",
            atlas="basecolor sRGB; normal = tangent space, OpenGL/glTF convention (+Y green); "
                  "orm = (1, roughness, metallic 0) for glTF metallicRoughness; roughness = grey (R=G=B)",
            lod="LOD1 ~ 30 %, LOD2 ~ 10 % (>= 12 tris) of LOD0; each distinct LOD has its own atlas region"),
        atlas=dict(size_px=S, files=atlas_files, allocated_fraction=used, region_halvings=n_half,
                   n_regions=len(regions)),
        classes=classes, files_bytes=files_bytes,
        timing_s=dict(atlas_s=T_atlas, export_s=T_export, total_s=time.perf_counter() - t_start),
        prototypes=protos,
    )
    with open(os.path.join(out, "library.json"), "w") as f:
        json.dump(lib, f, indent=1)
    log(f"library: {len(protos)} prototypes, atlas {S}^2 {used * 100:.1f} %, total {time.perf_counter() - t_start:.1f} s")
    return lib


# ----------------------------------------------------------------------------------------------
# preview (from the exported files)
# ----------------------------------------------------------------------------------------------
def obj_from_npz(name, path, mat):
    d = np.load(path)
    pos, nrm, uv, idx = d["positions"], d["normals"], d["uv"], d["indices"]
    me = bpy.data.meshes.new(name)
    me.from_pydata(pos.astype(np.float64).tolist(), [], idx.astype(np.int64).tolist())
    me.update()
    uvl = me.uv_layers.new(name="UVMap")
    lv = np.empty(len(me.loops), np.int32)
    me.loops.foreach_get("vertex_index", lv)
    uvb = uv[lv].astype(np.float32).copy()
    uvb[:, 1] = 1.0 - uvb[:, 1]                       # glTF -> Blender UV convention
    uvl.data.foreach_set("uv", uvb.ravel())
    try:
        me.normals_split_custom_set_from_vertices([tuple(map(float, n)) for n in nrm])
    except Exception as e:  # pragma: no cover
        log("WARN custom normals not set:", e)
        me.shade_smooth()
    me.materials.append(mat)
    obj = rp.link(bpy.data.objects.new(name, me))
    return obj


def sfd_sample(rng, n, d0, d1, b=2.8):
    u = rng.random(n)
    return (d0 ** -b - u * (d0 ** -b - d1 ** -b)) ** (-1.0 / b)


def preview(args, out):
    rp.reset_scene()
    sc = bpy.context.scene
    rp.setup_cycles(sc, args.device, args.preview_samples)
    with open(os.path.join(out, "library.json")) as f:
        lib = json.load(f)
    imgs = {}
    for key, cs in (("basecolor", "sRGB"), ("normal", "Non-Color"), ("orm", "Non-Color")):
        im = bpy.data.images.load(os.path.join(out, lib["atlas"]["files"][key]))
        im.colorspace_settings.name = cs
        imgs[key] = im
    mat = rp.make_pbr_material("lunar_rock_atlas", imgs["basecolor"], imgs["normal"], imgs["orm"])
    protos = lib["prototypes"]
    objs = {}
    for p in protos:
        for lr in p["lods"]:
            o = obj_from_npz(f"{p['id']}_lod{lr['lod']}", os.path.join(out, lr["file"]), mat)
            o.hide_render = True
            objs[(p["id"], lr["lod"])] = o
    rp.setup_stage(args)
    pv = os.path.join(out, "preview")
    os.makedirs(pv, exist_ok=True)
    rng = np.random.default_rng(args.seed + 7)
    times = {}

    def show(o, loc, burial, meta, yaw=0.0, scale=1.0):
        inst = o.copy()
        sc.collection.objects.link(inst)
        inst.hide_render = False
        inst.scale = (scale, scale, scale)
        inst.rotation_euler = (0.0, 0.0, yaw)
        inst.location = (loc[0], loc[1], scale * (meta["bottom_offset_m"] - burial * meta["dims_m"][2]))
        return inst

    # 1. overview / close-up: LOD0 rows (large at the back, medium, pebbles in front)
    rows = [("large", 1.6, 0.5), ("medium", 0.75, 0.08), ("pebble", 0.42, 0.035)]
    large_max_b = max([p["dims_m"][1] for p in protos if p["cls"] == "large"] or [0.0])
    for cls, y, gap in rows:
        ps = sorted([p for p in protos if p["cls"] == cls], key=lambda p: p["dims_m"][0])
        yy = y + (large_max_b / 2 if cls == "large" else 0.0)
        w = [p["dims_m"][0] for p in ps]
        x = -(sum(w) + gap * (len(ps) - 1)) / 2.0
        for p, wi in zip(ps, w):
            show(objs[(p["id"], 0)], (x + wi / 2, yy), 0.15, p)
            x += wi + gap
    yl = 1.6 + large_max_b / 2
    times["overview"] = rp.render((0.0, -1.6, 1.15), (0.0, yl * 0.75, 0.15), 30.0, (1600, 900),
                                  os.path.join(pv, "preview_overview.png"), args.preview_samples)
    times["closeup"] = rp.render((0.05, 0.02, 0.30), (0.0, 0.62, 0.02), 35.0, (1600, 900),
                                 os.path.join(pv, "preview_closeup.png"), args.preview_samples)
    big = max([p for p in protos if p["cls"] == "large"], key=lambda p: p["dims_m"][0], default=None)
    if big is not None:
        # the biggest boulder sits at the right end of the large row
        L0 = big["dims_m"][0]
        ps = sorted([p for p in protos if p["cls"] == "large"], key=lambda p: p["dims_m"][0])
        w = [p["dims_m"][0] for p in ps]
        xr = (sum(w) + 0.5 * (len(ps) - 1)) / 2.0 - L0 / 2.0
        tgt = (xr, yl, 0.25 * big["dims_m"][2])
        cam = (xr - 0.9 * L0, yl - 1.3 * L0, 0.55 * L0)
        times["large_detail"] = rp.render(cam, tgt, 50.0, (1600, 900), os.path.join(pv, "preview_large_detail.png"),
                                          args.preview_samples)
    # 2. LOD comparison: per class one mid-size rock at LOD0 / LOD1 / LOD2 (left to right)
    for xc, cls in ((22.0, "large"), (28.0, "medium"), (34.0, "pebble")):
        ps = sorted([p for p in protos if p["cls"] == cls], key=lambda p: p["dims_m"][0])
        if not ps:
            continue
        p = ps[len(ps) // 2] if cls != "large" else ps[min(2, len(ps) - 1)]
        w = p["dims_m"][0]
        yc = 20.0
        for lod in range(3):
            show(objs[(p["id"], lod)], (xc + (lod - 1) * 1.8 * w, yc), 0.15, p, yaw=0.3)
        dist = 7.0 * w                                  # 50 mm lens: ~5 w visible across
        cam = (xc, yc - 0.87 * dist, 0.5 * dist)
        tgt = (xc, yc, 0.15 * p["dims_m"][2])
        times[f"lods_{cls}"] = rp.render(cam, tgt, 50.0, (1600, 900), os.path.join(pv, f"preview_lods_{cls}.png"),
                                         args.preview_samples)
        times[f"lods_{cls}_clay"] = render_clay(cam, tgt, 50.0, os.path.join(pv, f"preview_lods_{cls}_clay.png"), args)
    # 3. scatter field drawn from the LM-site SFD, world class -> LOD rules (x = -20 m)
    by_cls = {c: [p for p in protos if p["cls"] == c] for c in CLASS_ORDER}
    area = (7.0, 8.0)
    n = int(round(21.6 * area[0] * area[1] * args.scatter_density))
    D = sfd_sample(rng, n, 0.02, 1.5)
    placed = []
    n_inst = 0
    for d in sorted(D, reverse=True):
        if d < 0.05:
            cls, lod = "pebble", 0
        elif d < 0.12:
            cls, lod = "medium", 2
        elif d < 0.30:
            cls, lod = "medium", 1
        else:
            cls, lod = "large", 0
        cands = [p for p in by_cls[cls] if abs(math.log(d / max(p["dims_m"]))) <= math.log(3.0)] or by_cls[cls]
        p = cands[int(rng.integers(len(cands)))]
        s = d / max(p["dims_m"])
        for _ in range(30):
            x = -20.0 + rng.uniform(-area[0] / 2, area[0] / 2)
            y = 8.0 + rng.uniform(-area[1] / 2, area[1] / 2)
            if all((x - px) ** 2 + (y - py) ** 2 > (0.6 * (d + pd) / 2) ** 2 for px, py, pd in placed):
                break
        if d >= 0.1:
            placed.append((x, y, d))
        lo_b, hi_b = p_burial(cls, d)
        burial = 0.0 if rng.random() < 0.3 else lo_b + (hi_b - lo_b) * rng.beta(1.5, 6.0)
        show(objs[(p["id"], lod)], (x, y), burial, p, yaw=float(rng.uniform(0, 2 * math.pi)), scale=s)
        n_inst += 1
    times["scatter"] = rp.render((-20.0, 3.2, 0.85), (-20.0, 8.5, 0.0), 24.0, (1600, 900),
                                 os.path.join(pv, "preview_scatter.png"), args.preview_samples)
    times["overview_clay"] = render_clay((0.0, -1.6, 1.15), (0.0, yl * 0.75, 0.15), 30.0,
                                         os.path.join(pv, "preview_overview_clay.png"), args)
    with open(os.path.join(pv, "preview_info.json"), "w") as f:
        json.dump(dict(render_s=times, sun_elev_deg=args.sun_elev, sun_az_deg=args.sun_az,
                       sun_strength=args.sun_strength, scatter_instances=n_inst,
                       view_transform=sc.view_settings.view_transform), f, indent=1)
    log(f"preview: {len(times)} renders, scatter {n_inst} rocks")
    return times


def p_burial(cls, d):
    if d < 0.05:
        return 0.2, 0.5
    if d < 0.30:
        return 0.1, 0.4
    return 0.1, 0.35


def render_clay(cam, tgt, lens, path, args):
    clay = rp.new_material("clay")
    nb = rp.NodeBuilder(clay)
    o_ = nb.node("ShaderNodeOutputMaterial")
    b_ = nb.node("ShaderNodeBsdfPrincipled")
    nb.put(b_.inputs["Base Color"], (0.3, 0.3, 0.3, 1.0))
    nb.put(b_.inputs["Roughness"], 0.9)
    nb.nt.links.new(b_.outputs["BSDF"], o_.inputs["Surface"])
    bpy.context.view_layer.material_override = clay
    t = rp.render(cam, tgt, lens, (1600, 900), path, max(16, args.preview_samples // 2))
    bpy.context.view_layer.material_override = None
    return t


# ----------------------------------------------------------------------------------------------
def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    ap = argparse.ArgumentParser(prog="rock_library.py", description="production lunar rock library",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--out", required=True, help="library directory (UNC //wsl.localhost/... works)")
    ap.add_argument("--mode", choices=["full", "preview"], default="full")
    ap.add_argument("--seed", type=int, default=16, help="library seed (each rock derives its own seed)")
    ap.add_argument("--n-large", type=int, default=10)
    ap.add_argument("--n-medium", type=int, default=20)
    ap.add_argument("--n-pebble", type=int, default=20)
    ap.add_argument("--only", default="", help="comma separated rock ids to build (debug)")
    ap.add_argument("--atlas", type=int, default=4096, help="atlas size (px, power of two)")
    ap.add_argument("--texel-scale", type=float, default=1.0, help="multiplier on all texel targets")
    ap.add_argument("--device", choices=["OPTIX", "CUDA", "CPU"], default="OPTIX")
    ap.add_argument("--threads", type=int, default=0, help="CPU threads for Cycles (0 = auto)")
    ap.add_argument("--bake-samples", type=int, default=8, help="Cycles AA samples per bake")
    ap.add_argument("--hull-max-verts", type=int, default=64)
    ap.add_argument("--preview", action="store_true", help="render previews after generation")
    ap.add_argument("--preview-samples", type=int, default=96)
    ap.add_argument("--scatter-density", type=float, default=1.0, help="x LM-site density in the scatter preview")
    ap.add_argument("--sun-elev", type=float, default=15.0, help="preview sun elevation [deg]")
    ap.add_argument("--sun-az", type=float, default=195.0, help="preview sun azimuth [deg from +X, CCW]")
    ap.add_argument("--sun-strength", type=float, default=6.0, help="preview sun irradiance (Blender W/m^2)")
    ap.add_argument("--exposure", type=float, default=0.0)
    args = ap.parse_args(argv)
    if args.atlas & (args.atlas - 1):
        ap.error("--atlas must be a power of two")
    return args


def main():
    args = parse_args()
    out = os.path.normpath(args.out)          # '//wsl.localhost/..' -> '\\\\wsl.localhost\\..' on Windows
    os.makedirs(out, exist_ok=True)
    t = time.perf_counter()
    if args.mode == "full":
        generate(args, out)
    if args.mode == "preview" or args.preview:
        preview(args, out)
    log(f"finished mode={args.mode} in {time.perf_counter() - t:.1f} s -> {out}")


if __name__ == "__main__":
    main()
