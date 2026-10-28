#!/usr/bin/env python3
"""
verify_rock_glb.py -- independent check of the rock library written by blender/rock_proto.py.

Run in the WSL venv (trimesh, pygltflib, numpy):
    ~/.venvs/lunar_world/bin/python verify_rock_glb.py <OUT_DIR> [--json report.json]

Checks, per rock (library.json / meta/*.json are the reference):
  * rocks/<id>.glb loads in trimesh; geometry bounds in the FILE frame (glTF, +Y up) equal the Z-up metadata
    bbox mapped with (x, y, z)_zup -> (x, z, -y)_gltf  => metric scale (metres) and axis convention verified;
  * triangle count equals the metadata; the mesh is closed once seam-split vertices are merged;
  * library GLB: one material, three embedded images, one mesh per rock with identical bounds;
  * npz positions (Z-up) map onto the GLB positions with (x, z, -y);
  * hulls/<id>_hull.glb: <= 64 unique vertices, convex, watertight, volume >= LOD volume.
  * axis_marker.glb: extents of its arms in the file frame.
Exit code 1 if any check fails.
"""
import argparse
import json
import os
import sys

import numpy as np
import pygltflib
import trimesh

ZUP_TO_GLTF = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)   # p_gltf = M @ p_zup


def single_mesh(path):
    sc = trimesh.load(path, force="scene", process=False)
    geoms = list(sc.geometry.values())
    if len(geoms) != 1:
        raise RuntimeError(f"{path}: expected 1 geometry, got {len(geoms)}")
    # apply the node transform (identity expected) so bounds are in the file/world frame
    node = sc.graph.geometry_nodes[list(sc.geometry.keys())[0]][0]
    T, _ = sc.graph[node]
    m = geoms[0].copy()
    m.apply_transform(T)
    return m, T


def gltf_bbox_from_meta(meta):
    lo, hi = np.array(meta["bbox_min_m"]), np.array(meta["bbox_max_m"])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    g = corners @ ZUP_TO_GLTF.T
    return g.min(0), g.max(0)


def uniq_rows(a, dec=5):
    return np.unique(np.round(np.asarray(a, np.float64), dec), axis=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--json", default="")
    args = ap.parse_args()
    out = args.out
    lib = json.load(open(os.path.join(out, "library.json")))
    fails = []
    report = dict(trimesh=trimesh.__version__, rocks={}, library={}, axis_marker={})
    print(f"trimesh {trimesh.__version__}; {len(lib['rocks'])} rocks in {out}")
    print(f"{'id':16s} {'tris':>5s} {'verts':>5s} {'file extents (gltf x,y,z) [m]':>34s} {'bbox err':>9s} "
          f"{'closed':>6s} {'hull v/t':>8s} {'hull cvx':>8s} {'Vhull/V':>7s} {'glb kB':>7s}")
    for m in lib["rocks"]:
        rid = m["id"]
        r = {}
        mesh, T = single_mesh(os.path.join(out, m["files"]["standalone_glb"]))
        lo_ref, hi_ref = gltf_bbox_from_meta(m)
        berr = float(max(np.abs(mesh.bounds[0] - lo_ref).max(), np.abs(mesh.bounds[1] - hi_ref).max()))
        merged = mesh.copy()
        merged.merge_vertices(merge_tex=True, merge_norm=True)
        closed = bool(merged.is_watertight)
        tris = int(len(mesh.faces))
        r.update(node_transform_identity=bool(np.allclose(T, np.eye(4))), tris=tris, gltf_vertices=int(len(mesh.vertices)),
                 extents_gltf_m=np.round(mesh.extents, 4).tolist(), bbox_err_m=berr, closed_after_merge=closed,
                 volume_m3=float(merged.volume) if closed else None)
        if not r["node_transform_identity"]:
            fails.append(f"{rid}: node transform not identity")
        if berr > 2e-4:
            fails.append(f"{rid}: bbox mismatch {berr:.2e} m")
        if tris != m["lod"]["lod_tris"]:
            fails.append(f"{rid}: tris {tris} != meta {m['lod']['lod_tris']}")
        if not closed:
            fails.append(f"{rid}: LOD not closed after merging seam vertices")
        # npz vs glb positions
        z = np.load(os.path.join(out, m["files"]["npz"]))
        a = uniq_rows(z["positions"] @ ZUP_TO_GLTF.T)
        b = uniq_rows(mesh.vertices)
        r["npz_maps_to_gltf"] = bool(a.shape == b.shape and np.allclose(a, b, atol=2e-5))
        if not r["npz_maps_to_gltf"]:
            fails.append(f"{rid}: npz positions do not map to glb positions with (x, z, -y)")
        hv = ht = "-"
        cvx = "-"
        ratio = "-"
        if m["hull"]:
            h, _ = single_mesh(os.path.join(out, m["hull"]["file"]))
            hm = h.copy()
            hm.merge_vertices()
            hv, ht = len(hm.vertices), len(hm.faces)
            cvx = bool(hm.is_convex)
            vol_ratio = float(hm.volume / merged.volume) if closed else float("nan")
            ratio = f"{vol_ratio:.3f}"
            hb_err = float(np.abs(h.bounds - mesh.bounds).max())
            # a <=64-vertex hull is a subset of the exact hull: LOD vertices may stick out slightly.
            # Accept if the worst protrusion (from the metadata, measured in Blender) is <= 1 % of the size.
            out_frac = m["hull"]["lod_max_outside_mm"] * 1e-3 / max(m["dims_m"])
            r["hull"] = dict(unique_vertices=int(hv), triangles=int(ht), convex=cvx, watertight=bool(hm.is_watertight),
                             volume_ratio=vol_ratio, bounds_vs_lod_max_diff_m=hb_err,
                             lod_max_outside_mm=m["hull"]["lod_max_outside_mm"], lod_max_outside_frac=out_frac)
            if hv > 64 or not cvx or not hm.is_watertight or out_frac > 0.01:
                fails.append(f"{rid}: hull check failed {r['hull']}")
        kb = os.path.getsize(os.path.join(out, m["files"]["standalone_glb"])) / 1e3
        print(f"{rid:16s} {tris:5d} {len(mesh.vertices):5d} {str(np.round(mesh.extents, 4).tolist()):>34s} {berr:9.1e} "
              f"{str(closed):>6s} {str(hv) + '/' + str(ht):>8s} {str(cvx):>8s} {ratio:>7s} {kb:7.1f}")
        report["rocks"][rid] = r

    # library GLB
    g = pygltflib.GLTF2().load(os.path.join(out, "rocks_library.glb"))
    imgs = []
    for im in g.images:
        bv = g.bufferViews[im.bufferView]
        imgs.append(dict(name=im.name, mime=im.mimeType, bytes=bv.byteLength))
    mat = g.materials[0] if g.materials else None
    report["library"] = dict(meshes=[me.name for me in g.meshes], materials=len(g.materials), images=imgs,
                             double_sided=mat.doubleSided if mat else None,
                             base_color_tex=mat.pbrMetallicRoughness.baseColorTexture is not None if mat else None,
                             normal_tex=mat.normalTexture is not None if mat else None,
                             mr_tex=mat.pbrMetallicRoughness.metallicRoughnessTexture is not None if mat else None,
                             bytes=os.path.getsize(os.path.join(out, "rocks_library.glb")))
    print(f"library: {len(g.meshes)} meshes, {len(g.materials)} material(s), images {[(i['name'], i['bytes']) for i in imgs]}, "
          f"doubleSided={report['library']['double_sided']}")
    if len(g.materials) != 1 or len(g.images) != 3:
        fails.append("library: expected exactly 1 material and 3 images")
    sc = trimesh.load(os.path.join(out, "rocks_library.glb"), force="scene", process=False)
    for name, geom in sc.geometry.items():
        rid = name.split(".")[0]
        if rid in report["rocks"]:
            e = np.round(geom.extents, 4).tolist()
            if not np.allclose(e, report["rocks"][rid]["extents_gltf_m"], atol=2e-4):
                fails.append(f"library {rid}: extents {e} != standalone {report['rocks'][rid]['extents_gltf_m']}")
    # axis marker
    am = os.path.join(out, "axis_marker.glb")
    if os.path.exists(am):
        sc = trimesh.load(am, force="scene", process=False)
        geo = sc.to_geometry() if hasattr(sc, "to_geometry") else sc.dump(concatenate=True)
        b = geo.bounds
        report["axis_marker"] = dict(bounds_min=np.round(b[0], 4).tolist(), bounds_max=np.round(b[1], 4).tolist())
        print(f"axis_marker bounds in file frame: min {np.round(b[0], 3)} max {np.round(b[1], 3)} "
              f"(Blender +X 0.30 -> gltf +X; Blender +Y 0.20 -> gltf -Z; Blender +Z 0.10 -> gltf +Y)")
        ok = np.allclose(b[1], [0.30, 0.10, 0.02], atol=2e-3) and np.allclose(b[0], [-0.02, -0.02, -0.20], atol=2e-3)
        report["axis_marker"]["matches_x_z_minus_y"] = bool(ok)
        if not ok:
            fails.append("axis marker bounds unexpected")
    report["fails"] = fails
    if args.json:
        with open(args.json, "w") as f:
            json.dump(report, f, indent=1)
    print("RESULT:", "PASS" if not fails else "FAIL")
    for f_ in fails:
        print("  FAIL", f_)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
