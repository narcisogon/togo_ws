#!/usr/bin/env python3
"""Bounding boxes for the rover's CAD collision meshes (offline, venv + trimesh).

Why (docs/impl/gz_validation.md 5.1): classic Bullet turns every mesh collision into a GImpact
shape. The A300's motor meshes (46,604 triangles each, x4) and the Seyond Robin W body
(173,872) then run BVH-vs-BVH tests against the terrain's GImpact tiles every 1 ms step, and the
real-time factor drops to 0.05-0.2 on slopes. A box against a GImpact mesh is a cheap
convex-vs-triangle test. The small meshes (chassis, spacers, bumpers, enclosure, arch: 2,160
triangles) still cost 20-35 % of the main thread on the slope (docs/impl/fix_runtime.md 1.4), so by
default every collision mesh is replaced (``--min-triangles 0``; 2000 keeps the small ones).

This tool expands the robot description exactly as the launch file does (``xacro
togo_gz.urdf.xacro ns:= tf_prefix:= is_sim:=true use_fake_hardware:=false``, ROS sourced in a
subshell) or reads a given URDF, loads every ``<collision>`` mesh, and replaces each mesh with at
least ``--min-triangles`` triangles by bounding boxes in the mesh's own frame:

* first the smallest single primitive: the axis-aligned box, or a cylinder along a mesh axis
  (minimum enclosing circle of the cross-section) when that is clearly smaller
  (``--cylinder-preference``);
* a bounding primitive also fills concave outlines and rounded edges, which lowers the ground
  clearance: one box under the arched suspension beam reaches 41 mm lower than the beam, one box
  around a motor 38 mm lower than its rounded bottom edge. The tool therefore measures, for the
  mesh direction that points at the ground where the mesh is mounted (from the URDF joint chain),
  how much further than the mesh each candidate reaches (``down_gap_mm``, 2 mm bins), and takes the
  candidate with the fewest boxes that stays within ``--max-down-gap`` (15 mm): the single
  primitive, or K slab boxes (the mesh cut into K slabs along one axis, each bounded by the box of
  its surface clipped to the slab; K = 2..16).

The result goes to ``config/robot/a300_collision_simplified.yaml``; ``launch/apollo16_sim.launch.py``
(``robot_collision:=simplified``, the default) only substitutes the XML at launch time: it composes
each box's pose with the collision's ``<origin>``, so the boxes cover the space of the original
mesh. Visuals and inertials are not touched.

Run in WSL from ``src/lunar_world_gen``::

    ~/.venvs/lunar_world/bin/python tools/robot_collision_simplify.py            # writes the YAML
    ~/.venvs/lunar_world/bin/python tools/robot_collision_simplify.py --urdf robot.urdf --out x.yaml
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import math
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent
DEFAULT_OUT = PKG_ROOT / "config" / "robot" / "a300_collision_simplified.yaml"
DEFAULT_INSTALL = Path("/home/nargon/togo_ws_native_clean/install")
DEFAULT_XACRO_REL = "togo_gz/share/togo_gz/urdf/togo_gz.urdf.xacro"
# the launch file's xacro arguments (launch/apollo16_sim.launch.py, as togo_deploy control.launch.py)
DEFAULT_XACRO_ARGS = ["ns:=", "tf_prefix:=", "is_sim:=true", "use_fake_hardware:=false"]
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
SCHEMA_VERSION = 1
# cylinder axis = local z (URDF/SDF); rpy that turns local z into the mesh's x / y / z axis
AXIS_RPY = {0: (0.0, math.pi / 2.0, 0.0), 1: (-math.pi / 2.0, 0.0, 0.0), 2: (0.0, 0.0, 0.0)}
AXIS_NAME = "xyz"


# ----------------------------------------------------------------------------------- geometry
def min_enclosing_circle(pts: np.ndarray, seed: int = 0) -> tuple[np.ndarray, float]:
    """Smallest circle containing all 2-D points (Welzl, iterative, expected O(n))."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        raise ValueError("no points")
    if len(pts) > 3:
        try:
            from scipy.spatial import ConvexHull
            pts = pts[ConvexHull(pts).vertices]
        except Exception:  # noqa: BLE001 - degenerate (collinear) input: use all points
            pass
    P = pts[np.random.default_rng(seed).permutation(len(pts))]

    def two(a, b):
        c = (a + b) / 2.0
        return c, float(np.linalg.norm(a - c))

    def three(a, b, c):
        d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if abs(d) < 1e-18:  # collinear: the widest pair
            return max((two(a, b), two(a, c), two(b, c)), key=lambda cr: cr[1])
        a2, b2, c2 = a @ a, b @ b, c @ c
        ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d
        uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d
        u = np.array([ux, uy])
        return u, float(np.linalg.norm(a - u))

    def inside(c, r, p):
        return float(np.linalg.norm(p - c)) <= r * (1.0 + 1e-12) + 1e-12

    c, r = P[0].copy(), 0.0
    for i in range(1, len(P)):
        if inside(c, r, P[i]):
            continue
        c, r = P[i].copy(), 0.0
        for j in range(i):
            if inside(c, r, P[j]):
                continue
            c, r = two(P[i], P[j])
            for k in range(j):
                if not inside(c, r, P[k]):
                    c, r = three(P[i], P[j], P[k])
    return c, r


def fit_primitive(vertices: np.ndarray, cylinder_preference: float = 0.9) -> dict:
    """Smallest-volume bounding box or axis-aligned cylinder of `vertices` (mesh frame, metres).

    Returns {type, size | radius+length, xyz (centre in the mesh frame), rpy (orientation in the mesh
    frame; a cylinder's axis is its local z), volume_m3, candidates}. Every vertex lies inside the
    primitive (asserted with a 1e-9 m tolerance plus a relative 1e-9).
    """
    v = np.asarray(vertices, dtype=float)
    lo, hi = v.min(axis=0), v.max(axis=0)
    ext = hi - lo
    centre = (lo + hi) / 2.0
    box = {"type": "box", "size": ext.tolist(), "xyz": centre.tolist(), "rpy": [0.0, 0.0, 0.0],
           "volume_m3": float(np.prod(ext))}
    cands = {"box": box["volume_m3"]}
    best_cyl = None
    for k in range(3):
        i, j = [a for a in range(3) if a != k]
        c2, r = min_enclosing_circle(v[:, [i, j]])
        length = float(ext[k])
        vol = math.pi * r * r * length
        cands[f"cylinder_{AXIS_NAME[k]}"] = vol
        xyz = np.empty(3)
        xyz[i], xyz[j], xyz[k] = c2[0], c2[1], centre[k]
        cyl = {"type": "cylinder", "radius": float(r), "length": length, "axis": AXIS_NAME[k],
               "xyz": xyz.tolist(), "rpy": list(AXIS_RPY[k]), "volume_m3": float(vol)}
        if best_cyl is None or vol < best_cyl["volume_m3"]:
            best_cyl = cyl
    prim = best_cyl if best_cyl["volume_m3"] < cylinder_preference * box["volume_m3"] else box
    prim["candidates_m3"] = {k: float(f"{x:.6g}") for k, x in cands.items()}
    assert primitive_contains(prim, v), "fitted primitive does not contain the mesh"
    return prim


def slab_clip_points(tri: np.ndarray, axis: int, a: float, b: float) -> np.ndarray:
    """Vertices of all triangles (n, 3, 3) clipped to a <= p[axis] <= b: the corners inside the slab
    plus every edge/plane intersection, i.e. the exact extreme points of the surface in the slab."""
    flat = tri.reshape(-1, 3)
    pts = [flat[(flat[:, axis] >= a) & (flat[:, axis] <= b)]]
    for i, j in ((0, 1), (1, 2), (2, 0)):
        p, q = tri[:, i], tri[:, j]
        for c in (a, b):
            dp, dq = p[:, axis] - c, q[:, axis] - c
            cut = dp * dq < 0
            if np.any(cut):
                t = dp[cut] / (dp[cut] - dq[cut])
                pts.append(p[cut] + t[:, None] * (q[cut] - p[cut]))
    return np.vstack(pts)


def fit_slab_boxes(tri: np.ndarray, axis: int, k: int) -> dict:
    """Compound of `k` boxes: the mesh cut into k equal slabs along `axis`, each slab bounded by the
    box of its surface. The union contains the mesh (a ray from an interior point in a direction
    normal to `axis` leaves the mesh through the surface inside the same slab) and follows concave
    outlines such as the arched suspension beam that a single box would fill."""
    flat = tri.reshape(-1, 3)
    lo, hi = flat[:, axis].min(), flat[:, axis].max()
    edges = np.linspace(lo, hi, k + 1)
    parts = []
    for a, b in zip(edges[:-1], edges[1:]):
        pts = slab_clip_points(tri, axis, a, b)
        if len(pts) == 0:
            continue
        bl, bh = pts.min(axis=0), pts.max(axis=0)
        bl[axis], bh[axis] = a, b
        parts.append({"type": "box", "size": (bh - bl).tolist(), "xyz": ((bl + bh) / 2).tolist(),
                      "rpy": [0.0, 0.0, 0.0], "volume_m3": float(np.prod(bh - bl))})
    return {"type": "compound", "axis": AXIS_NAME[axis], "parts": parts,
            "volume_m3": float(sum(p["volume_m3"] for p in parts))}


SLAB_COUNTS = (2, 3, 4, 6, 8, 12, 16)


def _half(p: dict, ax: int) -> float:
    """Half extent of an axis-aligned part (box, or cylinder with its axis along a mesh axis)."""
    if p["type"] == "box":
        return p["size"][ax] / 2
    return p["length"] / 2 if p["axis"] == AXIS_NAME[ax] else p["radius"]


def down_profiles(tri: np.ndarray, down: str = "-z", step: float = 0.002) -> tuple:
    """The mesh's extreme point in the `down` direction ('-z', '+y', ...) per bin of `step` along each
    of the two other axes: (d, sign, [(bin_axis, bin_centres, extreme * sign), ...])."""
    d = AXIS_NAME.index(down[1])
    s = 1.0 if down[0] == "+" else -1.0
    flat = tri.reshape(-1, 3)
    profs = []
    for axis in [a for a in range(3) if a != d]:
        lo, hi = flat[:, axis].min(), flat[:, axis].max()
        mids, ext = [], []
        for a in np.arange(lo, hi, step):
            b = min(a + step, hi)
            pts = slab_clip_points(tri, axis, a, b)
            if len(pts):
                mids.append((a + b) / 2)
                ext.append(float((s * pts[:, d]).max()))
        profs.append((axis, np.asarray(mids), np.asarray(ext)))
    return d, s, profs


def gap_from_profiles(prim: dict, prof: tuple) -> float:
    """down_gap_mm of `prim` against precomputed down_profiles."""
    d, s, profs = prof
    parts = prim["parts"] if prim["type"] == "compound" else [prim]
    worst = 0.0
    for axis, mids, ext in profs:
        reach = np.full(len(mids), -np.inf)
        for p in parts:
            cover = np.abs(mids - p["xyz"][axis]) <= _half(p, axis)
            reach[cover] = np.maximum(reach[cover], s * p["xyz"][d] + _half(p, d))
        ok = np.isfinite(reach)
        if np.any(ok):
            worst = max(worst, float(np.max(reach[ok] - ext[ok])))
    return 1000.0 * worst


def down_gap_mm(tri: np.ndarray, prim: dict, down: str = "-z", step: float = 0.002) -> float:
    """How much further than the mesh the primitive reaches in the `down` direction ('-z', '+y', ...):
    the largest difference between the primitive's and the mesh's extreme point in that direction,
    over bins of `step` along each of the two other axes. This is the ground clearance the primitive
    gives away where it is mounted with that mesh direction pointing at the ground."""
    return gap_from_profiles(prim, down_profiles(tri, down, step))


def choose_primitive(tri: np.ndarray, cylinder_preference: float = 0.9, downs: tuple[str, ...] = ("-z",),
                     max_gap_mm: float = 15.0) -> dict:
    """The simplest bounding primitive that keeps the clearance: the single box / cylinder of
    fit_primitive, else a compound of K slab boxes along one mesh axis (K in SLAB_COUNTS), the
    candidate with the fewest parts whose down_gap_mm (worst over `downs`, the mesh directions that
    point at the ground where the mesh is mounted) is <= max_gap_mm; ties go to the smaller volume.
    If no candidate reaches max_gap_mm, the one with the smallest gap."""
    v = tri.reshape(-1, 3)
    single = fit_primitive(v, cylinder_preference)
    profiles = [down_profiles(tri, dn) for dn in downs]

    def gap(p):
        return max(gap_from_profiles(p, pr) for pr in profiles) if profiles else 0.0

    g1 = gap(single)
    if g1 <= max_gap_mm:
        return dict(single, down_gap_mm=g1, down_gap_single_mm=g1)
    cands = [(g1, 1, single["volume_m3"], single)]
    best = None
    for k in SLAB_COUNTS:
        for axis in range(3):
            comp = fit_slab_boxes(tri, axis, k)
            cands.append((gap(comp), len(comp["parts"]), comp["volume_m3"], comp))
        ok = [c for c in cands if c[0] <= max_gap_mm]
        if ok:
            best = min(ok, key=lambda c: (c[1], c[2]))
            break
    if best is None:
        best = min(cands, key=lambda c: (c[0], c[1]))
    prim = best[3]
    prim["candidates_m3"] = dict(single["candidates_m3"],
                                 **{f"slabs_{c[3]['axis']}{c[1]}": float(f"{c[2]:.6g}") for c in cands[-3:]})
    prim["down_gap_mm"] = best[0]
    prim["down_gap_single_mm"] = g1
    return prim


def rounded_primitive(prim: dict) -> dict:
    """The primitive with 1 um rounding for the YAML: centre to the nearest um (shift <= 0.5 um per
    axis), sizes / radius / length rounded up plus 2 um, so the result still bounds the mesh."""
    up = lambda x: math.ceil(x * 1e6 - 1e-6) / 1e6 + 2e-6  # noqa: E731
    f6 = lambda x: float(f"{x:.6f}")  # noqa: E731
    if prim["type"] == "compound":
        return {"type": "compound", "axis": prim["axis"], "parts": [rounded_primitive(p) for p in prim["parts"]]}
    p = {"type": prim["type"], "xyz": [f6(c) for c in prim["xyz"]], "rpy": [f6(r) for r in prim["rpy"]]}
    if prim["type"] == "box":
        p["size"] = [f6(up(s)) for s in prim["size"]]
    else:
        p.update({"axis": prim["axis"], "radius": f6(up(prim["radius"])), "length": f6(up(prim["length"]))})
    return p


def _inside(prim: dict, v: np.ndarray, tol: float) -> np.ndarray:
    c = np.asarray(prim["xyz"], dtype=float)
    d = np.asarray(v, dtype=float) - c
    if prim["type"] == "box":
        half = np.asarray(prim["size"], dtype=float) / 2.0
        return np.all(np.abs(d) <= half * (1 + 1e-9) + tol, axis=1)
    k = AXIS_NAME.index(prim["axis"])
    i, j = [a for a in range(3) if a != k]
    radial = np.hypot(d[:, i], d[:, j])
    return (radial <= prim["radius"] * (1 + 1e-9) + tol) & (np.abs(d[:, k]) <= prim["length"] / 2.0 * (1 + 1e-9) + tol)


def primitive_contains(prim: dict, v: np.ndarray, tol: float = 1e-9) -> bool:
    """True if every point of `v` (mesh frame) lies inside the primitive (any part of a compound)."""
    v = np.asarray(v, dtype=float).reshape(-1, 3)
    if prim["type"] == "compound":
        ok = np.zeros(len(v), dtype=bool)
        for p in prim["parts"]:
            ok |= _inside(p, v, tol)
        return bool(np.all(ok))
    return bool(np.all(_inside(prim, v, tol)))


# ----------------------------------------------------------------------------------- inputs
def expand_xacro(xacro: Path, args: list[str], install: Path) -> str:
    """The robot description as the launch file generates it (ROS sourced in a subshell)."""
    setup = [ROS_SETUP, install / "setup.bash"]
    src = " && ".join(f"source {shlex.quote(str(s))}" for s in setup if s.exists())
    cmd = f"{src} && xacro {shlex.quote(str(xacro))} " + " ".join(shlex.quote(a) for a in args)
    res = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=120)
    if res.returncode != 0:
        raise SystemExit(f"xacro failed ({res.returncode}): {res.stderr[-2000:]}")
    return res.stdout


def share_dirs(install: Path, extra: list[str]) -> list[Path]:
    dirs = [Path(p) for p in extra]
    dirs += sorted(install.glob("*/share")) if install.is_dir() else []
    dirs.append(Path("/opt/ros/jazzy/share"))
    return dirs


def resolve_uri(uri: str, shares: list[Path]) -> Path | None:
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        for s in shares:
            p = s / pkg / rel
            if p.is_file():
                return p
        return None
    p = Path(uri[len("file://"):] if uri.startswith("file://") else uri)
    return p if p.is_file() else None


def parse_scale(text: str | None) -> list[float]:
    vals = [float(x) for x in (text or "1 1 1").split()]
    if len(vals) == 1:
        vals *= 3
    if len(vals) != 3:
        raise ValueError(f"bad mesh scale {text!r}")
    return vals


def rpy_matrix(rpy) -> np.ndarray:
    r, p, y = (float(a) for a in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin_rpy(el) -> list[float]:
    o = el.find("origin") if el is not None else None
    return [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]


def link_rotations(root) -> dict[str, np.ndarray]:
    """Rotation of every link relative to the root link (joints at zero position)."""
    joints = [(j.find("parent").get("link"), j.find("child").get("link"), _origin_rpy(j))
              for j in root.findall("joint") if j.find("parent") is not None and j.find("child") is not None]
    children = {c for _, c, _ in joints}
    rot = {ln.get("name"): np.eye(3) for ln in root.findall("link") if ln.get("name") not in children}
    pending = list(joints)
    while pending:
        done = [jt for jt in pending if jt[0] in rot]
        if not done:
            break
        for p, c, rpy in done:
            rot[c] = rot[p] @ rpy_matrix(rpy)
            pending.remove((p, c, rpy))
    return rot


def down_axis(R_mesh: np.ndarray) -> str:
    """Mesh axis ('-z', '+y', ...) closest to the root link's -z (the ground when the robot is upright)."""
    d = R_mesh.T @ np.array([0.0, 0.0, -1.0])
    k = int(np.argmax(np.abs(d)))
    return f"{'+' if d[k] > 0 else '-'}{AXIS_NAME[k]}"


def collision_meshes(urdf_xml: str) -> dict[tuple[str, tuple[float, ...]], dict]:
    """{(uri, scale): {'links': [...], 'downs': {mesh axes pointing at the ground}}} for every
    <collision> with a <mesh> geometry."""
    root = ET.fromstring(urdf_xml)
    rot = link_rotations(root)
    out: dict[tuple[str, tuple[float, ...]], dict] = {}
    for link in root.iter("link"):
        for col in link.findall("collision"):
            m = col.find("geometry/mesh")
            if m is None or not m.get("filename"):
                continue
            key = (m.get("filename"), tuple(parse_scale(m.get("scale"))))
            e = out.setdefault(key, {"links": [], "downs": set()})
            e["links"].append(link.get("name"))
            e["downs"].add(down_axis(rot.get(link.get("name"), np.eye(3)) @ rpy_matrix(_origin_rpy(col))))
    return out


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ----------------------------------------------------------------------------------- main
def build_spec(urdf_xml: str, shares: list[Path], min_triangles: int, cylinder_preference: float,
               source: dict, max_gap_mm: float = 15.0) -> dict:
    import trimesh

    meshes, kept = [], []
    tri_before = tri_after = n_before = n_replaced = 0
    for (uri, scale), info in sorted(collision_meshes(urdf_xml).items()):
        links, downs = info["links"], tuple(sorted(info["downs"]))
        path = resolve_uri(uri, shares)
        if path is None:
            raise SystemExit(f"collision mesh not found: {uri} (searched {[str(s) for s in shares]})")
        mesh = trimesh.load(str(path), force="mesh", process=False)
        n_tri = int(len(mesh.faces))
        n_before += len(links)
        tri_before += n_tri * len(links)
        if n_tri < min_triangles:
            kept.append({"uri": uri, "scale": list(scale), "triangles": n_tri, "used_by": links})
            tri_after += n_tri * len(links)
            continue
        v = np.asarray(mesh.vertices, dtype=float) * np.asarray(scale)
        tri = np.asarray(mesh.triangles, dtype=float) * np.asarray(scale)
        prim = choose_primitive(tri, cylinder_preference, downs, max_gap_mm)
        try:
            hull_vol = float(trimesh.convex.convex_hull(v).volume)
        except Exception:  # noqa: BLE001
            hull_vol = float("nan")
        lo, hi = v.min(axis=0), v.max(axis=0)
        p = rounded_primitive(prim)
        # the rounded (1 um, sizes rounded up + 2 um) primitive must still contain the mesh
        assert primitive_contains(p, v, tol=1e-9), uri
        p["volume_m3"] = float(f"{prim['volume_m3']:.6g}")
        p["fill"] = float(f"{hull_vol / prim['volume_m3']:.3f}") if prim["volume_m3"] > 0 else None
        p["candidates_m3"] = prim["candidates_m3"]
        # ground clearance given away: how much further than the mesh the primitive reaches towards
        # the ground (mesh axes `down_axes`), one-box value for reference
        p["down_gap_mm"] = float(f"{max(down_gap_mm(tri, p, d) for d in downs):.1f}")
        p["down_gap_single_box_mm"] = float(f"{prim['down_gap_single_mm']:.1f}")
        r6 = lambda a: [float(f"{x:.6f}") for x in a]  # noqa: E731
        meshes.append({"uri": uri, "scale": list(scale), "file": str(path), "bytes": path.stat().st_size,
                       "sha256": sha256_of(path), "triangles": n_tri, "used_by": links, "down_axes": list(downs),
                       "bounds_min": r6(lo), "bounds_max": r6(hi), "hull_volume_m3": float(f"{hull_vol:.6g}"),
                       "primitive": p})
        n_replaced += len(links)
    return {
        "schema_version": SCHEMA_VERSION,
        "generator": "lunar_world_gen tools/robot_collision_simplify.py",
        "created_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": source,
        "min_triangles": int(min_triangles),
        "cylinder_preference": float(cylinder_preference),
        "max_down_gap_mm": float(max_gap_mm),
        "summary": {"collision_meshes": n_before, "replaced_collisions": n_replaced,
                    "mesh_triangles_before": tri_before, "mesh_triangles_after": tri_after},
        "meshes": meshes,
        "kept": kept,
    }


HEADER = """\
# Bounding boxes for the robot's CAD collision meshes (all of them by default, --min-triangles 0).
# generated by tools/robot_collision_simplify.py - do not edit by hand; re-run the tool after a
# robot-description or Clearpath package update (the launch file compares each mesh's sha256 and
# keeps the original mesh, with a warning, when it changed).
# Used by launch/apollo16_sim.launch.py robot_collision:=simplified (default): every <collision>
# whose <mesh filename> and scale match an entry is replaced by the primitive (type box / cylinder)
# or by one <collision> per part (type compound), each placed at <origin> * (xyz, rpy) - xyz/rpy are
# the part's centre and orientation in the mesh frame (a cylinder's axis is its local z).
# down_gap_mm: how much further than the mesh the boxes reach towards the ground (down_axes = the
# mesh axis pointing down where it is mounted). Visuals and inertials are unchanged.
"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", help="read this URDF instead of expanding the xacro")
    ap.add_argument("--install", default=str(DEFAULT_INSTALL), help="native TOGO install (setup.bash, shares)")
    ap.add_argument("--xacro", help=f"default: <install>/{DEFAULT_XACRO_REL}")
    ap.add_argument("--xacro-arg", action="append", default=None,
                    help=f"xacro argument (repeatable; default: {' '.join(DEFAULT_XACRO_ARGS)})")
    ap.add_argument("--share", action="append", default=[], help="extra share dir searched first for package://")
    ap.add_argument("--min-triangles", type=int, default=0,
                    help="replace collision meshes with at least this many triangles (default 0: all; 2000 replaces "
                         "only the motor, beam and LiDAR meshes)")
    ap.add_argument("--cylinder-preference", type=float, default=0.9,
                    help="use a cylinder only if its volume < this x the box volume (default 0.9)")
    ap.add_argument("--max-down-gap", type=float, default=15.0,
                    help="largest ground clearance [mm] a primitive may give away against its mesh; more slab boxes "
                         "are used until it holds (default 15)")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--save-urdf", help="also write the expanded URDF here")
    a = ap.parse_args(argv)

    install = Path(a.install)
    if a.urdf:
        urdf = Path(a.urdf).read_text()
        source = {"urdf": str(Path(a.urdf).resolve())}
    else:
        xacro = Path(a.xacro) if a.xacro else install / DEFAULT_XACRO_REL
        xargs = a.xacro_arg if a.xacro_arg is not None else list(DEFAULT_XACRO_ARGS)
        urdf = expand_xacro(xacro, xargs, install)
        source = {"xacro": str(xacro), "xacro_args": xargs}
    if a.save_urdf:
        Path(a.save_urdf).write_text(urdf)
    spec = build_spec(urdf, share_dirs(install, a.share), a.min_triangles, a.cylinder_preference, source,
                      a.max_down_gap)

    import yaml

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # atomic replace: running launches read this file at any time
    tmp = out.with_name(f".{out.name}.tmp")
    tmp.write_text(HEADER + yaml.safe_dump(spec, sort_keys=False, default_flow_style=None, width=110))
    tmp.replace(out)
    s = spec["summary"]
    print(f"{out}: {s['replaced_collisions']} of {s['collision_meshes']} collision meshes replaced; "
          f"mesh triangles {s['mesh_triangles_before']:,} -> {s['mesh_triangles_after']:,}")
    for m in spec["meshes"]:
        p = m["primitive"]
        dims = (f"box {p['size']}" if p["type"] == "box"
                else f"{len(p['parts'])} slab boxes along {p['axis']}" if p["type"] == "compound"
                else f"cylinder r={p['radius']} l={p['length']} along {p['axis']}")
        print(f"  {m['uri'].rsplit('/', 1)[-1]:24s} {m['triangles']:7,d} tris x{len(m['used_by'])} -> {dims} "
              f"(fill {p['fill']}; reaches {p['down_gap_mm']} mm further towards the ground ({','.join(m['down_axes'])})"
              f" than the mesh, one box: {p['down_gap_single_box_mm']} mm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
