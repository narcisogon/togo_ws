"""tools/robot_collision_simplify.py: bounding primitives for the robot's CAD collision meshes."""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG / "tools"))
import robot_collision_simplify as rcs  # noqa: E402


def _launch_module():
    spec = importlib.util.spec_from_file_location("apollo16_sim_launch_ct", PKG / "launch" / "apollo16_sim.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _cylinder_tris(r=0.05, h=0.2, n=64, axis=1, centre=(0.1, -0.2, 0.3)):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    lo = np.insert(ring, 2, -h / 2, axis=1)
    hi = np.insert(ring, 2, h / 2, axis=1)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [[lo[i], lo[j], hi[i]], [hi[i], lo[j], hi[j]], [[0, 0, -h / 2], lo[j], lo[i]], [[0, 0, h / 2], hi[i], hi[j]]]
    t = np.asarray(tris, dtype=float)
    perm = {0: [2, 0, 1], 1: [0, 2, 1], 2: [0, 1, 2]}[axis]    # local z -> requested axis
    return t[..., perm] + np.asarray(centre)


def _arch_tris(length=0.6, height=0.1, thick=0.01, rise=0.04, n=60):
    """A plate whose bottom edge is an arch (like the A300 suspension beam), triangulated as a prism."""
    x = np.linspace(-length / 2, length / 2, n + 1)
    bottom = -height / 2 + rise * (1 - (2 * x / length) ** 2)
    top = np.full_like(x, height / 2)
    tris = []
    for y in (0.0, thick):
        for i in range(n):
            p = [[x[i], y, bottom[i]], [x[i + 1], y, bottom[i + 1]], [x[i + 1], y, top[i + 1]], [x[i], y, top[i]]]
            tris += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    for i in range(n):   # bottom and top strips
        tris += [[[x[i], 0, bottom[i]], [x[i + 1], 0, bottom[i + 1]], [x[i + 1], thick, bottom[i + 1]]],
                 [[x[i], 0, top[i]], [x[i + 1], thick, top[i + 1]], [x[i + 1], 0, top[i + 1]]]]
    return np.asarray(tris, dtype=float)


def test_min_enclosing_circle():
    c, r = rcs.min_enclosing_circle(np.array([[0, 0], [2, 0], [1, 0.1]]))
    assert r == pytest.approx(1.0) and c == pytest.approx([1.0, 0.0])
    rnd = np.random.default_rng(4)
    pts = rnd.normal(size=(500, 2))
    c, r = rcs.min_enclosing_circle(pts)
    assert np.max(np.linalg.norm(pts - c, axis=1)) <= r + 1e-9


def test_fit_primitive_box_and_cylinder():
    box = np.random.default_rng(1).uniform([-0.1, -0.2, 0.0], [0.3, 0.1, 0.05], size=(400, 3))
    box = np.vstack([box, [[-0.1, -0.2, 0.0], [0.3, 0.1, 0.05]]])
    p = rcs.fit_primitive(box)
    assert p["type"] == "box" and p["size"] == pytest.approx([0.4, 0.3, 0.05]) and p["xyz"] == pytest.approx([0.1, -0.05, 0.025])
    cyl = _cylinder_tris(axis=1).reshape(-1, 3)
    p = rcs.fit_primitive(cyl)
    assert p["type"] == "cylinder" and p["axis"] == "y"
    assert p["radius"] == pytest.approx(0.05, rel=1e-6) and p["length"] == pytest.approx(0.2)
    assert p["rpy"] == pytest.approx([-math.pi / 2, 0, 0])      # local z -> mesh y
    assert rcs.primitive_contains(p, cyl)
    r = rcs.rounded_primitive(p)
    assert rcs.primitive_contains(r, cyl) and r["radius"] >= p["radius"]


def test_slab_boxes_bound_the_mesh_and_follow_an_arch():
    tri = _arch_tris()
    v = tri.reshape(-1, 3)
    single = rcs.fit_primitive(v)
    g1 = rcs.down_gap_mm(tri, single, "-z")
    assert g1 == pytest.approx(40.0, abs=1.0)                    # one box fills the 40 mm arch
    comp = rcs.fit_slab_boxes(tri, 0, 8)
    assert len(comp["parts"]) == 8 and rcs.primitive_contains(comp, v)
    # surface points between the vertices are inside too (exact slab clipping)
    w = np.random.default_rng(2).dirichlet([1, 1, 1], size=(len(tri), 20))
    inner = np.einsum("tsk,tkj->tsj", w, tri).reshape(-1, 3)
    assert rcs.primitive_contains(comp, inner, tol=1e-9)
    assert rcs.down_gap_mm(tri, comp, "-z") < 0.5 * g1
    assert rcs.down_gap_mm(tri, rcs.fit_slab_boxes(tri, 0, 16), "-z") < 0.25 * g1
    # the selection takes the fewest boxes within the gap limit
    best = rcs.choose_primitive(tri, downs=("-z",), max_gap_mm=15.0)
    assert best["type"] == "compound" and best["axis"] == "x" and best["down_gap_mm"] <= 15.0
    assert rcs.primitive_contains(rcs.rounded_primitive(best), v)
    assert rcs.choose_primitive(tri, downs=("-z",), max_gap_mm=50.0)["type"] == "box"
    # a gap that faces sideways is not a ground clearance problem
    assert rcs.choose_primitive(tri, downs=("+y",), max_gap_mm=15.0)["type"] == "box"


def test_down_axes_from_the_urdf():
    urdf = """<robot name="r">
      <link name="base"/><link name="m"><collision><geometry><mesh filename="a.stl"/></geometry></collision></link>
      <link name="l"><collision><origin rpy="-1.5707963 0 1.5707963"/><geometry><mesh filename="b.stl"/></geometry>
      </collision></link>
      <joint name="j1" type="fixed"><parent link="base"/><child link="m"/><origin rpy="0 0 3.14159265"/></joint>
      <joint name="j2" type="fixed"><parent link="base"/><child link="l"/></joint>
      <ros2_control name="x"><joint name="j1"><command_interface name="velocity"/></joint></ros2_control>
    </robot>"""
    meshes = rcs.collision_meshes(urdf)
    assert meshes[("a.stl", (1.0, 1.0, 1.0))]["downs"] == {"-z"}      # yaw only: down stays -z
    assert meshes[("b.stl", (1.0, 1.0, 1.0))]["downs"] == {"+y"}      # the Seyond LiDAR mounting


def test_launch_substitutes_compounds():
    m = _launch_module()
    spec = {"schema_version": 1, "meshes": [{
        "uri": "package://p/beam.stl", "scale": [1, 1, 1], "triangles": 3004,
        "primitive": {"type": "compound", "axis": "x", "parts": [
            {"type": "box", "xyz": [-0.1, 0.0, -0.05], "rpy": [0, 0, 0], "size": [0.1, 0.01, 0.02]},
            {"type": "box", "xyz": [0.1, 0.0, -0.03], "rpy": [0, 0, 0], "size": [0.1, 0.01, 0.06]}]}}]}
    urdf = ('<robot name="r"><link name="beam"><visual><geometry><mesh filename="package://p/beam.stl"/></geometry>'
            '</visual><collision name="c"><origin xyz="0 0.2 0" rpy="0 0 1.5707963267948966"/><geometry>'
            '<mesh filename="package://p/beam.stl"/></geometry></collision></link></robot>')
    out, rep = m.simplify_collisions(urdf, spec, resolve=None)
    import xml.etree.ElementTree as ET

    cols = ET.fromstring(out).find("link").findall("collision")
    assert [c.get("name") for c in cols] == ["c_0", "c_1"]
    xyz0 = [float(v) for v in cols[0].find("origin").get("xyz").split()]
    assert xyz0 == pytest.approx([0.0, 0.2 - 0.1, -0.05], abs=1e-8)    # yaw 90 deg: mesh x -> link y
    assert cols[1].find("geometry/box").get("size") == "0.1 0.01 0.06"
    assert rep["primitives"] == {"box": 2} and rep["replaced"][0]["parts"] == 2
    assert ET.fromstring(out).find("link/visual/geometry/mesh") is not None
