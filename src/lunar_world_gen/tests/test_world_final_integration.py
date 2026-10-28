"""Regression tests of the final integration round (docs/impl/final_build.md).

* FID-08 remainder: the ring collision grid (25 cm nodes) takes each node's maximum of the rock hulls'
  upper surface over the square node +- w (``mesh.collision.ring_rock_dilation`` node spacings, default
  1), so the piecewise linear collision surface through the nodes covers every ring rock's whole
  footprint; ``w = 0`` restores the old point samples (which miss most of a 30-50 cm rock).
* fix_rocks 9.2: the contact stage keeps exactly the filleted rows on H0 (``fillet_mask``), not every
  row with burial > 0 (perched rocks carry a compliance burial since the rock fixes).
* U8: the patched-plugin build script leaves nothing in the caller's working directory (CMake runs
  from the build folder, where gz-physics creates its unversioned plugin symlinks).

The build-level wiring (ring sampled with ``avoid=core``, the stage results) is tested in
tests/test_world_build.py on the small layout.

Run from src/lunar_world_gen:  python -m pytest tests/test_world_final_integration.py -q
"""
from __future__ import annotations

import os
import subprocess
import tarfile
from pathlib import Path

import numpy as np
import pytest

from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.rocks import RockLibrary, write_synthetic_library
from lunar_world_gen.rocks.catalog import empty_catalog
from lunar_world_gen.rocks.collision_field import (RING_ROCK_DILATION, rasterize_tops, ring_collision_grid,
                                                   ring_rock_dilation, rock_top_grid, transformed_hulls)
from lunar_world_gen.rocks.placement import quat_from_axis_angle


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    path = tmp_path_factory.mktemp("rocklib_final")
    return RockLibrary(write_synthetic_library(path, counts={"pebble": 2, "medium": 3, "large": 5}, seed=11))


def _ring_rocks(lib, extent: float, seed: int, d_range=(0.30, 0.50), pitch: float = 2.5) -> np.ndarray:
    """Large-class rocks of D in d_range on a jittered lattice (pitch 2.5 m: no two footprints come
    within 1.4 m) with random yaws / tilts, resting slightly sunk on z = 0."""
    r = np.random.default_rng(seed)
    large = [p for p in lib.prototypes if p.cls == "large"]
    centres = np.arange(1.0, extent - 0.9, pitch)
    cx, cy = np.meshgrid(centres, centres)
    n = cx.size
    rows = empty_catalog(n)
    rows["id"] = 10_000_000 + np.arange(n)
    rows["zone"] = 1
    for k in range(n):
        p = large[k % len(large)]
        D = r.uniform(*d_range)
        yaw = r.uniform(0, 2 * np.pi)
        axis = np.array([np.cos(r.uniform(0, 2 * np.pi)), np.sin(r.uniform(0, 2 * np.pi)), 0.0])
        axis /= np.linalg.norm(axis)
        q = quat_from_axis_angle(np.array([[0.0, 0.0, 1.0]]), np.array([yaw]))[0]
        qt = quat_from_axis_angle(axis[None], np.array([np.radians(r.uniform(0, 20))]))[0]
        # q = qt * q (tilt after yaw)
        w1, x1, y1, z1 = qt
        w2, x2, y2, z2 = q
        qq = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                       w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
        qq /= np.linalg.norm(qq)
        rows[k]["proto"], rows[k]["lod"], rows[k]["D"] = p.index, 2, D
        rows[k]["scale"] = D / p.a
        rows[k]["qw"], rows[k]["qx"], rows[k]["qy"], rows[k]["qz"] = qq
        rows[k]["x"] = cx.ravel()[k] + r.uniform(-0.3, 0.3)
        rows[k]["y"] = cy.ravel()[k] + r.uniform(-0.3, 0.3)
    tris, owner = transformed_hulls(rows, lib)
    zmin = np.full(n, np.inf)
    np.minimum.at(zmin, owner, tris[:, :, 2].min(axis=1))
    rows["z"] = -zmin + (-0.25 * 0.3) * (rows["D"] * 0.5)            # sunk a little into the ground
    rows["burial"] = 0.05
    tris, owner = transformed_hulls(rows, lib)
    zmax = np.full(n, -np.inf)
    np.maximum.at(zmax, owner, tris[:, :, 2].max(axis=1))
    rows["exposed_h"] = zmax
    return rows


# ------------------------------------------------------------------------------------- the raster kernel
def test_square_max_raster_is_exact_against_dense_sampling():
    """Each node's value is the max of the triangles' heights over the square node +- hw: never below a
    dense sample of the square, and above it by no more than the sampling resolution allows."""
    r = np.random.default_rng(3)
    tris = []
    eq = np.array([[1.0, 0.0], [-0.5, np.sqrt(3) / 2], [-0.5, -np.sqrt(3) / 2]])
    for _ in range(60):                      # well-shaped triangles (circumradius 0.1-0.4 m), slopes < ~3.5
        c = r.uniform(0.3, 2.2, 2)
        th = r.uniform(0, 2 * np.pi)
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        P = c + r.uniform(0.1, 0.4) * (eq + r.uniform(-0.1, 0.1, (3, 2))) @ R.T
        z = r.uniform(-0.1, 0.2, 3)
        tris.append(np.column_stack([P, z]))
    tris = np.array(tris)
    sp, n = 0.25, 11
    fine = 0.005
    for hw in (0.125, 0.25):
        ex = rasterize_tops(tris, 0.0, 0.0, sp, n, n, half_width=hw)
        assert np.isfinite(ex).sum() > 50
        # independent exact reference: the max of a linear function over (triangle within the square) is
        # at a triangle vertex inside the square, a square corner inside the triangle or an edge crossing
        ref = np.full((n, n), -np.inf)
        for j in range(n):
            for i in range(n):
                box = (i * sp - hw, i * sp + hw, j * sp - hw, j * sp + hw)
                ref[j, i] = max([_ref_tri_square_max(t, *box) for t in tris])
        assert np.array_equal(np.isfinite(ex), np.isfinite(ref))
        f = np.isfinite(ref)
        np.testing.assert_allclose(ex[f], ref[f], rtol=0, atol=1e-9)
        # and never below a dense 5 mm sample of the square
        xs = np.arange(-hw, 2.5 + hw + fine / 2, fine)
        dense = rasterize_tops(tris, -hw, -hw, fine, len(xs), len(xs))
        k = int(round(hw / fine))
        for j in range(n):
            for i in range(n):
                c0, r0 = int(round(i * sp / fine)), int(round(j * sp / fine))
                assert ex[j, i] >= dense[r0:r0 + 2 * k + 1, c0:c0 + 2 * k + 1].max() - 1e-12
    # zero width: identical to the point raster
    assert np.array_equal(rasterize_tops(tris, 0.0, 0.0, sp, n, n, half_width=0.0),
                          rasterize_tops(tris, 0.0, 0.0, sp, n, n))


def _ref_tri_square_max(t: np.ndarray, x0: float, x1: float, y0: float, y1: float) -> float:
    """Max z of the planar triangle t (3 x 3) over the closed box [x0, x1] x [y0, y1] by enumerating the
    candidate points of the clipped polygon (no clipping code shared with the implementation)."""
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    det = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)

    def bary(x, y):
        l1 = ((x - ax) * (cy - ay) - (cx - ax) * (y - ay)) / det
        l2 = ((bx - ax) * (y - ay) - (x - ax) * (by - ay)) / det
        return 1.0 - l1 - l2, l1, l2

    eps = 1e-12
    cands = [z for x, y, z in t if x0 - eps <= x <= x1 + eps and y0 - eps <= y <= y1 + eps]
    for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
        l0, l1, l2 = bary(x, y)
        if min(l0, l1, l2) >= -eps:
            cands.append(l0 * az + l1 * bz + l2 * cz)
    for p, q in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        for axis, val, lo, hi in ((0, x0, y0, y1), (0, x1, y0, y1), (1, y0, x0, x1), (1, y1, x0, x1)):
            d = q[axis] - p[axis]
            if abs(d) < 1e-15:
                continue
            s = (val - p[axis]) / d
            if -eps <= s <= 1 + eps:
                other = p[1 - axis] + s * (q[1 - axis] - p[1 - axis])
                if lo - eps <= other <= hi + eps:
                    cands.append(p[2] + s * (q[2] - p[2]))
    return max(cands) if cands else -np.inf


# ------------------------------------------------------------------------------------- ring collision
def test_ring_dilation_covers_every_rock_footprint(cfg, lib):
    """With the default dilation (one node spacing) the four nodes around every point of a rock's
    footprint are at least as high as the rock there (rock_top + rock_offset), so any triangulation of
    the 25 cm grid covers the whole rock; point samples (dilation 0) do not."""
    assert ring_rock_dilation(cfg) == RING_ROCK_DILATION == 1.0          # code default (no config key)
    ext, sp = 12.0, 0.25
    n = int(round(ext / sp)) + 1
    G = Grid(np.zeros((n, n), dtype=np.float32), 0.0, 0.0, sp)
    rows = _ring_rocks(lib, ext, seed=5)
    assert len(rows) == 25
    off = float(cfg.mesh.collision.rock_offset_m)
    base = float(cfg.mesh.collision.ground_offset_m)
    C1 = ring_collision_grid(cfg, G, rows, lib)
    C0 = ring_collision_grid(cfg, G, rows, lib, dilation=0.0)
    assert C1.meta["rock_dilation_nodes"] == 1.0 and C1.meta["rock_dilation_half_width_m"] == pytest.approx(sp)
    assert C0.meta["rock_dilation_nodes"] == 0.0
    # fine reference of the rocks' upper surface (1 cm point samples)
    nf = int(round(ext / 0.01)) + 1
    F = Grid(np.zeros((nf, nf), dtype=np.float32), 0.0, 0.0, 0.01)
    top = rock_top_grid(F, rows, lib)
    inside = np.isfinite(top) & (top + off > base)
    jj, ii = np.nonzero(inside)
    px, py, want = ii * 0.01, jj * 0.01, top[inside] + off
    ci = np.clip(np.floor(px / sp + 1e-9).astype(int), 0, n - 2)
    cj = np.clip(np.floor(py / sp + 1e-9).astype(int), 0, n - 2)

    def corner_min(C):
        z = np.asarray(C.z, dtype=np.float64)
        return np.minimum.reduce([z[cj, ci], z[cj, ci + 1], z[cj + 1, ci], z[cj + 1, ci + 1]])

    lo1 = corner_min(C1)
    assert np.all(lo1 >= want - 2e-6), float((want - lo1).max())     # float32 grid rounding only
    lo0 = corner_min(C0)
    assert np.mean(lo0 < want - 0.01) > 0.3                           # point samples leave most of it out
    # every rock keeps its full rise at some node (the old grid lost most of it for many rocks)
    z1 = np.asarray(C1.z, dtype=np.float64)
    z0 = np.asarray(C0.z, dtype=np.float64)
    for row in rows:
        i0, i1 = int(np.floor((row["x"] - row["D"]) / sp)), int(np.ceil((row["x"] + row["D"]) / sp))
        j0, j1 = int(np.floor((row["y"] - row["D"]) / sp)), int(np.ceil((row["y"] + row["D"]) / sp))
        rise_max = row["exposed_h"] + off - base
        assert z1[j0:j1 + 1, i0:i1 + 1].max() - base >= rise_max - 2e-6
        assert z0[j0:j1 + 1, i0:i1 + 1].max() - base <= rise_max + 2e-6
    # the dilation is bounded: nothing is raised farther than one node spacing beyond a rock's reach
    reach = (lib.a_array()[rows["proto"]] * rows["scale"])
    X, Y = G.mesh_xy()
    far = np.ones(X.shape, dtype=bool)
    for row, a in zip(rows, reach):
        far &= np.maximum(np.abs(X - row["x"]), np.abs(Y - row["y"])) > a + sp + 1e-9
    assert np.all(z1[far] == pytest.approx(base))
    assert C1.meta["n_nodes_raised"] > 2 * C0.meta["n_nodes_raised"]


def test_ring_dilation_key_zero_restores_point_samples(cfg, lib):
    ext, sp = 6.0, 0.25
    n = int(round(ext / sp)) + 1
    G = Grid(np.zeros((n, n), dtype=np.float32), 0.0, 0.0, sp)
    rows = _ring_rocks(lib, ext, seed=9)
    cfg0 = load_config(overrides=["mesh.collision.ring_rock_dilation=0"])
    assert ring_rock_dilation(cfg0) == 0.0
    a = ring_collision_grid(cfg0, G, rows, lib)
    tris, _ = transformed_hulls(rows, lib)
    top = rasterize_tops(tris, G.x0, G.y0, sp, n, n)
    off, base = float(cfg.mesh.collision.rock_offset_m), float(cfg.mesh.collision.ground_offset_m)
    ref = np.where(np.isfinite(top) & (top + off > base), top + off, base)
    assert np.array_equal(np.asarray(a.z), ref.astype(np.float32))
    half = load_config(overrides=["mesh.collision.ring_rock_dilation=0.5"])
    b = ring_collision_grid(half, G, rows, lib)
    c = ring_collision_grid(cfg, G, rows, lib)
    # monotone in the width
    assert np.all(np.asarray(b.z) >= np.asarray(a.z)) and np.all(np.asarray(c.z) >= np.asarray(b.z))


# ------------------------------------------------------------------------------------- U8: plugin build
PKG = Path(__file__).resolve().parents[1]
BUILD_SH = PKG / "tools" / "gz_physics_bullet_lunar" / "build_lunar_bullet_plugin.sh"
TARBALL = Path("/home/nargon/togo_assets/lunar/build/gz_physics_bullet_lunar/gz-physics7_7.6.0.tar.gz")
VENDOR = Path("/opt/ros/jazzy/opt/gz_physics_vendor/lib/libgz-physics7.so.7.6.0")


@pytest.mark.skipif(not (TARBALL.is_file() and VENDOR.exists() and Path("/opt/ros/jazzy/setup.bash").is_file()),
                    reason="gz-physics 7.6.0 tarball / ROS Jazzy vendor library not present")
def test_plugin_build_leaves_nothing_in_the_callers_directory(tmp_path):
    """U8: gz-physics' plugin CMakeLists create their unversioned symlinks with execute_process in the
    current directory; the script must run CMake from its build folder. Fake cmake / make / ldd (the
    fake cmake creates a symlink in its working directory exactly like gz-physics does), the real
    patch step on the four patched source files of the real tarball."""
    members = ("JointFeatures.cc", "Base.hh", "SDFFeatures.cc", "ShapeFeatures.cc")
    prefix = "gz-physics-gz-physics7_7.6.0/bullet/src/"
    src = tmp_path / "srcroot"
    with tarfile.open(TARBALL) as tf:
        sel = [m for m in tf.getmembers() if m.name.startswith(prefix) and m.name[len(prefix):] in members]
        assert len(sel) == 4
        tf.extractall(src, members=sel, filter="data")
    fake_tgz = tmp_path / "fake.tar.gz"
    with tarfile.open(fake_tgz, "w:gz") as tf:
        tf.add(src / "gz-physics-gz-physics7_7.6.0", arcname="gz-physics-gz-physics7_7.6.0")
    fb = tmp_path / "fakebin"
    fb.mkdir()
    (fb / "cmake").write_text('#!/bin/sh\npwd > "$FAKE_LOG/cmake_cwd"\n'
                              'ln -sf libgz-physics7-bullet-plugin.so libgz-physics-bullet-plugin.so\n')
    (fb / "make").write_text('#!/bin/sh\nwhile [ $# -gt 0 ]; do case "$1" in -C) B="$2"; shift 2;; *) shift;; esac; '
                             'done\nmkdir -p "$B/lib" && printf fake > "$B/lib/libgz-physics7-bullet-plugin.so.7.6.0"\n')
    (fb / "ldd").write_text('#!/bin/sh\necho "\tlibgz-physics7.so.7 => /fake/libgz-physics7.so.7"\n')
    for f in fb.iterdir():
        f.chmod(0o755)
    cwd = tmp_path / "caller"
    cwd.mkdir()
    work, out = tmp_path / "work", tmp_path / "out" / "plugin.so"
    log = tmp_path / "log"
    log.mkdir()
    env = dict(os.environ, PATH=f"{fb}:{os.environ.get('PATH', '/usr/bin:/bin')}", FAKE_LOG=str(log))
    r = subprocess.run(["bash", str(BUILD_SH), "--out", str(out), "--work", str(work), "--tarball", str(fake_tgz),
                        "--jobs", "1"], cwd=cwd, env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout[-2000:] + r.stderr[-2000:]
    assert list(cwd.iterdir()) == []                                     # nothing left in the caller's dir
    assert (log / "cmake_cwd").read_text().strip() == str(work / "build")
    assert (work / "build" / "libgz-physics-bullet-plugin.so").is_symlink()
    assert out.read_text() == "fake" and Path(f"{out}.json").is_file()


# ------------------------------------------------------------------------------------- contact: fillets
def test_contact_keeps_only_filleted_rows_on_h0(cfg):
    """fix_rocks 9.2: perched rocks carry a compliance burial (2-6 %) but no fillet, so they are
    re-seated on the rendered ground; only the rows apply_fillets fillets stay on H0."""
    from lunar_world_gen.rocks.fillets import fillet_mask
    from lunar_world_gen.rocks.recontact import fillet_rows

    c = empty_catalog(6)
    c["zone"] = [0, 0, 0, 0, 0, 1]
    c["D"] = [0.2, 0.2, 0.2, 0.05, 0.2, 0.5]
    c["burial"] = [0.25, 0.04, 0.06, 0.25, 0.0, 0.25]
    c["exposed_h"] = 0.05
    exp = [True, False, False, False, False, False]
    assert fillet_rows(cfg, c).tolist() == exp
    assert np.array_equal(fillet_rows(cfg, c), fillet_mask(cfg, c))
    off = load_config(overrides=["terrain.fillets.enabled=false"])
    assert not fillet_rows(off, c).any()
