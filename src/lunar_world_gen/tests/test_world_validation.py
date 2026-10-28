"""Tests of the validation stage (lunar_world_gen/validate, tools/bench_world.py, tools/lidar_capture.py,
tools/run_robot_tests.py) on synthetic data.

Run (WSL, project venv, from src/lunar_world_gen):
    ~/.venvs/lunar_world/bin/python -m pytest tests/test_world_validation.py -q
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from lunar_world_gen.config import load_config  # noqa: E402
from lunar_world_gen.grid import Grid  # noqa: E402
from lunar_world_gen.layout import Layout  # noqa: E402
from lunar_world_gen.validate import lidar_checks as lc  # noqa: E402
from lunar_world_gen.validate import report as rep  # noqa: E402
from lunar_world_gen.validate import static_checks as sc  # noqa: E402
from lunar_world_gen.validate import stats as st  # noqa: E402


# ==============================================================================================
# helpers

def fbm(n: int, dx: float, H: float, S0: float, seed: int = 0) -> np.ndarray:
    """Periodic isotropic fBm with S(f) = S0 f^-(2+2H) (same synthesis as science/fbm_check.py)."""
    rng = np.random.default_rng(seed)
    fx = np.fft.fftfreq(n, d=dx)
    f = np.hypot(fx[:, None], fx[None, :])
    f[0, 0] = np.inf
    amp = np.sqrt(S0 * f ** -(2 + 2 * H)) / (n * dx)
    Z = amp * np.exp(2j * np.pi * rng.random((n, n))) * n * n
    return np.real(np.fft.ifft2(Z)) * np.sqrt(2.0)


def grid_tin(rect, step: float, hfun, offset: float = 0.0):
    """Regular-grid TIN over a rectangle (CCW from +z), analytic normals."""
    nx = int(round((rect.x1 - rect.x0) / step)) + 1
    ny = int(round((rect.y1 - rect.y0) / step)) + 1
    xs = rect.x0 + np.arange(nx) * step
    ys = rect.y0 + np.arange(ny) * step
    xs[-1], ys[-1] = rect.x1, rect.y1
    X, Y = np.meshgrid(xs, ys)
    Z = hfun(X, Y) + offset
    V = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    idx = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    T = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    eps = 1e-4
    gx = (hfun(X + eps, Y) - hfun(X - eps, Y)) / (2 * eps)
    gy = (hfun(X, Y + eps) - hfun(X, Y - eps)) / (2 * eps)
    N = np.stack([-gx.ravel(), -gy.ravel(), np.ones(nx * ny)], 1)
    N /= np.linalg.norm(N, axis=1, keepdims=True)
    return V, T, N


def hfun(x, y):
    return 0.3 * np.sin(0.5 * x) + 0.2 * np.cos(0.4 * y)


def mini_cfg(tmp: Path):
    sets = [f"paths.build_dir={tmp / 'build'}", f"paths.world_dir={tmp / 'world'}",
            f"paths.rock_library={tmp / 'lib'}", "zones.core.size_m=20.0", "zones.core.tile_size_m=10.0",
            "zones.core.spacing_m=0.02", "zones.ring.size_m=40.0", "zones.far.size_m=80.0",
            "zones.horizon.size_m=160.0", "mesh.collision.spacing_m=0.04"]
    return load_config(None, "full", sets)


def quantile_sizes(Nfun, D_lo: float, D_hi: float, n: int) -> np.ndarray:
    """Deterministic sample whose cumulative counts follow N(>=D) exactly (stratified quantiles)."""
    Dg = np.geomspace(D_lo, D_hi, 20001)
    Ng = Nfun(Dg) - Nfun(D_hi)
    Ng = Ng / Ng[0]
    u = (np.arange(n) + 0.5) / n
    return np.interp(u, Ng[::-1], Dg[::-1])


def build_mini_world(tmp: Path, cfg, *, tjunction: bool = False, gap: float = 0.0, step: float = 1.0,
                     per_tile_collision: bool = True):
    """Synthetic build + world folder exercising every static check (per-tile core collision
    ``tile_ii_jj_collision.glb`` as DESIGN 7, or the older single ``core_collision.glb``)."""
    lay = Layout(cfg)
    meshes = lay.meshes_dir
    rough = cfg.mesh.collision.ground_offset_m
    pieces = {}
    for t in lay.tiles():
        pieces[lay.tile_mesh_name(t, "ground")] = lay.tile_rect(t)
    for zone in ("ring", "far", "horizon"):
        for side, r in lay.strip_rects(zone).items():
            pieces[lay.strip_mesh_name(zone, side, "ground")] = r
    for name, r in pieces.items():
        V, T, N = grid_tin(r, step, hfun)
        if gap and name == "tile_01_00_ground.glb":
            V[:, 0] += gap
        if tjunction and name == "tile_00_00_ground.glb":
            # split the boundary triangle on the east edge at the edge midpoint: the extra vertex is a
            # (collinear) T-junction for the neighbouring tile
            on = np.flatnonzero(np.abs(V[:, 0] - r.x1) < 1e-9)
            k0, k1 = on[0], on[1]          # two consecutive vertices on the east edge
            ti = next(i for i, tri in enumerate(T) if k0 in tri and k1 in tri)
            tri = list(T[ti])
            V = np.vstack([V, 0.5 * (V[k0] + V[k1])])
            N = np.vstack([N, N[k0]])
            m = len(V) - 1
            t1, t2 = list(tri), list(tri)
            t1[tri.index(k1)] = m           # (k0, m, other) and (m, k1, other) keep the CCW order
            t2[tri.index(k0)] = m
            T = np.vstack([np.delete(T, ti, axis=0), [t1, t2]])
        sc.write_glb(meshes / name, V, T, N, V[:, :2] / 2.0, roughness=float(cfg.materials.regolith.roughness))
    if per_tile_collision:
        cols = {lay.tile_mesh_name(t, "collision"): lay.tile_rect(t) for t in lay.tiles()}
    else:
        cols = {"core_collision.glb": lay.zone_rect("core")}
    for zone in ("ring", "far"):
        for side, r in lay.strip_rects(zone).items():
            cols[lay.strip_mesh_name(zone, side, "collision")] = r
    for name, r in cols.items():
        V, T, N = grid_tin(r, step, hfun, offset=rough)
        sc.write_glb(meshes / name, V, T, None, None, material=False)
    # SDF with relative URIs to every mesh
    world = ET.Element("sdf", {"version": "1.11"})
    w = ET.SubElement(world, "world", {"name": "mini"})
    for fn in ("gz-sim-physics-system", "gz-sim-user-commands-system", "gz-sim-scene-broadcaster-system"):
        ET.SubElement(w, "plugin", {"name": fn, "filename": fn})
    m = ET.SubElement(w, "model", {"name": "apollo16_terrain"})
    ET.SubElement(m, "static").text = "true"
    link = ET.SubElement(m, "link", {"name": "link"})
    for k, name in enumerate(sorted(pieces) + sorted(cols)):
        tag = "collision" if "collision" in name else "visual"
        e = ET.SubElement(link, tag, {"name": f"{tag}_{k}"})
        ET.SubElement(ET.SubElement(ET.SubElement(e, "geometry"), "mesh"), "uri").text = f"meshes/{name}"
    ET.ElementTree(world).write(lay.world_dir / "apollo16_core.sdf", encoding="unicode", xml_declaration=True)
    # core height grid (fBm, 2 cm) and a collision grid
    core = lay.zone_rect("core")
    n = int(round(core.width / cfg.zones.core.spacing_m)) + 1
    z = fbm(1024, cfg.zones.core.spacing_m, 0.8, 2.54e-5, seed=3)[:n, :n] if n <= 1024 else None
    if z is None:
        zz = fbm(2048, cfg.zones.core.spacing_m, 0.8, 2.54e-5, seed=3)
        z = zz[:n, :n]
    Grid(z.astype(np.float32), core.x0, core.y0, cfg.zones.core.spacing_m).save(lay.stage_dir("terrain") / "core_H")
    Grid(np.zeros((11, 11), np.float32), core.x0, core.y0, 2.0).save(lay.stage_dir("terrain") / "core_B_coarse")
    # craters (stratified sizes, uniform centres over core + 20 m margin)
    cr = cfg.terrain.craters
    area_m = (core.width + 40.0) ** 2
    Nfun = lambda D: st.crater_expected_N(D, cr.sfd_C_per_m2, cr.sfd_b, 0.2, cr.D_max_m, tuple(cr.fade_D_m))  # noqa: E731
    ncr = int(round(float(Nfun(0.2)) * area_m))
    D = quantile_sizes(Nfun, 0.2, 16.0, ncr)
    g = np.random.default_rng(1)
    # exact-count placement: a deterministic share inside the core, the rest in the margin
    inside = np.zeros(ncr, bool)
    step_in = int(round(area_m / (core.width * core.height)))
    inside[step_in // 2::step_in] = True     # middle of each size stratum -> counts match the model
    xs = np.where(inside, g.uniform(core.x0, core.x1, ncr), g.uniform(core.x1 + 1, core.x1 + 19, ncr))
    ys = g.uniform(core.y0, core.y1, ncr)
    rho = np.exp(-g.uniform(0, 3, ncr))
    np.savez(lay.stage_dir("terrain") / "craters_all.npz", x=xs, y=ys, D=D, rho=rho, d=rho * 0.13 * D,
             h=rho * 0.03 * D, id=np.arange(ncr))
    # rocks (stratified sizes, core only)
    sfd = cfg.rocks.sfd
    Nr = lambda Dd: st.broken_power_law_N(Dd, sfd.A_ge_1m_per_m2, sfd.segments)  # noqa: E731
    area = core.width * core.height
    nrock = int(round(float(Nr(0.02) - Nr(3.0)) * area))
    Drk = quantile_sizes(Nr, 0.02, 3.0, nrock)
    rx = g.uniform(core.x0, core.x1, nrock)
    ry = g.uniform(core.y0, core.y1, nrock)
    T = lay.tile_size
    cat = {"id": np.arange(nrock), "zone": np.zeros(nrock, np.int64),
           "tile_i": np.floor((rx - core.x0) / T).astype(np.int64), "tile_j": np.floor((ry - core.y0) / T).astype(np.int64),
           "proto": np.zeros(nrock, np.int64), "cls": np.zeros(nrock, np.int64), "lod": np.zeros(nrock, np.int64),
           "x": rx, "y": ry, "z": hfun(rx, ry) + 0.2 * Drk, "qw": np.ones(nrock), "qx": np.zeros(nrock),
           "qy": np.zeros(nrock), "qz": np.zeros(nrock), "scale": Drk, "D": Drk, "exposed_h": 0.5 * Drk,
           "burial": np.full(nrock, 0.2), "collide": Drk >= cfg.rocks.collision_min_D_m}
    np.savez(lay.world_dir / "rocks_catalog.npz", **cat)
    (lay.world_dir / "world_info.yaml").write_text(
        "world: mini\nsites:\n  flat: {x: 1.0, y: 2.0, z: 0.5, yaw: 0.0, description: flat}\n"
        "  slope: {x: -3.0, y: 2.0, z: 0.6, yaw: 1.0}\n  rock: {x: 4.0, y: -5.0, z: 0.5, yaw: 0.0}\n"
        "  turn: {x: 6.0, y: 6.0, z: 0.5, yaw: 0.0}\n")
    return lay


# ==============================================================================================
# stats

def test_deviogram_matches_reference_estimator_and_hurst():
    dx = 0.01
    z = fbm(1024, dx, 0.8, 2.54e-5, seed=1)
    r = st.deviogram(z, (0.01, 0.03, 0.1, 0.3, 1.0), spacing=dx, detrend="none")

    def ref(L):  # science/fbm_check.py nu()
        k = int(round(L / dx))
        d = np.concatenate([(z[:, k:] - z[:, :-k]).ravel(), (z[k:, :] - z[:-k, :]).ravel()])
        return float(np.sqrt(np.mean(d * d)))

    for L, v in zip(r.lags_m, r.nu_m):
        assert v == pytest.approx(ref(L), rel=1e-12)
    H, nu1 = st.fit_hurst(r.lag_used_m[:3], r.nu_m[:3])
    assert H == pytest.approx(0.8, abs=0.05)
    assert r.rms_slope_deg[0] == pytest.approx(math.degrees(math.atan(r.nu_m[0] / 0.01)))


def test_deviogram_plane_detrend_removes_tilt_and_handles_nan_and_stride():
    dx = 0.02
    z = fbm(512, dx, 0.8, 2.54e-5, seed=2)
    yy, xx = np.mgrid[0:512, 0:512] * dx
    tilted = z + 0.15 * xx - 0.07 * yy + 3.0
    a = st.deviogram(z, (0.02, 0.2, 2.0), spacing=dx)
    b = st.deviogram(tilted, (0.02, 0.2, 2.0), spacing=dx)
    np.testing.assert_allclose(a.nu_m, b.nu_m, rtol=1e-9)
    raw = st.deviogram(tilted, (2.0,), spacing=dx, detrend="none")
    assert raw.nu_m[0] > 2 * b.nu_m[2]          # without detrending the tilt dominates
    zn = z.copy()
    zn[100:110, 200:230] = np.nan
    c = st.deviogram(zn, (0.02, 0.2), spacing=dx)
    assert np.all(np.isfinite(c.nu_m)) and c.n_pairs[0] < a.n_pairs[0]
    s = st.deviogram(z, (0.02, 0.2), spacing=dx, stride=4)
    np.testing.assert_allclose(s.nu_m, a.nu_m[:2], rtol=0.05)
    w = st.deviogram(z, (0.02, 0.2), spacing=dx, window_m=2.56)
    assert w.n_windows == 16
    rows = st.compare_deviogram(a, 0.069, 0.8, 0.25, check_lags=(0.02,))
    assert [r["checked"] for r in rows] == [True, False, False] and "pass" in rows[0]


def test_radial_psd_recovers_spectrum():
    dx = 0.01
    z = fbm(1024, dx, 0.8, 2.54e-5, seed=4)
    p = st.radial_psd(z, dx)
    f = st.fit_power_law(p["f"], p["S"], 0.5, 20.0)
    assert f["beta"] == pytest.approx(-3.6, abs=0.12)
    assert f["S0"] == pytest.approx(2.54e-5, rel=0.25)
    assert f["H"] == pytest.approx(0.8, abs=0.06)


def test_poisson_intervals():
    lo, hi = st.poisson_ci([0, 10], sigma=1.0)
    assert lo[0] == 0.0 and hi[0] == pytest.approx(1.8410, abs=1e-3)
    assert lo[1] == pytest.approx(6.891, abs=2e-3) and hi[1] == pytest.approx(14.267, abs=2e-3)
    lo3, hi3 = st.poisson_range([10000.0], sigma=3.0)
    assert lo3[0] == pytest.approx(10000 - 300, abs=8) and hi3[0] == pytest.approx(10000 + 300, abs=8)
    g = np.random.default_rng(0)
    n = g.poisson(50.0, 4000)
    lo, hi = st.poisson_range(50.0, 1.0)
    cover = np.mean((n >= lo) & (n <= hi))
    assert 0.66 < cover < 0.80   # >= 68.27 % by construction (discrete), not far above


def test_sfd_models_and_rows():
    bp = st.broken_power_law_N([0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0], 3.78e-4, [[0.005, 1.0, 2.8], [1.0, 50.0, 3.5]])
    np.testing.assert_allclose(bp * 100, [2160, 166, 23.9, 3.43, 0.263, 0.0378, 0.0033], rtol=0.02)
    lo = st.broken_power_law_N(1.0 - 1e-9, 3.78e-4, [[0.005, 1.0, 2.8], [1.0, 50.0, 3.5]])
    assert float(lo) == pytest.approx(3.78e-4, rel=1e-6)
    no_fade = st.crater_expected_N([0.5, 2.0], 0.05, 2.0, 0.2, 16.0, None)
    np.testing.assert_allclose(no_fade, 0.05 * (np.array([0.5, 2.0]) ** -2 - 16.0 ** -2), rtol=1e-6)
    with_fade = st.crater_expected_N([0.5, 10.0], 0.05, 2.0, 0.2, 16.0, (8.0, 16.0))
    assert with_fade[0] < no_fade[0] and with_fade[1] > 0
    Nfun = lambda D: st.power_law_N(D, 2.0, 2.0)  # noqa: E731
    sizes = quantile_sizes(Nfun, 0.1, 10.0, 20000)
    area = 20000 / float(Nfun(0.1) - Nfun(10.0))
    rows = st.sfd_rows(sizes, area, [0.2, 0.5, 1.0], lambda D: Nfun(D) - Nfun(10.0), sigma=3)
    assert all(r["pass"] for r in rows)
    rows = st.sfd_rows(np.concatenate([sizes, sizes[:4000]]), area, [0.2], lambda D: Nfun(D) - Nfun(10.0), sigma=3)
    assert not rows[0]["pass"]
    rows = st.sfd_rows(sizes, area, [0.2], lambda D: 1.1 * (Nfun(D) - Nfun(10.0)), frac_tol=0.2)
    assert rows[0]["pass"] and rows[0]["ratio"] == pytest.approx(1 / 1.1, rel=0.01)
    b = st.fit_cumulative_slope(sizes, 0.1, 10.0)
    assert b["b"] == pytest.approx(2.0, abs=0.05)


def test_slope_distribution_plane():
    dx = 0.05
    yy, xx = np.mgrid[0:200, 0:200] * dx
    z = math.tan(math.radians(10)) * xx
    d = st.slope_distribution(z, 0.5, dx, detrend="none")
    assert d["median_deg"] == pytest.approx(10.0, abs=0.05)
    d = st.slope_distribution(z, 0.5, dx)
    assert d["median_deg"] < 0.05


# ==============================================================================================
# GLB / TIN / stitching

def test_glb_roundtrip_and_trimesh(tmp_path):
    from lunar_world_gen.layout import Rect
    V, T, N = grid_tin(Rect(0, 4, 0, 3), 0.5, hfun)
    p = sc.write_glb(tmp_path / "a.glb", V, T, N, V[:, :2])
    g = sc.read_glb(p)
    assert g.n_vertices == len(V) and g.n_triangles == len(T)
    np.testing.assert_allclose(g.positions, V.astype(np.float32), atol=0)
    assert g.info["identity_transforms"] and g.info["materials"][0]["metallicFactor"] == 0
    assert g.info["primitives"][0]["index_type"] == "uint32"
    hdr = sc.read_glb(p, header_only=True)
    assert hdr.info["bounds_accessor"][0][0] == pytest.approx(0.0)
    tm = sc.trimesh_load_check(p)
    assert tm["ok"] is True and tm["n_faces"] == len(T)
    p2 = sc.write_glb(tmp_path / "b.glb", V, T, N, None, node_translation=(1.0, 2.0, 3.0))
    g2 = sc.read_glb(p2)
    assert not g2.info["identity_transforms"]
    np.testing.assert_allclose(g2.positions, V.astype(np.float32).astype(float) + [1, 2, 3], atol=1e-6)


def test_tin_locator_interpolates_planes():
    from scipy.spatial import Delaunay
    g = np.random.default_rng(5)
    P = np.vstack([g.uniform(0, 10, (400, 2)), [[0, 0], [10, 0], [10, 10], [0, 10]]])
    tri = Delaunay(P).simplices
    z = 0.3 * P[:, 0] - 0.2 * P[:, 1] + 1.0
    loc = sc.TinLocator(P, tri)
    q = g.uniform(0, 10, (5000, 2))
    out = loc.interpolate(z, q[:, 0], q[:, 1])
    np.testing.assert_allclose(out, 0.3 * q[:, 0] - 0.2 * q[:, 1] + 1.0, atol=1e-9)
    assert np.isnan(loc.interpolate(z, np.array([-1.0, 11.0]), np.array([5.0, 5.0]))).all()
    big = np.vstack([P, [[0, 10], [10, 0]]])  # huge triangles + small ones: bucket index stays bounded
    loc2 = sc.TinLocator(big[:, :2], np.vstack([tri, [[0, 1, 2]]]), cell=0.01)
    assert len(loc2.entries) <= max(8 * len(loc2.tris), 1_000_000)


def test_shared_segment():
    from lunar_world_gen.layout import Rect
    a, b = Rect(0, 10, 0, 10), Rect(10, 20, 0, 10)
    s = sc.shared_segment(a, b)
    assert s.orient == "v" and s.c == 10 and (s.t0, s.t1) == (0, 10)
    assert sc.shared_segment(Rect(0, 10, 0, 10), Rect(10, 20, 10, 20)) is None   # corner contact only
    s = sc.shared_segment(Rect(-250, 250, -250, -50), Rect(-50, -40, -50, -40))
    assert s.orient == "h" and s.c == -50 and (s.t0, s.t1) == (-50, -40)


def test_stitching_ok_tjunction_and_gap(tmp_path):
    cfg = mini_cfg(tmp_path / "ok")
    lay = build_mini_world(tmp_path / "ok", cfg)
    r = sc.check_stitching(cfg, sc.locate_inputs(cfg))
    assert r["status"] == "pass", r.get("failing_pairs")
    assert r["totals"]["pairs"] > 20 and r["complete"]
    # T-junction: an extra boundary vertex on tile_00_00's east edge
    cfg2 = mini_cfg(tmp_path / "tj")
    build_mini_world(tmp_path / "tj", cfg2, tjunction=True)
    r2 = sc.check_stitching(cfg2, sc.locate_inputs(cfg2))
    assert r2["status"] == "fail"
    assert r2["totals"]["t_junctions"] == 1 and r2["totals"]["t_junctions_noncollinear"] == 0
    bad = r2["failing_pairs"][0]
    assert {bad["a"], bad["b"]} == {"tile_00_00_ground", "tile_01_00_ground"}
    # gap: tile_01_00 shifted 2 mm east
    cfg3 = mini_cfg(tmp_path / "gap")
    build_mini_world(tmp_path / "gap", cfg3, gap=0.002)
    r3 = sc.check_stitching(cfg3, sc.locate_inputs(cfg3))
    assert r3["status"] == "fail" and r3["totals"]["mismatches"] > 0 and r3["totals"]["uncovered"] > 0
    del lay


def test_mesh_checks_detect_flipped_faces_and_cracks(tmp_path):
    from lunar_world_gen.layout import Rect
    rect = Rect(0, 4, 0, 4)
    V, T, N = grid_tin(rect, 1.0, hfun)
    g = sc.read_glb(sc.write_glb(tmp_path / "ok.glb", V, T, N, None))
    rec = sc.mesh_checks(g, "core_ground", rect, 0.95)
    assert rec["up_faces_frac"] == 1.0 and rec["up_normals_frac"] == 1.0 and rec["interior_boundary_edges"] == 0
    assert rec["bounds_ok"] and rec["material_ok"]
    T2 = T.copy()
    T2[:3] = T2[:3, ::-1]
    g2 = sc.read_glb(sc.write_glb(tmp_path / "flip.glb", V, T2, -N, None))
    rec2 = sc.mesh_checks(g2, "core_ground", rect, 0.95)
    assert rec2["up_faces_frac"] < 1.0 and rec2["up_normals_frac"] == 0.0
    g3 = sc.read_glb(sc.write_glb(tmp_path / "hole.glb", V, np.delete(T, [10, 11], axis=0), N, None))
    rec3 = sc.mesh_checks(g3, "core_ground", rect, 0.95)
    assert rec3["interior_boundary_edges"] > 0
    g4 = sc.read_glb(sc.write_glb(tmp_path / "yup.glb", V[:, [0, 2, 1]] * [1, 1, -1], T, N, None))
    rec4 = sc.mesh_checks(g4, "core_ground", rect, 0.95)
    assert not rec4["bounds_ok"]


def test_dz_stats():
    g = np.random.default_rng(7)
    dz = -0.02 + g.normal(0, 0.004, 100000)
    s = sc.dz_stats(dz, -0.02)
    assert s["pass"] and abs(s["median_m"] + 0.02) < 1e-3 and s["rms_about_median_m"] == pytest.approx(0.004, rel=0.05)
    assert s["p99_abs_dev_m"] == pytest.approx(2.576 * 0.004, rel=0.05)
    assert not sc.dz_stats(dz + 0.005, -0.02)["pass"]                     # biased median
    assert not sc.dz_stats(-0.02 + g.normal(0, 0.012, 100000), -0.02)["pass"]   # too rough
    heavy = dz.copy()
    heavy[:2000] += 0.05                                                    # 2 % outliers -> p99 fails
    s = sc.dz_stats(heavy, -0.02)
    assert s["pass_median"] and not s["pass_p99"]


# ==============================================================================================
# static checks end to end on a synthetic mini world

def test_run_static_checks_end_to_end(tmp_path):
    cfg = mini_cfg(tmp_path)
    lay = build_mini_world(tmp_path, cfg)
    inp = sc.locate_inputs(cfg)
    assert inp["core_grid"].endswith("core_H") and inp["rock_catalog"].endswith("rocks_catalog.npz")
    assert inp["crater_catalogs"] and len(inp["sdf_files"]) == 1
    rep_ = sc.run_static_checks(cfg, n_dz_samples=20000, verbose=False)
    ch = rep_["checks"]
    assert ch["stitching"]["status"] == "pass"
    assert ch["glb"]["status"] == "pass", ch["glb"]["problems"]
    assert ch["sdf"]["status"] == "pass"
    assert ch["world_info"]["status"] == "pass"
    assert ch["craters"]["status"] == "pass", ch["craters"]["rows"]
    assert ch["rocks"]["status"] == "pass", (ch["rocks"]["rows_core"], ch["rocks"]["sanity"])
    vc = ch["visual_vs_collision"]
    assert vc["status"] == "pass" and vc["source"]["kind"] == "meshes"
    assert vc["source"]["collision"] == "tile_*_collision.glb" and not vc["source"]["missing_tiles"]
    assert vc["median_m"] == pytest.approx(-0.02, abs=1e-5) and vc["rms_about_median_m"] < 1e-5
    assert ch["stitching"]["n_pieces_collision"] == len(lay.tiles()) + 8
    assert ch["roughness"]["status"] in ("pass", "fail") and len(ch["roughness"]["rows"]) == 7
    assert ch["roughness"]["site_aware"] is False          # no terrain/base/core grid in this mini build
    r001 = ch["roughness"]["rows"][0]
    assert r001["checked"] and r001["ratio"] == pytest.approx(1.0, abs=0.35)
    out = Path(rep_["written_to"])
    assert out == lay.world_dir / "reports" / "static_checks.json"
    js = json.loads(out.read_text())
    assert js["summary"]["n_error"] == 0 and js["kind"] == "static_checks"


def test_rock_check_respects_configured_D_min(tmp_path):
    """The fast preset samples core rocks from 3 cm: the 2 cm row is moved up to 3 cm."""
    cfg = load_config(None, "full", [f"paths.build_dir={tmp_path / 'build'}", f"paths.world_dir={tmp_path / 'world'}",
                                     f"paths.rock_library={tmp_path / 'lib'}", "zones.core.size_m=20.0",
                                     "zones.core.tile_size_m=10.0", "zones.core.spacing_m=0.02",
                                     "zones.ring.size_m=40.0", "zones.far.size_m=80.0", "zones.horizon.size_m=160.0",
                                     "mesh.collision.spacing_m=0.04", "rocks.D_min_m.core=0.03"])
    lay = build_mini_world(tmp_path, cfg)
    p = lay.world_dir / "rocks_catalog.npz"
    with np.load(p) as d:
        cat = {k: d[k] for k in d.files}
    keep = cat["D"] >= 0.03
    np.savez(p, **{k: v[keep] for k, v in cat.items()})
    r = sc.check_rocks(cfg, sc.locate_inputs(cfg))
    assert [x["D_m"] for x in r["rows_core"]] == [0.03, 0.05, 0.10, 0.20]
    assert r["status"] == "pass", r["rows_core"]


def test_visual_vs_collision_single_core_collision_glb(tmp_path):
    """The older single core_collision.glb layout is still read (fallback of the per-tile files)."""
    cfg = mini_cfg(tmp_path)
    build_mini_world(tmp_path, cfg, per_tile_collision=False)
    inp = sc.locate_inputs(cfg)
    vc = sc.check_visual_vs_collision(cfg, inp, n_samples=5000)
    assert vc["status"] == "pass" and vc["source"]["collision"].endswith("core_collision.glb")
    st_ = sc.check_stitching(cfg, inp)
    assert st_["status"] == "pass" and st_["n_pieces_collision"] == 1 + 8


def _write_roughness_inputs(tmp: Path, cfg, scale_detail: float = 1.0):
    """Core H = B + detail on the mini core (2 cm) and the DTM base B in terrain/base/core."""
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    s = cfg.zones.core.spacing_m
    n = int(round(core.width / s)) + 1
    X, Y = np.meshgrid(core.x0 + np.arange(n) * s, core.y0 + np.arange(n) * s)
    B = (0.5 * np.sin(2 * np.pi * X / 40.0) + 0.3 * np.cos(2 * np.pi * (Y + 3.0) / 35.0)).astype(np.float32)
    detail = (scale_detail * fbm(1024, s, 0.8, 2.54e-5, seed=3)[:n, :n]).astype(np.float32)
    Grid(B + detail, core.x0, core.y0, s).save(lay.stage_dir("terrain") / "core_H")
    Grid(B, core.x0, core.y0, s).save(lay.build_dir / "terrain" / "base" / "core")
    return B, detail


def test_roughness_site_aware_rule(tmp_path):
    cfg = mini_cfg(tmp_path / "a")
    B, detail = _write_roughness_inputs(tmp_path / "a", cfg)
    inp = sc.locate_inputs(cfg)
    assert inp["core_base_grid"].endswith(str(Path("terrain") / "base" / "core"))
    r = sc.check_roughness(cfg, inp, psd=False, slope_baselines=())
    assert r["site_aware"] is True
    rows = {x["L_m"]: x for x in r["rows"]}
    syn = {x["L_m"]: x for x in r["rows_synthetic"]}
    # checked lags: composite at 0.01-0.3 m and >= 10 m (vs B), synthetic at 1 and 3 m
    assert [L for L, x in rows.items() if x["checked"]] == [0.01, 0.03, 0.1, 0.3, 10.0]
    assert [L for L, x in syn.items() if x["checked"]] == [1.0, 3.0]
    # the synthetic rows are the deviogram of H0 - B, the 10 m rule compares H0 with B alone
    ref_syn = st.deviogram(Grid(detail, 0, 0, cfg.zones.core.spacing_m), st.DEFAULT_LAGS_M)
    ref_B = st.deviogram(Grid(B, 0, 0, cfg.zones.core.spacing_m), st.DEFAULT_LAGS_M)
    for L, v, vb in zip(st.DEFAULT_LAGS_M, ref_syn.nu_m, ref_B.nu_m):
        assert syn[L]["nu_m"] == pytest.approx(v, rel=1e-3)
        assert rows[L]["nu_B_m"] == pytest.approx(vb, rel=1e-9)
    r10 = rows[10.0]
    assert r10["ratio_to_B"] == pytest.approx(r10["nu_m"] / r10["nu_B_m"])
    assert r10["pass"] == (r10["ratio_to_B"] <= sc.ROUGH_DTM_RATIO_MAX)
    checked = [x for x in r["rows"] + r["rows_synthetic"] if x["checked"]]
    assert (r["status"] == "pass") == all(x["pass"] for x in checked)
    # 3x the synthetic detail: the small lags and the synthetic part leave the band -> fail
    cfg2 = mini_cfg(tmp_path / "b")
    _write_roughness_inputs(tmp_path / "b", cfg2, scale_detail=3.0)
    r2 = sc.check_roughness(cfg2, sc.locate_inputs(cfg2), psd=False, slope_baselines=())
    assert r2["status"] == "fail"
    assert not {x["L_m"]: x for x in r2["rows"]}[0.01]["pass"]
    assert not any(x["pass"] for x in r2["rows_synthetic"] if x["checked"])


def test_locate_inputs_ignores_components_and_assembles_tiles(tmp_path):
    cfg = mini_cfg(tmp_path)
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    s = cfg.zones.core.spacing_m
    n = int(round((core.width + 1.0) / s)) + 1
    # component grids covering the whole core must not be taken for H (terrain owner's layout)
    Grid(np.ones((n, n), np.float32), core.x0 - 0.5, core.y0 - 0.5, s).save(lay.build_dir / "terrain" / "base" / "core")
    Grid(np.ones((n, n), np.float32), core.x0 - 0.5, core.y0 - 0.5, s).save(lay.build_dir / "terrain" / "fbm" / "fine_unit")
    m = int(round((lay.tile_size + 1.0) / s)) + 1
    for t in lay.tiles():
        r = lay.tile_rect(t)
        X, Y = np.meshgrid(r.x0 - 0.5 + np.arange(m) * s, r.y0 - 0.5 + np.arange(m) * s)
        Grid((X + 2 * Y).astype(np.float32), r.x0 - 0.5, r.y0 - 0.5, s).save(
            lay.build_dir / "terrain" / "core_tiles" / t.name)
    inp = sc.locate_inputs(cfg)
    assert inp["core_grid"] is None and len(inp["core_tile_grids"]) == len(lay.tiles())
    g, src = sc.load_core_grid(inp, cfg)
    assert g.nx == int(round(core.width / s)) + 1 and src.endswith("tile grids")
    X, Y = g.mesh_xy()
    np.testing.assert_allclose(g.z, X + 2 * Y, atol=1e-4)    # tiles placed at the right nodes, overlaps agree


def test_contention_helper():
    import bench_world as bw
    t0 = {1: ("ruby", 100, 50), 2: ("python", 1000, 7), 3: ("bash", 5, 9)}
    t1 = {1: ("ruby", 300, 50), 2: ("python", 1100, 7), 3: ("bash", 5, 9), 4: ("new", 50, 4)}
    c = bw.contention(t0, t1, 1000, 1000 + int(2.5 * bw.CLK), 1.0, own_pgid=50)
    assert c["system_busy_cores"] == pytest.approx(2.5, abs=0.02)
    assert c["other_procs_top"][0][2] == "python" and all(p[2] != "ruby" for p in c["other_procs_top"])


def test_contention_counts_new_processes_and_series():
    import bench_world as bw
    C = bw.CLK
    t0 = {1: ("ruby", 100, 50), 2: ("python", 1000, 7)}
    t1 = {1: ("ruby", 100 + 2 * C, 50), 2: ("python", 1000 + C // 10, 7), 4: ("dtm_extract", 3 * C, 4)}
    c = bw.contention(t0, t1, 0, 6 * C, 2.0, own_pgid={50})
    top = c["other_procs_top"][0]
    assert top[2] == "dtm_extract" and top[3] is True and top[0] == pytest.approx(150.0)   # started mid-window
    assert c["own_group_cpu_pct"] == pytest.approx(100.0) and c["system_busy_cores"] == pytest.approx(3.0)
    # 2 s bins over 8 s: an "other" job uses 2 cores in [4, 6) s while the RTF drops to 0.8
    pts, procs = [], {1: ("ruby", 0, 50)}
    for k in range(17):                                    # samples every 0.5 s
        t = 0.5 * k
        pts.append((t, None, procs[1][1], dict(procs)))
        procs[1] = ("ruby", procs[1][1] + C // 2, 50)     # server: 1 core
        if 4.0 <= t < 6.0:
            old = procs.get(9, ("job", 0, 9))
            procs[9] = ("job", old[1] + C, 9)              # 2 cores
    stats, sim = [], 0.0
    for k in range(81):                                    # gz stats at 10 Hz
        t = 0.1 * k
        stats.append((t, sim, t, k, 1.0))
        sim += 0.1 * (0.8 if 4.0 <= t < 6.0 - 1e-9 else 1.0)
    cs = bw.contention_series(pts, stats, 2.0, {50})
    assert cs["n_bins"] == 4 and [b["other_cores"] for b in cs["bins"]] == pytest.approx([0, 0, 2.0, 0], abs=0.01)
    assert cs["bins"][2]["rtf"] == pytest.approx(0.8, abs=0.02) and cs["rtf_bin_min"] == pytest.approx(0.8, abs=0.02)
    assert cs["rtf_other_lt_1core"] == pytest.approx(1.0, abs=1e-6) and cs["n_bins_other_lt_1core"] == 3
    assert cs["other_top"][0][1] == "job" and cs["bins"][0]["server_cores"] == pytest.approx(1.0)


def test_build_height_sources_prefers_final_tiles(tmp_path):
    cfg = mini_cfg(tmp_path)
    lay = Layout(cfg)
    s = cfg.zones.core.spacing_m
    m = int(round((lay.tile_size + 1.0) / s)) + 1
    for t in lay.tiles():
        r = lay.tile_rect(t)
        X, Y = np.meshgrid(r.x0 - 0.5 + np.arange(m) * s, r.y0 - 0.5 + np.arange(m) * s)
        Grid((X + 2 * Y).astype(np.float32), r.x0 - 0.5, r.y0 - 0.5, s).save(lay.build_dir / "terrain" / "core_tiles" / t.name)
        Grid((X + 2 * Y + 0.01).astype(np.float32), r.x0 - 0.5, r.y0 - 0.5, s).save(lay.build_dir / "heights" / t.name)
    ring = lay.zone_rect("ring")
    rs = float(cfg.zones.ring.spacing_m)
    n = int(round(ring.width / rs)) + 1
    Grid(np.full((n, n), -3.0, np.float32), ring.x0, ring.y0, rs).save(lay.build_dir / "terrain" / "ring_height")
    sources, found = lc.build_height_sources(cfg)
    assert [n for n, _ in sources] == ["core_final_tiles", "ring_grid"] and len(sources[0][1]) == len(lay.tiles())
    h = lc.point_height_fn(sources)
    assert h(1.0, 2.0) == pytest.approx(5.01, abs=1e-4)       # final tiles (H0 + fillets), not the H0 tiles
    assert h(15.0, 0.0) == pytest.approx(-3.0) and math.isnan(h(100.0, 0.0))
    import bench_world as bw
    assert bw.ground_height_fn(None, None, None) is None      # no source requested -> no ground (as before)


def test_static_checks_skip_when_nothing_built(tmp_path):
    cfg = mini_cfg(tmp_path)
    rep_ = sc.run_static_checks(cfg, write=False, verbose=False)
    assert all(r["status"] == "skipped" for r in rep_["checks"].values())
    assert rep_["summary"]["pass"] is False and rep_["summary"]["n_skipped"] == len(sc.CHECKS)


# ==============================================================================================
# LiDAR checks on a synthetic capture (ray-cast against z = 0 and a cylinder rock)

def synthetic_capture(path: Path, drop: int = 0, sensor=(0.0, 0.0, 0.45), rock=(3.0, 0.0, 0.2, 0.1)):
    az = np.linspace(-1.0472, 1.0472, 121)
    el = np.linspace(-0.6109, 0.6109, 61)
    A, E = np.meshgrid(az, el)
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1)
    s = np.asarray(sensor, float)
    t = np.full(A.shape, np.inf)
    down = d[..., 2] < -1e-9
    t[down] = -s[2] / d[..., 2][down]
    # vertical cylinder rock: side + top
    rx, ry, D, h = rock
    R = D / 2
    ox, oy = s[0] - rx, s[1] - ry
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (ox * d[..., 0] + oy * d[..., 1])
    c = ox * ox + oy * oy - R * R
    disc = b * b - 4 * a * c
    with np.errstate(invalid="ignore", divide="ignore"):
        ts = (-b - np.sqrt(disc)) / (2 * a)
        zs = s[2] + ts * d[..., 2]
        side = (disc > 0) & (ts > 0) & (zs >= 0) & (zs <= h)
        tt = (h - s[2]) / d[..., 2]
        px, py = s[0] + tt * d[..., 0], s[1] + tt * d[..., 1]
        top = (tt > 0) & ((px - rx) ** 2 + (py - ry) ** 2 <= R * R)
    t = np.where(side & (ts < t), ts, t)
    t = np.where(top & (tt < t), tt, t)
    t[t > 150] = np.inf
    if drop:
        idx = np.argwhere(np.isfinite(t) & (t < 6))
        for j, i in idx[:: max(1, len(idx) // drop)][:drop]:
            t[j, i] = np.inf
    P = s + d * np.where(np.isfinite(t), t, np.nan)[..., None]
    meta = {"lidar": {"range_max": 150.0}, "request": [s[0], s[1], s[2], 0.0]}
    np.savez(path, points=P.astype(np.float32), points_sensor=(P - s).astype(np.float32),
             ranges=t.astype(np.float32), azimuth=az, elevation=el, pose=np.array([*s, 0, 0, 0.0]),
             request=np.array([s[0], s[1], s[2], 0.0]), ground_z=np.float64(0.0), stamp=np.float64(1.0),
             meta_json=json.dumps(meta))
    return path


def test_lidar_checks_synthetic(tmp_path):
    p = synthetic_capture(tmp_path / "capture_000.npz")
    grid = Grid(np.zeros((801, 801), np.float32), -20.0, -20.0, 0.05)
    heights = lc.GridHeights([grid], [0.0])
    rocks = {"x": np.array([3.0]), "y": np.array([0.0]), "D": np.array([0.2]), "exposed_h": np.array([0.1]),
             "z": np.array([0.05])}
    res = lc.run_lidar_checks([p], heights=heights, rocks=rocks, out=tmp_path / "lidar_checks.json")
    ch = res["checks"]
    assert ch["edge"]["pass"] and ch["edge"]["returns_per_zone"]["core"] > 0
    assert ch["ground_residuals"]["n_below"] == 0 and ch["ground_residuals"]["n_floating"] == 0
    assert ch["ground_residuals"]["n_above"] > 0          # rock returns are above the ground, inside the footprint
    assert ch["missing_returns"]["leak_candidates"] == 0
    rows = ch["rock_detection"]["rows"]
    assert len(rows) == 1 and rows[0]["p_detect"] == 1.0 and rows[0]["mean_returns"] > 3
    ideal = rows[0]["mean_ideal_returns"]
    assert 0.4 < rows[0]["mean_returns"] / ideal < 2.5
    assert res["beam_direction_max_dev_deg"] < 1e-3
    assert json.loads((tmp_path / "lidar_checks.json").read_text())["kind"] == "lidar_checks"
    # dropped returns (simulated crack leaks) are found by the ray march
    p2 = synthetic_capture(tmp_path / "leaky.npz", drop=5)
    res2 = lc.run_lidar_checks([p2], heights=heights, rocks=rocks)
    assert res2["checks"]["missing_returns"]["leak_candidates"] == 5
    assert res2["checks"]["missing_returns"]["status"] == "fail"
    # points beyond the ring from a core sensor fail the edge check
    cap = lc.load_capture(p)
    cap.points[0, 0] = [300.0, 0.0, 0.0]
    cap.ranges[0, 0] = 300.0
    assert not lc.edge_check([cap])["pass"]


# ==============================================================================================
# tools (no Gazebo needed)

def test_bench_prepare_world(tmp_path):
    import bench_world as bw
    (tmp_path / "w").mkdir()
    (tmp_path / "w" / "meshes").mkdir()
    sdf = tmp_path / "w" / "world.sdf"
    sdf.write_text("""<?xml version='1.0'?><sdf version='1.11'><world name='tw'>
      <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
      <model name='terrain'><static>true</static><link name='l'>
        <visual name='v'><geometry><mesh><uri>meshes/a.glb</uri></mesh></geometry>
          <material><pbr><metal><albedo_map>../tex/alb.png</albedo_map></metal></pbr></material></visual>
        <sensor name='old' type='gpu_lidar'><topic>/x</topic></sensor>
      </link></model></world></sdf>""")
    out = tmp_path / "copy.sdf"
    L = [{"name": "validation_lidar_0", "pose": (1, 2, 3, 0, 0, 0.5), "topic": "/validation/lidar_0",
          "preset": bw.LIDAR_PRESETS["real"]}]
    facts = bw.prepare_world(sdf, out, L, strip_lidars=True, gravity="0 0 -1.62")
    assert facts["world_name"] == "tw" and facts["uris_rewritten"] == 2 and facts["added_sensors_system"]
    assert facts["stripped_lidars"] == ["old"]
    root = ET.parse(out).getroot()
    assert root.find(".//visual/geometry/mesh/uri").text == str((tmp_path / "w" / "meshes" / "a.glb").resolve())
    assert any("sensors-system" in p.get("filename") for p in root.iter("plugin"))
    sens = [s for s in root.iter("sensor")]
    assert len(sens) == 1 and sens[0].find("lidar/scan/horizontal/samples").text == "209"
    assert sens[0].find("topic").text == "/validation/lidar_0"
    assert root.find("world/gravity").text == "0 0 -1.62"
    poses = bw.resolve_poses(["1,2,3"], ["4,5,0.5,1.0"], lambda x, y: 0.25, 0.449)
    assert poses[0]["pose"][2] == 3 and poses[1]["pose"][2] == pytest.approx(0.75) and poses[1]["pose"][5] == 1.0
    fake = {"status": "ok", "thresholds": {"load_time_max_s": 30, "rtf_min": 0.95, "server_rss_max_gb": 5},
            "load": {"first_lidar_s": 12.0}, "window": {"rtf_mean": 0.99, "wall_s": 30.0},
            "log": {"n_err": 0}, "renderer": {"d3d12": True, "software": False}, "mem": {"hwm_mb": 3000.0}}
    j = bw.judge(fake)
    assert all(v["pass"] for v in j.values())
    fake["window"]["rtf_mean"] = 0.9
    assert not bw.judge(fake)["rtf"]["pass"]


def test_lidar_capture_helpers():
    import lidar_capture as lcap
    poses = lcap.auto_poses(6, 0.449)
    assert len(poses) == 6 and poses[0].startswith("0.000,0.000,0.449")
    az = np.linspace(-1, 1, 11)
    el = np.linspace(-0.5, 0.5, 5)
    A, E = np.meshgrid(az[::-1], el)          # columns reversed w.r.t. the nominal order
    x, y, z = np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)
    az2, el2, info = lcap.verify_angles(x, y, z, az, el)
    assert info["azimuth_reversed"] and np.allclose(az2, az[::-1]) and np.allclose(el2, el)


def test_robot_test_helpers(tmp_path):
    import run_robot_tests as rr
    line = ("[ERROR] [spawner-7]: process has died [pid 4242, exit code 1, cmd '/opt/ros/jazzy/lib/controller_manager/"
            "spawner platform_velocity_controller --controller-manager-timeout 300'].")
    m = rr.SPAWNER_DIED.search(line)
    assert m and m.group(1) == "spawner-7" and m.group(3) == "1"
    lf = tmp_path / "x.launch.py"
    lf.write_text("DeclareLaunchArgument('site', default_value='flat')\nDeclareLaunchArgument(\"world\")\n")
    assert rr.declared_launch_args(lf) == ["site", "world"]
    cmd, facts = rr.build_launch_cmd("stock", Path("/w.sdf"), "flat", {"x": 1, "y": 2, "z": 0.3, "yaw": 0.5}, [])
    assert "robot_z:=0.3000" in cmd and facts["yaw_not_applied"] == 0.5
    gt = tmp_path / "gt.csv"
    rows = ["t_sim,x,y,z,qx,qy,qz,qw,roll,pitch,yaw,wall"]
    for k in range(600):
        t = k * 0.02
        x = 0.3 * t if not (5.0 < t < 7.0) else 0.3 * 5.0
        if t >= 7.0:
            x = 0.3 * (t - 2.0)
        rows.append(f"{t},{x},0,0,0,0,0,1,0,0,0,{t}")
    gt.write_text("\n".join(rows))
    sm = rr.stall_metric(gt, 0.0, 12.0)
    assert sm["available"] and sm["stalled"]
    sm2 = rr.stall_metric(gt, 0.0, 4.9)
    assert not sm2["stalled"]
    summary = {"status": {"status": "ok"}, "run": {"rtf_from_clocks": 0.97, "start_xyz": [0, 0, 0], "end_xyz": [3, 0, 0]},
               "segments": [{"name": "fwd", "dist_ratio_gt_over_cmd": 0.96, "speed_tracking_ratio": 1.015,
                             "ballistic_frac": 0.01},
                            {"name": "turn", "yawrate_tracking_ratio": 0.9}]}

    def ev_flat():
        return rr.evaluate_drive("flat", {"summary": summary, "rc": 0, "out": str(tmp_path)}, 1.62)

    ev = ev_flat()
    assert ev["pass"] and ev["rtf_pass"] and ev["pass_fwd_speed"] and ev["pass_fwd_distance"]
    # DESIGN 13 (post phase 2): the traction-limited start may cost up to 7 % of the distance ...
    summary["segments"][0]["dist_ratio_gt_over_cmd"] = 0.945
    assert ev_flat()["pass"]
    summary["segments"][0]["dist_ratio_gt_over_cmd"] = 0.925
    ev = ev_flat()
    assert not ev["pass"] and ev["pass_fwd_speed"] and not ev["pass_fwd_distance"]
    # ... but the steady-state speed must track the command
    summary["segments"][0]["dist_ratio_gt_over_cmd"] = 0.96
    summary["segments"][0]["speed_tracking_ratio"] = 0.93
    ev = ev_flat()
    assert not ev["pass"] and not ev["pass_fwd_speed"] and ev["pass_fwd_distance"]
    summary["segments"][0]["speed_tracking_ratio"] = 1.015
    summary["segments"][1]["yawrate_tracking_ratio"] = 0.3
    assert not ev_flat()["pass"]
    res = {"launches": [{"controllers": {"active": True, "t_s": 30}, "spawner_failures": [], "renderer": {"d3d12": True},
                         "site": "flat", "drive": {"evaluation": {"pass": True, "rtf_driving": 0.97}}}] * 3}
    j = rr.judge(res)
    assert j["controllers_active"]["pass"] and j["no_spawner_failure"]["pass"] and j["three_launches"]["pass"]


# ==============================================================================================
# report

def test_report_labels_and_comparison_runs(tmp_path):
    reports = tmp_path / "reports"
    reports.mkdir()
    (reports / "bench_world_flat.json").write_text(json.dumps({
        "kind": "bench_world", "created": "2026-10-25T12:00:00", "world_sdf": "/x/apollo16_core.sdf",
        "label": "flat site", "lidar_preset": "real", "status": "ok", "pass": True, "load": {"first_lidar_s": 19.5},
        "window": {"rtf_mean": 0.996, "wall_s": 40.0}, "mem": {"hwm_mb": 2700}, "renderer": {"d3d12": True},
        "log": {"n_err": 0}, "thresholds": {"load_time_max_s": 30, "rtf_min": 0.95, "server_rss_max_gb": 5}}))
    (reports / "robot_tests_stock.json").write_text(json.dumps({
        "kind": "robot_tests", "created": "2026-10-25T13:00:00", "world": "/x/apollo16_core_stock.sdf",
        "launch_kind": "ours", "role": "comparison", "label": "stock variant",
        "launches": [{"index": 0, "site": "flat", "controllers": {"active": True, "t_s": 25.0}, "spawner_failures": [],
                      "drive": {"evaluation": {"pass": False, "turn_yawrate_tracking": 0.66}}}],
        "checks": {"three_launches": {"pass": False, "launches": 1},
                   "drive_flat": {"pass": False, "runs": [{"turn_yawrate_tracking": 0.66}]}}}))
    txt = rep.build_report(reports, figures=False).read_text()
    summary = txt.split("## Summary")[1].split("## 1.")[0]
    assert "bench apollo16_core.sdf (real) [flat site]" in summary
    assert "stock variant: three_launches" in summary and "INFO" in summary and "FAIL" not in summary
    assert "**Overall: PASS**" in txt          # the comparison run does not fail the acceptance


def test_report_builds(tmp_path):
    cfg = mini_cfg(tmp_path)
    lay = build_mini_world(tmp_path, cfg)
    sc.run_static_checks(cfg, n_dz_samples=5000, verbose=False)
    reports = lay.reports_dir
    p = synthetic_capture(tmp_path / "capture_000.npz")
    grid = Grid(np.zeros((801, 801), np.float32), -20.0, -20.0, 0.05)
    lc.run_lidar_checks([p], heights=lc.GridHeights([grid]), rocks={"x": np.array([3.0]), "y": np.array([0.0]),
                        "D": np.array([0.2]), "exposed_h": np.array([0.1]), "z": np.array([0.05])},
                        out=reports / "lidar_checks.json")
    (reports / "bench_world_test.json").write_text(json.dumps({
        "kind": "bench_world", "created": "2026-10-25T12:00:00", "world_sdf": "/x/apollo16_core.sdf",
        "lidar_preset": "real", "status": "ok", "pass": True, "load": {"first_lidar_s": 12.3},
        "window": {"rtf_mean": 0.998, "wall_s": 30.0}, "cpu": {"server_pct": 70.0}, "mem": {"hwm_mb": 2500},
        "renderer": {"d3d12": True}, "log": {"n_err": 0, "errors": []},
        "thresholds": {"load_time_max_s": 30, "rtf_min": 0.95, "server_rss_max_gb": 5}}))
    (reports / "robot_tests.json").write_text(json.dumps({
        "kind": "robot_tests", "created": "2026-10-25T12:30:00", "world": "/x/apollo16_core.sdf", "launch_kind": "stock",
        "launches": [{"index": 0, "site": "flat", "controllers": {"active": True, "t_s": 25.0}, "spawner_failures": [],
                      "timeline_s": {"entity_created": 12.0}, "drive": {"evaluation": {"pass": True, "rtf_driving": 0.97,
                                                                                         "fwd_dist_ratio": 0.96}}}],
        "checks": {"controllers_active": {"pass": True, "active": 1, "launches": 1, "t_active_s": [25.0]}}}))
    out = rep.build_report(reports)
    txt = out.read_text()
    assert "# Validation report" in txt and "## Summary" in txt and "fig_deviogram.png" in txt
    for f in ("fig_deviogram.png", "fig_sfd_craters.png", "fig_sfd_rocks.png", "fig_dz_hist.png", "fig_lidar_range.png"):
        assert (reports / f).exists(), f
    assert "bench apollo16_core.sdf" in txt and "controllers_active" in txt
