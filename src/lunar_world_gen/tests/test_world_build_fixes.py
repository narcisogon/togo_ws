"""Regression tests of the build fix round (docs/impl/fix_build.md, CR-13).

Covered: max-edge refinement of collision TINs (RTF), rock re-seating on the rendered TIN (CR-05,
FID-08, FID-10), soft/hard budgets (CR-01), the tile cap headroom (CR-04), the static rock and SDF
checks (CR-03, CR-08a), the build lock and atomic temp names (CR-06), --status without a library
(CR-08b), cache keys (CR-11), float32 rock meshes (CR-12), the inspect_stock variant and world_info
without rock_mu (CR-09). The build-level cases are in tests/test_world_build.py.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

from lunar_world_gen import build as B
from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.layout import Layout, Rect
from lunar_world_gen.mesh.locate import PiecewiseTinHeight
from lunar_world_gen.mesh.refine import edge_lengths, refine_max_edge
from lunar_world_gen.mesh.stitch import check_stitched, make_piece, stitch_pieces, union_topology
from lunar_world_gen.mesh.tin import tin_from_grid, tin_max_error, tin_surface_on_grid
from lunar_world_gen.mesh.types import Mesh


def rough_grid(n: int = 251, sp: float = 0.02, amp: float = 0.03, seed: int = 0, x0: float = 0.0,
               y0: float = 0.0, slope: float = 0.2) -> Grid:
    r = np.random.default_rng(seed)
    w = r.standard_normal((n, n))
    f = np.fft.fftfreq(n, d=sp)
    F = np.hypot(f[:, None], f[None, :])
    F[0, 0] = 1.0
    S = F ** -1.8
    S[0, 0] = 0.0
    z = np.fft.ifft2(np.fft.fft2(w) * S).real
    z = z / z.std() * amp
    xs = np.arange(n) * sp
    z = z + slope * xs[None, :]                      # a tilted plane: long TIN triangles on it
    return Grid(z.astype(np.float32), x0, y0, sp)


# ----------------------------------------------------------------------------------- refinement
@pytest.mark.parametrize("L", [0.15, 0.3, 0.6])
def test_refine_max_edge_contract(L):
    g = rough_grid()
    m = tin_from_grid(g, 0.015, "t")
    r, rep = refine_max_edge(m, g, L)
    assert rep["edges_over_limit_after"] == 0 and edge_lengths(r).max() <= L + 1e-9
    assert r.n_triangles > m.n_triangles
    # every vertex is a grid node and carries the node height
    i = np.rint((r.vertices[:, 0] - g.x0) / g.spacing).astype(int)
    j = np.rint((r.vertices[:, 1] - g.y0) / g.spacing).astype(int)
    assert np.allclose(r.vertices[:, 0], g.x0 + i * g.spacing) and np.allclose(r.vertices[:, 1], g.y0 + j * g.spacing)
    assert np.array_equal(r.vertices[:, 2], np.asarray(g.z, dtype=np.float64)[j, i])
    # the input vertices are kept unchanged; the corners are present
    assert np.array_equal(r.vertices[:m.n_vertices], m.vertices)
    # conforming (no T-junction / crack inside the piece), CCW, covers the rectangle
    piece = make_piece(g, Rect(g.x0, g.x1, g.y0, g.y1), 0.015, 0, name="p", normals=False, max_edge_m=L)
    topo = union_topology([piece])
    assert topo["ok"] and topo["euler"] == 1, topo
    assert np.all(r.face_normals()[:, 2] > 0)
    err = tin_max_error(r, g)
    assert err["uncovered_nodes"] == 0 and err["max_abs_m"] <= 2 * 0.015


def test_refine_handles_one_node_boundary_strips():
    """pydelatin puts vertices one node inside the rectangle side; no grid node lies strictly inside
    such slivers, so plain snapped bisection got stuck (long edges left in the strip)."""
    g = rough_grid(seed=3, amp=0.002, slope=0.0)
    z = np.asarray(g.z, dtype=np.float64).copy()
    z[:, 1] += 0.05 * np.sin(np.arange(g.ny) * 0.37)          # a ridge one node inside the west side
    z[1, :] += 0.05 * np.cos(np.arange(g.nx) * 0.29)          # and one inside the south side
    g = Grid(z.astype(np.float32), g.x0, g.y0, g.spacing)
    m = tin_from_grid(g, 0.005, "strips")
    near = (np.abs(m.lattice[:, 0] - m.lattice[:, 0].min()) == 2) | (np.abs(m.lattice[:, 1] - m.lattice[:, 1].min()) == 2)
    assert near.sum() > 10                                     # the case is really present
    r, rep = refine_max_edge(m, g, 0.2)
    assert rep["edges_over_limit_after"] <= 2, rep
    p = make_piece(g, Rect(g.x0, g.x1, g.y0, g.y1), 0.005, 0, name="s", normals=False, max_edge_m=0.2)
    assert union_topology([p])["ok"]


def test_refined_pieces_stitch_with_neighbours():
    a = rough_grid(n=201, seed=1)
    b = Grid(rough_grid(n=101, sp=0.04, seed=2).z, 4.0, 0.0, 0.04)     # coarser neighbour east of a
    pa = make_piece(a, Rect(0.0, 4.0, 0.0, 4.0), 0.01, 0, name="tile_a_collision", normals=False, max_edge_m=0.3)
    pb = make_piece(b, Rect(4.0, 8.0, 0.0, 4.0), 0.01, 1, name="ring_b_collision", normals=False)
    stitch_pieces([pa, pb], copy_normals=False)
    chk = check_stitched([pa, pb], compare_normals=False)
    assert chk["ok"] and chk["mismatches"] == 0 and chk["t_junctions"] == 0


def test_seam_band_refinement_only_touches_the_band():
    g = rough_grid(seed=4)
    rect = Rect(g.x0, g.x1, g.y0, g.y1)
    p0 = make_piece(g, rect, 0.005, 0, name="t", uv_tile_m=2.0)
    p1 = make_piece(g, rect, 0.005, 0, name="t", uv_tile_m=2.0, max_edge_m=0.08, max_edge_band_m=0.2)
    assert p1.mesh.n_triangles > p0.mesh.n_triangles
    assert p1.mesh.normals is not None and p1.mesh.uvs is not None and p1.mesh.normals.shape[0] == p1.mesh.n_vertices
    new = p1.mesh.vertices[p0.mesh.n_vertices:]
    d = np.minimum.reduce([new[:, 0] - g.x0, g.x1 - new[:, 0], new[:, 1] - g.y0, g.y1 - new[:, 1]])
    # closure may split the longest edge of a band triangle, so new vertices stay near the band
    assert np.mean(d <= 0.4) > 0.9


# ----------------------------------------------------------------------------------- locate / recontact
def test_piecewise_tin_height_matches_tin_and_falls_back():
    g = rough_grid(n=101, seed=5)
    left = make_piece(g, Rect(0.0, 1.0, 0.0, 2.0), 0.01, 0, name="l", normals=False)
    right = make_piece(g, Rect(1.0, 2.0, 0.0, 2.0), 0.01, 1, name="r", normals=False)
    stitch_pieces([left, right], copy_normals=False)
    H = PiecewiseTinHeight([((0.0, 1.0, 0.0, 2.0), lambda: (left.mesh.vertices, left.mesh.triangles)),
                            ((1.0, 2.0, 0.0, 2.0), lambda: (right.mesh.vertices, right.mesh.triangles))],
                           fallback=lambda x, y: np.full(np.shape(x), -99.0))
    sub = g.crop(0.0, 1.0, 0.0, 2.0)
    ref = tin_surface_on_grid(left.mesh, sub)
    X, Y = sub.mesh_xy()
    assert np.allclose(H(X.ravel(), Y.ravel()), ref.ravel(), atol=1e-9)
    assert H(np.array([5.0]), np.array([5.0]))[0] == -99.0
    # on the shared seam both pieces give the same height
    y = np.linspace(0.0, 2.0, 37)
    zl = PiecewiseTinHeight([((0.0, 1.0, 0.0, 2.0), lambda: (left.mesh.vertices, left.mesh.triangles))])(np.ones_like(y), y)
    zr = PiecewiseTinHeight([((1.0, 2.0, 0.0, 2.0), lambda: (right.mesh.vertices, right.mesh.triangles))])(np.ones_like(y), y)
    assert np.allclose(zl, zr, atol=1e-9)


@pytest.fixture(scope="module")
def synth(tmp_path_factory):
    from lunar_world_gen.rocks.library import RockLibrary, write_synthetic_library

    root = tmp_path_factory.mktemp("lib")
    write_synthetic_library(root / "lib", counts={"pebble": 4, "medium": 4, "large": 2}, seed=1)
    lib = RockLibrary(root / "lib")
    cfg = load_config(overrides=[f"paths.rock_library={root / 'lib'}"])
    return cfg, lib, root


def _placed(cfg, lib, ground, n=400, seed=7):
    from lunar_world_gen.rocks.catalog import empty_catalog
    from lunar_world_gen.rocks.placement import place_rocks

    r = np.random.default_rng(seed)
    x = r.uniform(0.3, 3.7, n)
    y = r.uniform(0.3, 3.7, n)
    D = np.exp(r.uniform(np.log(0.02), np.log(0.4), n))
    f = place_rocks(cfg, r, x, y, D, "core", ground, lib)
    cat = empty_catalog(n)
    cat["id"] = np.arange(n)
    cat["x"], cat["y"], cat["D"] = x, y, D
    for k, v in f.items():
        cat[k] = v
    return cat


def test_recontact_reproduces_placement(synth):
    from lunar_world_gen.rocks.placement import GroundGrid
    from lunar_world_gen.rocks.recontact import recontact

    cfg, lib, _ = synth
    g = rough_grid(n=201, sp=0.02, seed=6)
    ground = GroundGrid(g)
    cat = _placed(cfg, lib, ground)
    again = recontact(cfg, cat, lib, ground.height)
    assert np.allclose(again["z"], cat["z"], rtol=0, atol=1e-12)
    assert np.allclose(again["exposed_h"], cat["exposed_h"], rtol=0, atol=1e-12)
    for f in ("x", "y", "qw", "qx", "qy", "qz", "scale", "burial", "D"):
        assert np.array_equal(again[f], cat[f])


def test_recontact_on_rendered_tin_removes_hovering_and_submerged_rocks(synth):
    """FID-10 / CR-05: placed on the grid, rocks hover above (or sink below) a coarse TIN of it; after
    re-seating on the TIN every re-seated rock touches it."""
    from lunar_world_gen.rocks.placement import GroundGrid
    from lunar_world_gen.rocks.recontact import contact_clearance, recontact

    cfg, lib, _ = synth
    g = rough_grid(n=201, sp=0.02, seed=8, amp=0.03)
    cat = _placed(cfg, lib, GroundGrid(g), n=600, seed=9)
    p = make_piece(g, Rect(g.x0, g.x1, g.y0, g.y1), 0.03, 0, name="coarse", normals=False)  # 3 cm "rendered" TIN
    tin = PiecewiseTinHeight([((g.x0, g.x1, g.y0, g.y1), lambda: (p.mesh.vertices, p.mesh.triangles))])
    lo0, hi0 = contact_clearance(tin, lib, cat)
    assert (lo0 > 0.002).sum() > 5                                      # the problem is reproduced
    new = recontact(cfg, cat, lib, tin)
    lo1, hi1 = contact_clearance(tin, lib, new)
    # the contact rule exactly: the lowest vertex is buried by burial x (vertical extent of the rock)
    from lunar_world_gen.rocks.placement import catalog_quats, quat_to_matrix

    R = quat_to_matrix(catalog_quats(new))
    depth = np.empty(len(new))
    for k, r in enumerate(new):
        V = (R[k] @ lib.contact_points(int(r["proto"]), int(r["lod"])).T).T * r["scale"]
        depth[k] = r["burial"] * (V[:, 2].max() - V[:, 2].min())
    assert np.allclose(lo1, -depth, rtol=0, atol=1e-9)
    assert (lo1 > 1e-9).sum() == 0                                      # nothing hovers
    assert (hi1 < 0).sum() <= (hi0 < 0).sum()                           # no more rocks hidden than before
    # only z / exposed_h change
    assert np.array_equal(new["x"], cat["x"]) and np.array_equal(new["qw"], cat["qw"])
    # rows= restricts the update
    keep = np.zeros(len(cat), dtype=bool)
    keep[:10] = True
    part = recontact(cfg, cat, lib, tin, rows=keep)
    assert np.array_equal(part["z"][10:], cat["z"][10:]) and np.allclose(part["z"][:10], new["z"][:10])


def test_fillet_rows_selection(synth):
    from lunar_world_gen.rocks.recontact import fillet_rows

    cfg, lib, _ = synth
    from lunar_world_gen.rocks.catalog import empty_catalog

    c = empty_catalog(5)
    c["zone"] = [0, 0, 0, 1, 0]
    c["D"] = [0.2, 0.05, 0.2, 0.5, 0.2]
    c["burial"] = [0.2, 0.2, 0.0, 0.2, 0.04]       # the last: a perched rock's compliance burial, no fillet
    c["exposed_h"] = 0.05
    assert fillet_rows(cfg, c).tolist() == [True, False, False, False, False]


# ----------------------------------------------------------------------------------- budgets (CR-01)
def _counts(rocks=6_590_000, visual=12_000_000, core_col=400_000, col_total=800_000, tile_max=40_000):
    return {"core_per_tile": {"ground": {"max": tile_max}}, "core_ground": 4_000_000, "core_rocks": rocks,
            "visual_total": visual, "core_collision": core_col, "collision_total": col_total}


def test_budgets_soft_and_hard():
    b = dict(B.DEFAULT_BUDGETS)
    ok = B.Builder._check_budgets(_counts(rocks=6_683_808), b, [])       # seed 17 of the review (E9)
    assert ok["core_rocks_max_tris"]["ok"] and ok["core_rocks_warn_tris"]["ok"] is False
    assert ok["core_rocks_warn_tris"]["hard"] is False
    hard = [k for k, v in ok.items() if v.get("ok") is False and v.get("hard", True)]
    assert hard == []
    bad = B.Builder._check_budgets(_counts(rocks=7_600_000, visual=21_000_000), b, [])
    assert {k for k, v in bad.items() if v.get("ok") is False and v.get("hard", True)} == \
        {"core_rocks_max_tris", "visual_total_max_tris"}
    # a config that still sets only the old single limit keeps it hard
    old = dict(b, core_rocks_max_tris=6_600_000)
    assert B.Builder._check_budgets(_counts(rocks=6_683_808), old, [])["core_rocks_max_tris"]["ok"] is False


def test_rock_triangle_spread_over_seeds_fits_the_new_hard_limit():
    """CR-01 (review E9): eight seeds gave 6.54-6.68 M core rock triangles (mean 6.60 M, sd 52 k);
    the hard limit 7.5 M is > 17 sd above the mean, the 6.6 M figure is a warning only."""
    seeds = [6_591_400, 6_683_808, 6_667_496, 6_553_572, 6_598_678, 6_615_008, 6_562_972, 6_542_674]
    b = dict(B.DEFAULT_BUDGETS)
    for t in seeds:
        res = B.Builder._check_budgets(_counts(rocks=t), b, [])
        assert res["core_rocks_max_tris"]["ok"]


def test_tile_cap_headroom_covers_overshoot_and_stitch_inserts():
    """CR-04 (review E1): cap 2000 gave 2013 triangles after stitching to a coarse neighbour."""
    zA = rough_grid(n=401, sp=0.01, amp=0.01, seed=2, slope=0.0)
    gB = Grid(rough_grid(n=17, sp=0.25, amp=0.01, seed=3, slope=0.0).z, 0.0, 0.0, 0.25)
    cap = 2000
    cap_eff = int(cap * (1 - B.TILE_CAP_HEADROOM))
    A = make_piece(zA, Rect(0.0, 2.0, 0.0, 4.0), 0.0005, 0, name="tile_A_ground", uv_tile_m=2.0, max_triangles=cap_eff)
    Bp = make_piece(gB, Rect(2.0, 4.0, 0.0, 4.0), 0.0005, 1, name="ring_B_ground", uv_tile_m=2.0)
    stitch_pieces([A, Bp])
    assert check_stitched([A, Bp])["ok"]
    assert A.mesh.n_triangles <= cap


# ----------------------------------------------------------------------------------- static checks
def _catalog_dict(cfg, n=200, seed=0):
    from lunar_world_gen.rocks.catalog import empty_catalog

    lay = Layout(cfg)
    core = lay.zone_rect("core")
    r = np.random.default_rng(seed)
    c = empty_catalog(n)
    c["id"] = np.arange(n)
    c["zone"] = 0
    c["x"] = r.uniform(core.x0 + 1, core.x1 - 1, n)
    c["y"] = r.uniform(core.y0 + 1, core.y1 - 1, n)
    c["tile_i"] = np.floor((c["x"] - core.x0) / lay.tile_size).astype(int)
    c["tile_j"] = np.floor((c["y"] - core.y0) / lay.tile_size).astype(int)
    c["D"] = r.uniform(0.02, 0.3, n)
    c["qw"] = 1.0
    c["scale"] = 1.0
    c["collide"] = c["D"] >= float(cfg.rocks.collision_min_D_m)
    return {k: c[k] for k in c.dtype.names}


@pytest.mark.parametrize("corruption,problem", [
    ("collide", "collide_flag_mismatch"), ("quat", "quat_norm_max_dev"), ("tiny", "core_below_D_min"),
    ("dup", "duplicate_ids"), ("proto", "proto_out_of_range"), (None, None)])
def test_check_rocks_judges_catalog_sanity(tmp_path, synth, corruption, problem):
    """CR-03 (review E2): inverted collide flags, x2 quaternions, a 1 mm rock and a duplicated id all
    passed before."""
    from lunar_world_gen.validate import static_checks as sc

    cfg, lib, root = synth
    d = _catalog_dict(cfg)
    if corruption == "collide":
        d["collide"] = ~d["collide"]
    elif corruption == "quat":
        d["qw"] = d["qw"] * 2.0
    elif corruption == "tiny":
        d["D"][0] = 0.001
    elif corruption == "dup":
        d["id"][1] = d["id"][0]
    elif corruption == "proto":
        d["proto"][0] = 999
    p = tmp_path / "rocks_catalog.npz"
    np.savez(p, **d)
    r = sc.check_rocks(cfg, {"rock_catalog": str(p), "rock_library": str(root / "lib" / "library.json")})
    if problem is None:
        assert r["sanity"]["problems"] == []
    else:
        assert problem in r["sanity"]["problems"] and r["status"] == "fail"


def test_check_rocks_ring_d_min_exempts_the_core_edge_band(tmp_path, synth):
    """Small ring rocks are wrong, except in the core-edge band where the core population continues
    on purpose (rocks.edge_band, FID-04)."""
    from lunar_world_gen.validate import static_checks as sc

    _, _, root = synth
    cfg = load_config(overrides=["rocks.edge_band={enabled: true, width_m: 6.0}"])
    d = _catalog_dict(cfg, n=10)
    d["zone"][:2] = 1
    d["x"][:2] = [52.0, 120.0]                     # 2 m and 70 m outside the 100 m core
    d["y"][:2] = 0.0
    d["D"][:2] = 0.1
    d["collide"][:2] = False
    p = tmp_path / "rocks_catalog.npz"
    np.savez(p, **d)
    s = sc.check_rocks(cfg, {"rock_catalog": str(p)})["sanity"]
    assert s["ring_edge_band_below_D_min"] == 1 and s["ring_below_D_min"] == 1
    assert "ring_below_D_min" in s["problems"]


def _mini_sdf(path: Path, engine: str, mesh: Path) -> None:
    path.write_text(f'<?xml version="1.0"?><sdf version="1.11"><world name="w">'
                    f'<plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">'
                    f'<engine><filename>{engine}</filename></engine></plugin>'
                    f'<model name="m"><link name="l"><visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh>'
                    f'</geometry></visual></link></model></world></sdf>')


def test_check_sdf_missing_patched_plugin_with_stock_fallback(tmp_path):
    """CR-08a (review E7): the build exited 2 without the patched plugin although the stock variant
    was complete; now that is a warning. A missing mesh still fails."""
    from lunar_world_gen.validate import static_checks as sc

    cfg = load_config()
    mesh = tmp_path / "m.glb"
    mesh.write_bytes(b"x")
    gone = str(tmp_path / "no_plugin.so")
    _mini_sdf(tmp_path / "w.sdf", gone, mesh)
    _mini_sdf(tmp_path / "w_inspect.sdf", gone, mesh)
    _mini_sdf(tmp_path / "w_stock.sdf", "gz-physics-bullet-plugin", mesh)
    _mini_sdf(tmp_path / "w_inspect_stock.sdf", "gz-physics-bullet-plugin", mesh)
    files = [str(p) for p in sorted(tmp_path.glob("*.sdf"))]
    r = sc.check_sdf(cfg, {"sdf_files": files})
    assert r["status"] == "pass" and len(r["warnings"]) == 2
    assert r["sdfs"]["w_inspect.sdf"]["engine_missing_fallback"] == "w_inspect_stock.sdf"
    (tmp_path / "w_inspect_stock.sdf").unlink()
    (tmp_path / "w_stock.sdf").unlink()
    r2 = sc.check_sdf(cfg, {"sdf_files": [str(tmp_path / "w.sdf")]})
    assert r2["status"] == "fail"
    _mini_sdf(tmp_path / "w_stock.sdf", "gz-physics-bullet-plugin", tmp_path / "missing_mesh.glb")
    r3 = sc.check_sdf(cfg, {"sdf_files": [str(tmp_path / "w.sdf"), str(tmp_path / "w_stock.sdf")]})
    assert r3["status"] == "fail"


def test_summarize_requires_all_checks():
    from lunar_world_gen.validate import static_checks as sc

    s = sc.summarize_results({"a": {"status": "pass"}, "b": {"status": "skipped"}})
    assert s["pass"] and not s["all_ran"]


# ----------------------------------------------------------------------------------- build plumbing
def test_status_without_rock_library(tmp_path):
    """CR-08b (review E7): --status raised FileNotFoundError without the rock library."""
    cfg = load_config(overrides=[f"paths.rock_library={tmp_path / 'none'}", f"paths.build_dir={tmp_path / 'b'}",
                                 f"paths.world_dir={tmp_path / 'w'}"])
    st = B.Builder(cfg, log=lambda *a: None).status()
    assert set(st) == set(B.STAGES)
    assert not st["rocks"][0] and "missing" in st["rocks"][1]
    assert B.main(["--status", "--set", f"paths.rock_library={tmp_path / 'none'}", "--set",
                   f"paths.build_dir={tmp_path / 'b'}", "--set", f"paths.world_dir={tmp_path / 'w'}"]) == 0


def test_build_lock_is_exclusive(tmp_path):
    """CR-06: a second build on the same dirs fails fast instead of corrupting shared temp files."""
    cfg = load_config(overrides=[f"paths.build_dir={tmp_path / 'b'}", f"paths.world_dir={tmp_path / 'w'}"])
    lay = Layout(cfg)
    with B.BuildLock([lay.build_dir / ".build.lock"]):
        with pytest.raises(B.BuildError, match="another build holds"):
            B.Builder(cfg, log=lambda *a: None).run(["validate"])
    # free again: the lock is released (the run then fails later for another reason, not the lock)
    with pytest.raises(B.BuildError) as e:
        B.Builder(cfg, log=lambda *a: None).run(["validate"])
    assert "another build" not in str(e.value)


def test_atomic_writes_use_unique_temp_names(tmp_path):
    p = tmp_path / "x.json"
    a, b = B._tmp_path(p, ".json"), B._tmp_path(p, ".json")
    assert a != b and str(os.getpid()) in a.name and a.parent == p.parent
    B._write_json(p, {"a": 1})
    g = Grid(np.zeros((3, 3), np.float32), 0.0, 0.0, 0.01)
    B._save_grid(g, tmp_path / "grid")
    m = Mesh(np.zeros((3, 3)), np.array([[0, 1, 2]]))
    B._save_mesh(tmp_path / "m.npz", m)
    assert sorted(f.name for f in tmp_path.iterdir()) == ["grid.json", "grid.npy", "m.npz", "x.json"]
    from lunar_world_gen.mesh.gltf import write_glb
    from lunar_world_gen.mesh.types import MaterialSpec

    write_glb(tmp_path / "t.glb", Mesh(np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], float), [[0, 1, 2]]),
              MaterialSpec("m"))
    assert not [f for f in tmp_path.iterdir() if "tmp" in f.name]


def test_keys_cover_code_env_and_library(synth, monkeypatch, tmp_path):
    """CR-11: terrain code, foundation code, third-party versions and every library file are in the keys."""
    cfg, lib, root = synth
    cfg = load_config(overrides=[f"paths.rock_library={root / 'lib'}", f"paths.build_dir={tmp_path / 'b'}",
                                 f"paths.world_dir={tmp_path / 'w'}"])
    b = B.Builder(cfg, log=lambda *a: None)
    inp = b.inputs("rocks")
    assert {"numpy", "scipy", "numba", "pydelatin"} <= set(inp["env"]) and inp["library_digest"]
    k0 = B.library_digest(root / "lib")
    npz = sorted((root / "lib" / "npz").glob("*.npz"))[0]
    os.utime(npz, ns=(npz.stat().st_atime_ns, npz.stat().st_mtime_ns + 10 ** 9))
    assert B.library_digest(root / "lib") != k0
    t0 = b.inputs("terrain")
    assert t0["terrain_code"] and t0["foundation"]
    monkeypatch.setattr(B, "terrain_code_hash", lambda: "changed")
    assert B.Builder(cfg, log=lambda *a: None).key("terrain") != b.key("terrain")
    assert "sdf_validator" in b.inputs("world")


# ----------------------------------------------------------------------------------- memory (CR-12)
def test_rock_meshes_stay_float32(synth):
    from lunar_world_gen.rocks.merge import rocks_mesh
    from lunar_world_gen.rocks.placement import GroundGrid

    cfg, lib, _ = synth
    cat = _placed(cfg, lib, GroundGrid(rough_grid(n=201, seed=11)), n=50)
    m = rocks_mesh(cat, lib, dtype=np.float32)
    assert m.vertices.dtype == np.float32
    m.lattice = None
    m.validate()
    m64 = Mesh(np.zeros((2, 3), dtype=np.float32).astype(np.float64), np.zeros((0, 3)))
    assert m64.vertices.dtype == np.float64
    assert Mesh([[0, 0, 0]], np.zeros((0, 3))).vertices.dtype == np.float64


# ----------------------------------------------------------------------------------- world files
def test_inspect_stock_variant_and_world_info(tmp_path):
    import xml.etree.ElementTree as ET

    from lunar_world_gen.world import sdf, world_info

    cfg = load_config(overrides=[f"paths.world_dir={tmp_path}"])
    lay = Layout(cfg)
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "g.glb").write_bytes(b"x")
    vis = [sdf.VisualSpec("g", "meshes/g.glb", "regolith_core")]
    col = [sdf.CollisionSpec("c", "meshes/g.glb", 0.4)]
    paths = sdf.write_all_variants(cfg, lay, vis, col, require_textures=False)
    assert set(paths) == {"", "inspect", "stock", "inspect_stock"}
    w = ET.parse(paths["inspect_stock"]).getroot().find("world")
    assert w.find("plugin/engine/filename").text == "gz-physics-bullet-plugin"
    assert float(w.find("physics/max_step_size").text) == pytest.approx(float(cfg.physics.step_s.stock))
    assert {l.get("name") for l in w.findall("light")} == {"sun", "earthshine", "overhead_fill"}
    assert not list(tmp_path.glob("*.tmp*"))
    info = world_info.build_world_info(cfg, lay, {})
    assert info["sdf_variants"]["inspect_stock"] == "apollo16_core_inspect_stock.sdf"
    assert "rock_mu" not in info["physics"] and "friction_note" in info["physics"]
