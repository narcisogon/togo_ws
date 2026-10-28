"""Tests for lunar_world_gen.rocks (library reader, population, placement, fillets, collision, merge,
catalog). Run from src/lunar_world_gen:  python -m pytest tests/test_world_rocks.py -q

Most tests use a synthetic library (random convex rocks in the production file format); the last
test checks the production library in cfg.paths.rock_library and is skipped if it is missing.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree

from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.rocks import (CATALOG_DTYPE, GroundFunction, GroundGrid, RockLibrary, apply_fillets,
                                   core_collision_grid, cumulative_counts, flat_ground, load_catalog,
                                   load_catalog_meta, ring_collision_grid, rocks_mesh, sample_rocks, save_catalog,
                                   validate_catalog, write_synthetic_library)
from lunar_world_gen.rocks.catalog import empty_catalog, poisson_interval, split_by_tile
from lunar_world_gen.rocks.collision_field import lowpass_sigma_m, rasterize_tops, transformed_hulls
from lunar_world_gen.rocks.fillets import fillet_field, fillet_params
from lunar_world_gen.rocks.placement import (catalog_quats, quat_from_axis_angle, quat_mul, quat_to_matrix,
                                             quat_z_to)
from lunar_world_gen.rocks.population import (BrokenPowerLaw, crater_factor, density_field, fresh_craters,
                                              _rect_mean)


# ---------------------------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    path = tmp_path_factory.mktemp("rocklib")
    return RockLibrary(write_synthetic_library(path, counts={"pebble": 5, "medium": 6, "large": 4}, seed=3))


def rough_height(x, y):
    """Analytic rough, sloping ground (slopes up to ~15 deg)."""
    return 0.05 * x + 0.03 * np.sin(2.1 * x) * np.cos(1.7 * y) + 0.008 * np.sin(9.0 * x + 4.0 * y)


@pytest.fixture(scope="module")
def rough_ground():
    return GroundFunction(rough_height)


@pytest.fixture(scope="module")
def core_sample(cfg, lib, rough_ground):
    """Rocks on rough ground in a 24 x 24 m window of the core (with a fresh crater annulus)."""
    craters = {"x": np.array([5.0]), "y": np.array([4.0]), "D": np.array([8.0]), "rho": np.array([0.9])}
    return sample_rocks(cfg, "core", rough_ground, craters=craters, library=lib, rect=(-12, 12, -12, 12),
                        return_meta=True)


# ---------------------------------------------------------------------------------------------
# size-frequency distribution
# ---------------------------------------------------------------------------------------------
def test_sfd_matches_calibration(cfg):
    sfd = BrokenPowerLaw.from_config(cfg)
    # calibration table: per 100 m^2, D >= 2/5/10/20/50/100 cm = 2160/166/23.9/3.43/0.263/0.0378
    want = [2160, 166, 23.9, 3.43, 0.263, 0.0378]
    got = sfd.N(np.array([0.02, 0.05, 0.10, 0.20, 0.50, 1.0])) * 100.0
    np.testing.assert_allclose(got, want, rtol=0.01)
    # continuity at the 1 m break and the inverse
    assert abs(sfd.N(1.0 - 1e-12) - sfd.N(1.0 + 1e-12)) < 1e-9
    D = np.array([0.02, 0.07, 0.5, 1.0, 1.7, 2.9])
    np.testing.assert_allclose(sfd.inverse(sfd.N(D)), D, rtol=1e-10)


def test_sfd_sampling_vs_target(cfg):
    sfd = BrokenPowerLaw.from_config(cfg)
    rng = np.random.default_rng(5)
    n = 2_000_000
    D = sfd.sample(rng, n, 0.02, 3.0)
    assert D.min() >= 0.02 and D.max() <= 3.0
    p_tot = sfd.count_density(0.02, 3.0)
    for d in (0.02, 0.05, 0.10, 0.20):
        mu = n * sfd.count_density(d, 3.0) / p_tot
        k = int(np.sum(D >= d))
        assert abs(k - mu) <= 3.0 * np.sqrt(mu), (d, k, mu)


def test_population_counts_within_3_sigma(cfg, lib):
    """Full pipeline on a large synthetic area (60 x 60 m, ~80 k rocks): N(>=D) at 2/5/10/20 cm."""
    cat, meta = sample_rocks(cfg, "core", flat_ground(), library=lib, rect=(-30, 30, -30, 30), return_meta=True,
                             include_dtm_craters=False)
    rows = cumulative_counts(cat, (0.02, 0.05, 0.10, 0.20), expected=meta["expected_counts"])
    for r in rows:
        assert r["within_3sigma"], r
    # the expected counts follow the SFD times the intensity integral
    sfd = BrokenPowerLaw.from_config(cfg)
    assert abs(meta["expected_counts"][0.02] - sfd.count_density(0.02, 3.0) * meta["intensity_integral_m2"]) < 1e-6


# ---------------------------------------------------------------------------------------------
# density field, craters
# ---------------------------------------------------------------------------------------------
def test_density_field_statistics(cfg):
    f = density_field(cfg)
    from lunar_world_gen.layout import Layout

    core = Layout(cfg).zone_rect("core")
    assert abs(_rect_mean(f, core, 0.5) - 1.0) < 1e-9
    lz = np.log(f.z)
    assert 0.45 < lz.std() < 0.75
    # correlation of ln m at the correlation length is ~ e^-1 (very loose: one realisation)
    lag = int(round(35.0 / f.spacing))
    a = lz - lz.mean()
    rho = np.mean(a[:, :-lag] * a[:, lag:]) / a.var()
    assert 0.1 < rho < 0.6
    f2 = density_field(cfg)
    assert np.array_equal(f.z, f2.z)


def test_crater_enhancement(cfg):
    craters = {"x": np.array([0.0, 20.0, 40.0]), "y": np.zeros(3), "D": np.array([10.0, 10.0, 3.0]),
               "rho": np.array([0.9, 0.2, 1.0])}
    fc = fresh_craters(cfg, craters, include_dtm=False)
    assert len(fc["x"]) == 1                                   # degraded and small craters excluded
    # FID-02: smooth radial profile, peak 1 + 0.5 rho at the rim for debris, nothing for blocks (D < 20 m)
    x = np.array([0.0, 4.9, 5.0, 5.1, 7.0, 9.9, 12.4, 12.6, 20.0])
    e = crater_factor(cfg, x, np.zeros(len(x)), fc, "small")
    assert abs(e[2] - 1.45) < 1e-9 and e.max() <= 1.45 + 1e-9
    assert 1.2 < e[0] < 1.25                                    # floor: half the rim excess at the centre
    assert e[7] == 1.0 and e[8] == 1.0 and e[6] < 1.01          # zero beyond 2.5 R, continuous there
    xd = np.arange(0.0, 15.0, 0.001)                            # continuous everywhere (no step at R, 2 R)
    ed = crater_factor(cfg, xd, np.zeros(len(xd)), fc, "small")
    assert np.abs(np.diff(ed)).max() < 0.002
    np.testing.assert_array_equal(crater_factor(cfg, x, np.zeros(len(x)), fc, "big"), 1.0)
    # the legacy step function is still available
    cfg_old = load_config(overrides=["rocks.crater_enhancement.profile=annulus"])
    e = crater_factor(cfg_old, np.array([0.0, 4.9, 5.1, 7.0, 9.9, 10.1, 20.0]), np.zeros(7), fc)
    np.testing.assert_array_equal(e, [1, 1, 2, 2, 2, 1, 1])
    fc_dtm = fresh_craters(cfg, None, include_dtm=True)          # the fresh 22.6 m DTM crater at (32, -60)
    assert len(fc_dtm["x"]) >= 1 and np.any(np.abs(fc_dtm["D"] - 22.6) < 0.1)


def test_crater_annulus_density(lib):
    # uniform density field, so the crater profile is the only modulation
    cfg = load_config(overrides=["rocks.density_field.sigma_ln=0.0"])
    craters = {"x": np.array([0.0]), "y": np.array([0.0]), "D": np.array([10.0]), "rho": np.array([1.0])}
    cat = sample_rocks(cfg, "core", flat_ground(), craters=craters, library=lib, rect=(-16, 16, -16, 16),
                       include_dtm_craters=False)
    fc = fresh_craters(cfg, craters, include_dtm=False)
    r = np.hypot(cat["x"], cat["y"])
    for lo, hi, D0, D1, band in ((5.0, 6.5, 0.02, 0.05, "small"), (5.0, 10.0, 0.10, 3.0, "big")):
        sel = (cat["D"] >= D0) & (cat["D"] < D1)
        rr = np.linspace(lo, hi, 400)
        mean_e = np.average(crater_factor(cfg, rr, np.zeros(400), fc, band), weights=rr)   # area-weighted
        dens_in = np.sum(sel & (r >= lo) & (r < hi)) / (np.pi * (hi ** 2 - lo ** 2))
        dens_out = np.sum(sel & (r > 13.0)) / (32 * 32 - np.pi * 13.0 ** 2)
        n_in = np.sum(sel & (r >= lo) & (r < hi))
        tol = 4.0 / np.sqrt(max(n_in, 1)) + 4.0 / np.sqrt(max(np.sum(sel & (r > 13.0)), 1))
        assert abs(dens_in / dens_out / mean_e - 1.0) < tol, (band, dens_in / dens_out, mean_e)
    # debris: a mild boost (<= 1.5x) at the rim; blocks: none around a 10 m crater (FID-02)
    assert 1.25 < np.average(crater_factor(cfg, np.linspace(5, 6.5, 400), np.zeros(400), fc, "small")) < 1.45


# ---------------------------------------------------------------------------------------------
# placement
# ---------------------------------------------------------------------------------------------
def test_determinism(cfg, lib, rough_ground):
    a = sample_rocks(cfg, "core", rough_ground, library=lib, rect=(-5, 5, -5, 5))
    b = sample_rocks(cfg, "core", rough_ground, library=lib, rect=(-5, 5, -5, 5))
    assert len(a) > 100
    assert a.tobytes() == b.tobytes()
    cfg2 = load_config(overrides=["world.seed=17"])
    c = sample_rocks(cfg2, "core", rough_ground, library=lib, rect=(-5, 5, -5, 5))
    assert len(c) != len(a) or not np.array_equal(c["x"], a["x"])


def _world_vertices(row, lib):
    pos = lib.contact_points(int(row["proto"]), int(row["lod"]))
    R = quat_to_matrix(catalog_quats(row[None]))[0]
    return pos @ R.T * row["scale"] + np.array([row["x"], row["y"], row["z"]])


def test_no_floating_rocks(core_sample, lib):
    cat, _ = core_sample
    assert len(cat) > 1000
    rng = np.random.default_rng(0)
    idx = np.concatenate([np.nonzero(cat["D"] >= 0.05)[0], rng.choice(len(cat), 400, replace=False)])
    worst = -np.inf
    for i in idx:
        row = cat[i]
        V = _world_vertices(row, lib)
        clearance = V[:, 2] - rough_height(V[:, 0], V[:, 1])
        c_rot = (V[:, 2].max() - V[:, 2].min())
        # lowest vertex at or below the ground: buried by burial * rotated height, perched rocks touch
        assert clearance.min() <= 1e-3, (i, clearance.min())
        np.testing.assert_allclose(clearance.min(), -row["burial"] * c_rot, atol=1e-6)
        worst = max(worst, clearance.min())
        # exposed height = top above the ground at the centre
        np.testing.assert_allclose(row["exposed_h"], V[:, 2].max() - rough_height(row["x"], row["y"]), atol=1e-9)
    assert worst <= 1e-3


def test_grid_ground_contact(cfg, lib):
    """Rocks on a 1 cm Grid ground (bilinear, as the integration uses for H0 tiles) touch it."""
    xs = np.arange(-3.5, 3.5 + 1e-9, 0.01)
    X, Y = np.meshgrid(xs, xs)
    g = Grid(rough_height(X, Y), -3.5, -3.5, 0.01)
    ground = GroundGrid(g)
    cat = sample_rocks(cfg, "core", g, library=lib, rect=(-3, 3, -3, 3))      # a bare Grid is wrapped too
    assert len(cat) > 300
    for row in cat[:300]:
        V = _world_vertices(row, lib)
        c = V[:, 2] - ground.height(V[:, 0], V[:, 1])
        np.testing.assert_allclose(c.min(), -row["burial"] * (V[:, 2].max() - V[:, 2].min()), atol=1e-6)
    n = ground.normal(np.array([0.3]), np.array([-0.2]))
    assert n.shape == (1, 3) and abs(np.linalg.norm(n) - 1) < 1e-12 and n[0, 2] > 0.9


def test_class_lod_burial_and_tilt(cfg, core_sample, lib):
    cat, meta = core_sample
    tab = cfg["rocks"]["classes"]                         # cls indexes the config list
    p_lo = float(cfg["rocks"]["perched_burial"][0])
    for k, c in enumerate(tab):
        sub = cat[cat["cls"] == k]
        if not len(sub):
            continue
        assert np.all(sub["lod"] == int(c["lod"]))
        assert all(lib.prototypes[p].cls == c["library_class"] for p in np.unique(sub["proto"]))
        # buried rocks lie in the class range; perched rocks below it (compliance burial 2-6 %, or a
        # bedding of at most 18 % for keel-shaped undersides / undulating ground, FID-01). Since FID-12
        # the class range is [0, 1] (Beta(1.5, 6) directly), so a buried rock may lie below the 2 %
        # compliance minimum: the lower bound is the smaller of the two ranges.
        lo, hi = float(c["burial"][0]), float(c["burial"][1])
        b = sub["burial"]
        assert np.all(b <= max(hi, 0.18) + 1e-9) and np.all(b >= min(lo, p_lo) - 1e-9)
        if lo >= 0.18:
            assert np.all((b >= lo - 1e-12) | (b <= 0.18 + 1e-9))
    pf = float(cfg["rocks"]["perched_fraction"])
    if min(float(c["burial"][0]) for c in tab) >= 0.2:
        peb = cat[cat["cls"] == 0]                       # pebbles: buried >= 0.2, perched <= 0.18
        assert abs(np.mean(peb["burial"] < 0.2) - pf) < 0.03
    else:
        # with a class range from 0 a perched rock cannot be told from its burial: use the placement's
        # own count of perched draws (binomial sd ~0.005 here)
        assert abs(meta["n_perched"] / meta["n_main"] - pf) < 0.03
    # D = scale * a and scale within the size window
    a = lib.a_array()[cat["proto"]]
    np.testing.assert_allclose(cat["scale"] * a, cat["D"], rtol=1e-12)
    assert np.all(cat["collide"] == (cat["D"] >= cfg["rocks"]["collision_min_D_m"]))
    assert validate_catalog(cat, lib, cfg) == []


def test_separation_rule(cfg, core_sample):
    cat, meta = core_sample
    fac = cfg["rocks"]["min_separation_factor"]
    big = cat[cat["D"] >= 0.10]
    assert len(big) > 50
    tree = cKDTree(np.column_stack([big["x"], big["y"]]))
    pairs = tree.query_pairs(fac * big["D"].max())
    for i, j in pairs:
        d = np.hypot(big["x"][i] - big["x"][j], big["y"][i] - big["y"][j])
        assert d >= fac * 0.5 * (big["D"][i] + big["D"][j]) - 1e-9
    # small rocks are not centred inside a big rock's exclusion disc
    small = cat[cat["D"] < 0.10]
    d, j = tree.query(np.column_stack([small["x"], small["y"]]))
    assert np.all(d >= fac * 0.5 * big["D"][j] - 1e-9)
    assert meta["separation"]["dropped_big"] == 0


def test_ring_zone(cfg, lib):
    cfg_nb = load_config(overrides=["rocks.edge_band.enabled=false"])
    cat = sample_rocks(cfg_nb, "ring", flat_ground(), library=lib, rect=(-100, 100, -100, 100))
    assert len(cat) > 100
    assert np.all(cat["zone"] == 1) and np.all(cat["D"] >= 0.30)
    inside_core = (np.abs(cat["x"]) < 50) & (np.abs(cat["y"]) < 50)
    assert not inside_core.any()
    assert np.all(cat["lod"] == cfg["rocks"]["ring_lod"])
    assert np.all(cat["tile_j"] == -1) and set(np.unique(cat["tile_i"])) <= {0, 1, 2, 3}
    south = cat["tile_i"] == 0
    assert np.all(cat["y"][south] < -50)
    assert np.all((cat["id"] >= 10_000_000) & (cat["id"] < 12_000_000))
    assert not cat["collide"].any()                      # DESIGN 5: collide is core only
    assert validate_catalog(cat, lib, cfg) == []
    # with the core-edge band (default): the same >= 30 cm rows plus band rocks < 30 cm (FID-04)
    both = sample_rocks(cfg, "ring", flat_ground(), library=lib, rect=(-100, 100, -100, 100))
    main = both[both["id"] < 12_000_000]
    band = both[both["id"] >= 12_000_000]
    assert len(main) == len(cat) and np.array_equal(main["x"], cat["x"])
    assert len(band) > 1000 and np.all(band["D"] < 0.30) and not band["collide"].any()
    assert validate_catalog(both, lib, cfg) == []


def test_tiny_window_and_empty_inputs(cfg, lib):
    cat = sample_rocks(cfg, "core", flat_ground(), library=lib, rect=(0.0, 0.01, 0.0, 0.01))
    assert cat.dtype == CATALOG_DTYPE and len(cat) < 5
    empty = empty_catalog(0)
    g = Grid(np.zeros((11, 11)), 0.0, 0.0, 0.02)
    assert np.all(core_collision_grid(cfg, g, empty, lib).z == -0.02)
    assert np.all(apply_fillets(cfg, g, empty, lib).z == 0.0)
    assert rocks_mesh(empty, lib).n_triangles == 0


def test_core_intensity_normalised(cfg):
    """m * e averages exactly 1 over the core, so crater annuli redistribute rocks (core totals stay nominal)."""
    from lunar_world_gen.layout import Layout
    from lunar_world_gen.rocks.population import IntensitySampler, core_intensity_scale

    field = density_field(cfg)
    craters = {"x": np.array([10.0, -30.0]), "y": np.array([5.0, 20.0]), "D": np.array([12.0, 6.0]),
               "rho": np.array([0.9, 0.8])}
    fc = fresh_craters(cfg, craters, include_dtm=True)
    s = core_intensity_scale(cfg, field, fc)
    assert 0.9 < s < 1.0                                   # annuli (incl. the 22.6 m DTM crater) add rocks
    core = Layout(cfg).zone_rect("core")
    lam = IntensitySampler(cfg, core, None, field, fc, np.random.default_rng(0), s).integral()
    assert abs(lam / (core.width * core.height) - 1.0) < 1e-9
    assert abs(core_intensity_scale(cfg, field, fresh_craters(cfg, None, include_dtm=False)) - 1.0) < 1e-3


def test_quaternions():
    rng = np.random.default_rng(1)
    q = quat_from_axis_angle(rng.normal(size=(50, 3)), rng.uniform(0, np.pi, 50))
    R = quat_to_matrix(q)
    np.testing.assert_allclose(np.einsum("kij,kil->kjl", R, R), np.tile(np.eye(3), (50, 1, 1)), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)
    q2 = quat_from_axis_angle(rng.normal(size=(50, 3)), rng.uniform(0, np.pi, 50))
    np.testing.assert_allclose(quat_to_matrix(quat_mul(q, q2)), R @ quat_to_matrix(q2), atol=1e-12)
    n = rng.normal(size=(50, 3))
    n[:, 2] = np.abs(n[:, 2]) + 0.1
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    np.testing.assert_allclose(quat_to_matrix(quat_z_to(n))[:, :, 2], n, atol=1e-12)


# ---------------------------------------------------------------------------------------------
# merge
# ---------------------------------------------------------------------------------------------
def _edge_counts(tris):
    e = np.sort(np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return c


def test_merge_validity(core_sample, lib):
    cat, _ = core_sample
    rows = cat[:3000]
    m = rocks_mesh(rows, lib, name="t")
    m.validate()
    assert m.triangles.min() >= 0 and m.triangles.max() < m.n_vertices
    np.testing.assert_allclose(np.linalg.norm(m.normals, axis=1), 1.0, atol=1e-5)
    assert np.all(m.lattice == -1)
    assert m.uvs.min() >= 0.0 and m.uvs.max() <= 1.0
    expected_tris = sum(lib.prototypes[int(p)].lod_tris[int(l)] for p, l in zip(rows["proto"], rows["lod"]))
    assert m.n_triangles == expected_tris
    # closed: weld equal positions; every edge then has exactly 2 triangles; outward (volume > 0)
    key = np.round(m.vertices, 7)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    tw = inv.ravel()[m.triangles]
    assert np.all(_edge_counts(tw) == 2)
    v = m.vertices
    vol6 = np.einsum("ij,ij->i", v[m.triangles[:, 0]], np.cross(v[m.triangles[:, 1]], v[m.triangles[:, 2]]))
    assert vol6.sum() > 0


def test_merge_transform_exact(lib):
    row = empty_catalog(1)
    row["proto"] = 6
    row["lod"] = 1
    q = quat_from_axis_angle(np.array([[1.0, 2.0, 0.5]]), np.array([0.7]))[0]
    row["qw"], row["qx"], row["qy"], row["qz"] = q
    row["scale"] = 1.7
    row["x"], row["y"], row["z"] = 3.0, -2.0, 0.25
    m = rocks_mesh(row, lib)
    pos, nrm, uv, idx = lib.lod_mesh(6, 1)
    R = quat_to_matrix(q[None])[0]
    np.testing.assert_allclose(m.vertices, pos.astype(np.float64) @ R.T * 1.7 + [3.0, -2.0, 0.25], atol=1e-6)
    np.testing.assert_allclose(m.normals, nrm.astype(np.float64) @ R.T, atol=1e-6)
    np.testing.assert_array_equal(m.uvs, uv)
    np.testing.assert_array_equal(m.triangles, idx.astype(np.int64))
    assert rocks_mesh(empty_catalog(0), lib).n_vertices == 0
    assert rocks_mesh(row, lib, lod_override=2).n_triangles == lib.prototypes[6].lod_tris[2]


# ---------------------------------------------------------------------------------------------
# collision field
# ---------------------------------------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1):
    V = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], dtype=np.float64)
    F = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                  [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])
    return V, F


def test_raster_box_top_is_exact():
    V, F = _box(0.105, 0.333, -0.21, 0.04, -0.05, 0.123)
    tris = V[F]
    g = rasterize_tops(tris, -0.5, -0.5, 0.02, 51, 51)
    xs = -0.5 + 0.02 * np.arange(51)
    X, Y = np.meshgrid(xs, xs)
    e = 1e-9                                  # nodes on an edge (y = 0.04) count as covered
    inside = (X >= 0.105 - e) & (X <= 0.333 + e) & (Y >= -0.21 - e) & (Y <= 0.04 + e)
    assert inside.sum() == 11 * 13
    assert np.all(np.isneginf(g[~inside]))
    np.testing.assert_allclose(g[inside], 0.123, atol=1e-12)


def test_raster_tilted_hull_top_is_plane():
    # a wedge: the top face is the plane z = 0.1 + 0.2 x - 0.1 y over the unit square
    P = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
    top = np.column_stack([P, 0.1 + 0.2 * P[:, 0] - 0.1 * P[:, 1]])
    bot = np.column_stack([P, -0.3 * np.ones(4)])
    V = np.vstack([bot, top])
    _, F = _box(0, 1, 0, 1, 0, 1)
    tris = V[F]
    g = rasterize_tops(tris, 0.0, 0.0, 0.05, 21, 21)
    xs = 0.05 * np.arange(21)
    X, Y = np.meshgrid(xs, xs)
    np.testing.assert_allclose(g, 0.1 + 0.2 * X - 0.1 * Y, atol=1e-12)


def _brute_top(tris, X, Y):
    """Independent numpy reference: max z of all triangles covering each point."""
    out = np.full(X.shape, -np.inf)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    ok = np.abs(area) > 1e-14
    a, b, c, area = a[ok], b[ok], c[ok], area[ok]
    for idx in np.ndindex(X.shape):
        x, y = X[idx], Y[idx]
        w0 = ((b[:, 0] - x) * (c[:, 1] - y) - (c[:, 0] - x) * (b[:, 1] - y)) / area
        w1 = ((c[:, 0] - x) * (a[:, 1] - y) - (a[:, 0] - x) * (c[:, 1] - y)) / area
        w2 = 1 - w0 - w1
        cov = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
        if cov.any():
            out[idx] = np.max((w0 * a[:, 2] + w1 * b[:, 2] + w2 * c[:, 2])[cov])
    return out


def test_core_collision_grid(cfg, lib, core_sample):
    cat, _ = core_sample
    sp = 0.02
    xs = np.arange(-1.0, 1.0 + 1e-9, sp)
    X, Y = np.meshgrid(xs, xs)
    H = rough_height(X, Y)
    ground = Grid(H, -1.0, -1.0, sp)
    # pick the largest rock inside the window, move it to the window centre
    rows = cat[cat["collide"]].copy()
    big = rows[np.argsort(-rows["D"])][:1].copy()
    big["x"], big["y"] = 0.13, -0.07
    big["z"] = 0.0
    C = core_collision_grid(cfg, ground, big, lib)
    from scipy import ndimage

    sigma = lowpass_sigma_m(0.25)
    assert abs(sigma - 0.0469) < 2e-4
    base = ndimage.gaussian_filter(H, sigma / sp, mode="nearest") - 0.02
    tris, _ = transformed_hulls(big, lib)
    top = _brute_top(tris, X, Y)
    want = np.where(np.isfinite(top) & (top - 0.01 > base), top - 0.01, base)
    np.testing.assert_allclose(C.z, want, atol=1e-9)
    assert C.meta["n_nodes_raised"] > 0
    # no rocks: pure low-passed ground with the offset
    C0 = core_collision_grid(cfg, ground, empty_catalog(0), lib)
    np.testing.assert_allclose(C0.z, base, atol=1e-12)


def test_ring_collision_grid(cfg, lib):
    from lunar_world_gen.rocks.collision_field import ring_rock_dilation

    cat = sample_rocks(cfg, "ring", flat_ground(), library=lib, rect=(-100, 100, -100, 100))
    g = Grid(np.zeros((801, 801)), -100.0, -100.0, 0.25)
    C = ring_collision_grid(cfg, g, cat, lib)                    # default: FID-08 footprint dilation
    C0 = ring_collision_grid(cfg, g, cat, lib, dilation=0.0)     # point samples (the pre-FID-08 rule)
    for G in (C, C0):
        assert np.isclose(np.min(G.z), -0.02)
        assert G.meta["n_rocks"] == int(np.sum(cat["D"] >= 0.3))
    big = cat[cat["D"] >= 0.3]                          # core-edge band rocks (< 30 cm) are not rasterised
    tree = cKDTree(np.column_stack([big["x"], big["y"]]))

    def reach_excess(G, w):
        """per raised node: min over the nearest rocks of distance - (0.6 D + sqrt(2) w)"""
        raised = np.argwhere(G.z > -0.02 + 1e-9)
        pts = np.column_stack([-100 + 0.25 * raised[:, 1], -100 + 0.25 * raised[:, 0]])
        d, j = tree.query(pts, k=min(8, len(big)))
        return raised, np.min(d - (0.6 * big["D"][j] + np.sqrt(2.0) * w), axis=1)

    # point samples: every raised node lies within the footprint of some rock
    r0, e0 = reach_excess(C0, 0.0)
    assert len(r0) > 0 and np.all(e0 <= 1e-9)
    # dilation: every raised node lies within w (one node spacing, L-inf) of a footprint; never lower
    # than the point samples, and more nodes raised
    w = ring_rock_dilation(cfg) * 0.25
    assert w == pytest.approx(0.25)
    r1, e1 = reach_excess(C, w)
    assert np.all(e1 <= 1e-9)
    assert np.all(C.z >= C0.z) and len(r1) > 2 * len(r0)


# ---------------------------------------------------------------------------------------------
# fillets
# ---------------------------------------------------------------------------------------------
def test_fillets_continuity_and_bounds(cfg, lib, core_sample):
    cat, _ = core_sample
    sp = 0.01
    n = 601
    full0 = Grid(np.zeros((n, n)), -3.0, -3.0, sp)
    H0 = Grid(rough_height(*full0.mesh_xy()), -3.0, -3.0, sp)        # the ground the rocks stand on
    F = fillet_field(cfg, H0, cat, lib)
    p = fillet_params(cfg, cat, lib)
    near = (np.abs(p["x"]) < 3.5) & (np.abs(p["y"]) < 3.5)
    assert near.sum() > 3, "test window should contain filleted rocks"
    assert F.min() >= 0.0 and F.max() > 0.0
    # heights bounded by 15 % of the exposed height
    rows = cat[(cat["zone"] == 0) & (cat["D"] >= 0.10) & (cat["burial"] > 0.06)]
    assert F.max() <= 0.15 * rows["exposed_h"].max() + 1e-12
    np.testing.assert_array_less(p["h"], 0.15 * cat["exposed_h"].max() + 1e-12)
    # only rocks >= 10 cm buried deeper than the perched compliance (min_burial = 0.06)
    assert len(p["x"]) == len(rows)
    # tiles: two overlapping sub-grids give exactly the same values, except within the gradient
    # stencil (8 nodes) of a sub-grid's own cut edge
    k = 8
    left = Grid(H0.z[:, :301].copy(), -3.0, -3.0, sp)
    right = Grid(H0.z[:, 300:].copy(), 0.0, -3.0, sp)
    Fl = fillet_field(cfg, left, cat, lib)
    Fr = fillet_field(cfg, right, cat, lib)
    np.testing.assert_array_equal(Fl[k:-k, :301 - k], F[k:-k, :301 - k])
    np.testing.assert_array_equal(Fr[k:-k, k:], F[k:-k, 300 + k:])
    # continuous wedge: no step between neighbouring nodes larger than the ramp allows (+ changes of
    # the local ground gradient)
    gy, gx = np.gradient(F, sp)
    gmax = np.max(1.5 * p["h"][near] / p["w"][near])
    assert np.hypot(gx, gy).max() <= 3.0 * gmax + 1e-9
    # apply_fillets adds the field and keeps the frame
    H = apply_fillets(cfg, H0, cat, lib)
    np.testing.assert_allclose(H.z - H0.z, F, atol=1e-12)
    assert (H.x0, H.y0, H.spacing) == (H0.x0, H0.y0, H0.spacing)


def test_fillets_disabled(cfg, lib, core_sample):
    cat, _ = core_sample
    cfg_off = load_config(overrides=["terrain.fillets.enabled=false"])
    g = Grid(np.zeros((101, 101)), -0.5, -0.5, 0.01)
    assert np.all(fillet_field(cfg_off, g, cat, lib) == 0)


# ---------------------------------------------------------------------------------------------
# catalog
# ---------------------------------------------------------------------------------------------
def test_catalog_io_and_validation(tmp_path, cfg, lib, core_sample):
    cat, meta = core_sample
    paths = save_catalog(cat, tmp_path / "rocks_catalog", meta=meta)
    assert [p.suffix for p in paths] == [".npz", ".csv"]
    back = load_catalog(tmp_path / "rocks_catalog.npz")
    assert back.dtype == CATALOG_DTYPE and back.tobytes() == cat.tobytes()
    with np.load(tmp_path / "rocks_catalog.npz") as d:          # column-wise: readable without this module
        assert {"x", "y", "z", "D", "cls", "proto", "collide", "meta_json"} <= set(d.files)
        np.testing.assert_array_equal(d["D"], cat["D"])
    m = load_catalog_meta(tmp_path / "rocks_catalog.npz")
    assert m["n"] == len(cat) and m["zone"] == "core"
    csv = load_catalog(tmp_path / "rocks_catalog.csv")
    assert len(csv) == len(cat)
    np.testing.assert_array_equal(csv["id"], cat["id"])
    np.testing.assert_allclose(csv["x"], cat["x"], atol=1e-6)
    np.testing.assert_allclose(csv["D"], cat["D"], rtol=1e-6)
    np.testing.assert_array_equal(csv["collide"], cat["collide"])
    assert validate_catalog(cat, lib, cfg) == []
    bad = cat.copy()
    bad["D"][0] *= 2
    bad["tile_i"][1] = (bad["tile_i"][1] + 1) % 10
    bad["qw"][2] = 2.0
    probs = validate_catalog(bad, lib, cfg)
    assert any("scale" in p for p in probs) and any("tile" in p for p in probs) and any("quaternion" in p for p in probs)
    tiles = split_by_tile(cat)
    assert sum(len(v) for v in tiles.values()) == len(cat)
    for (zone, ti, tj), idx in tiles.items():
        assert np.all(cat["tile_i"][idx] == ti) and np.all(cat["tile_j"][idx] == tj)


def test_poisson_interval():
    lo, hi = poisson_interval(0)
    assert lo == 0 and abs(hi - 3.689) < 1e-3
    lo, hi = poisson_interval(100)
    assert abs(lo - 81.36) < 0.05 and abs(hi - 121.63) < 0.05


# ---------------------------------------------------------------------------------------------
# production library (skipped until blender/rock_library.py has been run)
# ---------------------------------------------------------------------------------------------
def test_production_library(cfg):
    path = Path(cfg["paths"]["rock_library"])
    if not (path / "library.json").exists():
        pytest.skip(f"production library not built yet: {path}")
    lib = RockLibrary(path)
    counts = {c: len(lib.by_class(c)) for c in ("pebble", "medium", "large")}
    assert counts == {"pebble": 20, "medium": 20, "large": 10}
    budget = {"pebble": (12, 40), "medium": (200, 1000), "large": (2000, 5000)}
    for p in lib.prototypes:
        t0, t1, t2 = p.lod_tris
        lo, hi = budget[p.cls]
        assert lo <= t0 <= hi, (p.id, t0)
        assert t2 >= 12 and t1 >= t2
        if p.cls != "pebble":
            assert 0.2 <= t1 / t0 <= 0.4, (p.id, t1 / t0)
            assert 0.05 <= t2 / t0 <= 0.15 or t2 == 12, (p.id, t2 / t0)
        assert abs(p.a - p.dims[0]) < 1e-9 and p.dims[0] >= p.dims[1] and p.dims[0] >= p.dims[2]
        for lod in range(3):
            pos, nrm, uv, idx = lib.lod_mesh(p.index, lod)
            assert pos.dtype == np.float32 and nrm.dtype == np.float32 and uv.dtype == np.float32
            assert idx.dtype == np.uint32 and len(idx) == p.lod_tris[lod]
            np.testing.assert_allclose(np.linalg.norm(nrm, axis=1), 1.0, atol=1e-3)
            assert uv.min() >= 0 and uv.max() <= 1
            _, inv = np.unique(np.round(pos.astype(np.float64), 6), axis=0, return_inverse=True)
            assert np.all(_edge_counts(inv.ravel()[idx.astype(np.int64)]) == 2), (p.id, lod)
        hv = lib.hull(p.index)
        hf = lib.hull_faces(p.index)
        assert len(hv) <= 64
        c = hv.mean(axis=0)
        a, b, cc = hv[hf[:, 0]], hv[hf[:, 1]], hv[hf[:, 2]]
        nrm_f = np.cross(b - a, cc - a)
        nrm_f /= np.linalg.norm(nrm_f, axis=1, keepdims=True)
        # convex and outward: every vertex is behind every face plane
        dist = np.einsum("fj,fvj->fv", nrm_f, hv[None] - a[:, None])
        assert dist.max() <= 1e-6 * p.a
        assert np.all(np.einsum("ij,ij->i", nrm_f, a - c) > 0)
    for k in ("basecolor", "normal", "orm"):
        assert lib.atlas_paths[k].exists()
    assert lib.atlas_size in (2048, 4096)
