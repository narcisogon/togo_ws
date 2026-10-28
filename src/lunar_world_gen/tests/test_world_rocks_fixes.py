"""Regression tests for the phase-3 rock fixes (docs/impl/fix_rocks.md):

* FID-01  perched rocks rest on a stable hull facet, settled to three contacts on the real ground,
          bedded by a small compliance burial;
* FID-02  smooth, size-gated crater enhancement (no hard x2 annulus; blocks only around blocky craters);
* FID-04  core rock population continued into a raised-cosine band outside the core (rocks < 30 cm);
* FID-09  no rock centre inside a larger rock's footprint, no rock piercing a rock >= 5 cm;
* FID-11  fillets are wedges decaying outward from the rock's ground-level outline;
* FID-12  burial from the calibration's young-debris Beta(1.5, 6) when the class range is [0, 1].

Run from src/lunar_world_gen:  python -m pytest tests/test_world_rocks_fixes.py -q
"""
from __future__ import annotations

import numpy as np
import pytest
from scipy import ndimage

from lunar_world_gen.config import load_config, rng as cfg_rng
from lunar_world_gen.grid import Grid
from lunar_world_gen.rocks import (CATALOG_DTYPE, GroundFunction, RockLibrary, flat_ground, sample_rocks,
                                   validate_catalog, write_synthetic_library)
from lunar_world_gen.rocks.catalog import empty_catalog
from lunar_world_gen.rocks.fillets import fillet_field, fillet_params
from lunar_world_gen.rocks.placement import (burial_fractions, catalog_quats, place_rocks, quat_from_to,
                                             quat_to_matrix, settle_rocks, size_class)
from lunar_world_gen.rocks.population import (BAND_ID_OFFSET, crater_amplitudes, crater_factor, crater_profile,
                                              edge_band_weight, find_conflicts, fresh_craters)


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    path = tmp_path_factory.mktemp("rocklib_fix")
    return RockLibrary(write_synthetic_library(path, counts={"pebble": 5, "medium": 6, "large": 4}, seed=3))


def rough_height(x, y):
    """Analytic rough, sloping ground (slopes up to ~15 deg)."""
    return 0.05 * x + 0.03 * np.sin(2.1 * x) * np.cos(1.7 * y) + 0.008 * np.sin(9.0 * x + 4.0 * y)


def _vertices(row, lib):
    pos = lib.contact_points(int(row["proto"]), int(row["lod"]))
    R = quat_to_matrix(catalog_quats(row[None]))[0]
    return pos @ R.T * row["scale"] + np.array([row["x"], row["y"], row["z"]])


def _rows(fields, x, y, D):
    rows = empty_catalog(len(D))
    rows["x"], rows["y"], rows["D"] = x, y, D
    for k, v in fields.items():
        rows[k] = v
    return rows


# ---------------------------------------------------------------------------------------------- FID-01
def test_rest_faces_are_stable(lib):
    for p in lib.prototypes:
        f = lib.rest_faces(p.index)
        assert len(f["h"]) >= 1
        assert np.all(np.diff(f["h"]) >= 0) and np.all(f["h"] > 0)
        assert np.all(f["base_gap"] >= 0) and np.all(f["extent"] > f["h"])
        np.testing.assert_allclose(np.linalg.norm(f["normal"], axis=1), 1.0, atol=1e-9)
        # resting on a listed facet with a tipping margin on flat ground is an equilibrium: settling does
        # not rotate it (facets with ~0 deg margin balance on an edge; placement never picks them, < 3 deg)
        ok = f["tip_deg"] >= 1.0
        q = quat_from_to(f["normal"][ok], np.array([0.0, 0.0, -1.0]))
        n = len(q)
        D = np.full(n, p.a)
        qs, st = settle_rocks(flat_ground(), lib, np.zeros(n), np.zeros(n), np.ones(n), q,
                              np.full(n, p.index), np.zeros(n, np.int64))
        Rq, Rs = quat_to_matrix(q), quat_to_matrix(qs)
        ang = np.degrees(np.arccos(np.clip(np.einsum("kij,kij->k", Rq, Rs) * 0.5 - 0.5, -1, 1)))
        assert ang.max() < 0.5, (p.id, ang)
        del D


def test_perched_rocks_rest_on_three_contacts(lib):
    """FID-01: perched rocks on rough sloping ground touch it with >= 3 vertices, centre of mass over the
    support triangle (the old scheme balanced them on one vertex with a random 0-15 deg tilt)."""
    cfg = load_config(overrides=["rocks.perched_fraction=1.0", "rocks.perched_burial=[0.0,0.0]",
                                 "rocks.bedding.max_frac=0.0"])
    ground = GroundFunction(rough_height)
    rng = np.random.default_rng(4)
    n = 300
    x, y = rng.uniform(-4, 4, n), rng.uniform(-4, 4, n)
    D = np.exp(rng.uniform(np.log(0.03), np.log(0.8), n))
    f = place_rocks(cfg, cfg_rng(cfg, "test.fid01"), x, y, D, "core", ground, lib)
    rows = _rows(f, x, y, D)
    n3 = 0
    for r in rows:
        V = _vertices(r, lib)
        c = V[:, 2] - rough_height(V[:, 0], V[:, 1])
        assert abs(c.min()) < 1e-9                                     # touches, does not float
        tol = 5e-4 * r["D"] + 1e-7
        touch = V[c <= tol]
        if len(touch) >= 3:
            n3 += 1
            # centre of mass (rock origin) over the convex hull of the contacts (static equilibrium)
            from scipy.spatial import ConvexHull, QhullError
            try:
                hull = ConvexHull(touch[:, :2])
                eq = hull.equations
                assert np.all(eq[:, :2] @ np.array([r["x"], r["y"]]) + eq[:, 2] <= 1e-3 * r["D"])
            except QhullError:
                pass
    assert n3 >= 0.9 * n, n3
    # the old scheme (authored base, 15 deg random tilt, no settling) rests on fewer contacts
    old = load_config(overrides=["rocks.perched_fraction=1.0", "rocks.perched_burial=[0.0,0.0]",
                                 "rocks.settle.enabled=false", "rocks.rest_face.mode=authored",
                                 "rocks.perched_tilt_max_deg=15", "rocks.bedding.max_frac=0.0"])
    fo = place_rocks(old, cfg_rng(old, "test.fid01"), x, y, D, "core", ground, lib)
    n3_old = sum(int(np.sum(_c(r, lib) <= 5e-4 * r["D"] + 1e-7) >= 3) for r in _rows(fo, x, y, D))
    assert n3_old < 0.5 * n3


def _c(r, lib):
    V = _vertices(r, lib)
    return V[:, 2] - rough_height(V[:, 0], V[:, 1])


def test_perched_compliance_and_buried_ranges(cfg, lib):
    rng = np.random.default_rng(5)
    n = 4000
    cls = size_class(cfg, np.exp(rng.uniform(np.log(0.02), np.log(1.0), n)))
    f, perched = burial_fractions(cfg, cfg_rng(cfg, "test.burial"), cls, return_perched=True)
    assert abs(perched.mean() - cfg["rocks"]["perched_fraction"]) < 0.03
    assert np.all((f[perched] >= 0.02) & (f[perched] <= 0.06))            # bedded, never "not in contact"
    lo = np.array([c["burial"][0] for c in cfg["rocks"]["classes"]])[cls]
    hi = np.array([c["burial"][1] for c in cfg["rocks"]["classes"]])[cls]
    assert np.all((f[~perched] >= lo[~perched]) & (f[~perched] <= hi[~perched]))


def test_burial_calibration_young_debris():
    """FID-12: class range [0, 1] gives Beta(1.5, 6) (mean 0.2); with 30 % perched the overall median is
    ~0.1, the calibration's 'young South Ray debris: 0-20 %, median ~10 %, many perched'."""
    classes = ("[{name: pebble, D_max_m: 0.05, library_class: pebble, lod: 0, burial: [0.0, 1.0]},"
               "{name: small, D_max_m: 0.12, library_class: medium, lod: 2, burial: [0.0, 1.0]},"
               "{name: medium, D_max_m: 0.30, library_class: medium, lod: 1, burial: [0.0, 1.0]},"
               "{name: large, D_max_m: 50.0, library_class: large, lod: 0, burial: [0.0, 1.0]}]")
    cfg = load_config(overrides=[f"rocks.classes={classes}"])
    cls = np.zeros(200_000, dtype=np.int64)
    f, perched = burial_fractions(cfg, cfg_rng(cfg, "test.fid12"), cls, return_perched=True)
    b = f[~perched]
    assert abs(b.mean() - 0.2) < 0.005 and abs(np.median(b) - 0.174) < 0.01
    assert 0.09 < np.median(f) < 0.13
    assert np.mean(f <= 0.2) > 0.6


def test_bedding_only_for_perched_rocks(lib):
    """FID-01/FID-12: the bedding floor applies to perched rocks only, so the buried rocks keep the
    calibrated Beta(1.5, 6) exactly (no pile-up at the bedding cap)."""
    from scipy.stats import kstest

    classes = ("[{name: pebble, D_max_m: 0.05, library_class: pebble, lod: 0, burial: [0.0, 1.0]},"
               "{name: small, D_max_m: 0.12, library_class: medium, lod: 2, burial: [0.0, 1.0]},"
               "{name: medium, D_max_m: 0.30, library_class: medium, lod: 1, burial: [0.0, 1.0]},"
               "{name: large, D_max_m: 50.0, library_class: large, lod: 0, burial: [0.0, 1.0]}]")
    cfg = load_config(overrides=[f"rocks.classes={classes}"])
    rng = np.random.default_rng(8)
    n = 3000
    x, y = rng.uniform(-10, 10, n), rng.uniform(-10, 10, n)
    D = np.exp(rng.uniform(np.log(0.02), np.log(0.3), n))
    st: dict = {}
    f = place_rocks(cfg, cfg_rng(cfg, "test.bedding"), x, y, D, "core", flat_ground(), lib, stats=st)
    per = st["perched_mask"]
    b = f["burial"][~per]
    assert kstest(b, "beta", args=(1.5, 6.0)).pvalue > 1e-3
    from lunar_world_gen.rocks.placement import BEDDING

    cap = BEDDING["max_frac"]
    assert np.all(f["burial"][per] >= 0.02 - 1e-9) and np.all(f["burial"][per] <= cap + 1e-9)
    assert st["n_bedded"] > 0                         # the synthetic pebbles have keels


# ---------------------------------------------------------------------------------------------- FID-02
def test_crater_profile_smooth_and_size_gated(cfg):
    t = np.linspace(0.0, 3.0, 30001)
    p = crater_profile(t, 2.5, 0.5)
    assert abs(p[10000] - 1.0) < 1e-12 and p.max() <= 1.0 + 1e-12          # peak 1 at the rim
    assert abs(p[0] - 0.5) < 1e-12 and np.all(p[t >= 2.5] == 0.0)
    assert np.abs(np.diff(p)).max() < 5e-4                                   # continuous (no step)
    craters = {"x": np.array([0.0, 100.0, 200.0]), "y": np.zeros(3), "D": np.array([12.0, 50.0, 90.0]),
               "rho": np.array([1.0, 1.0, 0.8])}
    fc = fresh_craters(cfg, craters, include_dtm=False)
    small = crater_amplitudes(cfg, fc, "small")
    big = crater_amplitudes(cfg, fc, "big")
    np.testing.assert_allclose(small, [0.5, 0.5, 0.4])                      # mild debris boost, all fresh
    factor = cfg["rocks"]["crater_enhancement"]["factor"]
    np.testing.assert_allclose(big, [0.0, (factor - 1) * 0.5, (factor - 1) * 0.8])   # blocks: D >= 20 m ramp
    e_rim = crater_factor(cfg, np.array([6.0, 125.0]), np.zeros(2), fc, "big")
    np.testing.assert_allclose(e_rim, [1.0, 1.0 + (factor - 1) * 0.5])


# ---------------------------------------------------------------------------------------------- FID-09
def test_no_interpenetration_after_sampling(cfg, lib):
    ground = GroundFunction(rough_height)
    cat = sample_rocks(cfg, "core", ground, library=lib, rect=(-6, 6, -6, 6), include_dtm_craters=False)
    cols = ("x", "y", "z", "D", "scale", "proto", "lod", "qw", "qx", "qy", "qz")
    move, st = find_conflicts(cfg, {c: cat[c] for c in cols}, lib)
    assert st.get("conflicts", 0) == 0 and not move.any(), st
    off = load_config(overrides=["rocks.overlap.enabled=false"])
    cat0 = sample_rocks(off, "core", ground, library=lib, rect=(-6, 6, -6, 6), include_dtm_craters=False)
    _, st0 = find_conflicts(cfg, {c: cat0[c] for c in cols}, lib)
    assert st0["conflicts"] > 0                                              # the test can see conflicts
    assert validate_catalog(cat, lib, cfg) == []


def test_ring_avoids_core_rocks(cfg, lib):
    core = sample_rocks(cfg, "core", flat_ground(), library=lib, rect=(44, 50, -6, 6), include_dtm_craters=False)
    ring = sample_rocks(cfg, "ring", flat_ground(), library=lib, rect=(50, 56, -6, 6), include_dtm_craters=False,
                        avoid=core)
    cols = ("x", "y", "z", "D", "scale", "proto", "lod", "qw", "qx", "qy", "qz")
    move, st = find_conflicts(cfg, {c: ring[c] for c in cols}, lib, fixed=core)
    assert not move.any(), st


# ---------------------------------------------------------------------------------------------- FID-04
def test_core_edge_band(cfg, lib):
    from lunar_world_gen.rocks.population import EDGE_BAND

    W = float((cfg["rocks"].get("edge_band") or {}).get("width_m", EDGE_BAND["width_m"]))
    cat, meta = sample_rocks(cfg, "ring", flat_ground(), library=lib, rect=(50, 62, -20, 20),
                             include_dtm_craters=False, return_meta=True)
    band = cat[cat["id"] >= BAND_ID_OFFSET]
    main = cat[cat["id"] < BAND_ID_OFFSET]
    assert np.all(main["D"] >= 0.30) and len(band) > 1000
    assert np.all(band["D"] < 0.30) and band["D"].min() >= 0.02
    assert np.all(band["zone"] == 1) and not band["collide"].any()
    d = band["x"] - 50.0
    assert d.min() >= 0.0 and d.max() <= W
    # count within 4 sigma of the expectation
    eb = meta["edge_band"]
    assert abs(len(band) - eb["expected_total"]) < 4.0 * np.sqrt(eb["expected_total"]) + 5
    # density follows the raised cosine: compare 1 m bins with the weight, normalised at the edge
    bins = np.arange(0.0, W + 1e-9, 1.0)
    cnt, _ = np.histogram(d, bins=bins)
    w = edge_band_weight(cfg, 50.0 + 0.5 * (bins[1:] + bins[:-1]), np.zeros(len(bins) - 1), W)
    expect = cnt.sum() * w / w.sum()
    assert np.all(np.abs(cnt - expect) < 5.0 * np.sqrt(expect) + 10), (cnt, expect)
    assert cnt[0] > 5 * cnt[-1]
    # LODs: core class LOD with the band override (pebbles LOD1 by default)
    peb = band[band["D"] < 0.05]
    assert np.all(peb["lod"] == 1)
    assert validate_catalog(cat, lib, cfg) == []
    off = load_config(overrides=["rocks.edge_band.enabled=false"])
    cat0 = sample_rocks(off, "ring", flat_ground(), library=lib, rect=(50, 62, -20, 20), include_dtm_craters=False)
    assert np.all(cat0["D"] >= 0.30)


# ---------------------------------------------------------------------------------------------- FID-11
def test_fillet_is_a_wedge_on_a_slope(cfg, lib):
    """A buried boulder on an 11 deg slope: the fillet is h_f where the rock crosses the ground, decays
    outward with the distance from that ground-level outline, is 0 beyond w_f and has no plateau
    beyond the outline (the old ellipse made a flat apron on the downhill side)."""
    slope = 0.2
    p = 12                                                              # a large synthetic prototype
    proto = lib.prototypes[p]
    row = empty_catalog(1)
    row["id"], row["zone"], row["proto"], row["lod"] = 7, 0, p, 0
    row["scale"] = 0.9 / proto.a
    row["D"] = 0.9
    q = quat_from_to(np.array([0.2, 0.1, 1.0]), np.array([0.0, 0.0, 1.0]))[0]
    row["qw"], row["qx"], row["qy"], row["qz"] = q
    V = (lib.hull(p) @ quat_to_matrix(q[None])[0].T) * row["scale"][0]
    row["x"], row["y"] = 0.0, 0.0
    hv = V[:, 2].max() - V[:, 2].min()
    ground = lambda x, y: slope * x                                     # noqa: E731
    # sink 30 % below first contact with the sloped plane
    row["z"] = np.max(ground(V[:, 0], V[:, 1]) - V[:, 2]) - 0.3 * hv
    row["burial"] = 0.3
    row["exposed_h"] = row["z"][0] + V[:, 2].max()
    sp = 0.01
    n = 301
    xs = -1.5 + sp * np.arange(n)
    X, Y = np.meshgrid(xs, xs)
    G = Grid(slope * X, -1.5, -1.5, sp)
    F = fillet_field(cfg, G, row, lib)
    prm = fillet_params(cfg, row, lib)
    h, w = float(prm["h"][0]), float(prm["w"][0])
    assert h > 0.005 and F.max() <= h + 1e-12
    # the ground-level outline: nodes where the hull contains (x, y, ground)
    n_l, o_l = lib.hull_planes(p)
    nw = n_l @ quat_to_matrix(q[None])[0].T
    ow = row["scale"][0] * o_l + nw @ np.array([0.0, 0.0, row["z"][0]])
    P = np.stack([X, Y, slope * X], axis=-1)
    inside = np.all(P @ nw.T <= ow + 1e-12, axis=-1)
    d = ndimage.distance_transform_edt(~inside) * sp
    t = np.clip(d / w, 0, 1)
    ideal = h * (1 - t * t * (3 - 2 * t))
    assert np.all(F[d > w + 0.02] == 0.0)                                # nothing beyond w_f
    assert np.all(F <= ideal + 0.2 * h + 1e-9)                           # never a plateau beyond the wedge
    ring = (d > 0) & (d <= 0.02)
    assert np.median(F[ring]) > 0.8 * h                                  # full height at the contact line
    gy, gx = np.gradient(F, sp)
    assert np.hypot(gx, gy).max() < 3.0 * 1.5 * h / w                    # continuous ramp, no step


def test_fillet_skips_perched_rocks(cfg, lib):
    cat = sample_rocks(cfg, "core", GroundFunction(rough_height), library=lib, rect=(-4, 4, -4, 4))
    prm = fillet_params(cfg, cat, lib)
    sel = (cat["D"] >= 0.10) & (cat["burial"] > 0.06)
    assert len(prm["x"]) == int(sel.sum())


# ---------------------------------------------------------------------------------------------- format
def test_catalog_format_unchanged():
    names = ("id", "zone", "tile_i", "tile_j", "proto", "cls", "lod", "x", "y", "z", "qw", "qx", "qy", "qz",
             "scale", "D", "exposed_h", "burial", "collide")
    assert CATALOG_DTYPE.names == names
    assert CATALOG_DTYPE["id"] == np.dtype("<i8") and CATALOG_DTYPE["collide"] == np.dtype("?")
