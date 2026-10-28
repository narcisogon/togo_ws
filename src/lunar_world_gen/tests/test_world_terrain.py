"""Tests for lunar_world_gen.terrain (base, fbm, craters, compose, calibrate).

Run (WSL, project venv, from src/lunar_world_gen):
    ~/.venvs/lunar_world/bin/python -m pytest tests/test_world_terrain.py -q

Tests marked ``dtm`` use the real Apollo 16 DTM through tools/dtm_extract.py on a SMALL test layout
(core 4 m with 2 m tiles, ring 40 m, far 200 m, horizon 2 km) and are skipped if the DTM is missing.
"""
from __future__ import annotations

import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy import ndimage

from lunar_world_gen.config import DEFAULT_CONFIG, load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.layout import Layout, TileId
from lunar_world_gen.terrain import base as tbase
from lunar_world_gen.terrain import calibrate as tcal
from lunar_world_gen.terrain import compose as tcomp
from lunar_world_gen.terrain import craters as tcr
from lunar_world_gen.terrain import fbm as tfbm
from lunar_world_gen.terrain import talus as ttal

SMALL = [
    "zones.core.size_m=4.0", "zones.core.tile_size_m=2.0", "zones.core.margin_m=0.5",
    "zones.core.edge_fade_m=0.5", "zones.ring.size_m=40.0", "zones.ring.edge_fade_m=4.0",
    "zones.far.size_m=200.0", "zones.horizon.size_m=2000.0",
    "terrain.craters.D_max_m=4.0", "terrain.craters.fade_D_m=[2.0, 4.0]",
]


def small_cfg(build_dir: Path, *extra: str):
    return load_config(overrides=[f"paths.build_dir={build_dir}", *SMALL, *extra])


_cfg0 = load_config()
HAVE_DTM = Path(_cfg0.paths.dtm).exists()
dtm = pytest.mark.skipif(not HAVE_DTM, reason="NAC_DTM_APOLLO16.TIF not available")


@pytest.fixture(scope="session")
def built(tmp_path_factory):
    """Small layout built once with the real DTM (all stages)."""
    if not HAVE_DTM:
        pytest.skip("DTM not available")
    d = tmp_path_factory.mktemp("terrain_small")
    cfg = small_cfg(d)
    tcomp.build_all(cfg, log=lambda *a: None)
    return cfg


# ---------------------------------------------------------------------------------------- craters
@pytest.mark.parametrize("D,rho", [(0.2, 1.0), (0.2, math.exp(-3)), (1.0, 0.5), (4.0, 1.0), (12.0, 0.08)])
def test_crater_profile_continuity(D, rho):
    R = 0.5 * D * (1 + tcr.DEGRADED_WIDENING * (1 - rho))
    d, h = rho * 0.13 * D, rho * 0.03 * D
    hp = float(tcr.solve_rim_parameter(np.array([d]), np.array([h]), np.array([1 + rho]))[0])
    # C0 everywhere: dense radial sampling, steps bounded by the maximum wall slope
    r = np.linspace(0.0, 3.0 * R, 300_001)
    z = tcr.crater_profile(r, R, d, h, rho)
    dr = r[1] - r[0]
    max_slope = (d + hp) * (1 + rho) / R * 1.01         # steepest interior wall (at r -> R)
    assert np.abs(np.diff(z)).max() <= max_slope * dr + 1e-12
    # no step > 0.1 mm across the rim (r = R) and the ejecta edge (r = 2.5 R)
    for rb in (R, 2.5 * R):
        e = 1e-9 * R
        zz = tcr.crater_profile(np.array([rb - e, rb + e]), R, d, h, rho)
        assert abs(zz[1] - zz[0]) < 1e-4
    # shape anchors: floor at -d, rounded crest exactly h just outside R (FID-05), exactly 0 beyond 2.5 R
    zc = tcr.crater_profile(np.array([0.0, 2.5 * R, 2.6 * R]), R, d, h, rho)
    assert zc[0] == pytest.approx(-d, abs=1e-15)
    assert zc[1] == 0.0 and zc[2] == 0.0
    k = int(np.argmax(z))
    assert z[k] == pytest.approx(h, rel=1e-6, abs=1e-9)
    assert R <= r[k] <= 1.1 * R
    assert 1.0 < hp / h < 1.3


def test_rim_crest_matches_rim_height_fid05():
    """FID-05: the rounded crest of every sampled crater equals its rim height h (was 0.56-0.69 h)."""
    cfg = load_config()
    cs = tcr.sample_craters(cfg, "small")
    pick = np.random.default_rng(0).choice(len(cs), 200, replace=False)
    crest = np.array([tcr.crest_height(cs.d[k], cs.hp[k], cs.n[k], cs.k_over_h) for k in pick])
    assert np.abs(crest / cs.h[pick] - 1).max() < 1e-6
    fresh = cs.rho >= 0.9
    assert np.median(cs.h[fresh] / cs.D[fresh]) >= 0.027               # review acceptance (median, rho >= 0.9)
    # dense check of the profile max for a fresh 10 m crater: crest/D = 0.03
    D, rho = 10.0, 1.0
    R = 0.5 * D
    r = np.linspace(0.9 * R, 1.3 * R, 400_001)
    z = tcr.crater_profile(r, R, 0.13 * D, 0.03 * D, rho)
    assert z.max() / D == pytest.approx(0.03, rel=1e-6)


def test_crater_shape_variation_fid16(tmp_path):
    """FID-16: per-crater ellipticity, rim modulation and profile variation within the calibration ranges;
    deterministic; the population draws are unchanged by the shape stream."""
    cfg = load_config()
    a = tcr.sample_craters(cfg, "small")
    b = tcr.sample_craters(cfg, "small")
    for f in tcr._FIELDS + tcr._SHAPE_FIELDS:
        assert np.array_equal(getattr(a, f), getattr(b, f)), f
    flat = load_config(overrides=["terrain.craters.shape={d_spread: 0, rim_spread: 0, n_spread: 0, "
                                  "axis_ratio_min: 1, rim_mod_a2_max: 0, rim_mod_a3_max: 0}"])
    c = tcr.sample_craters(flat, "small")
    for f in ("x", "y", "D", "rho", "R", "id"):
        assert np.array_equal(getattr(a, f), getattr(c, f)), f            # same population
    assert np.all(c.axis == 1.0) and np.all(c.a2 == 0) and np.all(c.a3 == 0)
    assert np.allclose(c.d, c.rho * 0.13 * c.D, rtol=1e-15) and np.allclose(c.h, c.rho * 0.03 * c.D, rtol=1e-15)
    # ranges (calibration fresh_A: d/D 0.11-0.17, rim 0.022-0.04 D)
    dd = a.d / (a.rho * a.D)
    hh = a.h / (a.rho * a.D)
    assert 0.13 * 0.85 - 1e-12 <= dd.min() and dd.max() <= 0.13 * 1.15 + 1e-12
    assert 0.03 * 0.8 - 1e-12 <= hh.min() and hh.max() <= 0.03 * 1.2 + 1e-12
    assert 0.85 <= a.axis.min() and a.axis.max() <= 1.0 and a.axis.std() > 0.03
    assert a.a2.max() <= 0.2 and a.a3.max() <= 0.1
    # an isolated fresh crater: elliptical outline, azimuthally varying crest whose mean is h
    k = int(np.argmax(np.where(a.rho > 0.9, a.D * (1 - a.axis) * (a.a2 + 0.01), 0)))
    one = a.subset(np.arange(len(a)) == k)
    th = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    tt = np.linspace(0.85, 1.4, 2201)
    Ra, Rb = one.R[0] / math.sqrt(one.axis[0]), one.R[0] * math.sqrt(one.axis[0])
    u, v = tt[None, :] * Ra * np.cos(th)[:, None], tt[None, :] * Rb * np.sin(th)[:, None]
    cps, sps = math.cos(one.theta[0]), math.sin(one.theta[0])
    zc = tcr.crater_relief(one, one.x[0] + u * cps - v * sps, one.y[0] + u * sps + v * cps).max(axis=1)
    assert zc.mean() == pytest.approx(one.h[0], rel=0.01)
    assert zc.std() / zc.mean() > 0.03
    # the crest lies on an ellipse: its distance from the centre varies with the axis ratio
    rr = np.hypot(u, v)[np.arange(th.size), np.argmax(tcr.crater_relief(
        one, one.x[0] + u * cps - v * sps, one.y[0] + u * sps + v * cps), axis=1)]
    assert rr.max() / rr.min() == pytest.approx(1.0 / one.axis[0], rel=0.03)
    # loading a version-1 file (no shape fields) reproduces the old circular k = 2h profile
    path = tmp_path / "v1.npz"
    np.savez(path, **{f: getattr(c, f) for f in tcr._FIELDS}, which=np.array("small"),
             ejecta_extent_R=np.array(2.5), ejecta_exponent=np.array(-3.0), meta_json=np.array("{}"))
    old = tcr.CraterSet.load(path)
    assert old.k_over_h == 2.0 and np.array_equal(old.hp, old.h) and np.all(old.axis == 1)


def test_crater_relief_superposition_and_grid_identity():
    cfg = load_config(overrides=["terrain.craters.D_min_m={core: 0.2, ring: 1.0}"])
    cs = tcr.sample_craters(cfg, "small")
    sub = cs.subset((np.abs(cs.x) < 3) & (np.abs(cs.y) < 3))
    assert len(sub) > 20
    g = Grid(np.zeros((201, 201), np.float32), -2.0, -2.0, 0.02)
    kg = tcr.crater_relief_grid(sub, g)
    xs, ys = tcr.node_coordinates(g)
    X, Y = np.meshgrid(xs, ys)
    kp = tcr.crater_relief(sub, X, Y)
    assert np.array_equal(kg, kp)                         # grid and point paths are bit-identical
    brute = np.zeros_like(X)
    for j in range(len(sub)):
        brute += tcr.crater_relief(sub.subset(np.arange(len(sub)) == j), X, Y)
    assert np.abs(brute - kp).max() < 1e-12                # pure superposition
    # a circular, unmodulated crater equals the radial profile
    circ = sub.subset(np.arange(len(sub)) == 0)
    circ.axis[:] = 1.0
    circ.a2[:] = 0.0
    circ.a3[:] = 0.0
    circ._index = None
    r = np.hypot(X - circ.x[0], Y - circ.y[0])
    prof = tcr.crater_profile(r.ravel(), circ.R[0], circ.d[0], circ.h[0], circ.rho[0], hp=circ.hp[0],
                              n=circ.n[0]).reshape(X.shape)
    assert np.abs(tcr.crater_relief(circ, X, Y) - prof).max() < 1e-12


def test_crater_sfd_matches_target():
    rng = np.random.default_rng(123)
    C, b, D_lo, D_hi, fade = 0.05, 2.0, 1.0, 16.0, (8.0, 16.0)
    rect = (-1000.0, 1000.0, -1000.0, 1000.0)
    pop = tcr.sample_population(rng, C, b, D_lo, D_hi, rect, fade, 3.0)
    area = 2000.0 * 2000.0
    Ds = np.sort(pop["D"])
    for D in (1.0, 1.5, 2.0, 3.0, 5.0):
        n = Ds.size - np.searchsorted(Ds, D)
        exp = tcr.expected_count_ge(D, C, b, D_lo, D_hi, fade, area)
        assert abs(n / exp - 1) < max(0.03, 4 / math.sqrt(exp)), (D, n, exp)
    # below the fade the density follows the pure power law C D^-b (minus the truncation)
    n2 = Ds.size - np.searchsorted(Ds, 2.0)
    pure = C * (2.0 ** -b - D_hi ** -b) * area
    assert abs(n2 / pure - 1) < 0.03
    # fade: nothing at/above 16 m, linear thinning in 8-16 m (expected kept fraction ~0.667 for b = 2)
    assert Ds.max() < 16.0
    n8 = Ds.size - np.searchsorted(Ds, 8.0)
    exp8 = tcr.expected_count_ge(8.0, C, b, D_lo, D_hi, fade, area)
    assert abs(n8 / exp8 - 1) < 4 / math.sqrt(exp8)
    assert n8 < 0.75 * C * (8.0 ** -b - 16.0 ** -b) * area
    # freshness classes: ~10 % rho > 0.73, ~20 % 0.4-0.73, ~70 % < 0.4
    rho = pop["rho"]
    assert abs(np.mean(rho > 0.73) - 0.105) < 0.01
    assert abs(np.mean(rho < 0.4) - 0.695) < 0.01


def test_crater_set_io_and_determinism(tmp_path):
    cfg = small_cfg(tmp_path)
    a = tcr.sample_craters(cfg, "big")
    b = tcr.sample_craters(cfg, "big")
    for f in ("x", "y", "D", "rho", "d", "h", "R", "id"):
        assert np.array_equal(getattr(a, f), getattr(b, f))
    a.save(tmp_path / "c.npz")
    c = tcr.CraterSet.load(tmp_path / "c.npz")
    assert np.array_equal(a.D, c.D) and c.which == "big" and c.meta["D_lo"] == 1.0
    other = tcr.sample_craters(load_config(overrides=[*SMALL, "world.seed=17"]), "big")
    assert len(other) != len(a) or not np.array_equal(other.x, a.x)
    # margin covers the widened influence radius
    assert a.meta["margin_m"] == pytest.approx(2.5 * 1.15 * 4.0 / 2)
    assert np.all(np.abs(a.d / (a.rho * 0.13 * a.D) - 1) <= 0.15 + 1e-12)
    assert np.all(np.abs(a.h / (a.rho * 0.03 * a.D) - 1) <= 0.2 + 1e-12)
    for f in tcr._SHAPE_FIELDS:
        assert np.array_equal(getattr(a, f), getattr(c, f)), f


# ---------------------------------------------------------------------------------------- fBm
def test_fbm_bands_are_complementary():
    p_c = tfbm.band_params(_cfg0, "coarse")
    p_f = tfbm.band_params(_cfg0, "fine")
    f = np.geomspace(1e-3, 60.0, 5000)
    S = tfbm.fbm_psd(f, p_c["S0_m4"], p_c["hurst"])
    W = tfbm.dtm_weight(f, p_c["lambda_c_m"], p_c["order"])
    total = tfbm.band_psd(p_c, f) + tfbm.band_psd(p_f, f)
    inside = f <= p_f["f_max"]
    # the fine band is plain S (W^2 = 1 - 4e-11 at f >= 2 cycles/m), hence rtol 1e-9
    assert np.allclose(total[inside], (S * W ** 2)[inside], rtol=1e-9, atol=0)
    assert np.all((tfbm.band_psd(p_c, f) == 0) | (tfbm.band_psd(p_f, f) == 0))    # no overlap
    # power-complementary with the DTM Butterworth: |H|^2 + W^2 = 1
    Hb = 1 / np.sqrt(1 + (f * p_c["lambda_c_m"]) ** 8)
    assert np.allclose(Hb ** 2 + W ** 2, 1.0)


@pytest.mark.parametrize("band,spacing,n_fft,crop,fit", [
    ("fine", 0.01, 1536, 1400, (2.5, 35.0)),
    ("coarse", 0.10, 1536, 1400, (0.25, 1.8)),
])
def test_fbm_psd_slope(band, spacing, n_fft, crop, fit):
    p = dict(tfbm.band_params(_cfg0, band))
    slopes, ratios = [], []
    for seed in range(4):
        z = tfbm.synthesize(np.random.default_rng(seed), n_fft, spacing, lambda f: tfbm.band_transfer(p, f),
                            crop=(crop, crop))
        f, P = tfbm.radial_psd(z, spacing, nbins=40)
        slopes.append(tfbm.loglog_slope(f, P, *fit))
        m = (f >= fit[0]) & (f <= fit[1])
        ratios.append(np.median(P[m] / tfbm.band_psd(p, f[m])))
    expected = -(2 + 2 * p["hurst"])
    assert abs(np.mean(slopes) - expected) < 0.1, slopes
    assert abs(np.mean(ratios) - 1.0) < 0.15, ratios          # absolute PSD level (S0)


def test_fbm_unit_fields_cached_and_deterministic(tmp_path):
    cfg_a = small_cfg(tmp_path / "a")
    cfg_b = small_cfg(tmp_path / "b")
    for band in tfbm.BANDS:
        a = tfbm.unit_field(cfg_a, band)
        b = tfbm.unit_field(cfg_b, band)
        assert np.array_equal(np.asarray(a.z), np.asarray(b.z))
        again = tfbm.unit_field(cfg_a, band)                    # cache hit, same data
        assert np.array_equal(np.asarray(a.z), np.asarray(again.z))
        assert a.z.dtype == np.float32 and a.spacing == tfbm.band_params(cfg_a, band)["spacing_m"]
    # gain scales the amplitude by sqrt(gain), same noise (independent of the config's current gains).
    # The fine band has its own gain since FID-06 (fbm_gain_fine, set in the config), so the fine case
    # sets that key; the coarse gain must not leak into it, and without fbm_gain_fine the fine band
    # falls back to fbm_gain (the old single-gain behaviour).
    cfg_g = small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain_fine=0.25")
    f1 = tfbm.fine_field(cfg_a, unit=True)
    f2 = tfbm.fine_field(cfg_g)
    assert np.allclose(np.asarray(f2.z), 0.5 * np.asarray(f1.z), rtol=1e-6, atol=1e-9)
    f3 = tfbm.fine_field(small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain=4.0",
                                   "terrain.roughness.fbm_gain_fine=0.25"))
    assert np.array_equal(np.asarray(f3.z), np.asarray(f2.z))
    f4 = tfbm.fine_field(small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain=0.25",
                                   "terrain.roughness.fbm_gain_fine=null"))
    assert np.allclose(np.asarray(f4.z), 0.5 * np.asarray(f1.z), rtol=1e-6, atol=1e-9)
    c2 = tfbm.coarse_field(small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain=4.0"))
    assert np.allclose(np.asarray(c2.z), 2.0 * np.asarray(tfbm.coarse_field(cfg_a, unit=True).z), rtol=1e-6, atol=1e-9)
    # a different seed gives a different field
    cfg_s = small_cfg(tmp_path / "s", "world.seed=99")
    assert not np.array_equal(np.asarray(tfbm.unit_field(cfg_s, "fine").z), np.asarray(tfbm.unit_field(cfg_a, "fine").z))


def test_fbm_synthesis_independent_of_thread_count(monkeypatch):
    """The blocked FFT gives the same bits for any LUNAR_WORKERS (rfft2(workers=n) differed by ~6e-8 m,
    which threshold-based steps such as the talus limiter can amplify)."""
    p = dict(tfbm.band_params(_cfg0, "coarse"))
    out = []
    for w in ("1", "3", "8"):
        monkeypatch.setenv("LUNAR_WORKERS", w)
        out.append(tfbm.synthesize(np.random.default_rng(4), 1000, 0.1, lambda f: tfbm.band_transfer(p, f),
                                   crop=(900, 900)))
    assert np.array_equal(out[0], out[1]) and np.array_equal(out[0], out[2])
    assert out[0].dtype == np.float32 and out[0].shape == (900, 900) and float(out[0].std()) > 0


def test_fbm_separate_fine_gain_fid06(tmp_path):
    """FID-06: fbm_gain scales the coarse band, fbm_gain_fine the fine band (default: fbm_gain)."""
    base = small_cfg(tmp_path / "a")
    cfg = small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain=0.25", "terrain.roughness.fbm_gain_fine=0.64")
    assert tfbm.fbm_gains(cfg) == {"coarse": 0.25, "fine": 0.64}
    fu, cu = tfbm.fine_field(cfg, unit=True), tfbm.coarse_field(cfg, unit=True)
    assert np.allclose(np.asarray(tfbm.fine_field(cfg).z), 0.8 * np.asarray(fu.z), rtol=1e-6, atol=1e-9)
    assert np.allclose(np.asarray(tfbm.coarse_field(cfg).z), 0.5 * np.asarray(cu.z), rtol=1e-6, atol=1e-9)
    null = small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain=0.25", "terrain.roughness.fbm_gain_fine=null")
    assert tfbm.fbm_gains(null)["fine"] == 0.25
    with pytest.raises(ValueError):
        tfbm.fbm_gains(small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain_fine=-1"))
    # the composed-grid key follows both gains
    k = {tcomp.TerrainModel(c).key for c in (base, cfg, small_cfg(tmp_path / "a", "terrain.roughness.fbm_gain_fine=0.5"))}
    assert len(k) == 3
    # evaluate: (coarse, fine) amplitude override, Ff alone scales with the fine factor only
    m = tcomp.TerrainModel(cfg)
    x = np.linspace(-1.5, 1.5, 301)
    y = np.full_like(x, 0.37)
    ff1 = m.evaluate(x, y, "core", parts=("Ff",), fbm_scale=(0.0, 1.0))
    ff2 = m.evaluate(x, y, "core", parts=("Ff",))
    assert np.allclose(ff2, 0.8 * ff1, rtol=1e-12, atol=1e-15) and np.abs(ff1).max() > 0
    assert np.array_equal(m.evaluate(x, y, "core", parts=("Ff",), fbm_scale=(1.0, 0.0)), np.zeros_like(x))


def test_cubic_sampler_equals_grid_sample():
    rng = np.random.default_rng(5)
    g = Grid(rng.standard_normal((120, 150)).astype(np.float32), -3.0, 2.0, 0.1)
    cs = tfbm.CubicSampler(g)
    x = rng.uniform(g.x0, g.x1, 4000)
    y = rng.uniform(g.y0, g.y1, 4000)
    assert np.abs(cs(x, y) - g.sample(x, y, order=3)).max() < 1e-12
    xi, yi = g.xs()[rng.integers(0, g.nx, 300)], g.ys()[rng.integers(0, g.ny, 300)]
    assert np.abs(cs(xi, yi) - g.sample(xi, yi, order=1)).max() < 1e-12      # interpolating at nodes


# ---------------------------------------------------------------------------------------- talus (FID-03)
def _pit(n=161, sp=0.1, depth=2.5, R=4.0):
    """A steep compound pit on a tilted plane (walls ~50 deg), float64."""
    x = (np.arange(n) - n // 2) * sp
    X, Y = np.meshgrid(x, x)
    r = np.hypot(X, Y)
    z = 0.15 * X - depth * np.clip(1 - (r / R) ** 2, 0, None) ** 0.5
    return z


def test_talus_relaxation_limits_slope_and_conserves_mass():
    z = _pit()
    sp = 0.1
    relaxed, info = ttal.relax(z, sp, 34.0, 1.0)
    assert info["converged"] and info["max_slope_before_deg"] > 45
    g = ttal._max_edge_gradient_rows(relaxed, 1 / sp).max()
    assert math.degrees(math.atan(g)) <= 34.0 + 1e-9
    assert abs(info["mass_balance_m3"]) < 1e-9                     # mass-conserving to rounding
    d = relaxed - z
    assert np.all(d[:3, :] == 0) and np.all(d[-3:, :] == 0)      # untouched away from the pit (exact)
    far = np.hypot(*np.meshgrid((np.arange(161) - 80) * sp, (np.arange(161) - 80) * sp)) > 6.0
    assert np.all(d[far] == 0.0)
    # slumping: the upper wall loses height, the lower wall gains it, never above the old maximum
    assert d.max() > 0.05 and d.min() < -0.05 and relaxed.max() <= z.max() + 1e-12
    r = np.hypot(*np.meshgrid((np.arange(161) - 80) * sp, (np.arange(161) - 80) * sp))
    assert r[d < -1e-6].mean() > r[d > 1e-6].mean()             # material moved inward (downhill)
    again, _ = ttal.relax(z, sp, 34.0, 1.0)
    assert np.array_equal(again, relaxed)                          # deterministic
    # a gentle surface is returned unchanged
    flat = 0.2 * np.add.outer(np.arange(50.0), np.arange(60.0)) * sp
    same, info2 = ttal.relax(flat, sp, 34.0, 1.0)
    assert info2["iterations"] == 0 and np.array_equal(same, flat)


def test_talus_sampler_interpolates_and_is_exactly_zero_elsewhere():
    T = np.zeros((60, 60), np.float32)
    T[25:35, 20:30] = np.random.default_rng(3).normal(0, 0.1, (10, 10)).astype(np.float32)
    g = Grid(T, -3.0, -3.0, 0.1)
    s = ttal.TalusSampler(g)
    xi, yi = np.meshgrid(g.xs(), g.ys())
    assert np.allclose(s(xi, yi), T, atol=1e-6)                   # interpolating at the nodes
    far = np.array([[-2.5, -2.5], [2.5, 2.5], [-2.0, 1.9], [1.5, -0.95]])
    assert np.all(s(far[:, 0], far[:, 1]) == 0.0)                 # exactly zero where T and neighbours are 0
    xx = np.linspace(-1.2, 0.2, 2001)
    v = s(xx, np.full_like(xx, -0.27))
    assert np.abs(np.diff(v)).max() < 0.01                        # continuous
    assert ttal.TalusSampler(Grid(np.zeros((10, 10), np.float32), 0, 0, 0.1)).empty


# ---------------------------------------------------------------------------------------- fades
def test_core_edge_fade_width_fid04():
    """FID-04: zones.core.edge_fade_m widens the fine-detail fade (3 m proposed) and changes the grid key."""
    c1 = load_config(overrides=["zones.core.edge_fade_m=1.0"])
    c3 = load_config(overrides=["zones.core.edge_fade_m=3.0"])
    x = np.array([47.0, 48.5, 49.0, 49.9, 50.0])
    y = np.zeros_like(x)
    assert np.allclose(tcomp.w_core(c3, x, y), [1.0, 0.5, 0.25, 0.5 * (1 - math.cos(math.pi * 0.1 / 3)), 0.0])
    assert tcomp.w_core(c1, np.array([49.0]), np.zeros(1))[0] == 1.0
    assert tcomp.TerrainModel(c1).key != tcomp.TerrainModel(c3).key


def test_fade_weights_at_boundaries():
    cfg = _cfg0
    h, w = cfg.zones.core.size_m / 2, cfg.zones.core.edge_fade_m
    y0 = np.zeros(7)
    x = np.array([0.0, h - w - 1e-9, h - w, h - w / 2, h - 1e-12, h, h + 1.0])
    wc = tcomp.w_core(cfg, x, y0)
    assert wc[0] == 1.0 and wc[1] == 1.0 and wc[2] == 1.0
    assert wc[3] == pytest.approx(0.5)
    assert wc[5] == 0.0 and wc[6] == 0.0 and wc[4] < 1e-20
    assert np.all(tcomp.w_core(cfg, -x, y0) == wc) and np.all(tcomp.w_core(cfg, y0, x) == wc)
    hr, wr = cfg.zones.ring.size_m / 2, cfg.zones.ring.edge_fade_m
    xr = np.array([0.0, hr - wr, hr - wr / 2, hr])
    assert np.allclose(tcomp.w_ring(cfg, xr, np.zeros(4)), [1.0, 1.0, 0.5, 0.0])
    assert tcomp.w_ring(cfg, np.array([hr]), np.array([0.0]))[0] == 0.0
    # C1: the derivative is continuous (0 at both ends of the ramp)
    e = 1e-6
    for xb in (h - w, h):
        d1 = (tcomp.w_core(cfg, np.array([xb + e]), np.zeros(1)) - tcomp.w_core(cfg, np.array([xb]), np.zeros(1))) / e
        d0 = (tcomp.w_core(cfg, np.array([xb]), np.zeros(1)) - tcomp.w_core(cfg, np.array([xb - e]), np.zeros(1))) / e
        assert abs(d1[0]) < 1e-4 and abs(d0[0]) < 1e-4
    # horizon blend: 1 inside the far square, 0 beyond the skirt
    hf = cfg.zones.far.size_m / 2
    v = tcomp.horizon_blend_weight(cfg, np.array([0.0, hf, hf + tbase.FAR_SKIRT_M / 2, hf + tbase.FAR_SKIRT_M]),
                                   np.zeros(4))
    assert np.allclose(v, [1.0, 1.0, 0.5, 0.0]) and v[1] == 1.0


# ---------------------------------------------------------------------------------------- deviogram
def test_deviogram_estimator_known_fields():
    sp = 0.01
    n = 1001
    x = np.arange(n) * sp
    X, Y = np.meshgrid(x, x)
    lam, A = 0.5, 0.01
    z = A * np.sin(2 * np.pi * X / lam) + 0.3 * X - 0.2 * Y + 4.0          # sinusoid along x + a plane
    dv = tcal.deviogram(z.astype(np.float32), sp, (0.1, 0.25))
    # D_x = 2 A^2 sin^2(pi L / lam) (detrending a sinusoid over 20 full periods changes it < 1 %)
    for L, nux, nuy in zip(dv["lags_m"], dv["nu_x_m"], dv["nu_y_m"]):
        assert nux == pytest.approx(math.sqrt(2) * A * abs(math.sin(math.pi * L / lam)), rel=0.02)
        assert nuy < 1e-5                                                 # plane removed
    # linear combination statistics are exact
    rng = np.random.default_rng(1)
    a = rng.standard_normal((300, 300)).astype(np.float32)
    b = (np.cumsum(rng.standard_normal((300, 300)), axis=1) * 0.01).astype(np.float32)
    st = tcal.cross_stats(np.stack([a, b]), sp, (0.01, 0.05, 0.3))
    for s in (0.0, 0.3, 1.7):
        direct = tcal.deviogram((a.astype(np.float64) + s * b).astype(np.float32), sp, (0.01, 0.05, 0.3))["nu_m"]
        assert np.allclose(tcal.nu_of(st, [1.0, s])["nu"], direct, rtol=1e-5)


def test_fit_gain_recovers_known_gain():
    rng = np.random.default_rng(2)
    n = 400
    B = np.cumsum(np.cumsum(rng.standard_normal((n, n)), 0), 1) * 1e-4
    F = rng.standard_normal((n, n)) * 2e-3
    comps = np.stack([B, np.zeros((n, n)), F, np.zeros((n, n))]).astype(np.float32)
    st = tcal.cross_stats(comps, 0.01, (0.01, 0.03, 0.1, 0.3, 1.0))
    T = tcal.nu_of(st, tcal.fbm_coef(0.37))["nu"]
    fit = tcal.fit_gain(st, T, 0.25)
    assert fit["gain"] == pytest.approx(0.37, abs=2e-3) and fit["n_outside_tolerance"] == 0


def test_fit_gains_two_bands_site_aware_fid06():
    """Two gains: a smooth (coarse-like) and a rough (fine-like) field with known gains are recovered when
    the target equals their composite at the small lags and the synthetic part at 1-3 m."""
    rng = np.random.default_rng(7)
    n = 1200
    zero = np.zeros((n, n))
    Fc = np.cumsum(np.cumsum(rng.standard_normal((n, n)), 0), 1) * 2e-5    # smooth, grows with L
    Ff = rng.standard_normal((n, n)) * 3e-3                                    # white: saturates
    comps = np.stack([zero, zero, zero, Fc, Ff]).astype(np.float32)
    lags = (0.01, 0.03, 0.1, 0.3, 1.0, 3.0)
    st = tcal.cross_stats(comps, 0.01, lags)
    gc0, gf0 = 0.31, 0.77
    T = tcal.nu_of(st, tcal.fbm_coef(gc0, 5, gf0))["nu"]
    fit = tcal.fit_gains(st, T, 0.25)
    assert fit["n_criteria_met"] == fit["n_criteria"] == 6
    assert fit["gain"] == pytest.approx(gc0, abs=3e-3) and fit["gain_fine"] == pytest.approx(gf0, abs=3e-3)
    # the single shared gain cannot do this as well
    single = tcal.fit_gain(st, T, 0.25)
    nu1 = tcal.nu_of(st, tcal.fbm_coef(single["gain"], 5))["nu"]
    assert np.max(np.abs(np.log(nu1 / T))) > 10 * fit["max_abs_log_ratio"]


def test_write_gain_fine_inserted_after_fbm_gain(tmp_path):
    p = tmp_path / "cfg.yaml"
    shutil.copy(DEFAULT_CONFIG, p)
    before = p.read_text()
    if "fbm_gain_fine:" in before:                         # the line may already exist in a later config
        before = "\n".join(ln for ln in before.splitlines() if "fbm_gain_fine:" not in ln) + "\n"
        p.write_text(before)
    assert tcal.write_gain_to_config(p, 0.612, "fbm_gain_fine") is None
    after = p.read_text()
    lines_b, lines_a = before.splitlines(), after.splitlines()
    i = [k for k, ln in enumerate(lines_a) if "fbm_gain:" in ln][0]
    assert "fbm_gain_fine: 0.612" in lines_a[i + 1]
    assert lines_a[i + 1].index("#") == lines_a[i].index("#")                 # comment column aligned
    assert lines_a[:i + 1] + lines_a[i + 2:] == lines_b                        # nothing else changed
    r = yaml.safe_load(after)["terrain"]["roughness"]
    assert r["fbm_gain_fine"] == 0.612 and r["fbm_gain"] == yaml.safe_load(before)["terrain"]["roughness"]["fbm_gain"]
    assert tcal.write_gain_to_config(p, 0.5, "fbm_gain_fine") == "0.612"    # second write replaces the value
    assert yaml.safe_load(p.read_text())["terrain"]["roughness"]["fbm_gain_fine"] == 0.5


def test_nodata_fill_textured_fid17():
    """FID-17: the textured fill keeps the valid nodes, is continuous at the hole boundary and restores the
    surrounding roughness inside the hole (the harmonic fill alone is far too smooth)."""
    rng = np.random.default_rng(11)
    n = 256
    f = np.fft.fftfreq(n)
    k = np.hypot(f[:, None], np.fft.rfftfreq(n)[None, :])
    with np.errstate(divide="ignore"):
        amp = np.where(k > 0, k ** -1.85, 0.0)
    z = np.fft.irfft2(np.fft.rfft2(rng.standard_normal((n, n))) * amp, s=(n, n))
    z *= 5.0 / z.std()
    zz = z.copy()
    zz[200:250, 40:200] = np.nan                           # a hole like the horizon rectangles
    bad = ~np.isfinite(zz)
    out, nf, info = tbase.fill_nodata_textured(zz, 20.0, np.random.default_rng(5))
    harm, _ = tbase.fill_nodata_harmonic(zz)
    assert nf == int(bad.sum()) and np.isfinite(out).all()
    assert np.array_equal(out[~bad], zz[~bad].astype(np.float32))
    ref = tbase._deviogram_masked(z, ~bad, (1, 2))
    tex = tbase._deviogram_masked(out.astype(np.float64), bad, (1, 2))
    smooth = tbase._deviogram_masked(harm.astype(np.float64), bad, (1, 2))
    assert np.all(tex / ref > 0.6) and np.all(tex / ref < 1.6)
    assert np.all(smooth / ref < 0.5)
    # continuity: the first ring of hole nodes stays within the valid data's typical step of the harmonic fill
    ring1 = bad & ~ndimage.binary_erosion(bad)
    assert np.abs(out[ring1] - harm[ring1]).max() < 0.5 * ref[0]
    again, _, _ = tbase.fill_nodata_textured(zz, 20.0, np.random.default_rng(5))
    assert np.array_equal(again, out)
    assert info["hurst_fit"] > 0.3


def test_write_gain_to_config(tmp_path):
    p = tmp_path / "cfg.yaml"
    shutil.copy(DEFAULT_CONFIG, p)
    before = p.read_text()
    tcal.write_gain_to_config(p, 0.437)
    after = p.read_text()
    old_line = [ln for ln in before.splitlines() if "fbm_gain:" in ln][0]
    new_line = [ln for ln in after.splitlines() if "fbm_gain:" in ln][0]
    assert new_line.split("#")[1] == old_line.split("#")[1]                   # comment kept
    assert new_line.index("#") == old_line.index("#")                         # same column
    assert [ln for ln in before.splitlines() if "fbm_gain:" not in ln] == \
           [ln for ln in after.splitlines() if "fbm_gain:" not in ln]         # nothing else changed
    assert yaml.safe_load(after)["terrain"]["roughness"]["fbm_gain"] == 0.437


def test_nodata_fill_harmonic():
    yy, xx = np.mgrid[0:60, 0:80].astype(np.float64)
    z = 0.5 * xx - 0.25 * yy + 3.0
    zz = z.copy()
    zz[10:20, 30:50] = np.nan
    zz[50:60, 70:80] = np.nan            # touches the grid corner
    out, n = tbase.fill_nodata_harmonic(zz)
    assert n == 200 + 100
    assert np.isfinite(out).all()
    assert np.abs(out[10:20, 30:50] - z[10:20, 30:50]).max() < 1e-4        # linear fields are harmonic
    assert out[50:60, 70:80].min() >= np.nanmin(zz) and out[50:60, 70:80].max() <= np.nanmax(zz)


# ---------------------------------------------------------------------------------------- composition (DTM)
@dtm
def test_cross_zone_consistency(built):
    cfg = built
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    ring = tcomp.load_zone_height(cfg, "ring")
    tiles = {(t.i, t.j): tcomp.load_core_tile(cfg, t) for t in lay.tiles()}
    # ring grid vs core tiles at the ring nodes on the core boundary: within 0.1 mm
    s = np.arange(core.x0, core.x1 + 1e-9, ring.spacing)
    bx = np.concatenate([s, s, np.full_like(s, core.x0), np.full_like(s, core.x1)])
    by = np.concatenate([np.full_like(s, core.y0), np.full_like(s, core.y1), s, s])
    zc = tcomp._tile_values(cfg, tiles, bx, by)
    zr = ring.sample(bx, by, order=1)
    assert np.abs(zc - zr).max() == 0.0                    # global B: bit-identical shared nodes
    # the raw B extractions themselves agree at common nodes inside the core (< 0.1 mm)
    m = tcomp.get_model(cfg)
    bc, br = m.base("core"), m.base("ring")
    q = np.arange(-2.5, 2.5 + 1e-9, 0.25)
    Q1, Q2 = np.meshgrid(q, q)
    assert np.abs(bc.sample(Q1, Q2, order=1) - br.sample(Q1, Q2, order=1)).max() < 1e-4
    # neighbouring tiles agree exactly on their overlap (same function, same lattice coordinates)
    a, b = tiles[(0, 0)], tiles[(1, 0)]
    xa = a.xs()
    ov = (xa >= b.x0 - 1e-9)
    ib0 = int(round((xa[ov][0] - b.x0) / b.spacing))
    assert np.array_equal(np.asarray(a.z)[:, ov], np.asarray(b.z)[:, ib0:ib0 + int(ov.sum())])
    # ring vs far at the ring boundary (H = B there): identical (task limit 1 cm)
    far = tcomp.load_zone_height(cfg, "far")
    rr = lay.zone_rect("ring")
    s = np.arange(rr.x0, rr.x1 + 1e-9, far.spacing)
    bx = np.concatenate([s, s, np.full_like(s, rr.x0), np.full_like(s, rr.x1)])
    by = np.concatenate([np.full_like(s, rr.y0), np.full_like(s, rr.y1), s, s])
    assert np.abs(ring.sample(bx, by, order=1) - far.sample(bx, by, order=1)).max() == 0.0
    # far vs horizon at the far boundary: identical (the horizon blends to B_far there)
    hor = tcomp.load_zone_height(cfg, "horizon")
    fr = lay.zone_rect("far")
    s = np.arange(fr.x0, fr.x1 + 1e-9, hor.spacing)
    bx = np.concatenate([s, s, np.full_like(s, fr.x0), np.full_like(s, fr.x1)])
    by = np.concatenate([np.full_like(s, fr.y0), np.full_like(s, fr.y1), s, s])
    assert np.abs(far.sample(bx, by, order=1) - hor.sample(bx, by, order=1)).max() == 0.0
    # H0 is one global function: every zone returns the same value at the same point
    rng = np.random.default_rng(9)
    px = rng.uniform(-150, 150, 4000)
    py = rng.uniform(-150, 150, 4000)
    ref = tcomp.height_at(cfg, px, py, "core")
    for zone in ("ring", "far", "horizon"):
        assert np.array_equal(tcomp.height_at(cfg, px, py, zone), ref)
    # the nested B is continuous across every extraction border (no steps > 0.1 mm at fine sampling)
    for edge in (2.0, 2.5, 20.0, 40.0, 100.0, 300.0):
        xs = np.linspace(edge - 0.05, edge + 0.05, 2001)
        b = m.evaluate(xs, np.full_like(xs, 0.37), "core", parts=("B",))
        assert np.abs(np.diff(b)).max() < 1e-4, edge


@dtm
def test_height_at_equals_grids_at_nodes(built):
    cfg = built
    rng = np.random.default_rng(3)
    grids = {"core": tcomp.load_core_tile(cfg, TileId(1, 0)), "ring": tcomp.load_zone_height(cfg, "ring"),
             "far": tcomp.load_zone_height(cfg, "far"), "horizon": tcomp.load_zone_height(cfg, "horizon")}
    for zone, g in grids.items():
        jj, ii = rng.integers(0, g.ny, 2000), rng.integers(0, g.nx, 2000)
        x = g.x0 + ii * g.spacing            # float coordinates (not lattice-exact on purpose)
        y = g.y0 + jj * g.spacing
        h = tcomp.height_at(cfg, x, y, zone)
        assert h.dtype == np.float32
        assert np.array_equal(h, np.asarray(g.z)[jj, ii]), zone
    # the cached tile equals a fresh evaluation
    fresh = tcomp.core_tile_height(cfg, TileId(1, 0))
    assert np.array_equal(fresh.z, np.asarray(grids["core"].z))


@dtm
def test_composition_terms(built):
    """H0 = B + w_ring (Fc + Kb) + w_core (Ff + Ks), checked against the individual parts."""
    cfg = built
    m = tcomp.get_model(cfg)
    rng = np.random.default_rng(4)
    x = rng.uniform(-6, 6, 3000)
    y = rng.uniform(-6, 6, 3000)
    total = m.evaluate(x, y, "core")
    parts = {p: m.evaluate(x, y, "core", parts=(p,)) for p in tcomp.PARTS}
    assert np.allclose(total, sum(parts.values()), atol=1e-12)
    out = (np.abs(x) >= 2) | (np.abs(y) >= 2)
    assert np.all(parts["Ff"][out] == 0) and np.all(parts["Ks"][out] == 0)     # w_core = 0 outside the core
    # far outside the ring only B remains
    xf = np.array([25.0, -30.0, 90.0])
    yf = np.array([0.0, 21.0, -60.0])
    assert np.array_equal(m.evaluate(xf, yf, "far"), m.evaluate(xf, yf, "far", parts=("B",)))


@dtm
def test_calibration_end_to_end_on_temp_config(tmp_path):
    """Fit, apply (to a temporary config copy only), rebuild and verify on the small layout."""
    cpath = tmp_path / "cfg.yaml"
    shutil.copy(DEFAULT_CONFIG, cpath)
    real_before = Path(DEFAULT_CONFIG).read_bytes()
    cfg = load_config(cpath, overrides=[f"paths.build_dir={tmp_path / 'build'}", *SMALL])
    cal = tcal.run(cfg, apply=True, rebuild=True, config_path=cpath, figure=tmp_path / "dv.png",
                   lags_m=(0.01, 0.03, 0.1, 0.3, 1.0), log=lambda *a: None)
    g, gf = cal["fit"]["gain"], cal["fit"]["gain_fine"]
    assert 0.0 <= g <= tcal.G_MAX and 0.0 <= gf <= tcal.G_MAX
    r = yaml.safe_load(cpath.read_text())["terrain"]["roughness"]
    assert r["fbm_gain"] == pytest.approx(round(g, 3)) and r["fbm_gain_fine"] == pytest.approx(round(gf, 3))
    assert cal["applied"]["stable"]
    assert cal["verified"]["max_abs_rel_diff_vs_predicted"] < 1e-3      # linear model is exact
    assert (tmp_path / "dv.png").stat().st_size > 10_000
    assert json.loads((Path(cfg.paths.build_dir) / "terrain" / "calibration.json").read_text())["fit"]["gain"] == g
    assert Path(DEFAULT_CONFIG).read_bytes() == real_before                # the real config is untouched


@dtm
def test_talus_term_in_composition(built, tmp_path):
    """FID-03 on the real DTM (small layout, a low 28 deg limit so that windows exist): T relaxes the
    10 cm composite to the limit, is zero (and H0 bit-identical to the unlimited H0) away from the windows,
    and every zone boundary stays bit-identical."""
    src = Layout(built).stage_dir("terrain")
    dst = tmp_path / "build"
    (dst / "terrain").mkdir(parents=True)
    for sub in ("base", "fbm", "craters"):
        shutil.copytree(src / sub, dst / "terrain" / sub)
    cfg = small_cfg(dst, "terrain.talus.max_slope_deg=28.0")
    man = tcomp.build_all(cfg, log=lambda *a: None)
    m = tcomp.get_model(cfg)
    tg = m.talus_field()
    st = man["stages"]["talus"]
    assert st["n_windows"] >= 1 and st["n_changed_nodes"] > 0
    assert all(w["converged"] and not w["frame_touched"] for w in st["windows"])
    assert all(abs(w["mass_balance_m3"]) < 1e-9 for w in st["windows"])
    # the 10 cm composite + T respects the limit inside the ring's full-weight square
    half = cfg.zones.ring.size_m / 2 - cfg.zones.ring.edge_fade_m
    xs = np.round(np.arange(-half, half + 1e-9, 0.1), 2)
    X, Y = np.meshgrid(xs, xs)
    Hs = m.evaluate(X, Y, "ring", parts=("B", "Fc", "Kb", "Ks", "T"))
    g = ttal._max_edge_gradient_rows(Hs, 10.0).max()
    assert math.degrees(math.atan(g)) <= 28.0 + 0.05
    # away from relaxed walls T is exactly 0 and H0 equals the unlimited composition bit for bit
    T = m.evaluate(X, Y, "ring", parts=("T",))
    zero = ndimage.binary_erosion(T == 0.0, iterations=3)
    no_t = m.evaluate(X, Y, "ring", parts=("B", "Fc", "Kb", "Ff", "Ks"))
    full = m.evaluate(X, Y, "ring")
    assert zero.mean() > 0.5 and np.array_equal(full[zero], no_t[zero])
    assert np.abs(T).max() > 0.0 and float(np.abs(np.asarray(tg.z)).max()) == pytest.approx(st["max_abs_T_m"])
    chk = man["checks"]
    for k in ("core_vs_ring_on_core_boundary", "ring_vs_far_on_ring_boundary", "far_vs_horizon_on_far_boundary"):
        assert chk[k]["max_abs_m"] == 0.0, k
    assert chk["tile_tile_overlap_max_abs_m"] == 0.0 and chk["height_at_vs_tile_nodes_max_abs_m"] == 0.0
    # deterministic: a rebuild of the talus field gives the same bits
    again = ttal.compute_talus(m, log=lambda *a: None)
    assert np.array_equal(np.asarray(again.z), np.asarray(tg.z))


@dtm
def test_manifest_and_counts(built):
    cfg = built
    man = json.loads((Layout(cfg).stage_dir("terrain") / "manifest.json").read_text())
    for st in ("base", "fbm_coarse", "fbm_fine", "craters", "talus", "tiles", "zone_ring", "zone_far",
               "zone_horizon", "checks"):
        assert st in man["stages"], st
    chk = man["checks"]
    assert chk["core_vs_ring_on_core_boundary"]["max_abs_m"] == 0.0
    assert chk["ring_vs_far_on_ring_boundary"]["max_abs_m"] == 0.0
    assert chk["far_vs_horizon_on_far_boundary"]["max_abs_m"] == 0.0
    assert chk["tile_tile_overlap_max_abs_m"] == 0.0
    assert chk["height_at_vs_tile_nodes_max_abs_m"] == 0.0
    assert man["disk_bytes"]["total"] > 0 and man["peak_rss"]["self_mb"] > 0
