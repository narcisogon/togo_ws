"""Tests for lunar_world_gen.world (sdf, lighting, spawn, world_info), the launch-file helpers
and the physics-plugin patch script. Run with the venv, without ROS sourced:

    cd src/lunar_world_gen && /home/nargon/.venvs/lunar_world/bin/python -m pytest tests/test_world_sdf.py -q
"""
from __future__ import annotations

import importlib.util
import math
import shutil
import subprocess
import tarfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.layout import Layout, Rect
from lunar_world_gen.world import lighting, sdf, spawn, world_info

PKG = Path(__file__).resolve().parents[1]
BASE = (0.12, 0.107, 0.088)


# ------------------------------------------------------------------------------ fixtures
@pytest.fixture()
def cfg(tmp_path):
    return load_config(overrides=[f"paths.world_dir={tmp_path / 'world'}",
                                  f"paths.build_dir={tmp_path / 'build'}"])


@pytest.fixture()
def world(cfg):
    """Layout plus fake mesh and texture files (contents are irrelevant to the writer)."""
    lay = Layout(cfg)
    for rel in ("meshes/tile_00_00_ground.glb", "meshes/tile_00_00_rocks.glb", "meshes/ring_south_ground.glb",
                "meshes/core_collision.glb", "meshes/ring_south_collision.glb", "meshes/rock_hulls.glb",
                *sdf.REGOLITH_TEXTURES.values(),
                *(v for v in sdf.ROCK_TEXTURES.values() if v)):
        p = lay.world_dir / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b"x")
    visuals = [sdf.VisualSpec("tile_00_00_ground", "meshes/tile_00_00_ground.glb", "regolith_core"),
               sdf.VisualSpec("tile_00_00_rocks", "meshes/tile_00_00_rocks.glb", "rocks"),
               sdf.VisualSpec("ring_south_ground", "meshes/ring_south_ground.glb", "regolith_ring",
                              cast_shadows=False, visibility_flags=3)]
    collisions = [sdf.CollisionSpec("core_collision", "meshes/core_collision.glb", 0.4),
                  sdf.CollisionSpec("ring_south_collision", "meshes/ring_south_collision.glb", 0.4)]
    return lay, visuals, collisions


def _parse(path):
    return ET.parse(str(path)).getroot()


# ------------------------------------------------------------------------------ lighting
def test_sun_events_point_down_and_match_el_az(cfg):
    for event, (el, az, vec) in cfg.lighting.sun_events.items():
        light = lighting.sun_light(cfg, event)
        d = np.array(light.direction)
        assert d[2] < 0 and abs(np.linalg.norm(d) - 1) < 1e-9
        assert lighting.angle_between_deg(d, lighting.direction_from_el_az(el, az)) < 0.1
        el2, az2 = lighting.el_az_from_direction(d)
        assert abs(el2 - el) < 0.1 and abs(az2 - az) < 0.1


def test_direction_checks_reject_bad_vectors():
    with pytest.raises(ValueError):
        lighting.check_direction([0.5, 0.0, 0.3], 20.0, 90.0)        # upward
    with pytest.raises(ValueError):
        lighting.check_direction([-0.9, 0.3, -0.37], 21.94, 84.66)    # wrong azimuth by ~20 deg
    with pytest.raises(ValueError):
        lighting.check_direction([0, 0, 0], 10, 10)


def test_lighting_modes(cfg):
    real = lighting.lighting_setup(cfg, "realistic")
    assert real.sun_event == cfg.lighting.sun_event == "eva1_start"
    assert real.ambient[:3] == (0.01, 0.01, 0.01) and real.background == (0, 0, 0, 1)
    names = [l.name for l in real.lights]
    assert names == ["sun", "earthshine"]
    sun, earth = real.lights
    assert sun.cast_shadows and not earth.cast_shadows and sun.intensity == 1.0
    # the configured earthshine (FID-14: 3e-5 of the sun, the calibrated value; was 0.01)
    es = float(cfg.lighting.realistic.earthshine_intensity)
    assert earth.intensity == pytest.approx(es, rel=1e-12) and 0.0 < earth.intensity < sun.intensity
    insp_earth = [l for l in lighting.lighting_setup(cfg, "inspection").lights if l.name == "earthshine"][0]
    assert insp_earth.intensity == pytest.approx(es, rel=1e-12)
    # earth at el 75, az 321 (north-west): light travels toward south-east and down
    assert earth.direction[0] > 0 and earth.direction[1] < 0 and earth.direction[2] < -0.9
    insp = lighting.lighting_setup(cfg, "inspection")
    assert insp.sun_event == "eva2_start" and insp.ambient[0] == pytest.approx(0.25)
    fill = [l for l in insp.lights if l.name == "overhead_fill"][0]
    assert fill.direction == (0.0, 0.0, -1.0) and fill.intensity == pytest.approx(0.35) and not fill.cast_shadows
    with pytest.raises(ValueError):
        lighting.lighting_setup(cfg, "noon")


# ------------------------------------------------------------------------------ SDF
def test_sdf_structure_default_variant(cfg, world):
    lay, visuals, collisions = world
    path = sdf.write_world(cfg, lay, visuals, collisions, "")
    assert path == lay.world_dir / "apollo16_core.sdf"
    root = _parse(path)
    assert root.tag == "sdf" and root.get("version") == "1.11"
    w = root.find("world")
    assert w.get("name") == "apollo16_core"
    plugins = {p.get("name"): p for p in w.findall("plugin")}
    assert set(plugins) == set(sdf.REQUIRED_SYSTEMS)
    for name, fn in sdf.REQUIRED_SYSTEMS.items():
        assert plugins[name].get("filename") == fn
    for bad in sdf.FORBIDDEN_SYSTEMS:
        assert bad not in plugins
    assert not [p for p in root.iter("plugin") if "sensors" in (p.get("filename") or "")]
    engine = plugins["gz::sim::systems::Physics"].find("engine/filename").text
    assert engine == str(Path(cfg.paths.physics_plugin)) and Path(engine).is_absolute()
    ph = w.find("physics")
    assert ph.get("type") == "ignored"
    assert float(ph.find("max_step_size").text) == pytest.approx(0.001)
    assert float(ph.find("real_time_factor").text) == 1.0
    assert float(ph.find("real_time_update_rate").text) == pytest.approx(1000)
    assert [float(v) for v in w.find("gravity").text.split()] == [0.0, 0.0, -1.62]
    assert [float(v) for v in w.find("magnetic_field").text.split()] == [0.0, 0.0, 0.0]
    sc = w.find("spherical_coordinates")
    assert sc.find("surface_model").text == "CUSTOM_SURFACE"
    assert float(sc.find("surface_axis_equatorial").text) == 1737400.0
    assert float(sc.find("surface_axis_polar").text) == 1737400.0
    assert sc.find("world_frame_orientation").text == "ENU"
    assert float(sc.find("latitude_deg").text) == pytest.approx(-8.973448)
    assert float(sc.find("longitude_deg").text) == pytest.approx(15.511126)
    assert float(sc.find("elevation").text) == pytest.approx(8.1955)
    assert float(sc.find("heading_deg").text) == 0.0
    scene = w.find("scene")
    assert scene.find("background").text.split()[:3] == ["0", "0", "0"]
    assert scene.find("shadows").text == "true" and scene.find("grid").text == "false"
    assert float(scene.find("ambient").text.split()[0]) == pytest.approx(0.01)
    lights = {l.get("name"): l for l in w.findall("light")}
    assert set(lights) == {"sun", "earthshine"}
    sun_dir = [float(v) for v in lights["sun"].find("direction").text.split()]
    assert sun_dir == pytest.approx(list(cfg.lighting.sun_events.eva1_start[2]), abs=2e-4)
    assert lights["sun"].find("cast_shadows").text == "true"
    assert lights["earthshine"].find("cast_shadows").text == "false"
    # small intensities survive the SDF number format (3e-05 is written in exponent notation)
    assert float(lights["earthshine"].find("intensity").text) == pytest.approx(
        float(cfg.lighting.realistic.earthshine_intensity), rel=1e-6)
    assert lights["sun"].find("pose").text.split()[3:] == ["0", "0", "0"]
    models = w.findall("model")
    assert [m.get("name") for m in models] == ["apollo16_terrain"]
    m = models[0]
    assert m.find("static").text == "true"
    links = m.findall("link")
    assert [l.get("name") for l in links] == ["terrain"]
    vis = {v.get("name"): v for v in links[0].findall("visual")}
    assert set(vis) == {v.name for v in visuals}
    rs = vis["ring_south_ground"]
    assert rs.find("cast_shadows").text == "false" and rs.find("visibility_flags").text == "3"
    metal = vis["tile_00_00_ground"].find("material/pbr/metal")
    assert metal.find("albedo_map").text == "textures/regolith_albedo.png"
    assert metal.find("normal_map").text == "textures/regolith_normal.png"
    assert metal.find("roughness_map").text == "textures/regolith_roughness.png"
    assert float(metal.find("metalness").text) == 0.0
    # with a roughness map the scalar is 1 (Ogre multiplies scalar x map; the map holds 0.95)
    assert float(metal.find("roughness").text) == pytest.approx(1.0)
    assert vis["tile_00_00_ground"].find("material/diffuse").text == "1 1 1 1"
    assert vis["tile_00_00_ground"].find("material/specular").text == "0 0 0 1"
    rmetal = vis["tile_00_00_rocks"].find("material/pbr/metal")
    assert rmetal.find("albedo_map").text == "textures/rock_atlas_basecolor.png"
    assert rmetal.find("roughness_map").text == "textures/rock_atlas_roughness.png"
    assert float(rmetal.find("roughness").text) == pytest.approx(1.0)
    for v in vis.values():
        uri = v.find("geometry/mesh/uri").text
        assert not Path(uri).is_absolute() and (lay.world_dir / uri).is_file()
    cols = links[0].findall("collision")
    assert [c.get("name") for c in cols] == ["core_collision", "ring_south_collision"]
    for c in cols:
        ode = c.find("surface/friction/ode")
        assert float(ode.find("mu").text) == pytest.approx(0.4) and float(ode.find("mu2").text) == pytest.approx(0.4)


def test_stock_and_inspect_variants(cfg, world):
    lay, visuals, collisions = world
    paths = sdf.write_all_variants(cfg, lay, visuals, collisions)
    assert {k: p.name for k, p in paths.items()} == {
        "": "apollo16_core.sdf", "inspect": "apollo16_core_inspect.sdf", "stock": "apollo16_core_stock.sdf",
        "inspect_stock": "apollo16_core_inspect_stock.sdf"}
    st = _parse(paths["stock"]).find("world")
    assert st.find("plugin/engine/filename").text == "gz-physics-bullet-plugin"
    assert float(st.find("physics/max_step_size").text) == pytest.approx(0.004)
    assert float(st.find("physics/real_time_update_rate").text) == pytest.approx(250)
    assert {l.get("name") for l in st.findall("light")} == {"sun", "earthshine"}
    ins = _parse(paths["inspect"]).find("world")
    assert {l.get("name") for l in ins.findall("light")} == {"sun", "earthshine", "overhead_fill"}
    assert float(ins.find("scene/ambient").text.split()[0]) == pytest.approx(0.25)
    sun = [l for l in ins.findall("light") if l.get("name") == "sun"][0]
    assert [float(v) for v in sun.find("direction").text.split()] == pytest.approx(
        list(cfg.lighting.sun_events.eva2_start[2]), abs=2e-4)
    assert ins.find("plugin/engine/filename").text == str(Path(cfg.paths.physics_plugin))
    assert sdf.read_engine_filename(paths["stock"]) == "gz-physics-bullet-plugin"
    with pytest.raises(ValueError):
        sdf.write_world(cfg, lay, visuals, collisions, "fancy")


def test_engine_stock_config_applies_to_default_variant(tmp_path, world):
    lay, visuals, collisions = world
    cfg2 = load_config(overrides=[f"paths.world_dir={lay.world_dir}", "physics.engine=stock"])
    p = sdf.write_world(cfg2, Layout(cfg2), visuals, collisions, "")
    w = _parse(p).find("world")
    assert w.find("plugin/engine/filename").text == "gz-physics-bullet-plugin"
    assert float(w.find("physics/max_step_size").text) == pytest.approx(0.004)


def test_friction_groups_go_to_separate_links(cfg, world):
    lay, visuals, collisions = world
    cols = collisions + [sdf.CollisionSpec("rock_hulls", "meshes/rock_hulls.glb", 0.6, 0.5)]
    p = sdf.write_world(cfg, lay, visuals, cols, "")
    links = {l.get("name"): l for l in _parse(p).find("world/model").findall("link")}
    assert set(links) == {"terrain", "terrain_mu_0p600_0p500"}
    assert len(links["terrain"].findall("collision")) == 2 and len(links["terrain"].findall("visual")) == 3
    ode = links["terrain_mu_0p600_0p500"].find("collision/surface/friction/ode")
    assert float(ode.find("mu").text) == 0.6 and float(ode.find("mu2").text) == 0.5


def test_missing_uris_abort_before_writing(cfg, world):
    lay, visuals, collisions = world
    bad = visuals + [sdf.VisualSpec("tile_09_09_ground", "meshes/tile_09_09_ground.glb", "regolith_core")]
    badc = collisions + [sdf.CollisionSpec("far_east_collision", "/nonexistent/far_east_collision.glb", 0.4)]
    with pytest.raises(FileNotFoundError) as exc:
        sdf.write_world(cfg, lay, bad, badc, "")
    msg = str(exc.value)
    assert "meshes/tile_09_09_ground.glb" in msg and "/nonexistent/far_east_collision.glb" in msg
    assert not lay.sdf_path("").exists()
    # textures are checked too (only for materials in use)
    (lay.world_dir / "textures/regolith_normal.png").unlink()
    with pytest.raises(FileNotFoundError, match="regolith_normal.png"):
        sdf.write_world(cfg, lay, visuals, collisions, "")
    assert sdf.write_world(cfg, lay, visuals, collisions, "", require_textures=False).is_file()


def test_uri_forms(tmp_path, monkeypatch):
    f = tmp_path / "m" / "a.glb"
    f.parent.mkdir()
    f.write_bytes(b"x")
    monkeypatch.setenv("GZ_SIM_RESOURCE_PATH", str(tmp_path))
    assert sdf.find_missing(["m/a.glb", str(f), f"file://{f}", "model://m/a.glb"], tmp_path) == []
    assert sdf.find_missing(["m/b.glb", "model://m/b.glb", "https://x/y.glb"], tmp_path) == [
        "m/b.glb", "model://m/b.glb", "https://x/y.glb"]


def test_sdf_rejects_bad_input(cfg, world):
    lay, visuals, collisions = world
    with pytest.raises(ValueError, match="duplicate"):
        sdf.write_world(cfg, lay, visuals + visuals[:1], collisions, "")
    with pytest.raises(ValueError, match="undefined materials"):
        sdf.write_world(cfg, lay, [sdf.VisualSpec("x", visuals[0].uri, "basalt")], collisions, "",
                        require_textures=False)


def test_material_diffuse_rule(cfg):
    mats = sdf.default_materials(cfg)
    assert tuple(mats) == sdf.MATERIAL_NAMES
    assert mats["regolith_far"].effective_diffuse() == sdf.TEXTURED_DIFFUSE
    plain = sdf.MaterialDef("plain", roughness=0.9, base_color=BASE)
    assert plain.effective_diffuse() == BASE + (1.0,)
    assert plain.effective_roughness() == pytest.approx(0.9)          # no map: the configured value
    assert mats["rocks"].effective_roughness() == 1.0 and mats["rocks"].roughness == pytest.approx(0.85)


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz CLI not on PATH")
def test_gz_sdf_check(cfg, world):
    lay, visuals, collisions = world
    for p in sdf.write_all_variants(cfg, lay, visuals, collisions).values():
        ok, out = sdf.validate_sdf(p)
        assert ok, out


# ------------------------------------------------------------------------------ spawn
RAMP_X0 = 3.0
RAMP_DEG = 12.5


def _synthetic():
    """30 x 30 m, 2 cm collision grid: flat (z = 0) for x < 3, a 12.5 deg ramp over
    3 <= x <= 10, plateau beyond; a 5 cm bump at (-8, -8)."""
    h = 0.02
    xs = np.arange(-15, 15 + 1e-9, h)
    X, Y = np.meshgrid(xs, xs)
    t = math.tan(math.radians(RAMP_DEG))
    Z = np.clip(X - RAMP_X0, 0, 7.0) * t
    Z[(np.abs(X + 8) <= 0.05) & (np.abs(Y + 8) <= 0.05)] = 0.05
    return Grid(Z.astype(np.float32), -15.0, -15.0, h)


def _rocks():
    """A 12 cm rock at the centre (blocks flat/turn there), a 7 + 6 cm pair 0.3 m apart (not
    lone), the 8 cm target at (-6, 5) and one ring rock (ignored)."""
    return {
        "id": np.array([1, 2, 3, 4, 5]),
        "zone": np.array([0, 0, 0, 0, 1]),
        "x": np.array([0.0, -3.0, -3.0, -6.0, 0.0]),
        "y": np.array([0.0, -3.0, -2.7, 5.0, 2.0]),
        "D": np.array([0.12, 0.07, 0.06, 0.08, 0.5]),
        "exposed_h": np.array([0.07, 0.04, 0.03, 0.05, 0.3]),
        "collide": np.array([1, 1, 1, 1, 0]),
    }


def _clearance(site, rocks, min_D=0.05):
    core = (rocks["zone"] == 0) & (rocks["D"] >= min_D)
    d = np.hypot(rocks["x"][core] - site.x, rocks["y"][core] - site.y) - rocks["D"][core] / 2
    return float(d.min())


@pytest.fixture(scope="module")
def sites_result():
    cfg = load_config()
    C = _synthetic()
    res = spawn.find_sites(cfg, C, _rocks(), height_fn=lambda x, y: C.sample(x, y) + 0.02,
                           region=Rect(-15, 15, -15, 15))
    return cfg, C, res


def test_spawn_z_rule():
    cfg = load_config()
    C = _synthetic()
    off = cfg.spawn.base_link_height_m + cfg.spawn.clearance_m
    assert spawn.footprint_radius(cfg) == pytest.approx(math.hypot(0.55, 0.40))
    assert spawn.spawn_z(C, -8.0 + 0.6, -8.0, cfg) == pytest.approx(0.05 + off, abs=1e-6)   # bump inside
    assert spawn.spawn_z(C, -8.0 + 0.9, -8.0, cfg) == pytest.approx(0.0 + off, abs=1e-6)    # bump outside
    # on the ramp: the highest point is the uphill edge of the disc
    t = math.tan(math.radians(RAMP_DEG))
    z = spawn.spawn_z(C, 6.0, 0.0, cfg)
    assert z == pytest.approx((6.0 + 0.68 - RAMP_X0) * t + off, abs=0.006)


def test_flat_site(sites_result):
    cfg, C, res = sites_result
    s = res.sites["flat"]
    # nearest 0.5 m lattice point to the centre clear of the 12 cm centre rock by r_fp + 0.3 m:
    # distance 1.0 fails (1.0 - 0.06 < 0.98), the ring at sqrt(1.25) = 1.118 m passes
    assert math.hypot(s.x, s.y) == pytest.approx(math.sqrt(1.25), abs=1e-6)
    assert _clearance(s, _rocks()) >= spawn.footprint_radius(cfg) + 0.3
    assert s.z == pytest.approx(0.176, abs=1e-6) and s.slope_deg < 0.01
    assert s.ground_z_visual == pytest.approx(0.02, abs=1e-6)
    assert not s.relaxed
    assert s.extra["path"]["rocks_ge_D_in_corridor"] == 0        # the yaw avoids the centre rock


def test_turn_site(sites_result):
    cfg, C, res = sites_result
    s = res.sites["turn"]
    # needs 1.5 m + 0.06 m from the centre rock: the 1.5 m points fail, sqrt(2.5) = 1.581 m passes
    assert math.hypot(s.x, s.y) == pytest.approx(math.sqrt(2.5), abs=1e-6)
    assert _clearance(s, _rocks()) >= 1.5


def test_turn_site_rock_threshold():
    """DESIGN 13 decision: the turn site needs no rock >= spawn.turn_rock_free_D_m (0.10 m) within
    1.5 m; smaller rocks only constrain the flat site (no rock >= 5 cm within r_fp + 0.3 m)."""
    cfg = load_config()
    assert cfg.spawn.turn_rock_free_D_m == pytest.approx(0.10)
    xs = np.arange(-10, 10 + 1e-9, 0.02)
    C = Grid(np.zeros((xs.size, xs.size), dtype=np.float32), -10, -10, 0.02)
    rocks = {"id": np.array([1, 2]), "zone": np.array([0, 0]), "x": np.array([0.0, 3.0]),
             "y": np.array([0.0, 0.0]), "D": np.array([0.07, 0.12]), "exposed_h": np.array([0.03, 0.06]),
             "collide": np.array([1, 1])}
    res = spawn.find_sites(cfg, C, rocks, region=Rect(-10, 10, -10, 10), names=["flat", "turn"])
    turn, flat = res.sites["turn"], res.sites["flat"]
    assert (turn.x, turn.y) == pytest.approx((0.0, 0.0))            # the 7 cm rock does not block the turn
    assert not turn.relaxed and turn.extra["criteria"]["rock_free_D_m"] == pytest.approx(0.10)
    assert np.hypot(turn.x - 3.0, turn.y) - 0.06 >= 1.5
    assert np.hypot(flat.x, flat.y) - 0.035 >= spawn.footprint_radius(cfg) + 0.3   # but it blocks flat
    # with the old 5 cm threshold the centre is not a turn site any more
    cfg5 = load_config(overrides=["spawn.turn_rock_free_D_m=0.05"])
    t5 = spawn.find_sites(cfg5, C, rocks, region=Rect(-10, 10, -10, 10), names=["turn"]).sites["turn"]
    assert np.hypot(t5.x, t5.y) - 0.035 >= 1.5


def test_slope_site(sites_result):
    cfg, C, res = sites_result
    s = res.sites["slope"]
    assert abs(s.yaw) < 1e-6                     # facing uphill (+x)
    assert RAMP_X0 - 0.01 <= s.x <= RAMP_X0 + 0.5 + 0.01
    run = s.extra["slope_run"]
    assert run["mean_slope_deg"] == pytest.approx(RAMP_DEG, abs=0.05)
    assert run["length_m"] >= 5.0
    t = math.tan(math.radians(RAMP_DEG))
    assert s.z == pytest.approx(max(0.0, s.x + 0.68 - RAMP_X0) * t + 0.176, abs=0.006)


def test_rock_site(sites_result):
    cfg, C, res = sites_result
    s = res.sites["rock"]
    r = s.extra["rock"]
    assert r["id"] == 4 and r["D"] == pytest.approx(0.08)    # the (-3, -3) pair is not lone
    c, sn = math.cos(s.yaw), math.sin(s.yaw)
    dx, dy = r["x"] - s.x, r["y"] - s.y
    assert dx * c + dy * sn == pytest.approx(2.0, abs=1e-6)            # 2 m ahead
    assert -dx * sn + dy * c == pytest.approx(0.2829, abs=1e-6)        # on the left wheel line
    assert s.z == pytest.approx(0.176, abs=1e-6)


def test_missing_sites_are_reported():
    cfg = load_config()
    xs = np.arange(-10, 10 + 1e-9, 0.02)
    C = Grid(np.zeros((xs.size, xs.size), dtype=np.float32), -10, -10, 0.02)
    res = spawn.find_sites(cfg, C, None, region=Rect(-10, 10, -10, 10))
    assert set(res.sites) == {"flat", "turn"}
    assert set(res.missing) == {"slope", "rock"}
    assert (res.sites["flat"].x, res.sites["flat"].y) == pytest.approx((0.0, 0.0))


def test_rock_table_inputs(tmp_path):
    rocks = _rocks()
    np.savez(tmp_path / "cat.npz", **rocks)
    t = spawn.RockTable.from_any(tmp_path / "cat.npz")
    assert len(t) == 4                            # ring rock dropped
    arr = np.zeros(2, dtype=[("x", "f8"), ("y", "f8"), ("D", "f8")])
    assert len(spawn.RockTable.from_any(arr)) == 2
    assert len(spawn.RockTable.from_any(None)) == 0


# ------------------------------------------------------------------------------ world_info
def test_world_info_roundtrip(tmp_path, sites_result):
    _, _, res = sites_result
    cfg = load_config(overrides=[f"paths.world_dir={tmp_path}"])
    lay = Layout(cfg)
    p = world_info.write_world_info(cfg, lay, res, triangle_counts={"core_ground": 123},
                                    rock_counts={"core": 5})
    assert p == tmp_path / "world_info.yaml"
    info = world_info.read_world_info(p)
    assert info["world_name"] == "apollo16_core" and info["schema_version"] == 1
    assert info["sdf_variants"] == {"default": "apollo16_core.sdf", "inspect": "apollo16_core_inspect.sdf",
                                    "stock": "apollo16_core_stock.sdf",
                                    "inspect_stock": "apollo16_core_inspect_stock.sdf"}
    assert info["origin"]["lat_deg"] == pytest.approx(-8.973448) and info["origin"]["href_m"] == 8.1955
    assert info["gravity_m_s2"] == 1.62 and info["physics"]["step_s"]["stock"] == 0.004
    assert info["triangle_counts"] == {"core_ground": 123} and info["rock_counts"] == {"core": 5}
    assert len(info["config_hash"]) == 16 and info["generator_version"]
    x, y, z, yaw = world_info.site_pose(info, "flat")
    f = res.sites["flat"]
    assert (x, y, z, yaw) == pytest.approx((f.x, f.y, f.z, f.yaw), abs=1e-4)
    raw = yaml.safe_load(p.read_text())          # plain YAML, as the launch file reads it
    assert set(raw["sites"]) == {"flat", "slope", "rock", "turn"}
    with pytest.raises(KeyError):
        world_info.site_pose(info, "summit")


def test_world_info_rejects_bad_files(tmp_path):
    p = tmp_path / "w.yaml"
    p.write_text("schema_version: 99\n")
    with pytest.raises(ValueError):
        world_info.read_world_info(p)
    p.write_text("schema_version: 1\nsites: {flat: {x: 1, y: 2}}\n")
    with pytest.raises(ValueError):
        world_info.read_world_info(p)


# ------------------------------------------------------------------------------ launch helpers
def _launch_module():
    spec = importlib.util.spec_from_file_location("apollo16_sim_launch", PKG / "launch" / "apollo16_sim.launch.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _mini_world(path: Path, engine: str) -> None:
    path.write_text(f'<?xml version="1.0"?><sdf version="1.11"><world name="w">'
                    f'<plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">'
                    f'<engine><filename>{engine}</filename></engine></plugin></world></sdf>')


def test_launch_resolve_world_and_fallback(tmp_path):
    m = _launch_module()
    plugin = tmp_path / "lib.so"
    _mini_world(tmp_path / "w.sdf", str(plugin))
    _mini_world(tmp_path / "w_inspect.sdf", str(plugin))
    _mini_world(tmp_path / "w_stock.sdf", "gz-physics-bullet-plugin")
    # patched plugin missing -> stock sibling
    p, notes = m.resolve_world(str(tmp_path / "w.sdf"), "")
    assert p.name == "w_stock.sdf" and notes
    p, notes = m.resolve_world(str(tmp_path / "w.sdf"), "inspect")
    assert p.name == "w_stock.sdf" and notes
    plugin.write_bytes(b"so")
    assert m.resolve_world(str(tmp_path / "w.sdf"), "")[0].name == "w.sdf"
    assert m.resolve_world(str(tmp_path / "w.sdf"), "inspect")[0].name == "w_inspect.sdf"
    assert m.resolve_world(str(tmp_path / "w_inspect.sdf"), "stock")[0].name == "w_stock.sdf"
    with pytest.raises(RuntimeError):
        m.resolve_world(str(tmp_path / "nope.sdf"), "")


def test_launch_resolve_pose(tmp_path):
    m = _launch_module()
    w = tmp_path / "w.sdf"
    _mini_world(w, "gz-physics-bullet-plugin")
    (tmp_path / "world_info.yaml").write_text(yaml.safe_dump(
        {"schema_version": 1, "sites": {"flat": {"x": 1.5, "y": -2.0, "z": 0.3, "yaw": 0.25}},
         "missing_sites": {"slope": "none"}}))
    assert m.resolve_pose(w, "", "flat", "", "", "", "")[0] == (1.5, -2.0, 0.3, 0.25)
    assert m.resolve_pose(w, "", "flat", "", "", "", "1.0")[0] == (1.5, -2.0, 0.3, 1.0)
    assert m.resolve_pose(w, "", "flat", "3", "4", "5", "")[0] == (3.0, 4.0, 5.0, 0.0)
    with pytest.raises(RuntimeError, match="slope"):
        m.resolve_pose(w, "", "slope", "", "", "", "")
    with pytest.raises(RuntimeError):
        m.resolve_pose(w, "", "flat", "3", "", "", "")


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash not available")
def test_shell_scripts_parse():
    for rel in ("scripts/run_apollo16_sim.sh", "tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh"):
        res = subprocess.run(["bash", "-n", str(PKG / rel)], capture_output=True, text=True)
        assert res.returncode == 0, res.stderr
    res = subprocess.run(["bash", str(PKG / "scripts/run_apollo16_sim.sh"), "--help"], capture_output=True, text=True)
    assert res.returncode == 0 and "--site" in res.stdout and "--headless" in res.stdout


# ------------------------------------------------------------------------------ plugin patch script
def _run_patch(src: Path) -> str:
    script = PKG / "tools" / "gz_physics_bullet_lunar" / "patch_bullet_lunar.py"
    res = subprocess.run(["python3", str(script), str(src)], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
    return res.stdout


def test_patch_script_on_synthetic_tree(tmp_path):
    spec = importlib.util.spec_from_file_location("pbl", PKG / "tools" / "gz_physics_bullet_lunar" / "patch_bullet_lunar.py")
    pbl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pbl)
    files: dict[str, str] = {}
    for rel, _tag, old, _new in pbl.PATCHES:
        files[rel] = files.get(rel, "// header\n") + old + "\n// filler\n"
    for rel, txt in files.items():
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / rel).write_text(txt)
    out1 = _run_patch(tmp_path)
    assert out1.count("applied") == len(pbl.PATCHES) and "already" not in out1
    out2 = _run_patch(tmp_path)
    assert out2.count("already applied") == len(pbl.PATCHES)
    assert (tmp_path / "bullet/src/ShapeFeatures.cc.orig").is_file()
    assert "meshFriction" in (tmp_path / "bullet/src/Base.hh").read_text()


TARBALL = Path("/home/nargon/togo_assets/lunar/build/gz_physics_bullet_lunar/gz-physics7_7.6.0.tar.gz")


@pytest.mark.skipif(not TARBALL.is_file(), reason="gz-physics 7.6.0 tarball not present")
def test_patch_script_on_real_source(tmp_path):
    prefix = "gz-physics-gz-physics7_7.6.0/bullet/src/"
    with tarfile.open(TARBALL) as tf:
        members = [m for m in tf.getmembers() if m.name.startswith(prefix) and
                   m.name[len(prefix):] in ("JointFeatures.cc", "Base.hh", "SDFFeatures.cc", "ShapeFeatures.cc")]
        assert len(members) == 4
        tf.extractall(tmp_path, members=members, filter="data")
    src = tmp_path / "gz-physics-gz-physics7_7.6.0"
    out = _run_patch(src)
    assert out.count(" applied") == 5
    shape = (src / "bullet/src/ShapeFeatures.cc").read_text()
    assert "setFriction(mu)" in shape and "#include <cmath>" in shape
    assert "parentAngVel" in (src / "bullet/src/JointFeatures.cc").read_text()
