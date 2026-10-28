"""End-to-end test of the integration build (lunar_world_gen/build.py) on a small layout.

Core 20 m (2 x 2 tiles of 10 m at 1 cm), ring 60 m, far 200 m, horizon 2 km, the real DTM and a
synthetic rock library. Every stage runs once; a second run must skip everything, a mesh-only
config change must re-run only the meshes stage and its downstream stages, and a stage whose
upstream is stale must refuse to run.

Run (WSL, project venv, from src/lunar_world_gen):
    ~/.venvs/lunar_world/bin/python -m pytest tests/test_world_build.py -q
"""
from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

from lunar_world_gen import build as B
from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid
from lunar_world_gen.layout import Layout
from lunar_world_gen.rocks.library import write_synthetic_library

MINI = [
    "zones.core.size_m=20.0", "zones.core.tile_size_m=10.0", "zones.core.margin_m=0.5",
    "zones.core.edge_fade_m=0.5", "zones.ring.size_m=60.0", "zones.ring.edge_fade_m=4.0",
    "zones.far.size_m=200.0", "zones.horizon.size_m=2000.0",
    "terrain.craters.D_max_m=4.0", "terrain.craters.fade_D_m=[2.0, 4.0]",
    "materials.regolith.texture_px=64",
]

_cfg0 = load_config()
HAVE_DTM = Path(_cfg0.paths.dtm).exists()
pytestmark = pytest.mark.skipif(not HAVE_DTM, reason="NAC_DTM_APOLLO16.TIF not available")


def mini_cfg(root: Path, *extra: str):
    return load_config(overrides=[f"paths.build_dir={root / 'build'}", f"paths.world_dir={root / 'world'}",
                                  f"paths.rock_library={root / 'lib'}", *MINI, *extra])


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    root = tmp_path_factory.mktemp("build_mini")
    write_synthetic_library(root / "lib", counts={"pebble": 4, "medium": 4, "large": 2}, seed=1)
    cfg = mini_cfg(root)
    logs: list[str] = []
    b = B.Builder(cfg, log=logs.append)
    records = b.run(B.STAGES)
    return root, cfg, records, logs


def test_all_stages_build(built):
    root, cfg, records, logs = built
    assert list(records) == list(B.STAGES)
    assert all(r["status"] in ("built", "skipped") for r in records.values()), records
    assert all(records[s]["status"] == "built" for s in B.STAGES if s != "terrain")
    lay = Layout(cfg)
    for s in B.STAGES[1:]:
        man = B._read_json(lay.build_dir / s / "manifest.json")
        assert man["status"] == "ok" and man["key"] and man["peak_rss_mb"] > 0 and "elapsed_s" in man
        assert all(Path(o["path"]).is_file() for o in man["outputs"])
    assert (lay.build_dir / "build_manifest.json").is_file()


def test_world_outputs(built):
    root, cfg, records, _ = built
    lay = Layout(cfg)
    for v in ("", "inspect", "stock", "inspect_stock"):
        assert lay.sdf_path(v).is_file()
    meshes = sorted(p.name for p in lay.meshes_dir.glob("*.glb"))
    for t in lay.tiles():
        for kind in ("ground", "rocks", "collision"):
            assert lay.tile_mesh_name(t, kind) in meshes
    for zone, kinds in (("ring", ("ground", "collision")), ("far", ("ground", "collision")), ("horizon", ("ground",))):
        for side in ("south", "north", "west", "east"):
            for kind in kinds:
                assert lay.strip_mesh_name(zone, side, kind) in meshes
    for tex in ("regolith_albedo.png", "regolith_normal.png", "regolith_roughness.png",
                "rock_atlas_basecolor.png", "rock_atlas_normal.png", "rock_atlas_roughness.png"):
        assert (lay.textures_dir / tex).is_file()
    assert (lay.world_dir / "rocks_catalog.npz").is_file() and (lay.world_dir / "rocks_catalog.csv").is_file()
    info = yaml.safe_load(lay.world_info_path.read_text())
    assert {"flat", "turn"} <= set(info["sites"])
    tc = info["triangle_counts"]
    assert tc["core_ground"] > 0 and tc["core_rocks"] > 0 and tc["core_collision"] > 0
    assert tc["visual_total"] == tc["core_ground"] + tc["core_rocks"] + tc["outer_visual"]
    gman = B._read_json(lay.build_dir / "ground" / "manifest.json")["results"]
    man = B._read_json(lay.build_dir / "meshes" / "manifest.json")["results"]
    assert gman["check_visual"]["ok"] and man["check_collision"]["ok"]
    assert gman["check_visual"]["mismatches"] == 0 and gman["check_visual"]["t_junctions"] == 0
    assert all(v["ok"] for k, v in man["budgets"].items() if v.get("hard", True))
    assert "rock_mu" not in info["physics"]                                    # CR-09
    assert info["sdf_variants"]["inspect_stock"] == lay.sdf_path("inspect_stock").name
    # the world catalog is the FINAL one (re-seated on the rendered ground), not the placed one
    cman = B._read_json(lay.build_dir / "contact" / "manifest.json")["results"]
    assert cman["enabled"] and cman["n_reseated"] > 0
    assert cman["core_reseated"]["after"]["hover_gt_1mm"] == 0 and cman["ring"]["after"]["hover_gt_1mm"] == 0
    assert cman["core_reseated"]["after"]["submerged"] == 0


def test_validation_ran(built):
    root, cfg, records, _ = built
    lay = Layout(cfg)
    rep = B._read_json(lay.reports_dir / "static_checks.json")
    ch = rep["checks"]
    for name in ("stitching", "glb", "sdf", "visual_vs_collision"):
        assert ch[name]["status"] == "pass", (name, ch[name].get("problems") or ch[name].get("reason"))
    # a 20 m core may lack a 10-15 deg slope or a lone 6-10 cm rock: such sites are reported missing
    info = yaml.safe_load(lay.world_info_path.read_text())
    missing = set(info.get("missing_sites") or {})
    assert all(p.startswith("site ") and p.split()[1] in missing for p in ch["world_info"]["problems"])
    assert ch["roughness"]["site_aware"] is True and ch["roughness"]["source"].endswith("tile grids")
    assert ch["visual_vs_collision"]["source"]["collision"] == "tile_*_collision.glb"
    assert (lay.reports_dir / "validation_report.md").is_file()


def test_integration_wiring(built):
    """Final integration: the ring (and its core-edge band) is sampled with avoid=core, so no ring rock
    interpenetrates a core rock (fix_rocks 9.1); the heights stage fillets exactly fillet_mask's rows and
    the contact stage keeps exactly those on H0 (fix_rocks 9.2); the ring collision uses the FID-08
    footprint dilation (default one node spacing)."""
    root, cfg, _, _ = built
    from lunar_world_gen.rocks.catalog import load_catalog
    from lunar_world_gen.rocks.fillets import fillet_mask
    from lunar_world_gen.rocks.library import RockLibrary
    from lunar_world_gen.rocks.population import find_conflicts

    lay = Layout(cfg)
    placed = load_catalog(lay.build_dir / "rocks" / "rocks_catalog.npz")
    core, ring = placed[placed["zone"] == 0], placed[placed["zone"] == 1]
    assert len(core) and len(ring)
    cols = ("x", "y", "z", "D", "scale", "proto", "lod", "qw", "qx", "qy", "qz")
    move, st = find_conflicts(cfg, {c: ring[c] for c in cols}, RockLibrary(cfg.paths.rock_library), fixed=core)
    assert not move.any(), st
    rman = B._read_json(lay.build_dir / "rocks" / "manifest.json")["results"]
    assert rman["overlap"]["ring_avoids_core"] is True
    hman = B._read_json(lay.build_dir / "heights" / "manifest.json")["results"]
    n_fil = int(fillet_mask(cfg, placed).sum())
    assert hman["filleted_rocks"] == n_fil
    cman = B._read_json(lay.build_dir / "contact" / "manifest.json")["results"]
    assert cman["n_filleted_kept"] == n_fil and cman["n_reseated"] == len(placed) - n_fil
    assert "core_light_burial" in cman and "core_perched" not in cman
    final = load_catalog(lay.build_dir / "contact" / "rocks_catalog.npz")
    kept = fillet_mask(cfg, placed)
    assert np.array_equal(final["z"][kept], placed["z"][kept])            # filleted rocks stay on H0
    col = B._read_json(lay.build_dir / "collision" / "manifest.json")["results"]
    assert col["ring_rock_dilation_nodes"] == 1.0
    assert col["ring_rock_dilation_half_width_m"] == pytest.approx(cfg.zones.ring.spacing_m)


def test_stale_gazebo_results_are_archived(built):
    """A rebuilt world must not be reported with the previous world's Gazebo results: result JSONs and the
    LiDAR capture folder older than the world's SDF move to reports/stale_before_<time>/ (kept, not
    deleted); newer files and other JSONs stay."""
    import json
    import os

    root, cfg, _, _ = built
    lay = Layout(cfg)
    rep = lay.reports_dir
    t_old = lay.sdf_path("").stat().st_mtime - 3600
    old = rep / "bench_world_old.json"
    old.write_text(json.dumps({"kind": "bench_world", "created": "2000-01-01T00:00:00"}))
    (rep / "lidar").mkdir(exist_ok=True)
    cap = rep / "lidar" / "capture_run.json"
    cap.write_text("{}")
    other = rep / "notes.json"
    other.write_text(json.dumps({"kind": "something_else"}))
    for p in (old, cap, other):
        os.utime(p, (t_old, t_old))
    rec = B.Builder(cfg, force=True, explicit_stages=["validate"], log=lambda *a: None).run(["validate"])
    assert rec["validate"]["status"] == "built"
    arch = B._read_json(lay.build_dir / "validate" / "manifest.json")["results"]["archived_stale_results"]
    assert sorted(arch["moved"]) == ["bench_world_old.json", "lidar"]
    dest = Path(arch["to"])
    assert dest.parent == rep and dest.name.startswith("stale_before_")
    assert (dest / "bench_world_old.json").is_file() and (dest / "lidar" / "capture_run.json").is_file()
    assert not old.exists() and not (rep / "lidar").exists() and other.is_file()
    assert "bench_world_old.json" not in (rep / "validation_report.md").read_text()
    # nothing stale: a second run moves nothing
    B.Builder(cfg, force=True, explicit_stages=["validate"], log=lambda *a: None).run(["validate"])
    assert B._read_json(lay.build_dir / "validate" / "manifest.json")["results"]["archived_stale_results"] == {}


def test_fillets_and_collision_tiles(built):
    root, cfg, records, _ = built
    lay = Layout(cfg)
    b = B.Builder(cfg)
    from lunar_world_gen.terrain import compose as tc

    for t in lay.tiles():
        H = Grid.load(b.heights_prefix(t))
        H0 = tc.load_core_tile(cfg, t, build=False)
        assert H.z.shape == H0.z.shape and np.all(H.z >= H0.z)          # fillets only add material
        C = Grid.load(b.collision_prefix(t))
        r = lay.tile_rect(t)
        assert (C.x0, C.x1, C.y0, C.y1) == pytest.approx((r.x0, r.x1, r.y0, r.y1))
        assert C.spacing == pytest.approx(cfg.mesh.collision.spacing_m)
    core = Grid.load(lay.build_dir / "collision" / "core_collision")
    assert core.nx == int(round(cfg.zones.core.size_m / cfg.mesh.collision.spacing_m)) + 1


def test_rebuild_is_cached_and_selective(built):
    root, cfg, _, _ = built
    again = B.Builder(cfg, log=lambda *a: None).run(B.STAGES)
    assert all(r["status"] == "skipped" for r in again.values()), again
    # a visual-ground change: the rendered ground changes, so the rocks are re-seated (contact),
    # their collision proxies move (collision) and everything downstream re-runs
    cfg2 = mini_cfg(root, "mesh.core_visual_max_error_m=0.008")
    b2 = B.Builder(cfg2, log=lambda *a: None)
    st = b2.status()
    assert all(st[s][0] for s in ("terrain", "rocks", "heights"))
    assert not any(st[s][0] for s in ("ground", "contact", "collision", "meshes", "export", "world", "validate"))
    # a downstream stage alone refuses to run on a stale upstream
    with pytest.raises(B.BuildError, match="ground"):
        b2.run(["export"])
    stages = ["ground", "contact", "collision", "meshes", "export", "world"]
    rec = b2.run(stages)
    assert [rec[s]["status"] for s in stages] == ["built"] * len(stages)
    # restore the original world (other tests in this module read it)
    B.Builder(cfg, log=lambda *a: None).run(B.STAGES)
    # a collision-only change (max edge refinement) re-runs meshes and downstream, not the ground
    cfg3 = mini_cfg(root, "mesh.collision.core_max_edge_m=0.5")
    st3 = B.Builder(cfg3, log=lambda *a: None).status()
    assert all(st3[s][0] for s in ("terrain", "rocks", "heights", "ground", "contact", "collision"))
    assert not st3["meshes"][0] and not st3["export"][0]


def test_forced_upstream_invalidates_downstream(built):
    """CR-11: keys carry the upstream run ids, so a forced re-run of a stage makes everything below it
    stale, and --force-downstream re-runs them in the same build."""
    root, cfg, _, _ = built
    b = B.Builder(cfg, force=True, explicit_stages=["world"], log=lambda *a: None)
    b.run(["world"])
    st = B.Builder(cfg, log=lambda *a: None).status()
    assert st["export"][0] and not st["validate"][0]
    rec = B.Builder(cfg, explicit_stages=["export"], force_downstream=True, log=lambda *a: None).run(["export"])
    assert [rec[s]["status"] for s in ("export", "world", "validate")] == ["built"] * 3
    assert B.downstream(["contact"]) == {"collision", "meshes", "export", "world", "validate"}


def test_world_dir_change_rebuilds_world_outputs(built, tmp_path):
    """CR-02: the output paths are part of the keys: a new world_dir re-runs the stages that write
    into it (instead of skipping them and leaving the new world dir empty)."""
    root, cfg, _, _ = built
    w2 = tmp_path / "w2"
    cfg2 = load_config(overrides=[f"paths.build_dir={root / 'build'}", f"paths.world_dir={w2}",
                                  f"paths.rock_library={root / 'lib'}", *MINI])
    st = B.Builder(cfg2, log=lambda *a: None).status()
    assert all(st[s][0] for s in ("terrain", "rocks", "heights", "ground"))
    assert not any(st[s][0] for s in ("contact", "export", "world", "validate"))
    B.Builder(cfg2, log=lambda *a: None).run(["contact", "collision", "meshes", "export", "world"])
    lay2 = Layout(cfg2)
    assert lay2.sdf_path().is_file() and (w2 / "rocks_catalog.npz").is_file()
    assert len(list(lay2.meshes_dir.glob("*.glb"))) == len(list(Layout(cfg).meshes_dir.glob("*.glb")))
    B.Builder(cfg, log=lambda *a: None).run(B.STAGES)       # restore the module's world


def test_preset_gets_own_dirs():
    """CR-02: a non-default preset no longer shares (and silently replaces) the full world's dirs."""
    full = load_config()
    fast = B.apply_preset_dirs(load_config(preset="fast"), None, [])
    assert fast.paths.build_dir == full.paths.build_dir + "_fast"
    assert fast.paths.world_dir == full.paths.world_dir + "_fast"
    same = B.apply_preset_dirs(load_config(preset="full"), None, [])
    assert same.paths.build_dir == full.paths.build_dir
    given = B.apply_preset_dirs(load_config(preset="fast", overrides=["paths.world_dir=/tmp/x"]), None,
                                ["paths.world_dir=/tmp/x"])
    assert given.paths.world_dir == "/tmp/x" and given.paths.build_dir.endswith("_fast")


def test_tile_cap_with_headroom_passes_hard_budget(built):
    """CR-04: a capped tile plus pydelatin's overshoot and the stitching inserts stays within the
    hard per-tile limit (before: cap 2000 -> 2013 after stitching -> BuildError)."""
    root, cfg, _, _ = built
    lay = Layout(cfg)
    gman = B._read_json(lay.build_dir / "ground" / "manifest.json")["results"]
    smallest = min(p["n_triangles"] for p in gman["pieces"] if p["zone"] == "core")
    cap = int(smallest * 0.8)
    cfg_c = mini_cfg(root, f"budgets.core_tile_visual_max_tris={cap}")
    rec = B.Builder(cfg_c, log=lambda *a: None).run(["ground", "contact", "collision", "meshes"])
    assert rec["meshes"]["status"] == "built"
    g2 = B._read_json(lay.build_dir / "ground" / "manifest.json")["results"]
    assert len(g2["capped_tiles"]) == len(lay.tiles())
    m2 = B._read_json(lay.build_dir / "meshes" / "manifest.json")["results"]
    assert m2["budgets"]["core_tile_visual_max_tris"]["ok"]
    assert m2["counts"]["core_per_tile"]["ground"]["max"] <= cap
    B.Builder(cfg, log=lambda *a: None).run(B.STAGES)


def test_core_ground_and_tile_heights(built):
    root, cfg, _, _ = built
    from lunar_world_gen.terrain import compose as tc

    g = B.assemble_core_with_margin(cfg)
    half = cfg.zones.core.size_m / 2 + cfg.zones.core.margin_m
    assert (g.x0, g.x1) == pytest.approx((-half, half))
    ground = B.CoreGround(cfg, g)
    x = np.array([0.123, -9.87, half + 0.7])                  # the last point is beyond the margin
    y = np.array([4.56, 3.21, 0.0])
    ref = tc.height_at(cfg, x, y, "core").astype(np.float64)
    assert np.allclose(ground.height(x, y), ref, atol=2e-4)
    assert ground.normal(x[:1], y[:1])[0, 2] > 0.5
    b = B.Builder(cfg)
    th = B.TileHeights(cfg, b.heights_prefix)
    assert np.allclose(th(x[:2], y[:2]), ref[:2], atol=0.05)    # + fillets (cm) at most


def test_stale_world_glbs_are_removed(built):
    root, cfg, _, _ = built
    lay = Layout(cfg)
    junk = lay.meshes_dir / "tile_99_99_ground.glb"
    junk.write_bytes(b"x")
    shutil.copy(junk, lay.meshes_dir / "old.glb.tmp")
    b = B.Builder(cfg, force=True, explicit_stages=["export"], log=lambda *a: None)
    b.run(["export"])
    assert not junk.exists() and not (lay.meshes_dir / "old.glb.tmp").exists()
