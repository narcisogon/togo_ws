"""Tests for the documentation work package (docs/impl/fix_docs.md):

* U1: config/slam/seyond_dlio_graph_lunar.yaml is the user's DLIO config with only odom/gravity changed,
  and that value is the world's gravity;
* U2: requirements-venv.txt is fully pinned and matches the venv; README links and package paths exist,
  and the README documents exactly the options of scripts/run_apollo16_sim.sh (and of scripts/open_gui.sh);
* U6: config/gui/apollo16.gui.config is a valid gz-sim 8 GUI config (default plugins + ViewAngle, far clip
  that reaches the horizon, camera looking at the 'flat' spawn site of world_info.yaml).

Tests that need files outside this package (the user's SLAM config, the built world_info.yaml, the installed
default GUI config) skip when those are missing. Also: every test file belongs to one scripts/run_tests.sh group.

    cd src/lunar_world_gen && /home/nargon/.venvs/lunar_world/bin/python -m pytest tests/test_docs.py -q
"""
from __future__ import annotations

import hashlib
import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from lunar_world_gen.config import load_config

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]
LUNAR_DLIO = PKG / "config" / "slam" / "seyond_dlio_graph_lunar.yaml"
USER_DLIO = REPO / "lidarslam_ros2" / "lidarslam" / "param" / "seyond_dlio_graph.yaml"
GUI_CONFIG = PKG / "config" / "gui" / "apollo16.gui.config"
DEFAULT_GUI = Path("/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config")
README = PKG / "README.md"
GUIDE = PKG / "docs" / "USER_GUIDE.md"
REQUIREMENTS = PKG / "requirements-venv.txt"
WRAPPER = PKG / "scripts" / "run_apollo16_sim.sh"
OPEN_GUI = PKG / "scripts" / "open_gui.sh"
RUN_TESTS = PKG / "scripts" / "run_tests.sh"
DIRECT_DEPS = {"numpy", "scipy", "numba", "llvmlite", "rasterio", "pyproj", "trimesh", "pydelatin", "pyfqmr",
               "pygltflib", "matplotlib", "pyyaml", "tqdm", "pytest", "rosbags"}
DEFAULT_GUI_PLUGINS = {"MinimalScene", "EntityContextMenuPlugin", "GzSceneManager", "InteractiveViewControl",
                       "CameraTracking", "MarkerManager", "SelectEntities", "Spawn", "VisualizationCapabilities",
                       "WorldControl", "WorldStats", "Shapes", "Lights", "TransformControl", "Screenshot",
                       "CopyPaste", "ComponentInspector", "EntityTree"}


def _module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def dlio_gen():
    return _module(PKG / "config" / "slam" / "make_lunar_dlio_config.py", "make_lunar_dlio_config")


@pytest.fixture(scope="module")
def gui_gen():
    return _module(PKG / "config" / "gui" / "make_apollo16_gui_config.py", "make_apollo16_gui_config")


def _flatten(d: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(_flatten(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def _world_info(cfg) -> dict | None:
    p = Path(str(cfg.paths.world_dir)).expanduser() / "world_info.yaml"
    return yaml.safe_load(p.read_text()) if p.is_file() else None


def _split_header(text: str) -> tuple[str, str]:
    """(header, body) of a lunar DLIO copy: the header is framed by two '# ═══' lines."""
    lines = text.splitlines(keepends=True)
    marks = [i for i, line in enumerate(lines) if line.startswith("# ═")]
    assert len(marks) >= 2 and marks[0] == 0, "header frame lines missing"
    return "".join(lines[:marks[1] + 1]), "".join(lines[marks[1] + 1:])


def _canon(elem: ET.Element) -> bytes:
    return ET.tostring(elem).strip()          # tostring() includes the tail whitespace; ignore it


# ======================================================================================= U1: DLIO gravity
def test_lunar_dlio_gravity_is_world_gravity(cfg):
    params = yaml.safe_load(LUNAR_DLIO.read_text(encoding="utf-8"))["dlio_odom_node"]["ros__parameters"]
    g = params["odom/gravity"]
    assert isinstance(g, float), "odom/gravity must stay a YAML float (DLIO declares a double)"
    assert g == pytest.approx(float(cfg.physics.gravity_m_s2))
    text = LUNAR_DLIO.read_text(encoding="utf-8")
    assert "LUNAR-GRAVITY COPY" in text and "/home/er4-user/ws/src/lunar_world_gen/config/slam/" in text


@pytest.mark.skipif(not USER_DLIO.is_file(), reason="user's lidarslam_ros2 config not present")
def test_lunar_dlio_differs_from_user_config_only_in_gravity():
    a = _flatten(yaml.safe_load(USER_DLIO.read_text(encoding="utf-8")))
    b = _flatten(yaml.safe_load(LUNAR_DLIO.read_text(encoding="utf-8")))
    assert set(a) == set(b)
    differing = {k for k in a if a[k] != b[k]}
    assert differing <= {"dlio_odom_node.ros__parameters.odom/gravity"}, differing


@pytest.mark.skipif(not USER_DLIO.is_file(), reason="user's lidarslam_ros2 config not present")
def test_lunar_dlio_copy_is_up_to_date(dlio_gen):
    expected = dlio_gen.build(USER_DLIO)
    assert LUNAR_DLIO.read_text(encoding="utf-8") == expected, (
        "config/slam/seyond_dlio_graph_lunar.yaml is stale (the user's seyond_dlio_graph.yaml changed): "
        "run python3 src/lunar_world_gen/config/slam/make_lunar_dlio_config.py")
    # the body below the header is the source byte for byte except the gravity line
    _, body = _split_header(expected)
    src = USER_DLIO.read_text(encoding="utf-8").splitlines()
    new = body.splitlines()
    assert len(src) == len(new)
    changed = [i for i, (x, y) in enumerate(zip(src, new)) if x != y]
    assert len(changed) == 1 and "odom/gravity:" in new[changed[0]]


def test_lunar_dlio_generator_on_synthetic_source(dlio_gen):
    src = "# head\nnode:\n  ros__parameters:\n    a: 1\n    odom/gravity: 9.82\n    b: [1, 2]\n"
    out = dlio_gen.make_lunar(src, 1.62, hashlib.md5(src.encode()).hexdigest())
    header, body = _split_header(out)
    assert hashlib.md5(src.encode()).hexdigest() in header
    src_lines, body_lines = src.splitlines(), body.splitlines()
    assert len(src_lines) == len(body_lines)
    changed = [(a, b) for a, b in zip(src_lines, body_lines) if a != b]
    assert len(changed) == 1
    assert changed[0][0] == "    odom/gravity: 9.82" and changed[0][1].startswith("    odom/gravity: 1.62   # ")
    assert yaml.safe_load(out)["node"]["ros__parameters"] == {"a": 1, "odom/gravity": 1.62, "b": [1, 2]}
    assert "-8.20 m/s^2" in out                          # 1.62 - 9.82
    # an integer gravity stays a float, and the source must have exactly one gravity line
    assert yaml.safe_load(dlio_gen.make_lunar(src, 2.0, "x"))["node"]["ros__parameters"]["odom/gravity"] == 2.0
    with pytest.raises(ValueError):
        dlio_gen.make_lunar("a: 1\n", 1.62, "x")
    with pytest.raises(ValueError):
        dlio_gen.make_lunar(src + "    odom/gravity: 3.0\n", 1.62, "x")


def test_dlio_generator_reads_config_gravity(dlio_gen, cfg, tmp_path):
    assert dlio_gen.config_gravity() == pytest.approx(float(cfg.physics.gravity_m_s2))
    p = tmp_path / "c.yaml"
    p.write_text("world:\n  gravity_m_s2: 9.0\nphysics:\n  engine: x\n  gravity_m_s2: 3.71   # Mars\nlighting: {}\n")
    assert dlio_gen.config_gravity(p) == pytest.approx(3.71)
    assert dlio_gen.config_gravity(tmp_path / "missing.yaml") == pytest.approx(1.62)


# ======================================================================================= U2: venv, README
def test_requirements_fully_pinned():
    pins = {}
    for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        m = re.fullmatch(r"([A-Za-z0-9_.\-]+)==([A-Za-z0-9_.+\-]+)", line)
        assert m, f"not an exact pin: {line!r}"
        name = m.group(1).lower().replace("_", "-")
        assert name not in pins, f"duplicate {name}"
        pins[name] = m.group(2)
    assert DIRECT_DEPS <= set(pins), DIRECT_DEPS - set(pins)


@pytest.mark.skipif(".venvs/lunar_world" not in sys.prefix.replace("\\", "/"),
                    reason="only meaningful inside the project venv")
def test_requirements_match_project_venv():
    from importlib import metadata

    for line in REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        if line.strip() and not line.startswith("#"):
            name, ver = line.strip().split("==")
            assert metadata.version(name) == ver, f"{name}: venv has {metadata.version(name)}, file pins {ver}"


def _md_links(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    return [t for t in re.findall(r"\]\(([^)\s]+)\)", text) if not t.startswith(("http", "#", "mailto:"))]


@pytest.mark.parametrize("doc", [README, GUIDE], ids=["README", "USER_GUIDE"])
def test_markdown_links_exist(doc):
    missing = [t for t in _md_links(doc) if not (doc.parent / t.split("#")[0]).exists()]
    assert not missing, missing


@pytest.mark.parametrize("doc", [README, GUIDE], ids=["README", "USER_GUIDE"])
def test_package_paths_in_docs_exist(doc):
    text = doc.read_text(encoding="utf-8")
    rx = r"`(?:\$REPO/)?(?:src/lunar_world_gen/)?((?:scripts|tools|config|docs|launch|tests)/[A-Za-z0-9_./-]+)`"
    paths = {p.rstrip(".") for p in re.findall(rx, text)}
    for p in re.findall(r"(?:\$REPO/)?src/lunar_world_gen/((?:scripts|tools|config|docs|launch)/[A-Za-z0-9_./-]+)",
                        text):
        paths.add(p.rstrip("."))
    missing = sorted(p for p in paths if "*" not in p and not (PKG / p).exists())
    assert len(paths) >= 5 and not missing, missing


def _wrapper_options(script: Path = WRAPPER) -> set[str]:
    """The options listed in a script's header comment (its --help text)."""
    opts = set()
    for line in script.read_text(encoding="utf-8").splitlines():
        if not line.startswith("#"):
            if opts:
                break
            continue
        m = re.match(r"#\s+(-h \| )?(--[a-z][a-z-]*)", line)
        if m:
            opts.add(m.group(2))
    return opts


def test_readme_documents_the_open_gui_options():
    """README section 6 documents exactly the options of scripts/open_gui.sh (leak_script.md)."""
    opts = _wrapper_options(OPEN_GUI)
    assert {"--close", "--no-wait", "--default-gui", "--ready-timeout", "--keep-crash-dumps"} <= opts
    text = README.read_text(encoding="utf-8")
    section = text.split("## 6. Gazebo GUI", 1)[1].split("\n## 7.", 1)[0]
    documented = set(re.findall(r"`(--[a-z][a-z-]*)", section))
    assert opts - {"--help"} <= documented, f"undocumented open_gui.sh options: {sorted(opts - documented - {'--help'})}"
    used = set()
    for args in re.findall(r"open_gui\.sh([^`#|\n]*)", text):
        used |= set(re.findall(r"(?:^|\s)(--[a-z][a-z-]*)", args))
    assert "--close" in used
    assert used <= opts | {"--help"}, f"README uses options open_gui.sh does not have: {sorted(used - opts)}"


def test_readme_documents_the_wrapper_options():
    opts = _wrapper_options()
    assert {"--site", "--variant", "--headless", "--rviz"} <= opts
    text = README.read_text(encoding="utf-8")
    section = text.split("## 4. Running the simulation", 1)[1].split("\n## 5.", 1)[0]
    documented = set(re.findall(r"`(--[a-z][a-z-]*)", section))
    assert opts - {"--help"} <= documented, f"undocumented wrapper options: {sorted(opts - documented - {'--help'})}"
    used = set()                    # options written after the script name (until a comment, backtick or pipe)
    for args in re.findall(r"run_apollo16_sim\.sh([^`#|\n]*)", text):
        used |= set(re.findall(r"(?:^|\s)(--[a-z][a-z-]*)", args))
    assert {"--headless", "--site"} <= used
    assert used <= opts | {"--help"}, f"README uses options the wrapper does not have: {sorted(used - opts)}"


# ======================================================================================= U6: GUI config
def _parse_gui(path: Path) -> ET.Element:
    text = re.sub(r"^\s*<\?xml[^>]*\?>", "", path.read_text(encoding="utf-8"))
    return ET.fromstring(f"<root>{text}</root>")                  # gz-gui configs have several top-level elements


def test_gui_config_structure(cfg):
    text = GUI_CONFIG.read_text(encoding="utf-8")
    for comment in re.findall(r"<!--(.*?)-->", text, flags=re.S):
        assert "--" not in comment, "a double hyphen inside an XML comment makes the file invalid XML"
    root = _parse_gui(GUI_CONFIG)
    plugins = [p.get("filename") for p in root.findall("plugin")]
    assert len(plugins) == len(set(plugins))
    assert set(plugins) == DEFAULT_GUI_PLUGINS | {"ViewAngle"}
    ms = [p for p in root.findall("plugin") if p.get("filename") == "MinimalScene"][0]
    assert ms.findtext("engine") == "ogre2" and ms.findtext("scene") == "scene"
    near, far = float(ms.findtext("camera_clip/near")), float(ms.findtext("camera_clip/far"))
    half = float(cfg.zones.horizon.size_m) / 2.0
    assert 0.0 < near <= 0.5
    assert far >= math.hypot(half, half) + float(cfg.zones.core.size_m), "far clip must reach the horizon corners"
    pose = [float(v) for v in ms.findtext("camera_pose").split()]
    assert len(pose) == 6 and pose[3] == 0.0 and 0.0 < pose[4] < math.pi / 2


@pytest.mark.skipif(not DEFAULT_GUI.is_file(), reason="installed gz-sim 8 default GUI config not found")
def test_gui_config_keeps_the_installed_defaults():
    ours, default = _parse_gui(GUI_CONFIG), _parse_gui(DEFAULT_GUI)
    for tag in ("dialog", "window"):
        assert _canon(ours.find(tag)) == _canon(default.find(tag))
    mine = {p.get("filename"): _canon(p) for p in ours.findall("plugin")}
    for p in default.findall("plugin"):
        if p.get("filename") != "MinimalScene":
            assert mine[p.get("filename")] == _canon(p), p.get("filename")
    # MinimalScene: same children as the default apart from the values this file sets on purpose
    ms_ours = [p for p in ours.findall("plugin") if p.get("filename") == "MinimalScene"][0]
    ms_def = [p for p in default.findall("plugin") if p.get("filename") == "MinimalScene"][0]
    assert _canon(ms_ours.find("gz-gui")) == _canon(ms_def.find("gz-gui"))
    assert {c.tag for c in ms_ours} == {c.tag for c in ms_def} | {"camera_clip"}


def test_gui_camera_generator(gui_gen):
    site = {"x": -4.5, "y": -0.5, "z": -0.434, "yaw": math.radians(135.0), "ground_z_visual": -0.6222}
    x, y, z, roll, pitch, yaw = gui_gen.chase_pose(site)
    # the view axis passes through the aim point above the spawn
    d = math.hypot(site["x"] - x, site["y"] - y)
    assert d == pytest.approx(gui_gen.BEHIND)
    assert math.atan2(site["y"] - y, site["x"] - x) == pytest.approx(yaw)
    assert z - math.tan(pitch) * d == pytest.approx(site["ground_z_visual"] + gui_gen.AIM_DZ)
    info = {"sites": {"flat": site}, "lighting": {"sun_azimuth_deg": 84.66}}
    text = GUI_CONFIG.read_text(encoding="utf-8")
    new = gui_gen.apply(text, info)
    assert gui_gen.apply(new, info) == new                                 # idempotent
    assert gui_gen.read_pose(new) == pytest.approx([-0.964, -4.036, 1.578, 0.0, 0.3631, 2.3562], abs=1e-9)
    assert "130 deg to the right" in new
    # only the camera block changes
    other = {"sites": {"flat": dict(site, x=10.0)}, "lighting": {}}
    moved = gui_gen.apply(text, other)
    strip = lambda t: re.sub(r"<!-- START CAMERA.*?</camera_pose>", "", t, flags=re.S)  # noqa: E731
    assert strip(moved) == strip(text) and gui_gen.read_pose(moved)[0] == pytest.approx(10.0 + 3.536, abs=1e-3)


def test_every_test_file_has_a_group():
    """scripts/run_tests.sh picks its groups by file name: test_world_*, test_sim_*, test_slam_*, test_docs."""
    files = sorted(p.name for p in (PKG / "tests").glob("test_*.py"))
    stray = [f for f in files if f != "test_docs.py" and not f.startswith(("test_world_", "test_sim_", "test_slam_"))]
    assert not stray, f"name new test files test_world_*, test_sim_* or test_slam_* (run_tests.sh groups): {stray}"


@pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="needs bash (Linux)")
def test_run_tests_script_groups():
    run = lambda *a: subprocess.run(["bash", str(RUN_TESTS), *a], capture_output=True, text=True, timeout=30)  # noqa: E731
    res = run("--list")
    assert res.returncode == 0 and all(f"{g}:" in res.stdout for g in ("world", "sim", "slam", "docs"))
    assert "tests/test_slam_replay.py" in res.stdout and "quick leaves out" in res.stdout
    res = run("nope")
    assert res.returncode == 2 and "unknown group" in res.stderr
    res = run("--help")
    assert res.returncode == 0 and "quick" in res.stdout


def test_gui_camera_looks_at_flat_site_of_world_info(cfg, gui_gen):
    info = _world_info(cfg)
    if info is None or "flat" not in (info.get("sites") or {}):
        pytest.skip("no built world_info.yaml with a 'flat' site")
    expected = gui_gen.chase_pose(info["sites"]["flat"])
    got = gui_gen.read_pose(GUI_CONFIG.read_text(encoding="utf-8"))
    assert got == pytest.approx(list(expected), abs=2e-3), (
        "config/gui/apollo16.gui.config does not frame the current 'flat' site: run "
        "python3 src/lunar_world_gen/config/gui/make_apollo16_gui_config.py")
