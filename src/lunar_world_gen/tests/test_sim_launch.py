"""Runtime fixes in launch/apollo16_sim.launch.py (fix_runtime.md): plugin loadability check and
stock fallback (U7, U11), robot collision primitives (gz_validation 5.1), GUI camera per spawn (U6),
the gz guard that stops gz with SIGKILL (U5), and the ready marker that scripts/run_apollo16_sim.sh
waits for before it opens the Gazebo GUI (leak_script.md)."""
from __future__ import annotations

import hashlib
import importlib.util
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
LAUNCH = PKG / "launch" / "apollo16_sim.launch.py"
SPEC = PKG / "config" / "robot" / "a300_collision_simplified.yaml"


def _launch_module():
    spec = importlib.util.spec_from_file_location("apollo16_sim_launch_rt", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


m = _launch_module()


def _mini_world(path: Path, engine: str) -> None:
    path.write_text(f'<?xml version="1.0"?><sdf version="1.11"><world name="w">'
                    f'<plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">'
                    f'<engine><filename>{engine}</filename></engine></plugin></world></sdf>')


def _worlds(tmp_path: Path, plugin: Path, inspect_stock: bool = True) -> None:
    _mini_world(tmp_path / "w.sdf", str(plugin))
    _mini_world(tmp_path / "w_inspect.sdf", str(plugin))
    _mini_world(tmp_path / "w_stock.sdf", "gz-physics-bullet-plugin")
    if inspect_stock:
        _mini_world(tmp_path / "w_inspect_stock.sdf", "gz-physics-bullet-plugin")


# ------------------------------------------------------------------------------ world resolution
def test_variant_names_and_errors(tmp_path):
    plugin = tmp_path / "lib.so"
    plugin.write_bytes(b"x")
    _worlds(tmp_path, plugin)
    w = str(tmp_path / "w.sdf")
    assert m.resolve_world(w, "inspect_stock")[0].name == "w_inspect_stock.sdf"
    assert m.resolve_world(str(tmp_path / "w_inspect_stock.sdf"), "inspect")[0].name == "w_inspect.sdf"
    assert m.resolve_world(str(tmp_path / "w_inspect_stock.sdf"), "")[0].name == "w_inspect_stock.sdf"
    assert m.resolve_world(str(tmp_path / "w_stock.sdf"), "stock")[0].name == "w_stock.sdf"
    with pytest.raises(RuntimeError, match="unknown variant"):
        m.resolve_world(w, "bogus")
    with pytest.raises(RuntimeError, match="lunar_world_gen.build"):   # U10: next step in the message
        m.resolve_world(str(tmp_path / "nope.sdf"), "")


def test_missing_plugin_inspect_keeps_lighting(tmp_path):
    """U11: inspect + missing plugin -> <stem>_inspect_stock.sdf when it exists, else _stock + note."""
    plugin = tmp_path / "missing.so"
    _worlds(tmp_path, plugin)
    p, notes = m.resolve_world(str(tmp_path / "w.sdf"), "inspect")
    assert p.name == "w_inspect_stock.sdf"
    assert any("falling back to w_inspect_stock.sdf" in n and "build_lunar_bullet_plugin.sh" in n for n in notes)
    assert any("45-66 %" in n for n in notes)          # consequences named
    p, _ = m.resolve_world(str(tmp_path / "w.sdf"), "")
    assert p.name == "w_stock.sdf"
    (tmp_path / "w_inspect_stock.sdf").unlink()
    p, notes = m.resolve_world(str(tmp_path / "w_inspect.sdf"), "")
    assert p.name == "w_stock.sdf"
    assert any("inspect lighting is not available" in n for n in notes)
    (tmp_path / "w_stock.sdf").unlink()
    with pytest.raises(RuntimeError, match="build_lunar_bullet_plugin.sh"):
        m.resolve_world(str(tmp_path / "w.sdf"), "")


def test_unloadable_plugin_garbage_falls_back(tmp_path):
    """U7: a present but broken plugin is detected (dlopen) and the stock variant is used."""
    plugin = tmp_path / "lib.so"
    plugin.write_bytes(b"not an ELF file at all" * 10)
    _worlds(tmp_path, plugin)
    ok, why = m.plugin_loadable(str(plugin))
    assert ok is False and why
    p, notes = m.resolve_world(str(tmp_path / "w.sdf"), "", check_plugin=True)
    assert p.name == "w_stock.sdf"
    assert any("cannot be loaded" in n for n in notes)
    # without the check (old behaviour, used by the generic tests) the file only has to exist
    assert m.resolve_world(str(tmp_path / "w.sdf"), "")[0].name == "w.sdf"


def _gcc(tmp: Path, name: str, code: str, extra: list[str] | None = None) -> Path:
    src = tmp / f"{name}.c"
    src.write_text(code)
    out = tmp / f"lib{name}.so"
    subprocess.run(["gcc", "-shared", "-fPIC", "-o", str(out), str(src), *(extra or [])], check=True,
                   capture_output=True)
    return out


@pytest.mark.skipif(shutil.which("gcc") is None or not sys.platform.startswith("linux"), reason="needs gcc / Linux")
def test_plugin_loadable_real_elf_cases(tmp_path):
    good = _gcc(tmp_path, "good", "int f(void){return 1;}\n")
    assert m.plugin_loadable(str(good)) == (True, "")
    # a NEEDED library that disappears (e.g. libBulletDynamics after a distro upgrade)
    dep = _gcc(tmp_path, "dep", "int g(void){return 2;}\n")
    needs = _gcc(tmp_path, "needs", "int g(void); int f(void){return g();}\n", [f"-L{tmp_path}", "-ldep"])
    dep.unlink()
    ok, why = m.plugin_loadable(str(needs))
    assert ok is False and "libdep.so" in why
    # an undefined symbol (ABI change of a dependency): only RTLD_NOW finds it at load time
    undef = _gcc(tmp_path, "undef", "int nothere(void); int f(void){return nothere();}\n")
    ok, why = m.plugin_loadable(str(undef))
    assert ok is False and "nothere" in why
    _worlds(tmp_path, needs)
    p, notes = m.resolve_world(str(tmp_path / "w.sdf"), "", check_plugin=True)
    assert p.name == "w_stock.sdf" and any("libdep.so" in n for n in notes)
    _worlds(tmp_path, good)
    assert m.resolve_world(str(tmp_path / "w.sdf"), "", check_plugin=True) == (tmp_path / "w.sdf", [])


def test_resolve_pose_messages(tmp_path):
    w = tmp_path / "w.sdf"
    _mini_world(w, "gz-physics-bullet-plugin")
    with pytest.raises(RuntimeError, match="--pose X Y Z"):        # U10: says what to do in the wrapper too
        m.resolve_pose(w, "", "flat", "", "", "", "")
    with pytest.raises(RuntimeError, match="numbers"):
        m.resolve_pose(w, "", "flat", "1", "x", "3", "")


# ------------------------------------------------------------------------------ rotations
def test_rpy_roundtrip_and_gimbal_lock():
    rnd = random.Random(3)
    for _ in range(300):
        r, p, y = rnd.uniform(-math.pi, math.pi), rnd.uniform(-1.5, 1.5), rnd.uniform(-math.pi, math.pi)
        R = m._rpy_matrix(r, p, y)
        R2 = m._rpy_matrix(*m._matrix_rpy(R))
        assert max(abs(R[i][j] - R2[i][j]) for i in range(3) for j in range(3)) < 1e-9
    for p in (math.pi / 2, -math.pi / 2):
        R = m._rpy_matrix(0.4, p, -0.7)
        R2 = m._rpy_matrix(*m._matrix_rpy(R))
        assert max(abs(R[i][j] - R2[i][j]) for i in range(3) for j in range(3)) < 1e-9


# ------------------------------------------------------------------------------ collision substitution
URDF = """<?xml version="1.0" ?>
<!-- generated -->
<robot name="r" xmlns:gz="http://gazebosim.org/schema">
  <link name="motor">
    <visual><origin rpy="0 0 3.14159"/><geometry><mesh filename="package://p/m.stl"/></geometry></visual>
    <collision><geometry><mesh filename="package://p/m.stl"/></geometry></collision>
  </link>
  <link name="lidar">
    <inertial><mass value="1"/></inertial>
    <collision name="c"><origin xyz="1 2 3" rpy="0.3 -0.2 1.1"/>
      <geometry><mesh filename="package://p/l.stl"/></geometry></collision>
  </link>
  <link name="scaled"><collision><geometry><mesh filename="package://p/m.stl" scale="0.5 0.5 0.5"/></geometry>
  </collision></link>
  <link name="small"><collision><geometry><mesh filename="package://p/s.stl"/></geometry></collision></link>
  <joint name="j" type="fixed"><parent link="motor"/><child link="lidar"/></joint>
  <gazebo><plugin filename="x" name="y"/></gazebo>
</robot>
"""


def _spec(files: dict | None = None) -> dict:
    files = files or {}
    return {"schema_version": 1, "meshes": [
        {"uri": "package://p/m.stl", "scale": [1, 1, 1], "triangles": 46604, **files.get("m", {}),
         "primitive": {"type": "box", "xyz": [0.0, -0.055, -0.0005], "rpy": [0, 0, 0], "size": [0.126, 0.126, 0.125]}},
        {"uri": "package://p/l.stl", "scale": [1.0, 1.0, 1.0], "triangles": 173872, **files.get("l", {}),
         "primitive": {"type": "cylinder", "axis": "y", "xyz": [0.05, 0.04, 0.06], "rpy": [-math.pi / 2, 0, 0],
                       "radius": 0.05, "length": 0.12}},
    ]}


def _origin(col):
    o = col.find("origin")
    xyz = [float(v) for v in o.get("xyz").split()]
    rpy = [float(v) for v in o.get("rpy").split()]
    return xyz, rpy


def _to_frame(xyz, rpy, p):
    R = m._rpy_matrix(*rpy)
    return [xyz[i] + sum(R[i][k] * p[k] for k in range(3)) for i in range(3)]


def _from_frame(xyz, rpy, p):
    R = m._rpy_matrix(*rpy)
    d = [p[i] - xyz[i] for i in range(3)]
    return [sum(R[k][i] * d[k] for k in range(3)) for i in range(3)]   # R^T d


def test_simplify_collisions_frames_and_scope():
    import xml.etree.ElementTree as ET

    out, rep = m.simplify_collisions(URDF, _spec(), resolve=None)
    root = ET.fromstring(out)
    links = {ln.get("name"): ln for ln in root.iter("link")}
    # motor: box at the mesh-frame centre (no origin before -> one is created), visual untouched
    col = links["motor"].find("collision")
    assert col.find("geometry/box").get("size") == "0.126 0.126 0.125"
    xyz, rpy = _origin(col)
    assert xyz == pytest.approx([0, -0.055, -0.0005]) and rpy == pytest.approx([0, 0, 0])
    assert links["motor"].find("visual/geometry/mesh").get("filename") == "package://p/m.stl"
    # lidar: cylinder; the primitive pose is composed with the collision origin
    col = links["lidar"].find("collision")
    assert col.get("name") == "c" and col.find("geometry/mesh") is None
    cyl = col.find("geometry/cylinder")
    assert float(cyl.get("radius")) == pytest.approx(0.05) and float(cyl.get("length")) == pytest.approx(0.12)
    xyz, rpy = _origin(col)
    o_xyz, o_rpy = [1, 2, 3], [0.3, -0.2, 1.1]
    assert xyz == pytest.approx(_to_frame(o_xyz, o_rpy, [0.05, 0.04, 0.06]), abs=1e-8)   # 9 digits in the XML
    # points of a cylinder along the MESH y axis, placed by the original origin, are inside the new
    # cylinder (local z axis) placed by the new origin
    rnd = random.Random(1)
    for _ in range(500):
        a, rr, h = rnd.uniform(0, 2 * math.pi), 0.05 * math.sqrt(rnd.random()), rnd.uniform(-0.06, 0.06)
        p_mesh = [0.05 + rr * math.cos(a), 0.04 + h, 0.06 + rr * math.sin(a)]
        q = _from_frame(xyz, rpy, _to_frame(o_xyz, o_rpy, p_mesh))
        assert math.hypot(q[0], q[1]) <= 0.05 + 1e-7 and abs(q[2]) <= 0.06 + 1e-7
    # other scale / not in the spec: unchanged; inertial, joint, gazebo, comments kept
    assert links["scaled"].find("collision/geometry/mesh") is not None
    assert links["small"].find("collision/geometry/mesh") is not None
    assert links["lidar"].find("inertial/mass").get("value") == "1"
    assert root.find("joint") is not None and root.find("gazebo/plugin") is not None
    assert "<!-- generated -->" in out and "package://p/l.stl (173872 triangles) -> 1 primitive" in out
    assert len(rep["replaced"]) == 2 and rep["triangles_removed"] == 46604 + 173872
    assert rep["primitives"] == {"box": 1, "cylinder": 1}


def test_simplify_collisions_keeps_changed_meshes(tmp_path):
    mf = tmp_path / "m.stl"
    mf.write_bytes(b"mesh-v1")
    good = {"bytes": mf.stat().st_size, "sha256": hashlib.sha256(b"mesh-v1").hexdigest()}
    spec = _spec({"m": good, "l": {"bytes": 1, "sha256": "0" * 64}})
    resolve = {"package://p/m.stl": mf, "package://p/l.stl": None}.get
    out, rep = m.simplify_collisions(URDF, spec, resolve=resolve)
    assert [r["link"] for r in rep["replaced"]] == ["motor"]
    assert rep["kept_changed"] and rep["kept_changed"][0]["link"] == "lidar"
    assert "not found" in rep["kept_changed"][0]["why"]
    mf.write_bytes(b"mesh-v2")                                # same size, other content
    out, rep = m.simplify_collisions(URDF, spec, resolve=resolve)
    assert rep["replaced"] == [] and "sha256" in rep["kept_changed"][0]["why"]


def test_robot_description_modes(tmp_path):
    xml, lines = m.robot_description_xml(URDF, "original", str(tmp_path / "none.yaml"))
    assert xml == URDF and "original" in lines[0]
    xml, lines = m.robot_description_xml(URDF, "simplified", str(tmp_path / "none.yaml"))
    assert xml == URDF and lines[0].startswith("WARNING")     # missing spec: never breaks the launch
    with pytest.raises(RuntimeError):
        m.robot_description_xml(URDF, "fast", str(SPEC))


def test_checked_in_spec_is_valid():
    spec = m.load_collision_spec(SPEC)
    by_name = {e["uri"].rsplit("/", 1)[-1]: e for e in spec["meshes"]}
    assert {"motor.stl", "seyond_robin_w.stl", "suspension_beam.stl"} <= set(by_name)
    assert by_name["motor.stl"]["triangles"] == 46604 and len(by_name["motor.stl"]["used_by"]) == 4
    assert by_name["seyond_robin_w.stl"]["triangles"] == 173872
    assert all(e["triangles"] >= spec["min_triangles"] for e in spec["meshes"])
    assert all(k["triangles"] < spec["min_triangles"] for k in spec["kept"])
    assert spec["summary"]["mesh_triangles_after"] == sum(k["triangles"] * len(k["used_by"]) for k in spec["kept"])
    for e in spec["meshes"]:
        p = e["primitive"]
        parts = p["parts"] if p["type"] == "compound" else [p]
        assert all(q["type"] in ("box", "cylinder") for q in parts) and parts
        lo, hi = e["bounds_min"], e["bounds_max"]
        boxes = [q for q in parts if q["type"] == "box"]
        if len(boxes) == len(parts):   # the boxes together cover the recorded mesh bounds
            for i in range(3):
                assert min(q["xyz"][i] - q["size"][i] / 2 for q in boxes) <= lo[i] + 1e-9
                assert max(q["xyz"][i] + q["size"][i] / 2 for q in boxes) >= hi[i] - 1e-9
        # the clearance given away towards the ground is recorded and bounded
        if "down_gap_mm" in p:
            assert p["down_gap_mm"] <= max(spec.get("max_down_gap_mm", 15.0), p.get("down_gap_single_box_mm", 0)) + 1e-9
    s = spec["summary"]
    assert s["mesh_triangles_before"] == 368456 and s["mesh_triangles_after"] <= 2160
    assert s["replaced_collisions"] + sum(len(k["used_by"]) for k in spec["kept"]) == s["collision_meshes"] == 14


@pytest.mark.skipif(not Path("/opt/ros/jazzy/share/clearpath_platform_description").is_dir(),
                    reason="Clearpath meshes not installed")
def test_checked_in_spec_matches_installed_meshes():
    """The precomputed primitives still bound the installed meshes (sha256 + containment)."""
    trimesh = pytest.importorskip("trimesh")
    import numpy as np

    sys.path.insert(0, str(PKG / "tools"))
    import robot_collision_simplify as rcs

    spec = m.load_collision_spec(SPEC)
    for e in spec["meshes"]:
        path = Path(e["file"])
        assert hashlib.sha256(path.read_bytes()).hexdigest() == e["sha256"], path
        v = np.asarray(trimesh.load(str(path), force="mesh", process=False).vertices) * np.asarray(e["scale"])
        assert rcs.primitive_contains(e["primitive"], v), path


# ------------------------------------------------------------------------------ GUI camera per spawn
GUI = "<window/><plugin filename='MinimalScene'><camera_pose>-0.964 -4.036 1.578 0 0.3631 2.3562</camera_pose></plugin>"


def test_gui_config_follows_spawn(tmp_path):
    w = tmp_path / "w.sdf"
    _mini_world(w, "gz-physics-bullet-plugin")
    (tmp_path / "world_info.yaml").write_text(yaml.safe_dump({"sites": {
        "flat": {"x": -4.5, "y": -0.5, "z": -0.434, "yaw": 2.356194},
        "slope": {"x": 11.0, "y": -4.0, "z": -2.5397, "yaw": -2.748894}}}))
    same, note = m.gui_config_for_pose(GUI, w, "", (-4.5, -0.5, -0.434, 2.356194))
    assert same == GUI and "as in the file" in note
    out, note = m.gui_config_for_pose(GUI, w, "", (11.0, -4.0, -2.5397, -2.748894))
    import re
    cam = [float(v) for v in re.search(r"<camera_pose>([^<]*)<", out).group(1).split()]
    # same distance and height above the spawn, same pitch, same bearing relative to the rover's heading
    ref = [-0.964, -4.036, 1.578, 0, 0.3631, 2.3562]
    d_ref = math.hypot(ref[0] + 4.5, ref[1] + 0.5)
    assert math.hypot(cam[0] - 11.0, cam[1] + 4.0) == pytest.approx(d_ref, abs=1e-3)
    assert cam[2] - (-2.5397) == pytest.approx(ref[2] - (-0.434), abs=1e-3)
    assert cam[4] == pytest.approx(0.3631)
    bearing = lambda cx, cy, x, y, yaw: math.remainder(math.atan2(cy - y, cx - x) - yaw, 2 * math.pi)  # noqa: E731
    assert bearing(cam[0], cam[1], 11.0, -4.0, -2.748894) == pytest.approx(
        bearing(ref[0], ref[1], -4.5, -0.5, 2.356194), abs=1e-4)
    assert math.remainder(cam[5] - (-2.748894), 2 * math.pi) == pytest.approx(
        math.remainder(ref[5] - 2.356194, 2 * math.pi), abs=1e-4)
    assert "turned" in note
    assert m.gui_config_for_pose("<x/>", w, "", (0, 0, 0, 0))[0] == "<x/>"
    (tmp_path / "world_info.yaml").unlink()
    assert m.gui_config_for_pose(GUI, w, "", (0, 0, 0, 0))[0] == GUI     # no world_info: unchanged


# ------------------------------------------------------------------------------ gz guard (U5)
def _guard(args: list[str], **kw) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, str(LAUNCH), "--gz-guard", "--", *args], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True, **kw)


def _children(pid: int) -> list[int]:
    try:
        return [int(c) for c in Path(f"/proc/{pid}/task/{pid}/children").read_text().split()]
    except OSError:
        return []


def _alive(pid: int) -> bool:
    try:
        st = Path(f"/proc/{pid}/stat").read_text()
        return st[st.rindex(")") + 2] != "Z"
    except OSError:
        return False


def _wait(cond, timeout=10.0):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        if cond():
            return True
        time.sleep(0.02)
    return False


pytestmark_linux = pytest.mark.skipif(not Path("/proc/self/task").is_dir(), reason="needs Linux /proc")


@pytestmark_linux
@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_gz_guard_kills_child_with_sigkill(tmp_path, sig):
    marker = tmp_path / "child_got_signal"
    # the "server" records any INT/TERM it receives; it must never get one, only SIGKILL
    child = f"trap 'echo got > {marker}' INT TERM HUP; while :; do sleep 0.05; done"
    g = _guard(["bash", "-c", child])
    assert _wait(lambda: _children(g.pid)), "guard did not start the child"
    kid = _children(g.pid)[0]
    time.sleep(0.3)
    t0 = time.monotonic()
    os.kill(g.pid, sig)
    rc = g.wait(timeout=10)
    out = g.stdout.read()
    assert rc == 0, out
    assert _wait(lambda: not _alive(kid), 5.0)
    assert time.monotonic() - t0 < 3.0
    assert not marker.exists()
    assert "stopped with SIGKILL" in out


@pytestmark_linux
def test_gz_guard_exit_codes():
    g = _guard(["bash", "-c", "exit 3"])
    assert g.wait(timeout=10) == 3
    g = _guard(["bash", "-c", "kill -USR1 $$"])      # died of a signal -> 128 + n (USR1: no core dump)
    assert g.wait(timeout=10) == 128 + signal.SIGUSR1
    g = _guard(["/nonexistent/gz"])
    assert g.wait(timeout=10) == 127


@pytestmark_linux
def test_gz_guard_child_dies_with_guard():
    """SIGKILL of the guard (e.g. the launch group killed by a tool): the kernel kills gz (PDEATHSIG)."""
    g = _guard(["sleep", "60"])
    assert _wait(lambda: _children(g.pid))
    kid = _children(g.pid)[0]
    os.kill(g.pid, signal.SIGKILL)
    g.wait(timeout=5)
    assert _wait(lambda: not _alive(kid), 5.0)


@pytestmark_linux
def test_gz_guard_parent_exit_kills_child():
    """ros2 launch itself SIGKILLed: the guard notices (reparenting) and kills gz."""
    p = subprocess.Popen(["bash", "-c", f"{sys.executable} {LAUNCH} --gz-guard -- sleep 60 & wait"],
                         start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    assert _wait(lambda: _children(p.pid))
    guard = _children(p.pid)[0]
    assert _wait(lambda: _children(guard))
    kid = _children(guard)[0]
    os.kill(p.pid, signal.SIGKILL)
    p.wait(timeout=5)
    assert _wait(lambda: not _alive(kid), 5.0)
    assert _wait(lambda: not _alive(guard), 5.0)


def test_launch_declares_new_arguments():
    import re

    names = re.findall(r"DeclareLaunchArgument\(\s*['\"]([\w-]+)['\"]", LAUNCH.read_text())
    for n in ("robot_collision", "robot_collision_spec", "gz_guard", "check_plugin", "ready_timeout",
              "world", "variant", "site", "robot_x", "robot_yaw", "headless_rendering", "ready_file"):
        assert n in names


# ------------------------------------------------------------------------------ ready marker (leak_script.md)
def _kv(path: Path) -> dict:
    return dict(ln.split("=", 1) for ln in path.read_text().splitlines())


def test_ready_marker_paths(tmp_path, monkeypatch):
    for empty in ("", "  ", None):
        assert m.ready_marker_paths(empty) is None
    r, n = m.ready_marker_paths(str(tmp_path / "ready"))
    assert r == tmp_path / "ready" and n == tmp_path / "ready.not_ready"
    assert m.ready_marker_paths("~/x/ready")[0] == Path(os.path.expanduser("~/x/ready"))
    monkeypatch.chdir(tmp_path)
    assert m.ready_marker_paths("rel/ready")[0] == tmp_path / "rel" / "ready"     # absolute, from the cwd


def test_write_ready_marker_success_then_failure(tmp_path):
    p = tmp_path / "sub" / "ready"                     # the folder is created
    out = m.write_ready_marker(str(p), True, {"ready_after_s": "34.5", "world": "/a  b/w.sdf", "note": "two\nlines"})
    assert out == p and p.read_text().startswith("status=READY\n")
    assert _kv(p) == {"status": "READY", "ready_after_s": "34.5", "world": "/a  b/w.sdf", "note": "two lines"}
    assert [x.name for x in p.parent.iterdir()] == ["ready"]          # no temporary file is left
    out = m.write_ready_marker(str(p), False, {"reason": "x"})       # NOT READY: the other file, READY goes
    assert out == p.with_name("ready.not_ready") and not p.exists()
    assert _kv(out) == {"status": "NOT_READY", "reason": "x"}
    m.write_ready_marker(str(p), True)
    assert p.exists() and not out.exists()
    assert m.write_ready_marker("", True) is None


def test_clear_ready_markers(tmp_path):
    p = tmp_path / "ready"
    p.write_text("status=READY\n")
    p.with_name("ready.not_ready").write_text("status=NOT_READY\n")
    assert sorted(x.name for x in m.clear_ready_markers(str(p))) == ["ready", "ready.not_ready"]
    assert not list(tmp_path.iterdir())
    assert m.clear_ready_markers(str(p)) == [] and m.clear_ready_markers("") == []


def test_on_wait_ready_exit(tmp_path):
    p = tmp_path / "ready"
    assert "ready marker written" in m.on_wait_ready_exit(0, str(p), {"ready_after_s": "36.0"})
    assert _kv(p) == {"status": "READY", "ready_after_s": "36.0"}
    nr = p.with_name("ready.not_ready")
    assert "not-ready marker written" in m.on_wait_ready_exit(2, str(p), {"ready_after_s": "97.0"})
    kv = _kv(nr)
    assert not p.exists() and kv["status"] == "NOT_READY" and kv["wait_ready_exit_code"] == "2"
    assert "timed out" in kv["reason"] and kv["ready_after_s"] == "97.0"
    m.on_wait_ready_exit(3, str(p))
    assert "exit code 3" in _kv(nr)["reason"]
    assert m.on_wait_ready_exit(0, "") == "" and m.on_wait_ready_exit(0, None) == ""
    blocker = tmp_path / "a_file"
    blocker.write_text("")
    msg = m.on_wait_ready_exit(0, str(blocker / "ready"))               # never raises into the launch
    assert msg.startswith("WARNING: cannot write the ready marker")


def test_ready_marker_is_written_atomically(tmp_path):
    """A reader polling the marker (the wrapper does, every 0.5 s) sees no file or a whole one, never a part."""
    import threading

    p = tmp_path / "ready"
    written, seen, stop = set(), [], threading.Event()

    def reader():
        while not stop.is_set():
            try:
                seen.append(p.read_text())
            except FileNotFoundError:
                pass

    t = threading.Thread(target=reader)
    t.start()
    try:
        for i in range(40):
            m.write_ready_marker(str(p), True, {"i": i, "blob": "x" * 300_000})
            written.add(p.read_text())
    finally:
        stop.set()
        t.join()
    assert seen and all(s in written for s in seen)
    assert [x.name for x in tmp_path.iterdir()] == ["ready"]


def test_launch_wires_the_ready_marker():
    """launch_setup clears both markers at start (launch argument, else $APOLLO16_READY_FILE) and writes the
    marker from the wait_ready exit handler, after the READY banner and before RViz starts."""
    src = LAUNCH.read_text()
    setup = src[src.index("def launch_setup"):src.index("def generate_launch_description")]
    assert 'arg("ready_file")' in setup and "READY_FILE_ENV" in setup and "clear_ready_markers(ready_file)" in setup
    after = setup[setup.index("def after_ready"):]
    assert after.index("READY at") < after.index("OpaqueFunction(function=mark_ready") < after.index("in_ns(rviz")
    mark = setup[setup.index("def mark_ready"):setup.index("def after_ready")]
    assert "on_wait_ready_exit(returncode, ready_file" in mark
    assert m.READY_FILE_ENV == "APOLLO16_READY_FILE" and m.NOT_READY_SUFFIX == ".not_ready"
