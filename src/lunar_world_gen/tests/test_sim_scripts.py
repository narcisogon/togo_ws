"""scripts/run_apollo16_sim.sh argument handling and --dry-run (U4/CR-10, U6, U7, U10, U11),
scripts/env.sh (U2), and the headless-first start-up (docs/impl/leak_script.md): READY marker, GUI after
READY, scripts/open_gui.sh, Ctrl+C with SIGKILL for gz, WSL crash-dump handling.

No simulation is started by these tests. The start-up tests run the real scripts in a sandbox copy of the
package with test doubles: a fake env.sh (no ROS), a fake ``ros2`` (``ros2 launch`` that writes the ready
marker) and a fake ``gz`` (server and GUI that record every SIGTERM/SIGINT/SIGHUP they get)."""
from __future__ import annotations

import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
WRAPPER = PKG / "scripts" / "run_apollo16_sim.sh"
OPEN_GUI = PKG / "scripts" / "open_gui.sh"
LIB = PKG / "scripts" / "apollo16_lib.sh"
ENV_SH = PKG / "scripts" / "env.sh"
LAUNCH = PKG / "launch" / "apollo16_sim.launch.py"
PACKAGED_GUI = PKG / "config" / "gui" / "apollo16.gui.config"
ROS_OK = Path("/opt/ros/jazzy/setup.bash").is_file() and \
    Path(os.path.expanduser("~/togo_ws_native_clean/install/setup.bash")).is_file()
needs_bash = pytest.mark.skipif(shutil.which("bash") is None or os.name == "nt", reason="needs bash (Linux)")
needs_ros = pytest.mark.skipif(not ROS_OK, reason="needs ROS 2 Jazzy and the native TOGO install")


def run(args: list[str], env_extra: dict | None = None, timeout: float = 60) -> subprocess.CompletedProcess:
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("ROS_", "RMW_", "CYCLONEDDS", "GZ_", "AMENT", "COLCON", "APOLLO16"))}
    env.update(env_extra or {})
    return subprocess.run(["bash", str(WRAPPER), *args], capture_output=True, text=True, timeout=timeout, env=env)


@needs_bash
@pytest.mark.parametrize("args, msg", [
    (["--site"], "--site needs 1 value"),
    (["--world"], "--world needs 1 value"),
    (["--variant"], "--variant needs 1 value"),
    (["--pose", "1", "2"], "--pose needs 3 value"),
    (["--world-info"], "--world-info needs 1 value"),
    (["--robot-collision"], "--robot-collision needs 1 value"),
    (["--gui-config"], "--gui-config needs 1 value"),
    (["--site", "--headless"], "--site is missing a value"),
    (["--pose", "1", "--headless", "2"], "--pose is missing a value"),
    (["--variant", "bogus"], "--variant must be"),
    (["--robot-collision", "fast"], "--robot-collision must be"),
    (["--pose", "1", "2", "abc"], "--pose values must be numbers"),
    (["--gui-config", "/nonexistent/x.config"], "file not found"),
    (["--gui-config", str(WRAPPER), "--default-gui"], "either --gui-config or --default-gui"),
    (["--bogus"], "unknown option '--bogus'"),
    (["flat"], "unknown option 'flat'"),
    (["--", "notakeyvalue"], "name:=value"),
])
def test_bad_arguments_fail_fast(args, msg):
    """U4 / CR-10: a missing value used to make the option loop spin forever (timeout rc 124)."""
    res = run(args, timeout=10)
    assert res.returncode == 2, (res.stdout, res.stderr)
    assert msg in res.stderr
    assert "--help" in res.stderr


@needs_bash
def test_help_lists_options():
    res = run(["--help"])
    assert res.returncode == 0
    for opt in ("--site", "--headless", "--pose", "--variant", "--world-info", "--robot-collision", "--gui-config",
                "--default-gui", "--dry-run", "inspect_stock", "env.sh", "READY", "--gui-early", "--ready-timeout",
                "--keep-crash-dumps", "open_gui.sh", "wsl-crashes"):
        assert opt in res.stdout, opt
    assert "ROS_DISTRO=" not in res.stdout          # the old usage() printed a code line
    res = subprocess.run(["bash", str(OPEN_GUI), "--help"], capture_output=True, text=True, timeout=10)
    assert res.returncode == 0
    for opt in ("--default-gui", "--gui-config", "--site", "--pose", "--no-wait", "--ready-timeout", "--close",
                "--keep-crash-dumps", "--dry-run", "Ctrl+C", "X", "READY"):
        assert opt in res.stdout, opt


def _fake_world(tmp: Path, plugin: Path) -> Path:
    for name, engine in (("w.sdf", str(plugin)), ("w_inspect.sdf", str(plugin)),
                         ("w_stock.sdf", "gz-physics-bullet-plugin"),
                         ("w_inspect_stock.sdf", "gz-physics-bullet-plugin")):
        (tmp / name).write_text(f'<?xml version="1.0"?><sdf version="1.11"><world name="w">'
                                f'<plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">'
                                f'<engine><filename>{engine}</filename></engine></plugin></world></sdf>')
    (tmp / "world_info.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "gravity_m_s2": 1.62,
        "sites": {"flat": {"x": -4.5, "y": -0.5, "z": -0.434, "yaw": 2.356194},
                  "slope": {"x": 11.0, "y": -4.0, "z": -2.5397, "yaw": -2.748894}}}))
    return tmp / "w.sdf"


def _launch_line(out: str) -> str:
    return next(ln for ln in out.splitlines() if "ros2 launch" in ln)


@needs_bash
@needs_ros
def test_dry_run_resolves_and_prints_commands(tmp_path):
    good = shutil.which("gcc") is not None
    plugin = tmp_path / "libplugin.so"
    if good:
        src = tmp_path / "p.c"
        src.write_text("int f(void){return 1;}\n")
        subprocess.run(["gcc", "-shared", "-fPIC", "-o", str(plugin), str(src)], check=True)
    else:
        pytest.skip("needs gcc for a loadable plugin")
    world = _fake_world(tmp_path, plugin)
    res = run(["--dry-run", "--headless", "--world", str(world), "--site", "slope", "--robot-collision", "original",
               "--", "gz_verbosity:=3"])
    assert res.returncode == 0, res.stderr
    line = _launch_line(res.stdout)
    for part in (f"world:={world}", "site:=slope", "robot_collision:=original", "gz_verbosity:=3",
                 "headless_rendering:=false", "rviz:=false"):
        assert part in line, part
    assert "x=11.000 y=-4.000" in res.stdout and "Gravity:      1.62" in res.stdout
    assert "ROS domain:   29" in res.stdout and "Dry run" in res.stdout
    assert "gz sim -g" not in res.stdout                # --headless
    # explicit pose with yaw
    res = run(["--dry-run", "--headless", "--world", str(world), "--pose", "1", "-2.5", "0.3", "-1.2"])
    line = _launch_line(res.stdout)
    assert "robot_x:=1" in line and "robot_y:=-2.5" in line and "robot_yaw:=-1.2" in line
    # ROS_DOMAIN_ID from the caller wins, as in togo_sim_wsl.sh
    res = run(["--dry-run", "--headless", "--world", str(world)], {"ROS_DOMAIN_ID": "77"})
    assert "ROS domain:   77" in res.stdout


@needs_bash
@needs_ros
def test_dry_run_broken_plugin_falls_back_with_warning(tmp_path):
    """U7 + U11: an unloadable plugin is detected before launching; inspect keeps its lighting."""
    plugin = tmp_path / "libbroken.so"
    plugin.write_bytes(b"garbage, not ELF" * 64)
    world = _fake_world(tmp_path, plugin)
    res = run(["--dry-run", "--headless", "--world", str(world), "--variant", "inspect"])
    assert res.returncode == 0, res.stderr
    assert f"world:={tmp_path / 'w_inspect_stock.sdf'}" in _launch_line(res.stdout)
    assert "WARNING: physics plugin" in res.stderr and "cannot be loaded" in res.stderr
    assert "build_lunar_bullet_plugin.sh" in res.stderr
    (tmp_path / "w_stock.sdf").unlink()
    (tmp_path / "w_inspect_stock.sdf").unlink()
    res = run(["--dry-run", "--headless", "--world", str(world)])
    assert res.returncode == 1 and "Build the plugin" in res.stderr


@needs_bash
@needs_ros
def test_dry_run_errors_name_the_next_step(tmp_path):
    """U10: errors go to stderr with what to do next."""
    res = run(["--dry-run", "--headless", "--world", str(tmp_path / "missing.sdf")])
    assert res.returncode == 1
    assert "world file not found" in res.stderr and "lunar_world_gen.build" in res.stderr
    plugin = tmp_path / "x.so"
    world = _fake_world(tmp_path, plugin)
    res = run(["--dry-run", "--headless", "--world", str(world), "--site", "summit"])
    assert res.returncode == 1 and "available: ['flat', 'slope']" in res.stderr
    (tmp_path / "world_info.yaml").unlink()
    res = run(["--dry-run", "--headless", "--world", str(world)])
    assert res.returncode == 1 and "--pose X Y Z" in res.stderr and "--world-info" in res.stderr


@needs_bash
@needs_ros
def test_dry_run_gui_config_choice(tmp_path):
    """U6: the GUI gets the packaged config (camera moved to the spawn), a given file, or the default."""
    plugin = tmp_path / "x.so"
    world = _fake_world(tmp_path, plugin)
    cfg = tmp_path / "my.gui.config"
    cfg.write_text("<camera_pose>0 0 1 0 0 0</camera_pose>")
    res = run(["--dry-run", "--world", str(world), "--gui-config", str(cfg)])
    assert res.returncode == 0, res.stderr
    assert f"gz sim -g --gui-config {cfg}" in res.stdout
    res = run(["--dry-run", "--world", str(world), "--default-gui"])
    gui = next(ln for ln in res.stdout.splitlines() if "gz sim -g" in ln)
    assert "--gui-config" not in gui and "default GUI config" in res.stdout
    res = run(["--dry-run", "--world", str(world), "--site", "slope"])
    packaged = PKG / "config" / "gui" / "apollo16.gui.config"
    if packaged.is_file():
        assert "--gui-config" in next(ln for ln in res.stdout.splitlines() if "gz sim -g" in ln)
        assert "camera: the 'flat' view moved to this spawn" in res.stdout
    else:
        assert "default GUI config" in res.stdout


# ------------------------------------------------------------------------------ env.sh
def _bash(script: str, env_extra: dict | None = None) -> subprocess.CompletedProcess:
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("ROS_", "RMW_", "CYCLONEDDS", "GZ_", "AMENT", "COLCON", "GALLIUM", "APOLLO16",
                                "MESA_EXTENSION"))}
    env.update(env_extra or {})
    return subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=60, env=env)


@needs_bash
@needs_ros
def test_env_sh_sets_the_sim_environment():
    repo = PKG.parents[1]
    res = _bash(f"set -u; source {ENV_SH} --quiet; echo rc=$?; "
                "echo D=$ROS_DOMAIN_ID; echo R=$RMW_IMPLEMENTATION; echo C=$CYCLONEDDS_URI; echo G=$GALLIUM_DRIVER; "
                "echo A=$ROS_AUTOMATIC_DISCOVERY_RANGE; echo P=$GZ_SIM_RESOURCE_PATH; echo L=$LD_LIBRARY_PATH; "
                "echo U=$-; command -v ros2; echo M=$MESA_EXTENSION_OVERRIDE; "
                f"source {ENV_SH} --quiet; echo P2=$GZ_SIM_RESOURCE_PATH; echo L2=$LD_LIBRARY_PATH; "
                "echo M2=$MESA_EXTENSION_OVERRIDE")
    assert res.returncode == 0, res.stderr
    out = dict(ln.split("=", 1) for ln in res.stdout.splitlines() if "=" in ln and not ln.startswith("/"))
    assert out["rc"] == "0" and out["D"] == "29" and out["R"] == "rmw_cyclonedds_cpp"
    assert out["C"] == f"file://{repo}/config/cyclonedds.xml" and out["G"] == "d3d12"
    # as togo_sim_wsl.sh: ROS's ros_environment hook sets SUBNET when unset, before the LOCALHOST default
    # could apply (cyclonedds.xml pins DDS to the loopback interface either way)
    ros_default = subprocess.run(["bash", "-c", "unset ROS_AUTOMATIC_DISCOVERY_RANGE; source /opt/ros/jazzy/setup.bash;"
                                  " echo ${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"],
                                 capture_output=True, text=True).stdout.strip()
    assert out["A"] == ros_default
    assert "u" in out["U"]                                   # the caller's 'set -u' is restored
    assert "/ros2" in res.stdout
    assert out["P"] == out["P2"] and out["L"] == out["L2"]   # sourcing twice adds nothing
    assert out["P"].count("togo_gz/share/togo_gz") == 1 and out["L"].split(":").count("/usr/lib/wsl/lib") == 1
    res = _bash(f"source {ENV_SH} --quiet; echo D=$ROS_DOMAIN_ID", {"ROS_DOMAIN_ID": "42"})
    assert "D=42" in res.stdout
    # Mesa d3d12 memory-leak workaround (docs/impl/leak_diagnosis.md): set once, appended to the user's own
    # overrides, only for the d3d12 driver
    assert out["M"] == "-GL_ARB_multi_draw_indirect" and out["M2"] == out["M"]
    res = _bash(f"source {ENV_SH} --quiet; echo M=$MESA_EXTENSION_OVERRIDE",
                {"MESA_EXTENSION_OVERRIDE": "-GL_EXT_foo"})
    assert "M=-GL_EXT_foo -GL_ARB_multi_draw_indirect" in res.stdout
    res = _bash(f"source {ENV_SH} --quiet; echo M=${{MESA_EXTENSION_OVERRIDE:-none}}",
                {"GALLIUM_DRIVER": "llvmpipe"})
    assert "M=none" in res.stdout


@needs_bash
def test_env_sh_errors_do_not_exit_the_shell(tmp_path):
    res = _bash(f"source {ENV_SH} --quiet; echo rc=$?; echo still-here",
                {"TOGO_WSL_INSTALL": str(tmp_path / "none"), "ROS_DISTRO": "jazzy"})
    assert "still-here" in res.stdout
    assert ("rc=1" in res.stdout and "not found" in res.stderr) or not ROS_OK
    res = subprocess.run(["bash", str(ENV_SH)], capture_output=True, text=True, timeout=10)
    assert res.returncode == 2 and "only works when sourced" in res.stderr


@needs_bash
def test_scripts_parse():
    for p in (WRAPPER, ENV_SH, OPEN_GUI, LIB):
        res = subprocess.run(["bash", "-n", str(p)], capture_output=True, text=True)
        assert res.returncode == 0, res.stderr
    res = subprocess.run(["bash", str(LIB)], capture_output=True, text=True, timeout=10)
    assert res.returncode == 2 and "sourced" in res.stderr


# ------------------------------------------------------------------------------ sandbox with test doubles
FAKE_ENV_SH = """# shellcheck shell=bash
# test double for scripts/env.sh: no ROS; the fake ros2 and gz come first on PATH
export PATH="{bindir}:$PATH"
export ROS_DOMAIN_ID="${{ROS_DOMAIN_ID:-29}}" RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}}"
glxinfo() {{ :; }}
return 0
"""

# 'ros2 launch' runs fake_launch.py under another name, so that no 'pgrep -f "ros2 launch"' (of
# run_apollo16_sim.sh, or of any other tool on this machine) takes it for a real simulation
FAKE_ROS2 = """#!/bin/bash
exec -a fake-ros2 python3 "{bindir}/fake_launch.py" "$@"
"""

FAKE_LAUNCH = r'''
import os, signal, subprocess, sys, time
log = os.environ["FAKE_LOG"]
args = sys.argv[1:]                      # launch <file> name:=value ...
kv = dict(a.split(":=", 1) for a in args[2:] if ":=" in a)
with open(os.path.join(log, "launch_args"), "w") as f:
    f.write("\n".join(args) + "\n")
mode = os.environ.get("FAKE_LAUNCH_MODE", "ready")      # ready | not_ready | never | exit3
ready = kv.get("ready_file", "")
stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append("INT"))
signal.signal(signal.SIGTERM, lambda *_: stop.append("TERM"))
server = subprocess.Popen(["gz", "fake-server"])          # process name gz, in the launch's process group
if mode == "exit3":
    time.sleep(0.5)
    server.kill()
    server.wait()
    sys.exit(3)
t0 = time.monotonic()
written = False
while not stop:
    if (ready and not written and mode in ("ready", "not_ready")
            and time.monotonic() - t0 >= float(os.environ.get("FAKE_READY_DELAY", "1"))):
        target = ready if mode == "ready" else ready + ".not_ready"
        with open(target + ".tmp", "w") as f:
            f.write("status=READY\n" if mode == "ready" else "status=NOT_READY\nreason=fake controllers inactive\n")
        os.replace(target + ".tmp", target)
        with open(os.path.join(log, "marker_written"), "w") as f:
            f.write(f"{time.time():.3f}\n")
        written = True
    if server.poll() is not None:
        break
    time.sleep(0.05)
with open(os.path.join(log, "launch_stopped"), "w") as f:
    f.write(f"{','.join(stop)} server_alive={server.poll() is None}\n")
if server.poll() is None:      # ros2 launch would now stop gz with a signal: the wrapper must have SIGKILLed it
    server.terminate()
    server.wait(timeout=5)
sys.exit(0)
'''

# 'gz sim -g ...' (the GUI) re-executes itself under another name, so that no 'pgrep -f "gz sim"' of
# another tool counts it; 'gz fake-server' is the server (process name gz, as the wrapper looks for)
FAKE_GZ = r'''#!/bin/bash
if [[ "${1:-}" == sim ]]; then exec -a fake-gz-gui /bin/bash "$0" __gui__ "${@:2}"; fi
role="${1:-}"
shift
echo "$$ ${role} $(date +%s.%N) $*" >> "${FAKE_LOG}/gz_started"
for s in TERM INT HUP; do trap "echo '$$ ${role} ${s}' >> '${FAKE_LOG}/gz_got_signal'; exit 0" "${s}"; done
if [[ "${role}" == __gui__ ]]; then
  prev=""
  for a in "$@"; do [[ "${prev}" == --gui-config ]] && cp -- "${a}" "${FAKE_LOG}/gui_config.$$"; prev="${a}"; done
  if [[ "${FAKE_GUI:-}" == crash ]]; then
    sleep 0.5
    dump="${APOLLO16_WSL_CRASH_DIR}/wsl-crash-$(date +%s)-$$-_usr_bin_ruby3.2-11.dmp"
    # WSL streams a dump for a while after the crash: a writer outside the GUI's process group
    setsid bash -c 'for i in 1 2 3 4 5 6; do head -c 100000 /dev/zero >> "$1"; sleep 0.5; done; date +%s.%N > "$2"' \
      _ "${dump}" "${FAKE_LOG}/dump_done" &
    exit 139            # what a segfault looks like to the wrapper (no real core dump is made)
  fi
fi
while :; do sleep 0.05; done
'''

needs_linux = pytest.mark.skipif(not sys.platform.startswith("linux") or shutil.which("setsid") is None
                                 or shutil.which("pgrep") is None or shutil.which("bash") is None,
                                 reason="needs Linux with bash, setsid and pgrep")


@dataclass
class Sandbox:
    root: Path
    pkg: Path
    bindir: Path
    log: Path
    state: Path
    crash: Path
    world: Path
    env: dict

    @property
    def wrapper(self) -> Path:
        return self.pkg / "scripts" / "run_apollo16_sim.sh"

    @property
    def open_gui(self) -> Path:
        return self.pkg / "scripts" / "open_gui.sh"

    def run(self, script: Path, args: list[str], env: dict | None = None, timeout: float = 30
            ) -> subprocess.CompletedProcess:
        return subprocess.run(["bash", str(script), *args], capture_output=True, text=True, timeout=timeout,
                              env={**self.env, **(env or {})})

    def start(self, script: Path, args: list[str], name: str, env: dict | None = None) -> "Proc":
        out = self.log / f"{name}.out"
        p = subprocess.Popen(["bash", str(script), *args], stdout=open(out, "w"), stderr=subprocess.STDOUT,
                             env={**self.env, **(env or {})}, start_new_session=True)
        return Proc(p, out)

    def gz(self, role: str) -> list[tuple[int, float, str]]:
        f = self.log / "gz_started"
        rows = [ln.split(" ", 3) for ln in f.read_text().splitlines()] if f.is_file() else []
        return [(int(r[0]), float(r[2]), r[3] if len(r) > 3 else "") for r in rows if r[1] == role]

    def gz_signals(self) -> str:
        f = self.log / "gz_got_signal"
        return f.read_text() if f.is_file() else ""


@dataclass
class Proc:
    p: subprocess.Popen
    out_file: Path

    def out(self) -> str:
        return self.out_file.read_text(errors="replace")

    def ctrl_c(self) -> None:           # what the terminal does: SIGINT to the foreground process group
        os.killpg(self.p.pid, signal.SIGINT)

    def wait(self, timeout: float = 60) -> int:
        return self.p.wait(timeout=timeout)


def _until(cond, timeout: float = 20.0, what: str = "") -> None:
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        if cond():
            return
        time.sleep(0.05)
    raise AssertionError(f"timed out after {timeout} s waiting for {what or cond}")


def _sandbox_pids(root: Path) -> list[int]:
    """Processes whose command line contains the sandbox path (only ever this test's processes)."""
    pids, key = [], str(root).encode()
    for d in Path("/proc").iterdir():
        if not d.name.isdigit() or int(d.name) == os.getpid():
            continue
        try:
            if key in (d / "cmdline").read_bytes():
                pids.append(int(d.name))
        except OSError:
            pass
    return pids


def _session(sb: Sandbox, wrapper_pid: int) -> dict:
    f = sb.state / f"run.{wrapper_pid}" / "session"
    return dict(ln.split("=", 1) for ln in f.read_text().splitlines()) if f.is_file() else {}


@pytest.fixture
def sandbox(tmp_path):
    pkg = tmp_path / "pkg"
    for sub in ("scripts", "launch", "config/gui"):
        (pkg / sub).mkdir(parents=True)
    for p in (WRAPPER, OPEN_GUI, LIB):
        shutil.copy2(p, pkg / "scripts" / p.name)
    shutil.copy2(LAUNCH, pkg / "launch" / LAUNCH.name)
    if PACKAGED_GUI.is_file():
        shutil.copy2(PACKAGED_GUI, pkg / "config" / "gui" / PACKAGED_GUI.name)
    bindir, log, state, crash, wdir = (tmp_path / n for n in ("bin", "log", "state", "wsl-crashes", "world"))
    for d in (bindir, log, crash, wdir):
        d.mkdir()
    (pkg / "scripts" / "env.sh").write_text(FAKE_ENV_SH.format(bindir=bindir))
    for name, text in (("ros2", FAKE_ROS2.format(bindir=bindir)), ("fake_launch.py", FAKE_LAUNCH), ("gz", FAKE_GZ)):
        (bindir / name).write_text(text)
        (bindir / name).chmod(0o755)
    (wdir / "w.sdf").write_text('<?xml version="1.0"?><sdf version="1.11"><world name="w">'
                                '<plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">'
                                '<engine><filename>gz-physics-bullet-plugin</filename></engine></plugin></world></sdf>')
    (wdir / "world_info.yaml").write_text(yaml.safe_dump({
        "schema_version": 1, "gravity_m_s2": 1.62,
        "sites": {"flat": {"x": -4.5, "y": -0.5, "z": -0.434, "yaw": 2.356194},
                  "slope": {"x": 11.0, "y": -4.0, "z": -2.5397, "yaw": -2.748894}}}))
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("ROS_", "RMW_", "CYCLONEDDS", "GZ_", "AMENT", "COLCON", "APOLLO16", "FAKE_"))}
    env.update(APOLLO16_ALLOW_CONCURRENT="1", APOLLO16_STATE_DIR=str(state), APOLLO16_WSL_CRASH_DIR=str(crash),
               APOLLO16_CRASH_WAIT_S="20", FAKE_LOG=str(log), GZ_PARTITION="pytest_apollo16_sandbox",
               ROS_DOMAIN_ID="123")
    sb = Sandbox(tmp_path, pkg, bindir, log, state, crash, wdir / "w.sdf", env)
    yield sb
    for pid in _sandbox_pids(tmp_path):          # leftovers of a failed test: only this sandbox's processes
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


def _stop(proc: Proc, sb: Sandbox, timeout: float = 40) -> int:
    proc.ctrl_c()
    rc = proc.wait(timeout)
    return rc


def _no_leftovers(sb: Sandbox) -> None:
    _until(lambda: not _sandbox_pids(sb.root), 5, "all sandbox processes to end")


# ------------------------------------------------------------------------------ dry runs in the sandbox
@needs_linux
def test_sandbox_dry_run_new_flow(sandbox):
    sb = sandbox
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--site", "slope"])
    assert res.returncode == 0, res.stderr
    out = res.stdout
    launch = _launch_line(out)
    assert f"ready_file:={sb.state}/run." in launch and launch.count("ready_file:=") == 1
    assert "Gazebo GUI:   opens after READY" in out
    assert "then wait until the launch writes" in out and "at most 600 s" in out
    gui = next(ln for ln in out.splitlines() if "gz sim -g" in ln)
    assert gui.strip().startswith("setsid gz sim -g --gui-config")
    assert out.index("ros2 launch") < out.index("then wait until") < out.index("gz sim -g")
    assert not sb.state.exists()                       # a dry run creates nothing
    # the user's own marker path is used, not a second one
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--ready-timeout", "90", "--",
                              f"ready_file:={sb.root}/mine"])
    launch = _launch_line(res.stdout)
    assert launch.count("ready_file:=") == 1 and f"ready_file:={sb.root}/mine" in launch
    assert "at most 90 s" in res.stdout
    # --gui-early: the old order; --headless: no GUI, the command to open it later
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--gui-early", "--default-gui"])
    assert "8 s later (--gui-early)" in res.stdout
    gui = next(ln for ln in res.stdout.splitlines() if "gz sim -g" in ln)
    assert gui.strip() == "setsid gz sim -g"           # no empty '' argument
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--headless"])
    assert "gz sim -g" not in res.stdout and "open_gui.sh" in res.stdout and "--headless" in res.stdout
    # the printed command to open the GUI later carries the GUI choice
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--headless", "--default-gui"])
    assert "open_gui.sh --default-gui" in res.stdout
    cfg = sb.root / "my gui.config"
    cfg.write_text("<camera_pose>0 0 1 0 0 0</camera_pose>")
    res = sb.run(sb.wrapper, ["--dry-run", "--world", str(sb.world), "--headless", "--gui-config", str(cfg)])
    assert f"open_gui.sh --gui-config {str(cfg).replace(' ', chr(92) + ' ')}" in res.stdout


@needs_linux
@pytest.mark.parametrize("args, msg", [
    (["--headless", "--gui-early"], "either --headless or --gui-early"),
    (["--ready-timeout", "0"], "--ready-timeout must be"),
    (["--ready-timeout", "abc"], "--ready-timeout must be"),
    (["--ready-timeout"], "--ready-timeout needs 1 value"),
])
def test_sandbox_new_options_fail_fast(sandbox, args, msg):
    res = sandbox.run(sandbox.wrapper, args, timeout=10)
    assert res.returncode == 2 and msg in res.stderr


@needs_linux
@pytest.mark.parametrize("args, msg", [
    (["--site", "slope", "--pose", "1", "2", "3"], "either --site or --pose"),
    (["--gui-config", "/nonexistent.config"], "file not found"),
    (["--default-gui", "--gui-config", str(WRAPPER)], "either --gui-config or --default-gui"),
    (["--ready-timeout", "-3"], "--ready-timeout must be"),
    (["--pose", "1", "x", "3"], "--pose values must be numbers"),
    (["--bogus"], "unknown option '--bogus'"),
    (["--site"], "--site needs 1 value"),
])
def test_open_gui_bad_arguments(sandbox, args, msg):
    res = sandbox.run(sandbox.open_gui, args, timeout=10)
    assert res.returncode == 2 and msg in res.stderr and "--help" in res.stderr


@needs_linux
def test_open_gui_dry_run_without_simulation(sandbox):
    sb = sandbox
    res = sb.run(sb.open_gui, ["--dry-run", "--world", str(sb.world), "--site", "slope"])
    assert res.returncode == 0, res.stderr
    assert "No running run_apollo16_sim.sh found" in res.stdout
    assert "camera at site slope" in res.stdout and "setsid gz sim -g --gui-config" in res.stdout
    res = sb.run(sb.open_gui, ["--dry-run", "--default-gui"])
    assert res.returncode == 0 and res.stdout.rstrip().endswith("setsid gz sim -g")
    res = sb.run(sb.open_gui, ["--close"])
    assert res.returncode == 0 and "No Gazebo GUI" in res.stdout


# ------------------------------------------------------------------------------ the start-up flow (test doubles)
@needs_linux
def test_default_flow_gui_after_ready_and_sigkill_on_ctrl_c(sandbox):
    sb = sandbox
    sim = sb.start(sb.wrapper, ["--world", str(sb.world), "--site", "slope"], "sim",
                   {"FAKE_READY_DELAY": "1.5", "APOLLO16_LOW_MEM_MB": "100000000"})
    _until(lambda: sb.gz("__gui__"), 30, "the GUI to start")
    (gui_pid, gui_t, gui_args), = sb.gz("__gui__")
    marker_t = float((sb.log / "marker_written").read_text())
    assert gui_t >= marker_t, "the GUI started before the launch wrote the ready marker"
    _until(lambda: "Close only the GUI" in sim.out(), 5, "the GUI notice")
    out = sim.out()
    for text in ("READY after", "stays blank", "30-60 s", '"[WARN:COPY MODE]"', "wsl --shutdown", "Ctrl+C",
                 "X button", "open_gui.sh --close"):
        assert text in out, text
    assert out.index("READY after") > out.index("[1/2] Starting the Gazebo server")
    assert gui_args.startswith("-g --gui-config ")
    # the per-run GUI config has the camera at the spawn (site slope), as the launch module computes it
    import importlib.util
    spec = importlib.util.spec_from_file_location("a16_launch_test", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    expected, _ = mod.gui_config_for_pose(PACKAGED_GUI.read_text(), sb.world, "",
                                          (11.0, -4.0, -2.5397, -2.748894))
    assert (sb.log / f"gui_config.{gui_pid}").read_text() == expected
    s = _session(sb, sim.p.pid)
    assert s["gui_state"] == "open" and s["gui_pgid"] == str(gui_pid) and s["site"] == "slope"
    assert s["gz_partition"] == "pytest_apollo16_sandbox" and s["ready_file"].startswith(str(sb.state))
    assert (sb.state / f"gui.{gui_pid}").is_file()
    _until(lambda: "running out of memory" in sim.out(), 10, "the low-memory warning")
    rc = _stop(sim, sb)
    out = sim.out()
    assert rc in (0, 130), out
    assert sb.gz_signals() == "", "a gz process got SIGTERM/SIGINT/SIGHUP instead of SIGKILL"
    assert (sb.log / "launch_stopped").read_text().strip() == "INT server_alive=False"
    assert "Apollo 16 simulation stopped" in out and out.count("running out of memory") == 1
    assert not (sb.state / f"run.{sim.p.pid}").exists() and not (sb.state / f"gui.{gui_pid}").exists()
    _no_leftovers(sb)


@needs_linux
@pytest.mark.parametrize("mode, extra, expect", [
    ("not_ready", [], "reported NOT READY"),
    ("never", ["--ready-timeout", "2"], "No READY within 2 s"),
])
def test_no_gui_without_ready(sandbox, mode, extra, expect):
    sb = sandbox
    sim = sb.start(sb.wrapper, ["--world", str(sb.world), *extra], "sim", {"FAKE_LAUNCH_MODE": mode})
    _until(lambda: expect in sim.out(), 20, expect)
    time.sleep(1.0)
    out = sim.out()
    assert not sb.gz("__gui__") and "open_gui.sh" in out and "keeps running" in out
    if mode == "not_ready":
        assert "fake controllers inactive" in out
    assert _session(sb, sim.p.pid)["gui_state"] == "skipped"
    assert sim.p.poll() is None                          # the simulation keeps running
    assert _stop(sim, sb) in (0, 130)
    assert sb.gz_signals() == ""
    _no_leftovers(sb)


@needs_linux
def test_launch_exit_before_ready(sandbox):
    sb = sandbox
    sim = sb.start(sb.wrapper, ["--world", str(sb.world)], "sim", {"FAKE_LAUNCH_MODE": "exit3"})
    assert sim.wait(30) == 3
    out = sim.out()
    assert "exited with code 3 before READY" in out and not sb.gz("__gui__")
    assert "Apollo 16 simulation stopped" in out
    _no_leftovers(sb)


@needs_linux
def test_headless_then_open_gui(sandbox):
    sb = sandbox
    sim = sb.start(sb.wrapper, ["--world", str(sb.world), "--site", "slope", "--headless"], "sim",
                   {"FAKE_READY_DELAY": "2"})
    _until(lambda: (sb.state / f"run.{sim.p.pid}" / "session").is_file(), 20, "the session file")
    gui1 = sb.start(sb.open_gui, [], "gui1")             # before READY: it waits
    _until(lambda: "Waiting until the simulation is READY" in gui1.out(), 20, "open_gui.sh to wait")
    _until(lambda: sb.gz("__gui__"), 20, "the GUI")
    (pid1, t1, args1), = sb.gz("__gui__")
    assert t1 >= float((sb.log / "marker_written").read_text())
    out = gui1.out()
    assert "site slope" in out and "GZ_PARTITION pytest_apollo16_sandbox" in out and "Ctrl+C" in out
    assert "camera: the 'flat' view moved to this spawn" in out
    assert (sb.log / f"gui_config.{pid1}").is_file()
    # a second GUI is refused while one is open
    res = sb.run(sb.open_gui, [], timeout=20)
    assert res.returncode == 1 and "already open" in res.stderr
    # Ctrl+C in the open_gui.sh terminal: only the GUI ends, with SIGKILL
    assert _stop(gui1, sb) == 0
    assert "Gazebo GUI closed" in gui1.out() and sb.gz_signals() == ""
    _until(lambda: not (sb.state / f"gui.{pid1}").exists(), 5, "the registration to go")
    assert sim.p.poll() is None and not list(sb.state.glob("open_gui.*"))
    # reopen (already READY: at once); stopping the simulation closes it
    gui2 = sb.start(sb.open_gui, [], "gui2")
    _until(lambda: len(sb.gz("__gui__")) == 2, 20, "the second GUI")
    assert "Waiting until" not in gui2.out()
    assert _stop(sim, sb) in (0, 130)
    assert gui2.wait(20) == 0 and "The simulation stopped: closing the Gazebo GUI" in gui2.out()
    assert sb.gz_signals() == ""
    _no_leftovers(sb)


@needs_linux
def test_close_gui_keeps_simulation(sandbox):
    sb = sandbox
    sim = sb.start(sb.wrapper, ["--world", str(sb.world)], "sim")
    _until(lambda: sb.gz("__gui__"), 30, "the GUI")
    (pid1, _, _), = sb.gz("__gui__")
    res = sb.run(sb.open_gui, [], timeout=20)            # the wrapper's GUI is open
    assert res.returncode == 1 and "--close" in res.stderr
    res = sb.run(sb.open_gui, ["--close"], timeout=20)
    assert res.returncode == 0 and "Closed the Gazebo GUI of run_apollo16_sim.sh" in res.stdout
    _until(lambda: "The Gazebo GUI was closed (SIGKILL" in sim.out(), 10, "the wrapper to notice")
    assert sim.p.poll() is None and _session(sb, sim.p.pid)["gui_state"] == "closed"
    gui = sb.start(sb.open_gui, ["--default-gui"], "gui")  # reopen from another terminal
    _until(lambda: len(sb.gz("__gui__")) == 2, 20, "the reopened GUI")
    assert sb.gz("__gui__")[1][2] == "-g"
    assert _stop(gui, sb) == 0
    assert _stop(sim, sb) in (0, 130)
    assert sb.gz_signals() == ""
    _no_leftovers(sb)


@needs_linux
@pytest.mark.parametrize("keep", [False, True])
def test_crash_dumps_of_this_run_only(sandbox, keep):
    sb = sandbox
    now = int(time.time())
    old = sb.crash / f"wsl-crash-{now - 5000}-4321-_usr_bin_ruby3.2-11.dmp"   # before this run
    old.write_bytes(b"x" * 1000)
    sim = sb.start(sb.wrapper, ["--world", str(sb.world), *(["--keep-crash-dumps"] if keep else [])], "sim",
                   {"FAKE_GUI": "crash"})
    _until(lambda: "the Gazebo GUI crashed (exit code 139, signal 11)" in sim.out(), 30, "the GUI crash")
    (gui_pid, _, _), = sb.gz("__gui__")
    t = int(time.time())
    foreign = sb.crash / f"wsl-crash-{t}-999999-_usr_bin_ruby3.2-11.dmp"      # not a process of this run
    other = sb.crash / f"wsl-crash-{t}-{sim.p.pid}-_usr_bin_sleep-11.dmp"   # neither ruby nor gz
    foreign.write_bytes(b"y")
    other.write_bytes(b"z")
    assert "Reopen the GUI" in sim.out() and sim.p.poll() is None
    rc = _stop(sim, sb)                                   # while WSL still "writes" the GUI's dump
    t_end = time.time()
    out = sim.out()
    assert rc in (0, 130), out
    ours = list(sb.crash.glob(f"wsl-crash-*-{gui_pid}-_usr_bin_ruby3.2-11.dmp"))
    assert (sb.log / "dump_done").is_file() and float((sb.log / "dump_done").read_text()) <= t_end
    assert "Waiting for WSL to finish writing a crash dump" in out
    if keep:
        assert len(ours) == 1 and ours[0].stat().st_size == 600000 and "Kept (--keep-crash-dumps)" in out
    else:
        assert not ours and "Deleted the WSL crash dump of this run" in out and f"pid {gui_pid}, signal 11" in out
    assert old.is_file() and foreign.is_file() and other.is_file()
    assert "999999" in out and "left alone" in out and "older WSL crash dump" in out
    _no_leftovers(sb)


@needs_linux
def test_crash_dump_name_parsing_and_folder():
    script = (f"source {LIB}; for n in wsl-crash-1792966979-224896-_usr_bin_ruby3.2-11.dmp "
              "wsl-crash-1-2-_opt_ros_jazzy_lib_ros_gz_bridge_parameter_bridge-6.dmp wsl-crash-x.dmp; do "
              "if a16_parse_dump $n; then echo \"$A16_DUMP_TS|$A16_DUMP_PID|$A16_DUMP_EXE|$A16_DUMP_SIG\"; "
              "else echo no; fi; done; APOLLO16_WSL_CRASH_DIR= a16_crash_dir || echo disabled")
    res = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=20)
    assert res.stdout.split() == ["1792966979|224896|_usr_bin_ruby3.2|11",
                                  "1|2|_opt_ros_jazzy_lib_ros_gz_bridge_parameter_bridge|6", "no", "disabled"]
    if shutil.which("cmd.exe") and Path("/mnt/c").is_dir():   # WSL: the real folder, through %TEMP%
        res = subprocess.run(["bash", "-c", f"source {LIB}; a16_crash_dir"], capture_output=True, text=True,
                             timeout=30)
        assert re.fullmatch(r"/mnt/[a-z]/.+/wsl-crashes", res.stdout.strip()), res.stdout
