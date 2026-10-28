"""The validation tools stop Gazebo with SIGKILL only (docs/impl/fix_build.md, task 6).

gz sim segfaults when it shuts down on SIGTERM/SIGINT, and WSL then writes a 1.5-3.6 GB crash
dump. These tests run fake ``gz`` processes (bash scripts named ``gz``, so their process name is
``gz``) that record any SIGTERM/SIGINT they receive:

* ``bench_world.GzServer.stop`` must SIGKILL the server's group directly;
* ``run_robot_tests.Launch.stop`` must SIGKILL the gz processes of a launch first and still give
  the launch's ROS nodes a graceful SIGINT afterwards.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux") or not Path("/proc").is_dir(),
                                reason="needs Linux /proc and process groups")

FAKE_GZ = """#!/bin/bash
trap 'echo "$$ $1" >> {marks}/gz_got_term_or_int; exit 0' TERM INT
echo started >> {marks}/gz_started
while true; do sleep 0.05; done
"""
FAKE_NODE = """#!/bin/bash
trap 'echo int >> {marks}/node_got_int; exit 0' INT
trap 'echo term >> {marks}/node_got_term; exit 0' TERM
while true; do sleep 0.05; done
"""
FAKE_LAUNCH = """#!/usr/bin/env python3
# stands in for 'ros2 launch' (a bash script cannot: its background jobs start with SIGINT ignored):
# a gz server and a ROS node as children in the same process group; when the server exits, the
# node is stopped gracefully with SIGINT (on_exit_shutdown)
import signal, subprocess
gz = subprocess.Popen(["gz", "sim", "-s", "-r", "world.sdf"])
node = subprocess.Popen(["{node}"])
signal.signal(signal.SIGINT, lambda *a: node.send_signal(signal.SIGINT))
gz.wait()
node.send_signal(signal.SIGINT)
node.wait(timeout=10)
"""


def _write(path: Path, text: str) -> Path:
    path.write_text(text)
    path.chmod(0o755)
    return path


def _wait_for(path: Path, timeout: float = 10.0) -> bool:
    t = time.monotonic()
    while time.monotonic() - t < timeout:
        if path.exists():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def fakes(tmp_path):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    marks = tmp_path / "marks"
    marks.mkdir()
    _write(bindir / "gz", FAKE_GZ.format(marks=marks))
    node = _write(bindir / "fake_ros_node", FAKE_NODE.format(marks=marks))
    launch = _write(bindir / "fake_launch", FAKE_LAUNCH.format(node=node))
    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env.get('PATH', '')}"
    return env, marks, launch


def test_gzserver_stop_uses_sigkill_only(fakes, tmp_path):
    import bench_world as bw

    env, marks, _ = fakes
    srv = bw.GzServer(tmp_path / "world.sdf", tmp_path / "srv" / "server.log", env, limit_s=60)
    assert srv.cmd[:3] == ["timeout", "-s", "KILL"] and "--foreground" not in srv.cmd
    srv.start()
    assert _wait_for(marks / "gz_started")
    pids = bw.gz_pids(srv.pgid)
    assert pids, "the fake server must be recognised as a gz process"
    info = srv.stop()
    assert info["signal"] == "SIGKILL" and info["remaining_in_group"] == 0
    time.sleep(0.3)
    assert not (marks / "gz_got_term_or_int").exists(), "gz received SIGTERM/SIGINT"
    assert all(not Path(f"/proc/{p}").exists() for p in pids)


def test_launch_stop_kills_gz_then_stops_ros_gracefully(fakes, tmp_path):
    import run_robot_tests as rrt

    env, marks, launch = fakes
    L = rrt.Launch([str(launch)], env, tmp_path / "run", limit_s=60)
    assert L.cmd[:3] == ["timeout", "-s", "KILL"] and "--foreground" not in L.cmd
    L.start()
    assert _wait_for(marks / "gz_started")
    t = time.monotonic()
    while not any(c == "fake_ros_node" for _, c in rrt.bw.group_pids(L.pgid)) and time.monotonic() - t < 5:
        time.sleep(0.05)
    info = L.stop(ros_grace_s=1.0, int_grace_s=5.0)
    assert info["gz_killed"], info
    assert info["remaining_in_group"] == 0
    time.sleep(0.3)
    assert not (marks / "gz_got_term_or_int").exists(), "gz received SIGTERM/SIGINT"
    assert (marks / "node_got_int").exists(), "the ROS node did not get its graceful SIGINT"
    assert not (marks / "node_got_term").exists()


def test_is_gz_process_by_name_and_cmdline():
    import bench_world as bw

    assert bw.is_gz_process(1, "gz") and bw.is_gz_process(1, "ruby3.2") and bw.is_gz_process(1, "ruby")
    assert not bw.is_gz_process(os.getpid(), "python3")
