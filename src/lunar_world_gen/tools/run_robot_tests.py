#!/usr/bin/python3
"""Headless full-robot acceptance runs (docs/DESIGN.md section 13, "Gazebo, robot").

Run with the SYSTEM python3 from WSL (ROS 2 Jazzy is sourced by this script for the launched
processes, so a plain shell is enough):

    /usr/bin/python3 tools/run_robot_tests.py --out DIR \
        [--world SDF | --config CFG [--variant '']] [--sites flat,slope,rock] [--repeats 3] \
        [--launch auto|ours|stock] [--site-pose NAME:X,Y,Z[,YAW]]... [--gravity 1.62]

For each launch k (``max(repeats, len(sites))`` launches; launch k drives at
``sites[k % len(sites)]``):
  1. starts the simulation in its own session (setsid) with GZ_PARTITION / ROS_DOMAIN_ID, a
     private ROS_HOME (isolates the spawner file lock), the WSL GPU + WSLg display variables and
     line-buffered output; the launch log is time-stamped;
     * ``ours``  = ``ros2 launch src/lunar_world_gen/launch/apollo16_sim.launch.py site:=.. world:=..``
       (only arguments the launch file declares are passed),
     * ``stock`` = ``ros2 launch togo_gz sim_gz.launch.py world:=ABS rviz:=false robot_x/y/z:=..``
       (no yaw argument: the robot always faces +x),
     * ``auto``  = ours if the file exists, else stock;
  2. polls ``/controller_manager/list_controllers`` (the service behind
     ``ros2 control list_controllers``; called through a tiny rclpy helper because the CLI
     spawns a detached ros2 daemon that would outlive the test) until joint_state_broadcaster
     and platform_velocity_controller are active;
  3. runs ``tools/rover_drive_test.py`` with the site's sequence (flat: ``standard`` preset;
     slope: climb 0.4 m/s 15 s; rock: 0.3 m/s 15 s; turn: in-place 0.6 rad/s 6 s);
  4. stops the launch: the gz sim processes of the launch group get SIGKILL first (never
     SIGTERM/SIGINT: gz segfaults when asked to shut down and WSL then writes a 1.5-3.6 GB crash
     dump); ros2 launch then shuts its ROS nodes down itself (``on_exit_shutdown`` of the server),
     the remaining ROS processes get SIGINT (graceful) and, after a grace period, SIGKILL. Then
     the launch log is parsed: startup timeline, spawner failures (``process has died`` with a
     non-zero exit code before the stop), controller activation; the gz server's
     /proc/<pid>/maps gives the renderer.
Writes DIR/robot_tests.json with per-launch records, the DESIGN pass/fail checks and a summary.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent
REPO_SRC = PKG_ROOT.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PKG_ROOT))
import bench_world as bw  # noqa: E402

ROS_SETUP = ["/opt/ros/jazzy/setup.bash", "/home/nargon/togo_ws_native_clean/install/setup.bash"]
OUR_LAUNCH = PKG_ROOT / "launch" / "apollo16_sim.launch.py"
# DESIGN 13 forward-drive criterion (decision after phase 2): the start is traction-limited at lunar
# gravity (mu g = 0.65 m/s^2 < the controller's 2 m/s^2 limit), so the run passes on the steady-state
# speed and on a distance threshold of 0.93 of the nominal 3 m segment.
FWD_SPEED_MIN = 0.95        # steady-state ground-truth speed / commanded speed
FWD_DIST_MIN = 0.93         # ground-truth forward distance / nominal commanded distance
TURN_YAWRATE_MIN = 0.85
AIRBORNE_MAX = 0.02
DRIVE_TOOL = HERE / "rover_drive_test.py"
ANSI = re.compile(r"\x1b\[[0-9;]*m|\[[0-9;]*m")

SITE_SEQUENCES = {
    "flat": {"preset": "standard"},
    "slope": {"sequence": [{"name": "climb", "vx": 0.4, "wz": 0.0, "duration": 15.0},
                           {"name": "stop", "vx": 0.0, "wz": 0.0, "duration": 3.0}]},
    "rock": {"sequence": [{"name": "traverse", "vx": 0.3, "wz": 0.0, "duration": 15.0},
                          {"name": "stop", "vx": 0.0, "wz": 0.0, "duration": 3.0}]},
    "turn": {"sequence": [{"name": "turn", "vx": 0.0, "wz": 0.6, "duration": 6.0},
                          {"name": "stop", "vx": 0.0, "wz": 0.0, "duration": 3.0}]},
}

TIMELINE = [  # (event, regex) - first occurrence in the launch log (rover agent's timeline.py + extras)
    ("gz_process_started", r"\[INFO\] \[(gazebo|gz_sim|ruby)[\w-]*\]: process started"),
    ("world_initialized", r"World \[.*\] initialized with"),
    ("create_started", r"\[INFO\] \[create-\d+\]: process started"),
    ("bridge_started", r"\[INFO\] \[parameter_bridge-\d+\]: process started"),
    ("rsp_started", r"\[INFO\] \[robot_state_publisher-\d+\]: process started"),
    ("spawners_started", r"\[INFO\] \[spawner-\d+\]: process started"),
    ("entity_created", r"Entity creation successful"),
    ("cm_loading", r"Loading controller_manager"),
    ("hardware_initialized", r"Resource Manager has been successfully initialized"),
    ("render_engine_loading", r"Loading plugin \[.*gz-rendering-ogre2"),
    ("lidar_advertised", r"Lidar points for"),
    ("jsb_active", r"Configured and activated joint_state_broadcaster"),
    ("pvc_active", r"Configured and activated platform_velocity_controller"),
    ("spawner_lock_retry", r"Failed to acquire lock"),
    ("sensors_ready", r"\[apollo16_sim\] sensors ready"),          # world owner's launch (gate passed)
    ("sensor_wait_failed", r"\[apollo16_sim\] WARNING: wait_for_sensors exited"),
    ("ready_banner", r"\[apollo16_sim\] READY at"),                # wait_ready passed (controllers + /clock)
    ("not_ready", r"\[apollo16_sim\] WARNING: NOT READY"),
]
# the launch's own clock in the banner: seconds since its launch_setup() ran (slightly after `ros2 launch`)
READY_AT = re.compile(r"\[apollo16_sim\] READY at \+([0-9.]+) s")
OUR_SPAWNER_EXIT = re.compile(r"\[apollo16_sim\] (\w+) spawner exited with code (-?\d+)")
SPAWNER_DIED = re.compile(r"\[ERROR\] \[(spawner[\w-]*)\]: process has died \[pid (\d+), exit code (-?\d+), cmd '([^']*)'")
ANY_DIED = re.compile(r"\[ERROR\] \[([\w.-]+)\]: process has died \[pid (\d+), exit code (-?\d+)")

LIST_CONTROLLERS_PY = r"""
import json, sys
import rclpy
from controller_manager_msgs.srv import ListControllers
rclpy.init()
node = rclpy.create_node('validation_list_controllers')
cli = node.create_client(ListControllers, sys.argv[1])
out = {'service': bool(cli.wait_for_service(timeout_sec=float(sys.argv[2])))}
if out['service']:
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=float(sys.argv[3]))
    if fut.done() and fut.result() is not None:
        out['controllers'] = [{'name': c.name, 'state': c.state, 'type': c.type} for c in fut.result().controller]
print(json.dumps(out))
node.destroy_node()
rclpy.shutdown()
"""


# ----------------------------------------------------------------------------------------------
# environment

def ros_env(partition: str, domain: int, ros_home: Path, line_buffer: bool = True) -> dict:
    env = bw.gz_env(partition, domain, ros_home, display=True)
    env.update({"PYTHONUNBUFFERED": "1", "RCUTILS_LOGGING_BUFFERED_STREAM": "0", "RCUTILS_COLORIZED_OUTPUT": "0"})
    if line_buffer and os.path.exists("/usr/libexec/coreutils/libstdbuf.so"):
        env["LD_PRELOAD"] = "/usr/libexec/coreutils/libstdbuf.so"
        env["_STDBUF_O"] = "L"
    return env


def ros_cmd(args: list[str]) -> list[str]:
    """Wrap a command so it runs with ROS sourced and the WSL GPU libraries first."""
    src = " && ".join(f"source {shlex.quote(s)}" for s in ROS_SETUP if os.path.exists(s))
    script = (f"{src} && export LD_LIBRARY_PATH=/usr/lib/wsl/lib:${{LD_LIBRARY_PATH:-}} && "
              f"exec \"$@\"") if src else 'exec "$@"'
    return ["bash", "-c", script, "_"] + args


def declared_launch_args(path: Path) -> list[str]:
    try:
        txt = path.read_text()
    except OSError:
        return []
    return re.findall(r"DeclareLaunchArgument\(\s*['\"]([\w-]+)['\"]", txt)


# ----------------------------------------------------------------------------------------------
# a running launch

class Launch:
    def __init__(self, cmd: list[str], env: dict, run_dir: Path, limit_s: int):
        # no --foreground: at the limit, timeout SIGKILLs the whole group (gz must never get SIGTERM)
        self.cmd = ["timeout", "-s", "KILL", str(int(limit_s))] + cmd
        self.stop_requested_at: float | None = None
        self.env = env
        self.run_dir = run_dir
        self.proc: subprocess.Popen | None = None
        self.pgid: int | None = None
        self.t0 = 0.0
        self.events: dict[str, float] = {}
        self.deaths: list[dict] = []
        self.spawner_exits: list[dict] = []
        self.ready_launch_s: float | None = None     # "+T s" printed in the READY banner
        self.lines = 0
        self._reader: threading.Thread | None = None

    def start(self) -> None:
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.t0 = bw.clock()
        (self.run_dir / "launch_cmd.txt").write_text(" ".join(shlex.quote(c) for c in self.cmd) + "\n")
        self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, env=self.env, start_new_session=True)
        self.pgid = self.proc.pid
        (self.run_dir / "pgid").write_text(f"{self.pgid}\n")
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self) -> None:
        with open(self.run_dir / "launch.log", "w", errors="replace") as log:
            for raw in self.proc.stdout:
                t = bw.clock() - self.t0
                line = ANSI.sub("", raw.decode("utf-8", "replace")).rstrip("\n")
                log.write(f"{t:9.3f} {line}\n")
                log.flush()
                self.lines += 1
                for name, pat in TIMELINE:
                    if name not in self.events and re.search(pat, line):
                        self.events[name] = round(t, 3)
                ra = READY_AT.search(line)
                if ra and self.ready_launch_s is None:
                    self.ready_launch_s = float(ra.group(1))
                se = OUR_SPAWNER_EXIT.search(line)
                if se:
                    self.spawner_exits.append({"t": round(t, 3), "controller": se.group(1),
                                               "exit_code": int(se.group(2))})
                m = ANY_DIED.search(line)
                if m and self.stop_requested_at is None:      # deaths caused by our stop do not count
                    sm = SPAWNER_DIED.search(line)
                    self.deaths.append({"t": round(t, 3), "node": m.group(1), "pid": int(m.group(2)),
                                        "exit_code": int(m.group(3)),
                                        "cmd": sm.group(4)[:300] if sm else None})

    def server_pid(self) -> int | None:
        if not self.pgid:
            return None
        cands = [(p, c) for p, c in bw.group_pids(self.pgid) if c in ("ruby", "gz", "ruby3.2", "gz-sim-server")]
        if not cands:
            return None
        return max(cands, key=lambda pc: bw.mem_status(pc[0]).get("VmRSS", 0))[0]

    def stop(self, ros_grace_s: float = 10.0, int_grace_s: float = 10.0) -> dict:
        """Stop the launch group without ever sending SIGTERM/SIGINT to a gz process.

        1. SIGKILL every gz sim process of the group (server / GUI; ``bw.gz_pids``) and wait until
           they are gone; 2. ros2 launch reacts to the server's exit (``on_exit_shutdown``) and
           stops its ROS nodes gracefully - wait up to ``ros_grace_s``; 3. SIGINT to the remaining
           group (ROS processes only now, re-checked for gz first), wait up to ``int_grace_s``;
           4. SIGKILL whatever is left."""
        info: dict = {"pgid": self.pgid, "gz_signal": "SIGKILL"}
        if not self.pgid:
            return info
        t_stop = bw.clock()
        self.stop_requested_at = t_stop - self.t0

        def kill_gz() -> list[int]:
            pids = bw.gz_pids(self.pgid)
            for p in pids:
                try:
                    os.kill(p, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            tg = bw.clock()
            while bw.gz_pids(self.pgid) and bw.clock() - tg < 10:
                time.sleep(0.1)
            return pids

        info["gz_killed"] = kill_gz()
        while bw.clock() - t_stop < ros_grace_s and bw.group_pids(self.pgid):
            time.sleep(0.25)
        if bw.group_pids(self.pgid):
            info["gz_killed_late"] = kill_gz()
            if not bw.gz_pids(self.pgid):
                info["sigint_ros"] = bw.kill_group(self.pgid, signal.SIGINT)
                t1 = bw.clock()
                while bw.clock() - t1 < int_grace_s and bw.group_pids(self.pgid):
                    time.sleep(0.25)
        if bw.group_pids(self.pgid):
            info["needed_sigkill"] = bw.kill_group(self.pgid, signal.SIGKILL)
        try:
            self.proc.wait(timeout=10)
        except Exception:  # noqa: BLE001
            pass
        tg = bw.clock()
        while bw.group_pids(self.pgid, include_zombies=True) and bw.clock() - tg < 15:
            time.sleep(0.25)
        if self._reader:
            self._reader.join(timeout=5)
        info["remaining_in_group"] = len(bw.group_pids(self.pgid, include_zombies=True))
        info["stop_s"] = round(bw.clock() - t_stop, 2)
        return info


def list_controllers(env: dict, cm: str = "/controller_manager", svc_timeout: float = 5.0,
                     call_timeout: float = 10.0) -> dict:
    """One ListControllers call through rclpy (bounded by `timeout`)."""
    cmd = ros_cmd(["timeout", "-k", "5", str(int(svc_timeout + call_timeout + 15)), "/usr/bin/python3", "-c",
                   LIST_CONTROLLERS_PY, f"{cm}/list_controllers", str(svc_timeout), str(call_timeout)])
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=svc_timeout + call_timeout + 30,
                           start_new_session=True)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
        return json.loads(line[-1]) if line else {"service": False, "rc": r.returncode, "err": r.stderr[-300:]}
    except Exception as e:  # noqa: BLE001
        return {"service": False, "error": str(e)}


def wait_controllers(env: dict, launch: Launch, timeout_s: float, wanted=("joint_state_broadcaster",
                                                                           "platform_velocity_controller"),
                     poll_s: float = 3.0) -> dict:
    t_end = bw.clock() + timeout_s
    polls = 0
    last = {}
    while bw.clock() < t_end:
        if launch.proc.poll() is not None and not bw.group_pids(launch.pgid):
            return {"active": False, "reason": "launch exited", "polls": polls, "last": last}
        last = list_controllers(env)
        polls += 1
        states = {c["name"]: c["state"] for c in last.get("controllers", [])}
        if all(states.get(w) == "active" for w in wanted):
            return {"active": True, "t_s": round(bw.clock() - launch.t0, 2), "polls": polls, "states": states}
        time.sleep(poll_s)
    return {"active": False, "reason": "timeout", "polls": polls, "last": last}


# ----------------------------------------------------------------------------------------------
# drive tests

class LoadSampler:
    """Background ``bw.load_point`` samples (wall time, busy ticks, gz server ticks, process table)
    every ``period`` s. Wall time (not the monotonic clock) because the drive tool's stats.csv stamps
    gz stats with time.time(); ``drive_load_series`` bins both into RTF vs other agents' CPU use.
    Also samples the gz server's per-thread CPU ticks (``threads``: (wall, {tid: (comm, ticks)})) for
    ``thread_cpu_drive`` (main simulation thread = the server's main thread, tid == pid).
    """

    def __init__(self, server_pid: int | None, period: float = 0.5):
        self.pid, self.period = server_pid, period
        self.points: list[tuple] = []
        self.threads: list[tuple[float, dict]] = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.points.append(bw.load_point(self.pid, time.time()))
            if self.pid:
                th = bw.threads_cpu(self.pid)
                if th:
                    self.threads.append((time.time(), th))
            self._stop.wait(self.period)

    def stop(self) -> list[tuple]:
        self._stop.set()
        self._t.join(timeout=2)
        return list(self.points)


def _read_stats_csv(stats_csv: Path) -> list[tuple[float, float, float]]:
    """(wall, sim_time, real_time) rows of the drive tool's gz stats, sorted by wall time."""
    rows = []
    try:
        with open(stats_csv) as f:
            for r in csv.DictReader(f):
                rows.append((float(r["wall"]), float(r["sim_time"]), float(r["real_time"])))
    except (OSError, KeyError, ValueError):
        return []
    rows.sort()
    return rows


def segment_rtf(stats_csv: Path, segments: list[dict]) -> dict:
    """RTF (sim / gz real time) of each drive segment (sim-time interval [t0, t1]) from stats.csv."""
    st = _read_stats_csv(stats_csv)
    out = {}
    for s in segments or []:
        t0, t1 = s.get("t0"), s.get("t1")
        if t0 is None or t1 is None:
            continue
        S = [r for r in st if t0 <= r[1] <= t1]
        if len(S) >= 2 and S[-1][2] > S[0][2]:
            out[s.get("name")] = round((S[-1][1] - S[0][1]) / (S[-1][2] - S[0][2]), 4)
    return out


def thread_cpu_drive(threads: list[tuple[float, dict]], stats_csv: Path, server_pid: int | None,
                     segments: list[dict] | None = None) -> dict:
    """CPU seconds per SIMULATED second of the gz server's main thread (tid == pid, where the
    simulation loop and Bullet run) and of the whole server, over the drive (first to last gz stats
    sample of the drive tool) and over each moving segment. Thread samples come every ~0.5 s, so
    each interval is snapped to the samples inside it and its sim time is interpolated from the
    stats (wall -> sim)."""
    st = _read_stats_csv(stats_csv)
    if len(st) < 2 or len(threads) < 2 or not server_pid:
        return {"available": False}
    import numpy as np
    W = np.array([r[0] for r in st])
    Sm = np.array([r[1] for r in st])

    def sim_at(w: float) -> float:
        return float(np.interp(w, W, Sm))

    def wall_at(s: float) -> float:
        return float(np.interp(s, Sm, W))

    def interval(w0: float, w1: float) -> dict | None:
        smp = [t for t in threads if w0 - 0.05 <= t[0] <= w1 + 0.05]
        if len(smp) < 2:
            return None
        (a, th0), (b, th1) = smp[0], smp[-1]
        dsim, dwall = sim_at(b) - sim_at(a), b - a
        if dsim <= 0.2 or dwall <= 0:
            return None
        per: dict[str, float] = {}
        tot = 0.0
        for tid, (comm, t1) in th1.items():
            d = (t1 - th0[tid][1]) if tid in th0 else t1     # a thread started meanwhile: all its ticks
            d = max(d, 0) / bw.CLK
            tot += d
            per[comm] = per.get(comm, 0.0) + d
        main = th1.get(server_pid)
        m = ((main[1] - th0.get(server_pid, (None, main[1]))[1]) / bw.CLK) if main else None
        top = sorted(per.items(), key=lambda kv: -kv[1])[:6]
        return {"wall_s": round(dwall, 2), "sim_s": round(dsim, 3), "rtf": round(dsim / dwall, 4),
                "main_thread_cpu_s_per_sim_s": None if m is None else round(m / dsim, 3),
                "main_thread_cores": None if m is None else round(m / dwall, 3),
                "server_cpu_s_per_sim_s": round(tot / dsim, 3), "server_cores": round(tot / dwall, 3),
                "top_threads_cpu_s_per_sim_s": [(c, round(v / dsim, 3)) for c, v in top]}

    out: dict = {"available": True, "main_tid": server_pid, "n_samples": len(threads),
                 "tool": interval(st[0][0], st[-1][0]), "segments": {}}
    timed = [s for s in segments or [] if s.get("t0") is not None and s.get("t1") is not None]
    # "drive" = the command sequence (pre-roll to post-roll), the interval of the drive tool's RTF
    out["drive"] = interval(wall_at(timed[0]["t0"]), wall_at(timed[-1]["t1"])) if timed else out["tool"]
    for s in segments or []:
        if (s.get("vx") or s.get("wz")) and s.get("t0") is not None and s.get("t1") is not None:
            r = interval(wall_at(s["t0"]), wall_at(s["t1"]))
            if r:
                out["segments"][s.get("name")] = r
    return out


def drive_load_series(points: list[tuple], stats_csv: Path, own_pgids=(), bin_s: float = 2.0) -> dict:
    """RTF per bin (from the drive tool's gz stats) against the machine load sampled meanwhile."""
    stats = []
    try:
        with open(stats_csv) as f:
            for r in csv.DictReader(f):
                stats.append((float(r["wall"]), float(r["sim_time"]), float(r["real_time"]), 0, 0.0))
    except (OSError, KeyError, ValueError):
        return {"available": False}
    if not stats or len(points) < 2:
        return {"available": False}
    a, b = stats[0][0], stats[-1][0]
    pts = [p for p in points if a <= p[0] <= b]
    out = bw.contention_series(pts, stats, bin_s, own_pgids)
    out["available"] = True
    out["bins"] = [dict(r, t0=round(r["t0"] - a, 2)) for r in out.get("bins", [])]
    return out


def run_drive(env: dict, out: Path, site: str, gravity: float | None, limit_s: float, settle: bool = True,
              preset: str | None = None, sequence: list | None = None) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    args = ["/usr/bin/python3", str(DRIVE_TOOL), "--out", str(out), "--max-wall", str(int(limit_s - 30))]
    if preset:
        args += ["--preset", preset]
    if sequence:
        args += ["--sequence", json.dumps(sequence)]
    if gravity:
        args += ["--gravity", str(gravity)]
    if not settle:
        args += ["--no-settle"]
    cmd = ros_cmd(["timeout", "--signal=INT", "-k", "20", str(int(limit_s))] + args)
    t0 = bw.clock()
    with open(out / "tool.log", "w") as log:
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            rc = p.wait(timeout=limit_s + 60)
        except subprocess.TimeoutExpired:
            for sig in (signal.SIGINT, signal.SIGKILL):
                try:
                    os.killpg(p.pid, sig)
                except ProcessLookupError:
                    pass
                time.sleep(5)
            rc = p.wait(timeout=10)
    rec = {"site": site, "rc": rc, "wall_s": round(bw.clock() - t0, 1), "out": str(out), "pgid": p.pid}
    sp = out / "summary.json"
    if sp.exists():
        rec["summary"] = json.loads(sp.read_text())
    return rec


def stall_metric(gt_csv: Path, t0: float, t1: float, window: float = 1.0, skip: float = 1.5,
                 v_min: float = 0.05) -> dict:
    """Longest window (>= `window` s) inside [t0+skip, t1] with mean horizontal speed < v_min."""
    rows = []
    try:
        with open(gt_csv) as f:
            for r in csv.DictReader(f):
                rows.append((float(r["t_sim"]), float(r["x"]), float(r["y"])))
    except (OSError, KeyError, ValueError):
        return {"available": False}
    rows = [r for r in rows if t0 + skip <= r[0] <= t1]
    if len(rows) < 10:
        return {"available": False}
    stalls = 0
    longest = 0.0
    j = 0
    min_v = float("inf")
    for i in range(len(rows)):
        while rows[i][0] - rows[j][0] > window:
            j += 1
        if rows[i][0] - rows[j][0] >= 0.9 * window:
            d = math.hypot(rows[i][1] - rows[j][1], rows[i][2] - rows[j][2])
            v = d / (rows[i][0] - rows[j][0])
            min_v = min(min_v, v)
            if v < v_min:
                stalls += 1
                longest = max(longest, rows[i][0] - rows[j][0])
    return {"available": True, "min_window_speed_mps": min_v, "stalled_windows": stalls, "stalled": stalls > 0}


MU_GROUND = 0.40    # physics.ground_mu of the world (patched plugin honours it on the mesh collisions)


def traction_limited_reference(out_dir: Path, seg: dict, gravity: float | None, mu: float = MU_GROUND) -> dict:
    """Distance the rover can cover in a straight segment when it follows the controller-limited
    command (``cmd_vel_out``) but cannot accelerate faster than traction allows,
    a_max = g (mu cos(theta) - sin(theta)) on the segment's mean grade theta (uphill > 0).
    Returns the limited and the traction-limited command distances and the grade."""
    t0, t1 = seg.get("t0"), seg.get("t1")
    res: dict = {}
    if t0 is None or t1 is None or not gravity:
        return res
    try:
        with open(out_dir / "cmd_out.csv") as f:
            co = sorted((float(r["t_sim"]), float(r["vx"])) for r in csv.DictReader(f))
        with open(out_dir / "gt.csv") as f:
            gt = sorted((float(r["t_sim"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in csv.DictReader(f))
    except (OSError, KeyError, ValueError):
        return res
    co = [r for r in co if t0 <= r[0] <= t1]
    g = [r for r in gt if t0 <= r[0] <= t1]
    if len(co) < 3 or len(g) < 3:
        return res
    dh = math.hypot(g[-1][1] - g[0][1], g[-1][2] - g[0][2])
    theta = math.atan2(g[-1][3] - g[0][3], dh) if dh > 0.05 else 0.0
    a_max = gravity * (mu * math.cos(theta) - math.sin(theta))
    v_prev, t_prev = 0.0, t0
    d_lim = d_tr = 0.0
    c_prev = 0.0
    for t, c in co:
        dt = t - t_prev
        v = min(c, v_prev + max(a_max, 0.0) * dt) if c >= v_prev else c
        d_lim += 0.5 * (c + c_prev) * dt
        d_tr += 0.5 * (v + v_prev) * dt
        v_prev, c_prev, t_prev = v, c, t
    res.update({"grade_deg": round(math.degrees(theta), 3), "a_traction_mps2": round(a_max, 4),
                "cmd_dist_limited_m": round(d_lim, 4), "cmd_dist_traction_m": round(d_tr, 4),
                "mu_assumed": mu})
    return res


def evaluate_drive(site: str, rec: dict, gravity: float | None) -> dict:
    """DESIGN 13 robot criteria for one drive run."""
    S = rec.get("summary") or {}
    segs = {s.get("name"): s for s in S.get("segments", [])}
    run = S.get("run", {})
    st = (S.get("status") or {}).get("status")
    out: dict = {"tool_status": st, "rc": rec.get("rc")}
    rtf = run.get("rtf_from_clocks") or run.get("rtf_mean")
    out["rtf_driving"] = rtf
    out["rtf_pass"] = rtf is not None and rtf >= 0.9
    if rec.get("out"):
        out["rtf_segments"] = segment_rtf(Path(rec["out"]) / "stats.csv", S.get("segments", []))
    if site == "flat":
        f, t = segs.get("fwd", {}), segs.get("turn", {})
        out["fwd_dist_ratio"] = f.get("dist_ratio_gt_over_cmd")
        out["fwd_speed_tracking"] = f.get("speed_tracking_ratio")
        out["turn_yawrate_tracking"] = t.get("yawrate_tracking_ratio")
        out["airborne_frac_fwd"] = f.get("ballistic_frac")
        out["fwd_dist_m"] = f.get("gt_dist_long_m")
        out["fwd_cmd_dist_nominal_m"] = f.get("cmd_dist_nominal_m")
        if rec.get("out") and f:
            ref = traction_limited_reference(Path(rec["out"]), f, gravity)
            out["fwd_reference"] = ref
            d = f.get("gt_dist_long_m")
            if d is not None and ref.get("cmd_dist_limited_m"):
                out["fwd_dist_ratio_vs_limited"] = round(d / ref["cmd_dist_limited_m"], 4)
            if d is not None and ref.get("cmd_dist_traction_m"):
                out["fwd_dist_ratio_vs_traction"] = round(d / ref["cmd_dist_traction_m"], 4)
        # DESIGN 13 (post phase 2): steady-state speed >= 0.95 of the command AND distance >= 0.93 of
        # the nominal segment (the friction-limited start costs ~0.2 m of 3 m at mu 0.4, 1.62 m/s^2)
        out["pass_fwd_speed"] = (out["fwd_speed_tracking"] or 0) >= FWD_SPEED_MIN
        out["pass_fwd_distance"] = (out["fwd_dist_ratio"] or 0) >= FWD_DIST_MIN
        out["pass_fwd"] = bool(out["pass_fwd_speed"] and out["pass_fwd_distance"])
        out["fwd_rule"] = (f"steady-state speed >= {FWD_SPEED_MIN} of the command and distance >= "
                           f"{FWD_DIST_MIN} of the nominal segment")
        out["pass_turn"] = (out["turn_yawrate_tracking"] or 0) >= TURN_YAWRATE_MIN
        out["pass_airborne"] = (out["airborne_frac_fwd"] is not None and out["airborne_frac_fwd"] <= AIRBORNE_MAX) if gravity else None
        out["pass"] = bool(st == "ok" and out["pass_fwd"] and out["pass_turn"] and out["pass_airborne"] is not False
                           and out["rtf_pass"])
    elif site in ("slope", "rock"):
        name = "climb" if site == "slope" else "traverse"
        s = segs.get(name, {})
        out["dist_ratio"] = s.get("dist_ratio_gt_over_cmd")
        sx, ex = run.get("start_xyz"), run.get("end_xyz")
        if sx and ex:
            dh = math.hypot(ex[0] - sx[0], ex[1] - sx[1])
            out["dz_m"] = ex[2] - sx[2]
            out["climb_angle_deg"] = math.degrees(math.atan2(ex[2] - sx[2], dh)) if dh > 0 else None
        if s.get("t0") is not None:
            out["stall"] = stall_metric(Path(rec["out"]) / "gt.csv", s["t0"], s["t1"])
        # informational (not judged here): bounce, attitude and slip over the segment
        out["airborne_frac"] = s.get("ballistic_frac")
        out["speed_tracking"] = s.get("speed_tracking_ratio")
        out["long_slip_ratio"] = s.get("long_slip_ratio_segment")
        out["max_abs_pitch_deg"] = s.get("max_abs_pitch_deg")
        out["max_abs_roll_deg"] = s.get("max_abs_roll_deg")
        if site == "slope":
            out["pass"] = bool(st == "ok" and (out["dist_ratio"] or 0) >= 0.8 and (out.get("climb_angle_deg") or 0) >= 5.0)
            out["rule"] = "climbed: >= 80 % of the commanded distance and mean climb angle >= 5 deg"
        else:
            stalled = out.get("stall", {}).get("stalled")
            out["pass"] = bool(st == "ok" and (out["dist_ratio"] or 0) >= 0.8 and stalled is False)
            out["rule"] = "traversed: >= 80 % of the commanded distance, no 1 s window slower than 0.05 m/s"
    elif site == "turn":
        t = segs.get("turn", {})
        out["turn_yawrate_tracking"] = t.get("yawrate_tracking_ratio")
        out["turn_dyaw_deg"] = t.get("gt_dyaw_deg")              # informational
        out["turn_drift_m"] = t.get("gt_disp_m")
        out["airborne_frac"] = t.get("ballistic_frac")
        out["pass"] = bool(st == "ok" and (out["turn_yawrate_tracking"] or 0) >= TURN_YAWRATE_MIN)
    return out


# ----------------------------------------------------------------------------------------------
# main

def load_sites(world_info: Path | None, extra: list[str]) -> dict:
    sites: dict = {}
    if world_info and world_info.exists():
        import yaml
        from lunar_world_gen.validate.static_checks import parse_sites
        sites.update(parse_sites(yaml.safe_load(world_info.read_text())))
    for s in extra:
        name, _, rest = s.partition(":")
        v = bw.parse_pose(rest)
        sites[name] = {"x": v[0], "y": v[1], "z": v[2], "yaw": v[3] if len(v) > 3 else 0.0, "description": "cli"}
    return sites


def build_launch_cmd(kind: str, world: Path, site: str, pose: dict, extra: list[str],
                     world_info: Path | None = None, site_in_world_info: bool = True) -> tuple[list[str], dict]:
    facts: dict = {"launch": kind}
    if kind == "ours":
        decl = declared_launch_args(OUR_LAUNCH)
        facts["declared_args"] = decl
        args = ["ros2", "launch", str(OUR_LAUNCH)]
        cand = {"world": str(world), "rviz": "false", "use_sim_time": "true"}
        if site_in_world_info:
            cand["site"] = site
            if world_info is not None:
                cand["world_info"] = str(world_info)
        else:  # pose given on our command line: pass it explicitly (the launch then ignores site:=)
            cand.update({"robot_x": f"{pose['x']:.4f}", "robot_y": f"{pose['y']:.4f}",
                         "robot_z": f"{pose['z']:.4f}", "robot_yaw": f"{pose.get('yaw', 0.0):.6f}"})
        args += [f"{k}:={v}" for k, v in cand.items() if k in decl]
        facts["passed_args"] = [a for a in args[3:]]
    else:
        args = ["ros2", "launch", "togo_gz", "sim_gz.launch.py", f"world:={world}", "rviz:=false",
                "use_sim_time:=true", f"robot_x:={pose['x']:.4f}", f"robot_y:={pose['y']:.4f}",
                f"robot_z:={pose['z']:.4f}"]
        if abs(pose.get("yaw", 0.0)) > 1e-6:
            facts["yaw_not_applied"] = pose["yaw"]
    args += extra
    return args, facts


def run(a: argparse.Namespace) -> dict:
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    world = Path(a.world).resolve() if a.world else None
    world_info = Path(a.world_info).resolve() if a.world_info else None
    if a.config:
        from lunar_world_gen.config import load_config
        from lunar_world_gen.layout import Layout
        lay = Layout(load_config(a.config))
        world = world or lay.sdf_path(a.variant)
        world_info = world_info or lay.world_info_path
    if world is None or not world.exists():
        raise SystemExit(f"world SDF not found: {world}")
    if world_info is None and (world.parent / "world_info.yaml").exists():
        world_info = world.parent / "world_info.yaml"
    sites = load_sites(world_info, a.site_pose)
    wanted = [s for s in a.sites.split(",") if s]
    missing = [s for s in wanted if s not in sites]
    kind = a.launch
    if kind == "auto":
        kind = "ours" if OUR_LAUNCH.exists() else "stock"
    if missing and kind == "stock":
        raise SystemExit(f"no pose for sites {missing} (world_info: {world_info}); use --site-pose NAME:X,Y,Z,YAW")
    n_launch = max(a.repeats, len(wanted))
    res: dict = {"kind": "robot_tests", "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "world": str(world),
                 "world_info": str(world_info) if world_info else None, "sites": sites, "launch_kind": kind,
                 "role": a.role, "label": a.label, "clock": "monotonic",
                 "partition": a.partition, "domain": a.domain, "gravity": a.gravity, "launches": [],
                 "machine_before": bw.machine_state()}
    stop_flag = {"stop": False}

    def on_signal(signum, _frame):
        stop_flag["stop"] = True
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    try:
        for k in range(n_launch):
            site = wanted[k % len(wanted)]
            pose = sites.get(site, {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0})
            run_dir = out / f"launch_{k:02d}_{site}"
            env = ros_env(a.partition, a.domain, run_dir / "ros_home", not a.no_line_buffer)
            in_wi = site in sites and sites[site].get("description") != "cli"
            cmd, facts = build_launch_cmd(kind, world, site, pose, a.launch_arg, world_info, in_wi)
            L = Launch(ros_cmd(cmd), env, run_dir, a.launch_limit)
            rec: dict = {"index": k, "site": site, "pose": pose, **facts, "extra_launch_args": list(a.launch_arg),
                         "machine_before": bw.machine_state()}
            print(f"[run_robot_tests] launch {k}: {site} ({kind})", flush=True)
            try:
                L.start()
                rec["pgid"] = L.pgid
                ctl = wait_controllers(env, L, a.controller_timeout)
                rec["controllers"] = ctl
                pid = L.server_pid()
                if pid:
                    rec["renderer"] = bw.renderer_libs([p for p, _ in bw.group_pids(L.pgid)])
                    rec["server_rss_mb"] = round((bw.mem_status(pid).get("VmRSS") or 0) / 1024, 1)
                if ctl.get("active") and not a.no_drive:
                    seq = SITE_SEQUENCES.get(site, SITE_SEQUENCES["flat"])
                    t_d0, sys0, all0 = bw.clock(), bw.system_busy_ticks(), bw.all_proc_ticks()
                    srv_pid = L.server_pid()
                    sampler = LoadSampler(srv_pid)
                    drive = run_drive(env, run_dir / f"drive_{site}", site, a.gravity, a.drive_limit,
                                      preset=seq.get("preset"), sequence=seq.get("sequence"))
                    pts = sampler.stop()
                    rec["thread_cpu_drive"] = thread_cpu_drive(
                        sampler.threads, run_dir / f"drive_{site}" / "stats.csv", srv_pid,
                        (drive.get("summary") or {}).get("segments"))
                    all1 = bw.all_proc_ticks()
                    ours = {L.pgid} | ({drive["pgid"]} if drive.get("pgid") else set())
                    rec["contention_drive"] = bw.contention(all0, all1, sys0, bw.system_busy_ticks(),
                                                            bw.clock() - t_d0, ours)
                    # gz servers (comm ruby/gz) of OTHER process groups = other simulations running meanwhile
                    rec["contention_drive"]["other_gz_servers"] = sum(
                        1 for c, _, g in all1.values() if c in ("ruby", "gz") and g not in ours)
                    rec["contention_drive"]["note"] = ("other_procs: processes outside the launch and drive-tool "
                                                       "groups alive at the end (incl. ones started meanwhile); "
                                                       "load_series_drive: per 2 s bin, from 0.5 s process samples")
                    rec["load_series_drive"] = drive_load_series(pts, run_dir / f"drive_{site}" / "stats.csv",
                                                                 ours | {os.getpgid(0)})
                    drive["evaluation"] = evaluate_drive(site, drive, a.gravity)
                    rec["drive"] = {k2: v for k2, v in drive.items() if k2 != "summary"}
                    rec["drive"]["summary_brief"] = _brief(drive.get("summary"))
                if a.hold > 0:
                    time.sleep(a.hold)
                pid = L.server_pid()
                if pid:
                    rec["server_hwm_mb"] = round((bw.mem_status(pid).get("VmHWM") or 0) / 1024, 1)
            finally:
                rec["stop"] = L.stop()
                rec["timeline_s"] = dict(L.events)
                rec["ready_banner_launch_s"] = L.ready_launch_s
                rec["process_deaths"] = L.deaths
                rec["spawner_exits"] = L.spawner_exits
                # launch's own "process has died" lines; the world owner's launch also logs each spawner's
                # exit code - both describe the same processes, so the second source is only a fallback
                fails = [d for d in L.deaths if d["node"].startswith("spawner") and d["exit_code"] != 0]
                if not fails:
                    fails = [{"node": f"spawner:{e['controller']}", "exit_code": e["exit_code"], "t": e["t"],
                              "source": "apollo16_sim log"} for e in L.spawner_exits if e["exit_code"] != 0]
                rec["spawner_failures"] = fails
                rec["log_lines"] = L.lines
                res["launches"].append(rec)
                (out / "robot_tests.partial.json").write_text(json.dumps(res, indent=1, default=str))
            ctl = rec.get("controllers", {})
            print(f"[run_robot_tests] launch {k} done: controllers={ctl.get('active')} t={ctl.get('t_s')} "
                  f"spawner_failures={len(rec['spawner_failures'])} "
                  f"drive_pass={rec.get('drive', {}).get('evaluation', {}).get('pass')}", flush=True)
    except KeyboardInterrupt as e:
        res["interrupted"] = str(e)
    res["machine_after"] = bw.machine_state()
    res["checks"] = judge(res)
    res["pass"] = all(c.get("pass") is not False for c in res["checks"].values()) and bool(res["launches"])
    (out / "robot_tests.json").write_text(json.dumps(res, indent=1, default=str))
    try:
        (out / "robot_tests.partial.json").unlink()
    except OSError:
        pass
    return res


def _brief(S: dict | None) -> dict | None:
    if not S:
        return None
    keep = ("name", "dist_ratio_gt_over_cmd", "speed_tracking_ratio", "yawrate_tracking_ratio", "gt_dist_long_m",
            "gt_dyaw_deg", "ballistic_frac", "max_abs_roll_deg", "max_abs_pitch_deg", "stop_dist_m")
    return {"status": (S.get("status") or {}).get("status"), "spawn": S.get("spawn"), "run": S.get("run"),
            "segments": [{k: s.get(k) for k in keep if k in s} for s in S.get("segments", [])]}


def judge(res: dict) -> dict:
    L = res.get("launches", [])
    c: dict = {}
    n = len(L)
    ok_ctl = sum(1 for r in L if r.get("controllers", {}).get("active"))
    fails = sum(len(r.get("spawner_failures", [])) for r in L)
    c["controllers_active"] = {"pass": n > 0 and ok_ctl == n, "launches": n, "active": ok_ctl,
                               "t_active_s": [r.get("controllers", {}).get("t_s") for r in L]}
    c["no_spawner_failure"] = {"pass": n > 0 and fails == 0, "failures": fails,
                               "launches_with_failure": sum(1 for r in L if r.get("spawner_failures"))}
    c["three_launches"] = {"pass": n >= 3, "launches": n}
    ready = [r.get("ready_banner_launch_s") for r in L]
    c["ready_banner"] = {"pass": n > 0 and all(x is not None for x in ready), "launch_s": ready,
                         "tool_s": [r.get("timeline_s", {}).get("ready_banner") for r in L]}
    if res.get("launch_kind") == "stock":     # the stock chain (sim_gz.launch.py) prints no READY banner
        c["ready_banner"].update({"pass": None, "note": "not applicable to the stock launch chain"})
    rtfs = [r["drive"]["evaluation"].get("rtf_driving") for r in L if r.get("drive")]
    rtfs = [x for x in rtfs if x is not None]
    per_site: dict = {}
    for r in L:
        v = (r.get("drive") or {}).get("evaluation", {}).get("rtf_driving")
        if v is not None:
            per_site.setdefault(r.get("site"), []).append(v)
    c["rtf_driving"] = {"pass": (min(rtfs) >= 0.9) if rtfs else None, "values": rtfs, "per_site": per_site}
    for site in ("flat", "slope", "rock", "turn"):
        ev = [r["drive"]["evaluation"] for r in L if r.get("site") == site and r.get("drive")]
        if ev:
            c[f"drive_{site}"] = {"pass": all(e.get("pass") for e in ev), "runs": ev}
    c["d3d12"] = {"pass": all(r.get("renderer", {}).get("d3d12") for r in L) if L else None}
    return c


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Headless robot acceptance runs (DESIGN 13)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--world", help="world SDF (absolute or relative)")
    ap.add_argument("--world-info", help="world_info.yaml (default: next to the SDF)")
    ap.add_argument("--config", help="generator config: world = Layout.sdf_path(variant)")
    ap.add_argument("--variant", default="", help="SDF variant with --config ('' | inspect | stock)")
    ap.add_argument("--sites", default="flat,slope,rock")
    ap.add_argument("--site-pose", action="append", default=[], help="NAME:X,Y,Z[,YAW] (overrides world_info)")
    ap.add_argument("--repeats", type=int, default=3, help="minimum number of launches")
    ap.add_argument("--launch", default="auto", choices=["auto", "ours", "stock"])
    ap.add_argument("--launch-arg", action="append", default=[], help="extra key:=value for ros2 launch")
    ap.add_argument("--gravity", type=float, default=1.62, help="for the airborne metric (0 = off)")
    ap.add_argument("--controller-timeout", type=float, default=300.0)
    ap.add_argument("--launch-limit", type=int, default=900, help="hard wall limit per launch [s]")
    ap.add_argument("--drive-limit", type=float, default=400.0)
    ap.add_argument("--hold", type=float, default=2.0, help="seconds to keep the sim up after driving")
    ap.add_argument("--no-drive", action="store_true", help="only test the launch / controllers")
    ap.add_argument("--no-line-buffer", action="store_true")
    ap.add_argument("--partition", default=bw.DEFAULT_PARTITION)
    ap.add_argument("--domain", type=int, default=bw.DEFAULT_DOMAIN)
    ap.add_argument("--role", default="acceptance", choices=["acceptance", "comparison"],
                    help="'comparison' runs (e.g. 1 launch of the stock variant) are reported as information, "
                         "not judged against DESIGN 13")
    ap.add_argument("--label", default="", help="short name of this series, shown in the report")
    a = ap.parse_args(argv)
    if not a.gravity:
        a.gravity = None
    res = run(a)
    brief = {k: v.get("pass") for k, v in res["checks"].items()}
    print(json.dumps({"pass": res["pass"], **brief}), flush=True)
    print(f"[run_robot_tests] wrote {Path(a.out).resolve() / 'robot_tests.json'}", flush=True)
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    finally:
        sys.stdout.flush()
        os._exit(code)
