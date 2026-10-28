#!/usr/bin/env python3
"""Sample RAM and CPU of a process tree to CSV until SIGTERM/SIGINT (plain python3, Linux /proc).

Usage: proc_sampler.py --root-cmd PATTERN --out FILE.csv [--interval 0.5]

The root is the newest bash process executing a script whose path ends in PATTERN (scripts/replay_slam_bag.sh
passes run_live_seyond_dlio_slam.sh: the SLAM in the Docker container, visible from WSL because docker-compose.yml
runs it with pid: host). Every sample covers the root and all its descendants (found again each time, so nodes that
start late are included). One row per process and sample:
  t (epoch s), pid, name, rss_mb, cpu_s (utime+stime, cumulative), threads, mem_available_mb (whole WSL)
name = the executable's base name, or the script's for python3/bash (e.g. dlio_odom_node, graph_based_slam_node).
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time

TICK = os.sysconf("SC_CLK_TCK")


def cmdline(pid: int) -> list[str]:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return [p.decode(errors="replace") for p in f.read().split(b"\0") if p]
    except OSError:
        return []


def find_root(pattern: str) -> int | None:
    """Newest bash/sh process that EXECUTES a script ending in pattern (argv[1]). Processes that only mention it
    (the local `docker compose exec` client, shells with it in a -c string) do not count."""
    best = None
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == os.getpid():
            continue
        cl = cmdline(int(d))
        if len(cl) > 1 and os.path.basename(cl[0]) in ("bash", "sh") and cl[1].endswith(pattern):
            try:
                start = os.stat(f"/proc/{d}").st_mtime
            except OSError:
                continue
            if best is None or start > best[0]:
                best = (start, int(d))
    return best[1] if best else None


def children_map() -> dict[int, list[int]]:
    kids: dict[int, list[int]] = {}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            continue
        kids.setdefault(ppid, []).append(int(d))
    return kids


def tree(root: int) -> list[int]:
    kids = children_map()
    out, todo = [], [root]
    while todo:
        p = todo.pop()
        out.append(p)
        todo.extend(kids.get(p, []))
    return out


def name_of(pid: int) -> str:
    cl = cmdline(pid)
    if not cl:
        return "?"
    exe = os.path.basename(cl[0])
    if exe.startswith(("python", "bash", "sh")) and len(cl) > 1:
        args = [a for a in cl[1:] if not a.startswith("-")]
        if args:
            base = os.path.basename(args[0])
            return f"ros2 {args[1]}" if base == "ros2" and len(args) > 1 else base
    return exe


def sample(pid: int) -> tuple[float, float, int] | None:
    try:
        with open(f"/proc/{pid}/status") as f:
            st = f.read()
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
    except OSError:
        return None
    rss = next((int(l.split()[1]) for l in st.splitlines() if l.startswith("VmRSS:")), 0) / 1024.0
    thr = next((int(l.split()[1]) for l in st.splitlines() if l.startswith("Threads:")), 0)
    cpu = (int(fields[11]) + int(fields[12])) / TICK          # utime + stime
    return rss, cpu, thr


def mem_available() -> float:
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024.0
    return float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root-cmd", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--wait", type=float, default=60.0, help="seconds to wait for the root process")
    a = ap.parse_args()
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    t0 = time.time()
    root = None
    while root is None and not stop and time.time() - t0 < a.wait:
        root = find_root(a.root_cmd)
        if root is None:
            time.sleep(0.5)
    if root is None:
        print(f"proc_sampler: no process matching {a.root_cmd!r}", file=sys.stderr)
        return 1
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "pid", "name", "rss_mb", "cpu_s", "threads", "mem_available_mb"])
        names: dict[int, str] = {}
        while not stop:
            now = time.time()
            if not os.path.exists(f"/proc/{root}"):
                break
            avail = mem_available()
            for pid in tree(root):
                s = sample(pid)
                if s is None:
                    continue
                if pid not in names:
                    names[pid] = name_of(pid)
                w.writerow([f"{now:.3f}", pid, names[pid], f"{s[0]:.1f}", f"{s[1]:.2f}", s[2], f"{avail:.0f}"])
            f.flush()
            time.sleep(max(0.0, a.interval - (time.time() - now)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
