#!/usr/bin/env python3
"""Drive the planned benchmark loop for the bag (system python: rclpy + gz-transport bindings).

1. Standstill: zero commands until the rover has been still (ground-truth speed < --still-speed) for --hold s of sim
   time; the hold restarts whenever the rover moves, so the bag starts with a true standstill for IMU calibration.
2. Pure pursuit along route.json for --laps laps on the gz ground-truth pose; slower in turns and on slopes, in-place
   turns for large heading errors; if stuck (< 0.2 m progress in 20 s): back up 1 m and retry, skip 3 m after three
   tries at one spot, abort after 8 stuck events or |roll|/|pitch| > 25 deg or > 3 m off the path.
3. Final standstill: zero commands for --end-hold s.
Publishes geometry_msgs/TwistStamped on /platform_velocity_controller/cmd_vel (20 Hz, sim-time stamps) and
nav_msgs/Odometry ground truth on /ground_truth/odom (frame 'world', child 'base_link_gt', gz pose stamps, ~50 Hz).
Writes OUT/drive.csv (10 Hz) and OUT/drive_summary.json.
"""
import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

CMD = "/platform_velocity_controller/cmd_vel"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rpy(x, y, z, w):
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return r, p, yw


class Driver(Node):
    def __init__(self):
        super().__init__("bag_driver", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, CMD, 10)
        self.gt_pub = self.create_publisher(Odometry, "/ground_truth/odom", 50)
        self.lock = threading.Lock()
        self.pose = None          # (sim, x, y, z, roll, pitch, yaw)
        self.speed = float("nan")
        self._prev = None
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, "/model/togo/pose", self.on_pose)

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name != "togo":
                continue
            o = p.orientation
            sim = p.header.stamp.sec + p.header.stamp.nsec * 1e-9
            r, pi, y = rpy(o.x, o.y, o.z, o.w)
            odo = Odometry()
            odo.header.stamp.sec, odo.header.stamp.nanosec = p.header.stamp.sec, p.header.stamp.nsec
            odo.header.frame_id = "world"
            odo.child_frame_id = "base_link_gt"
            odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z = p.position.x, p.position.y, p.position.z
            odo.pose.pose.orientation.x, odo.pose.pose.orientation.y = o.x, o.y
            odo.pose.pose.orientation.z, odo.pose.pose.orientation.w = o.z, o.w
            with self.lock:
                if self._prev is not None and sim > self._prev[0]:
                    dt = sim - self._prev[0]
                    vx, vy, vz = ((p.position.x - self._prev[1]) / dt, (p.position.y - self._prev[2]) / dt,
                                  (p.position.z - self._prev[3]) / dt)
                    self.speed = math.sqrt(vx * vx + vy * vy + vz * vz)
                    cy, sy = math.cos(y), math.sin(y)
                    odo.twist.twist.linear.x = cy * vx + sy * vy      # heading-frame velocity (approximate body frame)
                    odo.twist.twist.linear.y = -sy * vx + cy * vy
                    odo.twist.twist.linear.z = vz
                    odo.twist.twist.angular.z = wrap(y - self._prev[4]) / dt
                if self._prev is None or sim > self._prev[0]:
                    self._prev = (sim, p.position.x, p.position.y, p.position.z, y)
                self.pose = (sim, p.position.x, p.position.y, p.position.z, r, pi, y)
            self.gt_pub.publish(odo)
            break

    def state(self):
        with self.lock:
            return self.pose, self.speed

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--hold", type=float, default=15.0, help="sim s of continuous standstill before driving")
    ap.add_argument("--end-hold", type=float, default=5.0)
    ap.add_argument("--still-speed", type=float, default=0.005, help="m/s")
    ap.add_argument("--max-drive", type=float, default=780.0, help="wall s of driving")
    ap.add_argument("--v", type=float, default=0.45)
    ap.add_argument("--lookahead", type=float, default=0.9)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    route = json.load(open(a.route))
    lap = np.array(route["path"], dtype=float)
    path = np.vstack([lap] + [lap[1:]] * (a.laps - 1))
    seg = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
    total = float(seg[-1])

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Driver()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    log = open(os.path.join(a.out, "drive.csv"), "w", newline="")
    wr = csv.writer(log)
    wr.writerow(["wall", "sim", "x", "y", "z", "roll", "pitch", "yaw", "speed", "v_cmd", "w_cmd", "s_path", "phase"])
    summary = {"route_length_m": round(total, 1), "laps": a.laps, "events": []}

    def ev(text):
        summary["events"].append([round(time.time(), 2), text])
        print(f"[bag_drive] {text}", flush=True)

    def tick(v, w, phase, s_path=float("nan")):
        node.send(v, w)
        st, spd = node.state()
        if st is not None:
            wr.writerow([f"{time.time():.3f}", f"{st[0]:.3f}", *(f"{q:.4f}" for q in st[1:4]),
                         *(f"{math.degrees(q):.2f}" for q in st[4:7]), f"{spd:.4f}", f"{v:.3f}", f"{w:.3f}",
                         f"{s_path:.2f}", phase])
        time.sleep(0.05)

    # ---------------------------------------------------------------- 1. standstill
    t0 = time.time()
    while node.state()[0] is None and time.time() - t0 < 30 and not stop.is_set():
        time.sleep(0.1)
    st, _ = node.state()
    if st is None:
        ev("no ground-truth pose on /model/togo/pose: abort")
        return 2
    still_since = None
    hold_start_wall = time.time()
    while not stop.is_set():
        st, spd = node.state()
        if not math.isfinite(spd) or spd > a.still_speed:
            still_since = None
        elif still_since is None:
            still_since = st[0]
        if still_since is not None and st[0] - still_since >= a.hold:
            break
        if time.time() - hold_start_wall > 120:
            ev(f"rover not still within 120 s (speed {spd:.4f} m/s): abort")
            return 3
        tick(0.0, 0.0, "hold")
    summary["start_hold"] = {"sim_s": round(a.hold, 1), "wall_s": round(time.time() - hold_start_wall, 1),
                             "start_pose": [round(q, 4) for q in node.state()[0][1:4]]}
    ev(f"standstill {a.hold:.0f} s sim confirmed (speed < {a.still_speed * 1000:.0f} mm/s); driving {a.laps} lap(s), {total:.0f} m")

    # ---------------------------------------------------------------- 2. pure pursuit
    s_idx = 0
    stuck_events = 0
    tries_here = 0
    stuck_s = -1e9
    last_prog = (time.time(), 0.0)
    drive_t0 = time.time()
    status = "ok"
    while not stop.is_set():
        st, spd = node.state()
        sim, x, y, z, roll, pitch, yaw = st
        # progress: closest path point within a forward window
        hi = min(len(path), s_idx + 60)
        d = np.hypot(path[s_idx:hi, 0] - x, path[s_idx:hi, 1] - y)
        s_idx += int(np.argmin(d))
        s_now = float(seg[s_idx])
        off = float(d.min())
        if s_idx >= len(path) - 2 and math.hypot(path[-1, 0] - x, path[-1, 1] - y) < 0.5:
            ev(f"route complete: {s_now:.1f} m in {time.time() - drive_t0:.0f} s wall")
            break
        if abs(math.degrees(roll)) > 25 or abs(math.degrees(pitch)) > 25:
            status = f"abort: tilt roll {math.degrees(roll):.0f} pitch {math.degrees(pitch):.0f} deg"
            ev(status)
            break
        if off > 3.0:
            status = f"abort: {off:.1f} m off the path"
            ev(status)
            break
        if time.time() - drive_t0 > a.max_drive:
            status = f"stopped at the time limit after {s_now:.1f} of {total:.0f} m"
            ev(status)
            break
        if s_now - last_prog[1] >= 0.2:
            last_prog = (time.time(), s_now)
            if s_now - stuck_s > 3.0:
                tries_here = 0
        elif time.time() - last_prog[0] > 20:
            stuck_events += 1
            tries_here += 1
            stuck_s = s_now
            ev(f"stuck at s={s_now:.1f} m ({x:.2f}, {y:.2f}); event {stuck_events}, try {tries_here}")
            if stuck_events >= 8:
                status = "abort: stuck 8 times"
                ev(status)
                break
            t1 = time.time()
            while time.time() - t1 < 4.0 and not stop.is_set():
                tick(-0.25, 0.0, "backup", s_now)
            if tries_here >= 3:
                target = s_now + 3.0
                s_idx = int(np.searchsorted(seg, target))
                tries_here = 0
                ev(f"skipping ahead to s={seg[min(s_idx, len(seg) - 1)]:.1f} m")
            last_prog = (time.time(), float(seg[min(s_idx, len(seg) - 1)]))
            continue
        # lookahead point
        la = int(np.searchsorted(seg, s_now + a.lookahead))
        tx, ty = path[min(la, len(path) - 1)]
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > math.radians(70):
            v, w = 0.0, math.copysign(0.6, err)
        else:
            v = a.v * max(0.3, 1.0 - abs(err) / 1.2)
            if abs(math.degrees(pitch)) > 6:
                v = min(v, 0.35)
            w = max(-0.8, min(0.8, 1.6 * err))
        tick(v, w, "drive", s_now)

    # ---------------------------------------------------------------- 3. final standstill
    t1 = time.time()
    while time.time() - t1 < a.end_hold / 0.9:
        tick(0.0, 0.0, "end_hold")
    st, _ = node.state()
    summary.update({"status": status, "driven_path_m": round(float(seg[min(s_idx, len(seg) - 1)]), 1),
                    "drive_wall_s": round(t1 - drive_t0, 1), "stuck_events": stuck_events,
                    "end_pose": [round(q, 4) for q in st[1:4]]})
    json.dump(summary, open(os.path.join(a.out, "drive_summary.json"), "w"), indent=1)
    log.close()
    ev(f"done: {summary['status']}")
    return 0 if status == "ok" else 1


if __name__ == "__main__":
    rc = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # Skip interpreter teardown: destroying the gz-transport node there aborts ("terminate called without an active
    # exception"), which made WSL write a 195 MB crash dump after the first recording.
    os._exit(rc)
