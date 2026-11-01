#!/usr/bin/env python3
"""Gate for the stereo rig in the simulation: listens to both images and the rig IMU for a few seconds and reports
the rates, whether the left/right stamps are identical (rendered in the same step), the image size and format,
the mean pixel value of each camera (0 = black, nothing rendered) and the IMU rate. With --spin RATE it then
turns the rover in place (TwistStamped on the platform controller's cmd_vel) and checks that the cameras turn
with it: the horizontal image shift between consecutive left frames (phase correlation of the horizon band)
against the yaw the rover's imu_0 integrated over the same frames, f * d_yaw, and the rig IMU's yaw rate against
imu_0's. The 2026-10-31 bag failed this silently: the rig's preserved fixed joints were free about z in the
physics, the cameras kept their world heading while the rover turned, and the VIO diverged.

usage: check_stereo_sync.py [--seconds 8] [--spin 0.3] [--left T] [--right T] [--imu T]
exit 0 when the pair is in lockstep at the expected rate, the images are not black and (with --spin) the cameras
follow the rover's yaw, 1 otherwise.
"""
import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, Imu

FOCAL_PX = 320.0          # the rig's pinhole focal length (640 px wide, 90 deg horizontal field of view)
BAND = (120, 330)         # image rows used for the shift: horizon and far ground, where the rotation's flow is uniform


class Check(Node):
    def __init__(self, left, right, imu, imu0, cmd):
        # sim-time stamps on the velocity commands (the controller checks their age against the simulation clock)
        super().__init__("stereo_rig_check", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.left, self.right, self.imu = {}, {}, []
        self.meta = {}
        self.first = {}     # side -> (width, height, bytes) of the first frame, saved as a PGM for a look
        self.frames = []    # (stamp s, HxW uint8) left frames while keep_frames is set
        self.keep_frames = False
        self.gyro0, self.gyro_rig = [], []       # (stamp s, yaw rate) of imu_0 and of the rig's IMU
        self.create_subscription(Image, left, lambda m: self.on_image("left", m), qos)
        self.create_subscription(Image, right, lambda m: self.on_image("right", m), qos)
        self.create_subscription(Imu, imu, lambda m: self.on_imu(self.gyro_rig, m, True), qos)
        self.create_subscription(Imu, imu0, lambda m: self.on_imu(self.gyro0, m, False), qos)
        self.pub = self.create_publisher(TwistStamped, cmd, 10)

    def on_image(self, side, m):
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        store = self.left if side == "left" else self.right
        if len(store) < 400:
            data = np.frombuffer(m.data, dtype=np.uint8)
            store[stamp] = float(data.mean()) if data.size else 0.0
            if side not in self.first and m.encoding == "mono8":
                self.first[side] = (m.width, m.height, bytes(m.data))
        if side == "left" and self.keep_frames and m.encoding == "mono8" and len(self.frames) < 240:
            self.frames.append((stamp * 1e-9, np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width].copy()))
        self.meta[side] = (m.width, m.height, m.encoding, m.header.frame_id)

    def on_imu(self, store, m, is_rig):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if is_rig:
            self.imu.append(st)
        store.append((st, m.angular_velocity.z))

    def send(self, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.angular.z = float(w)
        self.pub.publish(m)


def integrate(gyro, ta, tb):
    """yaw turned between ta and tb (trapezoid over the samples inside, rate held across the ends)."""
    g = np.array(gyro)
    if len(g) < 2:
        return float("nan")
    t, w = g[:, 0], g[:, 1]
    if ta < t[0] or tb > t[-1]:
        return float("nan")
    ts = np.concatenate(([ta], t[(t > ta) & (t < tb)], [tb]))
    ws = np.interp(ts, t, w)
    return float(np.sum(0.5 * (ws[1:] + ws[:-1]) * np.diff(ts)))


def spin_check(node, rate):
    """(ok, text): the cameras' yaw against imu_0's, from the frames kept during the spin."""
    try:
        import cv2
    except ImportError:
        return False, "spin check needs python3-opencv (cv2)"
    frames = node.frames
    if len(frames) < 10:
        return False, f"spin check: only {len(frames)} left frames kept during the spin"
    win = cv2.createHanningWindow((frames[0][1].shape[1], BAND[1] - BAND[0]), cv2.CV_32F)
    ratios, resps, dys, rig_ratios = [], [], [], []
    for (ta, A), (tb, B) in zip(frames, frames[1:]):
        if not 0 < tb - ta < 0.1:
            continue
        dyaw = integrate(node.gyro0, ta, tb)
        if not np.isfinite(dyaw) or abs(dyaw) < 0.3 * rate * (tb - ta):
            continue                                   # not turning yet (or no longer)
        (dx, dy), resp = cv2.phaseCorrelate(np.float32(A[BAND[0]:BAND[1]]), np.float32(B[BAND[0]:BAND[1]]), win)
        ratios.append(dx / (FOCAL_PX * dyaw))
        resps.append(resp)
        dys.append(dy)
        dyaw_rig = integrate(node.gyro_rig, ta, tb)
        if np.isfinite(dyaw_rig):
            rig_ratios.append(dyaw_rig / dyaw)
    if len(ratios) < 8:
        return False, f"spin check: only {len(ratios)} frame pairs while turning (imu_0 saw no yaw? commands not reaching the controller?)"
    r, resp, dy = np.median(ratios), np.median(resps), np.median(np.abs(dys))
    rig = np.median(rig_ratios) if rig_ratios else float("nan")
    text = (f"spin check: {len(ratios)} frame pairs while turning at {rate} rad/s; image shift / (f * imu_0 yaw) median {r:+.2f} "
            f"(1 = the cameras turn with the rover, 0 = they do not; correlation response {resp:.2f}, vertical shift {dy:.1f} px); "
            f"rig IMU yaw / imu_0 yaw median {rig:.2f}")
    # 1.0 for a level camera; a camera pitched down sees nearer ground in the band and the rig's lever arm adds
    # translation flow in a turn on the spot (1.2-1.4 at 20 deg down), while the failure this guards against gives
    # about 0 (the 2026-10-31 rig: 0.15)
    ok = 0.6 < abs(r) < 1.6 and resp > 0.03 and dy < 6.0 and (not rig_ratios or 0.8 < rig < 1.2)
    return ok, text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--left", default="/husky/sensors/stereo_rig/left/image_raw")
    ap.add_argument("--right", default="/husky/sensors/stereo_rig/right/image_raw")
    ap.add_argument("--imu", default="/husky/sensors/stereo_rig/imu/data_raw")
    ap.add_argument("--imu0", default="/husky/sensors/imu_0/data_raw")
    ap.add_argument("--cmd", default="/platform_velocity_controller/cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0, help="expected image rate (Hz, simulation time)")
    ap.add_argument("--spin", type=float, default=0.0, help="rad/s: turn the rover in place after the rate check and verify the cameras turn with it (0 = skip)")
    ap.add_argument("--spin-seconds", type=float, default=5.0, help="wall seconds of turning")
    ap.add_argument("--save-dir", default="", help="write the first left/right frames there as PGM files")
    a = ap.parse_args()
    rclpy.init()
    node = Check(a.left, a.right, a.imu, a.imu0, a.cmd)
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    span = time.monotonic() - t0
    nl, nr, ni = len(node.left), len(node.right), len(node.imu)
    common = set(node.left) & set(node.right)

    def stamp_rate(store):
        if len(store) < 3:
            return float("nan")
        return 1e9 / float(np.median(np.diff(sorted(store))))
    rl, rr = stamp_rate(node.left), stamp_rate(node.right)
    print(f"left  {nl} frames ({nl / span:.1f} Hz wall, {rl:.1f} Hz by stamps)  {node.meta.get('left')}")
    print(f"right {nr} frames ({nr / span:.1f} Hz wall, {rr:.1f} Hz by stamps)  {node.meta.get('right')}")
    if a.save_dir:
        for side, (w, h, raw) in node.first.items():
            with open(f"{a.save_dir}/first_{side}.pgm", "wb") as f:
                f.write(("P5" + chr(10) + f"{w} {h}" + chr(10) + "255" + chr(10)).encode() + raw)
        print(f"first frames saved as PGM in {a.save_dir}")
    print(f"pairs with identical stamps: {len(common)} of {min(nl, nr)}")
    if node.left:
        print(f"mean pixel value left {np.mean(list(node.left.values())):.1f}, right {np.mean(list(node.right.values())):.1f}")
    if len(node.imu) > 2:
        dts = np.diff(sorted(node.imu))
        print(f"imu   {ni} samples, {1.0 / np.median(dts):.0f} Hz by stamps ({ni / span:.0f} Hz wall)")
    else:
        print("imu   no samples")
    ok = (nl > 20 and nr > 20 and rl > 0.75 * a.rate and rr > 0.75 * a.rate and len(common) >= 0.9 * min(nl, nr)
          and node.left and np.mean(list(node.left.values())) > 2.0 and ni > 10)
    spin_ok = True
    if a.spin > 0:
        node.keep_frames = True
        t1 = time.monotonic()
        while time.monotonic() - t1 < a.spin_seconds:          # turn in place, 20 Hz commands
            node.send(a.spin)
            rclpy.spin_once(node, timeout_sec=0.05)
        t2 = time.monotonic()
        while time.monotonic() - t2 < 1.0:                      # stop, let the last frames in
            node.send(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.keep_frames = False
        spin_ok, text = spin_check(node, a.spin)
        print(text)
        print("SPIN", "ok" if spin_ok else "FAIL: the cameras do not follow the rover's yaw (rig joints not rigid?)")
    print("RESULT", "ok" if ok and spin_ok else "FAIL")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok and spin_ok else 1)


if __name__ == "__main__":
    main()
