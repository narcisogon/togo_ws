#!/usr/bin/env python3
"""Labeled live monitor for RKO-LIO diagnostic arrays."""

from __future__ import annotations

import argparse
import math
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


REG_FIELDS = (
    "valid",
    "keypoints",
    "correspondences",
    "inlier_ratio",
    "mean_error",
    "hessian_min",
    "hessian_max",
    "hessian_condition",
    "registration_failures",
    "coarse_to_fine",
    "damping_alpha",
    "eigen_projection_min_scale",
    "adaptive_prior_translation_weight",
    "adaptive_prior_rotation_weight",
    "rover_motion_constraint",
)

RUNTIME_FIELDS = (
    "process_sec",
    "scan_age_before_sec",
    "scan_age_after_sec",
    "queued_lidar",
    "raw_points",
    "deskewed_points",
    "dropped_lidar",
    "success",
)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_bool(value: float) -> str:
    return "yes" if value >= 0.5 else "no"


def as_dict(msg: Float32MultiArray | None, fields: tuple[str, ...]) -> dict[str, float]:
    if msg is None:
        return {}
    return {name: float(msg.data[i]) for i, name in enumerate(fields) if i < len(msg.data)}


@dataclass
class PoseSample:
    wall_time: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class MonitorState:
    registration_msg: Float32MultiArray | None = None
    runtime_msg: Float32MultiArray | None = None
    registration_wall_time: float = 0.0
    runtime_wall_time: float = 0.0
    odom_samples: deque[PoseSample] = field(default_factory=lambda: deque(maxlen=200))
    last_dropped: float | None = None


class RkoDiagnosticsMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rko_diagnostics_monitor")
        self.args = args
        self.state = MonitorState()
        self.start_time = time.monotonic()

        self.create_subscription(
            Float32MultiArray,
            args.registration_topic,
            self.on_registration,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            args.runtime_topic,
            self.on_runtime,
            10,
        )
        self.create_subscription(
            Odometry,
            args.odom_topic,
            self.on_odom,
            20,
        )
        self.create_timer(args.print_period, self.print_status)

        self.get_logger().info(
            f"Monitoring RKO diagnostics: registration={args.registration_topic}, "
            f"runtime={args.runtime_topic}, odom={args.odom_topic}"
        )

    def on_registration(self, msg: Float32MultiArray) -> None:
        self.state.registration_msg = msg
        self.state.registration_wall_time = time.monotonic()

    def on_runtime(self, msg: Float32MultiArray) -> None:
        self.state.runtime_msg = msg
        self.state.runtime_wall_time = time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.state.odom_samples.append(
            PoseSample(time.monotonic(), p.x, p.y, p.z, yaw_from_quat(q.x, q.y, q.z, q.w))
        )

    def odom_rate(self) -> float:
        samples = self.state.odom_samples
        if len(samples) < 2:
            return 0.0
        dt = samples[-1].wall_time - samples[0].wall_time
        return 0.0 if dt <= 0.0 else (len(samples) - 1) / dt

    def odom_delta(self) -> tuple[float, float]:
        samples = self.state.odom_samples
        if len(samples) < 2:
            return 0.0, 0.0
        newest = samples[-1]
        horizon = self.args.drift_window_sec
        oldest = samples[0]
        for sample in reversed(samples):
            if newest.wall_time - sample.wall_time >= horizon:
                oldest = sample
                break
        trans = math.hypot(newest.x - oldest.x, newest.y - oldest.y)
        yaw_delta = math.atan2(
            math.sin(newest.yaw - oldest.yaw),
            math.cos(newest.yaw - oldest.yaw),
        )
        return trans, math.degrees(abs(yaw_delta))

    def health_flags(self, reg: dict[str, float], runtime: dict[str, float]) -> list[str]:
        flags: list[str] = []

        process_sec = runtime.get("process_sec")
        if process_sec is not None and process_sec > self.args.process_warn_sec:
            flags.append(f"SLOW process={process_sec:.3f}s")

        queued = runtime.get("queued_lidar")
        if queued is not None and queued >= 1.0:
            flags.append(f"BACKLOG queued={queued:.0f}")

        dropped = runtime.get("dropped_lidar")
        if dropped is not None:
            if self.state.last_dropped is not None and dropped > self.state.last_dropped:
                flags.append(f"DROPPING +{dropped - self.state.last_dropped:.0f}")
            self.state.last_dropped = dropped

        if runtime.get("success", 1.0) < 0.5:
            flags.append("RUNTIME_FAIL")

        if reg.get("valid", 1.0) < 0.5:
            flags.append("REG_INVALID")

        inlier_ratio = reg.get("inlier_ratio")
        if inlier_ratio is not None and inlier_ratio < self.args.inlier_warn:
            flags.append(f"LOW_INLIER {inlier_ratio:.2f}")

        mean_error = reg.get("mean_error")
        if mean_error is not None and mean_error > self.args.mean_error_warn:
            flags.append(f"HIGH_ERROR {mean_error:.3f}m")

        condition = reg.get("hessian_condition")
        if condition is not None and condition > self.args.condition_warn:
            flags.append(f"DEGENERATE cond={condition:.0f}")

        failures = reg.get("registration_failures")
        if failures is not None and failures > 0:
            flags.append(f"FAILURES {failures:.0f}")

        drift_m, drift_yaw_deg = self.odom_delta()
        if drift_m > self.args.stationary_drift_warn_m:
            flags.append(f"POSE_DRIFT {drift_m:.3f}m/{self.args.drift_window_sec:.0f}s")
        if drift_yaw_deg > self.args.stationary_yaw_warn_deg:
            flags.append(f"YAW_DRIFT {drift_yaw_deg:.2f}deg/{self.args.drift_window_sec:.0f}s")

        return flags

    def print_status(self) -> None:
        now = time.monotonic()
        if self.args.duration > 0.0 and now - self.start_time >= self.args.duration:
            rclpy.shutdown()
            return

        reg = as_dict(self.state.registration_msg, REG_FIELDS)
        runtime = as_dict(self.state.runtime_msg, RUNTIME_FIELDS)
        flags = self.health_flags(reg, runtime)

        pose = self.state.odom_samples[-1] if self.state.odom_samples else None
        drift_m, drift_yaw_deg = self.odom_delta()
        reg_age = now - self.state.registration_wall_time if self.state.registration_msg else math.inf
        runtime_age = now - self.state.runtime_wall_time if self.state.runtime_msg else math.inf

        print("\n=== RKO-LIO diagnostics ===", flush=True)
        print(
            "runtime: "
            f"process={runtime.get('process_sec', math.nan):.3f}s "
            f"age_before={runtime.get('scan_age_before_sec', math.nan):.3f}s "
            f"age_after={runtime.get('scan_age_after_sec', math.nan):.3f}s "
            f"queue={runtime.get('queued_lidar', math.nan):.0f} "
            f"raw={runtime.get('raw_points', math.nan):.0f} "
            f"deskewed={runtime.get('deskewed_points', math.nan):.0f} "
            f"dropped={runtime.get('dropped_lidar', math.nan):.0f} "
            f"success={fmt_bool(runtime.get('success', 0.0))} "
            f"msg_age={runtime_age:.1f}s",
            flush=True,
        )
        print(
            "registration: "
            f"valid={fmt_bool(reg.get('valid', 0.0))} "
            f"keypoints={reg.get('keypoints', math.nan):.0f} "
            f"corr={reg.get('correspondences', math.nan):.0f} "
            f"inlier={reg.get('inlier_ratio', math.nan):.2f} "
            f"mean_err={reg.get('mean_error', math.nan):.3f}m "
            f"hess_min={reg.get('hessian_min', math.nan):.3g} "
            f"hess_max={reg.get('hessian_max', math.nan):.3g} "
            f"cond={reg.get('hessian_condition', math.nan):.0f} "
            f"fail={reg.get('registration_failures', math.nan):.0f} "
            f"coarse={fmt_bool(reg.get('coarse_to_fine', 0.0))} "
            f"damp={reg.get('damping_alpha', math.nan):.2f} "
            f"eig={reg.get('eigen_projection_min_scale', math.nan):.2f} "
            f"prior_t={reg.get('adaptive_prior_translation_weight', math.nan):.3f} "
            f"prior_r={reg.get('adaptive_prior_rotation_weight', math.nan):.2f} "
            f"rover={fmt_bool(reg.get('rover_motion_constraint', 0.0))} "
            f"msg_age={reg_age:.1f}s",
            flush=True,
        )
        if pose is None:
            print("odometry: waiting", flush=True)
        else:
            print(
                "odometry: "
                f"hz={self.odom_rate():.1f} "
                f"pose=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) "
                f"yaw={math.degrees(pose.yaw):.2f}deg "
                f"delta={drift_m:.3f}m/{self.args.drift_window_sec:.0f}s "
                f"yaw_delta={drift_yaw_deg:.2f}deg/{self.args.drift_window_sec:.0f}s",
                flush=True,
            )
        print("flags: " + (", ".join(flags) if flags else "OK"), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until Ctrl-C.")
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--registration-topic", default="/rko_lio/registration_diagnostics")
    parser.add_argument("--runtime-topic", default="/rko_lio/runtime_diagnostics")
    parser.add_argument("--odom-topic", default="/rko_lio/odometry")
    parser.add_argument("--process-warn-sec", type=float, default=0.09)
    parser.add_argument("--condition-warn", type=float, default=1500.0)
    parser.add_argument("--inlier-warn", type=float, default=0.70)
    parser.add_argument("--mean-error-warn", type=float, default=0.08)
    parser.add_argument("--drift-window-sec", type=float, default=5.0)
    parser.add_argument("--stationary-drift-warn-m", type=float, default=0.08)
    parser.add_argument("--stationary-yaw-warn-deg", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RkoDiagnosticsMonitor(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
