#!/usr/bin/env python3
"""Print one /dlio/frontend_diagnostics message with field labels."""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


FIELDS = [
    "status",
    "dlio_initialized",
    "first_imu_received",
    "imu_calibrated",
    "first_valid_scan",
    "sensor",
    "raw_points",
    "filtered_points",
    "imu_buffer_size",
    "scan_stamp",
    "latest_imu_stamp",
    "oldest_imu_stamp",
    "pos_x",
    "pos_y",
    "pos_z",
    "quat_w",
    "deskew_status",
    "deskew_size",
    "correction_translation_m",
    "correction_rotation_deg",
    "rejected",
    "bad_correction_streak",
    "fitness",
    "inliers",
    "overlap",
    "hessian_min_eigen",
    "hessian_max_eigen",
    "hessian_condition",
    "gicp_solve_ms",
    "quality_mode",
    "angular_rate_rad_s",
    "spin_protection_active",
    "imu_age_ms",
    "timing_protection_active",
    "lidar_header_stamp",
    "scan_start",
    "scan_end",
    "oldest_imu_snapshot",
    "latest_imu_snapshot",
    "imu_covers_scan_start",
    "imu_covers_scan_end",
    "latest_imu_minus_lidar_ms",
    "deskew_time_buckets",
    "stale_scan_drop_count",
    "scan_duration_ms",
    "drop_stale_enabled",
    "drop_imu_age_ms",
    "drop_reject_streak",
    "timing_max_iterations",
    "ground_constraint_engaged",
    "ground_constraint_reason",
    "ground_constraint_inliers",
    "ground_constraint_tilt_deg",
    "ground_constraint_z_correction",
    "ground_constraint_tracked_height",
    "pipeline_stage",
    "lidar_callbacks_started",
    "lidar_callbacks_completed",
]


class DiagOnce(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("dlio_diag_once")
        self.sub = self.create_subscription(Float32MultiArray, topic, self.on_msg, 10)

    def on_msg(self, msg: Float32MultiArray) -> None:
        print("=== Labeled DLIO diagnostics ===")
        for i, value in enumerate(msg.data):
            name = FIELDS[i] if i < len(FIELDS) else f"field_{i}"
            print(f"{i:02d} {name}: {value}")
        rclpy.shutdown()


def main() -> None:
    topic = sys.argv[1] if len(sys.argv) > 1 else "/dlio/frontend_diagnostics"
    rclpy.init()
    node = DiagOnce(topic)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
