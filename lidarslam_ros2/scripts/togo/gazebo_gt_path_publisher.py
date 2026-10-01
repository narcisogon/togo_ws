#!/usr/bin/env python3
"""
Gazebo ground truth path publisher.
Aligns Gazebo world pose to SLAM odom frame with XYZ + rotation offset.
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path


def quat_multiply(q1: tuple, q2: tuple) -> tuple:
    """Multiply two quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def quat_inverse(q: tuple) -> tuple:
    """Inverse of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)

def yaw_from_quat(q) -> float:
    """Extract yaw from quaternion."""
    x = q.x; y = q.y; z = q.z; w = q.w
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

def yaw_to_quat(yaw: float) -> tuple:
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def rotate_point_by_yaw(x: float, y: float, yaw: float) -> tuple:
    """Rotate a 2D point by yaw angle."""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return (
        cos_y * x - sin_y * y,
        sin_y * x + cos_y * y,
    )


class GazeboGroundTruthPath(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_gt_path_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('fixed_frame',  'odom')
        self.declare_parameter('model_topic',  '/model/togo/pose')
        self.declare_parameter('slam_topic',   '/dlio/path_simple')
        self.declare_parameter('output_topic', '/ground_truth/path')
        self.declare_parameter('pose_index',   5)
        self.declare_parameter('buffer_size',  500)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        model_topic      = self.get_parameter('model_topic').value
        slam_topic       = self.get_parameter('slam_topic').value
        output_topic     = self.get_parameter('output_topic').value
        self.pose_index  = self.get_parameter('pose_index').value
        self.buffer_size = self.get_parameter('buffer_size').value

        # ── State ──────────────────────────────────────────────────────────────
        self.offset: tuple[float, float, float] | None = None
        self.yaw_offset: float | None = None             # ✅ rotation offset

        self.slam_initial:     tuple | None = None
        self.slam_initial_yaw: float | None = None
        self.slam_first_stamp: float | None = None

        # Buffer of (timestamp, (x, y, z), quaternion)
        self.gt_pose_buffer: list = []

        # ── Path ───────────────────────────────────────────────────────────────
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(PoseArray, model_topic,  self.on_gz_pose,   10)
        self.create_subscription(Path,      slam_topic,   self.on_slam_path, 10)

        # ── Publisher ──────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Path, output_topic, 10)

        self.get_logger().info(
            f'gazebo_gt_path_publisher started\n'
            f'  model topic : {model_topic}\n'
            f'  slam topic  : {slam_topic}\n'
            f'  output topic: {output_topic}\n'
            f'  fixed frame : {self.fixed_frame}\n'
            f'  pose index  : {self.pose_index}'
        )

    # ── SLAM path callback ─────────────────────────────────────────────────────
    def on_slam_path(self, msg: Path) -> None:
        if self.slam_initial is not None:
            return
        if not msg.poses:
            return

        first = msg.poses[0]
        p = first.pose.position
        o = first.pose.orientation

        self.slam_initial = (p.x, p.y, p.z)
        self.slam_initial_yaw = yaw_from_quat(o)
        self.slam_first_stamp = (
            first.header.stamp.sec +
            first.header.stamp.nanosec * 1e-9
        )

        self.get_logger().info(
            f'SLAM first pose  t={self.slam_first_stamp:.2f} '
            f'x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} '
            f'yaw={math.degrees(self.slam_initial_yaw):.2f}°'
        )
        self.try_compute_offset()

    # ── Gazebo pose callback ───────────────────────────────────────────────────
    def on_gz_pose(self, msg: PoseArray) -> None:
        if len(msg.poses) <= self.pose_index:
            self.get_logger().warn(
                f'Expected at least {self.pose_index + 1} poses, '
                f'got {len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return

        world_pose = msg.poses[self.pose_index]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Buffer GT pose + orientation
        self.gt_pose_buffer.append((
            stamp,
            (world_pose.position.x,
             world_pose.position.y,
             world_pose.position.z),
            world_pose.orientation
        ))
        if len(self.gt_pose_buffer) > self.buffer_size:
            self.gt_pose_buffer.pop(0)

        if self.offset is None and self.slam_first_stamp is not None:
            self.try_compute_offset()

        if self.offset is None:
            self.get_logger().info(
                'Buffering GT poses, waiting for SLAM first pose...',
                throttle_duration_sec=2.0,
            )
            return

        # ── Apply position offset + rotation ──────────────────────────────────
        # Translate relative to GT origin
        rel_x = world_pose.position.x - self.offset[0]
        rel_y = world_pose.position.y - self.offset[1]
        rel_z = world_pose.position.z - self.offset[2]

        # ✅ Rotate position by yaw offset
        rot_x, rot_y = rotate_point_by_yaw(rel_x, rel_y, -self.yaw_offset)

        # ✅ Rotate orientation by yaw offset
        gz_quat = (
            world_pose.orientation.x,
            world_pose.orientation.y,
            world_pose.orientation.z,
            world_pose.orientation.w,
        )
        yaw_correction = yaw_to_quat(-self.yaw_offset)
        corrected_quat = quat_multiply(yaw_correction, gz_quat)

        # ── Build pose ────────────────────────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp    = msg.header.stamp
        ps.header.frame_id = self.fixed_frame
        ps.pose.position.x = rot_x
        ps.pose.position.y = rot_y
        ps.pose.position.z = rel_z
        ps.pose.orientation.x = corrected_quat[0]
        ps.pose.orientation.y = corrected_quat[1]
        ps.pose.orientation.z = corrected_quat[2]
        ps.pose.orientation.w = corrected_quat[3]

        self.path.poses.append(ps)
        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)

    # ── Offset computation ─────────────────────────────────────────────────────
    def try_compute_offset(self) -> None:
        if self.slam_first_stamp is None or not self.gt_pose_buffer:
            return
        if self.offset is not None:
            return

        # Find GT pose closest in time to SLAM first pose
        closest_time, closest_pos, closest_ori = min(
            self.gt_pose_buffer,
            key=lambda e: abs(e[0] - self.slam_first_stamp)
        )

        time_diff = abs(closest_time - self.slam_first_stamp)
        gt_yaw = yaw_from_quat(closest_ori)

        # ✅ Position offset = GT position at t0 (SLAM starts at origin)
        self.offset = (
            closest_pos[0],
            closest_pos[1],
            closest_pos[2],
        )

        # ✅ Yaw offset = difference between GT yaw and SLAM yaw at t0
        self.yaw_offset = gt_yaw - self.slam_initial_yaw

        self.get_logger().info(
            f'GT pose matched  t={closest_time:.2f} '
            f'(diff={time_diff:.3f}s) '
            f'x={closest_pos[0]:.4f} y={closest_pos[1]:.4f} z={closest_pos[2]:.4f} '
            f'yaw={math.degrees(gt_yaw):.2f}°'
        )
        self.get_logger().info(
            f'Offset applied   '
            f'dx={self.offset[0]:.4f} '
            f'dy={self.offset[1]:.4f} '
            f'dz={self.offset[2]:.4f} '
            f'dyaw={math.degrees(self.yaw_offset):.2f}°'
        )


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
