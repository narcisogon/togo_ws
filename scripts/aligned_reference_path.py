#!/usr/bin/env python3
"""Publish a reference Path aligned to the frontend odometry initial pose."""

from __future__ import annotations

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class AlignedReferencePath(Node):
    def __init__(self) -> None:
        super().__init__('aligned_reference_path')
        self.declare_parameter('frontend_odom_topic', '/rko_lio/odometry')
        self.declare_parameter('reference_odom_topic', '/a300_0000/platform/odom')
        self.declare_parameter('path_topic', '/reference/path')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('max_poses', 12000)
        self.declare_parameter('min_distance_m', 0.02)
        self.declare_parameter('extra_yaw_offset_deg', 0.0)

        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.extra_yaw_offset = math.radians(float(self.get_parameter('extra_yaw_offset_deg').value))

        frontend_odom_topic = str(self.get_parameter('frontend_odom_topic').value)
        reference_odom_topic = str(self.get_parameter('reference_odom_topic').value)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.frontend_sub = self.create_subscription(Odometry, frontend_odom_topic, self.on_frontend, qos)
        self.reference_sub = self.create_subscription(Odometry, reference_odom_topic, self.on_reference, qos)
        self.publisher = self.create_publisher(Path, self.path_topic, 10)

        self.front0 = None
        self.ref0 = None
        self.alignment_ready = False
        self.yaw_delta = 0.0
        self.poses: deque[PoseStamped] = deque(maxlen=max(1, self.max_poses))
        self.last_xy: tuple[float, float] | None = None

        self.get_logger().info(
            f'Publishing aligned reference path {self.path_topic}: {reference_odom_topic} aligned to {frontend_odom_topic}'
        )

    def on_frontend(self, msg: Odometry) -> None:
        if self.front0 is None:
            self.front0 = msg.pose.pose
            self.try_init_alignment()

    def on_reference(self, msg: Odometry) -> None:
        if self.ref0 is None:
            self.ref0 = msg.pose.pose
            self.try_init_alignment()
        if not self.alignment_ready:
            return
        self.publish_aligned(msg)

    def try_init_alignment(self) -> None:
        if self.front0 is None or self.ref0 is None or self.alignment_ready:
            return
        self.yaw_delta = yaw_from_quat(self.front0.orientation) - yaw_from_quat(self.ref0.orientation) + self.extra_yaw_offset
        self.alignment_ready = True
        self.get_logger().info(f'Aligned reference path with yaw offset {math.degrees(self.yaw_delta):.2f} deg')

    def transform_reference_pose(self, ref_pose):
        # Reference displacement relative to its start.
        dx = ref_pose.position.x - self.ref0.position.x
        dy = ref_pose.position.y - self.ref0.position.y
        c = math.cos(self.yaw_delta)
        s = math.sin(self.yaw_delta)
        x = self.front0.position.x + c * dx - s * dy
        y = self.front0.position.y + s * dx + c * dy
        z = self.front0.position.z + (ref_pose.position.z - self.ref0.position.z)
        yaw = yaw_from_quat(ref_pose.orientation) + self.yaw_delta
        return x, y, z, quat_from_yaw(yaw)

    def publish_aligned(self, msg: Odometry) -> None:
        x, y, z, q = self.transform_reference_pose(msg.pose.pose)
        if self.last_xy is not None:
            dist = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if dist < self.min_distance_m:
                return
        self.last_xy = (x, y)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = q
        self.poses.append(pose)

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = self.fixed_frame
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main() -> None:
    rclpy.init()
    node = AlignedReferencePath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()