#!/usr/bin/env python3
"""Publish an aligned ground-truth path from Gazebo Pose_V bridged as TFMessage."""

from __future__ import annotations

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


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


class GazeboPoseToAlignedPath(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_pose_to_aligned_path')
        self.declare_parameter('pose_topic', '/gazebo/dynamic_pose')
        self.declare_parameter('frontend_odom_topic', '/rko_lio/odometry')
        self.declare_parameter('path_topic', '/reference/path')
        self.declare_parameter('model_names', 'a300-0000,a300_0000,a300,robot,base_link,chassis_link')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('max_poses', 12000)
        self.declare_parameter('min_distance_m', 0.02)
        self.declare_parameter('extra_yaw_offset_deg', 0.0)
        self.declare_parameter('debug_candidates', True)
        self.declare_parameter('allow_fallback_entity', True)
        self.declare_parameter('republish_rate_hz', 2.0)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        frontend_odom_topic = str(self.get_parameter('frontend_odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.model_names = [x.strip().lower() for x in str(self.get_parameter('model_names').value).split(',') if x.strip()]
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.extra_yaw_offset = math.radians(float(self.get_parameter('extra_yaw_offset_deg').value))
        self.debug_candidates = bool(self.get_parameter('debug_candidates').value)
        self.allow_fallback_entity = bool(self.get_parameter('allow_fallback_entity').value)
        republish_rate_hz = float(self.get_parameter('republish_rate_hz').value)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.frontend_sub = self.create_subscription(Odometry, frontend_odom_topic, self.on_frontend, qos)
        self.pose_sub = self.create_subscription(TFMessage, self.pose_topic, self.on_pose_msg, 100)
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.status_timer = self.create_timer(2.0, self.log_status)
        self.republish_timer = self.create_timer(1.0 / max(republish_rate_hz, 0.1), self.republish_path)

        self.front0 = None
        self.gz0: TransformStamped | None = None
        self.model_name: str | None = None
        self.yaw_delta = 0.0
        self.poses: deque[PoseStamped] = deque(maxlen=max(1, self.max_poses))
        self.last_xy: tuple[float, float] | None = None
        self.seen_candidates: set[str] = set()
        self.last_path: Path | None = None
        self.pose_msgs_seen = 0
        self.frontend_msgs_seen = 0
        self.empty_child_warned = False
        self.get_logger().info(
            f'Publishing {self.path_topic} from Gazebo pose topic {self.pose_topic}; candidates={self.model_names}'
        )

    def on_frontend(self, msg: Odometry) -> None:
        self.frontend_msgs_seen += 1
        if self.front0 is None:
            self.front0 = msg.pose.pose
            self.try_init_alignment()

    def on_pose_msg(self, msg: TFMessage) -> None:
        self.pose_msgs_seen += 1
        transform = self.pick_transform(msg)
        if transform is None:
            return
        if self.gz0 is None:
            self.gz0 = transform
            self.model_name = transform.child_frame_id
            self.get_logger().info(f'Using Gazebo pose entity: {self.model_name}')
            self.try_init_alignment()
        if self.front0 is None or self.gz0 is None:
            return
        self.publish_aligned(transform)

    def score_child(self, child: str) -> int:
        normalized = child.replace('_', '-').lower()
        score = 0
        if any(name in normalized for name in self.model_names):
            score += 50
        if 'base-link' in normalized or 'base' in normalized:
            score += 35
        if 'chassis' in normalized:
            score += 30
        if 'robot' in normalized:
            score += 20
        if 'a300' in normalized:
            score += 20
        if any(bad in normalized for bad in ('wheel', 'sensor', 'lidar', 'imu', 'terrain', 'sun')):
            score -= 40
        return score

    def pick_transform(self, msg: TFMessage):
        if self.model_name:
            for transform in msg.transforms:
                if transform.child_frame_id == self.model_name:
                    return transform

        best = None
        best_score = -999
        for transform in msg.transforms:
            child = transform.child_frame_id
            if not child:
                if not self.empty_child_warned:
                    self.get_logger().warn('Gazebo pose bridge produced empty child_frame_id entries; cannot identify robot from TFMessage names.')
                    self.empty_child_warned = True
                continue
            if self.debug_candidates and child not in self.seen_candidates:
                self.seen_candidates.add(child)
                if len(self.seen_candidates) <= 30:
                    self.get_logger().info(f'Gazebo pose candidate: {child}')
            score = self.score_child(child)
            if score > best_score:
                best = transform
                best_score = score

        if best is not None and best_score > 0:
            return best
        return None

    def try_init_alignment(self) -> None:
        if self.front0 is None or self.gz0 is None:
            return
        gz_pose = self.gz0.transform
        self.yaw_delta = yaw_from_quat(self.front0.orientation) - yaw_from_quat(gz_pose.rotation) + self.extra_yaw_offset
        self.get_logger().info(f'Aligned Gazebo truth path with yaw offset {math.degrees(self.yaw_delta):.2f} deg')

    def publish_aligned(self, transform: TransformStamped) -> None:
        gz_start = self.gz0.transform
        gz_now = transform.transform
        dx = gz_now.translation.x - gz_start.translation.x
        dy = gz_now.translation.y - gz_start.translation.y
        dz = gz_now.translation.z - gz_start.translation.z
        c = math.cos(self.yaw_delta)
        s = math.sin(self.yaw_delta)
        x = self.front0.position.x + c * dx - s * dy
        y = self.front0.position.y + s * dx + c * dy
        z = self.front0.position.z + dz
        if self.last_xy is not None and math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < self.min_distance_m:
            self.republish_path()
            return
        self.last_xy = (x, y)

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = quat_from_yaw(yaw_from_quat(gz_now.rotation) + self.yaw_delta)
        self.poses.append(pose)
        self.publish_path(transform.header.stamp)

    def publish_path(self, stamp) -> None:
        if not self.poses:
            return
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.fixed_frame
        path.poses = list(self.poses)
        self.last_path = path
        self.publisher.publish(path)

    def republish_path(self) -> None:
        if self.last_path is not None:
            self.publisher.publish(self.last_path)

    def log_status(self) -> None:
        if self.front0 is None:
            self.get_logger().warn(f'Waiting for frontend odometry; msgs_seen={self.frontend_msgs_seen}')
        if self.gz0 is None:
            self.get_logger().warn(
                f'Waiting for Gazebo robot pose; pose_msgs_seen={self.pose_msgs_seen}; '
                f'candidates_seen={list(self.seen_candidates)[:10]}'
            )


def main() -> None:
    rclpy.init()
    node = GazeboPoseToAlignedPath()
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