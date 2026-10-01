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
        self.declare_parameter('frontend_odom_topic', '/dlio/odometry')
        self.declare_parameter('path_topic', '/reference/path')
        self.declare_parameter('model_names', 'a300-0000,a300_0000,a300,robot,base_link,chassis_link')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('align_to_frontend', False)
        self.declare_parameter('zero_start', True)
        self.declare_parameter('max_poses', 12000)
        self.declare_parameter('min_distance_m', 0.02)
        self.declare_parameter('extra_yaw_offset_deg', 0.0)
        self.declare_parameter('debug_candidates', True)
        self.declare_parameter('allow_fallback_entity', True)
        self.declare_parameter('fallback_transform_index', 0)
        self.declare_parameter('republish_rate_hz', 2.0)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        frontend_odom_topic = str(self.get_parameter('frontend_odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.model_names = [x.strip().lower() for x in str(self.get_parameter('model_names').value).split(',') if x.strip()]
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.align_to_frontend = bool(self.get_parameter('align_to_frontend').value)
        self.zero_start = bool(self.get_parameter('zero_start').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        self.extra_yaw_offset = math.radians(float(self.get_parameter('extra_yaw_offset_deg').value))
        self.debug_candidates = bool(self.get_parameter('debug_candidates').value)
        self.allow_fallback_entity = bool(self.get_parameter('allow_fallback_entity').value)
        self.fallback_transform_index = int(self.get_parameter('fallback_transform_index').value)
        republish_rate_hz = float(self.get_parameter('republish_rate_hz').value)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.frontend_sub = None
        if self.align_to_frontend:
            self.frontend_sub = self.create_subscription(Odometry, frontend_odom_topic, self.on_frontend, qos)
        self.pose_sub = self.create_subscription(TFMessage, self.pose_topic, self.on_pose_msg, 100)
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.status_timer = self.create_timer(2.0, self.log_status)
        self.republish_timer = self.create_timer(1.0 / max(republish_rate_hz, 0.1), self.republish_path)

        self.front0 = None
        self.gz0: TransformStamped | None = None
        self.model_key: str | None = None
        self.yaw_delta = 0.0
        self.poses: deque[PoseStamped] = deque(maxlen=max(1, self.max_poses))
        self.last_xy: tuple[float, float] | None = None
        self.seen_candidates: set[str] = set()
        self.last_path: Path | None = None
        self.pose_msgs_seen = 0
        self.frontend_msgs_seen = 0
        self.empty_child_warned = False
        self.fallback_warned = False
        self.pending_model_key: str | None = None
        self.get_logger().info(
            f'Publishing {self.path_topic} from Gazebo pose topic {self.pose_topic}; '
            f'frame={self.fixed_frame}; align_to_frontend={self.align_to_frontend}; '
            f'zero_start={self.zero_start}; candidates={self.model_names}'
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
            self.model_key = self.pending_model_key or self.transform_key(transform)
            self.get_logger().info(f'Using Gazebo pose entity: {self.model_key}')
            self.try_init_alignment()
        if self.gz0 is None:
            return
        if self.align_to_frontend and self.front0 is None:
            return
        self.publish_reference(transform)

    def transform_key(self, transform: TransformStamped, index: int | None = None) -> str:
        if transform.child_frame_id:
            return transform.child_frame_id
        if transform.header.frame_id:
            return transform.header.frame_id
        if index is not None:
            return f'unnamed_transform_{index}'
        return 'unnamed_transform'

    def score_name(self, name: str) -> int:
        normalized = name.replace('_', '-').lower()
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
        if self.model_key:
            for index, transform in enumerate(msg.transforms):
                if self.transform_key(transform, index) == self.model_key:
                    return transform

        best = None
        best_score = -999
        for index, transform in enumerate(msg.transforms):
            key = self.transform_key(transform, index)
            if not transform.child_frame_id:
                if not self.empty_child_warned:
                    self.get_logger().warn(
                        'Gazebo pose bridge produced empty child_frame_id entries; '
                        'trying header.frame_id/fallback_transform_index.'
                    )
                    self.empty_child_warned = True
            if not transform.child_frame_id and not transform.header.frame_id:
                continue
            if self.debug_candidates and key not in self.seen_candidates:
                self.seen_candidates.add(key)
                if len(self.seen_candidates) <= 30:
                    self.get_logger().info(f'Gazebo pose candidate: {key}')
            score = self.score_name(key)
            if score > best_score:
                best = transform
                best_score = score

        if best is not None and best_score > 0:
            self.pending_model_key = self.transform_key(best)
            return best
        if self.allow_fallback_entity and msg.transforms:
            fallback_index = min(max(0, self.fallback_transform_index), len(msg.transforms) - 1)
            if not self.fallback_warned:
                self.get_logger().warn(
                    f'No named Gazebo pose candidate matched; using transform index {fallback_index}. '
                    'Set fallback_transform_index if this is not the rover.'
                )
                self.fallback_warned = True
            self.pending_model_key = self.transform_key(msg.transforms[fallback_index], fallback_index)
            return msg.transforms[fallback_index]
        return None

    def try_init_alignment(self) -> None:
        if not self.align_to_frontend or self.front0 is None or self.gz0 is None:
            return
        gz_pose = self.gz0.transform
        self.yaw_delta = yaw_from_quat(self.front0.orientation) - yaw_from_quat(gz_pose.rotation) + self.extra_yaw_offset
        self.get_logger().info(f'Aligned Gazebo truth path with yaw offset {math.degrees(self.yaw_delta):.2f} deg')

    def publish_reference(self, transform: TransformStamped) -> None:
        gz_start = self.gz0.transform
        gz_now = transform.transform
        if self.zero_start or self.align_to_frontend:
            dx = gz_now.translation.x - gz_start.translation.x
            dy = gz_now.translation.y - gz_start.translation.y
            dz = gz_now.translation.z - gz_start.translation.z
            yaw = yaw_from_quat(gz_now.rotation) - yaw_from_quat(gz_start.rotation)
        else:
            dx = gz_now.translation.x
            dy = gz_now.translation.y
            dz = gz_now.translation.z
            yaw = yaw_from_quat(gz_now.rotation)

        if self.align_to_frontend:
            c = math.cos(self.yaw_delta)
            s = math.sin(self.yaw_delta)
            x = self.front0.position.x + c * dx - s * dy
            y = self.front0.position.y + s * dx + c * dy
            z = self.front0.position.z + dz
            yaw += self.yaw_delta
        else:
            x = dx
            y = dy
            z = dz
            yaw += self.extra_yaw_offset

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
        pose.pose.orientation = quat_from_yaw(yaw)
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
        if self.align_to_frontend and self.front0 is None:
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
