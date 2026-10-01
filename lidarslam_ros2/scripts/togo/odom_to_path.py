#!/usr/bin/env python3
"""Convert an Odometry stream into a nav_msgs/Path for RViz comparison."""

from __future__ import annotations

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException


class OdomToPath(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_path')
        self.declare_parameter('odom_topic', '/rko_lio/odometry')
        self.declare_parameter('path_topic', '/rko_lio/path')
        self.declare_parameter('fixed_frame', '')
        self.declare_parameter('max_poses', 5000)
        self.declare_parameter('publish_every_n', 1)
        self.declare_parameter('min_distance_m', 0.1)
        self.declare_parameter('publish_period_sec', 0.5)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.max_poses = max(1, self.get_parameter('max_poses').get_parameter_value().integer_value)
        self.publish_every_n = max(1, self.get_parameter('publish_every_n').get_parameter_value().integer_value)
        self.min_distance_m = max(0.0, self.get_parameter('min_distance_m').get_parameter_value().double_value)
        self.publish_period_sec = max(
            0.01,
            self.get_parameter('publish_period_sec').get_parameter_value().double_value,
        )
        self.count = 0
        self.last_xy: tuple[float, float] | None = None
        self.latest_pose: PoseStamped | None = None
        self.dirty = False
        self.poses: deque[PoseStamped] = deque(maxlen=self.max_poses)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Path, path_topic, path_qos)
        self.subscription = self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        self.publish_timer = self.create_timer(self.publish_period_sec, self.publish_path)
        self.get_logger().info(
            f'Publishing Path {path_topic} from Odometry {odom_topic} at no more than '
            f'{1.0 / self.publish_period_sec:.1f} Hz (max_poses={self.max_poses}, '
            f'min_distance_m={self.min_distance_m:.3f})'
        )

    def on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None and math.hypot(x - self.last_xy[0], y - self.last_xy[1]) < self.min_distance_m:
            return
        self.last_xy = (x, y)
        self.count += 1
        if self.count % self.publish_every_n != 0:
            return

        pose = PoseStamped()
        pose.header = msg.header
        if self.fixed_frame:
            pose.header.frame_id = self.fixed_frame
        pose.pose = msg.pose.pose
        self.poses.append(pose)
        self.latest_pose = pose
        self.dirty = True

    def publish_path(self) -> None:
        if not self.dirty or self.latest_pose is None:
            return

        path = Path()
        path.header.stamp = self.latest_pose.header.stamp
        path.header.frame_id = self.latest_pose.header.frame_id
        path.poses = list(self.poses)
        self.publisher.publish(path)
        self.dirty = False


def main() -> None:
    rclpy.init()
    node = OdomToPath()
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
