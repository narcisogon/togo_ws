#!/usr/bin/env python3
"""Convert an Odometry stream into a nav_msgs/Path for RViz comparison."""

from __future__ import annotations

from collections import deque

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

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.max_poses = max(1, self.get_parameter('max_poses').get_parameter_value().integer_value)
        self.publish_every_n = max(1, self.get_parameter('publish_every_n').get_parameter_value().integer_value)
        self.count = 0
        self.poses: deque[PoseStamped] = deque(maxlen=self.max_poses)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Path, path_topic, 10)
        self.subscription = self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        self.get_logger().info(f'Publishing Path {path_topic} from Odometry {odom_topic}')

    def on_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if self.fixed_frame:
            pose.header.frame_id = self.fixed_frame
        pose.pose = msg.pose.pose
        self.poses.append(pose)
        self.count += 1
        if self.count % self.publish_every_n != 0:
            return

        path = Path()
        path.header.stamp = pose.header.stamp
        path.header.frame_id = pose.header.frame_id
        path.poses = list(self.poses)
        self.publisher.publish(path)


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