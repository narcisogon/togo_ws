#!/usr/bin/env python3
"""Convert a PoseStamped stream into Odometry for comparison helpers."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PoseToOdom(Node):
    def __init__(self) -> None:
        super().__init__('pose_to_odom')
        self.declare_parameter('pose_topic', '/current_pose')
        self.declare_parameter('odom_topic', '/small_vgicp/odometry')
        self.declare_parameter('child_frame_id', 'base_link')

        pose_topic = str(self.get_parameter('pose_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Odometry, odom_topic, qos)
        self.subscription = self.create_subscription(PoseStamped, pose_topic, self.on_pose, qos)
        self.get_logger().info(f'Publishing Odometry {odom_topic} from PoseStamped {pose_topic}')

    def on_pose(self, msg: PoseStamped) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose
        self.publisher.publish(odom)


def main() -> None:
    rclpy.init()
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
