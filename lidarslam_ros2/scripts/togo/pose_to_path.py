#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPath(Node):
    def __init__(self):
        super().__init__('pose_to_path')

        self.declare_parameter('pose_topic', '/dlio/pose')
        self.declare_parameter('path_topic', '/dlio/path_simple')
        self.declare_parameter('max_poses', 12000)

        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.max_poses = self.get_parameter('max_poses').value

        self.path = Path()

        self.sub = self.create_subscription(
            PoseStamped, pose_topic, self.on_pose, 10
        )
        self.pub = self.create_publisher(Path, path_topic, 10)
        self.get_logger().info(f'{pose_topic} -> {path_topic}')

    def on_pose(self, msg: PoseStamped):
        # Set frame from incoming message
        self.path.header.frame_id = msg.header.frame_id
        self.path.header.stamp = msg.header.stamp

        self.path.poses.append(msg)

        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = PoseToPath()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
