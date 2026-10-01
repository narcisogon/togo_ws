#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path

class GazeboGroundTruthPath(Node):
    def __init__(self):
        super().__init__('gazebo_gt_path_publisher')

        # ✅ Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('z_offset', 0.0)
        self.declare_parameter('slam_path_topic', '/dlio/path')
        self.declare_parameter('fixed_frame', 'odom')

        self.z_offset = self.get_parameter('z_offset').value
        self.slam_path_topic = self.get_parameter('slam_path_topic').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        self.get_logger().info(f'Z offset: {self.z_offset}')
        self.get_logger().info(f'SLAM path topic: {self.slam_path_topic}')
        self.get_logger().info(f'Fixed frame: {self.fixed_frame}')

        # ✅ Path message
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        # ✅ Subscribers
        self.sub = self.create_subscription(
            PoseArray,
            '/model/togo/pose',
            self.pose_callback,
            10
        )

        # ✅ Publisher
        self.pub = self.create_publisher(Path, '/ground_truth/path', 10)

        self.get_logger().info('Gazebo ground truth path publisher started!')

    def pose_callback(self, msg: PoseArray):
        # Safety check — make sure index 5 exists
        if len(msg.poses) < 6:
            self.get_logger().warn(
                f'Expected at least 6 poses, got {len(msg.poses)}',
                throttle_duration_sec=2.0
            )
            return

        # Index 5 = togo world pose
        world_pose = msg.poses[5]

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = self.fixed_frame

        pose_stamped.pose.position.x = world_pose.position.x
        pose_stamped.pose.position.y = world_pose.position.y
        pose_stamped.pose.position.z = world_pose.position.z - self.z_offset  # ✅ apply offset
        pose_stamped.pose.orientation = world_pose.orientation

        self.path.poses.append(pose_stamped)
        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = GazeboGroundTruthPath()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
