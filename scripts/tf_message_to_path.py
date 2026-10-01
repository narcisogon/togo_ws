#!/usr/bin/env python3
"""Convert a specific transform from a TFMessage topic into a nav_msgs/Path."""

from __future__ import annotations

from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfMessageToPath(Node):
    def __init__(self) -> None:
        super().__init__('tf_message_to_path')
        self.declare_parameter('tf_topic', '/a300_0000/tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('path_topic', '/reference/path')
        self.declare_parameter('max_poses', 12000)
        self.declare_parameter('min_distance_m', 0.02)
        self.declare_parameter('publish_frame', 'odom')

        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.publish_frame = str(self.get_parameter('publish_frame').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)
        tf_topic = str(self.get_parameter('tf_topic').value)

        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.subscription = self.create_subscription(TFMessage, tf_topic, self.on_tf, 100)
        self.poses: deque[PoseStamped] = deque(maxlen=max(1, self.max_poses))
        self.last_xy: tuple[float, float] | None = None
        self.last_stamp_ns: int | None = None
        self.get_logger().info(
            f'Publishing {self.path_topic} from {tf_topic} transform '
            f'{self.parent_frame} -> {self.child_frame}'
        )

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.header.frame_id != self.parent_frame:
                continue
            if transform.child_frame_id != self.child_frame:
                continue
            self.append_transform(transform)

    def append_transform(self, transform) -> None:
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return
        self.last_stamp_ns = stamp_ns

        t = transform.transform.translation
        q = transform.transform.rotation
        xy = (t.x, t.y)
        if self.last_xy is not None:
            dx = xy[0] - self.last_xy[0]
            dy = xy[1] - self.last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self.min_distance_m:
                return
        self.last_xy = xy

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.publish_frame or transform.header.frame_id
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = q
        self.poses.append(pose)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = pose.header.frame_id
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main() -> None:
    rclpy.init()
    node = TfMessageToPath()
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