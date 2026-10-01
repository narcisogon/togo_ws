#!/usr/bin/env python3
"""Publish a nav_msgs/Path from a TF transform, useful for RViz trajectory comparison."""

from __future__ import annotations

from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TfToPath(Node):
    def __init__(self) -> None:
        super().__init__('tf_to_path')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('path_topic', '/reference/path')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_poses', 12000)
        self.declare_parameter('min_distance_m', 0.02)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.path_topic = self.get_parameter('path_topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.poses: deque[PoseStamped] = deque(maxlen=max(1, self.max_poses))
        self.last_xy: tuple[float, float] | None = None
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self.on_timer)
        self.get_logger().info(
            f'Publishing {self.path_topic} from TF {self.parent_frame} -> {self.child_frame}'
        )

    def on_timer(self) -> None:
        try:
            tf = self.buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.get_logger().debug(f'TF unavailable: {exc}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        xy = (t.x, t.y)
        if self.last_xy is not None:
            dx = xy[0] - self.last_xy[0]
            dy = xy[1] - self.last_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self.min_distance_m:
                return
        self.last_xy = xy

        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z
        pose.pose.orientation = q
        self.poses.append(pose)

        path = Path()
        path.header.stamp = pose.header.stamp
        path.header.frame_id = self.parent_frame
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main() -> None:
    rclpy.init()
    node = TfToPath()
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