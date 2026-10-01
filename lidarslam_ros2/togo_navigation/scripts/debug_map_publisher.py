#!/usr/bin/env python3
"""Publish a simple transient-local OccupancyGrid for first Nav2/RViz tests."""

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DebugMapPublisher(Node):
    def __init__(self):
        super().__init__('debug_map_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/map')
        self.declare_parameter('resolution', 0.20)
        self.declare_parameter('width_m', 40.0)
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('border_width_cells', 2)
        self.declare_parameter('publish_period_sec', 1.0)

        topic = self.get_parameter('topic').value
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, topic, qos)
        self.map_msg = self._make_map()

        period = float(self.get_parameter('publish_period_sec').value)
        self.timer = self.create_timer(period, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'Publishing debug OccupancyGrid {topic} in {self.map_msg.header.frame_id} '
            f'({self.map_msg.info.width}x{self.map_msg.info.height}, '
            f'{self.map_msg.info.resolution:.2f} m/cell)'
        )

    def _make_map(self):
        frame_id = self.get_parameter('frame_id').value
        resolution = float(self.get_parameter('resolution').value)
        width_m = float(self.get_parameter('width_m').value)
        height_m = float(self.get_parameter('height_m').value)
        border = int(self.get_parameter('border_width_cells').value)

        width = max(1, int(math.ceil(width_m / resolution)))
        height = max(1, int(math.ceil(height_m / resolution)))

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = -0.5 * width * resolution
        msg.info.origin.position.y = -0.5 * height * resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (width * height)

        for y in range(height):
            for x in range(width):
                if x < border or y < border or x >= width - border or y >= height - border:
                    msg.data[y * width + x] = 100

        center_x = width // 2
        center_y = height // 2
        marker_half_width = max(1, int(round(0.20 / resolution)))
        marker_half_length = max(1, int(round(3.0 / resolution)))
        for offset in range(-marker_half_length, marker_half_length + 1):
            for thickness in range(-marker_half_width, marker_half_width + 1):
                x = center_x + offset
                y = center_y + thickness
                if 0 <= x < width and 0 <= y < height:
                    msg.data[y * width + x] = 100
                x = center_x + thickness
                y = center_y + offset
                if 0 <= x < width and 0 <= y < height:
                    msg.data[y * width + x] = 100

        obstacle_centers_m = [(4.0, 2.0), (6.0, -3.0), (-5.0, 4.0)]
        obstacle_radius_cells = max(1, int(round(0.45 / resolution)))
        for ox_m, oy_m in obstacle_centers_m:
            ox = int(round((ox_m - msg.info.origin.position.x) / resolution))
            oy = int(round((oy_m - msg.info.origin.position.y) / resolution))
            for dy in range(-obstacle_radius_cells, obstacle_radius_cells + 1):
                for dx in range(-obstacle_radius_cells, obstacle_radius_cells + 1):
                    if dx * dx + dy * dy > obstacle_radius_cells * obstacle_radius_cells:
                        continue
                    x = ox + dx
                    y = oy + dy
                    if 0 <= x < width and 0 <= y < height:
                        msg.data[y * width + x] = 100

        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main():
    rclpy.init()
    node = DebugMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
