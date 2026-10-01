#!/usr/bin/env python3
"""Republish a PointCloud2 with an intensity field for PointXYZI consumers."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class AddIntensityToCloud(Node):
    def __init__(self):
        super().__init__('add_intensity_to_cloud')
        self.declare_parameter('input_topic', '/rko_lio/frame')
        self.declare_parameter('output_topic', '/rko_lio/frame_xyzi')
        self.declare_parameter('intensity', 1.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.intensity = float(self.get_parameter('intensity').value)

        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.convert,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Adding intensity={self.intensity} to {self.input_topic} -> {self.output_topic}'
        )

    def convert(self, msg: PointCloud2):
        names = [field.name for field in msg.fields]
        if 'intensity' in names:
            self.pub.publish(msg)
            return

        points = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z, self.intensity))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out = point_cloud2.create_cloud(msg.header, fields, points)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AddIntensityToCloud()
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