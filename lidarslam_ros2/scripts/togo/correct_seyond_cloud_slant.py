#!/usr/bin/env python3
"""Correct the Gazebo Seyond GPU lidar's range-dependent ground slant."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CorrectSeyondCloudSlant(Node):
    def __init__(self):
        super().__init__("correct_seyond_cloud_slant")
        self.declare_parameter("input_topic", "/a300_0000/sensors/seyond_robin_w/scan/points")
        self.declare_parameter("output_topic", "/a300_0000/sensors/seyond_robin_w/scan/points_corrected")
        self.declare_parameter("z_correction_per_meter", 0.0268)
        self.declare_parameter("reference_range_m", 1.25)
        self.declare_parameter("min_range_m", 0.2)
        self.declare_parameter("max_range_m", 12.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.z_correction_per_meter = float(self.get_parameter("z_correction_per_meter").value)
        self.reference_range_m = float(self.get_parameter("reference_range_m").value)
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.correct,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Correcting Seyond cloud slant "
            f"{self.input_topic} -> {self.output_topic}, "
            f"z += {self.z_correction_per_meter:.5f} * (range - {self.reference_range_m:.2f})"
        )

    def correct(self, msg: PointCloud2) -> None:
        field_names = [field.name for field in msg.fields]
        if not {"x", "y", "z"}.issubset(field_names):
            self.publisher.publish(msg)
            return

        x_index = field_names.index("x")
        y_index = field_names.index("y")
        z_index = field_names.index("z")

        corrected_points = []
        for point in point_cloud2.read_points(msg, field_names=field_names, skip_nans=False):
            values = list(point)
            x = float(values[x_index])
            y = float(values[y_index])
            z = float(values[z_index])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xy_range = math.hypot(x, y)
                if self.min_range_m <= xy_range <= self.max_range_m:
                    values[z_index] = z + self.z_correction_per_meter * (
                        xy_range - self.reference_range_m
                    )
            corrected_points.append(tuple(values))

        corrected = point_cloud2.create_cloud(msg.header, msg.fields, corrected_points)
        if msg.height > 1 and len(corrected_points) == msg.height * msg.width:
            corrected.height = msg.height
            corrected.width = msg.width
            corrected.row_step = corrected.point_step * corrected.width
        corrected.is_dense = msg.is_dense
        self.publisher.publish(corrected)


def main():
    rclpy.init()
    node = CorrectSeyondCloudSlant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
