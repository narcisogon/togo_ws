#!/usr/bin/env python3
"""Ground truth path for RViz, in the SLAM odom frame.

Takes the simulator's model pose (geometry_msgs/PoseArray, one entry per link; `pose_index` picks the body) and
publishes it as a nav_msgs/Path aligned to DLIO's frame, so the truth overlays /dlio/path_simple in RViz.

The anchoring is the one slam_eval.py scores with ("start-anchored"):
  - DLIO's frame starts at its first REAL odometry pose; the exact-identity poses it publishes from stamp 0 until
    its IMU calibration ends are skipped (same test as slam_eval.drop_placeholders);
  - the truth is interpolated at that stamp (position linearly, orientation by nlerp);
  - the yaw-only transform that makes the two coincide is applied to every later truth sample:
        p_odom = Rz(-th0) (g - g0) + e0,   th0 = yaw(g0) - yaw(e0).
So what RViz shows is what the scorer measures. No standstill is needed for the anchor to be right.

Cheap by design: it reads /dlio/odometry (not the growing Path), keeps a bounded path with a pose every
`min_distance_m`, publishes at most every `publish_period_sec`, and never reads a clock (every stamp comes from
the messages), so run it WITHOUT use_sim_time (with it, rclpy would process every /clock message, about 1 kHz).

    python3 gazebo_gt_path_publisher.py --ros-args -p model_topic:=/model/togo/pose -p slam_topic:=/dlio/odometry
"""

from __future__ import annotations

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def quat_multiply(q1: tuple, q2: tuple) -> tuple:
    """Hamilton product of two quaternions (x, y, z, w): rotation q2 followed by q1."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def yaw_of(q: tuple) -> float:
    """Yaw of a quaternion (x, y, z, w): atan2(R10, R00), as slam_eval.yaw_of."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def nlerp(qa: tuple, qb: tuple, alpha: float) -> tuple:
    """Normalised linear interpolation between unit quaternions (shortest arc), as slam_eval.interpolate_gt."""
    if sum(a * b for a, b in zip(qa, qb)) < 0.0:
        qb = tuple(-v for v in qb)
    q = tuple((1.0 - alpha) * a + alpha * b for a, b in zip(qa, qb))
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return tuple(v / n for v in q)


def stamp_of(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class GazeboGroundTruthPath(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_gt_path_publisher')

        self.declare_parameter('fixed_frame', 'odom')          # frame of the output path (DLIO's odom frame)
        self.declare_parameter('model_topic', '/model/togo/pose')  # geometry_msgs/PoseArray from the simulator
        self.declare_parameter('slam_topic', '/dlio/odometry')     # nav_msgs/Odometry of the frontend
        self.declare_parameter('output_topic', '/ground_truth/path')
        self.declare_parameter('pose_index', 5)                # entry of the PoseArray that is the body
        self.declare_parameter('buffer_size', 500)             # truth samples kept before the anchor (48 Hz: 10 s)
        self.declare_parameter('min_distance_m', 0.05)         # the path gets a pose every this much travel
        self.declare_parameter('publish_period_sec', 0.5)      # publish at most this often
        self.declare_parameter('max_poses', 20000)             # cap on the path length

        self.fixed_frame = self.get_parameter('fixed_frame').value
        model_topic = self.get_parameter('model_topic').value
        slam_topic = self.get_parameter('slam_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.pose_index = int(self.get_parameter('pose_index').value)
        buffer_size = max(2, int(self.get_parameter('buffer_size').value))
        self.min_distance_m = max(0.0, float(self.get_parameter('min_distance_m').value))
        publish_period = max(0.05, float(self.get_parameter('publish_period_sec').value))
        max_poses = max(1, int(self.get_parameter('max_poses').value))

        # Truth samples: (stamp, (x, y, z), (qx, qy, qz, qw), header.stamp of the message)
        self.truth: deque = deque(maxlen=buffer_size)
        # DLIO's first real pose (the anchor instant) and the resulting yaw-only alignment
        self.anchor_stamp: float | None = None
        self.e0: tuple | None = None
        self.yaw_e0: float = 0.0
        self.g0: tuple | None = None
        self.cos_th: float = 1.0
        self.sin_th: float = 0.0
        self.q_corr: tuple = (0.0, 0.0, 0.0, 1.0)
        self.warned_before_buffer = False

        self.poses: deque = deque(maxlen=max_poses)
        self.last_xyz: tuple | None = None
        self.latest_stamp = None
        self.dirty = False

        odom_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        path_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Path, output_topic, path_qos)
        self.truth_sub = self.create_subscription(PoseArray, model_topic, self.on_truth, 10)
        self.odom_sub = self.create_subscription(Odometry, slam_topic, self.on_odom, odom_qos)
        self.timer = self.create_timer(publish_period, self.publish_path)

        self.get_logger().info(
            "gazebo_gt_path_publisher started\n"
            f"  model topic : {model_topic}\n"
            f"  slam topic  : {slam_topic} (nav_msgs/Odometry)\n"
            f"  output topic: {output_topic} (at most every {publish_period:.2f} s, a pose per "
            f"{self.min_distance_m:.3f} m, cap {max_poses})\n"
            f"  fixed frame : {self.fixed_frame}\n"
            f"  pose index  : {self.pose_index}"
        )

    # -- DLIO odometry: only its first real pose matters ---------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        if self.anchor_stamp is not None:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        real = any(abs(v) > 0.0 for v in (p.x, p.y, p.z, q.x, q.y, q.z)) or q.w != 1.0
        if not real:
            return  # a stamp-0 placeholder published before DLIO's frame exists
        self.anchor_stamp = stamp_of(msg.header)
        self.e0 = (p.x, p.y, p.z)
        self.yaw_e0 = yaw_of((q.x, q.y, q.z, q.w))
        self.get_logger().info(
            f"SLAM first real pose  t={self.anchor_stamp:.3f} x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} "
            f"yaw={math.degrees(self.yaw_e0):.2f} deg"
        )
        self.try_anchor()

    # -- simulator truth -----------------------------------------------------------------------------------------
    def on_truth(self, msg: PoseArray) -> None:
        if len(msg.poses) <= self.pose_index:
            self.get_logger().warn(
                f"Expected at least {self.pose_index + 1} poses in the PoseArray, got {len(msg.poses)}",
                throttle_duration_sec=2.0)
            return
        pose = msg.poses[self.pose_index]
        sample = (stamp_of(msg.header),
                  (pose.position.x, pose.position.y, pose.position.z),
                  (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                  msg.header.stamp)
        if self.g0 is not None:
            self.append(sample)
            return
        self.truth.append(sample)
        if self.anchor_stamp is not None:
            self.try_anchor()
        else:
            self.get_logger().info("Buffering truth, waiting for the first real DLIO pose...",
                                   throttle_duration_sec=5.0)

    # -- anchoring ------------------------------------------------------------------------------------------------
    def try_anchor(self) -> None:
        """Interpolate the truth at the anchor stamp once samples on both sides of it exist."""
        if self.g0 is not None or self.anchor_stamp is None or len(self.truth) < 2:
            return
        t0 = self.anchor_stamp
        if self.truth[-1][0] < t0:
            return  # the truth has not reached the anchor instant yet; keep buffering
        if self.truth[0][0] > t0:
            if not self.warned_before_buffer:
                self.get_logger().warn(
                    f"The first real DLIO pose (t={t0:.3f}) is older than the oldest truth sample held "
                    f"(t={self.truth[0][0]:.3f}); anchoring on that sample instead (raise buffer_size)")
                self.warned_before_buffer = True
            ta, ga, qa, _ = self.truth[0]
            tb, gb, qb = ta, ga, qa
        else:
            after = next(i for i, s in enumerate(self.truth) if s[0] >= t0)
            tb, gb, qb, _ = self.truth[after]
            ta, ga, qa, _ = self.truth[after - 1] if after > 0 else self.truth[after]
        alpha = 0.0 if tb <= ta else (t0 - ta) / (tb - ta)
        g0 = tuple(a + alpha * (b - a) for a, b in zip(ga, gb))
        yaw_g0 = yaw_of(nlerp(qa, qb, alpha))
        th0 = math.atan2(math.sin(yaw_g0 - self.yaw_e0), math.cos(yaw_g0 - self.yaw_e0))
        self.g0 = g0
        self.cos_th, self.sin_th = math.cos(-th0), math.sin(-th0)
        self.q_corr = yaw_quat(-th0)
        self.get_logger().info(
            f"Truth at the anchor  t={t0:.3f} (interpolated between samples {ta:.3f} and {tb:.3f}) "
            f"x={g0[0]:.4f} y={g0[1]:.4f} z={g0[2]:.4f} yaw={math.degrees(yaw_g0):.2f} deg")
        self.get_logger().info(
            f"Offset applied   dx={g0[0] - self.e0[0]:.4f} dy={g0[1] - self.e0[1]:.4f} "
            f"dz={g0[2] - self.e0[2]:.4f} dyaw={math.degrees(th0):.2f} deg  "
            f"(truth -> {self.fixed_frame}: p = Rz(-dyaw) (g - g0) + e0, same anchoring as slam_eval.py)")
        self.destroy_subscription(self.odom_sub)
        for sample in list(self.truth):
            if sample[0] >= t0:
                self.append(sample)
        self.truth.clear()

    # -- output path ----------------------------------------------------------------------------------------------
    def append(self, sample: tuple) -> None:
        _, g, q, stamp = sample
        rx, ry, rz = g[0] - self.g0[0], g[1] - self.g0[1], g[2] - self.g0[2]
        xyz = (self.cos_th * rx - self.sin_th * ry + self.e0[0],
               self.sin_th * rx + self.cos_th * ry + self.e0[1],
               rz + self.e0[2])
        self.latest_stamp = stamp
        if self.last_xyz is not None and math.dist(xyz, self.last_xyz) < self.min_distance_m:
            return
        self.last_xyz = xyz
        qo = quat_multiply(self.q_corr, q)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.fixed_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xyz
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qo
        self.poses.append(ps)
        self.dirty = True

    def publish_path(self) -> None:
        if not self.dirty or self.latest_stamp is None:
            return
        path = Path()
        path.header.stamp = self.latest_stamp
        path.header.frame_id = self.fixed_frame
        path.poses = list(self.poses)
        self.pub.publish(path)
        self.dirty = False


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthPath()
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
