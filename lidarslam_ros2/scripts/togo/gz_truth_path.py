#!/usr/bin/env python3
"""
gz_truth_path.py
All-in-one:
  - Launches ros_gz_bridge
  - Publishes aligned Gazebo ground truth path
  - Shows live position error plot (full history, growing timeline)
"""

from __future__ import annotations

import math
import subprocess
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
import matplotlib.patheffects as patheffects


# ── Helpers ────────────────────────────────────────────────────────────────────

def quat_multiply(q1: tuple, q2: tuple) -> tuple:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z)
    )

def yaw_to_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def rotate_point_by_yaw(x: float, y: float, yaw: float) -> tuple:
    return (
        math.cos(yaw)*x - math.sin(yaw)*y,
        math.sin(yaw)*x + math.cos(yaw)*y,
    )


# ── Node ───────────────────────────────────────────────────────────────────────

class GZTruthPath(Node):
    def __init__(self) -> None:
        super().__init__('gz_truth_path')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('fixed_frame',  'odom')
        self.declare_parameter('model_topic',  '/model/togo/pose')
        self.declare_parameter('slam_topic',   '/dlio/path_simple')
        self.declare_parameter('output_topic', '/ground_truth/path')
        self.declare_parameter('pose_index',   5)
        self.declare_parameter('buffer_size',  500)

        self.fixed_frame  = self.get_parameter('fixed_frame').value
        model_topic       = self.get_parameter('model_topic').value
        slam_topic        = self.get_parameter('slam_topic').value
        output_topic      = self.get_parameter('output_topic').value
        self.pose_index   = self.get_parameter('pose_index').value
        self.buffer_size  = self.get_parameter('buffer_size').value

        # ── GT path state ──────────────────────────────────────────────────────
        self.offset:           tuple | None = None
        self.yaw_offset:       float | None = None
        self.slam_initial:     tuple | None = None
        self.slam_initial_yaw: float | None = None
        self.slam_first_stamp: float | None = None
        self.gt_pose_buffer:   list         = []

        self.path = Path()
        self.path.header.frame_id = self.fixed_frame

        # ── Error plot state ───────────────────────────────────────────────────
        # Unbounded deques — we keep the FULL history forever. No eviction.
        self.lock         = threading.Lock()
        self.t0:          float | None = None
        self.times:       deque[float] = deque()
        self.errors:      deque[float] = deque()
        self.gt_latest:   tuple | None = None
        self.slam_latest: tuple | None = None

        # ── Distance-driven tracking ────────────────────────────────────────────
        self.total_distance: float        = 0.0
        self.last_gt_pos:    tuple | None = None

        # ── Subscribers ────────────────────────────────────────────────────────
        self.create_subscription(PoseArray, model_topic,  self.on_gz_pose,   10)
        self.create_subscription(Path,      slam_topic,   self.on_slam_path, 10)

        # ── Publisher ──────────────────────────────────────────────────────────
        self.pub = self.create_publisher(Path, output_topic, 10)

        self.get_logger().info(
            f'gz_truth_path started\n'
            f'  model topic : {model_topic}\n'
            f'  slam topic  : {slam_topic}\n'
            f'  output topic: {output_topic}\n'
            f'  fixed frame : {self.fixed_frame}\n'
            f'  pose index  : {self.pose_index}'
        )

    # ── SLAM path callback ─────────────────────────────────────────────────────
    def on_slam_path(self, msg: Path) -> None:
        if msg.poses:
            p = msg.poses[-1].pose.position
            with self.lock:
                self.slam_latest = (p.x, p.y, p.z)
            self.try_update_error()

        if self.slam_initial is not None or not msg.poses:
            return

        first = msg.poses[0]
        p     = first.pose.position
        o     = first.pose.orientation

        self.slam_initial     = (p.x, p.y, p.z)
        self.slam_initial_yaw = yaw_from_quat(o)
        self.slam_first_stamp = (
            first.header.stamp.sec +
            first.header.stamp.nanosec * 1e-9
        )

        self.get_logger().info(
            f'SLAM first pose  t={self.slam_first_stamp:.2f} '
            f'x={p.x:.4f} y={p.y:.4f} z={p.z:.4f} '
            f'yaw={math.degrees(self.slam_initial_yaw):.2f}°'
        )
        self.try_compute_offset()

    # ── Gazebo pose callback ───────────────────────────────────────────────────
    def on_gz_pose(self, msg: PoseArray) -> None:
        if len(msg.poses) <= self.pose_index:
            self.get_logger().warn(
                f'Expected at least {self.pose_index + 1} poses, '
                f'got {len(msg.poses)}',
                throttle_duration_sec=2.0,
            )
            return

        world_pose = msg.poses[self.pose_index]
        stamp      = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.gt_pose_buffer.append((
            stamp,
            (world_pose.position.x,
             world_pose.position.y,
             world_pose.position.z),
            world_pose.orientation
        ))
        if len(self.gt_pose_buffer) > self.buffer_size:
            self.gt_pose_buffer.pop(0)

        if self.offset is None and self.slam_first_stamp is not None:
            self.try_compute_offset()

        if self.offset is None:
            self.get_logger().info(
                'Buffering GT poses, waiting for SLAM first pose...',
                throttle_duration_sec=2.0,
            )
            return

        rel_x = world_pose.position.x - self.offset[0]
        rel_y = world_pose.position.y - self.offset[1]
        rel_z = world_pose.position.z - self.offset[2]

        rot_x, rot_y   = rotate_point_by_yaw(rel_x, rel_y, -self.yaw_offset)
        yaw_correction = yaw_to_quat(-self.yaw_offset)
        corrected_quat = quat_multiply(
            yaw_correction,
            (world_pose.orientation.x, world_pose.orientation.y,
             world_pose.orientation.z, world_pose.orientation.w)
        )

        ps = PoseStamped()
        ps.header.stamp       = msg.header.stamp
        ps.header.frame_id    = self.fixed_frame
        ps.pose.position.x    = rot_x
        ps.pose.position.y    = rot_y
        ps.pose.position.z    = rel_z
        ps.pose.orientation.x = corrected_quat[0]
        ps.pose.orientation.y = corrected_quat[1]
        ps.pose.orientation.z = corrected_quat[2]
        ps.pose.orientation.w = corrected_quat[3]

        self.path.poses.append(ps)
        self.path.header.stamp = msg.header.stamp
        self.pub.publish(self.path)

        # ── Update distance driven ──────────────────────────────────────────────
        with self.lock:
            if self.last_gt_pos is not None:
                seg_dx = rot_x - self.last_gt_pos[0]
                seg_dy = rot_y - self.last_gt_pos[1]
                seg_dz = rel_z - self.last_gt_pos[2]
                seg_len = math.sqrt(seg_dx**2 + seg_dy**2 + seg_dz**2)
                self.total_distance += seg_len
            self.last_gt_pos = (rot_x, rot_y, rel_z)

            self.gt_latest = (rot_x, rot_y, rel_z)

        self.try_update_error()

    # ── Offset computation ─────────────────────────────────────────────────────
    def try_compute_offset(self) -> None:
        if self.slam_first_stamp is None or not self.gt_pose_buffer:
            return
        if self.offset is not None:
            return

        closest_time, closest_pos, closest_ori = min(
            self.gt_pose_buffer,
            key=lambda e: abs(e[0] - self.slam_first_stamp)
        )

        time_diff       = abs(closest_time - self.slam_first_stamp)
        gt_yaw          = yaw_from_quat(closest_ori)
        self.offset     = (closest_pos[0], closest_pos[1], closest_pos[2])
        self.yaw_offset = gt_yaw - self.slam_initial_yaw

        self.get_logger().info(
            f'GT pose matched  t={closest_time:.2f} (diff={time_diff:.3f}s) '
            f'x={closest_pos[0]:.4f} y={closest_pos[1]:.4f} z={closest_pos[2]:.4f} '
            f'yaw={math.degrees(gt_yaw):.2f}°'
        )
        self.get_logger().info(
            f'Offset applied   '
            f'dx={self.offset[0]:.4f} dy={self.offset[1]:.4f} '
            f'dz={self.offset[2]:.4f} dyaw={math.degrees(self.yaw_offset):.2f}°'
        )

    # ── Error update ───────────────────────────────────────────────────────────
# ── Error update ───────────────────────────────────────────────────────────
    def try_update_error(self) -> None:
        with self.lock:
            if self.gt_latest is None or self.slam_latest is None:
                return

            dx   = self.gt_latest[0] - self.slam_latest[0]
            dy   = self.gt_latest[1] - self.slam_latest[1]
            dz   = self.gt_latest[2] - self.slam_latest[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            stamp = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = stamp
            t = stamp - self.t0

            # No eviction — we keep every sample forever. Full history.
            self.times.append(t)
            self.errors.append(dist)


# ── Plot ───────────────────────────────────────────────────────────────────────

def make_plot(node: GZTruthPath) -> None:

    BG_WHITE   = '#ffffff'
    BG_AXES    = '#f9f9f9'
    COL_LINE   = '#1a6fbb'
    COL_FILL   = '#1a6fbb'
    COL_GRID   = '#dddddd'
    COL_TEXT   = '#1a1a1a'
    COL_WARN   = '#e63946'
    COL_GOOD   = '#2a9d4e'
    COL_BORDER = '#bbbbbb'
    COL_BOX_BG = '#f0f4ff'
    COL_MARKER = '#0d3b66'

    plt.rcParams.update({
        'figure.autolayout': False,
        'font.family':       'DejaVu Sans',
        'font.size':         11,
    })

    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    fig.patch.set_facecolor(BG_WHITE)
    fig.canvas.manager.set_window_title('SLAM Accuracy Monitor')
    fig.suptitle(
        'SLAM Accuracy Monitor — Live Position Error',
        fontsize=16, color=COL_TEXT, fontweight='bold',
        x=0.01, ha='left'
    )

    ax.set_facecolor(BG_AXES)
    for spine in ax.spines.values():
        spine.set_edgecolor(COL_BORDER)
        spine.set_linewidth(1.2)

    # ── Bigger, bold ticks and axis labels ─────────────────────────────────────
    ax.tick_params(colors=COL_TEXT, labelsize=12, which='both', width=1.2)
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontweight('bold')

    ax.set_xlabel('Elapsed Time (s)', labelpad=10,
                   fontsize=14, fontweight='bold', color=COL_TEXT)
    ax.set_ylabel('Position Error (m)', labelpad=10,
                   fontsize=14, fontweight='bold', color=COL_TEXT)

    ax.grid(True, color=COL_GRID, linewidth=0.8, linestyle='--', alpha=0.9)
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(True, which='minor', color=COL_GRID, linewidth=0.4, alpha=0.5)

    line, = ax.plot(
        [], [], color=COL_LINE,
        linewidth=2.4, zorder=3,
        solid_capstyle='round',
        solid_joinstyle='round',
        path_effects=[
            patheffects.SimpleLineShadow(offset=(0, -0.5), alpha=0.25),
            patheffects.Normal()
        ]
    )

    current_dot, = ax.plot(
        [], [], marker='o', markersize=8,
        color=COL_MARKER, markeredgecolor='white',
        markeredgewidth=1.5, zorder=4
    )

    ax.axhline(y=1.0, color=COL_WARN, linewidth=1.4,
               linestyle='--', alpha=0.75, zorder=1)
    ax.text(
        0.995, 1.03, '1.0 m threshold',
        transform=ax.get_yaxis_transform(),
        ha='right', fontsize=9.5, fontweight='bold',
        color=COL_WARN, alpha=0.9, style='italic'
    )

    stats_box = ax.text(
        0.99, 0.97, '',
        transform=ax.transAxes,
        fontsize=11.5,
        verticalalignment='top',
        horizontalalignment='right',
        color=COL_TEXT,
        fontfamily='monospace',
        fontweight='bold',
        bbox=dict(
            boxstyle='round,pad=0.7',
            facecolor=COL_BOX_BG,
            edgecolor=COL_BORDER,
            linewidth=1.2,
            alpha=0.95
        ),
        zorder=5
    )

    status_dot = ax.text(
        0.01, 0.97, '●  Waiting for data...',
        transform=ax.transAxes,
        fontsize=11.5,
        fontweight='bold',
        verticalalignment='top',
        color='#aaaaaa',
        zorder=5
    )

    def update(_frame):
        with node.lock:
            if not node.times:
                return line, current_dot, stats_box, status_dot

            times    = list(node.times)
            errors   = list(node.errors)
            distance = node.total_distance

        t_max   = times[-1]
        e_max   = max(errors)
        current = errors[-1]
        mean    = sum(errors) / len(errors)

        ax.set_xlim(0.0, max(t_max * 1.05, 10.0))
        ax.set_ylim(0, max(e_max * 1.3, 1.2))

        line.set_data(times, errors)
        current_dot.set_data([times[-1]], [errors[-1]])

        for coll in ax.collections:
            coll.remove()
        ax.fill_between(times, errors, alpha=0.12,
                        color=COL_FILL, zorder=2)

        dot_color    = COL_GOOD if current < 1.0 else COL_WARN
        status_label = '●  GOOD' if current < 1.0 else '●  DRIFT DETECTED'
        current_dot.set_color(dot_color)

        stats_box.set_text(
            f'Current  : {current:>7.3f} m\n'
            f'Mean     : {mean:>7.3f} m\n'
            f'Max      : {e_max:>7.3f} m\n'
            f'Distance : {distance:>7.2f} m\n'
            f'Samples  : {len(errors):>7d}'
        )
        status_dot.set_text(f'{status_label}   {current:.3f} m')
        status_dot.set_color(dot_color)

        return line, current_dot, stats_box, status_dot

    ani = animation.FuncAnimation(
        fig, update, interval=500, blit=False
    )

    fig.canvas.mpl_connect('resize_event', lambda e: fig.canvas.draw_idle())
    plt.show()


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    # ✅ Launch bridge
    bridge_proc = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/model/togo/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'
    ])
    print('[bridge] ros_gz_bridge started')

    rclpy.init()
    node = GZTruthPath()

    # ✅ Spin ROS in background thread
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    ros_thread.start()

    try:
        make_plot(node)
    except KeyboardInterrupt:
        pass
    finally:
        print('[bridge] shutting down...')
        bridge_proc.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
