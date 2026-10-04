#!/usr/bin/env python3
"""Isolated ROS smoke test; requires the built graph/Nav2 overlay, no rover.

Run with ROS_DOMAIN_ID=199 after sourcing the overlay. Logs go to --log-dir.
Synthetic odometry/clouds and TF exercise the real backend and Nav2 processes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool, String
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Probe(Node):
    def __init__(self):
        super().__init__('hazard_integration_probe')
        self.odom = self.create_publisher(Odometry, '/dlio/odometry', 10)
        self.cloud = self.create_publisher(point_cloud2.PointCloud2, '/dlio/deskewed', 10)
        self.goals = self.create_publisher(PoseStamped, '/goal_pose_requested', 5)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        lidar = TransformStamped()
        lidar.header.stamp = self.get_clock().now().to_msg()
        lidar.header.frame_id, lidar.child_frame_id = 'base_link', 'lidar3d_0_laser'
        lidar.transform.translation.x, lidar.transform.translation.z = 0.4, 0.313
        lidar.transform.rotation.w = 1.0
        self.static_tf.sendTransform(lidar)
        self.maps = {}
        self.ready = False
        self.ready_events = []
        self.status = ''
        self.commands = []
        self.plan = None
        self.pitch = 0.0
        self.publish_inputs = True
        self.publish_odom = True
        self.publish_cloud = True
        self.diagnostics = []
        self.obstacles = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ['/map', '/hazard/raw', '/global_costmap/costmap', '/local_costmap/costmap']:
            self.create_subscription(OccupancyGrid, topic,
                lambda msg, key=topic: self.maps.__setitem__(key, msg), qos)
        self.create_subscription(Bool, '/hazard/ready', self.health, 10)
        self.create_subscription(String, '/hazard/status', lambda msg: setattr(self, 'status', msg.data), qos)
        self.create_subscription(String, '/hazard/diagnostics',
            lambda msg: self.diagnostics.append((time.monotonic(), json.loads(msg.data))), 10)
        self.create_subscription(point_cloud2.PointCloud2, '/hazard/obstacles',
            lambda msg: setattr(self, 'obstacles', list(point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))), qos)
        self.create_subscription(TwistStamped, '/platform_velocity_controller/cmd_vel',
                                 self.command, 10)
        self.create_subscription(RosPath, '/plan', lambda msg: setattr(self, 'plan', msg), 10)
        # Offset cell centers avoid floating-point bin boundaries.
        self.points = []
        for ix in range(-40, 41):
            for iy in range(-40, 41):
                x, y = (ix + 0.5) * 0.1, (iy + 0.5) * 0.1
                rise = 0.65 * max(0.0, min(x - 1.5, 0.9)) if abs(y) < 0.65 else 0.0
                # Seed the entire ramp below the body before applying pitch.
                # Elevated unsupported tops intentionally do not bootstrap ground.
                self.points.append((x, y, -0.9 + rise))
        self.create_timer(0.05, self.inputs)

    def health(self, msg):
        self.ready = msg.data
        self.ready_events.append((time.monotonic(), msg.data))

    def command(self, msg):
        self.commands.append((time.monotonic(), msg.twist.linear.x, msg.twist.angular.z))

    def inputs(self, force_cloud=False):
        stamp = self.get_clock().now().to_msg()
        qy, qw = math.sin(self.pitch / 2), math.cos(self.pitch / 2)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.w = qy, qw
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        if self.publish_inputs:
            if self.publish_odom:
                self.odom.publish(odom)
            if self.publish_cloud or force_cloud:
                self.cloud.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=stamp, frame_id='odom'), self.points))

    def wait(self, predicate, seconds, description):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                print('PASS: ' + description, flush=True)
                return
        raise AssertionError('Timed out: ' + description)

    def settle(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def parameter(self, name, value):
        client = self.create_client(SetParametersAtomically,
                                    '/graph_based_slam/set_parameters_atomically')
        assert client.wait_for_service(timeout_sec=5)
        parameter_value = (ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                         integer_value=value) if isinstance(value, int) else
                           ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                          double_value=value))
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name=name, value=parameter_value)])
        future = client.call_async(request)
        self.wait(future.done, 5, 'parameter service response')
        return future.result().result

    def check_import(self, key):
        source, imported = self.maps['/map'], self.maps[key]
        assert source.info.width == imported.info.width
        assert source.info.height == imported.info.height
        assert abs(source.info.resolution - imported.info.resolution) < 1e-6
        assert source.info.origin == imported.info.origin
        # Jazzy StaticLayer rescales 0..99 to 0..251 internally. The published
        # OccupancyGrid translates those costs back using Nav2's cost table.
        def translated(cost):
            if cost < 0:
                return -1
            if cost == 100:
                return 100
            internal = int(cost * 254.0 / 100)
            return 0 if internal == 0 else 1 + 97 * (internal - 1) // 251
        expected = [translated(v) for v in source.data]
        assert list(imported.data) == expected, key + ' changed the backend costs'
        assert any(0 < v < 100 for v in imported.data), 'gradient was lost'

    def raw_at(self, x, y):
        source = self.maps['/hazard/raw']
        ix = int(math.floor((x-source.info.origin.position.x)/source.info.resolution))
        iy = int(math.floor((y-source.info.origin.position.y)/source.info.resolution))
        return source.data[iy*source.info.width+ix]

    def target_confidence(self):
        values = [float(p[3]) for p in self.obstacles
                  if abs(float(p[0])-1.05)<0.02 and abs(float(p[1])-0.05)<0.02
                  and abs(float(p[2])-0.35)<0.02]
        return max(values, default=0.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-dir', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '199', 'Use isolated ROS_DOMAIN_ID=199'
    repo = Path(__file__).resolve().parents[3]
    logs = Path(args.log_dir)
    logs.mkdir(parents=True, exist_ok=True)
    processes, handles = [], []
    def start(name, command):
        handle = (logs / (name + '.log')).open('w')
        handles.append(handle)
        processes.append(subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                          start_new_session=True))
    rclpy.init()
    probe = Probe()
    try:
        start('backend', ['ros2', 'run', 'graph_based_slam', 'graph_based_slam_node',
            '--ros-args', '--params-file',
            str(repo / 'lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml'),
            '-p', 'use_sim_time:=false', '-p', 'dem/enabled:=false',
            '-p', 'odom_input_cloud_in_odom_frame:=true',
            # Keep the regression fixture independent of operator tuning.
            '-p', 'hazard/max_slope_deg:=25.0', '-p', 'hazard/lethal_inflation_m:=0.1',
            '-p', 'hazard/soft_inflation_m:=0.3', '-p', 'hazard/max_input_age_sec:=1.0',
            '-p', 'hazard/unknown_cost:=-1',
            '-p', 'hazard/update_period_sec:=0.2',
            '-p', 'hazard/resolution_m:=0.1', '-p', 'use_scan_context:=false',
            '-p', 'use_triangle_descriptor:=false', '-p', 'use_distance_loop_candidates:=false',
            '-p', 'map_save_dir:=/tmp/hazard-test-map',
            '-p', 'pcd_cache_dir:=/tmp/hazard-test-pcd',
            '-r', 'odom_input:=/dlio/odometry', '-r', 'cloud_input:=/dlio/deskewed'])
        probe.wait(lambda: probe.ready and '/hazard/raw' in probe.maps, 30,
                   'backend worker publishes a fresh map')
        original = sum(v == 100 for v in probe.maps['/hazard/raw'].data)
        assert original > 0, 'steep terrain must be lethal before inflation'
        probe.pitch = math.radians(20)
        probe.settle(2)
        tilted = sum(v == 100 for v in probe.maps['/hazard/raw'].data)
        assert abs(tilted - original) <= max(2, original * 0.05), (original, tilted)
        print('PASS: body pitch preserves terrain classification', flush=True)
        assert not probe.parameter('hazard/soft_inflation_m', 0.05).successful
        assert not probe.parameter('hazard/height_noise_m', 0.0).successful
        assert not probe.parameter('hazard/min_plane_coverage', 0.9).successful
        assert not probe.parameter('hazard/clear_slope_margin_deg', 25.0).successful
        assert not probe.parameter('hazard/clear_confirmations', 0).successful
        assert not probe.parameter('hazard/obstacle_clear_probability', 0.8).successful
        assert not probe.parameter('hazard/max_rays', 0).successful
        assert not probe.parameter('hazard/unknown_cost', -2).successful
        assert not probe.parameter('hazard/unknown_cost', 101).successful
        assert probe.parameter('hazard/soft_inflation_m', 0.4).successful
        probe.wait(lambda: probe.ready, 5, 'valid runtime settings rebuild the map')
        probe.pitch = 0.0
        probe.settle(1)
        assert probe.diagnostics[-1][1]['refitted_cells'] == 0, probe.diagnostics[-1]
        print('PASS: unchanged stationary scans do not refit historical terrain', flush=True)
        assert any(v['batch_clouds'] == 3 for _, v in probe.diagnostics)
        print('PASS: three fresh clouds are processed as a batch', flush=True)
        probe.publish_cloud = False
        probe.settle(0.3)
        count = len(probe.diagnostics)
        sent = time.monotonic()
        probe.inputs(force_cloud=True)
        probe.wait(lambda: len(probe.diagnostics) > count, 0.8,
                   'a partial stationary batch is processed without waiting for movement')
        assert probe.diagnostics[-1][1]['batch_clouds'] == 1, probe.diagnostics[-1]
        assert probe.diagnostics[-1][0] - sent < 0.6
        probe.publish_cloud = True
        original_points = probe.points
        retained = list(probe.maps['/hazard/raw'].data)
        # Repeated flat returns in one previously hazardous cell cannot establish
        # that the surrounding slope is safe. Keep the rest of the terrain cached.
        probe.points = [(2.05, 0.05, -0.9)]
        probe.settle(1.5)
        current = probe.maps['/hazard/raw'].data
        assert all(current[i] == 100 for i, v in enumerate(retained) if v == 100)
        assert probe.ready, probe.status
        print('PASS: isolated contradictory returns retain hazards while maps stay fresh', flush=True)
        cleared_before = len(probe.diagnostics)
        probe.points = [(x, y, -0.9) for x, y, z in original_points]
        probe.wait(lambda: all(v <= 0 for v in probe.maps['/map'].data), 5,
                   'confirmed well-supported flat terrain removes hazards and inflation')
        probe.wait(lambda: any(v['cleared_hazards'] > 0 for _, v in
                               probe.diagnostics[cleared_before:]), 2,
                   'diagnostics report confirmed hazard clearances')
        flat_points = probe.points
        probe.points = flat_points + [(1.03, 0.03, 0.339), (1.05, 0.05, 0.349),
                                     (1.07, 0.07, 0.359)]
        probe.wait(lambda: probe.raw_at(1.05, 0.05) == 100 and
                   probe.target_confidence() > 89.9, 5,
                   'above-ground obstacle marks and saturates bounded confidence')
        probe.settle(2)
        assert probe.target_confidence() <= 90.01
        assert probe.raw_at(1.05, 0.05) == 100
        # Rays cross the formerly occupied height, with endpoints beyond it.
        # Repeat enough distinct rays to exercise the bounded sampling budget.
        probe.points = flat_points + [(2.05, 0.10 + i*0.0001, 0.406 + i*0.0001)
                                     for i in range(21)]
        probe.wait(lambda: probe.raw_at(1.05, 0.05) == 0, 5,
                   'observed free occupied volume clears a long-standing obstacle')
        probe.wait(lambda: any(v['cleared_obstacles'] > 0 for _, v in probe.diagnostics),
                   2, 'diagnostics report confirmed obstacle clearance')
        assert all(v['traced_rays'] <= 3000 for _, v in probe.diagnostics)
        print('PASS: bounded obstacle confidence clears visible volume while preserving ground',
              flush=True)
        probe.points = original_points
        probe.wait(lambda: any(v == 100 for v in probe.maps['/hazard/raw'].data), 5,
                   'fresh steep observations restore hazards')
        probe.ready_events.clear()
        probe.publish_odom = False
        probe.settle(0.4)
        assert probe.ready_events and all(v for _, v in probe.ready_events), probe.ready_events
        print('PASS: a brief odometry wait preserves a still-fresh completed map', flush=True)
        probe.publish_odom = True
        probe.settle(0.5)
        assert probe.parameter('hazard/max_input_age_sec', 3.0).successful
        assert probe.parameter('hazard/update_period_sec', 1.3).successful
        probe.settle(0.3)
        probe.wait(lambda: probe.ready, 5, 'slow-cadence fixture becomes ready')
        probe.ready_events.clear()
        probe.settle(1.7)
        assert len(probe.ready_events) >= 5 and all(v for _, v in probe.ready_events)
        gaps = [b[0] - a[0] for a, b in zip(probe.ready_events, probe.ready_events[1:])]
        assert max(gaps) < 0.8, gaps
        print('PASS: readiness heartbeats continue independently of rasterization cadence', flush=True)
        assert probe.parameter('hazard/update_period_sec', 0.2).successful
        assert probe.parameter('hazard/max_input_age_sec', 1.0).successful
        probe.settle(0.3)
        probe.wait(lambda: probe.ready, 5, 'normal freshness settings restored')
        start('nav2', ['ros2', 'launch',
            str(repo / 'lidarslam_ros2/togo_navigation/launch/rover_nav2.launch.py'),
            'use_sim_time:=false', 'rviz:=false', 'params_file:=' +
            str(repo / 'lidarslam_ros2/togo_navigation/config/nav2_slam_params.yaml')])
        probe.wait(lambda: '/local_costmap/costmap' in probe.maps and
                   '/global_costmap/costmap' in probe.maps, 40, 'both Nav2 costmaps import the grid')
        probe.settle(2)
        probe.check_import('/global_costmap/costmap')
        probe.check_import('/local_costmap/costmap')
        print('PASS: Nav2 preserves geometry and adaptive gradient without extra layers', flush=True)
        # Numeric unknown costs affect planning, while raw evidence stays unknown.
        for unknown_cost in (50, 100, -1):
            assert probe.parameter('hazard/unknown_cost', unknown_cost).successful
            def unknown_cost_applied():
                if not probe.ready:
                    return False
                raw, final = probe.maps['/hazard/raw'], probe.maps['/map']
                unknown = [i for i, value in enumerate(raw.data) if value < 0]
                return bool(unknown) and all(
                    final.data[i] == 100 or final.data[i] == -1
                    if unknown_cost == -1 else final.data[i] >= unknown_cost
                    for i in unknown)
            probe.wait(unknown_cost_applied, 5, 'unknown planning cost applies without changing raw evidence')
            probe.settle(1.5)
            probe.check_import('/global_costmap/costmap')
            probe.check_import('/local_costmap/costmap')
            if unknown_cost == 100:
                raw = probe.maps['/hazard/raw']
                index = next(i for i, value in enumerate(raw.data) if value < 0)
                blocked = PoseStamped()
                blocked.header.frame_id = 'map'
                blocked.header.stamp = probe.get_clock().now().to_msg()
                blocked.pose.orientation.w = 1.0
                blocked.pose.position.x = raw.info.origin.position.x + (
                    index % raw.info.width + 0.5) * raw.info.resolution
                blocked.pose.position.y = raw.info.origin.position.y + (
                    index // raw.info.width + 0.5) * raw.info.resolution
                probe.goals.publish(blocked)
                probe.wait(lambda: 'Goal rejected at requested position: cell cost=100' in
                           (logs / 'nav2.log').read_text(), 5,
                           'unknown cost 100 rejects a goal in unknown terrain')
                assert probe.plan is None
        print('PASS: both Nav2 costmaps preserve configurable unknown costs', flush=True)
        # Allow lifecycle activation to finish before the RViz-style goal.
        probe.settle(3)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = probe.get_clock().now().to_msg()
        goal.pose.position.x = 3.6
        goal.pose.position.y = 2.5
        goal.pose.orientation.w = 1.0
        source = probe.maps['/map']
        gx = int((goal.pose.position.x - source.info.origin.position.x) / source.info.resolution)
        gy = int((goal.pose.position.y - source.info.origin.position.y) / source.info.resolution)
        assert 0 <= source.data[gy * source.info.width + gx] < 100, 'test goal must be observed and nonlethal'
        probe.publish_inputs = False
        probe.wait(lambda: not probe.ready, 4, 'temporary map staleness before waypoint request')
        superseded = PoseStamped()
        superseded.header = goal.header
        superseded.pose.position.x = -2.0
        superseded.pose.position.y = -2.0
        superseded.pose.orientation.w = 1.0
        probe.goals.publish(superseded)
        probe.settle(0.15)
        probe.goals.publish(goal)
        probe.settle(0.5)
        assert probe.plan is None, 'pending waypoint must not reach Nav2 while the map is stale'
        probe.publish_inputs = True
        probe.wait(lambda: probe.plan is not None and len(probe.plan.poses) > 2, 15,
                   'pending RViz-style waypoint is forwarded after readiness recovers')
        endpoint = probe.plan.poses[-1].pose.position
        assert math.hypot(endpoint.x - goal.pose.position.x, endpoint.y - goal.pose.position.y) < 0.6
        print('PASS: the latest pending waypoint replaces the previous request', flush=True)
        for pose in probe.plan.poses:
            gx = int((pose.pose.position.x - source.info.origin.position.x) / source.info.resolution)
            gy = int((pose.pose.position.y - source.info.origin.position.y) / source.info.resolution)
            assert 0 <= gx < source.info.width and 0 <= gy < source.info.height
            assert source.data[gy * source.info.width + gx] != 100, 'path crosses a backend lethal cell'
        print('PASS: planned path avoids backend lethal cells', flush=True)
        probe.wait(lambda: any(abs(x) + abs(z) > 0.01 for _, x, z in probe.commands[-20:]),
                   5, 'Nav2 controller commands reach the platform relay')
        probe.publish_inputs = False
        stopped_at = time.monotonic()
        probe.wait(lambda: not probe.ready, 4, 'backend detects stalled inputs')
        probe.settle(max(0.4, 2.0 - (time.monotonic() - stopped_at)))
        recent = [(x, z) for t, x, z in probe.commands if t > stopped_at + 1.5]
        assert recent and all(x == 0.0 and z == 0.0 for x, z in recent), recent
        print('PASS: stale backend blocks continued Nav2 motion', flush=True)
        assert probe.status == 'published_map_stale', probe.status
        goal.pose.position.x = -2.0
        probe.goals.publish(goal)
        probe.settle(5.5)
        text = (logs / 'nav2.log').read_text()
        assert 'Goal expired after' in text, 'a pending waypoint must expire'
        probe.publish_inputs = True
        probe.wait(lambda: probe.ready, 5, 'backend recovers after pending goal expiration')
        probe.settle(0.5)
        assert (logs / 'nav2.log').read_text().count('Forwarded waypoint to Nav2') == 1
        print('PASS: expired waypoint is not executed after a later recovery', flush=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        for process in processes:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for process in processes:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    main()
