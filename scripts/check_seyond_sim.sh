#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

echo "== ROS topics =="
ros2 topic list -t | grep -E "seyond|clock|tf|rko" || true

echo
echo "== Gazebo Seyond topics =="
gz topic -l | grep -i seyond || true

echo
echo "== ROS rates =="
timeout 6 ros2 topic hz /clock || true
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/scan/points || true
timeout 6 ros2 topic hz /a300_0000/sensors/seyond_robin_w/imu || true

echo
echo "== Message headers =="
timeout 4 ros2 topic echo /a300_0000/sensors/seyond_robin_w/scan/points --once | grep frame_id || true
timeout 4 ros2 topic echo /a300_0000/sensors/seyond_robin_w/imu --once | grep frame_id || true

echo
echo "== TF checks =="
timeout 5 ros2 run tf2_ros tf2_echo odom base_link || true
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame || true
timeout 5 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_imu_frame || true

echo
echo "== RKO output rates =="
timeout 6 ros2 topic hz /rko_lio/odometry || true
timeout 6 ros2 topic hz /rko_lio/frame || true
timeout 6 ros2 topic hz /rko_lio/local_map || true