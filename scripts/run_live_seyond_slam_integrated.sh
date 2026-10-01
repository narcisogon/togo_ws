#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_live_slam.launch.py

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi

ros2 launch "$LAUNCH_FILE" \
  use_sim_time:=true \
  lidar_topic:=/a300_0000/sensors/seyond_robin_w/scan/points \
  imu_topic:=/a300_0000/sensors/seyond_robin_w/imu \
  lidar_frame:=seyond_robin_w_lidar_frame \
  imu_frame:=seyond_robin_w_imu_frame \
  base_frame:=base_link \
  odom_frame:=odom \
  save_dir:=/ws/src/lidarslam_ros2/output/husky_seyond_graph \
  map_save_period:=10 \
  rviz:=true