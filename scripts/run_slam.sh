#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/er4-user/ws/install/setup.bash
source /home/er4-user/slam_ws/install/setup.bash
set -u

LAUNCH_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/launch/seyond_live_slam.launch.py

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure lidarslam_ros2 is mounted into the container." >&2
  exit 1
fi

ros2 launch "$LAUNCH_FILE" \
  use_sim_time:=true \
  lidar_topic:=/husky/sensors/seyond/points \
  imu_topic:=/husky/sensors/imu_0/data_raw \
  lidar_frame:=lidar3d_0_laser \
  imu_frame:=imu_0_link \
  base_frame:=base_link \
  odom_frame:=odom \
  save_dir:=/home/er4-user/slam_ws/src/lidarslam_ros2/output/husky_seyond_graph \
  map_save_period:=10 \
  rviz:=true
