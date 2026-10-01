#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_faster_lio_slam.launch.py
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-/ws/src/lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml}
FASTER_LIO_PARAM_FILE=${FASTER_LIO_PARAM_FILE:-/ws/src/lidarslam_ros2/faster_lio/faster-lio/config/seyond_robin_w.yaml}
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
SLAM_RVIZ=${SLAM_RVIZ:-true}

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi

if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "Missing $SLAM_PARAM_FILE" >&2
  exit 1
fi

if [[ ! -f "$FASTER_LIO_PARAM_FILE" ]]; then
  echo "Missing $FASTER_LIO_PARAM_FILE" >&2
  exit 1
fi

ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  faster_lio_param_file:="$FASTER_LIO_PARAM_FILE" \
  use_sim_time:=true \
  map_save_period:=60 \
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE" \
  rviz:="$SLAM_RVIZ"
