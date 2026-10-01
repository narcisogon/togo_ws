#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_live_slam.launch.py
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-/ws/src/lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml}
ENABLE_FRONTEND_STABILITY_FILTER=${ENABLE_FRONTEND_STABILITY_FILTER:-false}
ENABLE_IMU_PREDICTION=${ENABLE_IMU_PREDICTION:-false}
BACKEND_ODOM_TOPIC=${BACKEND_ODOM_TOPIC:-/rko_lio/odometry}
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
SLAM_RVIZ=${SLAM_RVIZ:-false}

if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi

if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "Missing $SLAM_PARAM_FILE" >&2
  echo "Edit or create the unified frontend/backend YAML before launching." >&2
  exit 1
fi

ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  use_sim_time:=true \
  map_save_period:=60 \
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE" \
  enable_frontend_stability_filter:="$ENABLE_FRONTEND_STABILITY_FILTER" \
  enable_imu_prediction:="$ENABLE_IMU_PREDICTION" \
  backend_odom_topic:="$BACKEND_ODOM_TOPIC" \
  rviz:="$SLAM_RVIZ"
