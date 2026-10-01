#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

OUT_NAME="${1:-seyond_slam_run}"
mkdir -p /bags

ros2 bag record \
  /clock \
  /a300_0000/sensors/seyond_robin_w/scan/points \
  /seyond_robin_w/points_xyzi \
  /a300_0000/sensors/seyond_robin_w/imu \
  /a300_0000/platform/odom \
  /a300_0000/tf \
  /a300_0000/tf_static \
  /tf \
  /tf_static \
  /rko_lio/odometry \
  /rko_lio/odometry_stable \
  /rko_lio/path \
  /rko_lio/path_stable \
  /rko_lio/registration_diagnostics \
  /rko_lio/runtime_diagnostics \
  /rko_lio/stability_status \
  /rko_lio/frame \
  /rko_lio/frame_xyzi \
  /rko_lio/local_map \
  /small_vgicp/current_pose \
  /small_vgicp/odometry \
  /small_vgicp/path \
  /modified_map \
  /modified_path \
  /modified_map_array \
  /reference/path \
  -o "/bags/${OUT_NAME}"
