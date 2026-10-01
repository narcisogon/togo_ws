#!/usr/bin/env bash`nset -eo pipefail`n`nsource /opt/ros/jazzy/setup.bash`nset -u

ros2 launch rko_lio odometry.launch.py \
  mode:=online \
  lidar_topic:=/a300_0000/sensors/seyond_robin_w/scan/points \
  imu_topic:=/a300_0000/sensors/seyond_robin_w/imu \
  lidar_frame:=seyond_robin_w_lidar_frame \
  imu_frame:=seyond_robin_w_imu_frame \
  base_frame:=base_link \
  odom_frame:=odom \
  odom_topic:=/rko_lio/odometry \
  deskew:=false \
  voxel_size:=0.25 \
  double_downsample:=false \
  max_correspondance_distance:=3.0 \
  max_scan_delta_sec:=5.0 \
  min_range:=0.2 \
  max_range:=80.0 \
  publish_deskewed_scan:=true \
  publish_local_map:=true \
  use_sim_time:=true \
  rviz:=true \
  log_level:=info