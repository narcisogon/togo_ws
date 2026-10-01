#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

LIDAR_TOPIC=/a300_0000/sensors/seyond_robin_w/scan/points
IMU_TOPIC=/a300_0000/sensors/seyond_robin_w/imu

wait_for_topic() {
  local topic="$1"
  local label="$2"
  echo "Waiting for ${label}: ${topic}"
  for _ in $(seq 1 20); do
    if timeout 3 ros2 topic echo "$topic" --once >/tmp/rko_wait_topic.log 2>&1; then
      echo "  ${label} is publishing."
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: ${label} did not publish on ${topic}" >&2
  cat /tmp/rko_wait_topic.log >&2 || true
  exit 1
}

wait_for_tf() {
  local parent="$1"
  local child="$2"
  echo "Waiting for TF: ${parent} -> ${child}"
  for _ in $(seq 1 20); do
    timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" >/tmp/rko_wait_tf.log 2>&1 || true
    if grep -q "Translation:" /tmp/rko_wait_tf.log; then
      echo "  TF ${parent} -> ${child} is available."
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: TF ${parent} -> ${child} was not available" >&2
  cat /tmp/rko_wait_tf.log >&2 || true
  exit 1
}

wait_for_topic /clock "sim clock"
wait_for_topic "$LIDAR_TOPIC" "Seyond LiDAR point cloud"
wait_for_topic "$IMU_TOPIC" "Seyond IMU"
wait_for_tf base_link seyond_robin_w_lidar_frame
wait_for_tf base_link seyond_robin_w_imu_frame

ros2 launch rko_lio odometry.launch.py \
  mode:=online \
  lidar_topic:=$LIDAR_TOPIC \
  imu_topic:=$IMU_TOPIC \
  lidar_frame:=seyond_robin_w_lidar_frame \
  imu_frame:=seyond_robin_w_imu_frame \
  base_frame:=base_link \
  odom_frame:=odom \
  odom_topic:=/rko_lio/odometry \
  deskew:=false \
  voxel_size:=0.20 \
  double_downsample:=false \
  max_correspondance_distance:=4.0 \
  max_scan_delta_sec:=10.0 \
  min_range:=0.2 \
  max_range:=80.0 \
  publish_deskewed_scan:=true \
  publish_local_map:=true \
  use_sim_time:=true \
  rviz:=true \
  log_level:=info