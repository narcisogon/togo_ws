#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

# ── Launch / param files ────────────────────────────────────────────────────────
LAUNCH_FILE=/ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py
COMBINED_PARAM_FILE=${COMBINED_PARAM_FILE:-/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph_husky.yaml}
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-$COMBINED_PARAM_FILE}
DLIO_PARAM_FILE=${DLIO_PARAM_FILE:-$COMBINED_PARAM_FILE}

# ── Sensor wiring — matches /husky sim topic list ───────────────────────────────
LIDAR_TOPIC=${LIDAR_TOPIC:-/husky/sensors/seyond/points}
LIDAR_TOPIC_TIMED="${LIDAR_TOPIC}_timed"
IMU_TOPIC=${IMU_TOPIC:-/husky/sensors/imu_0/data_raw}
LIDAR_FRAME=${LIDAR_FRAME:-lidar3d_0_laser}
IMU_FRAME=${IMU_FRAME:-imu_0_link}
BASE_FRAME=${BASE_FRAME:-base_link}
ODOM_FRAME=${ODOM_FRAME:-odom}

# ── Behaviour flags ─────────────────────────────────────────────────────────────
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
SLAM_RVIZ=${SLAM_RVIZ:-true}
TIMED_CLOUD_SCAN_PERIOD=${TIMED_CLOUD_SCAN_PERIOD:-0.0666666667}
TIMED_CLOUD_REVERSE_COLUMNS=${TIMED_CLOUD_REVERSE_COLUMNS:-false}
DLIO_DESKEW=${DLIO_DESKEW:-true}

# ── Pre-flight checks ───────────────────────────────────────────────────────────
if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "Missing $LAUNCH_FILE" >&2
  echo "Make sure ./lidarslam_ros2 is mounted into the Docker container." >&2
  exit 1
fi
if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "Missing $SLAM_PARAM_FILE" >&2
  exit 1
fi
if [[ ! -f "$DLIO_PARAM_FILE" ]]; then
  echo "Missing $DLIO_PARAM_FILE" >&2
  exit 1
fi

echo "──────────────────────────────────────────────"
echo "  LIDAR  topic : $LIDAR_TOPIC"
echo "  LIDAR  timed : $LIDAR_TOPIC_TIMED"
echo "  IMU    topic : $IMU_TOPIC"
echo "  LiDAR  frame : $LIDAR_FRAME"
echo "  IMU    frame : $IMU_FRAME"
echo "  Base   frame : $BASE_FRAME"
echo "  Odom   frame : $ODOM_FRAME"
echo "  Deskew       : $DLIO_DESKEW"
echo "──────────────────────────────────────────────"

ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  dlio_param_file:="$DLIO_PARAM_FILE" \
  use_sim_time:=true \
  map_save_period:=60 \
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE" \
  timed_cloud_scan_period:="$TIMED_CLOUD_SCAN_PERIOD" \
  timed_cloud_reverse_columns:="$TIMED_CLOUD_REVERSE_COLUMNS" \
  dlio_deskew:="$DLIO_DESKEW" \
  lidar_topic:="$LIDAR_TOPIC" \
  lidar_topic_timed:="$LIDAR_TOPIC_TIMED" \
  imu_topic:="$IMU_TOPIC" \
  lidar_frame:="$LIDAR_FRAME" \
  imu_frame:="$IMU_FRAME" \
  base_frame:="$BASE_FRAME" \
  odom_frame:="$ODOM_FRAME" \
  rviz:="$SLAM_RVIZ"