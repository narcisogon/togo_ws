#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/er4-user/slam_ws/install/setup.bash

set -u

# ── Bag file ────────────────────────────────────────────────────────────────
BAG_PATH=${BAG_PATH:-/home/er4-user/ws/src/rosbag2_2026_07_16-21_12_33}
PLAYBACK_RATE=${PLAYBACK_RATE:-1.0}

# ── Launch / param files ────────────────────────────────────────────────────
LAUNCH_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py
COMBINED_PARAM_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-$COMBINED_PARAM_FILE}
DLIO_PARAM_FILE=${DLIO_PARAM_FILE:-$COMBINED_PARAM_FILE}

# ── Sensor wiring ───────────────────────────────────────────────────────────
# Both topics point directly at what the bag actually publishes.
# --remap silently fails in Jazzy bag play so we avoid it entirely.
LIDAR_TOPIC=/iv_points
LIDAR_TOPIC_TIMED="${LIDAR_TOPIC}_timed"
IMU_TOPIC=/husky/sensors/imu_0/data
LIDAR_FRAME=lidar3d_0_laser
IMU_FRAME=imu_0_link
BASE_FRAME=base_link
ODOM_FRAME=odom

# ── Flags ───────────────────────────────────────────────────────────────────
ENABLE_MAP_SAVE_PULSE=false
SLAM_RVIZ=true
TIMED_CLOUD_SCAN_PERIOD=0.0666666667
TIMED_CLOUD_REVERSE_COLUMNS=false
DLIO_DESKEW=false

# ── Pre-flight checks ────────────────────────────────────────────────────────
if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "ERROR: Missing $LAUNCH_FILE" >&2; exit 1
fi
if [[ ! -f "$SLAM_PARAM_FILE" ]]; then
  echo "ERROR: Missing $SLAM_PARAM_FILE" >&2; exit 1
fi
if [[ ! -d "$BAG_PATH" ]]; then
  echo "ERROR: Bag not found at: $BAG_PATH" >&2; exit 1
fi

echo "──────────────────────────────────────────────"
echo "  BAG          : $BAG_PATH"
echo "  Rate         : $PLAYBACK_RATE"
echo "  LIDAR  topic : $LIDAR_TOPIC  (direct from bag)"
echo "  IMU    topic : $IMU_TOPIC    (direct from bag)"
echo "  LiDAR  frame : $LIDAR_FRAME"
echo "  IMU    frame : $IMU_FRAME"
echo "  Base   frame : $BASE_FRAME"
echo "  Odom   frame : $ODOM_FRAME"
echo "  Deskew       : $DLIO_DESKEW"
echo "──────────────────────────────────────────────"

# ── 1. Start SLAM stack ─────────────────────────────────────────────────────
ros2 launch "$LAUNCH_FILE" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  dlio_param_file:="$DLIO_PARAM_FILE" \
  use_sim_time:=true \
  map_save_period:=60 \
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE" \
  timed_cloud_scan_period:="$TIMED_CLOUD_SCAN_PERIOD" \
  timed_cloud_reverse_columns:="$TIMED_CLOUD_REVERSE_COLUMNS" \
  dlio_deskew:="$DLIO_DESKEW" \
  correct_cloud_slant:=false \
  lidar_topic:="$LIDAR_TOPIC" \
  lidar_topic_timed:="$LIDAR_TOPIC_TIMED" \
  imu_topic:="$IMU_TOPIC" \
  lidar_frame:="$LIDAR_FRAME" \
  imu_frame:="$IMU_FRAME" \
  base_frame:="$BASE_FRAME" \
  odom_frame:="$ODOM_FRAME" \
  rviz:="$SLAM_RVIZ" &

SLAM_PID=$!

# ── 2. Wait for SLAM to be ready ────────────────────────────────────────────
echo "Waiting for DLIO to be ready..."
until ros2 topic list 2>/dev/null | grep -qx /dlio/odometry; do
  sleep 0.5
done
echo "SLAM ready — starting bag playback."

# ── 3. Play bag — no remaps needed, SLAM subscribes to bag topics directly ──
ros2 bag play "$BAG_PATH" \
  --clock \
  --rate "$PLAYBACK_RATE"

# ── 4. Bag done — final map save ────────────────────────────────────────────
echo "Bag finished. Waiting 10s for final graph optimisation..."
sleep 10

if ros2 service list 2>/dev/null | grep -qx /map_save; then
  echo "Saving map..."
  ros2 service call /map_save std_srvs/srv/Empty
fi

wait "$SLAM_PID"
