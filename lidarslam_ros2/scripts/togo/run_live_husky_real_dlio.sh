#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
SLAM_WS=${SLAM_WS:-/home/er4-user/slam_ws}
if [[ ! -f "$SLAM_WS/install/setup.bash" ]]; then
  echo "ERROR: missing $SLAM_WS/install/setup.bash" >&2
  echo "Set SLAM_WS to the rover workspace and build it first." >&2
  exit 1
fi
source "$SLAM_WS/install/setup.bash"
set -u

REPO=${REPO:-$SLAM_WS/src/lidarslam_ros2}
LAUNCH_FILE=${LAUNCH_FILE:-$REPO/lidarslam/launch/seyond_dlio_live_real.launch.py}
SOURCE_LAUNCH=${SOURCE_LAUNCH:-$REPO/lidarslam/launch/seyond_dlio_slam.launch.py}
BASE_PARAM_FILE=${BASE_PARAM_FILE:-$REPO/lidarslam/param/seyond_dlio_graph.yaml}
LIVE_PARAM_FILE=${LIVE_PARAM_FILE:-$REPO/lidarslam/param/seyond_dlio_live_real.yaml}

RAW_LIDAR_TOPIC=${RAW_LIDAR_TOPIC:-/iv_points}
CORRECTED_LIDAR_TOPIC=${CORRECTED_LIDAR_TOPIC:-/iv_points/time_corrected}
IMU_TOPIC=${IMU_TOPIC:-/husky/sensors/imu_0/data}
LIDAR_TIME_OFFSET_SEC=${LIDAR_TIME_OFFSET_SEC:--6.575}
ENFORCE_TIMING_SYNC=${ENFORCE_TIMING_SYNC:-false}
DLIO_DESKEW=${DLIO_DESKEW:-false}
DLIO_GROUND_CONSTRAINT=${DLIO_GROUND_CONSTRAINT:-false}
DLIO_GROUND_BASELINK_HEIGHT=${DLIO_GROUND_BASELINK_HEIGHT:-0.30}
PUBLISH_SENSOR_STATIC_TF=${PUBLISH_SENSOR_STATIC_TF:-false}
SLAM_RVIZ=${SLAM_RVIZ:-true}
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
MAP_SAVE_PERIOD=${MAP_SAVE_PERIOD:-60}
STARTUP_DIAGNOSTIC_DELAY=${STARTUP_DIAGNOSTIC_DELAY:-8}
GRAPH_MAP_SAVE_DIR=${GRAPH_MAP_SAVE_DIR:-$REPO/output/husky_seyond_live_real}
GRAPH_DEM_OUTPUT_DIR=${GRAPH_DEM_OUTPUT_DIR:-$GRAPH_MAP_SAVE_DIR/lunar_dem}

# Per-process cache names prevent surviving files from an earlier launch from
# being visible to this graph. Only this run's own tmpfs directory is removed.
DEFAULT_GRAPH_CACHE=/dev/shm/graph_slam_pcd_cache_live_real_$$
GRAPH_PCD_CACHE_DIR=${GRAPH_PCD_CACHE_DIR:-$DEFAULT_GRAPH_CACHE}

for required in \
  "$LAUNCH_FILE" \
  "$SOURCE_LAUNCH" \
  "$BASE_PARAM_FILE" \
  "$LIVE_PARAM_FILE"; do
  if [[ ! -e "$required" ]]; then
    echo "ERROR: missing $required" >&2
    echo "Build the workspace before starting the live rover launch." >&2
    exit 1
  fi
done

for executable in ros2 timeout awk grep; do
  if ! command -v "$executable" >/dev/null 2>&1; then
    echo "ERROR: required command '$executable' is unavailable." >&2
    exit 1
  fi
done

existing_nodes=$(ros2 node list 2>/dev/null || true)
for stale_node in \
  /dlio_odom_node \
  /dlio_map_node \
  /graph_based_slam \
  /seyond_live_timestamp_corrector; do
  if grep -qx "$stale_node" <<<"$existing_nodes"; then
    echo "ERROR: $stale_node is already alive; refusing to start a duplicate map." >&2
    echo "Stop the owning launch and verify it disappears from 'ros2 node list'." >&2
    exit 1
  fi
done

existing_corrected_publishers=$(ros2 topic info "$CORRECTED_LIDAR_TOPIC" 2>/dev/null | \
  awk '/Publisher count:/ {print $3}' || true)
if [[ ${existing_corrected_publishers:-0} -gt 0 ]]; then
  echo "ERROR: $CORRECTED_LIDAR_TOPIC already has a publisher." >&2
  echo "Stop the old timestamp adapter before starting this live launch." >&2
  exit 1
fi

LAUNCH_PID=''
LAUNCH_IS_SESSION=false
cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Stopping the live SLAM launch cleanly..."
    if [[ "$LAUNCH_IS_SESSION" == true ]]; then
      kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    else
      kill -INT "$LAUNCH_PID" 2>/dev/null || true
    fi
    for _ in {1..50}; do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "Launch did not finish after SIGINT; sending SIGTERM." >&2
      if [[ "$LAUNCH_IS_SESSION" == true ]]; then
        kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      else
        kill -TERM "$LAUNCH_PID" 2>/dev/null || true
      fi
    fi
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  # Never delete a caller-provided cache. The generated path is uniquely ours.
  if [[ "$GRAPH_PCD_CACHE_DIR" == "$DEFAULT_GRAPH_CACHE" && -d "$GRAPH_PCD_CACHE_DIR" ]]; then
    rm -rf -- "$GRAPH_PCD_CACHE_DIR"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "Starting live Husky DLIO + graph SLAM"
echo "  LiDAR:    $RAW_LIDAR_TOPIC -> $CORRECTED_LIDAR_TOPIC"
echo "  IMU:      $IMU_TOPIC"
echo "  time:     wall clock (use_sim_time=false), LiDAR offset=${LIDAR_TIME_OFFSET_SEC}s"
echo "  safety:   enforce_timing_sync=$ENFORCE_TIMING_SYNC"
echo "  DLIO:     deskew=$DLIO_DESKEW crop_box_half_size=0.5m ground_constraint=$DLIO_GROUND_CONSTRAINT"
echo "  TF:       publish_sensor_static_tf=$PUBLISH_SENSOR_STATIC_TF"
echo "Keep the rover stationary for at least the first three seconds."

launch_command=(
  ros2 launch "$LAUNCH_FILE"
  source_launch:="$SOURCE_LAUNCH"
  repo_root:="$REPO"
  base_param_file:="$BASE_PARAM_FILE"
  live_param_file:="$LIVE_PARAM_FILE"
  raw_lidar_topic:="$RAW_LIDAR_TOPIC"
  corrected_lidar_topic:="$CORRECTED_LIDAR_TOPIC"
  imu_topic:="$IMU_TOPIC"
  lidar_time_offset_sec:="$LIDAR_TIME_OFFSET_SEC"
  enforce_timing_sync:="$ENFORCE_TIMING_SYNC"
  deskew:="$DLIO_DESKEW"
  ground_constraint:="$DLIO_GROUND_CONSTRAINT"
  ground_baselink_height:="$DLIO_GROUND_BASELINK_HEIGHT"
  publish_sensor_static_tf:="$PUBLISH_SENSOR_STATIC_TF"
  graph_pcd_cache_dir:="$GRAPH_PCD_CACHE_DIR"
  graph_map_save_dir:="$GRAPH_MAP_SAVE_DIR"
  graph_dem_output_dir:="$GRAPH_DEM_OUTPUT_DIR"
  map_save_period:="$MAP_SAVE_PERIOD"
  enable_map_save_pulse:="$ENABLE_MAP_SAVE_PULSE"
  rviz:="$SLAM_RVIZ"
)

if command -v setsid >/dev/null 2>&1; then
  setsid "${launch_command[@]}" &
  LAUNCH_IS_SESSION=true
else
  "${launch_command[@]}" &
fi
LAUNCH_PID=$!

echo "Waiting for the live sensor chain..."
raw_publishers=0
corrected_publishers=0
imu_publishers=0
corrected_subscribers=0
for _ in {1..80}; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: live launch exited during startup." >&2
    exit 1
  fi
  raw_publishers=$(ros2 topic info "$RAW_LIDAR_TOPIC" 2>/dev/null | \
    awk '/Publisher count:/ {print $3}' || true)
  corrected_publishers=$(ros2 topic info "$CORRECTED_LIDAR_TOPIC" 2>/dev/null | \
    awk '/Publisher count:/ {print $3}' || true)
  corrected_subscribers=$(ros2 topic info "$CORRECTED_LIDAR_TOPIC" 2>/dev/null | \
    awk '/Subscription count:/ {print $3}' || true)
  imu_publishers=$(ros2 topic info "$IMU_TOPIC" 2>/dev/null | \
    awk '/Publisher count:/ {print $3}' || true)
  if [[ ${raw_publishers:-0} -ge 1 && ${corrected_publishers:-0} -ge 1 && \
        ${corrected_subscribers:-0} -ge 1 && ${imu_publishers:-0} -ge 1 ]]; then
    break
  fi
  sleep 0.5
done

if [[ ${raw_publishers:-0} -lt 1 || ${imu_publishers:-0} -lt 1 ]]; then
  echo "ERROR: rover sensor publishers were not discovered within 40 seconds." >&2
  echo "  $RAW_LIDAR_TOPIC publishers=${raw_publishers:-0}" >&2
  echo "  $IMU_TOPIC publishers=${imu_publishers:-0}" >&2
  exit 1
fi
if [[ ${corrected_publishers:-0} -lt 1 || ${corrected_subscribers:-0} -lt 1 ]]; then
  echo "ERROR: corrected LiDAR chain did not connect within 40 seconds." >&2
  exit 1
fi

sleep "$STARTUP_DIAGNOSTIC_DELAY"
echo
echo "================ LIVE STARTUP DIAGNOSTICS ================"
echo "--- Endpoints ---"
ros2 topic info "$RAW_LIDAR_TOPIC" 2>&1 || true
ros2 topic info "$CORRECTED_LIDAR_TOPIC" 2>&1 || true
ros2 topic info "$IMU_TOPIC" 2>&1 || true
ros2 topic info /dlio/odometry 2>&1 || true

echo "--- Receive rates (short samples) ---"
timeout 4 ros2 topic hz "$RAW_LIDAR_TOPIC" --window 10 2>&1 || true
timeout 4 ros2 topic hz "$CORRECTED_LIDAR_TOPIC" --window 10 2>&1 || true
timeout 4 ros2 topic hz "$IMU_TOPIC" --window 100 2>&1 || true

echo "--- Corrected sensor stamps ---"
if ! timeout 3 ros2 topic echo "$CORRECTED_LIDAR_TOPIC" --once --field header.stamp 2>&1; then
  echo "ERROR: no corrected LiDAR cloud received within 3s." >&2
  echo "Check the timestamp-corrector warning; the offset safety gate may be blocking bad data." >&2
  exit 1
fi
timeout 3 ros2 topic echo "$IMU_TOPIC" --once --field header.stamp 2>&1 || \
  echo "ERROR: no IMU message received within 3s."

echo "--- DLIO output ---"
timeout 4 ros2 topic echo /dlio/odometry --once 2>&1 >/dev/null && \
  echo "OK: DLIO is publishing odometry." || \
  echo "WAITING: DLIO has not published odometry; inspect calibration/timing logs above."
timeout 5 python3 "$REPO/scripts/togo/dlio_diag_once.py" 2>&1 || \
  echo "WAITING: no labeled DLIO diagnostic message received."
echo "=========================================================="
echo "Live mapping is running. Press Ctrl-C once for coordinated shutdown."

set +e
wait "$LAUNCH_PID"
launch_status=$?
set -e
LAUNCH_PID=''
exit "$launch_status"
