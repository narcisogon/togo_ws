#!/usr/bin/env bash
# Run the flat live-real Husky DLIO + graph SLAM launch.
# No timestamp corrector, no nested include chain.
# /iv_points is fed directly to DLIO.
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

# ── Paths ──────────────────────────────────────────────────────────────
REPO=${REPO:-$SLAM_WS/src/lidarslam_ros2}
LAUNCH_FILE=${LAUNCH_FILE:-$REPO/lidarslam/launch/seyond_dlio_live_real_flat.launch.py}
PARAM_FILE=${PARAM_FILE:-$REPO/lidarslam/param/seyond_dlio_live_real_flat.yaml}

# ── Topics / frames ────────────────────────────────────────────────────
LIDAR_TOPIC=${LIDAR_TOPIC:-/iv_points}
IMU_TOPIC=${IMU_TOPIC:-/husky/sensors/imu_0/data}
ODOM_FRAME=${ODOM_FRAME:-odom}
BASE_FRAME=${BASE_FRAME:-base_link}
LIDAR_FRAME=${LIDAR_FRAME:-lidar3d_0_laser}
IMU_FRAME=${IMU_FRAME:-imu_0_link}

# ── DLIO tuning ────────────────────────────────────────────────────────
DLIO_DESKEW=${DLIO_DESKEW:-false}
DLIO_GROUND_CONSTRAINT=${DLIO_GROUND_CONSTRAINT:-false}
DLIO_GROUND_BASELINK_HEIGHT=${DLIO_GROUND_BASELINK_HEIGHT:-0.30}

# ── TF control ─────────────────────────────────────────────────────────
# Set true only for a sensor-only bench run without the rover bringup.
PUBLISH_SENSOR_STATIC_TF=${PUBLISH_SENSOR_STATIC_TF:-false}

# ── Graph SLAM I/O ─────────────────────────────────────────────────────
# Per-process cache name prevents a surviving directory from an earlier
# launch being visible to this run. Only this run's own tmpfs dir is removed.
DEFAULT_GRAPH_CACHE=/dev/shm/graph_slam_pcd_cache_live_real_$$
GRAPH_PCD_CACHE_DIR=${GRAPH_PCD_CACHE_DIR:-$DEFAULT_GRAPH_CACHE}
GRAPH_MAP_SAVE_DIR=${GRAPH_MAP_SAVE_DIR:-$REPO/output/husky_seyond_live_real}
GRAPH_DEM_OUTPUT_DIR=${GRAPH_DEM_OUTPUT_DIR:-$GRAPH_MAP_SAVE_DIR/lunar_dem}

# ── Misc ───────────────────────────────────────────────────────────────
ENABLE_MAP_SAVE_PULSE=${ENABLE_MAP_SAVE_PULSE:-false}
MAP_SAVE_PERIOD=${MAP_SAVE_PERIOD:-60}
SLAM_RVIZ=${SLAM_RVIZ:-true}
STARTUP_DIAGNOSTIC_DELAY=${STARTUP_DIAGNOSTIC_DELAY:-10}

# ── Pre-flight: required files ─────────────────────────────────────────
for required in "$LAUNCH_FILE" "$PARAM_FILE"; do
  if [[ ! -e "$required" ]]; then
    echo "ERROR: missing $required" >&2
    exit 1
  fi
done

for cmd in ros2 awk grep timeout; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: required command '$cmd' not found." >&2
    exit 1
  fi
done

# ── Stale node check (warning only — does not prevent launch) ──────────
stale=$(ros2 node list 2>/dev/null | grep -E \
  '^/(dlio_odom_node|dlio_map_node|dlio_odometry_to_tf|dlio_path_publisher\
|graph_based_slam|live_base_to_lidar_tf|live_base_to_imu_tf|rviz2)$' \
  || true)
if [[ -n "$stale" ]]; then
  echo "WARNING: nodes from an earlier session are still alive:" >&2
  printf '  %s\n' $stale >&2
  echo "WARNING: continuing anyway — duplicate nodes may corrupt TF or the map." >&2
  echo "         If things misbehave, Ctrl-C, run:" >&2
  echo "         ros2 node list | grep -E 'dlio|graph|rviz'" >&2
  echo "         and kill the old launch before retrying." >&2
else
  echo "No stale nodes found — environment is clean."
fi

# ── PID tracking ───────────────────────────────────────────────────────
LAUNCH_PID=''
LAUNCH_IS_SESSION=false

# ── Graceful kill helper ───────────────────────────────────────────────
# Usage: _kill_wait <pid> <is_session> <label>
# Sends INT, waits up to 5 s, escalates to TERM, waits 3 s, then KILL.
_kill_wait() {
  local pid=$1 is_session=$2 label=$3
  local target
  if [[ "$is_session" == true ]]; then
    target="-$pid"   # negative PID = entire process group
  else
    target="$pid"
  fi

  if ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  echo "  Stopping $label (pid=$pid)..."
  kill -INT $target 2>/dev/null || true

  local i
  for i in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done

  echo "  $label still alive after SIGINT; sending SIGTERM." >&2
  kill -TERM $target 2>/dev/null || true

  for i in {1..30}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.1
  done

  echo "  $label still alive after SIGTERM; sending SIGKILL." >&2
  kill -KILL $target 2>/dev/null || true

  wait "$pid" 2>/dev/null || true
}

# ── Cleanup ────────────────────────────────────────────────────────────
cleanup() {
  local exit_status=$?
  # Disarm immediately so a second Ctrl-C during cleanup cannot
  # re-enter this function and race against itself.
  trap - EXIT INT TERM

  echo
  echo "==> Shutting down live SLAM stack..."

  # Stop the SLAM launch and every node it owns via the process group.
  if [[ -n "$LAUNCH_PID" ]]; then
    _kill_wait "$LAUNCH_PID" "$LAUNCH_IS_SESSION" "SLAM launch"
    wait "$LAUNCH_PID" 2>/dev/null || true
    LAUNCH_PID=''
  fi

  # Remove the per-run tmpfs cache only if this script created it.
  if [[ "$GRAPH_PCD_CACHE_DIR" == "$DEFAULT_GRAPH_CACHE" && \
        -d "$GRAPH_PCD_CACHE_DIR" ]]; then
    echo "  Removing tmpfs cache: $GRAPH_PCD_CACHE_DIR"
    rm -rf -- "$GRAPH_PCD_CACHE_DIR"
  fi

  # Announce any survivors — warning only, never blocks exit.
  echo "  Checking for stale nodes..."
  local survivors
  survivors=$(ros2 node list 2>/dev/null | grep -E \
    '^/(dlio_odom_node|dlio_map_node|dlio_odometry_to_tf|dlio_path_publisher\
|graph_based_slam|live_base_to_lidar_tf|live_base_to_imu_tf|rviz2)$' \
    || true)
  if [[ -n "$survivors" ]]; then
    echo "WARNING: the following nodes are still alive after shutdown:" >&2
    printf '  %s\n' $survivors >&2
    echo "  Kill them manually before the next launch:" >&2
    echo "  ros2 node list | grep -E 'dlio|graph|rviz'" >&2
  else
    echo "  All expected nodes are gone."
  fi

  echo "==> Shutdown complete."
  exit "$exit_status"
}
trap cleanup EXIT INT TERM

# ── Start ──────────────────────────────────────────────────────────────
echo "Starting flat live-real Husky DLIO + graph SLAM"
echo "  LiDAR:  $LIDAR_TOPIC  ->  DLIO (direct, no corrector)"
echo "  IMU:    $IMU_TOPIC"
echo "  DLIO:   deskew=$DLIO_DESKEW  ground=$DLIO_GROUND_CONSTRAINT"
echo "  TF:     publish_sensor_static_tf=$PUBLISH_SENSOR_STATIC_TF"
echo "  cache:  $GRAPH_PCD_CACHE_DIR"
echo "Keep the rover stationary for the first 3 s (IMU calibration)."
echo

launch_cmd=(
  ros2 launch "$LAUNCH_FILE"
  repo_root:="$REPO"
  slam_ws:="$SLAM_WS"
  param_file:="$PARAM_FILE"
  lidar_topic:="$LIDAR_TOPIC"
  imu_topic:="$IMU_TOPIC"
  odom_frame:="$ODOM_FRAME"
  base_frame:="$BASE_FRAME"
  lidar_frame:="$LIDAR_FRAME"
  imu_frame:="$IMU_FRAME"
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
  setsid "${launch_cmd[@]}" &
  LAUNCH_IS_SESSION=true
else
  "${launch_cmd[@]}" &
fi
LAUNCH_PID=$!

# ── Wait for sensors ───────────────────────────────────────────────────
echo "Waiting for sensor publishers and DLIO subscriptions..."
lidar_pubs=0; imu_pubs=0; lidar_subs=0

for _ in {1..80}; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: launch exited during startup." >&2
    exit 1
  fi

  lidar_pubs=$(ros2 topic info "$LIDAR_TOPIC" 2>/dev/null | \
    awk '/Publisher count:/    {print $3}' || true)
  imu_pubs=$(ros2 topic info "$IMU_TOPIC" 2>/dev/null | \
    awk '/Publisher count:/    {print $3}' || true)
  lidar_subs=$(ros2 topic info "$LIDAR_TOPIC" 2>/dev/null | \
    awk '/Subscription count:/ {print $3}' || true)

  if [[ ${lidar_pubs:-0} -ge 1 && \
        ${imu_pubs:-0}   -ge 1 && \
        ${lidar_subs:-0} -ge 1 ]]; then
    break
  fi
  sleep 0.5
done

if [[ ${lidar_pubs:-0} -lt 1 || ${imu_pubs:-0} -lt 1 ]]; then
  echo "ERROR: rover sensor publishers not found within 40 s." >&2
  echo "  $LIDAR_TOPIC publishers=${lidar_pubs:-0}" >&2
  echo "  $IMU_TOPIC   publishers=${imu_pubs:-0}" >&2
  exit 1
fi
if [[ ${lidar_subs:-0} -lt 1 ]]; then
  echo "ERROR: DLIO did not subscribe to $LIDAR_TOPIC within 40 s." >&2
  exit 1
fi

echo "Sensor chain ready. Waiting ${STARTUP_DIAGNOSTIC_DELAY}s for DLIO init..."
sleep "$STARTUP_DIAGNOSTIC_DELAY"

# ── Startup diagnostics ────────────────────────────────────────────────
echo
echo "================ LIVE STARTUP DIAGNOSTICS ================"

echo "--- Topic endpoints ---"
ros2 topic info "$LIDAR_TOPIC" 2>&1 || true
ros2 topic info "$IMU_TOPIC"   2>&1 || true
ros2 topic info /dlio/odometry 2>&1 || true
ros2 topic info /dlio/deskewed 2>&1 || true
ros2 topic info /modified_map  2>&1 || true

echo "--- Receive rates (short samples) ---"
timeout 4 ros2 topic hz "$LIDAR_TOPIC" --window 10  2>&1 || true
timeout 4 ros2 topic hz "$IMU_TOPIC"   --window 100 2>&1 || true

echo "--- DLIO output ---"
timeout 4 ros2 topic echo /dlio/odometry --once 2>&1 >/dev/null && \
  echo "OK: DLIO is publishing odometry." || \
  echo "WAITING: no DLIO odometry yet — check extrinsics/timing logs above."

echo "--- Labeled DLIO diagnostic ---"
timeout 5 python3 "$REPO/scripts/togo/dlio_diag_once.py" 2>&1 || \
  echo "WAITING: no labeled diagnostic message received within 5 s."

echo "--- Active nodes ---"
ros2 node list 2>&1 | grep -E \
  'dlio|graph|path_publisher|odometry_to_tf|rviz' || \
  echo "No expected nodes visible yet."

echo "=========================================================="
echo
echo "Live mapping running. Press Ctrl-C once for coordinated shutdown."

# ── Wait for launch to exit ────────────────────────────────────────────
set +e
wait "$LAUNCH_PID"
launch_status=$?
set -e
LAUNCH_PID=''

# Final map save on clean exit
if [[ $launch_status -eq 0 ]]; then
  if ros2 service list 2>/dev/null | grep -qx /map_save; then
    echo "Requesting final map save..."
    ros2 service call /map_save std_srvs/srv/Empty || true
  fi
fi

exit "$launch_status"
