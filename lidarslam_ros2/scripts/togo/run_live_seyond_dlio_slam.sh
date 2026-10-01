#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/er4-user/slam_ws/install/setup.bash

set -u

# ── Launch / param files ────────────────────────────────────────────────────────
LAUNCH_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py
COMBINED_PARAM_FILE=${COMBINED_PARAM_FILE:-/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml}
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-$COMBINED_PARAM_FILE}
DLIO_PARAM_FILE=${DLIO_PARAM_FILE:-$COMBINED_PARAM_FILE}

# ── Sensor wiring ───────────────────────────────────────────────────────────────
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

SHUTDOWN_GRACE_SEC=${SHUTDOWN_GRACE_SEC:-5}

# ── Tracked PIDs ────────────────────────────────────────────────────────────────
LAUNCH_PID=""
_SHUTDOWN_CALLED=false

# ── Cleanup ─────────────────────────────────────────────────────────────────────
cleanup() {
  # Guard against being called twice (EXIT fires after signal handlers)
  if [[ "$_SHUTDOWN_CALLED" == "true" ]]; then
    return
  fi
  _SHUTDOWN_CALLED=true

  echo ""
  echo "[slam_launch] Shutting down ..."

  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then

    # Kill the entire process group -- this reaches every node ros2
    # launch spawned, not just the launch process itself
    local pgid
    pgid=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')

    if [[ -n "$pgid" && "$pgid" != "0" ]]; then
      echo "[slam_launch] Sending SIGTERM to process group $pgid ..."
      kill -- "-$pgid" 2>/dev/null || true

      # Wait up to SHUTDOWN_GRACE_SEC for clean exit
      local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
      while kill -0 "$LAUNCH_PID" 2>/dev/null; do
        if (( $(date +%s) >= deadline )); then
          echo "[slam_launch] Grace period elapsed -- sending SIGKILL to group $pgid"
          kill -9 -- "-$pgid" 2>/dev/null || true
          break
        fi
        sleep 0.2
      done
    else
      # Fallback: can't get pgid, kill direct PID only
      echo "[slam_launch] Warning: could not get pgid, killing PID $LAUNCH_PID only"
      kill "$LAUNCH_PID" 2>/dev/null || true
      sleep "$SHUTDOWN_GRACE_SEC"
      kill -9 "$LAUNCH_PID" 2>/dev/null || true
    fi
  fi

  echo "[slam_launch] Done."
}

# Register on every exit path
trap 'cleanup; exit 130' SIGINT
trap 'cleanup; exit 143' SIGTERM
trap 'cleanup; exit 129' SIGHUP
trap 'cleanup'           EXIT

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

# ── Launch in background so we own the PID ─────────────────────────────────────
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
  rviz:="$SLAM_RVIZ" &

LAUNCH_PID=$!
echo "[slam_launch] ros2 launch PID: $LAUNCH_PID"

# ── Wait -- signals will interrupt this and trigger the trap ───────────────────
wait "$LAUNCH_PID"
LAUNCH_EXIT=$?
echo "[slam_launch] ros2 launch exited with code $LAUNCH_EXIT"
exit $LAUNCH_EXIT
