#!/usr/bin/env bash
set -eo pipefail


SLAM_WS=${SLAM_WS:-/home/er4-user/slam_ws}

source /opt/ros/jazzy/setup.bash
source "$SLAM_WS/install/setup.bash"
set -u

# ── Config ──────────────────────────────────────────────────────────────────────
BAG_PATH=${BAG_PATH:-/home/er4-user/slam_ws/src/lidarslam_ros2/bags/rosbag2_2026_07_27-21_54_36}
PLAYBACK_RATE=${PLAYBACK_RATE:-1.5}
STARTUP_DIAGNOSTIC_DELAY=${STARTUP_DIAGNOSTIC_DELAY:-15}
REPO="$SLAM_WS/src/lidarslam_ros2"
LAUNCH_FILE="$REPO/lidarslam/launch/seyond_dlio_replay.launch.py"
SLAM_PARAM_FILE=${SLAM_PARAM_FILE:-$REPO/lidarslam/param/seyond_dlio_graph.yaml}
DLIO_PARAM_FILE=${DLIO_PARAM_FILE:-$SLAM_PARAM_FILE}
DLIO_OVERRIDE_PARAM_FILE=${DLIO_OVERRIDE_PARAM_FILE:-$REPO/lidarslam/param/seyond_dlio_replay.yaml}
DLIO_DESKEW=${DLIO_DESKEW:-true}
SYNTHETIC_TIMING=${SYNTHETIC_TIMING:-false}
SLAM_RVIZ=${SLAM_RVIZ:-true}
SHUTDOWN_GRACE_SEC=${SHUTDOWN_GRACE_SEC:-5}

# ── Bag topics — space-separated list of topics to replay ───────────────────────
# Override from the command line:
#   BAG_TOPICS="/iv_points /husky/sensors/imu_0/data /tf /tf_static" ./replay.sh
# To replay ALL topics, set to empty string:
#   BAG_TOPICS="" ./replay.sh
BAG_TOPICS=${BAG_TOPICS:-"/iv_points /husky/sensors/imu_0/data /fixposition/gnss1"}

# ── Pre-flight checks ───────────────────────────────────────────────────────────
for required in "$BAG_PATH" "$LAUNCH_FILE" "$SLAM_PARAM_FILE" "$DLIO_PARAM_FILE" "$DLIO_OVERRIDE_PARAM_FILE"; do
  if [[ ! -e "$required" ]]; then
    echo "[replay] Missing: $required" >&2
    exit 1
  fi
done

# ── PID tracking ────────────────────────────────────────────────────────────────
SLAM_PID=''
PLAY_PID=''
WATCHDOG_PID=''
_SHUTDOWN_CALLED=false

# ── Process group kill — used for bag play ───────────────────────────────────────
kill_proc_group() {
  local pid=$1
  local label=$2

  [[ -z "$pid" ]] && return
  kill -0 "$pid" 2>/dev/null || return

  local pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]') || true

  if [[ -n "$pgid" && "$pgid" =~ ^[0-9]+$ ]]; then
    echo "[replay] SIGTERM → process group $pgid ($label)"
    kill -TERM -- "-$pgid" 2>/dev/null || true

    local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
    while kill -0 "$pid" 2>/dev/null; do
      if (( $(date +%s) >= deadline )); then
        echo "[replay] Grace elapsed → SIGKILL group $pgid ($label)"
        kill -KILL -- "-$pgid" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
  else
    echo "[replay] Warning: no pgid for $label (PID $pid), killing direct"
    kill -TERM "$pid" 2>/dev/null || true
    sleep "$SHUTDOWN_GRACE_SEC"
    kill -KILL "$pid" 2>/dev/null || true
  fi

  wait "$pid" 2>/dev/null || true
  echo "[replay] $label stopped."
}

# ── Session/group kill for SLAM ──────────────────────────────────────────────────
# Primary:  kill -TERM -- "-$pgid"  targets only the process group this script
#           created via setsid — no other processes are touched.
# Fallback: pkill -KILL -f <name>   matches only specific ROS node binary names
#           (dlio_odom_node, dlio_map_node, etc.) — will not match random system
#           processes since these names are unique to this SLAM stack.
kill_slam_session() {
  local pid=$1
  local label=$2

  [[ -z "$pid" ]] && return
  kill -0 "$pid" 2>/dev/null || return

  # Strip ALL whitespace (tabs + spaces) — plain tr -d ' ' misses tabs in Docker
  local pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]') || true

  echo "[replay] SLAM_PID=$pid  PGID='$pgid'"

  if [[ -n "$pgid" && "$pgid" =~ ^[0-9]+$ ]]; then
    echo "[replay] SIGTERM → process group -$pgid ($label)"
    # Targets ONLY the process group spawned by setsid — safe, no random kills
    kill -TERM -- "-$pgid" 2>/dev/null || true

    local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
    while kill -0 "$pid" 2>/dev/null; do
      if (( $(date +%s) >= deadline )); then
        echo "[replay] Grace elapsed → SIGKILL group -$pgid ($label)"
        kill -KILL -- "-$pgid" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
  else
    echo "[replay] WARNING: PGID parse failed (got: '$pgid'), killing by PID directly"
    kill -TERM "$pid" 2>/dev/null || true
    sleep "$SHUTDOWN_GRACE_SEC"
    kill -KILL "$pid" 2>/dev/null || true
  fi

  wait "$pid" 2>/dev/null || true
  echo "[replay] $label stopped."

  # ── Name-based fallback — only matches these exact SLAM stack binary names ─────
  # These strings (dlio_odom_node, dlio_map_node, etc.) do not appear in any
  # standard system process — safe to pkill -f on.
  local -a node_patterns=("dlio_odom_node" "dlio_map_node" "lidarslam" "graph_based_slam" "rviz2")
  local any_survivors=false
  for pattern in "${node_patterns[@]}"; do
    if pgrep -f "$pattern" &>/dev/null; then
      echo "[replay] WARNING: '$pattern' still alive — force killing"
      pkill -KILL -f "$pattern" 2>/dev/null || true
      any_survivors=true
    fi
  done
  [[ "$any_survivors" == "false" ]] && echo "[replay] All nodes confirmed dead. ✅"
}

cleanup() {
  if [[ "$_SHUTDOWN_CALLED" == "true" ]]; then
    return
  fi
  _SHUTDOWN_CALLED=true
  echo ""
  echo "[replay] Cleaning up ..."

  if [[ -n "$WATCHDOG_PID" ]] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    pkill -P "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
  fi

  kill_proc_group   "$PLAY_PID" "ros2 bag play"
  kill_slam_session "$SLAM_PID" "ros2 launch (SLAM)"

  echo "[replay] Shutdown complete."
}

trap 'cleanup; exit 130' SIGINT
trap 'cleanup; exit 143' SIGTERM
trap 'cleanup; exit 129' SIGHUP
trap 'cleanup'           EXIT

# ── Launch SLAM in its own session ──────────────────────────────────────────────
setsid ros2 launch "$LAUNCH_FILE" \
  source_launch:="$REPO/lidarslam/launch/seyond_dlio_slam.launch.py" \
  repo_root:="$REPO" \
  slam_param_file:="$SLAM_PARAM_FILE" \
  dlio_param_file:="$DLIO_PARAM_FILE" \
  dlio_override_param_file:="$DLIO_OVERRIDE_PARAM_FILE" \
  deskew:="$DLIO_DESKEW" \
  synthetic_timing:="$SYNTHETIC_TIMING" \
  rviz:="$SLAM_RVIZ" &
SLAM_PID=$!
echo "[replay] SLAM launch PID: $SLAM_PID"

# ── Wait for DLIO subscriptions ─────────────────────────────────────────────────
echo "[replay] Waiting for DLIO sensor subscriptions..."
lidar_subs=0
imu_subs=0
for _ in {1..60}; do
  if ! kill -0 "$SLAM_PID" 2>/dev/null; then
    echo "[replay] SLAM launch exited before subscriptions became ready." >&2
    exit 1
  fi
  lidar_subs=$(ros2 topic info /iv_points 2>/dev/null | \
    awk '/Subscription count:/ {print $3}' || true)
  imu_subs=$(ros2 topic info /husky/sensors/imu_0/data 2>/dev/null | \
    awk '/Subscription count:/ {print $3}' || true)
  if [[ ${lidar_subs:-0} -ge 1 && ${imu_subs:-0} -ge 1 ]]; then
    echo "[replay] DLIO subscriptions ready (lidar=${lidar_subs} imu=${imu_subs})"
    break
  fi
  sleep 0.5
done

if [[ ${lidar_subs:-0} -lt 1 || ${imu_subs:-0} -lt 1 ]]; then
  echo "[replay] Timed out waiting for DLIO subscriptions (lidar=${lidar_subs:-0} imu=${imu_subs:-0})" >&2
  exit 1
fi

# ── Start bag replay ─────────────────────────────────────────────────────────────
echo "[replay] Starting bag at ${PLAYBACK_RATE}x ..."
echo "[replay] Topics: ${BAG_TOPICS:-"(all)"}"

# Build --topics flag only if BAG_TOPICS is non-empty
TOPICS_ARG=""
if [[ -n "$BAG_TOPICS" ]]; then
  TOPICS_ARG="--topics $BAG_TOPICS"
fi

# shellcheck disable=SC2086  # word splitting intentional for TOPICS_ARG
setsid ros2 bag play "$BAG_PATH" \
  --clock \
  --rate "$PLAYBACK_RATE" \
  $TOPICS_ARG &
PLAY_PID=$!
echo "[replay] Bag play PID: $PLAY_PID"

# ── Watchdog diagnostics subshell ────────────────────────────────────────────────
(
  sleep "$STARTUP_DIAGNOSTIC_DELAY"
  kill -0 "$PLAY_PID" 2>/dev/null || exit 0
  print_replay_diagnostics
) &
WATCHDOG_PID=$!

# ── Wait for bag to finish ───────────────────────────────────────────────────────
set +e
wait "$PLAY_PID"
PLAY_STATUS=$?
set -e
PLAY_PID=''

if [[ -n "$WATCHDOG_PID" ]] && kill -0 "$WATCHDOG_PID" 2>/dev/null; then
  kill "$WATCHDOG_PID" 2>/dev/null || true
  pkill -P "$WATCHDOG_PID" 2>/dev/null || true
  wait "$WATCHDOG_PID" 2>/dev/null || true
fi
WATCHDOG_PID=''

if [[ $PLAY_STATUS -ne 0 ]]; then
  echo "[replay] Bag exited with status $PLAY_STATUS" >&2
  print_replay_diagnostics
  exit "$PLAY_STATUS"
fi

echo "[replay] Bag complete."

# ── Save map immediately after bag finishes ──────────────────────────────────────
if ros2 service list 2>/dev/null | grep -qx /map_save; then
  echo "[replay] Saving map..."
  ros2 service call /map_save std_srvs/srv/Empty || true
fi

echo ""
echo "[replay] ✅ Bag finished. SLAM nodes are still running."
echo "[replay] 👉 Press Ctrl+C when you are ready to shut down all nodes."

while true; do sleep 1; done
