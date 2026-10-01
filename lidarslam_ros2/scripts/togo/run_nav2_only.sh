#!/usr/bin/env bash
set -eo pipefail

# ── Workspace ──────────────────────────────────────────────────────────────────
SLAM_WS=${SLAM_WS:-/home/er4-user/slam_ws}

source /opt/ros/jazzy/setup.bash
source "$SLAM_WS/install/setup.bash"
set -u

# ── Config ─────────────────────────────────────────────────────────────────────
REPO="$SLAM_WS/src/lidarslam_ros2"
NAV_PKG="togo_navigation"
LAUNCH_FILE="$REPO/$NAV_PKG/launch/rover_nav2.launch.py"

USE_SIM_TIME=${USE_SIM_TIME:-true}
AUTOSTART=${AUTOSTART:-true}
RVIZ=${RVIZ:-false}
DEBUG_MAP=${DEBUG_MAP:-false}
USE_SLAM_MAP=${USE_SLAM_MAP:-true}
USE_PATCH_HAZARD_MAP=${USE_PATCH_HAZARD_MAP:-true}
REQUEST_INITIAL_MAP_SAVE=${REQUEST_INITIAL_MAP_SAVE:-true}
PARAMS_FILE=${PARAMS_FILE:-""}          # leave empty → launch default kicks in

SHUTDOWN_GRACE_SEC=${SHUTDOWN_GRACE_SEC:-5}

# ── PID tracking ───────────────────────────────────────────────────────────────
NAV_PID=''
_SHUTDOWN_CALLED=false

# ── Process-group kill (generic) ───────────────────────────────────────────────
kill_proc_group() {
  local pid=$1
  local label=$2

  [[ -z "$pid" ]] && return
  kill -0 "$pid" 2>/dev/null || return

  local pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]') || true

  if [[ -n "$pgid" && "$pgid" =~ ^[0-9]+$ ]]; then
    echo "[nav2] SIGTERM → process group -$pgid ($label)"
    kill -TERM -- "-$pgid" 2>/dev/null || true

    local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
    while kill -0 "$pid" 2>/dev/null; do
      if (( $(date +%s) >= deadline )); then
        echo "[nav2] Grace elapsed → SIGKILL group -$pgid ($label)"
        kill -KILL -- "-$pgid" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
  else
    echo "[nav2] WARNING: no pgid for $label (PID $pid), killing direct"
    kill -TERM "$pid" 2>/dev/null || true
    sleep "$SHUTDOWN_GRACE_SEC"
    kill -KILL "$pid" 2>/dev/null || true
  fi

  wait "$pid" 2>/dev/null || true
  echo "[nav2] $label stopped."
}

# ── Nav2 session kill with name-based fallback sweep ──────────────────────────
# Primary:  SIGTERM the entire setsid process group.
# Fallback: pkill -f on node binary names that are unique to this Nav2 stack —
#           none of these strings appear in standard system processes.
kill_nav2_session() {
  local pid=$1
  local label=$2

  [[ -z "$pid" ]] && return
  kill -0 "$pid" 2>/dev/null || return

  local pgid
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]') || true

  echo "[nav2] NAV_PID=$pid  PGID='$pgid'"

  if [[ -n "$pgid" && "$pgid" =~ ^[0-9]+$ ]]; then
    echo "[nav2] SIGTERM → process group -$pgid ($label)"
    kill -TERM -- "-$pgid" 2>/dev/null || true

    local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
    while kill -0 "$pid" 2>/dev/null; do
      if (( $(date +%s) >= deadline )); then
        echo "[nav2] Grace elapsed → SIGKILL group -$pgid ($label)"
        kill -KILL -- "-$pgid" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
  else
    echo "[nav2] WARNING: PGID parse failed (got: '$pgid'), killing by PID directly"
    kill -TERM "$pid" 2>/dev/null || true
    sleep "$SHUTDOWN_GRACE_SEC"
    kill -KILL "$pid" 2>/dev/null || true
  fi

  wait "$pid" 2>/dev/null || true
  echo "[nav2] $label stopped."

  # ── Name-based fallback — exact binary names unique to this Nav2 stack ────────
  local -a node_patterns=(
    "controller_server"
    "smoother_server"
    "planner_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
    "velocity_smoother"
    "lifecycle_manager"
    "slam_to_occupancy_grid"
    "hazard_patch_node"
    "global_costmap_composer"
    "goal_safety_relay"
    "local_hazard_grid"
    "occupancy_grid_to_points"
    "debug_map_publisher"
    "rviz2"
  )

  local any_survivors=false
  for pattern in "${node_patterns[@]}"; do
    if pgrep -f "$pattern" &>/dev/null; then
      echo "[nav2] WARNING: '$pattern' still alive — force killing"
      pkill -KILL -f "$pattern" 2>/dev/null || true
      any_survivors=true
    fi
  done
  [[ "$any_survivors" == "false" ]] && echo "[nav2] All nodes confirmed dead. ✅"
}

# ── Cleanup ────────────────────────────────────────────────────────────────────
cleanup() {
  if [[ "$_SHUTDOWN_CALLED" == "true" ]]; then
    return
  fi
  _SHUTDOWN_CALLED=true

  echo ""
  echo "[nav2] Cleaning up ..."
  kill_nav2_session "$NAV_PID" "ros2 launch (Nav2)"
  echo "[nav2] Shutdown complete."
}

trap 'cleanup; exit 130' SIGINT
trap 'cleanup; exit 143' SIGTERM
trap 'cleanup; exit 129' SIGHUP
trap 'cleanup'           EXIT

# ── Pre-flight check ───────────────────────────────────────────────────────────
if [[ ! -f "$LAUNCH_FILE" ]]; then
  echo "[nav2] ERROR: launch file not found: $LAUNCH_FILE" >&2
  exit 1
fi

# ── Build optional params_file arg ────────────────────────────────────────────
PARAMS_ARG=""
if [[ -n "$PARAMS_FILE" ]]; then
  if [[ ! -f "$PARAMS_FILE" ]]; then
    echo "[nav2] ERROR: params file not found: $PARAMS_FILE" >&2
    exit 1
  fi
  PARAMS_ARG="params_file:=$PARAMS_FILE"
fi

# ── Launch Nav2 in its own session ────────────────────────────────────────────
echo "[nav2] Launching rover_nav2 ..."
setsid ros2 launch "$LAUNCH_FILE" \
  use_sim_time:="$USE_SIM_TIME" \
  autostart:="$AUTOSTART" \
  rviz:="$RVIZ" \
  debug_map:="$DEBUG_MAP" \
  use_slam_map:="$USE_SLAM_MAP" \
  use_patch_hazard_map:="$USE_PATCH_HAZARD_MAP" \
  request_initial_map_save:="$REQUEST_INITIAL_MAP_SAVE" \
  ${PARAMS_ARG} &
NAV_PID=$!
echo "[nav2] Nav2 launch PID: $NAV_PID"

# ── Wait for lifecycle manager to become active ───────────────────────────────
echo "[nav2] Waiting for Nav2 lifecycle manager..."
for _ in {1..90}; do
  if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "[nav2] ERROR: Nav2 launch exited prematurely." >&2
    exit 1
  fi
  if ros2 node list 2>/dev/null | grep -q "lifecycle_manager_navigation"; then
    echo "[nav2] lifecycle_manager_navigation is up. ✅"
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -q "lifecycle_manager_navigation"; then
  echo "[nav2] WARNING: lifecycle_manager_navigation never appeared — stack may not be fully active." >&2
fi

echo ""
echo "[nav2] ✅ Nav2 stack is running."
echo "[nav2] 👉 Press Ctrl+C to shut down all nodes."

# ── Keep alive ────────────────────────────────────────────────────────────────
# Exit immediately if the launch process itself dies unexpectedly.
while true; do
  if ! kill -0 "$NAV_PID" 2>/dev/null; then
    echo "[nav2] Nav2 launch process exited unexpectedly." >&2
    exit 1
  fi
  sleep 1
done
