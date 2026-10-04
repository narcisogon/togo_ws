#!/usr/bin/env bash
set -eo pipefail

SLAM_WS=${SLAM_WS:-/home/er4-user/slam_ws}
source /opt/ros/jazzy/setup.bash
source "$SLAM_WS/install/setup.bash"
set -u

REPO="$SLAM_WS/src/lidarslam_ros2"
LAUNCH_FILE="$REPO/togo_navigation/launch/rover_nav2.launch.py"
PARAMS_FILE=${PARAMS_FILE:-"$REPO/togo_navigation/config/nav2_slam_params.yaml"}
USE_SIM_TIME=${USE_SIM_TIME:-true}
AUTOSTART=${AUTOSTART:-true}
RVIZ=${RVIZ:-false}
SHUTDOWN_GRACE_SEC=${SHUTDOWN_GRACE_SEC:-5}
NAV_PID=""
_SHUTDOWN_CALLED=false

cleanup() {
  [[ "$_SHUTDOWN_CALLED" == "true" ]] && return
  _SHUTDOWN_CALLED=true
  if [[ -n "$NAV_PID" ]] && kill -0 -- "-$NAV_PID" 2>/dev/null; then
    # setsid makes this launch PID its own process-group leader. Never sweep
    # generic node names: that used to kill RViz/SLAM from the other terminal.
    kill -TERM -- "-$NAV_PID" 2>/dev/null || true
    local deadline=$(( $(date +%s) + SHUTDOWN_GRACE_SEC ))
    while kill -0 -- "-$NAV_PID" 2>/dev/null; do
      if (( $(date +%s) >= deadline )); then
        kill -KILL -- "-$NAV_PID" 2>/dev/null || true
        break
      fi
      sleep 0.2
    done
    wait "$NAV_PID" 2>/dev/null || true
  fi
}
trap 'cleanup; exit 130' SIGINT
trap 'cleanup; exit 143' SIGTERM
trap 'cleanup; exit 129' SIGHUP
trap cleanup EXIT

[[ -f "$LAUNCH_FILE" ]] || { echo "Missing launch: $LAUNCH_FILE" >&2; exit 1; }
[[ -f "$PARAMS_FILE" ]] || { echo "Missing params: $PARAMS_FILE" >&2; exit 1; }

echo "[nav2] Backend /map -> planner/controller -> smoother -> freshness relay -> rover"
echo "[nav2] Hazard settings live under graph_based_slam in seyond_dlio_graph.yaml."
setsid ros2 launch "$LAUNCH_FILE" \
  params_file:="$PARAMS_FILE" \
  use_sim_time:="$USE_SIM_TIME" \
  autostart:="$AUTOSTART" \
  rviz:="$RVIZ" &
NAV_PID=$!
wait "$NAV_PID"
