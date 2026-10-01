#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

PERIOD_SEC="${1:-15}"

echo "Waiting for graph SLAM /map_save service..."
until ros2 service list | grep -qx '/map_save'; do
  sleep 1
done

echo "Publishing graph map every ${PERIOD_SEC}s by calling /map_save. Press Ctrl+C to stop."
while true; do
  date '+[%H:%M:%S] calling /map_save'
  ros2 service call /map_save std_srvs/srv/Empty || true
  sleep "$PERIOD_SEC"
done