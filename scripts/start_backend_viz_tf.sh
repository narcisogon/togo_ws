#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

echo "Publishing identity TF map -> odom for RViz backend/frontend visualization."
echo "Use RViz Fixed Frame: map"
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id map \
  --child-frame-id odom