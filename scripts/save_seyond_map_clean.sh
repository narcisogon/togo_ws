#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

echo "Calling graph SLAM map_save service..."
ros2 service call /map_save std_srvs/srv/Empty
sleep 1

echo "\nRecent graph SLAM outputs:"
find /ws/src/lidarslam_ros2/output -maxdepth 4 \
  \( -name 'map.pcd' -o -name 'pose_graph.g2o' -o -name 'pointcloud_map_metadata.yaml' -o -name 'map_projector_info.yaml' \) \
  -print