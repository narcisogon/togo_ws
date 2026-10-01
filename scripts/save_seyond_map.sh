#!/usr/bin/env bash`nset -eo pipefail`n`nsource /opt/ros/jazzy/setup.bash`nset -u
ros2 service call /map_save std_srvs/srv/Empty
sleep 1
find /ws/src/lidarslam_ros2/output -maxdepth 4 \( -name 'map.pcd' -o -name 'pose_graph.g2o' -o -name 'pointcloud_map_metadata.yaml' -o -name 'map_projector_info.yaml' \) -print