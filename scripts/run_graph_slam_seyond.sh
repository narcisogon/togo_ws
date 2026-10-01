#!/usr/bin/env bash`nset -eo pipefail`n`nsource /opt/ros/jazzy/setup.bash`nset -u

OUT_DIR="${1:-/ws/src/lidarslam_ros2/output/husky_seyond_graph}"
mkdir -p "$OUT_DIR"

ros2 run graph_based_slam graph_based_slam_node \
  --ros-args \
  --params-file /ws/src/lidarslam_ros2/lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  -p use_sim_time:=true \
  -p use_odom_input:=true \
  -p global_frame_id:=map \
  -p map_save_dir:="$OUT_DIR" \
  -p submap_distance_threshold:=1.5 \
  -r odom_input:=/rko_lio/odometry \
  -r cloud_input:=/rko_lio/frame