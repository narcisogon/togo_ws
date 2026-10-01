#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u

OUT_DIR="${1:-/ws/src/lidarslam_ros2/output/husky_seyond_graph}"
mkdir -p "$OUT_DIR"

cleanup() {
  if [[ -n "${CONVERTER_PID:-}" ]]; then
    kill "$CONVERTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 /scripts/add_intensity_to_cloud.py \
  --ros-args \
  -p input_topic:=/rko_lio/frame \
  -p output_topic:=/rko_lio/frame_xyzi \
  -p intensity:=1.0 &
CONVERTER_PID=$!

echo "Waiting for converted XYZI cloud: /rko_lio/frame_xyzi"
for _ in $(seq 1 30); do
  if timeout 3 ros2 topic echo /rko_lio/frame_xyzi --once >/tmp/frame_xyzi_wait.log 2>&1; then
    echo "  /rko_lio/frame_xyzi is publishing."
    break
  fi
  sleep 0.5
done

ros2 run graph_based_slam graph_based_slam_node \
  --ros-args \
  --params-file /ws/src/lidarslam_ros2/lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  -p use_sim_time:=true \
  -p use_odom_input:=true \
  -p global_frame_id:=map \
  -p map_save_dir:="$OUT_DIR" \
  -p submap_distance_threshold:=0.5 \
  -p debug_flag:=true \
  -r odom_input:=/rko_lio/odometry \
  -r cloud_input:=/rko_lio/frame_xyzi