#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi
set -u

DURATION="${1:-6}"
RAW_CLOUD="${RAW_CLOUD:-/a300_0000/sensors/seyond_robin_w/scan/points}"
TIMED_CLOUD="${TIMED_CLOUD:-${RAW_CLOUD}}"
IMU_TOPIC="${IMU_TOPIC:-/a300_0000/sensors/seyond_robin_w/imu}"
DIAG_TOPIC="${DIAG_TOPIC:-/dlio/frontend_diagnostics}"
ADAPTER_DIAG_TOPIC="${ADAPTER_DIAG_TOPIC:-/seyond_cloud_time_adapter/diagnostics}"
VERBOSE="${VERBOSE:-false}"

echo "=== Seyond Sensor Reliability (${DURATION}s windows) ==="
echo "raw_cloud:   ${RAW_CLOUD}"
echo "dlio_cloud:  ${TIMED_CLOUD}"
echo "imu:         ${IMU_TOPIC}"
echo "diag:        ${DIAG_TOPIC}"
echo "adapter:     ${ADAPTER_DIAG_TOPIC}"
echo

echo "=== Time adapter heartbeat ==="
if ros2 topic list | grep -qx "${ADAPTER_DIAG_TOPIC}"; then
  timeout "${DURATION}" ros2 topic echo "${ADAPTER_DIAG_TOPIC}" --once || true
else
  echo "Standalone adapter not active; synthetic timing is integrated into DLIO."
fi
echo

echo "=== Topic availability ==="
ros2 topic list | grep -E "seyond_robin_w|dlio|modified_map|clock" || true
echo

echo "=== QoS / endpoints ==="
if [[ "${VERBOSE}" == "true" ]]; then
  ros2 topic info "${RAW_CLOUD}" -v || true
  echo
  ros2 topic info "${TIMED_CLOUD}" -v || true
  echo
  ros2 topic info "${IMU_TOPIC}" -v || true
else
  ros2 topic info "${RAW_CLOUD}" || true
  ros2 topic info "${TIMED_CLOUD}" || true
  ros2 topic info "${IMU_TOPIC}" || true
  echo "Set VERBOSE=true for full endpoint QoS."
fi
echo

echo "=== Rates ==="
timeout "${DURATION}" ros2 topic hz /clock || true
timeout "${DURATION}" ros2 topic hz "${RAW_CLOUD}" || true
timeout "${DURATION}" ros2 topic hz "${TIMED_CLOUD}" || true
timeout "${DURATION}" ros2 topic hz "${IMU_TOPIC}" || true
echo

echo "=== Cloud geometry and fields ==="
echo "raw height/width:"
ros2 topic echo "${RAW_CLOUD}" --once --field height || true
ros2 topic echo "${RAW_CLOUD}" --once --field width || true
echo "raw fields:"
ros2 topic echo "${RAW_CLOUD}" --once --field fields || true
echo
echo "timed height/width:"
ros2 topic echo "${TIMED_CLOUD}" --once --field height || true
ros2 topic echo "${TIMED_CLOUD}" --once --field width || true
echo "timed fields:"
ros2 topic echo "${TIMED_CLOUD}" --once --field fields || true
echo

echo "=== Header/frame sanity ==="
ros2 topic echo "${RAW_CLOUD}" --once --field header || true
ros2 topic echo "${TIMED_CLOUD}" --once --field header || true
ros2 topic echo "${IMU_TOPIC}" --once --field header || true
echo

echo "=== IMU sample sanity ==="
ros2 topic echo "${IMU_TOPIC}" --once --field angular_velocity || true
ros2 topic echo "${IMU_TOPIC}" --once --field linear_acceleration || true
echo

echo "=== Static extrinsics ==="
timeout 4 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_lidar_frame || true
timeout 4 ros2 run tf2_ros tf2_echo base_link seyond_robin_w_imu_frame || true
timeout 4 ros2 run tf2_ros tf2_echo seyond_robin_w_lidar_frame seyond_robin_w_imu_frame || true
echo

echo "=== DLIO timing/gate snapshot ==="
echo "Fields of interest:"
echo "  [16]=deskew_status [17]=deskew_size [20]=rejected [21]=bad_streak"
echo "  [30]=angular_rate [32]=imu_age_ms [39]=imu_covers_start [40]=imu_covers_end"
echo "  [41]=latest_imu_minus_lidar_ms [42]=deskew_time_buckets [44]=scan_duration_ms"
echo "  [55]=pipeline_stage [56]=callbacks_started [57]=callbacks_completed"
echo "Pipeline stages: 0=waiting 1=entry 2=init 3=convert 4=deskew 5=metrics"
echo "                 6=input 7=registration 8=keyframes 9=submap 10=publish"
if [[ -f /ws/src/lidarslam_ros2/scripts/togo/dlio_diag_once.py ]]; then
  timeout "${DURATION}" python3 /ws/src/lidarslam_ros2/scripts/togo/dlio_diag_once.py "${DIAG_TOPIC}" || true
else
  ros2 topic echo "${DIAG_TOPIC}" --once || true
fi
