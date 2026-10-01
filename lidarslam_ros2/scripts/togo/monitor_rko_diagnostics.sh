#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC="${1:-0}"
if [[ $# -gt 0 ]]; then
  shift
fi
SCRIPT_PATH="/ws/src/lidarslam_ros2/scripts/togo/rko_diagnostics_monitor.py"

if [[ ! -f "${SCRIPT_PATH}" ]]; then
  SCRIPT_PATH="/mnt/c/Users/Username/OneDrive/Desktop/husky/lidarslam_ros2/scripts/togo/rko_diagnostics_monitor.py"
fi

if ! [[ "$DURATION_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Usage: $0 [duration_seconds]" >&2
  echo "Use 0 or omit the argument to run until Ctrl-C." >&2
  exit 2
fi

python3 "${SCRIPT_PATH}" --duration "${DURATION_SEC}" "$@"
