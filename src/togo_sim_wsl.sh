
#!/usr/bin/env bash

# Native WSL / WSLg launcher for the Togo lunar simulation.
#
# Optional environment overrides:
#   TOGO_WSL_INSTALL   ROS install prefix
#                      (default: ~/togo_ws_native_clean/install)
#
#   TOGO_WORLD         Gazebo world file
#                      (default: lunar_surface.world)
#
#   TOGO_ROBOT_X/Y/Z   Initial robot pose
#                      (defaults: -2.0, 3.0, 2.0)
#
#   TOGO_RVIZ          Launch RViz
#                      (default: true)
#
#   TOGO_GZ_GUI        Launch the Gazebo GUI
#                      (default: true)
#
#   TOGO_USE_SIM_TIME  Make ROS nodes use Gazebo simulation time
#                      (default: true)

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

TOGO_WSL_INSTALL="${TOGO_WSL_INSTALL:-${HOME}/togo_ws_native_clean/install}"
TOGO_SETUP="${TOGO_WSL_INSTALL}/setup.bash"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

WORLD="${TOGO_WORLD:-lunar_surface.world}"

ROBOT_X="${TOGO_ROBOT_X:--2.0}"
ROBOT_Y="${TOGO_ROBOT_Y:-3.0}"
ROBOT_Z="${TOGO_ROBOT_Z:-2.0}"

START_RVIZ="${TOGO_RVIZ:-true}"
START_GZ_GUI="${TOGO_GZ_GUI:-true}"

# Use Gazebo's simulation clock by default.
USE_SIM_TIME="${TOGO_USE_SIM_TIME:-true}"

SERVER_PID=""
GUI_PID=""

# ------------------------------------------------------------
# Validate ROS installation
# ------------------------------------------------------------

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${TOGO_SETUP}" ]]; then
  echo "ERROR: Native Togo install not found: ${TOGO_SETUP}" >&2
  echo "Build the native workspace in ~/togo_ws_native_clean first." >&2
  exit 1
fi

# ------------------------------------------------------------
# Prevent accidentally starting multiple simulations
# ------------------------------------------------------------

if pgrep -f '[r]os2 launch togo_gz sim_gz.launch.py' >/dev/null 2>&1; then
  echo "ERROR: A Togo Gazebo simulation is already running." >&2
  echo "Stop it with Ctrl+C in its original terminal before starting another." >&2
  exit 1
fi

# ------------------------------------------------------------
# Source ROS environments
# ------------------------------------------------------------

# Source only the native WSL installation.
# Do not source the Docker workspace here.
source "${ROS_SETUP}"
source "${TOGO_SETUP}"

# ROS-generated setup scripts may inspect unset tracing variables,
# so enable strict unset-variable checking only after both
# environments are loaded.
set -u

# ------------------------------------------------------------
# WSL / WSLg graphics
# ------------------------------------------------------------

# WSLg uses Mesa's D3D12 renderer to access the Windows GPU.
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

# ------------------------------------------------------------
# ROS 2 DDS configuration
# ------------------------------------------------------------

# Match the Docker development service so native Gazebo and
# containerized SLAM participate in the same ROS 2 DDS domain.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-29}"

export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

DDS_CONFIG="${REPO_ROOT}/config/cyclonedds.xml"

if [[ -f "${DDS_CONFIG}" ]]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${DDS_CONFIG}}"
else
  echo "WARNING: CycloneDDS config not found: ${DDS_CONFIG}" >&2
fi

# ------------------------------------------------------------
# Gazebo resource paths
# ------------------------------------------------------------

TOGO_GZ_SHARE="${TOGO_WSL_INSTALL}/togo_gz/share/togo_gz"

export GZ_SIM_RESOURCE_PATH="${TOGO_GZ_SHARE}:/opt/ros/${ROS_DISTRO}/share${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

cleanup() {
  local exit_code=$?

  trap - EXIT INT TERM

  echo
  echo "Stopping Togo simulation..."

  if [[ -n "${GUI_PID}" ]] && kill -0 "${GUI_PID}" 2>/dev/null; then
    kill -INT -- "-${GUI_PID}" 2>/dev/null \
      || kill -INT "${GUI_PID}" 2>/dev/null \
      || true
  fi

  if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
    kill -INT -- "-${SERVER_PID}" 2>/dev/null \
      || kill -INT "${SERVER_PID}" 2>/dev/null \
      || true
  fi

  [[ -z "${GUI_PID}" ]] \
    || wait "${GUI_PID}" 2>/dev/null \
    || true

  [[ -z "${SERVER_PID}" ]] \
    || wait "${SERVER_PID}" 2>/dev/null \
    || true

  echo "Togo simulation stopped."

  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

# ------------------------------------------------------------
# Startup information
# ------------------------------------------------------------

echo "=========================================="
echo "Starting Togo lunar simulation in WSL"
echo "=========================================="
echo "World:        ${WORLD}"
echo "Robot pose:   x=${ROBOT_X}, y=${ROBOT_Y}, z=${ROBOT_Z}"
echo "RViz:         ${START_RVIZ}"
echo "Gazebo GUI:   ${START_GZ_GUI}"
echo "Use sim time: ${USE_SIM_TIME}"
echo "ROS domain:   ${ROS_DOMAIN_ID}"
echo "RMW:          ${RMW_IMPLEMENTATION}"
echo "=========================================="

# ------------------------------------------------------------
# Check graphics renderer
# ------------------------------------------------------------

if command -v glxinfo >/dev/null 2>&1; then
  RENDERER="$(
    glxinfo -B 2>/dev/null \
      | sed -n 's/^OpenGL renderer string: //p' \
      | head -n 1
  )"

  if [[ -n "${RENDERER}" ]]; then
    echo "OpenGL renderer: ${RENDERER}"

    if [[ "${RENDERER}" == *llvmpipe* ]]; then
      echo "WARNING: WSL is using software rendering instead of the GPU." >&2
    fi
  fi
fi

# ------------------------------------------------------------
# Launch Gazebo server + ROS nodes + RViz
# ------------------------------------------------------------

echo
echo "[1/2] Starting Gazebo server and ROS nodes..."

setsid ros2 launch togo_gz sim_gz.launch.py \
  world:="${WORLD}" \
  robot_x:="${ROBOT_X}" \
  robot_y:="${ROBOT_Y}" \
  robot_z:="${ROBOT_Z}" \
  rviz:="${START_RVIZ}" \
  use_sim_time:="${USE_SIM_TIME}" &

SERVER_PID=$!

# ------------------------------------------------------------
# Give Gazebo server time to initialize
# ------------------------------------------------------------

for _ in {1..16}; do
  if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
    wait "${SERVER_PID}"
    exit $?
  fi

  sleep 0.5
done

# ------------------------------------------------------------
# Launch Gazebo GUI
# ------------------------------------------------------------

if [[ "${START_GZ_GUI}" == "true" ]]; then

  echo "[2/2] Starting Gazebo GUI through WSLg..."

  setsid gz sim -g &

  GUI_PID=$!

else

  echo "[2/2] Gazebo GUI disabled (TOGO_GZ_GUI=${START_GZ_GUI})."

fi

# ------------------------------------------------------------
# Running
# ------------------------------------------------------------

echo
echo "=========================================="
echo "Simulation running. Press Ctrl+C to exit."
echo "=========================================="
echo "LiDAR:       /husky/sensors/seyond/points"
echo "Clock:       /clock"
echo "Sim time:    ${USE_SIM_TIME}"
echo "=========================================="
echo
echo "Useful checks:"
echo
echo "  ros2 topic hz /clock"
echo
echo "  ros2 param get /rviz2 use_sim_time"
echo
echo "  ros2 topic info /clock -v"
echo
echo "=========================================="

wait "${SERVER_PID}"
```
