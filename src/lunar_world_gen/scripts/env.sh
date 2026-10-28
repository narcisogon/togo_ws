# shellcheck shell=bash
# Same ROS 2 / DDS / GPU / Gazebo environment as scripts/run_apollo16_sim.sh, for a second terminal:
#
#   source <repo>/src/lunar_world_gen/scripts/env.sh
#   ros2 control list_controllers                # both 'active' once the sim printed [apollo16_sim] READY
#   ros2 topic hz /husky/sensors/seyond/points
#   gz model -m togo -p                          # ground-truth pose, one shot (needs the sim's GZ_PARTITION)
#
# Sets ROS 2 (/opt/ros/$ROS_DISTRO) + the native TOGO install (TOGO_WSL_INSTALL, default
# ~/togo_ws_native_clean/install), ROS_DOMAIN_ID (29), RMW_IMPLEMENTATION (rmw_cyclonedds_cpp),
# ROS_AUTOMATIC_DISCOVERY_RANGE (as togo_sim_wsl.sh: LOCALHOST only if still unset after sourcing ROS, whose
# ros_environment hook sets SUBNET; cyclonedds.xml pins DDS to loopback either way),
# CYCLONEDDS_URI (file://<repo>/config/cyclonedds.xml),
# GZ_SIM_RESOURCE_PATH and the WSLg GPU variables (GALLIUM_DRIVER=d3d12, MESA_D3D12_DEFAULT_ADAPTER_NAME,
# MESA_EXTENSION_OVERRIDE (memory-leak workaround), /usr/lib/wsl/lib, DISPLAY, WAYLAND_DISPLAY, XDG_RUNTIME_DIR). Values already set in the shell win, as in
# src/togo_sim_wsl.sh (e.g. ROS_DOMAIN_ID=30 source env.sh). GZ_PARTITION is not touched: use the sim's.
# Safe to source more than once. Option: --quiet. Returns non-zero (never exits the shell) on errors.

_apollo16_env() {
  local quiet=0 a
  for a in "$@"; do [[ "${a}" == "--quiet" ]] && quiet=1; done
  local here pkg repo
  here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  pkg="$(cd -- "${here}/.." && pwd)"                     # src/lunar_world_gen
  repo="$(cd -- "${pkg}/../.." && pwd)"                   # repository root
  local distro="${ROS_DISTRO:-jazzy}"
  local ros_setup="/opt/ros/${distro}/setup.bash"
  local install="${TOGO_WSL_INSTALL:-${HOME}/togo_ws_native_clean/install}"
  if [[ ! -f "${ros_setup}" ]]; then
    echo "ERROR: ROS setup not found: ${ros_setup} (ROS 2 ${distro} is required)" >&2
    return 1
  fi
  if [[ ! -f "${install}/setup.bash" ]]; then
    echo "ERROR: native TOGO install not found: ${install}/setup.bash" >&2
    echo "       Build the native workspace ~/togo_ws_native_clean first, or set TOGO_WSL_INSTALL." >&2
    return 1
  fi
  # ROS-generated setup scripts read unset variables: no 'set -u' while sourcing them
  local had_u=0
  [[ $- == *u* ]] && had_u=1
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${install}/setup.bash"
  (( had_u )) && set -u

  # WSL / WSLg graphics: GPU through Mesa's D3D12 driver (without GALLIUM_DRIVER Mesa may fall back to llvmpipe)
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  # Mesa's d3d12 driver leaks one ID3D12CommandSignature per indirect draw (d3d12_get_cmd_signature searches its
  # cache with &key; Mesa 25.2.8 ... 26.2.5) and Ogre-Next draws everything with glMultiDrawElementsIndirect: 2.7-10 MB/s
  # per gz process. Without the extension Ogre-Next uses plain draws, same image (docs/impl/leak_diagnosis.md).
  if [[ "${GALLIUM_DRIVER}" == "d3d12" && " ${MESA_EXTENSION_OVERRIDE:-} " != *" -GL_ARB_multi_draw_indirect "* ]]; then
    export MESA_EXTENSION_OVERRIDE="${MESA_EXTENSION_OVERRIDE:+${MESA_EXTENSION_OVERRIDE} }-GL_ARB_multi_draw_indirect"
  fi
  if [[ -d /usr/lib/wsl/lib && ":${LD_LIBRARY_PATH:-}:" != *":/usr/lib/wsl/lib:"* ]]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  export DISPLAY="${DISPLAY:-:0}"
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"

  # ROS 2 DDS: the same domain and CycloneDDS config as src/togo_sim_wsl.sh and the SLAM container
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-29}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  local dds="${repo}/config/cyclonedds.xml"
  if [[ -f "${dds}" ]]; then
    export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${dds}}"
  else
    echo "WARNING: CycloneDDS config not found: ${dds}" >&2
  fi

  # Gazebo resource paths (as togo_sim_wsl.sh), each added once
  local p
  for p in "/opt/ros/${distro}/share" "${install}/togo_gz/share/togo_gz"; do
    if [[ ":${GZ_SIM_RESOURCE_PATH:-}:" != *":${p}:"* ]]; then
      export GZ_SIM_RESOURCE_PATH="${p}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
    fi
  done
  export TOGO_WSL_INSTALL="${install}"
  export APOLLO16_PKG_DIR="${pkg}"

  if (( ! quiet )); then
    echo "Apollo 16 sim environment: ROS ${distro} + ${install}"
    echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
         "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}"
    echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
    echo "  GALLIUM_DRIVER=${GALLIUM_DRIVER} MESA_EXTENSION_OVERRIDE=${MESA_EXTENSION_OVERRIDE:-<unset>}" \
         "GZ_PARTITION=${GZ_PARTITION:-<default>}"
  fi
  return 0
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "env.sh only works when sourced: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi
if _apollo16_env "$@"; then
  unset -f _apollo16_env
  return 0
fi
unset -f _apollo16_env
return 1
