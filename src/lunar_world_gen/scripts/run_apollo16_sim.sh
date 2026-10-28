#!/usr/bin/env bash
# Native WSL / WSLg launcher for the TOGO rover in the generated Apollo 16 world.
# Same environment as src/togo_sim_wsl.sh (scripts/env.sh: native install, WSLg/D3D12 GPU, ROS domain 29,
# CycloneDDS config, resource paths) but runs launch/apollo16_sim.launch.py, which starts the controller
# spawners only after the robot's sensors are up and spawns the robot at a world_info.yaml site.
#
# By default the simulation starts without a GUI, and the Gazebo GUI opens by itself once the launch is
# READY (it writes a ready marker file): in WSL the GUI is reliable only when it starts after the server
# has loaded the world. The GUI window then stays blank for about 30-60 s while it loads the world.
#
# usage: run_apollo16_sim.sh [options] [-- extra launch args (name:=value ...)]
#   --world PATH          world SDF (default ~/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf)
#   --variant V           '' (default) | inspect | stock | inspect_stock  -> <world stem>_<V>.sdf
#   --site NAME           world_info.yaml site: flat (default) | slope | rock | turn
#   --pose X Y Z [YAW]    explicit spawn pose instead of a site (base_link z; YAW in rad)
#   --world-info PATH     world_info.yaml (default: next to the world)
#   --robot-collision M   simplified (default: bounding boxes for the CAD collision meshes) | original
#   --headless            never open the Gazebo GUI (open it later with scripts/open_gui.sh)
#   --gui-early           open the GUI right after the server starts, before READY (the old behaviour)
#   --ready-timeout S     open the GUI only if READY comes within S s (default 600)
#   --headless-rendering  also render the server's sensors without an X display (EGL)
#   --gui-config FILE     Gazebo GUI config, used as is (default: config/gui/apollo16.gui.config with its
#                         camera moved to the spawn; without that file: Gazebo's default GUI config)
#   --default-gui         use Gazebo's default GUI config (~/.gz/sim/8/gui.config, far clip 1000 m)
#   --rviz                start RViz (togo_deploy robot_sensor_checkout, LiDAR /husky/sensors/seyond/points)
#   --keep-crash-dumps    keep WSL crash dumps of this run's gz processes (default: deleted at exit)
#   --dry-run             check and resolve everything, print the commands, start nothing
#   -h | --help
#
# Ready when the log prints "[apollo16_sim] READY" (about 35-60 s). Ctrl+C stops everything in a few
# seconds: the gz server and GUI with SIGKILL (gz-sim 8 segfaults when stopped with SIGINT/SIGTERM and
# WSL would write a 1.5-3.6 GB crash dump), the ROS nodes with SIGINT. A second Ctrl+C forces SIGKILL.
# Close only the GUI with scripts/open_gui.sh --close (another terminal; the simulation keeps running) and
# reopen it with scripts/open_gui.sh. The window's X button can make gz crash (a multi-GB crash dump).
# Another terminal gets the same ROS/DDS/GPU environment with: source <this dir>/env.sh
# At exit, WSL crash dumps (%TEMP%\wsl-crashes) of this run's ruby/gz processes are deleted and any other
# dump written meanwhile is listed. A WARNING appears when WSL has less than APOLLO16_LOW_MEM_MB (3000) MB.
#
# Environment overrides (as togo_sim_wsl.sh): TOGO_WSL_INSTALL (default ~/togo_ws_native_clean/install),
# ROS_DOMAIN_ID (default 29), RMW_IMPLEMENTATION (default rmw_cyclonedds_cpp), CYCLONEDDS_URI,
# TOGO_USE_SIM_TIME (default true), GZ_PARTITION. APOLLO16_ALLOW_CONCURRENT=1 skips the "already running"
# check (only for isolated test runs with their own GZ_PARTITION / ROS_DOMAIN_ID / ROS_HOME).
# APOLLO16_STATE_DIR: session and ready-marker files (default /tmp/apollo16_sim-<uid>);
# APOLLO16_WSL_CRASH_DIR: the crash-dump folder (default %TEMP%\wsl-crashes; empty: no crash-dump check).

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"            # src/lunar_world_gen
LAUNCH_FILE="${PKG_DIR}/launch/apollo16_sim.launch.py"
PACKAGED_GUI_CONFIG="${PKG_DIR}/config/gui/apollo16.gui.config"
OPEN_GUI="${SCRIPT_DIR}/open_gui.sh"
BUILD_HINT="cd ${PKG_DIR} && ~/.venvs/lunar_world/bin/python -m lunar_world_gen.build --preset full"
GUI_EARLY_DELAY_S=8
# shellcheck source=apollo16_lib.sh
source "${SCRIPT_DIR}/apollo16_lib.sh" || exit 1

WORLD="${HOME}/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf"
VARIANT=""
SITE="flat"
POSE=()
WORLD_INFO=""
ROBOT_COLLISION="simplified"
HEADLESS="false"
GUI_EARLY="false"
READY_TIMEOUT=600
HEADLESS_RENDERING="false"
START_RVIZ="false"
GUI_CONFIG=""
USE_DEFAULT_GUI="false"
KEEP_DUMPS=0
DRY_RUN="false"
USE_SIM_TIME="${TOGO_USE_SIM_TIME:-true}"
EXTRA_ARGS=()
READY_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)           a16_need_val "$1" 1 "${@:2}"; WORLD="$2"; shift 2 ;;
    --variant)         a16_need_val "$1" 1 "${@:2}"; VARIANT="$2"; shift 2 ;;
    --site)            a16_need_val "$1" 1 "${@:2}"; SITE="$2"; shift 2 ;;
    --pose)            a16_need_val "$1" 3 "${@:2}"; POSE=("$2" "$3" "$4"); shift 4
                       if [[ $# -gt 0 ]] && a16_is_number "$1"; then POSE+=("$1"); shift; fi ;;
    --world-info)      a16_need_val "$1" 1 "${@:2}"; WORLD_INFO="$2"; shift 2 ;;
    --robot-collision) a16_need_val "$1" 1 "${@:2}"; ROBOT_COLLISION="$2"; shift 2 ;;
    --gui-config)      a16_need_val "$1" 1 "${@:2}"; GUI_CONFIG="$2"; shift 2 ;;
    --default-gui)     USE_DEFAULT_GUI="true"; shift ;;
    --headless)        HEADLESS="true"; shift ;;
    --gui-early)       GUI_EARLY="true"; shift ;;
    --ready-timeout)   a16_need_val "$1" 1 "${@:2}"; READY_TIMEOUT="$2"; shift 2 ;;
    --headless-rendering) HEADLESS_RENDERING="true"; shift ;;
    --rviz)            START_RVIZ="true"; shift ;;
    --keep-crash-dumps) KEEP_DUMPS=1; shift ;;
    --dry-run)         DRY_RUN="true"; shift ;;
    -h|--help)         a16_usage; exit 0 ;;
    --)                shift; EXTRA_ARGS=("$@"); break ;;
    *:=*)              EXTRA_ARGS+=("$1"); shift ;;
    *)                 a16_die_usage "unknown option '$1'" ;;
  esac
done

case "${VARIANT}" in
  ""|inspect|stock|inspect_stock) ;;
  *) a16_die_usage "--variant must be '', inspect, stock or inspect_stock (got '${VARIANT}')" ;;
esac
case "${ROBOT_COLLISION}" in
  simplified|original) ;;
  *) a16_die_usage "--robot-collision must be simplified or original (got '${ROBOT_COLLISION}')" ;;
esac
[[ -n "${SITE}" ]] || a16_die_usage "--site needs a site name (flat, slope, rock, turn)"
for v in "${POSE[@]}"; do a16_is_number "${v}" || a16_die_usage "--pose values must be numbers (got '${v}')"; done
[[ -z "${GUI_CONFIG}" || -f "${GUI_CONFIG}" ]] || a16_die_usage "--gui-config: file not found: ${GUI_CONFIG}"
[[ -z "${GUI_CONFIG}" || "${USE_DEFAULT_GUI}" != "true" ]] || a16_die_usage "use either --gui-config or --default-gui"
[[ "${HEADLESS}" != "true" || "${GUI_EARLY}" != "true" ]] || a16_die_usage "use either --headless or --gui-early"
a16_is_seconds "${READY_TIMEOUT}" || a16_die_usage "--ready-timeout must be a whole number of seconds > 0 (got '${READY_TIMEOUT}')"
READY_TIMEOUT=$(( 10#${READY_TIMEOUT} ))
for a in "${EXTRA_ARGS[@]}"; do
  [[ "${a}" == *:=* ]] || a16_die_usage "extra launch arguments must look like name:=value (got '${a}')"
  [[ "${a}" != world_info:=* ]] || WORLD_INFO="${a#world_info:=}"
  [[ "${a}" != ready_file:=* ]] || READY_FILE="${a#ready_file:=}"
done
[[ -f "${LAUNCH_FILE}" ]] || { echo "ERROR: launch file not found: ${LAUNCH_FILE}" >&2; exit 1; }
if [[ "${HEADLESS}" == "true" ]]; then GUI_WHEN="never"
elif [[ "${GUI_EARLY}" == "true" ]]; then GUI_WHEN="early"
else GUI_WHEN="ready"; fi

# ------------------------------------------------------------ one simulation at a time
if [[ "${DRY_RUN}" != "true" && "${APOLLO16_ALLOW_CONCURRENT:-0}" != "1" ]]; then
  if pgrep -f '[r]os2 launch togo_gz sim_gz.launch.py' >/dev/null 2>&1 \
     || pgrep -f '[r]os2 launch .*apollo16_sim[.]launch[.]py' >/dev/null 2>&1; then
    echo "ERROR: a Togo Gazebo simulation is already running." >&2
    echo "Stop it with Ctrl+C in its original terminal before starting another." >&2
    exit 1
  fi
fi

# ------------------------------------------------------------ ROS / DDS / GPU / Gazebo environment
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh" --quiet || exit 1
set -u

# ------------------------------------------------------------ this run's folder (session, ready marker, GUI config)
RUN_DIR=""
if [[ "${DRY_RUN}" == "true" ]]; then
  STATE_DIR="${APOLLO16_STATE_DIR:-/tmp/apollo16_sim-$(id -u)}"
  [[ -n "${READY_FILE}" ]] || READY_FILE="${STATE_DIR}/run.$$/ready"
else
  STATE_DIR="$(a16_state_dir)" || exit 1
  a16_prune_state "${STATE_DIR}"
  RUN_DIR="${STATE_DIR}/run.$$"
  rm -rf -- "${RUN_DIR}"
  mkdir -m 700 -- "${RUN_DIR}" || exit 1
  trap 'rm -rf -- "${RUN_DIR}"' EXIT
  [[ -n "${READY_FILE}" ]] || READY_FILE="${RUN_DIR}/ready"
fi
NOT_READY_FILE="${READY_FILE}.not_ready"

# ------------------------------------------------------------ resolve world / pose / GUI (the launch file's code)
GUI_MODE="none"; GUI_SRC=""; GUI_FILE=""
if [[ "${GUI_WHEN}" != "never" && "${USE_DEFAULT_GUI}" != "true" ]]; then
  if [[ -n "${GUI_CONFIG}" ]]; then
    GUI_MODE="verbatim"; GUI_SRC="$(cd -- "$(dirname -- "${GUI_CONFIG}")" && pwd)/$(basename -- "${GUI_CONFIG}")"
    GUI_FILE="${GUI_SRC}"
  elif [[ -f "${PACKAGED_GUI_CONFIG}" ]]; then
    GUI_MODE="follow"; GUI_SRC="${PACKAGED_GUI_CONFIG}"
    [[ -z "${RUN_DIR}" ]] || GUI_FILE="${RUN_DIR}/gui.config"
  fi
fi
RESOLVED="$(A16_LAUNCH="${LAUNCH_FILE}" A16_WORLD="${WORLD}" A16_VARIANT="${VARIANT}" A16_SITE="${SITE}" \
            A16_WORLD_INFO="${WORLD_INFO}" A16_CHECK_PLUGIN=1 A16_GUI_MODE="${GUI_MODE}" A16_GUI_SRC="${GUI_SRC}" \
            A16_GUI_OUT="$([[ "${GUI_MODE}" == follow ]] && echo "${GUI_FILE}")" a16_resolve "${POSE[@]}")"
RESOLVE_RC=$?
if [[ ${RESOLVE_RC} -ne 0 ]]; then
  echo "Cannot start: fix the world / site / pose above (world built? ${BUILD_HINT})." >&2
  exit 1
fi
WORLD_FILE="$(a16_field "${RESOLVED}" WORLD_FILE)"
GRAVITY="$(a16_field "${RESOLVED}" GRAVITY)"
GUI_NOTE="$(a16_field "${RESOLVED}" GUI_NOTE)"
POSE_VALUES="$(a16_field "${RESOLVED}" POSE_VALUES)"
mapfile -t NOTES < <(a16_field "${RESOLVED}" NOTE)

LAUNCH_ARGS=("world:=${WORLD_FILE}" "site:=${SITE}" "rviz:=${START_RVIZ}" "use_sim_time:=${USE_SIM_TIME}"
             "headless_rendering:=${HEADLESS_RENDERING}" "robot_collision:=${ROBOT_COLLISION}")
[[ -z "${WORLD_INFO}" ]] || LAUNCH_ARGS+=("world_info:=${WORLD_INFO}")
if [[ ${#POSE[@]} -ge 3 ]]; then
  LAUNCH_ARGS+=("robot_x:=${POSE[0]}" "robot_y:=${POSE[1]}" "robot_z:=${POSE[2]}")
  [[ ${#POSE[@]} -lt 4 ]] || LAUNCH_ARGS+=("robot_yaw:=${POSE[3]}")
fi
USER_READY_FILE="false"
for a in "${EXTRA_ARGS[@]}"; do [[ "${a}" != ready_file:=* ]] || USER_READY_FILE="true"; done
[[ "${USER_READY_FILE}" == "true" ]] || LAUNCH_ARGS+=("ready_file:=${READY_FILE}")
GUI_ARGS=()
[[ -z "${GUI_FILE}" && "${GUI_MODE}" != "follow" ]] || GUI_ARGS=(--gui-config "${GUI_FILE:-<per-run copy of ${GUI_SRC}>}")
OPEN_GUI_CMD="${OPEN_GUI}"                              # how to open the GUI from another terminal
[[ "${USE_DEFAULT_GUI}" != "true" ]] || OPEN_GUI_CMD+=" --default-gui"
if [[ -n "${GUI_CONFIG}" ]]; then
  OPEN_GUI_CMD+=" --gui-config $(printf '%q' "$(cd -- "$(dirname -- "${GUI_CONFIG}")" && pwd)/$(basename -- "${GUI_CONFIG}")")"
fi

case "${GUI_MODE}:${USE_DEFAULT_GUI}" in
  verbatim:*) GUI_CFG_LINE="config ${GUI_SRC}" ;;
  follow:*)   GUI_CFG_LINE="config ${GUI_SRC} (${GUI_NOTE:-camera as in the file})" ;;
  none:true)  GUI_CFG_LINE="Gazebo's default GUI config (--default-gui; far clip 1000 m)" ;;
  *)          GUI_CFG_LINE="Gazebo's default GUI config (no ${PACKAGED_GUI_CONFIG}; far clip 1000 m hides the horizon)" ;;
esac
case "${GUI_WHEN}" in
  ready) GUI_LINE="opens after READY, ${GUI_CFG_LINE}" ;;
  early) GUI_LINE="opens ${GUI_EARLY_DELAY_S} s after the server starts (--gui-early), ${GUI_CFG_LINE}" ;;
  *)     GUI_LINE="not opened (--headless); open it later from another terminal: ${OPEN_GUI_CMD}" ;;
esac

echo "=========================================="
echo "Starting TOGO in the Apollo 16 world (WSL)"
echo "=========================================="
echo "World:        ${WORLD_FILE}"
echo "Physics:      $(a16_field "${RESOLVED}" PHYSICS)"
echo "Robot pose:   $(a16_field "${RESOLVED}" POSE)"
echo "Collision:    ${ROBOT_COLLISION}$([[ "${ROBOT_COLLISION}" == simplified ]] && echo ' (bounding boxes for the CAD motor / LiDAR / beam meshes; --robot-collision original)')"
echo "Site:         ${SITE}${VARIANT:+ (variant ${VARIANT})}"
echo "RViz:         ${START_RVIZ}"
echo "Gazebo GUI:   ${GUI_LINE}"
echo "Use sim time: ${USE_SIM_TIME}"
echo "ROS domain:   ${ROS_DOMAIN_ID}   GZ_PARTITION: ${GZ_PARTITION:-<default>}"
echo "RMW:          ${RMW_IMPLEMENTATION}"
[[ -z "${GRAVITY}" ]] || echo "Gravity:      ${GRAVITY} m/s^2 -> SLAM: DLIO odom/gravity must be ${GRAVITY} (seyond_dlio_graph.yaml has 9.82)"
echo "Ready marker: ${READY_FILE}"
echo "Crash dumps:  $([[ ${KEEP_DUMPS} -eq 1 ]] && echo 'kept (--keep-crash-dumps)' || echo 'WSL crash dumps of this run are deleted at exit (--keep-crash-dumps keeps them)')"
echo "Launch file:  ${LAUNCH_FILE}"
echo "=========================================="
for n in "${NOTES[@]}"; do
  echo "WARNING: ${n}" >&2
done

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "Dry run, nothing started. Would run:"
  printf '  setsid ros2 launch %q' "${LAUNCH_FILE}"; printf ' %q' "${LAUNCH_ARGS[@]}" "${EXTRA_ARGS[@]}"; echo
  case "${GUI_WHEN}" in
    ready) echo "  then wait until the launch writes ${READY_FILE} (READY; at most ${READY_TIMEOUT} s) and run:" ;;
    early) echo "  then, ${GUI_EARLY_DELAY_S} s later (--gui-early):" ;;
    *)     echo "  and no GUI (--headless). Open it later from another terminal: ${OPEN_GUI_CMD}" ;;
  esac
  if [[ "${GUI_WHEN}" != "never" ]]; then
    printf '  setsid gz sim -g'; (( ${#GUI_ARGS[@]} == 0 )) || printf ' %q' "${GUI_ARGS[@]}"; echo
  fi
  exit 0
fi

# ------------------------------------------------------------ cleanup (bounded; gz only ever gets SIGKILL)
SERVER_PID=""
GUI_PID=""
GUI_DONE=0
EXPECT_DUMPS=""
START_EPOCH="$(date +%s)"
T_LAUNCH=${SECONDS}
wait_group() {   # wait_group <pgid> <seconds>: 0 when the process group is gone
  local i
  for ((i = 0; i < $2 * 10; i++)); do
    a16_group_alive "$1" || return 0
    (( A16_FORCE_STOP )) && return 1
    sleep 0.1
  done
  return 1
}
gz_server_pids() { pgrep -g "$1" '^(ruby.*|gz)$' 2>/dev/null || true; }

stop_launch() {  # stop_launch <pgid of ros2 launch>
  local pg="$1" pid guard i
  a16_group_alive "${pg}" || return 0
  # 1. gz server: never SIGINT/SIGTERM. The launch's gz guard stops it with SIGKILL on SIGINT; any
  #    server still running after that (or without a guard) gets SIGKILL here.
  guard="$(pgrep -g "${pg}" -f -- '--gz-guard' 2>/dev/null || true)"
  if [[ -n "${guard}" ]]; then
    # shellcheck disable=SC2086   # one or more PIDs
    kill -INT ${guard} 2>/dev/null || true
    for ((i = 0; i < 30; i++)); do [[ -z "$(gz_server_pids "${pg}")" ]] && break; sleep 0.1; done
  fi
  for pid in $(gz_server_pids "${pg}"); do kill -KILL "${pid}" 2>/dev/null || true; done
  # 2. ROS nodes: SIGINT to ros2 launch, which stops its processes normally
  kill -INT "${pg}" 2>/dev/null || true
  wait_group "${pg}" 15 && return 0
  if (( ! A16_FORCE_STOP )); then
    echo "ros2 launch: still running after 15 s, sending SIGTERM" >&2
    for pid in $(gz_server_pids "${pg}"); do kill -KILL "${pid}" 2>/dev/null || true; done
    kill -TERM -- "-${pg}" 2>/dev/null || true
    wait_group "${pg}" 5 && return 0
  fi
  echo "ros2 launch: sending SIGKILL" >&2
  kill -KILL -- "-${pg}" 2>/dev/null || true
  wait_group "${pg}" 5 || true
}

cleanup() {
  local exit_code=$? t0="${EPOCHREALTIME:-}"
  trap 'A16_FORCE_STOP=1; echo "forcing stop (SIGKILL)" >&2' INT TERM HUP
  trap - EXIT
  echo
  echo "Stopping Apollo 16 simulation..."
  if [[ -n "${GUI_PID}" ]]; then
    # Gazebo GUI: SIGKILL (no segfault, no crash dump); kill and reap in one silenced block, else bash
    # prints a "Killed  setsid gz sim -g ..." job notice
    { if a16_group_alive "${GUI_PID}"; then kill -KILL -- "-${GUI_PID}"; fi; wait "${GUI_PID}"; } 2>/dev/null
  fi
  [[ -z "${SERVER_PID}" ]] || stop_launch "${SERVER_PID}"
  [[ -z "${SERVER_PID}" ]] || wait "${SERVER_PID}" 2>/dev/null || true
  a16_unregister_gui "${STATE_DIR}" "${GUI_PID}"
  rm -rf -- "${RUN_DIR}"
  a16_crash_dumps_after_run "${START_EPOCH}" "${KEEP_DUMPS}" "${EXPECT_DUMPS}"
  if [[ -n "${t0}" ]]; then
    echo "Apollo 16 simulation stopped ($(awk -v a="${t0}" -v b="${EPOCHREALTIME}" 'BEGIN { printf "%.1f", b - a }') s)."
  else
    echo "Apollo 16 simulation stopped."
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM HUP

start_gui() {
  a16_gui_notice "Close the GUI and the simulation: Ctrl+C in this terminal." \
                 "Close only the GUI, from another terminal: ${OPEN_GUI} --close   (open_gui.sh alone reopens it)" \
                 "Do not close it with the window's X button: gz can crash, and WSL then writes a multi-GB crash dump."
  setsid gz sim -g "${GUI_ARGS[@]}" &
  GUI_PID=$!
  a16_register_gui "${STATE_DIR}" "${GUI_PID}"
  a16_kv_set "${RUN_DIR}/session" "gui_state=open" "gui_pgid=${GUI_PID}"
  a16_record_pids "${GUI_PID}"
}

gui_exited() {   # the GUI ended while the simulation runs
  local rc
  wait "${GUI_PID}" 2>/dev/null
  rc=$?
  GUI_DONE=1
  a16_unregister_gui "${STATE_DIR}" "${GUI_PID}"
  a16_kv_set "${RUN_DIR}/session" "gui_state=closed"
  if a16_core_signal_rc "${rc}"; then
    EXPECT_DUMPS+=" ${GUI_PID}:$(date +%s)"
    echo "WARNING: the Gazebo GUI crashed (exit code ${rc}, signal $(( rc - 128 ))). WSL writes a crash dump of it to" \
         "%TEMP%\\wsl-crashes; it is deleted when the simulation stops (unless --keep-crash-dumps)." >&2
  elif (( rc == 137 )); then
    echo "The Gazebo GUI was closed (SIGKILL, e.g. by open_gui.sh --close)."
  else
    echo "The Gazebo GUI exited (code ${rc})."
  fi
  echo "The simulation keeps running. Reopen the GUI from another terminal: ${OPEN_GUI_CMD}"
}

if command -v glxinfo >/dev/null 2>&1; then
  RENDERER="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -n 1)"
  if [[ -n "${RENDERER}" ]]; then
    echo "OpenGL renderer: ${RENDERER}"
    [[ "${RENDERER}" != *llvmpipe* ]] || echo "WARNING: WSL is using software rendering instead of the GPU." >&2
  fi
fi

echo
case "${GUI_WHEN}" in
  ready) echo "[1/2] Starting the Gazebo server and ROS nodes (the GUI follows at READY)..." ;;
  early) echo "[1/2] Starting the Gazebo server and ROS nodes..." ;;
  *)     echo "[1/2] Starting the Gazebo server and ROS nodes (no GUI: --headless)..." ;;
esac
rm -f -- "${READY_FILE}" "${NOT_READY_FILE}"            # a marker of an earlier run must not count
START_EPOCH="$(date +%s)"
T_LAUNCH=${SECONDS}
setsid ros2 launch "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" "${EXTRA_ARGS[@]}" &
SERVER_PID=$!
a16_write_kv "${RUN_DIR}/session" "wrapper_pid=$$" "launch_pgid=${SERVER_PID}" "started=${START_EPOCH}" \
             "world=${WORLD_FILE}" "world_info=${WORLD_INFO}" "site=${SITE}" "pose=${POSE_VALUES}" \
             "gz_partition=${GZ_PARTITION:-}" "ros_domain_id=${ROS_DOMAIN_ID}" "ready_file=${READY_FILE}" \
             "not_ready_file=${NOT_READY_FILE}" "gui=${GUI_WHEN}" \
             "gui_state=$([[ "${GUI_WHEN}" == never ]] && echo none || echo pending)"
a16_record_pids "${SERVER_PID}"

echo
echo "=========================================="
case "${GUI_WHEN}" in
  ready) echo "Starting up: wait for \"[apollo16_sim] READY\" (about 35-60 s; the robot's sensors build the"
         echo "render scene first). The Gazebo GUI opens after it. Ctrl+C stops everything (a few seconds)." ;;
  *)     echo "Starting up: wait for \"[apollo16_sim] READY\" (about 35-60 s; the robot's sensors build the"
         echo "render scene first). Ctrl+C stops everything (a few seconds)." ;;
esac
echo "=========================================="
echo "LiDAR:       /husky/sensors/seyond/points"
echo "Drive:       /platform_velocity_controller/cmd_vel (geometry_msgs/TwistStamped)"
echo "Clock:       /clock"
echo "Other terminal: source ${SCRIPT_DIR}/env.sh"
echo "  then e.g.  ros2 control list_controllers"
echo "Ground truth (gz): gz model -m togo -p   (one shot; same GZ_PARTITION)"
echo "Gazebo GUI:  ${OPEN_GUI_CMD}  (opens it; --close closes it, the simulation keeps running)"
echo "=========================================="

case "${GUI_WHEN}" in
  early)
    for ((j = 0; j < GUI_EARLY_DELAY_S * 2; j++)); do
      if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
        wait "${SERVER_PID}"
        exit $?
      fi
      a16_nap 0.5
    done
    a16_record_pids "${SERVER_PID}"
    echo "[2/2] Starting the Gazebo GUI through WSLg (--gui-early; ${GUI_CFG_LINE})..."
    start_gui
    ;;
  ready)
    RESULT=""
    i=0
    while [[ -z "${RESULT}" ]]; do
      if [[ -f "${READY_FILE}" ]]; then RESULT="ready"
      elif [[ -f "${NOT_READY_FILE}" ]]; then RESULT="not_ready"
      elif ! kill -0 "${SERVER_PID}" 2>/dev/null; then RESULT="exited"
      elif (( SECONDS - T_LAUNCH >= READY_TIMEOUT )); then RESULT="timeout"
      else
        a16_nap 0.5
        if (( ++i % 4 == 0 )); then
          a16_record_pids "${SERVER_PID}"
          a16_check_memory "" "${SERVER_PID}"
        fi
      fi
    done
    a16_record_pids "${SERVER_PID}"
    case "${RESULT}" in
      ready)
        echo "[2/2] READY after $(( SECONDS - T_LAUNCH )) s: starting the Gazebo GUI through WSLg..."
        start_gui ;;
      not_ready)
        a16_kv_set "${RUN_DIR}/session" "gui_state=skipped"
        REASON="$(a16_kv "${NOT_READY_FILE}" reason)"
        echo "[2/2] The launch reported NOT READY after $(( SECONDS - T_LAUNCH )) s (${REASON:-see its WARNING above})," \
             "so the Gazebo GUI was not opened. The simulation keeps running." >&2
        echo "      Check: ros2 control list_controllers (after source ${SCRIPT_DIR}/env.sh)." \
             "Open the GUI anyway from another terminal: ${OPEN_GUI_CMD}" >&2 ;;
      timeout)
        a16_kv_set "${RUN_DIR}/session" "gui_state=skipped"
        echo "[2/2] No READY within ${READY_TIMEOUT} s, so the Gazebo GUI was not opened. The simulation keeps" \
             "running; its last [wait_for_sensors] / [wait_ready] lines above say what it waits for (WSL memory" \
             "available: $(a16_mem_available_mb) MB). Open the GUI anyway from another terminal: ${OPEN_GUI_CMD} --no-wait" >&2 ;;
      exited)
        wait "${SERVER_PID}"
        RC=$?
        echo "[2/2] The launch exited with code ${RC} before READY (see its messages above); no GUI was opened." >&2
        exit "${RC}" ;;
    esac
    ;;
  *)
    echo "[2/2] No Gazebo GUI (--headless). Open it from another terminal (it waits for READY): ${OPEN_GUI_CMD}" ;;
esac

i=0
while kill -0 "${SERVER_PID}" 2>/dev/null; do
  a16_nap 0.5
  (( ++i % 4 == 0 )) || continue
  a16_record_pids "${SERVER_PID}" "${GUI_PID}"
  if [[ -n "${GUI_PID}" && ${GUI_DONE} -eq 0 ]] && ! kill -0 "${GUI_PID}" 2>/dev/null; then
    gui_exited
  fi
  a16_check_memory "$([[ ${GUI_DONE} -eq 0 ]] && echo "${GUI_PID}")" "${SERVER_PID}"
done
wait "${SERVER_PID}"
