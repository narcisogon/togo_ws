#!/usr/bin/env bash
# Opens the Gazebo GUI for an Apollo 16 simulation that is already running: after
# run_apollo16_sim.sh --headless, or to reopen the GUI after closing it. Run it in a second terminal.
# It finds the running run_apollo16_sim.sh, takes that simulation's GZ_PARTITION and spawn pose (the start
# camera looks at the rover), waits until the simulation is READY, then starts the GUI in its own process
# group. The window appears within seconds but stays blank for about 30-60 s while it loads the world.
#
# Close the GUI with Ctrl+C in this terminal; the simulation keeps running. Do not use the window's X
# button: gz can crash, and WSL then writes a multi-GB crash dump. The GUI also closes when the
# simulation stops.
#
# usage: open_gui.sh [options]
#   --default-gui         use Gazebo's default GUI config (~/.gz/sim/8/gui.config, far clip 1000 m)
#   --gui-config FILE     Gazebo GUI config, used as is (default: config/gui/apollo16.gui.config with its
#                         camera moved to the simulation's spawn)
#   --site NAME           start camera at this world_info.yaml site instead of the simulation's spawn
#   --pose X Y Z [YAW]    start camera at this spawn pose instead (base_link z; YAW in rad)
#   --world PATH          world SDF whose world_info.yaml has the sites (default: the simulation's world)
#   --world-info PATH     world_info.yaml (default: the simulation's, else next to the world)
#   --no-wait             open at once, also before READY
#   --ready-timeout S     wait at most S s for READY (default 600)
#   --close               close the GUI that run_apollo16_sim.sh or open_gui.sh opened (SIGKILL, no crash
#                         dump); the simulation keeps running
#   --keep-crash-dumps    keep WSL crash dumps of this GUI (default: deleted at exit)
#   --dry-run             find the simulation and print the command, start nothing
#   -h | --help
#
# Without a running run_apollo16_sim.sh (a simulation started some other way) the GUI opens at once, in
# this terminal's GZ_PARTITION, with the camera at the 'flat' site of the default world. With several
# simulations (APOLLO16_ALLOW_CONCURRENT=1) choose one: GZ_PARTITION=<its partition> open_gui.sh.
# One GUI at a time: each needs about 3.5 GB of memory. At exit, WSL crash dumps of this GUI are deleted
# (other dumps are listed, never touched). Environment variables as for run_apollo16_sim.sh.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"            # src/lunar_world_gen
LAUNCH_FILE="${PKG_DIR}/launch/apollo16_sim.launch.py"
PACKAGED_GUI_CONFIG="${PKG_DIR}/config/gui/apollo16.gui.config"
DEFAULT_WORLD="${HOME}/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf"
# shellcheck source=apollo16_lib.sh
source "${SCRIPT_DIR}/apollo16_lib.sh" || exit 1

GUI_CONFIG=""
USE_DEFAULT_GUI="false"
SITE=""
POSE=()
WORLD=""
WORLD_INFO=""
NO_WAIT="false"
READY_TIMEOUT=600
CLOSE="false"
KEEP_DUMPS=0
DRY_RUN="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --default-gui)      USE_DEFAULT_GUI="true"; shift ;;
    --gui-config)       a16_need_val "$1" 1 "${@:2}"; GUI_CONFIG="$2"; shift 2 ;;
    --site)             a16_need_val "$1" 1 "${@:2}"; SITE="$2"; shift 2 ;;
    --pose)             a16_need_val "$1" 3 "${@:2}"; POSE=("$2" "$3" "$4"); shift 4
                        if [[ $# -gt 0 ]] && a16_is_number "$1"; then POSE+=("$1"); shift; fi ;;
    --world)            a16_need_val "$1" 1 "${@:2}"; WORLD="$2"; shift 2 ;;
    --world-info)       a16_need_val "$1" 1 "${@:2}"; WORLD_INFO="$2"; shift 2 ;;
    --no-wait)          NO_WAIT="true"; shift ;;
    --ready-timeout)    a16_need_val "$1" 1 "${@:2}"; READY_TIMEOUT="$2"; shift 2 ;;
    --close)            CLOSE="true"; shift ;;
    --keep-crash-dumps) KEEP_DUMPS=1; shift ;;
    --dry-run)          DRY_RUN="true"; shift ;;
    -h|--help)          a16_usage; exit 0 ;;
    *)                  a16_die_usage "unknown option '$1'" ;;
  esac
done
for v in "${POSE[@]}"; do a16_is_number "${v}" || a16_die_usage "--pose values must be numbers (got '${v}')"; done
[[ -z "${GUI_CONFIG}" || -f "${GUI_CONFIG}" ]] || a16_die_usage "--gui-config: file not found: ${GUI_CONFIG}"
[[ -z "${GUI_CONFIG}" || "${USE_DEFAULT_GUI}" != "true" ]] || a16_die_usage "use either --gui-config or --default-gui"
[[ -z "${SITE}" || ${#POSE[@]} -eq 0 ]] || a16_die_usage "use either --site or --pose"
a16_is_seconds "${READY_TIMEOUT}" || a16_die_usage "--ready-timeout must be a whole number of seconds > 0 (got '${READY_TIMEOUT}')"
READY_TIMEOUT=$(( 10#${READY_TIMEOUT} ))
[[ -f "${LAUNCH_FILE}" ]] || { echo "ERROR: launch file not found: ${LAUNCH_FILE}" >&2; exit 1; }

# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh" --quiet || exit 1
set -u

if [[ "${DRY_RUN}" == "true" ]]; then
  STATE_DIR="${APOLLO16_STATE_DIR:-/tmp/apollo16_sim-$(id -u)}"
else
  STATE_DIR="$(a16_state_dir)" || exit 1
  a16_prune_state "${STATE_DIR}"
fi

# ------------------------------------------------------------ --close
if [[ "${CLOSE}" == "true" ]]; then
  mapfile -t REGS < <(a16_open_guis "${STATE_DIR}")
  if (( ${#REGS[@]} == 0 )); then
    echo "No Gazebo GUI opened by run_apollo16_sim.sh or open_gui.sh is running${GZ_PARTITION+ in GZ_PARTITION=${GZ_PARTITION}}."
    exit 0
  fi
  for f in "${REGS[@]}"; do
    PG="$(a16_kv "${f}" pgid)"
    WHO="$(a16_kv "${f}" owner) (pid $(a16_kv "${f}" owner_pid))"
    if [[ "${DRY_RUN}" == "true" ]]; then
      echo "Dry run: would close the Gazebo GUI of ${WHO}: kill -KILL -- -${PG}"
      continue
    fi
    kill -KILL -- "-${PG}" 2>/dev/null || true
    for ((i = 0; i < 30; i++)); do a16_group_alive "${PG}" || break; sleep 0.1; done
    echo "Closed the Gazebo GUI of ${WHO} with SIGKILL (no crash dump); the simulation keeps running."
  done
  exit 0
fi

# ------------------------------------------------------------ the running simulation
mapfile -t SESSIONS < <(a16_live_sessions "${STATE_DIR}")
CHOSEN=()
for f in "${SESSIONS[@]}"; do
  if [[ -z "${GZ_PARTITION+x}" || "$(a16_kv "${f}" gz_partition)" == "${GZ_PARTITION}" ]]; then CHOSEN+=("${f}"); fi
done
if (( ${#CHOSEN[@]} > 1 )); then
  echo "ERROR: ${#CHOSEN[@]} simulations are running; choose one with GZ_PARTITION=<partition> $0:" >&2
  for f in "${CHOSEN[@]}"; do
    echo "  GZ_PARTITION=$(a16_kv "${f}" gz_partition)   (site $(a16_kv "${f}" site), run_apollo16_sim.sh pid" \
         "$(a16_kv "${f}" wrapper_pid))" >&2
  done
  exit 1
fi
SESSION="${CHOSEN[0]:-}"
WRAPPER_PID=""
LAUNCH_PGID=""
READY_FILE=""
NOT_READY_FILE=""
CAMERA_AT=""
if [[ -n "${SESSION}" ]]; then
  WRAPPER_PID="$(a16_kv "${SESSION}" wrapper_pid)"
  LAUNCH_PGID="$(a16_kv "${SESSION}" launch_pgid)"
  S_PARTITION="$(a16_kv "${SESSION}" gz_partition)"
  if [[ -z "${GZ_PARTITION+x}" && -n "${S_PARTITION}" ]]; then export GZ_PARTITION="${S_PARTITION}"; fi
  READY_FILE="$(a16_kv "${SESSION}" ready_file)"
  NOT_READY_FILE="$(a16_kv "${SESSION}" not_ready_file)"
  [[ -n "${WORLD}" ]] || WORLD="$(a16_kv "${SESSION}" world)"
  [[ -n "${WORLD_INFO}" ]] || WORLD_INFO="$(a16_kv "${SESSION}" world_info)"
  if [[ -z "${SITE}" && ${#POSE[@]} -eq 0 ]]; then
    read -r -a POSE <<< "$(a16_kv "${SESSION}" pose)"
    CAMERA_AT="the simulation's spawn (site $(a16_kv "${SESSION}" site))"
  fi
  echo "Simulation:   run_apollo16_sim.sh pid ${WRAPPER_PID}, site $(a16_kv "${SESSION}" site)," \
       "GZ_PARTITION ${GZ_PARTITION:-<default>}"
  case "$(a16_kv "${SESSION}" gui_state)" in
    pending)
      echo "ERROR: this simulation opens its own Gazebo GUI $([[ "$(a16_kv "${SESSION}" gui)" == early ]] \
            && echo 'a few seconds after its start' || echo 'when it is READY'). One GUI at a time (each needs" \
           "about 3.5 GB of memory). To open the GUI from here instead, start the simulation with --headless." >&2
      exit 1 ;;
  esac
else
  echo "No running run_apollo16_sim.sh found$([[ -n "${GZ_PARTITION+x}" ]] && echo " in GZ_PARTITION=${GZ_PARTITION}"):" \
       "opening the GUI at once in GZ_PARTITION ${GZ_PARTITION:-<default>} (for a simulation started some other way)."
  NO_WAIT="true"
fi
[[ -n "${WORLD}" ]] || WORLD="${DEFAULT_WORLD}"
if [[ -z "${CAMERA_AT}" ]]; then
  if (( ${#POSE[@]} )); then CAMERA_AT="pose ${POSE[*]}"; else SITE="${SITE:-flat}"; CAMERA_AT="site ${SITE}"; fi
fi
SITE="${SITE:-flat}"

mapfile -t OPEN_GUIS < <(a16_open_guis "${STATE_DIR}")
if (( ${#OPEN_GUIS[@]} )); then
  OWNER="$(a16_kv "${OPEN_GUIS[0]}" owner)"
  echo "ERROR: a Gazebo GUI is already open (opened by ${OWNER} pid $(a16_kv "${OPEN_GUIS[0]}" owner_pid)). One GUI" \
       "at a time: each needs about 3.5 GB of memory. Close it first: $0 --close$([[ "${OWNER}" == open_gui.sh ]] \
       && echo ' (or Ctrl+C in its terminal)' || echo ' (the simulation keeps running)')" >&2
  exit 1
fi

# ------------------------------------------------------------ wait for READY
if [[ "${NO_WAIT}" != "true" && -n "${READY_FILE}" && ! -f "${READY_FILE}" ]]; then
  if [[ -f "${NOT_READY_FILE}" ]]; then
    echo "WARNING: the simulation reported NOT READY ($(a16_kv "${NOT_READY_FILE}" reason)); opening the GUI anyway." >&2
  elif [[ "${DRY_RUN}" == "true" ]]; then
    echo "Dry run: would wait until ${READY_FILE} appears (READY; at most ${READY_TIMEOUT} s)."
  else
    echo "Waiting until the simulation is READY (at most ${READY_TIMEOUT} s; --no-wait opens the GUI at once)..."
    T0=${SECONDS}
    RESULT=""
    while [[ -z "${RESULT}" ]]; do
      if [[ -f "${READY_FILE}" ]]; then RESULT="ready"
      elif [[ -f "${NOT_READY_FILE}" ]]; then RESULT="not_ready"
      elif ! a16_is_wrapper "${WRAPPER_PID}"; then RESULT="gone"
      elif (( SECONDS - T0 >= READY_TIMEOUT )); then RESULT="timeout"
      else a16_nap 0.5
      fi
    done
    case "${RESULT}" in
      ready)     echo "READY (waited $(( SECONDS - T0 )) s)." ;;
      not_ready) echo "WARNING: the simulation reported NOT READY ($(a16_kv "${NOT_READY_FILE}" reason));" \
                      "opening the GUI anyway." >&2 ;;
      gone)      echo "ERROR: the simulation stopped before it was READY; no GUI opened." >&2; exit 1 ;;
      *)         echo "ERROR: the simulation was not READY within ${READY_TIMEOUT} s, so no GUI was opened. Open it" \
                      "anyway with: $0 --no-wait" >&2; exit 1 ;;
    esac
  fi
fi

# ------------------------------------------------------------ GUI config (camera at the spawn)
TMP_DIR=""
GUI_PID=""
EXPECT_DUMPS=""
START_EPOCH="$(date +%s)"
cleanup() {
  local rc=$?
  trap 'A16_FORCE_STOP=1' INT TERM HUP
  trap - EXIT
  if [[ -n "${GUI_PID}" ]] && a16_group_alive "${GUI_PID}"; then
    # SIGKILL: no segfault, no crash dump; kill and reap in one silenced block (no "Killed" job notice)
    { kill -KILL -- "-${GUI_PID}"; wait "${GUI_PID}"; } 2>/dev/null
    echo "Gazebo GUI closed."
  fi
  [[ -z "${GUI_PID}" ]] || wait "${GUI_PID}" 2>/dev/null || true
  a16_unregister_gui "${STATE_DIR}" "${GUI_PID}"
  [[ -z "${TMP_DIR}" ]] || rm -rf -- "${TMP_DIR}"
  [[ -z "${GUI_PID}" ]] || a16_crash_dumps_after_run "${START_EPOCH}" "${KEEP_DUMPS}" "${EXPECT_DUMPS}"
  exit "${rc}"
}
if [[ "${DRY_RUN}" != "true" ]]; then
  trap cleanup EXIT
  trap 'exit 0' INT TERM HUP                              # Ctrl+C is the normal way to close the GUI
fi

GUI_MODE="none"; GUI_SRC=""; GUI_FILE=""; GUI_NOTE=""
if [[ "${USE_DEFAULT_GUI}" != "true" ]]; then
  if [[ -n "${GUI_CONFIG}" ]]; then
    GUI_MODE="verbatim"; GUI_SRC="$(cd -- "$(dirname -- "${GUI_CONFIG}")" && pwd)/$(basename -- "${GUI_CONFIG}")"
    GUI_FILE="${GUI_SRC}"
  elif [[ -f "${PACKAGED_GUI_CONFIG}" ]]; then
    GUI_MODE="follow"; GUI_SRC="${PACKAGED_GUI_CONFIG}"
    if [[ "${DRY_RUN}" != "true" ]]; then
      TMP_DIR="${STATE_DIR}/open_gui.$$"
      rm -rf -- "${TMP_DIR}"
      mkdir -m 700 -- "${TMP_DIR}" || exit 1
      GUI_FILE="${TMP_DIR}/gui.config"
    fi
    ERR_FILE="$(mktemp)"
    RESOLVED="$(A16_LAUNCH="${LAUNCH_FILE}" A16_WORLD="${WORLD}" A16_VARIANT="" A16_SITE="${SITE}" \
                A16_WORLD_INFO="${WORLD_INFO}" A16_CHECK_PLUGIN=0 A16_GUI_MODE="follow" A16_GUI_SRC="${GUI_SRC}" \
                A16_GUI_OUT="${GUI_FILE}" a16_resolve "${POSE[@]}" 2>"${ERR_FILE}")"
    RESOLVE_RC=$?
    ERR="$(sed -n 's/^ERROR: //p' "${ERR_FILE}" | head -n 1)"
    rm -f -- "${ERR_FILE}"
    if [[ ${RESOLVE_RC} -eq 0 ]]; then
      GUI_NOTE="$(a16_field "${RESOLVED}" GUI_NOTE)"
    else
      echo "WARNING: cannot place the start camera at ${CAMERA_AT} (${ERR:-error}); the GUI uses ${GUI_SRC} as it is." >&2
      GUI_FILE="${GUI_SRC}"
      GUI_NOTE="camera as in the file"
    fi
  fi
fi
GUI_ARGS=()
case "${GUI_MODE}:${USE_DEFAULT_GUI}" in
  verbatim:*) GUI_ARGS=(--gui-config "${GUI_FILE}"); GUI_CFG_LINE="config ${GUI_SRC}" ;;
  follow:*)   GUI_ARGS=(--gui-config "${GUI_FILE:-<copy of ${GUI_SRC}>}")
              GUI_CFG_LINE="config ${GUI_SRC} (${GUI_NOTE:-camera as in the file}; camera at ${CAMERA_AT})" ;;
  none:true)  GUI_CFG_LINE="Gazebo's default GUI config (--default-gui; far clip 1000 m)" ;;
  *)          GUI_CFG_LINE="Gazebo's default GUI config (no ${PACKAGED_GUI_CONFIG}; far clip 1000 m hides the horizon)" ;;
esac
echo "Gazebo GUI:   ${GUI_CFG_LINE}"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "Dry run, nothing started. Would run:"
  printf '  setsid gz sim -g'; (( ${#GUI_ARGS[@]} == 0 )) || printf ' %q' "${GUI_ARGS[@]}"; echo
  exit 0
fi

# ------------------------------------------------------------ the GUI, in its own process group
a16_gui_notice "Close the GUI with Ctrl+C in this terminal; the simulation keeps running." \
               "Do not close it with the window's X button: gz can crash, and WSL then writes a multi-GB crash dump."
setsid gz sim -g "${GUI_ARGS[@]}" &
GUI_PID=$!
a16_register_gui "${STATE_DIR}" "${GUI_PID}"
a16_record_pids "${GUI_PID}"
i=0
while kill -0 "${GUI_PID}" 2>/dev/null; do
  a16_nap 0.5
  (( ++i % 4 == 0 )) || continue
  a16_record_pids "${GUI_PID}"
  a16_check_memory "${GUI_PID}" "${LAUNCH_PGID}"
  if [[ -n "${WRAPPER_PID}" ]] && ! a16_is_wrapper "${WRAPPER_PID}"; then
    echo "The simulation stopped: closing the Gazebo GUI."
    exit 0
  fi
done
wait "${GUI_PID}"
RC=$?
a16_unregister_gui "${STATE_DIR}" "${GUI_PID}"
if a16_core_signal_rc "${RC}"; then
  EXPECT_DUMPS+=" ${GUI_PID}:$(date +%s)"
  echo "WARNING: the Gazebo GUI crashed (exit code ${RC}, signal $(( RC - 128 )))." >&2
  exit "${RC}"
elif (( RC == 137 )); then
  echo "The Gazebo GUI was closed (SIGKILL, e.g. by open_gui.sh --close)."
  exit 0
fi
echo "The Gazebo GUI exited (code ${RC})."
exit "${RC}"
