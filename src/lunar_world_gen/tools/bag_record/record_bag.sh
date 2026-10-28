#!/usr/bin/env bash
# Records a SLAM benchmark bag on the Apollo 16 world; the bags in ~/togo_assets/lunar/bags were made with it.
#   1. scripts/run_apollo16_sim.sh --headless at the route's level start pose, on a pseudo-terminal, in its own ROS
#      domain and GZ_PARTITION (never domain 29, which your own simulation and the SLAM container use);
#   2. at READY: a ground-truth bridge (/model/togo/pose as PoseArray) and ros2 bag record (MCAP, zstd);
#   3. bag_drive.py drives the route: a standstill until the rover has been still for 15 s (DLIO calibrates its IMU
#      at the start of every replay), the laps, a 5 s standstill; it also publishes /ground_truth/odom;
#   4. stops the recorder (SIGTERM to its process group), the bridge and the simulation (Ctrl+C to the launcher);
#   5. check_bag.py writes ground_truth.tum, check.json and check.png into the bag folder.
# The logs end up in BAG/recording/. Run from WSL with no other Gazebo running: about 1 min until READY, then about
# 6.5 min per lap. One lap out of a longer bag: tools/trim_bag.py (README section 5).
set -o pipefail
# Background jobs of a non-interactive bash start with SIGINT ignored, so the recorder and the bridge are stopped with
# SIGTERM (handled gracefully by ros2 bag record) to their process group (2026-10-27: the first recording ignored
# SIGINT, needed SIGKILL and a reindexed metadata.yaml). Job control (set -m) is no fix: it makes setsid fork.

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd -- "${HERE}/../.." && pwd)"
REPO="$(cd -- "${PKG}/../.." && pwd)"
BAGS="${HOME}/togo_assets/lunar/bags"
NAME="apollo16_loop_$(date +%Y%m%d_%H%M)"
ROUTE="${HERE}/route.json"
LAPS=2
DOMAIN=141
PARTITION=bag_record
DRY=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Records a benchmark bag: the rover drives the planned route while ros2 bag records the sensors and the ground truth.

Options:
  --name NAME      bag folder name under ${BAGS} (default apollo16_loop_<date>_<time>); must not exist yet
  --laps N         laps of the route (default ${LAPS})
  --route FILE     route.json from plan_route.py (default: the benchmark route next to this script)
  --domain D       ROS domain of the recording (default ${DOMAIN}; never 29)
  --dry-run        check the inputs and print the plan, start nothing
  -h, --help       this help
EOF
}

die() { echo "ERROR: $*" >&2; exit 2; }
need() { [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 needs a value"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --name) need "$@"; [[ "$2" =~ ^[A-Za-z0-9._-]+$ ]] || die "--name may only use letters, digits, '.', '_' and '-'"
            NAME="$2"; shift 2 ;;
    --laps) need "$@"; [[ "$2" =~ ^[1-9][0-9]*$ ]] || die "--laps must be a positive integer"; LAPS="$2"; shift 2 ;;
    --route) need "$@"; ROUTE="$2"; shift 2 ;;
    --domain) need "$@"; [[ "$2" =~ ^[0-9]+$ && "$2" != 29 ]] || die "--domain must be a number other than 29"
              DOMAIN="$2"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option '$1' (see --help)" ;;
  esac
done
[[ -f "${ROUTE}" ]] || die "route file not found: ${ROUTE}"
BAG="${BAGS}/${NAME}"
RUN="${BAGS}/.recording_${NAME}"              # logs while recording; moved to BAG/recording at the end
STATE="${APOLLO16_STATE_DIR:-/tmp/apollo16_sim-$(id -u)}"
read -r SX SY SZ SYAW < <(python3 -c "import json, sys; s = json.load(open(sys.argv[1]))['start']; print(s['x'], s['y'], s['z'], s['yaw'])" "${ROUTE}") \
  || die "no start pose in ${ROUTE}"
[[ -e "${BAG}" ]] && die "the bag folder exists already: ${BAG} (choose another --name)"

if [[ ${DRY} -eq 1 ]]; then
  echo "bag:        ${BAG}  (${LAPS} lap(s) of ${ROUTE})"
  echo "start pose: ${SX} ${SY} ${SZ} ${SYAW}"
  echo "isolation:  ROS_DOMAIN_ID=${DOMAIN} GZ_PARTITION=${PARTITION}"
  echo "  ${PKG}/scripts/run_apollo16_sim.sh --headless --pose ${SX} ${SY} ${SZ} ${SYAW}   # on a pty, until READY"
  echo "  ros2 run ros_gz_bridge parameter_bridge /model/togo/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"
  echo "  ros2 bag record -o ${BAG} -s mcap --storage-preset-profile zstd_fast <sensors, tf, odom, ground truth>"
  echo "  python3 ${HERE}/bag_drive.py --route ${ROUTE} --laps ${LAPS} --hold 15 --end-hold 5"
  echo "  python3 ${HERE}/check_bag.py ${BAG} ${ROUTE}"
  pgrep -f "^gz sim" > /dev/null && echo "NOTE: a Gazebo simulation is running now; a real recording would refuse to start"
  exit 0
fi

mkdir -p "${RUN}" "${BAGS}" || die "cannot create ${RUN}"
log() { echo "[$(date +%T)] $*" | tee -a "${RUN}/record.log"; }

export ROS_DOMAIN_ID=${DOMAIN} GZ_PARTITION=${PARTITION} ROS_HOME=${RUN}/ros_home
# shellcheck disable=SC1091
source "${PKG}/scripts/env.sh" --quiet
export ROS_DOMAIN_ID=${DOMAIN} GZ_PARTITION=${PARTITION}
# shellcheck disable=SC1091
source "${PKG}/scripts/apollo16_lib.sh"
CRASH="$(a16_crash_dir 2>/dev/null)"
log "bag ${BAG}; start pose ${SX} ${SY} ${SZ} ${SYAW}; laps ${LAPS}; domain ${DOMAIN}"

if pgrep -f "^gz sim" > /dev/null; then log "another Gazebo simulation is running: abort"; exit 1; fi
[[ -n "${CRASH}" ]] && ls -la "${CRASH}" > "${RUN}/crash_before.txt" 2>&1

# ------------------------------------------------------------------ 1. the launcher on a pty
rm -f "${RUN}/wrapper.fifo"; mkfifo "${RUN}/wrapper.fifo"
CMD="bash -lc 'export ROS_DOMAIN_ID=${DOMAIN} GZ_PARTITION=${PARTITION} ROS_HOME=${RUN}/ros_home_sim; cd ${REPO} && ./src/lunar_world_gen/scripts/run_apollo16_sim.sh --headless --pose ${SX} ${SY} ${SZ} ${SYAW}'"
setsid timeout -s KILL $((1200 + 600 * LAPS)) script -qfe -c "${CMD}" "${RUN}/wrapper.log" 0<>"${RUN}/wrapper.fifo" > /dev/null 2>&1 &
SCRIPT_PID=$!
T0=$(date +%s.%N)
stop_sim() {
  log "stopping the launcher (^C)"
  printf '\003' > "${RUN}/wrapper.fifo"
  for _ in $(seq 1 120); do kill -0 "${SCRIPT_PID}" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "${SCRIPT_PID}" 2>/dev/null; then
    log "launcher still running after 60 s: second ^C"; printf '\003' > "${RUN}/wrapper.fifo"; sleep 10
  fi
  pgrep -af "^gz sim" > "${RUN}/gz_left.txt" && log "WARNING: gz processes left: $(cat "${RUN}/gz_left.txt")"
}

SESSION=""
for _ in $(seq 1 300); do
  SESSION=$(find "${STATE}" -maxdepth 2 -name session -newer "${RUN}/wrapper.fifo" 2>/dev/null | head -n 1)
  [[ -n "${SESSION}" ]] && break
  sleep 0.2
done
[[ -n "${SESSION}" ]] || { log "no session file: abort"; stop_sim; exit 1; }
READY_FILE=$(awk -F= '$1=="ready_file"{print $2}' "${SESSION}")
log "session ${SESSION}, ready marker ${READY_FILE}"
RESULT=""
for _ in $(seq 1 6000); do
  if [[ -f "${READY_FILE}" ]]; then RESULT=ready; break; fi
  if [[ -f "${READY_FILE}.not_ready" ]]; then RESULT=not_ready; break; fi
  kill -0 "${SCRIPT_PID}" 2>/dev/null || { RESULT=exited; break; }
  sleep 0.1
done
log "READY result: ${RESULT:-timeout} after $(awk -v a="$(date +%s.%N)" -v b="${T0}" 'BEGIN { printf "%.1f", a - b }') s"
[[ "${RESULT}" == ready ]] || { stop_sim; exit 1; }

# ------------------------------------------------------------------ 2. ground-truth bridge (PoseArray)
setsid ros2 run ros_gz_bridge parameter_bridge "/model/togo/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V" \
  --ros-args -r __node:=bag_gt_bridge > "${RUN}/gt_bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 3

# ------------------------------------------------------------------ 3. recorder
TOPICS=(/husky/sensors/seyond/points /husky/sensors/imu_0/data_raw /clock /tf /tf_static
        /platform_velocity_controller/odom /platform_velocity_controller/cmd_vel /joint_states
        /ground_truth/odom /model/togo/pose)
setsid ros2 bag record -o "${BAG}" -s mcap --storage-preset-profile zstd_fast "${TOPICS[@]}" > "${RUN}/record.out" 2>&1 &
REC_PID=$!
for _ in $(seq 1 150); do
  n=$(grep -c -E "Subscribed to topic '/(husky/sensors/seyond/points|husky/sensors/imu_0/data_raw|clock|tf_static)'" "${RUN}/record.out" 2>/dev/null)
  [[ "${n:-0}" -ge 4 ]] && break
  sleep 0.2
done
log "recorder pid ${REC_PID} subscribed to $(grep -c "Subscribed to topic" "${RUN}/record.out") topics"
sleep 2

# ------------------------------------------------------------------ 4. drive
log "driver: standstill, ${LAPS} lap(s), standstill"
timeout -s INT $((300 + 600 * LAPS)) python3 "${HERE}/bag_drive.py" --route "${ROUTE}" --out "${RUN}" --laps "${LAPS}" \
  --hold 15 --end-hold 5 --max-drive $((480 * LAPS)) > "${RUN}/drive.out" 2>&1     # a lap took about 375 s
DRV_RC=$?
log "driver exit ${DRV_RC}: $(tail -n 2 "${RUN}/drive.out" | tr '\n' ' ')"
sleep 1

# ------------------------------------------------------------------ 5. stop recorder, bridge, sim
stop_pid() {   # pid name: SIGTERM (graceful for ros2 bag record), then SIGINT, then SIGKILL; to the process group
  local pid=$1 name=$2 sig
  for sig in TERM INT KILL; do
    kill -"${sig}" -- "-${pid}" 2>/dev/null || kill -"${sig}" "${pid}" 2>/dev/null
    for _ in $(seq 1 60); do kill -0 "${pid}" 2>/dev/null || { log "${name} stopped (SIG${sig})"; return 0; }; sleep 0.5; done
    log "${name} still running 30 s after SIG${sig}"
  done
  return 1
}
stop_pid "${REC_PID}" recorder
if [[ ! -f "${BAG}/metadata.yaml" ]]; then
  log "no metadata.yaml: rebuilding it (ros2 bag reindex)"
  ros2 bag reindex -s mcap "${BAG}" >> "${RUN}/record.out" 2>&1
fi
stop_pid "${BRIDGE_PID}" "ground-truth bridge"
stop_sim
if [[ -n "${CRASH}" ]]; then
  ls -la "${CRASH}" > "${RUN}/crash_after.txt" 2>&1
  log "crash dumps before/after: $(($(wc -l < "${RUN}/crash_before.txt") - 3)) / $(($(wc -l < "${RUN}/crash_after.txt") - 3))"
fi
ros2 bag info "${BAG}" > "${RUN}/bag_info.txt" 2>&1

# ------------------------------------------------------------------ 6. check, ground truth, logs into the bag folder
log "checking the bag (check_bag.py): ground_truth.tum, check.json, check.png"
python3 "${HERE}/check_bag.py" "${BAG}" "${ROUTE}" > "${RUN}/check.out" 2>&1 || log "WARNING: check_bag.py failed, see check.out"
du -sh "${BAG}" | tee -a "${RUN}/record.log"
log "done (driver rc ${DRV_RC})"
rm -f "${RUN}/wrapper.fifo"
if [[ -d "${BAG}" ]]; then
  mv "${RUN}" "${BAG}/recording" && echo "Logs: ${BAG}/recording/"
fi
