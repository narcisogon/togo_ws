#!/usr/bin/env bash
# Replays a recorded simulation bag into the DLIO SLAM in the Docker 'dev' container, with SLAM up first:
#   1. starts the container (docker compose start dev; 'up dev -d' only if it does not exist) and in it the usual
#      lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh with COMBINED_PARAM_FILE = --config;
#   2. waits until dlio_odom_node is subscribed to the LiDAR and IMU topics and graph_based_slam to /dlio/odometry;
#   3. only then plays the bag (its own /clock drives sim time), so DLIO calibrates the IMU on the bag's opening
#      standstill, and records what SLAM publishes;
#   4. stops SLAM a few seconds after the bag ends; 5. scores it against the bag's ground truth (tools/slam_eval.py):
#      accuracy, DLIO's per-scan matching numbers, RAM and CPU per SLAM process (tools/proc_sampler.py), the hazard
#      mapper. scripts/benchmark_slam.sh repeats it and compares labels (tools/slam_compare.py).
# Ctrl+C stops everything. Run from WSL; no simulation and no other SLAM may run at the same time.
set -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd -- "${HERE}/.." && pwd)"
REPO="$(cd -- "${PKG}/../.." && pwd)"
SLAM_SCRIPT=/home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
PIDFILE=/tmp/replay_slam_bag.pid                       # inside the container
BAG="${HOME}/togo_assets/lunar/bags/apollo16_lap1_20261027"
CONFIG="${PKG}/config/slam/seyond_dlio_graph_lunar.yaml"
RUNS="${HOME}/togo_assets/lunar/bags/slam_runs"
NAME=""
LABEL=""
RATE=1.0
DURATION=""
RVIZ=true
RECORD=1
EVAL=1
START_DELAY=3
TAIL=5
READY_TIMEOUT=180
ISOLATE=1
TRUTH_PATH=1
GT_TOOL="${REPO}/lidarslam_ros2/scripts/togo/gazebo_gt_path_publisher.py"
DRY_RUN=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Replays a simulation bag into the DLIO SLAM in the Docker 'dev' container: SLAM starts first, the bag starts once
SLAM is subscribed (so DLIO calibrates the IMU on the bag's opening standstill), SLAM stops after the bag, and the
run is scored against the bag's ground truth.

Options:
  --bag DIR            bag to replay (default: ${BAG})
  --config FILE        SLAM parameter file, passed as COMBINED_PARAM_FILE: a path under REPO/src or
                       REPO/lidarslam_ros2 (translated to the container), or a container path
                       (default: config/slam/seyond_dlio_graph_lunar.yaml, DLIO gravity 1.62)
  --name NAME          run name; results go to ${RUNS}/NAME
                       (default: <bag>__<config>__<date_time>)
  --label LABEL        group label for tools/slam_compare.py (default: the run name)
  --rate R             playback rate (default ${RATE})
  --duration S         play only the first S seconds of the bag
  --no-rviz            no RViz window from the container
  --no-record          do not record the SLAM output (no scoring either)
  --no-eval            record the SLAM output but do not score it
  --start-delay S      wait after SLAM is subscribed, before the bag starts (default ${START_DELAY})
  --tail S             keep SLAM running after the bag ends (default ${TAIL})
  --ready-timeout S    give up if SLAM is not subscribed after S seconds (default ${READY_TIMEOUT})
  --no-truth-path      do not run lidarslam_ros2/scripts/togo/gazebo_gt_path_publisher.py (by default it runs from
                       before the bag starts and publishes the ground truth on /ground_truth/path, aligned to
                       DLIO's start, for the SLAM RViz)
  --output-in-place    let SLAM write its map / DEM where the config says (the live runs' output folder);
                       default: a copy of the config with map_save_dir and dem/output_dir pointed at
                       REPO/slam_output/replay_runs/NAME, so a replay never overwrites a live run's outputs
  --dry-run            print what would run, start nothing
  -h, --help           this help

Results in the run folder: metrics.json (accuracy, scan matching, RAM/CPU per process, hazard mapper, loop-closure
check), eval.png, resources.csv (RAM/CPU samples), slam.log (SLAM's console), play.log, odometry.tum, slam_output/
(bag of /dlio/odometry, /dlio/frontend_diagnostics, /hazard/diagnostics, /tf, /tf_static).
Compare configs: scripts/benchmark_slam.sh (repeats + tools/slam_compare.py).
EOF
}

die() { echo "ERROR: $*" >&2; exit 2; }
need() { [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 needs a value"; }
num() { [[ "$2" =~ ^[0-9]+([.][0-9]+)?$ ]] || die "$1 must be a number, got '$2'"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) need "$@"; BAG="$2"; shift 2 ;;
    --config) need "$@"; CONFIG="$2"; shift 2 ;;
    --name) need "$@"; NAME="$2"; shift 2 ;;
    --label) need "$@"; LABEL="$2"; shift 2 ;;
    --rate) need "$@"; num "$1" "$2"; RATE="$2"; shift 2 ;;
    --duration) need "$@"; num "$1" "$2"; DURATION="$2"; shift 2 ;;
    --start-delay) need "$@"; num "$1" "$2"; START_DELAY="$2"; shift 2 ;;
    --tail) need "$@"; num "$1" "$2"; TAIL="$2"; shift 2 ;;
    --ready-timeout) need "$@"; num "$1" "$2"; READY_TIMEOUT="$2"; shift 2 ;;
    --no-rviz) RVIZ=false; shift ;;
    --no-record) RECORD=0; EVAL=0; shift ;;
    --no-eval) EVAL=0; shift ;;
    --output-in-place) ISOLATE=0; shift ;;
    --no-truth-path) TRUTH_PATH=0; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option '$1' (see --help)" ;;
  esac
done

# Host path -> container path, following the mounts of REPO/docker-compose.yml (services dev / hw-dev).
to_container() {
  local p="$1"
  case "${p}" in
    /home/er4-user/*) echo "${p}"; return 0 ;;
  esac
  [[ -f "${p}" ]] || return 1
  p="$(realpath -- "${p}")"
  case "${p}" in
    "${REPO}"/src/*) echo "/home/er4-user/ws/src/${p#"${REPO}"/src/}" ;;
    "${REPO}"/lidarslam_ros2/*) echo "/home/er4-user/slam_ws/src/lidarslam_ros2/${p#"${REPO}"/lidarslam_ros2/}" ;;
    "${REPO}"/slam_output/*) echo "/home/er4-user/slam_ws/src/lidarslam_ros2/output/${p#"${REPO}"/slam_output/}" ;;
    *) return 1 ;;
  esac
}

# The run's own copy of the config: map_save_dir and dem/output_dir pointed at the run's output folder.
derive_config() {   # src dst out_container
  sed -E -e "s|^([[:space:]]*map_save_dir:)[[:space:]]*[^[:space:]#]+|\1 $3|" \
         -e "s|^([[:space:]]*dem/output_dir:)[[:space:]]*[^[:space:]#]+|\1 $3/lunar_dem|" "$1" > "$2"
  grep -q -E "^[[:space:]]*(map_save_dir: $3|dem/output_dir: $3/lunar_dem)( |$)" "$2"
}

[[ -f "${BAG}/metadata.yaml" ]] || die "not a bag (no metadata.yaml): ${BAG}"
GT="${BAG}/ground_truth.tum"
if [[ ! -f "${GT}" ]]; then
  [[ ${EVAL} -eq 1 ]] && echo "NOTE: ${GT} not found: the run is recorded but not scored" >&2
  EVAL=0
fi
CONFIG_C="$(to_container "${CONFIG}")" || die "--config must be an existing file under ${REPO}/src or ${REPO}/lidarslam_ros2 (the folders the container mounts), or a container path: ${CONFIG}"
if [[ -z "${NAME}" ]]; then
  cfg="$(basename -- "${CONFIG}")"
  NAME="$(basename -- "${BAG}")__${cfg%.yaml}__$(date +%Y%m%d_%H%M%S)"
fi
[[ -n "${LABEL}" ]] || LABEL="${NAME}"
RUN="${RUNS}/${NAME}"
OUT_HOST="${REPO}/slam_output/replay_runs/${NAME}"
OUT_C="/home/er4-user/slam_ws/src/lidarslam_ros2/output/replay_runs/${NAME}"
SLAM_CONFIG_C="${CONFIG_C}"
if [[ ${ISOLATE} -eq 1 ]]; then
  if [[ -f "${CONFIG}" ]]; then
    SLAM_CONFIG_C="${OUT_C}/config.yaml"
  else
    echo "NOTE: --config is a container path, so SLAM writes its outputs where that config says" >&2
    ISOLATE=0
  fi
fi
if [[ ${TRUTH_PATH} -eq 1 ]]; then
  if [[ ! -f "${GT_TOOL}" ]]; then
    echo "NOTE: ${GT_TOOL} not found: no /ground_truth/path" >&2
    TRUTH_PATH=0
  elif ! grep -q "name: /model/togo/pose" "${BAG}/metadata.yaml"; then
    echo "NOTE: the bag has no /model/togo/pose: no /ground_truth/path" >&2
    TRUTH_PATH=0
  fi
fi
PLAY_ARGS=(--rate "${RATE}" --disable-keyboard-controls)
[[ -n "${DURATION}" ]] && PLAY_ARGS+=(--playback-duration "${DURATION}")
REC_TOPICS=(/dlio/odometry /tf /tf_static /dlio/frontend_diagnostics /hazard/diagnostics)

echo "=========================================="
echo "SLAM bag replay"
echo "Bag:     ${BAG}"
echo "Config:  ${CONFIG_C}"
if [[ ${ISOLATE} -eq 1 ]]; then
  echo "SLAM out: ${OUT_HOST}   (map + DEM; your live-run output folder is not touched)"
else
  echo "SLAM out: where the config says (--output-in-place)"
fi
echo "Run:     ${RUN}"
echo "Rate:    ${RATE}${DURATION:+   (first ${DURATION} s only)}   RViz: ${RVIZ}"
echo "=========================================="
if [[ ${DRY_RUN} -eq 1 ]]; then
  echo "Dry run, nothing started. Would run:"
  [[ ${ISOLATE} -eq 1 ]] && echo "  copy ${CONFIG} -> ${OUT_HOST}/config.yaml with map_save_dir: ${OUT_C} and dem/output_dir: ${OUT_C}/lunar_dem"
  echo "  (cd ${REPO} && docker compose start dev)   # 'up dev -d' only if the container does not exist"
  echo "  (cd ${REPO} && docker compose exec -T -e COMBINED_PARAM_FILE=${SLAM_CONFIG_C} -e SLAM_RVIZ=${RVIZ} dev bash -c 'echo \$\$ > ${PIDFILE}; exec bash ${SLAM_SCRIPT}') > ${RUN}/slam.log"
  echo "  python3 ${PKG}/tools/slam_replay_check.py wait --timeout ${READY_TIMEOUT}; sleep ${START_DELAY}"
  [[ ${RECORD} -eq 1 ]] && echo "  ros2 bag record -s sqlite3 -o ${RUN}/slam_output ${REC_TOPICS[*]}"
  [[ ${TRUTH_PATH} -eq 1 ]] && echo "  python3 ${GT_TOOL} --ros-args -p use_sim_time:=true   # /ground_truth/path"
  echo "  ros2 bag play ${BAG} ${PLAY_ARGS[*]}"
  echo "  sleep ${TAIL}; stop recorder (SIGTERM); stop SLAM (SIGINT in the container)"
  echo "  python3 ${PKG}/tools/proc_sampler.py --root-cmd run_live_seyond_dlio_slam.sh --out ${RUN}/resources.csv   # during the bag"
  [[ ${EVAL} -eq 1 ]] && echo "  python3 ${PKG}/tools/slam_eval.py ${RUN}/slam_output ${GT} --out ${RUN} --resources ${RUN}/resources.csv"
  exit 0
fi

# ------------------------------------------------------------------ environment (domain 29, CycloneDDS)
# shellcheck disable=SC1091
source "${PKG}/scripts/env.sh" --quiet || die "could not load ${PKG}/scripts/env.sh"
if [[ "${ROS_DOMAIN_ID}" != 29 ]]; then
  echo "NOTE: ROS_DOMAIN_ID was ${ROS_DOMAIN_ID}; using 29, the domain docker-compose.yml gives the container" >&2
  export ROS_DOMAIN_ID=29
fi
command -v docker > /dev/null || die "docker not found in WSL"
# docker compose substitutes CYCLONEDDS_URI and FASTRTPS_DEFAULT_PROFILES_FILE from this shell into the container's
# volume list. env.sh sets CYCLONEDDS_URI to a file:// URI, which compose rejects ("too many colons"); without the
# two variables compose uses the repo's config files, as from a plain WSL terminal.
dc() { (cd "${REPO}" && env -u CYCLONEDDS_URI -u FASTRTPS_DEFAULT_PROFILES_FILE docker compose "$@"); }
[[ -e "${RUN}" ]] && die "run folder exists already: ${RUN} (use another --name)"
[[ ${ISOLATE} -eq 1 && -e "${OUT_HOST}" ]] && die "SLAM output folder exists already: ${OUT_HOST} (use another --name)"

SLAM_CLIENT="" REC_PID="" PLAY_PID="" GT_PID="" SAMPLER_PID="" CLEANED=0
stop_group() {   # pid name: SIGTERM to its process group (background jobs of a script ignore SIGINT), then SIGKILL
  local pid=$1 name=$2
  kill -0 "${pid}" 2>/dev/null || return 0
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null
  for _ in $(seq 1 40); do kill -0 "${pid}" 2>/dev/null || return 0; sleep 0.5; done
  echo "WARNING: ${name} still running 20 s after SIGTERM: SIGKILL" >&2
  kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null
}
stop_slam() {
  [[ -n "${SLAM_CLIENT}" ]] && kill -0 "${SLAM_CLIENT}" 2>/dev/null || return 0
  echo "Stopping SLAM in the container..."
  local sig
  for sig in INT TERM; do
    dc exec -T dev bash -c "p=\$(cat ${PIDFILE} 2>/dev/null) && kill -${sig} \"\$p\" 2>/dev/null"
    for _ in $(seq 1 60); do kill -0 "${SLAM_CLIENT}" 2>/dev/null || { echo "SLAM stopped."; return 0; }; sleep 0.5; done
    echo "WARNING: SLAM still running 30 s after SIG${sig}" >&2
  done
  echo "WARNING: SLAM did not stop; look with: docker compose exec dev ps -ef" >&2
}
cleanup() {
  [[ ${CLEANED} -eq 1 ]] && return 0
  CLEANED=1
  [[ -n "${PLAY_PID}" ]] && stop_group "${PLAY_PID}" "bag play"
  [[ -n "${REC_PID}" ]] && stop_group "${REC_PID}" "recorder"
  [[ -n "${GT_PID}" ]] && stop_group "${GT_PID}" "ground-truth path publisher"
  [[ -n "${SAMPLER_PID}" ]] && stop_group "${SAMPLER_PID}" "resource sampler"
  stop_slam
  return 0
}
trap 'echo; echo "Interrupted: stopping everything"; cleanup; exit 130' INT TERM
trap cleanup EXIT

# ------------------------------------------------------------------ 1. nothing else on domain 29, container up
echo "[1/5] Checking that no simulation, bag or SLAM is running on ROS domain 29..."
python3 "${PKG}/tools/slam_replay_check.py" precheck || exit 1
mkdir -p "${RUN}"
if [[ ${ISOLATE} -eq 1 ]]; then
  mkdir -p "${OUT_HOST}"
  derive_config "${CONFIG}" "${OUT_HOST}/config.yaml" "${OUT_C}" || \
    echo "NOTE: ${CONFIG} sets neither map_save_dir nor dem/output_dir; SLAM uses its defaults for them" >&2
  cp -- "${OUT_HOST}/config.yaml" "${RUN}/config.yaml"
elif [[ -f "${CONFIG}" ]]; then
  cp -- "${CONFIG}" "${RUN}/config.yaml"
fi
{
  echo "bag=${BAG}"; echo "config=${SLAM_CONFIG_C}"; echo "config_source=${CONFIG}"; echo "rate=${RATE}"
  echo "duration=${DURATION:-all}"; echo "slam_output=$([[ ${ISOLATE} -eq 1 ]] && echo "${OUT_HOST}" || echo "as configured")"
  echo "label=${LABEL}"
  echo "config_sha1=$([[ -f "${CONFIG}" ]] && sha1sum "${CONFIG}" | cut -c1-12)"   # the settings, not the run's copy
  echo "host_cpu=$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2- | xargs) x $(nproc)"
  echo "started=$(date -Iseconds)"
} > "${RUN}/run_info.txt"
# 'start' never recreates the container (its SLAM build lives in the container); 'up' only if there is none yet.
case "$(dc ps -a --format '{{.State}}' dev 2>/dev/null)" in
  running) echo "[1/5] The 'dev' container is running." ;;
  "") echo "[1/5] Creating the 'dev' container (docker compose up dev -d)..."
      dc up dev -d 2>&1 | sed 's/^/      /' || die "docker compose up dev -d failed" ;;
  *) echo "[1/5] Starting the 'dev' container..."
     dc start dev 2>&1 | sed 's/^/      /' || die "docker compose start dev failed" ;;
esac
dc exec -T dev test -f "${SLAM_SCRIPT}" || die "not found in the container: ${SLAM_SCRIPT}"
dc exec -T dev test -f "${SLAM_CONFIG_C}" || die "not found in the container: ${SLAM_CONFIG_C}"
# which DLIO build is measured: the binary's modification time in the container
echo "dlio_build=$(dc exec -T dev bash -c 'b=$(find /home/er4-user/slam_ws/install -name dlio_odom_node -type f 2>/dev/null | head -n 1); [ -n "$b" ] && date -r "$b" -Iseconds')" >> "${RUN}/run_info.txt"

# ------------------------------------------------------------------ 2. SLAM, then wait until it is subscribed
echo "[2/5] Starting SLAM in the container (console: ${RUN}/slam.log)..."
dc exec -T -e COMBINED_PARAM_FILE="${SLAM_CONFIG_C}" -e SLAM_RVIZ="${RVIZ}" dev \
  bash -c "echo \$\$ > ${PIDFILE}; exec bash ${SLAM_SCRIPT}" > "${RUN}/slam.log" 2>&1 &
SLAM_CLIENT=$!
if ! python3 "${PKG}/tools/slam_replay_check.py" wait --timeout "${READY_TIMEOUT}" --watch-pid "${SLAM_CLIENT}"; then
  echo "---- last lines of ${RUN}/slam.log:" >&2
  tail -n 25 "${RUN}/slam.log" >&2
  exit 1
fi
sleep "${START_DELAY}"

# ------------------------------------------------------------------ 3. record SLAM's output, play the bag
if [[ ${RECORD} -eq 1 ]]; then
  setsid ros2 bag record -s sqlite3 -o "${RUN}/slam_output" "${REC_TOPICS[@]}" > "${RUN}/record.log" 2>&1 &
  REC_PID=$!
  for _ in $(seq 1 60); do grep -q "Subscribed to topic '/dlio/odometry'" "${RUN}/record.log" 2>/dev/null && break; sleep 0.25; done
  grep -q "Subscribed to topic '/dlio/odometry'" "${RUN}/record.log" || echo "WARNING: the recorder has not subscribed to /dlio/odometry yet" >&2
fi
if [[ ${TRUTH_PATH} -eq 1 ]]; then
  # Must run before the bag starts: it aligns the ground truth to DLIO's first pose from its last 10 s of poses.
  setsid python3 "${GT_TOOL}" --ros-args -p use_sim_time:=true > "${RUN}/truth_path.log" 2>&1 &
  GT_PID=$!
  for _ in $(seq 1 40); do grep -q "pose index" "${RUN}/truth_path.log" 2>/dev/null && break; sleep 0.25; done
  sleep 1
  echo "      ground truth for RViz: /ground_truth/path (gazebo_gt_path_publisher.py; log: ${RUN}/truth_path.log)"
fi
# RAM and CPU of every SLAM process (they are visible from WSL: the container runs with pid: host)
setsid python3 "${PKG}/tools/proc_sampler.py" --root-cmd run_live_seyond_dlio_slam.sh --out "${RUN}/resources.csv" \
  --interval 0.5 > "${RUN}/sampler.log" 2>&1 &
SAMPLER_PID=$!
sleep 1
echo "[3/5] Playing the bag ($(awk '/duration:/ {getline; print $2 / 1e9; exit}' "${BAG}/metadata.yaml" | xargs printf '%.0f') s at rate ${RATE}${DURATION:+, first ${DURATION} s})..."
T_PLAY=$(date +%s)
echo "play_start=$(date +%s.%N)" >> "${RUN}/run_info.txt"
setsid ros2 bag play "${BAG}" "${PLAY_ARGS[@]}" > "${RUN}/play.log" 2>&1 &
PLAY_PID=$!
shown_bias=0
next_note=$((T_PLAY + 60))
while kill -0 "${PLAY_PID}" 2>/dev/null; do
  sleep 1
  if [[ ${shown_bias} -eq 0 ]] && grep -q "Accel biases" "${RUN}/slam.log" 2>/dev/null; then
    echo "      DLIO IMU calibration: $(grep -m1 -o 'Accel biases.*' "${RUN}/slam.log")   (last value near 0 = gravity config right)"
    shown_bias=1
  fi
  if [[ ${shown_gt:-0} -eq 0 && -n "${GT_PID}" ]] && grep -q "Offset applied" "${RUN}/truth_path.log" 2>/dev/null; then
    echo "      ground truth aligned to DLIO's start: $(grep -m1 -o 'Offset applied.*' "${RUN}/truth_path.log" | tr -s ' ')"
    shown_gt=1
  fi
  if ! kill -0 "${SLAM_CLIENT}" 2>/dev/null; then
    echo "ERROR: SLAM exited during the replay; see ${RUN}/slam.log" >&2
    break
  fi
  if [[ $(date +%s) -ge ${next_note} ]]; then
    echo "      ... $(( $(date +%s) - T_PLAY )) s"
    next_note=$((next_note + 60))
  fi
done
wait "${PLAY_PID}" 2>/dev/null
PLAY_PID=""
echo "play_end=$(date +%s.%N)" >> "${RUN}/run_info.txt"
echo "[3/5] Bag finished after $(( $(date +%s) - T_PLAY )) s."
[[ ${shown_bias} -eq 0 ]] && echo "NOTE: no 'Accel biases' line from DLIO in slam.log" >&2

# ------------------------------------------------------------------ 4. stop
echo "[4/5] Letting SLAM finish for ${TAIL} s, then stopping it..."
sleep "${TAIL}"
[[ -n "${REC_PID}" ]] && { stop_group "${REC_PID}" "recorder"; REC_PID=""; }
[[ -n "${GT_PID}" ]] && { stop_group "${GT_PID}" "ground-truth path publisher"; GT_PID=""; }
[[ -n "${SAMPLER_PID}" ]] && { stop_group "${SAMPLER_PID}" "resource sampler"; SAMPLER_PID=""; }
stop_slam
echo "finished=$(date -Iseconds)" >> "${RUN}/run_info.txt"

# ------------------------------------------------------------------ 5. score
if [[ ${EVAL} -eq 1 ]]; then
  echo "[5/5] Scoring against ${GT}:"
  EVAL_ARGS=(--out "${RUN}" --label "${LABEL}" --resources "${RUN}/resources.csv" --run-info "${RUN}/run_info.txt"
             --bag "${BAG}")
  [[ -f "${BAG}/segments.json" ]] && EVAL_ARGS+=(--segments "${BAG}/segments.json")
  python3 "${PKG}/tools/slam_eval.py" "${RUN}/slam_output" "${GT}" "${EVAL_ARGS[@]}" 2>&1 | grep -v '^\[INFO\]' | sed 's/^/      /'
else
  echo "[5/5] Not scored (--no-eval / --no-record / no ground truth)."
fi
echo "Done: ${RUN}"
