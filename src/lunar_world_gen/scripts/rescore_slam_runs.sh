#!/usr/bin/env bash
# Re-scores finished replay runs (scripts/replay_slam_bag.sh) with the current tools/slam_eval.py: new metrics.json,
# pictures and errors_vs_distance.csv from the data each run folder already holds (the recorded SLAM output,
# run_info.txt, resources.csv) and its bag's ground truth. Use it after the scoring changed, so that old and new
# runs are compared on the same terms. Nothing is replayed. Run from WSL.
set -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd -- "${HERE}/.." && pwd)"
RUNS="${HOME}/togo_assets/lunar/bags/slam_runs"

usage() {
  cat <<EOF
Usage: $(basename "$0") RUN_DIR...
       $(basename "$0") --label LABEL [--root DIR]

Re-scores replay runs with the current tools/slam_eval.py (metrics.json, map.png, profile.png, errors.png).
  RUN_DIR         a run folder (holds run_info.txt and slam_output/)
  --label LABEL   every run of that benchmark label under --root (default ${RUNS})
Then refresh a label's summary:  python3 tools/slam_compare.py --baseline LABEL --out ... --plot ...
EOF
}

die() { echo "ERROR: $*" >&2; exit 2; }
DIRS=()
LABEL=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --label) [[ -n "${2:-}" ]] || die "--label needs a value"; LABEL="$2"; shift 2 ;;
    --root) [[ -n "${2:-}" ]] || die "--root needs a value"; RUNS="$2"; shift 2 ;;
    -*) die "unknown option '$1' (see --help)" ;;
    *) DIRS+=("$1"); shift ;;
  esac
done
if [[ -n "${LABEL}" ]]; then
  while IFS= read -r info; do DIRS+=("$(dirname "${info}")"); done \
    < <(grep -lxF "label=${LABEL}" "${RUNS}"/*/run_info.txt 2>/dev/null | sort)
  [[ ${#DIRS[@]} -gt 0 ]] || die "no runs with label '${LABEL}' under ${RUNS}"
fi
[[ ${#DIRS[@]} -gt 0 ]] || { usage >&2; exit 2; }

# reading the run bags needs rosbag2_py
python3 -c "import rosbag2_py" 2>/dev/null || source /opt/ros/jazzy/setup.bash

kv() { awk -v k="$2" 'index($0, k "=") == 1 { print substr($0, length(k) + 2) }' "$1/run_info.txt"; }   # RUN KEY
ok=0
for RUN in "${DIRS[@]}"; do
  RUN="${RUN%/}"
  if [[ ! -f "${RUN}/run_info.txt" || ! -d "${RUN}/slam_output" ]]; then
    echo "skip ${RUN}: not a replay run folder (no run_info.txt or slam_output/)" >&2
    continue
  fi
  BAG="$(kv "${RUN}" bag)"
  LBL="$(kv "${RUN}" label)"
  if [[ ! -f "${BAG}/ground_truth.tum" ]]; then
    echo "skip ${RUN}: its bag has no ground_truth.tum (${BAG})" >&2
    continue
  fi
  ARGS=(--out "${RUN}" --label "${LBL:-$(basename "${RUN}")}" --run-info "${RUN}/run_info.txt" --bag "${BAG}")
  [[ -f "${RUN}/resources.csv" ]] && ARGS+=(--resources "${RUN}/resources.csv")
  [[ -f "${BAG}/segments.json" ]] && ARGS+=(--segments "${BAG}/segments.json")
  echo "=== $(basename "${RUN}")"
  if python3 "${PKG}/tools/slam_eval.py" "${RUN}/slam_output" "${BAG}/ground_truth.tum" "${ARGS[@]}" 2>&1 \
       | grep -v '^\[INFO\]'; then
    ok=$((ok + 1))
  fi
done
echo "Re-scored ${ok} of ${#DIRS[@]} runs."
[[ ${ok} -gt 0 ]]
