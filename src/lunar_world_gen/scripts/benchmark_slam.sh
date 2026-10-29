#!/usr/bin/env bash
# Benchmarks the DLIO SLAM on the standard one-lap bag: runs scripts/replay_slam_bag.sh --repeats times with the same
# config, no RViz and no truth-path tool (the score reads the bag's ground_truth.tum; the tool only draws it), then prints mean ± spread of every metric (tools/slam_compare.py); with --compare-to, the change
# against an earlier label. Replays of the same bag differ a little from run to run, so compare labels (several
# runs each), never single runs. Run from WSL; no simulation and no other SLAM may run at the same time.
set -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd -- "${HERE}/.." && pwd)"
RUNS="${HOME}/togo_assets/lunar/bags/slam_runs"
LABEL=""
CONFIG=""
BAG=""
REPEATS=3
COMPARE=""
PAUSE=10
EXTRA=()

usage() {
  cat <<EOF
Usage: $(basename "$0") --label NAME [options] [-- replay options]

Runs the SLAM replay --repeats times under one label and summarises the runs (mean ± spread per metric).

Options:
  --label NAME         name of this benchmark (e.g. baseline, frobenius); runs go to ${RUNS}/NAME__rN__<time>
  --config FILE        SLAM parameter file (default: the replay's default, the lunar config)
  --bag DIR            bag (default: the replay's default, the one-lap bag)
  --repeats N          number of replays (default ${REPEATS})
  --compare-to LABEL   also compare against the runs of an earlier label (LABEL = baseline)
  --pause S            pause between replays (default ${PAUSE})
  -h, --help           this help
Anything after -- goes to scripts/replay_slam_bag.sh (e.g. -- --rate 0.5).

Summary only, without new runs:  python3 tools/slam_compare.py --baseline LABEL [--candidate LABEL2]
EOF
}

die() { echo "ERROR: $*" >&2; exit 2; }
need() { [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || die "$1 needs a value"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --label) need "$@"; LABEL="$2"; shift 2 ;;
    --config) need "$@"; CONFIG="$2"; shift 2 ;;
    --bag) need "$@"; BAG="$2"; shift 2 ;;
    --repeats) need "$@"; [[ "$2" =~ ^[1-9][0-9]*$ ]] || die "--repeats must be a positive integer"; REPEATS="$2"; shift 2 ;;
    --compare-to) need "$@"; COMPARE="$2"; shift 2 ;;
    --pause) need "$@"; [[ "$2" =~ ^[0-9]+$ ]] || die "--pause must be whole seconds"; PAUSE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA=("$@"); break ;;
    *) die "unknown option '$1' (see --help)" ;;
  esac
done
[[ -n "${LABEL}" ]] || die "--label is required"
[[ "${LABEL}" =~ ^[A-Za-z0-9._-]+$ ]] || die "--label may only use letters, digits, '.', '_' and '-'"

ARGS=(--label "${LABEL}" --no-rviz --no-truth-path)
[[ -n "${CONFIG}" ]] && ARGS+=(--config "${CONFIG}")
[[ -n "${BAG}" ]] && ARGS+=(--bag "${BAG}")
trap 'echo; echo "Benchmark interrupted."; exit 130' INT TERM

ok=0
for i in $(seq 1 "${REPEATS}"); do
  name="${LABEL}__r${i}__$(date +%Y%m%d_%H%M%S)"
  echo "================ ${LABEL}: run ${i} of ${REPEATS} (${name})"
  if "${HERE}/replay_slam_bag.sh" "${ARGS[@]}" --name "${name}" "${EXTRA[@]}"; then
    ok=$((ok + 1))
  else
    echo "WARNING: run ${i} failed; continuing" >&2
  fi
  [[ ${i} -lt ${REPEATS} ]] && sleep "${PAUSE}"
done
echo "================ ${LABEL}: ${ok} of ${REPEATS} runs finished"
[[ ${ok} -gt 0 ]] || exit 1

if [[ -n "${COMPARE}" ]]; then
  out="${RUNS}/compare__${COMPARE}__vs__${LABEL}"
  python3 "${PKG}/tools/slam_compare.py" --baseline "${COMPARE}" --candidate "${LABEL}" --out "${out}.md" \
    --plot "${out}.png" && echo "Saved: ${out}.md, ${out}.png"
else
  out="${RUNS}/summary__${LABEL}"
  python3 "${PKG}/tools/slam_compare.py" --baseline "${LABEL}" --out "${out}.md" --plot "${out}.png" \
    && echo "Saved: ${out}.md, ${out}.png"
fi
