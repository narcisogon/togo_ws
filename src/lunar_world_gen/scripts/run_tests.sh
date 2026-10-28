#!/usr/bin/env bash
# Runs this package's tests with the right Python: the venv ~/.venvs/lunar_world (LUNAR_VENV overrides), which has
# everything the world-generator tests need. The test files are grouped by what they protect:
#   world  tests/test_world_*.py   generating the world: terrain, mesh, rocks, DTM, SDF, the build and its checks
#   sim    tests/test_sim_*.py     running it: run_apollo16_sim.sh, env.sh, open_gui.sh, the launch file, the
#                                  robot collision boxes, stopping Gazebo safely, the drive-test metrics
#   slam   tests/test_slam_*.py    the SLAM replay, its scoring, the ground-truth check and the benchmark scripts
#   docs   tests/test_docs.py      the README matches the scripts' --help, linked files exist, generated configs
#                                  (lunar DLIO copy, GUI camera) are up to date
# None of them starts Gazebo, ROS nodes or Docker. Run from WSL, from any folder.
set -o pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd -- "${HERE}/.." && pwd)"
VENV="${LUNAR_VENV:-${HOME}/.venvs/lunar_world}"
# the three slow files that 'quick' leaves out (measured 2026-10-27: 147 s, 55 s and 34 s of the 300 s total)
SLOW=(tests/test_world_build.py tests/test_sim_scripts.py tests/test_world_terrain.py)

usage() {
  cat <<EOF
Usage: $(basename "$0") [GROUP] [pytest options...]

GROUP:
  quick   (default) every group, without the three slow files: the end-to-end build (2.5 min), the launcher
          scripts run against real processes (1 min) and the terrain synthesis (0.5 min); about 1 minute
  world   generating the world (about 4 minutes)
  sim     running the simulation: scripts, launch file, robot collision, stopping Gazebo (about 1 minute)
  slam    SLAM replay, scoring, ground-truth check, benchmark scripts (seconds)
  docs    README and --help in step, links, generated configs (seconds)
  all     everything (about 5 minutes)
  --list  print the test files of every group

Anything after GROUP goes to pytest, e.g.  $(basename "$0") slam -k heading   or   $(basename "$0") all -x
EOF
}

files_of() {   # GROUP -> test files (relative to PKG)
  local g=$1 f out=()
  case "${g}" in
    world) out=(tests/test_world_*.py) ;;
    sim) out=(tests/test_sim_*.py) ;;
    slam) out=(tests/test_slam_*.py) ;;
    docs) out=(tests/test_docs.py) ;;
    all) out=(tests/test_*.py) ;;
    quick)
      for f in tests/test_*.py; do
        [[ " ${SLOW[*]} " == *" ${f} "* ]] || out+=("${f}")
      done ;;
    *) return 1 ;;
  esac
  printf '%s\n' "${out[@]}"
}

cd "${PKG}" || exit 2
GROUP="${1:-quick}"
case "${GROUP}" in
  -h|--help) usage; exit 0 ;;
  --list)
    for g in world sim slam docs; do echo "${g}:"; files_of "${g}" | sed 's/^/  /'; done
    echo "quick leaves out: ${SLOW[*]}"
    exit 0 ;;
  -*) GROUP=quick ;;             # pytest options only: run the quick set with them
  quick|world|sim|slam|docs|all) shift ;;
  *) echo "ERROR: unknown group '${GROUP}' (see --help)" >&2; exit 2 ;;
esac
mapfile -t FILES < <(files_of "${GROUP}")
[[ ${#FILES[@]} -gt 0 && -f "${FILES[0]}" ]] || { echo "ERROR: no test files for '${GROUP}'" >&2; exit 2; }

PY="${VENV}/bin/python"
if [[ ! -x "${PY}" ]]; then
  echo "WARNING: no venv at ${VENV} (README section 3.2); using the system python3, the world tests may fail" >&2
  PY=python3
fi
echo "Running ${#FILES[@]} test files (${GROUP}) with ${PY}"
exec "${PY}" -m pytest -q -p no:cacheprovider "$@" "${FILES[@]}"
