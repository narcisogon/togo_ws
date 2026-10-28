#!/usr/bin/env bash
# Build the patched classic-Bullet gz-physics engine plugin for the lunar world
# (docs/DESIGN.md section 10). No sudo; nothing is installed system-wide.
#
#   patches (patch_bullet_lunar.py; unified diffs for reference in *.patch):
#     1. velcmd        SetJointVelocityCommand keeps the parent's angular velocity
#                      (rover baseline fix of the artificial yaw damper)
#     2. mesh-friction mesh collisions honour SDF <surface><friction><ode><mu>/<mu2>
#
# usage: build_lunar_bullet_plugin.sh [--out FILE] [--work DIR] [--tarball FILE] [--jobs N]
#   --out      installed plugin (default ~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so,
#              = cfg.paths.physics_plugin)
#   --work     build directory (default ~/togo_assets/lunar/build/gz_physics_bullet_lunar)
#   --tarball  local gz-physics7_7.6.0.tar.gz (default: <work>/gz-physics7_7.6.0.tar.gz, downloaded
#              from GitHub if missing)
#   --jobs     parallel compile jobs (default 4; the compile runs under nice -n 10)
#
# Use the result from a world SDF by absolute path:
#   <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system">
#     <engine><filename>/home/nargon/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so</filename></engine>
#   </plugin>
# gz sim -v 4 then logs "Loaded [gz::physics::bullet::Plugin] from library [...lunar...]" and, per
# mesh collision, "[lunar patch] mesh collision [<name>]: friction mu <mu>, mu2 <mu2>".
set -euo pipefail

VER=7.6.0
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${HOME}/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so"
WORK="${HOME}/togo_assets/lunar/build/gz_physics_bullet_lunar"
TARBALL=""
JOBS=4
URL="https://github.com/gazebosim/gz-physics/archive/refs/tags/gz-physics7_${VER}.tar.gz"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --work) WORK="$2"; shift 2 ;;
    --tarball) TARBALL="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    -h|--help) sed -n 2,26p "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

log() { echo "[build_lunar_bullet_plugin] $*"; }
T0=$(date +%s)

# ---------------------------------------------------------------- version check
VENDOR_LIB=/opt/ros/jazzy/opt/gz_physics_vendor/lib
if [[ ! -e "${VENDOR_LIB}/libgz-physics7.so.${VER}" ]]; then
  echo "installed gz-physics is not ${VER} (no ${VENDOR_LIB}/libgz-physics7.so.${VER}); the patches target ${VER}" >&2
  ls "${VENDOR_LIB}"/libgz-physics7.so.* >&2 || true
  exit 1
fi

# ---------------------------------------------------------------- source
mkdir -p "${WORK}" "$(dirname "${OUT}")"
[[ -n "${TARBALL}" ]] || TARBALL="${WORK}/gz-physics7_${VER}.tar.gz"
if [[ ! -s "${TARBALL}" ]]; then
  log "downloading ${URL}"
  curl -sSfL --retry 3 -o "${TARBALL}.part" "${URL}"
  mv "${TARBALL}.part" "${TARBALL}"
fi
SHA=$(sha256sum "${TARBALL}" | cut -d' ' -f1)
log "source ${TARBALL} sha256 ${SHA}"
SRC="${WORK}/src/gz-physics-gz-physics7_${VER}"
rm -rf "${WORK}/src"
mkdir -p "${WORK}/src"
tar xzf "${TARBALL}" -C "${WORK}/src"
[[ -d "${SRC}/bullet/src" ]] || { echo "unexpected tarball layout (no ${SRC}/bullet/src)" >&2; exit 1; }

# ---------------------------------------------------------------- patches
python3 "${HERE}/patch_bullet_lunar.py" "${SRC}" | tee "${WORK}/patch.log"
(cd "${WORK}/src" && for f in bullet/src/JointFeatures.cc bullet/src/Base.hh bullet/src/SDFFeatures.cc bullet/src/ShapeFeatures.cc; do
   diff -u "gz-physics-gz-physics7_${VER}/${f}.orig" "gz-physics-gz-physics7_${VER}/${f}" || true
 done) > "${WORK}/lunar_applied.patch"
NMARK=$(grep -c "\[lunar patch\]" "${SRC}"/bullet/src/{JointFeatures.cc,Base.hh,SDFFeatures.cc,ShapeFeatures.cc} | awk -F: '{s+=$2} END {print s}')
[[ "${NMARK}" -ge 4 ]] || { echo "patch markers missing (${NMARK})" >&2; exit 1; }

# ---------------------------------------------------------------- configure + build (bullet plugin only)
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash      # build against the vendor libraries gz sim loads
set -u
BUILD="${WORK}/build"
rm -rf "${BUILD}"
mkdir -p "${BUILD}"
log "cmake configure"
# CMake runs from the build folder: the gz-physics plugin CMakeLists (bullet, bullet-featherstone,
# dartsim, tpe) create their unversioned plugin symlinks with execute_process(create_symlink ...),
# which writes into the CURRENT directory. Started from elsewhere this left four dangling
# libgz-physics-*-plugin.so links in the caller's working directory (U8); in the build folder they are
# where gz-physics' own install rules expect them (PROJECT_BINARY_DIR).
(cd "${BUILD}" && cmake -S "${SRC}" -B "${BUILD}" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF) \
      > "${WORK}/cmake_configure.log" 2>&1 || { tail -40 "${WORK}/cmake_configure.log"; exit 1; }
log "make -j${JOBS} gz-physics7-bullet-plugin (nice -n 10)"
nice -n 10 make -C "${BUILD}" -j"${JOBS}" gz-physics7-bullet-plugin > "${WORK}/make.log" 2>&1 \
  || { tail -60 "${WORK}/make.log"; exit 1; }
LIB="${BUILD}/lib/libgz-physics7-bullet-plugin.so.${VER}"
[[ -f "${LIB}" ]] || { echo "build produced no ${LIB}" >&2; exit 1; }

# ---------------------------------------------------------------- install (atomic) + provenance
cp -L "${LIB}" "${OUT}.tmp"
mv -f "${OUT}.tmp" "${OUT}"
LIBSHA=$(sha256sum "${OUT}" | cut -d' ' -f1)
cat > "${OUT}.json" <<EOF
{
  "plugin": "${OUT}",
  "plugin_sha256": "${LIBSHA}",
  "gz_physics_version": "${VER}",
  "source_tarball": "${TARBALL}",
  "source_sha256": "${SHA}",
  "patches": ["velcmd", "mesh-friction"],
  "patch_script": "${HERE}/patch_bullet_lunar.py",
  "built_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "build_seconds": $(( $(date +%s) - T0 ))
}
EOF
log "linked libraries:"
ldd "${OUT}" | grep -E "gz-physics7|sdformat|Bullet|gz-common" || true
if ldd "${OUT}" | grep -q "not found"; then
  echo "unresolved libraries:" >&2; ldd "${OUT}" | grep "not found" >&2; exit 1
fi
log "installed ${OUT} ($(stat -c %s "${OUT}") bytes) in $(( $(date +%s) - T0 )) s"
