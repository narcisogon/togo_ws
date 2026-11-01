#!/usr/bin/env bash
# Builds OpenVINS (ov_core, ov_init, ov_msckf) in the SLAM container's workspace, as a separate ROS 2 process next
# to DLIO (GPL-3 code, never linked into the SLAM packages). Run from WSL in the repo:
#
#   ./src/lunar_world_gen/scripts/setup_openvins.sh            # clone (pinned commit), patch for Jazzy, build
#   ./src/lunar_world_gen/scripts/setup_openvins.sh --check    # only report what is there
#
# The clone lives at ~/slam_ws/src/open_vins INSIDE the container (not a mounted folder): it survives
# `docker compose start/stop dev` but not a container recreation, so run this again after `docker compose up`.
# What it changes in the upstream source (2025-11-30 master, which targets Humble): the ROS header names that
# Jazzy renamed (.h -> .hpp for image_transport, cv_bridge, tf2_geometry_msgs). The build dependency
# libceres-dev is in the Dockerfile; this script installs it if the running container predates that line.
# It also applies scripts/openvins_patch_msckf_stages.py (a DEBUG-level line of per-update feature stage counts
# in the MSCKF update, silent at the INFO verbosity of the replays) and scripts/openvins_patch_dark_mask.py (node
# parameters dark_mask_threshold / dark_mask_dilate_px: the rover's own shadow and the sky masked per frame) and
# scripts/openvins_patch_diverge_exit.py (diverge_velocity_mps: the node exits when it has diverged; the launch
# respawns it).
set -o pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b
URL=https://github.com/rpng/open_vins.git
dc() { env -u CYCLONEDDS_URI -u FASTRTPS_DEFAULT_PROFILES_FILE docker compose "$@"; }
cd "${REPO}" || exit 2
if [[ "${1:-}" == "--check" ]]; then
  dc exec -T dev bash -lc 'cd ~/slam_ws/src/open_vins 2>/dev/null && git log -1 --format="open_vins %h %ad" --date=short; ls ~/slam_ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf 2>/dev/null && echo "ov_msckf built" || echo "ov_msckf NOT built"'
  exit $?
fi
dc exec -T dev bash -lc "set -e
source /opt/ros/jazzy/setup.bash
dpkg -s libceres-dev > /dev/null 2>&1 || sudo apt-get install -q -y libceres-dev
cd ~/slam_ws/src
if [ ! -d open_vins ]; then git clone ${URL} open_vins; fi
cd open_vins && git fetch -q origin ${COMMIT} 2>/dev/null || true
git checkout -q ${COMMIT} 2>/dev/null || echo 'WARNING: pinned commit not found, building the checked-out revision'
grep -rl 'image_transport/image_transport.h>\|cv_bridge/cv_bridge.h>\|tf2_geometry_msgs/tf2_geometry_msgs.h>' ov_msckf/src ov_core/src ov_init/src ov_eval/src 2>/dev/null \
  | xargs -r sed -i -e 's#image_transport/image_transport.h>#image_transport/image_transport.hpp>#' \
                    -e 's#cv_bridge/cv_bridge.h>#cv_bridge/cv_bridge.hpp>#' \
                    -e 's#tf2_geometry_msgs/tf2_geometry_msgs.h>#tf2_geometry_msgs/tf2_geometry_msgs.hpp>#'
python3 /home/er4-user/ws/src/lunar_world_gen/scripts/openvins_patch_msckf_stages.py ov_msckf/src/update/UpdaterMSCKF.cpp
python3 /home/er4-user/ws/src/lunar_world_gen/scripts/openvins_patch_dark_mask.py ov_msckf/src/ros
python3 /home/er4-user/ws/src/lunar_world_gen/scripts/openvins_patch_diverge_exit.py ov_msckf/src/ros
cd ~/slam_ws
MAKEFLAGS=-j6 colcon build --executor sequential --symlink-install --packages-select ov_core ov_init ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON 2>&1 | tail -n 5
ls install/ov_msckf/lib/ov_msckf/run_subscribe_msckf && echo 'OpenVINS ready: ros2 run ov_msckf run_subscribe_msckf <estimator_config.yaml>'
"
