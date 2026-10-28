#!/bin/bash
# Build a patched classic-Bullet gz-physics engine plugin (no sudo, nothing installed system-wide).
# Tested 2026-10-25 in WSL2 Ubuntu 24.04 + ROS 2 Jazzy vendor gz packages (gz-sim 8.11.0,
# gz-physics 7.6.0): configure + build of the single target took ~20 s.
#
# usage: build_patched_bullet_plugin.sh <work_dir>
#   -> <work_dir>/plugin/libgz-physics7-bullet-yawfix-plugin.so
# Use it from a world SDF by absolute path:
#   <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'>
#     <engine><filename>/abs/path/libgz-physics7-bullet-yawfix-plugin.so</filename></engine>
#   </plugin>
# The launch log then shows: "Loaded [gz::physics::bullet::Plugin] from library [/abs/path/...]".
set -e
W=${1:?usage: $0 <work_dir>}
HERE=$(cd "$(dirname "$0")" && pwd)
VER=7.6.0   # must match the installed gz-physics (check: ls /opt/ros/jazzy/opt/gz_physics_vendor/lib/libgz-physics7.so.*)
mkdir -p "$W/src" "$W/plugin"
cd "$W/src"
[ -f gz-physics7_$VER.tar.gz ] || curl -sSfL -o gz-physics7_$VER.tar.gz \
  https://github.com/gazebosim/gz-physics/archive/refs/tags/gz-physics7_$VER.tar.gz
[ -d gz-physics-gz-physics7_$VER ] || tar xzf gz-physics7_$VER.tar.gz
python3 "$HERE/patch_bullet_velcmd.py" "$W/src/gz-physics-gz-physics7_$VER"
source /opt/ros/jazzy/setup.bash          # build against the same vendor libraries gz sim loads
cmake -S "$W/src/gz-physics-gz-physics7_$VER" -B "$W/build" -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF > "$W/cmake_configure.log" 2>&1
make -C "$W/build" -j6 gz-physics7-bullet-plugin > "$W/make.log" 2>&1
cp -L "$W/build/lib/libgz-physics7-bullet-plugin.so.$VER" "$W/plugin/libgz-physics7-bullet-yawfix-plugin.so"
ldd "$W/plugin/libgz-physics7-bullet-yawfix-plugin.so" | grep -E "gz-physics7|sdformat|Bullet"
echo "built $W/plugin/libgz-physics7-bullet-yawfix-plugin.so"
