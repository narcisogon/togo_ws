# Operating modes and source map

Paths in this reference are relative to the repository root unless marked as
native or container paths. Verify current files before relying on this snapshot.

## Workspaces

| Environment | Source | Install / role |
| --- | --- | --- |
| Native WSL | Repository `src/` | `~/togo_ws_native_clean/install`; Gazebo/RViz |
| Docker `dev` | `/home/er4-user/ws/src` | `/home/er4-user/ws/install`; simulation packages |
| Docker `dev` | `/home/er4-user/slam_ws/src/lidarslam_ros2` | `/home/er4-user/slam_ws/install`; SLAM/Nav2 |

The Compose bind mounts share source with WSL, but native and Docker install
trees are distinct. Do not source Docker-generated `install/setup.bash` in
native WSL. Old prefixes chained into setup scripts can cause missing-package
warnings even when the new install exists. Inspect a clean shell and the active
`AMENT_PREFIX_PATH` before deleting or rebuilding anything.

## Choose the actual launcher

| Task | Current entrypoint |
| --- | --- |
| Native lunar simulation | `bash src/togo_sim_wsl.sh` |
| Gazebo Seyond DLIO + graph SLAM | `lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh` |
| Nav2 alongside an already running simulation/SLAM | `lidarslam_ros2/scripts/togo/run_nav2_only.sh` |
| Real live Husky/Seyond | Inspect `run_live_husky_real_dlio.sh` and its real-sensor parameter override |
| Bag replay | Inspect `run_replay_husky_real.sh`, the bag metadata, and timing audit tools |

There are conflicting operator documents. At skill creation,
`scripts/togo/README.md` describes `run_nav2_with_slam.sh` as a Nav2 launcher,
but that script actually launches DLIO and plays a hard-coded real-sensor bag.
Do not select it for live Gazebo based on its name. Root `scripts/run_slam.sh`
launches `seyond_live_slam.launch.py`, an alternative pipeline, rather than the
DLIO path above. Trace wrappers instead of assuming all similarly named scripts
are interchangeable. A documented runbook link may be absent; use actual code.

## Live simulation examples

These are operator commands in separate WSL terminals, from the repository
root. They start processes; use them when launching or testing is in scope.

Native simulation:

```bash
bash src/togo_sim_wsl.sh
```

The launcher defaults to `TOGO_WORLD=lunar_surface.world`, robot position
`(-2, 3, 2)`, and `TOGO_WSL_INSTALL=$HOME/togo_ws_native_clean/install`.
It exposes `TOGO_RVIZ`, `TOGO_GZ_GUI`, and `TOGO_USE_SIM_TIME` overrides.
Inspect it and the installed launch before invoking: the checked-in script has
a trailing Markdown fence, and its `use_sim_time` argument is not declared in
the checked-in `sim_gz.launch.py`. Record these discrepancies if they affect
the requested task; do not assume the runtime uses the same installed revision.

Container SLAM, with container RViz disabled when using native visualization:

```bash
docker compose up -d dev
docker compose exec -e SLAM_RVIZ=false dev bash \
  /home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
```

Nav2 in another terminal, after confirming odometry and TF are available:

```bash
docker compose exec -e RVIZ=false dev bash \
  /home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/run_nav2_only.sh
```

Run Compose commands on the WSL host. An `er4-user@...(docker)` prompt is
already inside the container. Do not attempt nested Docker as a startup fix.

## Sensor and output contract

| Signal | Simulation topic / frame |
| --- | --- |
| LiDAR input | `/husky/sensors/seyond/points`; `lidar3d_0_laser` |
| IMU input | `/husky/sensors/imu_0/data_raw`; `imu_0_link` |
| Frontend odometry | `/dlio/odometry`; `odom`, `base_link` |
| Frontend cloud | `/dlio/deskewed`; inspect message frame |
| Corrected global cloud | `/modified_map`, `/modified_map_timed` |
| Navigation grids | `/map`, `/local_hazard_map` |
| Rover velocity command | `/platform_velocity_controller/cmd_vel`; inspect actual type |

`seyond_dlio_slam.launch.py` currently subscribes DLIO to the raw LiDAR topic
and enables internal synthetic timing. A `_timed` launch argument does not
establish an active timing adapter. Synthetic scan period currently defaults to
0.0666666667 seconds; compare against sensor configuration and measured
simulation-time stamps. Check real PointCloud2 fields and native point-time
units before adapting hardware/replay data; avoid applying synthetic timing
or deskew twice. Verify sensor extrinsics against TF and URDF before changing
their parameter copies.

## Where to edit

- Frontend: `lidarslam_ros2/direct_lidar_inertial_odometry/`.
- Graph backend: `lidarslam_ros2/graph_based_slam/`.
- Shared simulation settings: `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml`.
- Active SLAM wiring: `lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py`.
- Hazard patches and global grid: `lidarslam_ros2/hazard_mapping/` and Nav2 launch.
- Local hazards, cloud helpers, navigation: `lidarslam_ros2/togo_navigation/`.
- Nav2: `togo_navigation/launch/rover_nav2.launch.py` and
  `togo_navigation/config/nav2_slam_params.yaml` under the SLAM tree.
- Simulation/clock/sensors: `src/togo/togo_gz/launch/`, `config/bridge.yaml`.
- Native RViz: `src/togo/togo_deploy/launch/robot_sensor_checkout.launch.py`.
- Terrain: `src/moon_mesh_generator/generate_lunar_heightmap.py`, then
  `generate_lunar_mesh.py`; outputs go into the generator directory and
  `src/togo/togo_gz/media/`. Inspect constants: generator comments currently
  disagree about terrain size and physics engine.

Launch/YAML changes may require refreshing the installed package depending on
symlink mode and how the launcher resolves resources. C++ changes require a
build in the owning workspace. Use `colcon build --packages-up-to` for the
affected package and its workspace dependencies after inspecting existing
build defaults. Keep native build/install/log paths separate from Docker's.
Confirm `ros2 pkg prefix` and effective parameters after rebuilding.
