# Local changes from upstream

This file tracks functional changes in this workspace relative to the public
`narcisogon/togo_ws` repository. Dependency restoration, downloaded source
trees, build products, bags, logs, and generated SLAM output are intentionally
excluded.

## Baseline

- Upstream: `https://github.com/narcisogon/togo_ws.git`
- Compared commit: `f649dd2fc106e1a47e39bb16a21c6eae01199fc0`
- Upstream commit date: 2026-08-27
- Comparison date: 2026-09-30
- Important: this workspace was provided without its original `.git`
  metadata. The baseline above is therefore a current upstream comparison,
  not proof of the exact commit from which every local change originated.

## Current architecture

- Gazebo and the primary RViz instance run natively in WSL / WSLg so OpenGL
  uses the Windows GPU through Mesa D3D12.
- SLAM and Nav2 run in Docker.
- Native and Docker ROS processes use CycloneDDS, ROS domain 29, localhost
  discovery, and Docker host networking.
- Native ROS builds use `~/togo_ws_native_clean` for build, install, and log
  output.

## Changes made or confirmed during the WSL lunar-simulation work

### WSL and Docker integration

- Added `src/togo_sim_wsl.sh` as the native WSL launcher.
  - Sources the native install prefix.
  - Configures WSLg / D3D12 graphics.
  - Configures ROS domain 29 and CycloneDDS.
  - Starts the Gazebo server, optional RViz, and Gazebo GUI.
  - Prevents duplicate simulation launches and cleans up child processes.
- Changed `docker-compose.yml` so development containers use host networking.
- Added / updated WSLg display mounts, WSL GPU libraries, DDS configuration,
  and NVIDIA compute-only container settings.
- Added / updated `config/cyclonedds.xml` and `config/fastdds.xml`.

### Lunar terrain generation

- Added the `src/moon_mesh_generator` workflow.
- Made `generate_lunar_heightmap.py` and `generate_lunar_mesh.py` resolve paths
  relative to their script location instead of the caller's directory.
- The mesh generator creates two assets:
  - `lunar_terrain.dae`: detailed visual / lidar mesh.
  - `lunar_terrain_collision.dae`: decimated physics mesh.
- Current visual grid resolution: 769.
- Current collision grid resolution: 257.
- Added `src/togo/togo_gz/worlds/lunar_surface.world`.
- Added diagnostic worlds including `flat_bullet_test.sdf`.

### Physics engine and rover wheels

- `lunar_surface.world` explicitly uses classic Bullet:
  `gz-physics-bullet-plugin`.
- Reason: DART could not construct the lunar mesh collision, while Bullet
  Featherstone loaded the mesh but prevented the A300 skid-steer rover from
  rotating. Classic Bullet was verified to rotate correctly on a flat world
  and on the lunar mesh.
- Added local A300 outdoor-wheel wrappers:
  - `outdoor_wheel_no_slip.urdf.xacro`
  - `outdoor_wheels_no_slip.urdf.xacro`
- Updated `a300.urdf.xacro` to use those wrappers.
- Reason: Clearpath's optional `WheelSlip` system requests friction-pyramid
  slip compliance that Bullet Featherstone did not implement and produced
  warnings every simulation step. Ordinary wheel collision and friction are
  retained.
- The A300 wheel dimensions and controller calibration remain consistent with
  upstream Clearpath:
  - Physical outdoor-wheel radius: 0.1651 m.
  - Controller wheel radius: 0.1625 m.
  - Diff-4WD wheel-separation multiplier: 1.75.

### Current gravity setting (temporary test state)

- `lunar_surface.world` currently uses Earth gravity: `0 0 -9.82`.
- `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml` currently uses
  `odom/gravity: 9.82` to match the simulated IMU.
- These values were changed together for a traction A/B test.
- If lunar gravity is restored, change both values together to 1.62. Never
  leave Gazebo and DLIO with different gravity magnitudes.

### Simulation launch and sensors

- Expanded `sim_gz.launch.py` launch arguments and startup sequencing,
  including robot pose, controller delay, RViz, and simulation time.
- Updated `robot_sensor_checkout.launch.py` to accept a lidar-topic remap.
- Simulation RViz uses `/husky/sensors/seyond/points` instead of the hardware
  `/iv_points` topic.
- Added / updated Gazebo bridge, controller, and RGB-D point-fix
  configurations.
- Gazebo's `platform_velocity_controller` keeps `enable_odom_tf: false` so it
  does not compete with SLAM for `odom -> base_link`.

### SLAM and TF

- Removed the redundant `dlio_odometry_to_tf` process from
  `seyond_dlio_slam.launch.py`.
- Reason: DLIO already publishes `odom -> base_link`; the extra publisher
  produced two different transforms at the same timestamp and caused RViz to
  jump vertically.
- Graph SLAM remains responsible for `map -> odom`.
- DLIO remains responsible for `odom -> base_link`.
- Robot state publisher remains responsible for fixed and articulated robot
  transforms below `base_link`.

### Navigation

- Added / updated the `togo_navigation` Nav2 configuration and launch scripts
  under `lidarslam_ros2`.
- Nav2 consumes `/dlio/odometry` and uses simulation time.
- The controller uses Regulated Pure Pursuit and a stamped velocity command
  path to `/platform_velocity_controller/cmd_vel`.

## Other differences detected against current upstream

The following upstream-tracked areas differ locally and predate or extend
beyond the focused fixes above. They should be reviewed before attempting an
upstream merge:

- Root development environment:
  `.dockerignore`, `.env`, `.gitignore`, `.pre-commit-config.yaml`, Dockerfile,
  README, and the formatting workflow.
- `togo_deploy` sensor, controller, localization, diagnostics, launch, RViz,
  and package configuration.
- `togo_description` package metadata, robot-view launch, A300 description,
  and the main Togo macro.
- `togo_gz` package metadata, CMake installation, bridge/controller settings,
  launch files, RGB-D fixer, and worlds.
- Several packages and documentation paths present in current upstream are
  absent locally, including `practice_worlds`, the older `togo_nav2` package,
  `togo_status_handler`, and portions of upstream hardware documentation.
- The substantial `lidarslam_ros2` tree is treated as a separate local SLAM
  workspace and was excluded from the upstream file-by-file comparison.

## Known follow-up work

- Decide whether the final lunar simulation should return to 1.62 m/s^2;
  update Gazebo and DLIO together.
- Make RViz's `use_sim_time: true` setting unconditional in the native launch.
  A runtime check previously found `/rviz2` using wall time while Nav2 used
  simulation time, causing goal timestamp extrapolation errors.
- Optimize or throttle path-visualization helpers; they have consumed nearly
  one CPU core while republishing growing paths.
- Add an automated smoke test that verifies:
  - one `/clock` publisher;
  - one owner for each dynamic TF edge;
  - expected lidar and IMU rates;
  - forward motion and nonzero yaw under direct velocity commands;
  - successful Nav2 goal acceptance in the `map` frame.

## Maintenance rule

For every future non-dependency change, append an entry below with the date,
files, reason, validation, and whether the change is permanent or temporary.

## Change log

### 2026-09-30

- Added this ledger.
- Recorded the WSL/native-Gazebo and Docker-SLAM architecture.
- Recorded lunar mesh generation and collision-mesh handling.
- Recorded the switch from Bullet Featherstone to classic Bullet.
- Recorded duplicate-TF removal and matched Gazebo/DLIO gravity handling.
- Status: current configuration validated with Gazebo, SLAM, and Nav2.
