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

### 2026-10-02

- Added `.agents/skills/togo-slam/` with a repo-specific skill and operating and
  troubleshooting references for WSL Gazebo, Docker DLIO/graph SLAM, and Nav2.
- Captured clock and TF ownership, matched gravity, sensor wiring, launcher
  distinctions, and source/runtime discrepancies without changing ROS behavior.
- Reason: reuse the working integration knowledge for future SLAM changes and
  evidence-based diagnosis.
- Validation: skill frontmatter validation and reference/source review; no
  Gazebo, SLAM, navigation, or hardware motion test is needed for these documents.
- Status: permanent guidance; check source and runtime for drift before reuse.

### 2026-09-30

- Added this ledger.
- Recorded the WSL/native-Gazebo and Docker-SLAM architecture.
- Recorded lunar mesh generation and collision-mesh handling.
- Recorded the switch from Bullet Featherstone to classic Bullet.
- Recorded duplicate-TF removal and matched Gazebo/DLIO gravity handling.
- Status: current configuration validated with Gazebo, SLAM, and Nav2.

### 2026-10-03 — Backend-owned hazards and Nav2 map import

- Added `lidarslam_ros2/graph_based_slam/hazard/`: terrain projection, robust
  gravity-relative slope classification, backend worker, diagnostics, and
  operator documentation. Compiled directly into the existing backend component.
- Matched deskewed clouds to buffered acquisition odometry; corrected graph
  poses reproject cached submap terrain. Stationary clouds refresh the live map.
- Centralized hazard parameters in the backend section of
  `lidarslam/param/seyond_dlio_graph.yaml`. Added physical footprint clearance,
  a 0.1 m hard margin, and an adaptive linear gradient to a 0.3 m total outer
  margin. Overlaps take maximum cost and soft costs preserve unknown cells.
- Removed the old `hazard_mapping` and `hazard_mapping_msgs` packages,
  `local_hazard_grid`, and `slam_to_occupancy_grid`. Removed their launch
  plumbing, map-save requests, duplicate Nav2 obstacle/inflation processing,
  and inactive collision-monitor configuration.
- Updated Nav2 YAML and launch: global and local costmaps import the same complete
  backend map; planning, controller, recoveries, waypoint navigation, and
  velocity smoothing remain. Existing goal relay checks exact requested goals
  and gates final platform commands on fresh backend heartbeats and commands.
- Updated `run_nav2_only.sh`, both active RViz configurations, and workflow
  READMEs. Shutdown targets only the Nav2 launch process group. Added core
  regression tests and an isolated ROS backend/Nav2 integration harness.
- Reason: one authoritative hazard/cost pipeline, fewer nodes and parameter
  locations, rover tilt independent terrain classification, and visible worker
  behavior for iterative algorithm changes.
- Validation: native WSL core regression suite passed all 11 tests, including
  pitch/roll, steep terrain, full graph rotation, height filtering, support,
  adaptive/overlapping inflation, distance-transform brute-force comparison,
  and invalid/bounded settings. Docker ROS Jazzy build passed for
  `graph_based_slam`, `togo_navigation`, and `lidarslam` using the existing
  dependency underlay and an isolated build/install overlay.
- Validation: isolated domain 199, network-disabled Docker smoke test passed
  fresh backend map publication, pitch invariance, invalid runtime parameter
  rejection and valid rebuild, exact Nav2 grid/cost import with preserved
  gradient, RViz-style waypoint planning around lethal cells, controller command
  forwarding, and zero commands after cloud inputs stalled. Python, YAML/RViz,
  XML, shell syntax, and whitespace checks passed.
- Limits: no Gazebo or physical rover motion validation in this change. The
  algorithm is slope-only, the physical footprint is conservatively circular,
  historical terrain is static, and the prior simulation allow-unknown policy
  remains enabled. Tune/validate real terrain and update timing before using
  these initial thresholds as rover limits. Rebuild the normal runtime overlay
  before using the operator scripts; validation artifacts are under ignored build/.
- Status: permanent architecture change; initial hazard thresholds require
  terrain validation. Existing unrelated .env, skill, and submodule changes
  were left intact.

### 2026-10-03 — Transient hazard readiness and pending waypoints

- Updated `graph_based_slam/hazard/hazard_worker.{hpp,cpp}` so readiness describes
  the last published map, with an independent 200 ms backend timer. Waiting for
  acquisition odometry or appending unchanged submap geometry no longer drops a
  still-fresh map. Config changes, changed/removed graph geometry, worker errors,
  and expired map source timestamps still withdraw readiness.
- Added `/hazard/status` and readiness transition logs with map source age.
  Debug-cloud serialization no longer holds the snapshot lock after map commit.
- Updated `togo_navigation/src/goal_safety_relay.cpp` to retain only the latest
  waypoint while backend readiness or the Nav2 action server is unavailable.
  Added `goal_wait_timeout_sec: 5.0` to the Nav2 YAML. Pending goals are validated
  against the current map on recovery and discarded on timeout, with a reason.
- Updated the hazard README and ROS integration harness. Operator changes to
  slope, inflation margins, and the zero footprint in the backend YAML were
  preserved.
- Reason: reported intermittent "backend hazard map is not ready/fresh" goal
  rejections and possible command jitter from transient readiness drops.
- Validation: Docker Jazzy build passed for graph_based_slam, togo_navigation,
  and lidarslam. The isolated domain-199 ROS/Nav2 test passed brief odometry-gap
  continuity, independent heartbeats at a 1.3 s mapping cadence, parameter
  validation, costmap import, queued goal recovery/replacement, planning and
  command forwarding, stale-map stopping, explicit stale status, and pending
  goal expiration without later execution. Python/YAML/XML and whitespace checks
  passed. The active development container's ROS stack was stopped; no live
  terrain run or physical jitter measurement was performed.
- Status: permanent fix. Build validation used the separate ignored overlay;
  rebuild the normal SLAM workspace and restart SLAM/Nav2 to activate it.

### 2026-10-03 — Fresh-cloud batches and persistent hazard terrain

- Updated `graph_based_slam/hazard/hazard_mapping.{hpp,cpp}` with acquisition-aware
  batch projection and a persistent terrain grid. Fresh measured heights replace
  older observations; unchanged cells retain their planes. Changed heights refit
  only their slope-support neighborhoods using dense grid lookups. Inflation
  updates include competing nearby hazards and remove obsolete hard/soft halos.
- Updated `hazard_worker.{hpp,cpp}` to process up to `hazard/batch_clouds: 3`
  fresh scans, or a partial batch after `hazard/update_period_sec: 0.20` wall
  seconds. The bounded queue drops older pending scans on overload and rejects
  duplicate/out-of-order stamps. Every scan uses its own acquisition odometry.
  Stationary updates use the existing DLIO cloud stream; frontend keyframe and
  distance-based graph submap rules are unchanged.
- Retained latest terrain observations with graph-anchor coordinates. Geometry
  corrections rebuild historical terrain and reproject retained observations in
  acquisition order; removed sources disappear and older source clouds cannot
  overwrite newer measured terrain. Added source timestamps in
  `src/graph_based_slam_optimization.cpp`. Ordinary appends/cloud batches do not
  reread or refit historical submaps.
- Added the batch parameter and wait-limit explanation to
  `lidarslam/param/seyond_dlio_graph.yaml`. Preserved operator slope (15 degrees),
  hard/soft margins (0.10/0.40 m), zero footprint, and one-second freshness limit.
  Added batch/drop/change/refit/inflation counters to `/hazard/diagnostics` and
  updated the hazard README.
- Updated `togo_navigation/config/nav2_slam_params.yaml` to use
  `PoseProgressChecker`: 0.20 rad rotation or 0.25 m translation counts as progress
  within ten ROS seconds. Existing controller speed/acceleration settings remain.
- Added meaningful incremental-grid/worker regressions and expanded
  `test/test_hazard_nav2_integration.py` for batching, partial stationary batches,
  unchanged terrain, and old hazard/inflation removal. Registered the worker
  test in the backend CMake configuration with isolated domain 199.
- Reason: live capture found mostly suppressed drive commands because full-map
  hazard rebuilds took 1.6–2.1 wall seconds and repeatedly exceeded freshness.
  Controller progress failures also penalized stationary turns.
- Validation: 16 native WSL core tests and the isolated ROS worker correction/
  removal regression passed. Docker builds passed in both the isolated overlay
  and the normal `/home/er4-user/slam_ws` runtime workspace. The real backend/Nav2
  synthetic integration harness passed against both installed overlays, including
  costmap import, planning, command forwarding, stale stopping, and goal expiry.
  Configuration/Python/XML validation and whitespace checks passed.
- Performance: an 81,693-cell synthetic terrain benchmark, with 14,641 changed
  cells, refitted 18,677 cells and averaged 56.2 ms versus 418.6 ms for the full
  reference rebuild (same classifications/costs). Identical observations took
  1.6 ms and refitted zero cells. This measures terrain processing, not the full
  Gazebo sensor pipeline or physical navigation.
- Limits: graph corrections/filter changes still require a full historical
  rebuild. Missing returns do not clear stored terrain; this remains slope-only
  hazard classification. Gazebo was stopped by validation time, so rover turn
  behavior and successive-goal jitter remain to be verified in simulation.
- Status: permanent implementation; normal Docker runtime rebuilt and ready for
  the existing operator launchers. Generated test/benchmark artifacts remain
  under ignored `build/hazard-validation/`.

### 2026-10-03 — Gazebo navigation validation and turning acceleration

- Updated `togo_navigation/config/nav2_slam_params.yaml`:
  `FollowPath.max_angular_accel` is now 4.0 rad/s² instead of 1.5. Preserved
  turning speed, smoother speed/acceleration limits, hazard thresholds, and
  import-only Nav2 costmaps. Updated the backend hazard README to explain this
  setting and the distinction between unknown and out-of-bounds waypoints.
  Corrected obsolete process-name cleanup advice in the main runbook.
- Reason: with the rebuilt incremental backend, readiness remained true and
  commands were continuous, but the initial turn still stalled. During four
  simulation seconds, median turn command was 0.081 rad/s, measured DLIO yaw
  rate was 0.006 rad/s, and heading changed only 0.055 rad. RPP limits each turn
  request relative to measured rotation; the former acceleration setting did
  not provide enough command headroom to start this simulated rover's turn.
- Validation: used the normal Docker install and native WSL lunar Gazebo,
  original spawn/physics/sensors, domain 29, and simulation time. Launched
  headless with both RViz instances disabled. Confirmed Gazebo clock, DLIO
  odometry, map->odom and odom->base_link, single relay drive publisher, and
  both StaticLayer-only costmaps. With the live 4.0 setting, a rear-facing
  waypoint and two successive driving goals all returned Nav2 success with
  error code zero. Neither driving goal had interior zero-command pauses;
  median final driving command was 0.6 m/s. All 900 readiness samples were true.
- Persisted the setting, restarted only the test-owned Nav2 stack, and verified
  effective 4.0 acceleration, PoseProgressChecker, and unchanged smoother
  acceleration. An additional mapped goal 1.3 m behind the rover required
  turning and driving and succeeded in 10.75 simulation seconds (79.8 wall
  seconds). All 691 readiness samples were true, with no interior zero-command
  pauses. Direct Gazebo model poses independently confirmed approximately
  3.02 rad of rotation and 0.85 m net displacement over this final test.
- Performance: median hazard worker time was 106 ms during the three-goal run
  (maximum 149 ms), and 117 ms during the final turn-and-drive run (maximum
  187 ms). Source age remained about 0.1 simulation seconds. Ordinary graph
  appends reached 13 submaps without rebuilding historical sources. Slow
  simulation delivery exercised partial one-cloud batches, as intended.
- Initial behind-map test requests were rejected because the forward lidar's
  startup map did not cover those coordinates. Unknown-goal permission does
  not override map bounds. Tests used an in-bounds heading change first to
  obtain coverage; no goal relocation or map-bound policy change was added.
- Configuration/Python/XML and whitespace checks passed. Recorded captures,
  bounded test drivers, and Gazebo poses are under ignored
  `build/hazard-validation/`. Each test ended with zero drive commands; no
  teleop or direct velocity publisher ran. Gazebo, SLAM, and the restarted
  Nav2 remain running with the rover stopped.
- Status: permanent simulated-navigation tuning, validated in Gazebo. Physical
  rover traction and turning limits still require their own validation.

### 2026-10-03 — Confidence-aware terrain and conservative hazard clearing

- Updated `graph_based_slam/hazard/hazard_mapping.{hpp,cpp}` to estimate bounded
  height uncertainty from acquisition range, point spread, and cell support.
  Sparse cells use an interpolated median rather than a minimum-like percentile.
  Consistent heights fuse by uncertainty; large disagreements stay separate until
  reliable observations confirm a new height mode. Different surfaces are not
  averaged into an artificial ramp. Sub-nanometer acquisition roundoff does not
  trigger stationary plane refits.
- Added uncertainty-weighted robust plane fits with spatial coverage, residual,
  and slope uncertainty checks. Existing hazards survive missing, weak, invalid,
  or isolated observations. Clearing requires clearly safe, well-distributed
  fresh neighborhood support in repeated batches; duplicate stamps and graph
  replay do not contribute confirmations. Inflation removes halos only after
  confirmed clearance, retaining maximum-cost overlap with other hazards.
- Updated `hazard_worker.{hpp,cpp}` to preserve fused heights, uncertainty,
  alternative height modes, hazard classifications, and clearing history through
  graph corrections. Added per-job `held_hazards`, `cleared_hazards`, and
  `rejected_heights` diagnostics. Mapping remains in the backend worker thread;
  Nav2 still imports the same grid without additional cost analysis.
- Added five validated backend YAML controls in
  `lidarslam/param/seyond_dlio_graph.yaml`: `hazard/height_noise_m: 0.01`,
  `hazard/min_plane_coverage: 0.20`, `hazard/max_plane_error_m: 0.05`,
  `hazard/clear_confirmations: 3`, and `hazard/clear_slope_margin_deg: 3.0`.
  Preserved operator slope limit 15 degrees, margins 0.10/0.40 m, zero footprint,
  batch/cadence, and freshness settings. Corrected the minimum-point comment:
  hazards receive the full deskewed cloud, not the odometry voxel-filter output.
- Updated the hazard README and terrain/worker/Nav2 integration tests. Reason:
  latest-height replacement and immediate binary slope reclassification allowed
  uncertain newer terrain to erase stronger older hazards. The implementation
  now distinguishes reliable safe observations from insufficient evidence.
- Validation: all 24 registered terrain tests and the isolated ROS graph-worker
  correction/removal regression passed. Rebuilt the normal Docker runtime
  `/home/er4-user/slam_ws`; the real backend/Nav2 domain-199 integration passed
  isolated contradictory-return retention, confirmed hazard/halo removal,
  parameter rejection, body pitch, stationary batching, both costmap imports,
  waypoint planning, command forwarding, freshness stopping, and goal expiry.
- Native WSL lunar Gazebo with the usual SLAM launcher, domain 29, and simulation
  time passed LiDAR/IMU reception, DLIO cloud/odometry timing, both dynamic TF
  edges, and effective new parameters. The steady capture had 20 completed maps,
  all 56 readiness samples true, median worker time 83.1 ms, maximum 105.0 ms,
  source age about 0.1 simulation seconds, and no historical-source rebuilds.
  A separate stationary 20-map capture had no lethal-to-free/unknown transitions.
  No rover motion commands were issued in these domain-29 checks; the synthetic
  command test ran only in isolated domain 199. Test-owned live stacks were
  stopped afterward; Docker remains running.
- Performance: the 81,693-cell synthetic benchmark with 14,641 changed cells
  refitted 18,677 cells in about 103 ms. Settled identical inputs took about
  4.3 ms with zero plane refits. Partial inflation matched a complete distance
  transform. These numbers measure terrain processing, not the full pipeline.
  Terrain state occupies 128 bytes per allocated cell in this build, excluding
  grids, retained graph anchors, cloud buffers, and container overhead.
- Limits: uncertainty is a tuning proxy, not calibrated sensor/pose covariance
  or independent-viewpoint evidence. A confirmed conflicting height and its
  subsequent safe classification can require separate confirmation sequences.
  Recorded near/far terrain still needs validation of the thresholds. This
  remains slope-only classification without drop-off inference or time-only
  hazard decay. Explicit height/filter/noise changes rebuild source terrain;
  geometry corrections retain history. Broad driving tests were not repeated.
- Configuration/Python/XML and whitespace checks passed. Generated validation
  artifacts remain under ignored `build/hazard-validation/`.
- Status: permanent implementation; normal Docker runtime rebuilt for the
  existing two-terminal operator workflow.

### 2026-10-04 — Bounded temporary obstacle confidence and observed-space clearing

- Updated `graph_based_slam/hazard/hazard_mapping.{hpp,cpp}` to retain ground
  separately from sparse 3D blockage evidence. Above-ground obstacle returns do
  not replace ground heights. Reliable ground support is required to identify
  obstacle height; unsupported surfaces above the acquisition body cannot
  bootstrap traversable ground. Terrain slope hazards retain their existing
  uncertainty and confirmation rules.
- Occupancy uses bounded log-odds and mark/clear hysteresis, with one evidence
  update per fresh batch/voxel. Defaults are hit 0.85, miss 0.25, mark 0.70,
  clear 0.30, cap 0.90, and three supporting returns/rays. A saturated voxel
  clears after three qualified observed-free batches, regardless of dwell
  duration. Any occupied return vetoes clearing for that voxel, including sparse
  returns. A newer sparse hit advances acquisition ordering without increasing
  confidence, preventing older deferred rays from clearing it.
- Clearing traces actual LiDAR-origin rays through previously occupied 3D
  voxels, before the return endpoint with a guard margin. Ground seen below an
  obstacle, missing returns, occlusion, duplicates, historical replay, and age
  alone cannot clear an unobserved occupied volume. Fully cleared blockage
  reveals retained ground or unknown terrain; it cannot erase a slope hazard.
  The final map combines terrain and occupancy and applies inflation once.
- Updated `hazard_worker.{hpp,cpp}` to obtain the sensor origin through existing
  body-to-LiDAR TF on the backend executor. No additional node or spin thread
  is created. Common-origin clearing requires odometry covering 100 ms before
  and after the scan header, with motion within 1 cm and 0.5 degrees. Up to
  16 deferred scans wait for coverage, then contribute clearing only. Latest
  marking precedes deferred clearing; both share a 3,000-ray job budget. Terrain
  and marking continue when ray clearing is unsuitable. Sparse state is limited
  to 200,000 voxels; exceeding the allocation budget fails readiness explicitly.
- Graph corrections preserve obstacle confidence and latches through full
  SE(3) voxel-center reprojection and conservative collision merging. Removed
  sources remove associated terrain and obstacles. Ordinary appends and fresh
  scans continue incrementally without historical reconstruction.
- Added validated backend YAML settings for obstacle heights 0.15–1.80 m above
  trusted ground, vertical voxel size 0.10 m, evidence/support thresholds,
  budgets, and `hazard/lidar_frame: lidar3d_0_laser`. Preserved current operator
  settings, including slope 14 degrees, inflation 0.15/0.60 m, zero footprint,
  and existing batching/freshness controls. Nav2 remains import-only for costs.
- Added `/hazard/obstacles` voxel-center confidence visualization and diagnostics
  for marking, clearing, ray work, and deferred scans. Updated the hazard README
  and added disabled Temporary Obstacle Confidence displays to both mapping and
  navigation RViz configurations, using fixed intensity limits 0–100.
- Validation: all 42 registered core and two graph-worker regressions passed.
  Coverage includes dwell-independent clearance, different obstacle heights,
  occlusion/endpoints, terrain preservation, unknown ground, hit vetoes,
  duplicate/replayed/deferred observations, shared ray budgets, configuration
  rejection, graph translation/rotation, confidence preservation, and removal.
  Rebuilt the normal Docker runtime `/home/er4-user/slam_ws`. Isolated domain-199
  real backend/Nav2 integration passed bounded marking/clearing, body pitch,
  stationary batching, retained terrain, both map imports, gradient preservation,
  planning, command forwarding, freshness stopping, and pending-goal expiry.
- Native WSL Gazebo with the usual SLAM launcher, domain 29 and simulation time
  passed LiDAR/IMU/DLIO reception, clock alignment, both dynamic TF edges, new
  effective parameters, and obstacle debug publication. Twenty completed maps
  had all 58 readiness samples true, median worker 52.1 ms, maximum 74.1 ms,
  at most 2,979 traced rays, and no historical rebuilds. A separate stationary
  29-map capture had no lethal-to-free/unknown transitions. No rover motion
  commands were issued in domain 29; command-flow testing was isolated to 199.
  Test-owned stacks were stopped afterward; Docker remains running.
- Synthetic core benchmark: 40,006 points, 40,000 terrain cells, 52,900 grid
  cells and 2,858 traced rays. Identical settled geometry took median 30.8 ms
  (maximum 50.8 ms) with zero refits. Dense stationary ±1 mm measurement noise
  took median 260.6 ms (maximum 285.4 ms), refitting all 40,000 terrain cells.
  Saturated blockage cleared in three qualified batches while preserving ground
  and background occupancy. These are core timings, not full pipeline latency;
  changing terrain remains the dominant cost in the dense noisy case.
- Limits: evidence values are tuning proxies, not calibrated safety
  probabilities. Clearing while appreciably moving requires per-return origins
  or another validated visibility model. Unobserved upper obstacle voxels can
  remain occupied after a person leaves. Obstacles over entirely unmapped ground
  may initially remain unknown; existing Nav2 unknown-space permission is
  unchanged. Real sensor timing, physical rover behavior, and these thresholds
  still require their own validation; no broad driving tests were repeated.
- Python/YAML/RViz/XML, single-source costmap structure, and whitespace checks
  passed. Generated benchmarks and captures remain ignored under
  `build/hazard-validation/` and `build/blockage-validation/`.
- Status: permanent implementation, rebuilt for the existing SLAM/Nav2 workflow.

### 2026-10-04 — Configurable unknown-space planning cost

- Added integer `hazard/unknown_cost` to the backend config, parameter declaration,
  runtime callback, and active `lidarslam/param/seyond_dlio_graph.yaml`. Valid
  values are -1 through 100; default -1 preserves the existing unknown sentinel
  and planner/relay unknown-space policy. Zero prices unknown as free, 1–99 assigns
  a traversable penalty, and 100 blocks it. Numeric costs apply inside the existing
  map bounds and use the same final `/map` imported by both Nav2 costmaps.
- Updated `hazard_mapping.{hpp,cpp}` so published costs price unknown independently
  of the raw terrain/occupancy evidence. `/hazard/raw`, ground qualification,
  diagnostic hazard counts, and readiness retain the actual unknown classification.
  Unknown costs do not seed inflation. Hard exclusions override the price; soft
  halos combine with numeric unknown costs using max. Partial updates and clearing
  restore the configured unknown price. Runtime changes rebuild costs while
  preserving terrain and obstacle evidence, without refiltering historical clouds.
- Updated the hazard README with YAML/runtime examples and the distinction between
  numeric planning costs and Nav2's sentinel-based `allow_unknown` setting.
- Validation: all 44 native core tests passed, including new tests for -1/0/50/99/100
  modes, empty startup maps, unchanged raw evidence, no inflation from unknown,
  range validation, max-cost combination, and incremental/full inflation equivalence
  through hazard clearing. Rebuilt normal Docker `graph_based_slam_node` in
  `/home/er4-user/slam_ws`. The isolated domain-199 backend/Nav2 integration passed
  runtime -1/50/100 pricing, preservation by both Nav2 costmaps, invalid-value
  rejection, rejection of an unknown-terrain goal at cost 100, and the existing
  obstacle/terrain, planning, command-forwarding and freshness regressions.
  Python/YAML/RViz/XML and whitespace checks passed. No Gazebo or physical rover
  motion was needed or performed for this cost-publication change.
- Preserved operator settings and `.env`; no additional costmap layer or ROS node.
  Generated logs remain ignored under `build/hazard-validation/`.
- Status: permanent implementation, normal Docker backend rebuilt.
