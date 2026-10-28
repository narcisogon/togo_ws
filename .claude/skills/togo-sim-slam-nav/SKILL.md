---
name: togo-sim-slam-nav
description: Orientation for the parts of this repo the team actually uses, the Seyond LiDAR + IMU → DLIO → graph_based_slam (hazard /map) → Nav2 stack on the TOGO/Husky rover, in Gazebo simulation (WSL), on the real rover, and replaying real bags. Use when running, debugging, tuning, or changing anything in that stack (run_live_seyond_dlio_slam.sh, run_live_real_flat.sh, run_live_husky_real_dlio.sh, run_replay_husky_real.sh, run_nav2_only.sh, seyond_dlio_*.yaml, nav2_slam_params.yaml, togo_navigation, graph_based_slam hazard), or when deciding whether some other part of the repo is relevant.
---

# TOGO rover: DLIO SLAM + Nav2 (what's actually used)

This gives a **basic understanding of the parts of the repo in real use**. It isn't a full spec.
For details, read the runbooks listed below and the code itself.

**Scope:** everything built on **DLIO + `graph_based_slam` + `togo_navigation`** is in use.
Most of the rest of `lidarslam_ros2/` comes from upstream and is a relic: mid360, KITTI,
Autoware, RKO-LIO, faster-lio, small_gicp/vgicp, benchmarks, report generators, and the
`lidarslam*.yaml` / `rko_lio*` / `*mid360*` params. Don't recommend it, model fixes on it, or
assume it works. If a task seems to need it, say so and ask first.

## One core stack, three ways to feed it

All modes use the same core:
- **`direct_lidar_inertial_odometry`** (`dlio_odom_node`, `dlio_map_node`) publishes `/dlio/odometry`
  and `/dlio/deskewed`, and owns **TF `odom → base_link`**.
- **`graph_based_slam`** owns **TF `map → odom`**, does loop closure and map saving (`/map_save`),
  and publishes **`/map`** with the hazard/terrain grid and inflation built in (worker thread
  inside the backend).
- **`togo_navigation`** handles Nav2 and `goal_safety_relay`.
- **`lidarslam/param/seyond_dlio_graph.yaml`** is the base YAML for both DLIO and the backend.
  Each mode adds overrides on top of it.

| Mode | Entry script (`lidarslam_ros2/scripts/togo/`) | Launch file (`lidarslam/launch/`) | Params | Clock | LiDAR / IMU topics | Runbook |
|---|---|---|---|---|---|---|
| **Simulation** | `run_live_seyond_dlio_slam.sh` | `seyond_dlio_slam.launch.py` | `seyond_dlio_graph.yaml` | sim `/clock` | `/husky/sensors/seyond/points`, `/husky/sensors/imu_0/data_raw` | `lidarslam_ros2/README.md` |
| **Real rover, live (flat)** | `run_live_real_flat.sh` | `seyond_dlio_live_real_flat.launch.py` | DLIO `cfg/{dlio,params}.yaml` + `seyond_dlio_live_real_flat.yaml` (does **not** load `seyond_dlio_graph.yaml`) | wall | `/iv_points`, `/husky/sensors/imu_0/data` | `RUN_ON_REAL_ROVER.md` |
| **Real rover, live (corrected)** | `run_live_husky_real_dlio.sh` | `seyond_dlio_live_real.launch.py` → includes `seyond_dlio_slam.launch.py` | `seyond_dlio_graph.yaml` + `seyond_dlio_live_real.yaml` | wall | `/iv_points` → timestamp corrector (`LIDAR_TIME_OFFSET_SEC`, default -6.575) → `/iv_points/time_corrected` | `RUN_ON_REAL_ROVER.md` §9 |
| **Replay real bag** | `run_replay_husky_real.sh` (`BAG_PATH=...`) | `seyond_dlio_replay.launch.py` → includes `seyond_dlio_slam.launch.py` | `seyond_dlio_graph.yaml` (see quirks) | bag `--clock` | `/iv_points`, `/husky/sensors/imu_0/data` (+ `/fixposition/gnss1`) | `REPLAY_BAG.md` |
| **Nav2 (any mode)** | `run_nav2_only.sh` (`USE_SIM_TIME=false` on the real rover) | `togo_navigation/launch/rover_nav2.launch.py` | `togo_navigation/config/nav2_slam_params.yaml` | per `USE_SIM_TIME` | uses `/map`, `/dlio/odometry` | README + hazard README |

**Which live-real launcher is primary is unclear.** `RUN_ON_REAL_ROVER.md` says the flat launch
replaced `run_live_husky_real_dlio.sh` and that the old one is the "known to work" fallback.
`scripts/togo/README.md` still lists `run_live_husky_real_dlio.sh` as primary. Ask the user
which one they're running before changing either.

### Simulation setup specifics
- Gazebo runs **natively in WSL** via `src/togo_sim_wsl.sh` (lunar world, built into
  `~/togo_ws_native_clean/install`). SLAM and Nav2 run in the Docker container (`dev`/`hw-dev`).
  The README's container `togo_sim.sh` isn't the path in use.
- WSL ↔ Docker communication needs ROS_DOMAIN_ID=29, CycloneDDS, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  and host networking. If SLAM sees no topics, check this before touching SLAM.
- Sim-only extras: `gazebo_pose_to_aligned_path.py` / `gz_truth_path.py` publish ground truth for
  drift comparison.
- `src/togo_sim_wsl.sh` env overrides: `TOGO_WORLD`, `TOGO_ROBOT_X/Y/Z`, `TOGO_RVIZ`, `TOGO_GZ_GUI`.

### Apollo 16 lunar world
- The realistic Apollo 16 world lives in `src/lunar_world_gen`; read the **lunar-world** skill before building,
  running or debugging it. Its launcher is `src/lunar_world_gen/scripts/run_apollo16_sim.sh` (not
  `togo_sim_wsl.sh`). Topics, frames, ROS domain 29 and CycloneDDS are the same as in the simulation mode above.
- Its gravity is 1.62 m/s², so DLIO must use `src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml`:
  in the container, `COMBINED_PARAM_FILE=/home/er4-user/ws/src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml ./run_live_seyond_dlio_slam.sh`.
- Status (2026-10-27): validated and ready for SLAM testing; run it `--headless` (RTF 0.82-0.92 with the GUI open), and note its LiDAR has no per-point time field (limits: `src/lunar_world_gen/README.md` section 9).
- **SLAM benchmark on it:** a one-lap bag with verified ground truth.
  - `src/lunar_world_gen/scripts/replay_slam_bag.sh` replays it into the container's SLAM and scores it.
  - `benchmark_slam.sh --label X --config F --compare-to baseline` compares a config change over 3 runs.
  - Details: README section 5; `README.md` section 0 maps Windows / WSL / container.
  - Start the container with `docker compose start dev`, not `up`. `up` can recreate it, and the slam_ws build
    inside is then lost.

### Real rover specifics
- Rover bringup must publish `base_link → lidar3d_0_laser` and `base_link → imu_0_link`.
  Use `PUBLISH_SENSOR_STATIC_TF=true` only for a bench test without bringup.
- The real-live scripts refuse to start or warn if DLIO/graph nodes from an earlier run are still
  alive. They use a per-run `/dev/shm/graph_slam_pcd_cache_live_real_<pid>`, and maps go to
  `output/husky_seyond_live_real/` (DEM in `lunar_dem/`).
- Bag timing tools in `scripts/togo/` are used for real bags: `audit_replay_bag_timing.py`,
  `rewrite_lidar_timestamps_bag.py`, `retime_lidar_storage_to_imu.py`, `timed_map_diag_once.py`,
  `dlio_diag_once.py`. See `scripts/togo/README.md`.

## Paths: host repo vs. container
Scripts hardcode container paths.
- `./lidarslam_ros2` → `/home/er4-user/slam_ws/src/lidarslam_ros2`
- `./slam_output` → `.../lidarslam_ros2/output`
- `./src` → `/home/er4-user/ws/src`
- `./bags` → `/bags`

Edit on the host, then rebuild in the container after C++ or installed-file changes:
```bash
cd ~/slam_ws && colcon build --symlink-install --packages-select graph_based_slam togo_navigation lidarslam direct_lidar_inertial_odometry --packages-ignore gtsam
```

## Nav2 data flow
- Global and local costmaps use **only a StaticLayer on `/map`**, with no obstacle or inflation
  layers. Hazard and inflation tuning lives in `graph_based_slam.ros__parameters.hazard/*` in the
  SLAM YAML, **not** in `nav2_slam_params.yaml`. See `graph_based_slam/hazard/README.md`.
- RViz 2D Goal Pose → `/goal_pose_requested` → `goal_safety_relay` checks the goal against `/map`
  → `navigate_to_pose`.
- Velocity: `/nav2/cmd_vel` → velocity_smoother → `/nav2/cmd_vel_smoothed` → relay →
  `/platform_velocity_controller/cmd_vel` (TwistStamped). The relay publishes zero velocity on a
  stale map or stale commands.
- ⚠️ On the real Husky, `togo_deploy/config/husky/twist_mux.yaml` expects nav input on `cmd_vel_nav`.
  The relay publishes straight to the controller topic, which may bypass twist_mux priorities and
  e-stop. This is unverified, so raise it if a real-rover Nav2 issue involves driving or safety.
- Never add a second `odom → base_link` publisher. The sim controller has `enable_odom_tf: false`,
  and a duplicate DLIO TF node was removed because it made RViz jump.

## Known quirks (read from code on 2026-10-19; recheck before relying on them)
- **Sim deskew default:** `run_live_seyond_dlio_slam.sh` defaults `DLIO_DESKEW=true`, while
  `seyond_dlio_slam.launch.py` defaults to `false`. The script wins. Both real-live scripts default to `false`.
- **Sim `correct_cloud_slant` doesn't reach DLIO:** DLIO always reads the raw `lidar_topic`.
  `lidar_topic_timed` and `timed_cloud_input_topic` are declared but unused, because DLIO does
  synthetic per-point timing internally.
- **The replay override YAML is ignored** (per `REPLAY_BAG.md`): edit `seyond_dlio_graph.yaml`, or a
  copy passed as `SLAM_PARAM_FILE`. `seyond_dlio_replay.yaml` must still exist.
- **Gravity is coupled:** the world's gravity and `odom/gravity` must match. `lunar_surface.world` and
  `seyond_dlio_graph.yaml` are both currently 9.82 (temporary test); change them together. The Apollo 16
  world has 1.62: run DLIO there with the lunar copy `src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml`
  (identical except `odom/gravity`) instead of editing the shared file.
  The real-live YAMLs set their own `odom/gravity: 9.80665`, so they're unaffected.
- **Stale docs:**
  - The README says `run_nav2_only.sh` polls for the lifecycle manager, but it doesn't.
  - `RUN_ON_REAL_ROVER.md` gives `scripts/run_live_real_flat.sh` and
    `scripts/run_live_husky_real_dlio.sh`, but both actually live in `scripts/togo/`.
  - The `seyond_dlio_slam.launch.py` docstring mentions a nonexistent `run_dlio_slam_husky.sh`.
- **Stale utilities:** `record_seyond_slam_bag.sh` records old a300/RKO-LIO topics, not DLIO.
  `save_seyond_map_clean.sh` sources `/ws/install`, not `~/slam_ws`. To save a map, call
  `ros2 service call /map_save std_srvs/srv/Empty` directly.

## Status of other togo scripts
- ⚠️ `run_nav2_with_slam.sh`: an old bag + sim-SLAM combo with hardcoded paths. Its name is
  misleading because it doesn't launch Nav2.
- ❌ `run_live_seyond_slam_integrated.sh` (RKO-LIO), `run_live_seyond_faster_lio_slam.sh`,
  `monitor_rko_diagnostics.sh`, `rko_diagnostics_monitor.py`, and the root `/scripts/*` copies
  (Docker compatibility wrappers). Edit `lidarslam_ros2/scripts/togo/` instead.

## Rules for working here
1. **Don't claim a fix works unless it ran.** This Windows session usually can't run Gazebo, ROS
   or the rover. Say what changed, which terminal or script to restart, and what to check, then
   let the user confirm.
2. **Ask which mode first** (sim / real live / replay) when it isn't clear. Topics, clock and param
   files all differ between modes.
3. **Diagnose upstream first:** sensors publish (`ros2 topic hz`) → clock ticks (sim/replay) →
   `/dlio/odometry` → `map→odom` TF (`tf2_echo map base_link`) → `/map` → Nav2.
4. **Put param changes in the YAML that mode actually loads** (see the table). A change to
   `seyond_dlio_graph.yaml` affects sim, replay and the corrected live-real path, but **not** the
   flat live-real path.
5. **Keep changes in the workflow's own files.** Don't refactor relic code or delete unused launch
   args unless asked. If something contradicts this skill, tell the user and suggest updating it.
