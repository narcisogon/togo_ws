# TOGO Rover — Sim + SLAM + Nav2 Runbook

Everything needed to bring up the TOGO/Husky rover in Gazebo, run **Seyond LiDAR + IMU → DLIO SLAM**,
publish **Gazebo ground truth** for comparison, and run **Nav2** on top of the SLAM output so the path
planner drives the rover.

All of this runs inside the ROS 2 **Jazzy** Docker container (`er4-user@<container>`).

---

## 1. Workspace layout

Two colcon workspaces live side by side in the container home directory. `ws` is **sim only**; `slam_ws`
holds SLAM + navigation.

```
/home/er4-user/
├── ws/                     # simulation workspace (Gazebo only)
│   ├── install/
│   └── src/
│       └── togo_sim.sh     # <- sim bringup script
│
└── slam_ws/                # SLAM + Nav2 workspace
    ├── install/            # sourced by every script below
    └── src/
        └── lidarslam_ros2/ # <- THIS REPO
            ├── lidarslam/
            │   ├── launch/seyond_dlio_slam.launch.py
            │   └── param/seyond_dlio_graph.yaml
            ├── togo_navigation/
            │   └── launch/rover_nav2.launch.py
            └── scripts/togo/
                ├── run_live_seyond_dlio_slam.sh
                ├── run_nav2_only.sh
                └── gz_truth_path.py
```

### Where each thing lives

| Stage | Entrypoint | Location |
|-------|-----------|----------|
| 1. Gazebo sim | `togo_sim.sh` | `/home/er4-user/ws/src/togo_sim.sh` 
| 2. DLIO SLAM | [`run_live_seyond_dlio_slam.sh`](scripts/togo/run_live_seyond_dlio_slam.sh) | `slam_ws/src/lidarslam_ros2/scripts/togo/` |
| 3. Ground truth | [`gz_truth_path.py`](scripts/togo/gz_truth_path.py) | `slam_ws/src/lidarslam_ros2/scripts/togo/` |
| 4. Nav2 | [`run_nav2_only.sh`](scripts/togo/run_nav2_only.sh) | `slam_ws/src/lidarslam_ros2/scripts/togo/` |

Files the scripts drive:

| Purpose | Path in repo |
|---------|--------------|
| SLAM launch file | [`lidarslam/launch/seyond_dlio_slam.launch.py`](lidarslam/launch/seyond_dlio_slam.launch.py) |
| Combined DLIO + graph-SLAM params | [`lidarslam/param/seyond_dlio_graph.yaml`](lidarslam/param/seyond_dlio_graph.yaml) |
| Nav2 launch file | [`togo_navigation/launch/rover_nav2.launch.py`](togo_navigation/launch/rover_nav2.launch.py) |
| Helper scripts | [`scripts/togo/`](scripts/togo/) |

---

## 2. Prerequisites

1. **Docker container running**, with both workspaces mounted:
   - simulation repo → `/home/er4-user/ws/src`
   - this repo → `/home/er4-user/slam_ws/src/lidarslam_ros2`

   > If `run_live_seyond_dlio_slam.sh` aborts with *"Make sure ./lidarslam_ros2 is mounted into the
   > Docker container"*, the bind mount is missing.

2. **Build both workspaces** (once, and after any source change):

   ```bash
   cd ~/ws        && colcon build --symlink-install
   cd ~/slam_ws   && colcon build --symlink-install
   ```

   Both SLAM scripts source `/opt/ros/jazzy/setup.bash` and `~/slam_ws/install/setup.bash` themselves,
so you do **not** need to source anything manually before running them.

3. **Make the scripts executable:**

   ```bash
   chmod +x ~/ws/src/togo_sim.sh
   chmod +x ~/slam_ws/src/lidarslam_ros2/scripts/togo/*.sh
   ```

4. **Sim time.** Every stage below runs with `use_sim_time:=true`. The simulator must be up **and
   unpaused** before starting SLAM/Nav2/ground-truth, otherwise all nodes block at `t=0` waiting on
   `/clock`.

---

## 3. Quickstart — 4 terminals, in order

Open four shells into the container. Start them in this order and let each one settle before moving on.

### Terminal 1 — Gazebo simulation

```bash
cd ~/ws/src
./togo_sim.sh
```

Brings up Gazebo with the rover. This is the source of:

- `/husky/sensors/seyond/points` — Seyond LiDAR point cloud
- `/husky/sensors/imu_0/data_raw` — IMU
- `/clock` — sim time for everything downstream
- ground-truth model pose (consumed by stage 3)
- velocity command input driven by Nav2 in stage 4

Wait until the world is loaded and running before continuing.

---

### Terminal 2 — DLIO SLAM (live against the sim)

```bash
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo
./run_live_seyond_dlio_slam.sh
```

The script prints a config banner (topics/frames/deskew), then launches
[`seyond_dlio_slam.launch.py`](lidarslam/launch/seyond_dlio_slam.launch.py) with
[`seyond_dlio_graph.yaml`](lidarslam/param/seyond_dlio_graph.yaml) as both the SLAM and DLIO param file.

Expected banner:

```
──────────────────────────────────────────────
  LIDAR  topic : /husky/sensors/seyond/points
  LIDAR  timed : /husky/sensors/seyond/points_timed
  IMU    topic : /husky/sensors/imu_0/data_raw
  LiDAR  frame : lidar3d_0_laser
  IMU    frame : imu_0_link
  Base   frame : base_link
  Odom   frame : odom
  Deskew       : true
──────────────────────────────────────────────
[slam_launch] ros2 launch PID: <pid>
```

RViz opens by default (`SLAM_RVIZ=true`). The SLAM path is published on `/dlio/path_simple`.

---

### Terminal 3 — Gazebo ground truth

```bash
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo
python3 gz_truth_path.py --ros-args \
  -p use_sim_time:=true \
  -p fixed_frame:=odom \
  -p slam_topic:=/dlio/path_simple
```

Pulls the true rover pose out of Gazebo and republishes it as a path in the `odom` frame, keyed off the
SLAM path on `/dlio/path_simple`. Use it to overlay estimate vs. truth in RViz and to measure drift.

Parameters:

| Param | Value used | Meaning |
|-------|-----------|---------|
| `use_sim_time` | `true` | must match the rest of the stack |
| `fixed_frame` | `odom` | frame the truth path is published in — matches `ODOM_FRAME` in the SLAM script |
| `slam_topic` | `/dlio/path_simple` | SLAM path this is compared/aligned against |

---

### Terminal 4 — Nav2 (planner + controller on top of SLAM)

```bash
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo
./run_nav2_only.sh
```

Launches [`rover_nav2.launch.py`](togo_navigation/launch/rover_nav2.launch.py) in its own session
(`setsid`), then polls `ros2 node list` for up to 90 s waiting on `lifecycle_manager_navigation`:

```
[nav2] Launching rover_nav2 ...
[nav2] Nav2 launch PID: <pid>
[nav2] Waiting for Nav2 lifecycle manager...
[nav2] lifecycle_manager_navigation is up. 

[nav2]  Nav2 stack is running.
[nav2]  Press Ctrl+C to shut down all nodes.
```

This consumes the SLAM map/odometry, builds costmaps, plans, and sends velocity commands to the rover in
Gazebo. Send goals from RViz (`2D Goal Pose`) or through the `navigate_to_pose` action.

If you see `WARNING: lifecycle_manager_navigation never appeared`, the stack didn't come up fully —
check Terminal 2 is actually publishing before retrying.

---

## 4. Shutdown

`Ctrl+C` in each terminal, ideally in reverse order (Nav2 → truth → SLAM → sim).

Both shell scripts trap `SIGINT` / `SIGTERM` / `SIGHUP` / `EXIT` and kill the **entire process group**, so
every node `ros2 launch` spawned goes down — not just the launch process. Grace period is
`SHUTDOWN_GRACE_SEC` (default `5`) seconds, then `SIGKILL`.

`run_nav2_only.sh` additionally does a name-based sweep afterward and force-kills any survivor from:

```
controller_server        smoother_server          planner_server
behavior_server          bt_navigator             waypoint_follower
velocity_smoother        lifecycle_manager        slam_to_occupancy_grid
hazard_patch_node        global_costmap_composer   goal_safety_relay
local_hazard_grid        occupancy_grid_to_points  debug_map_publisher
rviz2
```

Clean exit prints `[nav2] All nodes confirmed dead. `. If you instead see
`WARNING: '<node>' still alive — force killing`, it was cleaned up but shut down uncleanly.

---

## 5. Configuration — SLAM script

`run_live_seyond_dlio_slam.sh` takes everything from the environment; no need to edit the file.

```bash
LIDAR_TOPIC=/my/points DLIO_DESKEW=false SLAM_RVIZ=false ./run_live_seyond_dlio_slam.sh
```

**Sensor wiring**

| Var | Default |
|-----|---------|
| `LIDAR_TOPIC` | `/husky/sensors/seyond/points` |
| `LIDAR_TOPIC_TIMED` | *derived* — `${LIDAR_TOPIC}_timed` |
| `IMU_TOPIC` | `/husky/sensors/imu_0/data_raw` |
| `LIDAR_FRAME` | `lidar3d_0_laser` |
| `IMU_FRAME` | `imu_0_link` |
| `BASE_FRAME` | `base_link` |
| `ODOM_FRAME` | `odom` |

**Param files**

| Var | Default |
|-----|---------|
| `COMBINED_PARAM_FILE` | `lidarslam/param/seyond_dlio_graph.yaml` |
| `SLAM_PARAM_FILE` | falls back to `COMBINED_PARAM_FILE` |
| `DLIO_PARAM_FILE` | falls back to `COMBINED_PARAM_FILE` |

**Behaviour**

| Var | Default | Notes |
|-----|---------|-------|
| `SLAM_RVIZ` | `true` | open RViz |
| `DLIO_DESKEW` | `true` | IMU-based motion compensation of the cloud |
| `ENABLE_MAP_SAVE_PULSE` | `false` | periodic map-save trigger |
| `TIMED_CLOUD_SCAN_PERIOD` | `0.0666666667` | 15 Hz LiDAR — per-point timestamp reconstruction |
| `TIMED_CLOUD_REVERSE_COLUMNS` | `false` | flip column order if deskew is backwards |
| `SHUTDOWN_GRACE_SEC` | `5` | seconds before `SIGKILL` |

Hardcoded in the launch call: `use_sim_time:=true`, `map_save_period:=60`.

**Pre-flight checks** — the script exits non-zero if the launch file or either param file is missing.

---

## 6. Configuration — Nav2 script

| Var | Default | Notes |
|-----|---------|-------|
| `SLAM_WS` | `/home/er4-user/slam_ws` | workspace root to source |
| `USE_SIM_TIME` | `true` | keep `true` in sim |
| `AUTOSTART` | `true` | lifecycle nodes auto-configure/activate |
| `RVIZ` | `false` | off by default — SLAM RViz is usually already open |
| `DEBUG_MAP` | `false` | enables `debug_map_publisher` |
| `USE_SLAM_MAP` | `true` | consume the live SLAM map instead of a static one |
| `USE_PATCH_HAZARD_MAP` | `true` | enable hazard patching in the costmap |
| `REQUEST_INITIAL_MAP_SAVE` | `true` | ask SLAM for a map snapshot at startup |
| `PARAMS_FILE` | *(empty)* | set to override the launch file's default Nav2 params; passed as `params_file:=` |
| `SHUTDOWN_GRACE_SEC` | `5` | seconds before `SIGKILL` |

Example:

```bash
RVIZ=true DEBUG_MAP=true ./run_nav2_only.sh
PARAMS_FILE=/home/er4-user/slam_ws/src/lidarslam_ros2/togo_navigation/params/my_nav2.yaml ./run_nav2_only.sh
```

**Pre-flight checks** — exits non-zero if `rover_nav2.launch.py` is missing, or if `PARAMS_FILE` is set
but doesn't exist.

---

## 7. Data flow

```
togo_sim.sh  (~/ws)
   Gazebo
     ├── /clock ─────────────────────────────► everything (use_sim_time:=true)
     ├── /husky/sensors/seyond/points ──┐
     ├── /husky/sensors/imu_0/data_raw ─┤
     └── ground-truth model pose ───────┼──────────────┐
                                        │              │
                                        ▼              ▼
                     run_live_seyond_dlio_slam.sh   gz_truth_path.py
                        DLIO + graph SLAM              truth path
                        ├── /dlio/path_simple ───────► (comparison in odom)
                        ├── odom → base_link TF
                        └── map / occupancy grid
                                        │
                                        ▼
                                run_nav2_only.sh
                            costmaps → planner → controller
                                        │
                                        ▼
                              velocity commands ──► rover in Gazebo
```

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Nothing moves, no TF, nodes silent | Sim not running or paused — `/clock` isn't ticking. Start Terminal 1 first and unpause. |
| `Missing .../seyond_dlio_slam.launch.py` | Repo not mounted at `~/slam_ws/src/lidarslam_ros2`, or workspace not built. |
| `Missing .../seyond_dlio_graph.yaml` | Param file moved/renamed — override with `COMBINED_PARAM_FILE=...`. |
| `lifecycle_manager_navigation never appeared` | SLAM isn't publishing yet (Nav2 started too early), or a Nav2 node crashed on startup. Confirm Terminal 2 is up, then relaunch. |
| `Nav2 launch exited prematurely` | Bad params file or a missing node executable — scroll up in the same terminal for the real launch error. |
| SLAM path drifts/warps badly | Try `DLIO_DESKEW=false`, or flip `TIMED_CLOUD_REVERSE_COLUMNS=true` if the per-point timestamp order is reversed. |
| Deskew looks wrong / stretched clouds | `TIMED_CLOUD_SCAN_PERIOD` doesn't match the actual LiDAR rate — default `0.0666666667` assumes 15 Hz. |
| Truth path missing or in the wrong place | `fixed_frame` must match `ODOM_FRAME` (`odom`), and `slam_topic` must match the SLAM path topic (`/dlio/path_simple`). |
| Nodes survive `Ctrl+C` | Should not happen — `run_nav2_only.sh` sweeps by name. If it does, `pkill -f <node_name>` manually. |
| Second run behaves oddly | Leftover nodes from a previous session. Verify with `ros2 node list` before relaunching. |

Useful checks:

```bash
ros2 node list
ros2 topic hz /clock
ros2 topic hz /husky/sensors/seyond/points
ros2 topic hz /husky/sensors/imu_0/data_raw
ros2 topic echo /dlio/path_simple --once
ros2 run tf2_tools view_frames
```

---

## 9. Cheat sheet

```bash
# 1 — sim
cd ~/ws/src && ./togo_sim.sh

# 2 — SLAM
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo && ./run_live_seyond_dlio_slam.sh

# 3 — ground truth
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo && python3 gz_truth_path.py --ros-args \
  -p use_sim_time:=true -p fixed_frame:=odom -p slam_topic:=/dlio/path_simple

# 4 — Nav2
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo && ./run_nav2_only.sh
```

Stages 2–4 are independent processes: you can restart SLAM or Nav2 without touching the simulator, as long
as you respect the ordering (SLAM before Nav2).
