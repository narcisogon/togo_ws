# Live Real Husky/Seyond — DLIO + Graph SLAM (Flat Launch)

Runs DLIO odometry and graph SLAM directly on live Husky/Seyond sensor data.
No timestamp corrector, no nested launch includes. `/iv_points` feeds DLIO
directly. `use_sim_time` is always false.

This launch **replaces** `run_live_husky_real_dlio.sh` as the primary way to
run the live stack. If the flat launch does not work, `run_live_husky_real_dlio.sh`
is still on the rover and is known to work — fall back to it while diagnosing.

Script: [`scripts/togo/run_live_real_flat.sh`](scripts/togo/run_live_real_flat.sh)

---

## 1. Files

| Role | Path |
|---|---|
| Launch file | `lidarslam/launch/seyond_dlio_live_real_flat.launch.py` |
| Parameters | `lidarslam/param/seyond_dlio_live_real_flat.yaml` |
| Run script | `scripts/run_live_real_flat.sh` |

---

## 2. Prerequisites

**1. Build and source the workspace:**

```bash
cd /home/er4-user/slam_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Verify the Seyond driver is running:**

```bash
ros2 topic echo /iv_points --once
```

**3. Verify the IMU is running:**

```bash
ros2 topic echo /husky/sensors/imu_0/data --once
```

**4. The rover bringup must be publishing these TF transforms:**

```
base_link  ->  lidar3d_0_laser
base_link  ->  imu_0_link
```

If running without the full rover bringup (bench test only), set
`PUBLISH_SENSOR_STATIC_TF=true` — this tells the launch to publish
those transforms itself.

---

## 3. Quickstart

```bash
bash /home/er4-user/slam_ws/src/lidarslam_ros2/scripts/run_live_real_flat.sh
```

Keep the rover **completely stationary for the first 3 seconds** after launch.
DLIO runs IMU gyro/accel/gravity calibration during that window.

What happens, in order:

1. Sources ROS 2 Jazzy and `slam_ws/install/setup.bash`.
2. Pre-flight: checks the launch file and YAML exist, warns about any stale
   nodes from a previous session (does not block launch).
3. Starts [`seyond_dlio_live_real_flat.launch.py`](lidarslam/launch/seyond_dlio_live_real_flat.launch.py)
   in its own process group via `setsid`. RViz opens.
4. Polls up to 40 s for the sensor publishers to be discovered and for DLIO
   to subscribe to `/iv_points` — ensures the node is ready before diagnostics run.
5. Waits `STARTUP_DIAGNOSTIC_DELAY` seconds, then prints a full diagnostic
   snapshot: topic endpoints, receive rates, DLIO odometry check.
6. Runs until `Ctrl+C`. On exit, calls `/map_save` if the service is present,
   then shuts down the process group cleanly (SIGINT → SIGTERM → SIGKILL).

---

## 4. Stopping

Press `Ctrl+C` **once**. The script sends SIGINT to the entire launch process
group, waits for a clean exit, escalates to SIGTERM then SIGKILL if needed,
removes the tmpfs PCD cache, and reports any nodes that did not exit.

> Do not press Ctrl+C a second time unless the first shutdown has clearly hung.

### If Ctrl+C does not shut it down

Open a second terminal and run:

```bash
ros2 node list | grep -E 'dlio|graph|rviz'
```

Kill the owning launch or the individual nodes, then confirm they are gone:

```bash
ros2 node list   # should return nothing relevant
```

---

## 5. Configuration

All settings have working defaults. Override any of them before running:

| Variable | Default | Notes |
|---|---|---|
| `SLAM_WS` | `/home/er4-user/slam_ws` | workspace root |
| `REPO` | `$SLAM_WS/src/lidarslam_ros2` | source root |
| `PARAM_FILE` | `$REPO/lidarslam/param/seyond_dlio_live_real_flat.yaml` | single combined param file |
| `LIDAR_TOPIC` | `/iv_points` | Seyond point cloud topic |
| `IMU_TOPIC` | `/husky/sensors/imu_0/data` | IMU topic |
| `DLIO_DESKEW` | `false` | validate timing and extrinsics before enabling |
| `DLIO_GROUND_CONSTRAINT` | `false` | enable only on confirmed flat terrain |
| `DLIO_GROUND_BASELINK_HEIGHT` | `0.30` | expected base_link height above ground in metres |
| `PUBLISH_SENSOR_STATIC_TF` | `false` | set `true` only for bench runs without rover bringup |
| `GRAPH_MAP_SAVE_DIR` | `$REPO/output/husky_seyond_live_real` | final map output |
| `GRAPH_DEM_OUTPUT_DIR` | `$GRAPH_MAP_SAVE_DIR/lunar_dem` | DEM output |
| `GRAPH_PCD_CACHE_DIR` | `/dev/shm/graph_slam_pcd_cache_live_real_<pid>` | unique per-run tmpfs cache |
| `ENABLE_MAP_SAVE_PULSE` | `false` | periodically call `/map_save` automatically |
| `MAP_SAVE_PERIOD` | `60` | seconds between automatic map saves |
| `SLAM_RVIZ` | `true` | open RViz |
| `STARTUP_DIAGNOSTIC_DELAY` | `10` | seconds after sensor ready before printing diagnostics |

Examples:

```bash
# Run with deskew enabled and RViz off
DLIO_DESKEW=true \
SLAM_RVIZ=false \
bash scripts/run_live_real_flat.sh

# Save maps to a custom directory
GRAPH_MAP_SAVE_DIR=/data/maps/run_001 \
bash scripts/run_live_real_flat.sh

# Bench test without full rover bringup
PUBLISH_SENSOR_STATIC_TF=true \
bash scripts/run_live_real_flat.sh
```

---

## 6. Saving the Map Manually

While the stack is running, open a second terminal and call:

```bash
source /opt/ros/jazzy/setup.bash
source /home/er4-user/slam_ws/install/setup.bash
ros2 service call /map_save std_srvs/srv/Empty
```

The map and DEM are also saved automatically when the script exits cleanly.

Output lands under `GRAPH_MAP_SAVE_DIR`:

```
output/husky_seyond_live_real/
├── pointcloud_map/
├── map.pcd
├── pose_graph.g2o
└── lunar_dem/
```

---

## 7. Verifying the Stack is Healthy

After the startup diagnostics print, open a second terminal and run:

```bash
# Is DLIO publishing odometry?
ros2 topic hz /dlio/odometry

# Is graph SLAM publishing the corrected map transform?
ros2 topic hz /modified_map_timed

# Are there unexpected nodes from a previous session?
ros2 node list | grep -E 'dlio|graph|rviz'

# Watch the live DLIO diagnostic history
tail -f /tmp/dlio_live_diagnostic_history.log
```

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `ERROR: missing .../seyond_dlio_live_real_flat.launch.py` | Workspace not built or `SLAM_WS` is wrong — run `colcon build` |
| `ERROR: rover sensor publishers not found within 40 s` | Seyond or IMU driver is not running — check with `ros2 topic echo /iv_points --once` |
| `ERROR: DLIO did not subscribe to /iv_points within 40 s` | `dlio_odom_node` crashed on init — scroll up for a Python traceback or missing parameter error |
| `WAITING: no DLIO odometry yet` in diagnostics | Still in IMU calibration — wait 10-15 s; if it persists check extrinsics in the YAML |
| `WARNING: nodes from an earlier session are still alive` | Previous run did not shut down cleanly — kill the old nodes before retrying |
| Tuning changes have no effect | Confirm you are editing `seyond_dlio_live_real_flat.yaml`, not an old override file |
| Won't die on Ctrl+C | See section 4 — `ros2 node list \| grep -E 'dlio\|graph\|rviz'` then kill manually |

Useful checks:

```bash
ros2 topic hz /iv_points
ros2 topic hz /husky/sensors/imu_0/data
ros2 node list
ros2 topic echo /dlio/odometry --once
```

---

## 9. Fallback

If the flat launch fails and the cause is not quickly identified,
`run_live_husky_real_dlio.sh` is on the rover and is known to work.
Use it to keep operations running while the flat launch is debugged:

```bash
bash /home/er4-user/slam_ws/src/lidarslam_ros2/scripts/run_live_husky_real_dlio.sh
```