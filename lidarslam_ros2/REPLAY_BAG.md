# Replaying Recorded Bags — Seyond + DLIO + Graph SLAM

Runs the SLAM stack against a **recorded rosbag** instead of the live Gazebo sim. Same DLIO + graph-SLAM
backend, same params — only the data source changes.

Script: [`scripts/togo/run_replay_husky_real.sh`](scripts/togo/run_replay_husky_real.sh)

---

## 1. Put your bags here, and edit the path

Bags belong in:

```
/home/er4-user/slam_ws/src/lidarslam_ros2/bags/
```

The script defaults to **one specific bag**:

```bash
BAG_PATH=${BAG_PATH:-/home/er4-user/slam_ws/src/lidarslam_ros2/bags/rosbag2_2026_07_27-21_54_36}
```

 **You must change the last part of that path to your own bag directory name.** Either edit the default
in the script, or override it per-run:

```bash
BAG_PATH=/home/er4-user/slam_ws/src/lidarslam_ros2/bags/rosbag2_2026_08_14-09_12_02 \
  ./run_replay_husky_real.sh
```

Point it at the **bag directory** (the folder containing `metadata.yaml` + `.mcap`/`.db3`), not the file
inside it. The script aborts immediately if the path doesn't exist:

```
[replay] Missing: /home/er4-user/slam_ws/src/lidarslam_ros2/bags/<name>
```

---

##  2. Params: the override does not work

The script passes a replay override file, but **it has no effect**. In practice the run is configured
entirely by:

```
lidarslam/param/seyond_dlio_graph.yaml
```

That single file drives both DLIO and the graph backend, exactly as in the live setup. So:

- Editing `seyond_dlio_graph.yaml` **does** change replay behaviour.
- Editing `seyond_dlio_replay.yaml` **does not** — don't waste time tuning it.
- `DLIO_OVERRIDE_PARAM_FILE` is still checked for existence at startup (the script exits if the file is
  missing), so the file must exist even though its contents are ignored.

If you need different values for replay, edit `seyond_dlio_graph.yaml` directly — or copy it, edit the
copy, and pass it as `SLAM_PARAM_FILE`.

---

## 3. Quickstart

```bash
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo
BAG_PATH=/home/er4-user/slam_ws/src/lidarslam_ros2/bags/<your_bag_dir> \
  ./run_replay_husky_real.sh
```

What happens, in order:

1. Sources ROS 2 Jazzy + `~/slam_ws/install/setup.bash`.
2. Pre-flight: verifies the bag, launch file, and all three param files exist.
3. Launches SLAM via [`seyond_dlio_replay.launch.py`](lidarslam/launch/seyond_dlio_replay.launch.py),
   which wraps [`seyond_dlio_slam.launch.py`](lidarslam/launch/seyond_dlio_slam.launch.py). RViz opens.
4. Polls up to **30 s** for DLIO to actually subscribe to `/iv_points` and
   `/husky/sensors/imu_0/data` — the bag is not started until both have a subscriber, so no early
   messages are lost.
5. Plays the bag with `--clock` at **1.5×** by default.
6. On bag completion, calls the `/map_save` service automatically (if present).
7. Leaves SLAM running so you can inspect RViz. `Ctrl+C` to shut everything down.

Expected startup:

```
[replay] SLAM launch PID: <pid>
[replay] Waiting for DLIO sensor subscriptions...
[replay] DLIO subscriptions ready (lidar=1 imu=1)
[replay] Starting bag at 1.5x ...
[replay] Topics: /iv_points /husky/sensors/imu_0/data /fixposition/gnss1
[replay] Bag play PID: <pid>
```

---

## 4. Topics — note these differ from the live setup

Replay defaults to the **recorded hardware topic names**, not the sim ones:

| | Replay | Live sim |
|---|---|---|
| LiDAR | `/iv_points` | `/husky/sensors/seyond/points` |
| IMU | `/husky/sensors/imu_0/data` | `/husky/sensors/imu_0/data_raw` |
| GNSS | `/fixposition/gnss1` | — |

Only those three topics are replayed by default:

```bash
BAG_TOPICS=${BAG_TOPICS:-"/iv_points /husky/sensors/imu_0/data /fixposition/gnss1"}
```

Override the list:

```bash
BAG_TOPICS="/iv_points /husky/sensors/imu_0/data /tf /tf_static" ./run_replay_husky_real.sh
```

Replay **everything** in the bag:

```bash
BAG_TOPICS="" ./run_replay_husky_real.sh
```

 If your bag uses different topic names, the subscription wait times out after 30 s:

```
[replay] Timed out waiting for DLIO subscriptions (lidar=0 imu=0)
```

Check what's actually in the bag first:

```bash
ros2 bag info /home/er4-user/slam_ws/src/lidarslam_ros2/bags/<your_bag_dir>
```

---

## 5. Configuration

| Var | Default | Notes |
|---|---|---|
| `BAG_PATH` | `.../bags/rosbag2_2026_07_27-21_54_36` |  **change this to your bag** |
| `BAG_TOPICS` | `/iv_points /husky/sensors/imu_0/data /fixposition/gnss1` | space-separated; `""` = all topics |
| `PLAYBACK_RATE` | `1.5` | `1.0` = real time; lower it if the frontend lags |
| `SLAM_PARAM_FILE` | `lidarslam/param/seyond_dlio_graph.yaml` | the file that actually governs the run |
| `DLIO_PARAM_FILE` | = `SLAM_PARAM_FILE` | same combined file |
| `DLIO_OVERRIDE_PARAM_FILE` | `lidarslam/param/seyond_dlio_replay.yaml` |  ignored, but must exist |
| `DLIO_DESKEW` | `true` | IMU motion compensation |
| `SYNTHETIC_TIMING` | `false` | set `true` only if the recorded cloud lacks per-point timestamps |
| `SLAM_RVIZ` | `true` | open RViz |
| `STARTUP_DIAGNOSTIC_DELAY` | `10` | seconds before the diagnostic dump |
| `SHUTDOWN_GRACE_SEC` | `5` | grace period before `SIGKILL` |
| `SLAM_WS` | `/home/er4-user/slam_ws` | workspace root |

Examples:

```bash
PLAYBACK_RATE=1.0 ./run_replay_husky_real.sh                   # real time
PLAYBACK_RATE=0.5 SLAM_RVIZ=false ./run_replay_husky_real.sh   # slow, headless
SYNTHETIC_TIMING=true ./run_replay_husky_real.sh               # cloud has no point timestamps
```

 **On `PLAYBACK_RATE`:** 1.5× gives the frontend 33 % less wall-time per scan. If the map visibly falls
behind, or you see `lag_spike` / `imu_age` events in `/tmp/dlio_diagnostic_history.log`, drop to `1.0` or
lower. Replay speed changes results — a rate the frontend can't keep up with produces worse odometry than
the same bag played slower.

---

## 6. Sim time

Playback uses `--clock`, so the bag publishes `/clock` and everything runs on recorded time. This matches
`use_sim_time: true` in `seyond_dlio_graph.yaml`.

 Consequence: **nothing advances until the bag starts.** If the bag fails to start, SLAM sits at t=0 and
looks hung rather than broken.

---

## 7. Map saving

The script calls `/map_save` automatically once the bag finishes. Output lands under `map_save_dir` from
`seyond_dlio_graph.yaml`:

```
output/husky_seyond_graph/
├── pointcloud_map/          # Autoware grid tiles + metadata
├── map.pcd                  # full downsampled cloud
├── map_projector_info.yaml
├── pose_graph.g2o
└── lunar_dem/               # 9 ESRI .asc layers + metadata.yaml
```

To save again mid-run:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

---

## 8. Shutdown

SLAM stays up after the bag ends so you can inspect the result. Press `Ctrl+C` when you're done.

### If `Ctrl+C` doesn't shut it down

Open a second terminal into the container and run:

```bash
pkill -f ros2
```

That kills every lingering ROS process. Confirm nothing survived:

```bash
ros2 node list        # should return nothing
```

 This kills **all** ROS 2 processes in the container, not just replay — so don't use it if you have
other ROS nodes running that you want to keep.

---

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `[replay] Missing: <path>` | Bag path wrong —  almost always the bag directory name |
| `Timed out waiting for DLIO subscriptions` | Bag topic names don't match `BAG_TOPICS`; check `ros2 bag info` |
| `SLAM launch exited before subscriptions became ready` | Launch error — scroll up for the real message |
| Everything silent, nothing moves | Bag never started → no `/clock`; check `ros2 topic hz /clock` |
| Frontend lagging, map behind | Lower `PLAYBACK_RATE` |
| Tuning changes have no effect | You edited `seyond_dlio_replay.yaml` — edit `seyond_dlio_graph.yaml` instead |
| Deskew looks wrong | Recorded cloud may lack point timestamps → try `SYNTHETIC_TIMING=true` |
| Won't die on `Ctrl+C` | `pkill -f ros2` in another terminal (see §8) |

Useful checks:

```bash
ros2 bag info <bag_dir>
ros2 topic hz /clock
ros2 topic hz /iv_points
ros2 node list

```
