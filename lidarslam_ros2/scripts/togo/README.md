# TOGO/Husky SLAM and Nav2 Scripts

The operator workflow is documented in [the SLAM README](../../README.md).
Backend hazard settings and diagnostics are documented in
[backend hazard mapping](../../graph_based_slam/hazard/README.md).

## Primary entrypoints

| Script | Purpose | Clock | Main YAML |
| --- | --- | --- | --- |
| `run_live_seyond_dlio_slam.sh` | Gazebo Seyond simulation using DLIO + graph SLAM | Sim `/clock` | `lidarslam/param/seyond_dlio_graph.yaml` |
| `run_replay_husky_real.sh` | Corrected/retimed real Husky bag | Bag `/clock` | Standalone `seyond_dlio_replay.yaml` |
| `run_live_husky_real_dlio.sh` | Real live Husky/Seyond sensors | Wall time | `seyond_dlio_graph.yaml`, then `seyond_dlio_live_real.yaml` |
| `run_nav2_only.sh` | Nav2 planning and driving using the backend hazard grid | Selected with `USE_SIM_TIME` | `togo_navigation/config/nav2_slam_params.yaml` |

`run_live_seyond_slam_integrated.sh` is the older RKO-LIO simulation path. It
is retained for comparison but is not the current Husky/Seyond default.

## Bag timing tools

- `audit_replay_bag_timing.py`: reports LiDAR/IMU rates, header alignment,
  native point-time span, missing frames, and storage gaps.
- `rewrite_lidar_timestamps_bag.py`: copies a bag while shifting the selected
  LiDAR header and every absolute FLOAT64 point timestamp.
- `retime_lidar_storage_to_imu.py`: reschedules corrected LiDAR records against
  the recorded IMU storage timeline without changing message payloads.
- `timed_map_diag_once.py`: validates `/modified_map_timed` time precision and
  span.

## Diagnostics and utilities

- `dlio_diag_once.py`: prints one labeled DLIO diagnostic message.
- `monitor_rko_diagnostics.sh`: legacy RKO-LIO monitoring.
- `save_seyond_map_clean.sh`: calls `/map_save`.
- `record_seyond_slam_bag.sh`: records the simulation SLAM topics.
- `odom_to_path.py`: publishes a lightweight path from odometry.
- `gazebo_pose_to_aligned_path.py`: publishes the simulation reference path.

The root `/scripts` copies are Docker compatibility wrappers. Edit the scripts
under `lidarslam_ros2/scripts/togo/`.

Hazards now run in a worker thread inside `graph_based_slam`. Configure them in
`graph_based_slam.ros__parameters.hazard/*` in `seyond_dlio_graph.yaml`.
See [backend hazard mapping](../../graph_based_slam/hazard/README.md) for the
algorithm, inflation margins, diagnostics, and rebuild instructions.
