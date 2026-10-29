# SLAM speed-up and accuracy plan (handoff brief, 2026-10-27)

For the next Claude Code session in this repo. Before changing anything, read this file, then sections 0 and 5
of `src/lunar_world_gen/README.md`. The memory index and the `lunar-world` and `togo-sim-slam-nav` skills load
on their own.

## Status (2026-10-28): round 1 coded, benchmarks pending

Round 1 = the low-risk items only, in one build: Step 1 (all four), Step 2's exact speed-ups, Step 4.3
investigated (no change needed). Everything new is switchable at runtime (`hazard/fit_threads`,
`hazard/fit_early_exit`, `odom/numThreads`), so a bad result is bisected with replays, not rebuilds. Two
accuracy experiments, an `ADAPTIVE` regularisation and ring-aware normals, were built, measured worse, and
REMOVED from the code on 2026-10-28; their results stay below under Step 4.1 as negative findings. Labels, three runs each: `speed` (all speed items, against
`baseline`; accuracy must stay inside the baseline spread) and `reg_adaptive` (the speed build with ADAPTIVE,
against `speed`). Medium and high-risk items (Step 3, the lag fix, the hazard restructure, footprint
traversability, paging, IEKF) are untouched and wait for the user's decision after these numbers.
Corrections to the 2026-10-27 text are marked DONE / Found in the steps below.

**Results (2026-10-28, 3 runs per label, `compare__baseline__vs__speed.md`, `compare__speed__vs__reg_adaptive.md`
in `~/togo_assets/lunar/bags/slam_runs/`):**
- `speed` vs `baseline`: all SLAM CPU 490 → 365 % (helpers 1.6 cores → 0.11), DLIO 145 → 116 %, GICP 18.8 → 9.8 ms
  mean / 44 → 27 p95 / 217 → 111 max, backend RAM peak 2110 → 1549 MB and growth 329 → 235 MB/min (exactly the
  removed copies), hazard update 249 → 124 ms, dropped clouds 6223 → 4429, backend CPU 161 → 219 % (the mapper now
  processes three times more scans). Accuracy did not degrade: ATE 0.232 ± 0.081 → 0.181 ± 0.029 m, heading lag
  46.6 → 38.2 ms (the GICP time it no longer waits for), vertical RMSE −54 %, end drift −56 %, run-to-run spread
  much smaller. Keep everything.
- `reg_adaptive` vs `speed`: ATE 0.181 → 0.190 m (inside spread) but ATE max +31 %, RPE 1 m +16 %, zig-zag RMS
  +17 % and p95 +19 %, per-scan correction +9 %; speed unchanged. Not adopted; PLANE stays. The option remains
  selectable.
- Height scale: `vertical_vs_climb_pct` fell from 3.06 ± 1.21 to 0.40 ± 0.51 % with the speed changes alone, and the
  per-segment `vertical_rate_vs_slope_pct` is 0.06-1.26 % with correlation ≤ 0.05 in all six runs: drift noise,
  not a scale error, as the analysis said.
- Still open, measured: the backend's `Memory stores:` log accounts for about 570 MB of its 1.5 GB at lap end
  (map 195, timed voxels 54, timed message 19, hazard grid 162, hazard observations 138; plus 68 MB in tmpfs), so
  about 0.9 GB sits outside those stores (candidates: middleware buffers for the 19 MB timed map, DEM job
  transients, allocator fragmentation); the mapper still skips 63 % of scans, now limited by its serial parts
  (projection, merge, inflation, publish), not the fit.

## Goal

Make the SLAM pipeline use less CPU and memory without losing accuracy or reliability. The pipeline is the DLIO
frontend, the `graph_based_slam` backend with its hazard mapper, the path-helper scripts, and Nav2. Where it is
cheap, also fix the accuracy defects the benchmark measured. Every change is proven on the replay benchmark
before it is kept.

## The user's rules

- **Excluded methods:**
  - no wheel odometry as a factor or prior (slip on the regolith is likely);
  - no GPU;
  - loop closure stays off (the user finds it unreliable);
  - do not re-enable or tune DLIO's correction gates (`rejectBadCorrections`, innovation gate, recovery, ...).
    They never worked. Weighting by uncertainty is the agreed direction instead.
- **Effort:** do not spend an hour to gain 1 %. Rank work by measured gain. If a change does not clearly move its
  target metric after about an hour, stop and report.
- **Editing and git:**
  - No git commands; the user commits and pushes.
  - Edit outside `src/lunar_world_gen` only with the user's explicit OK. That covers `lidarslam_ros2/`, the Nav2
    config and `src/togo*`. Ask once per step and list the files.
- **Scope:**
  - The simulated sensor stays as it is; the realistic Robin W mode is separate work.
  - Generated data goes under `~/togo_assets/` in WSL, never into the repo.
- **WSL and Docker hazards** (details in the `lunar-world` skill and the tooling memory):
  - never kill processes by name pattern: the container's `sleep infinity` is visible from WSL;
  - use `docker compose start dev`, not `up`, and `env -u CYCLONEDDS_URI` in shells that sourced `env.sh`;
  - never edit a bash script while it runs;
  - stop every helper process you start.

## How to measure (all of this exists)

```bash
cd $REPO      # /mnt/c/Users/Username/OneDrive/Desktop/togo_ws-slam-nav/togo_ws-slam-nav, in WSL
./src/lunar_world_gen/scripts/benchmark_slam.sh --label <change> [--config <yaml>] --compare-to baseline
```

- **What a run is.** Three replays of the one-lap bag `~/togo_assets/lunar/bags/apollo16_lap1_20261027`, about
  25 minutes in all. Each run lands in `~/togo_assets/lunar/bags/slam_runs/`.
  - `compare__baseline__vs__<change>.md` lists every metric with a verdict: *better*, *worse*, or `~` for within
    noise.
  - `compare__baseline__vs__<change>.png` plots the errors along the lap. The lap's places are marked: start,
    rocks, slope, plain.
- **Noise.** Replays are not deterministic: identical runs gave an ATE of 0.17, 0.32 and 0.20 m. Always compare
  labels of 3 runs. ATE is only reliable for big changes; RPE, zig-zag, heading lag, GICP time and CPU/RAM are
  much steadier.
- **Coverage.** The replay covers DLIO, the backend and hazard mapper, and the helper scripts. Their per-process
  CPU and RAM are in each run's `metrics.json` under `resources.processes`. The replay does **not** run Nav2;
  Nav2 changes need a live drive (README section 4) measured with `tools/proc_sampler.py`.
- **Rebuilding.** C++ changes in `lidarslam_ros2` need a rebuild in the container:
  `docker compose exec dev bash -lc "cd ~/slam_ws && MAKEFLAGS=-j4 colcon build --executor sequential --symlink-install --packages-select <pkgs> --packages-ignore gtsam --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON" (sequential and -j4: the default parallel build of both packages exhausts WSL's 16 GB and the compiler gets OOM-killed; the Release flag is explicit so a rebuild cannot lose it)`.
  Each run records the DLIO binary time (`dlio_build` in `run_info.txt`), so results show which build they used.
- **Accuracy guard.** No *worse* verdict on ATE RMSE, RPE 10 m, zig-zag, vertical RMSE or heading lag, unless
  the user accepts the trade.

## Baseline (label `baseline`, 3 runs, 2026-10-27)

| what | value |
|---|---|
| ATE RMSE (4-DoF) | 0.232 ± 0.081 m (along 0.161, cross 0.160, vertical 0.041) |
| RPE 10 m / zig-zag RMS | 2.04 % / 57 mm |
| heading error RMSE / heading lag in turns | 0.77° / 47 ± 5 ms |
| vertical drift per metre climbed | +3.1 ± 1.2 % |
| GICP solve time | mean 18.8 ms, p95 44 ms, max 217 ms; DLIO 76.8 ms CPU per scan |
| hazard map update | mean 249 ms, p95 468 ms; 6,223 clouds dropped per lap |
| all SLAM processes | 490 % CPU (about 5 cores), 3.0 GB peak |

Per process, the same in all 3 runs (6-minute lap):

| process | CPU | RAM peak | RAM growth |
|---|---|---|---|
| `graph_based_slam_node` | 161 % | 2.1 GB | **+315-331 MB/min** |
| `dlio_odom_node` | 145 % | 550 MB | +84 MB/min |
| `pose_to_path.py` | **99.6 %** | 77 MB | +2 MB/min |
| `gazebo_pose_to_aligned_path.py` | **63-68 %** | 62 MB | 0 |
| `dlio_map_node` | 20 % | 215 MB | +12 MB/min |

## Plan, ranked by the measured baseline

The expected gains are estimates from the earlier code analysis; the benchmark decides.

### Step 1: quick wins (hours)

1. **Path-helper scripts: about 1.6 cores, a third of all SLAM CPU.** DONE 2026-10-28 (label `speed`).
   - Found: `pose_to_path.py` republished the whole 12000-pose path on every 100 Hz `/dlio/pose` message
     (about 100 MB/s of serialisation, one core); the launch passed it two parameters it never declared.
     `gazebo_pose_to_aligned_path.py` had no input in replays (a bag carries no `/gazebo/dynamic_pose`) and
     still cost most of a core, most likely rclpy processing the ~1 kHz `/clock` under `use_sim_time`.
   - Done: `seyond_dlio_slam.launch.py` starts `odom_to_path.py` instead (a pose per 0.1 m from
     `/dlio/odometry`, published at most twice a second, capped), and skips the reference-path node when
     `reference_path:=false` or `SLAM_REFERENCE_PATH=false`, which the replay sets. Same topic
     `/dlio/path_simple`, so RViz configs are unchanged.
   - Also: `gazebo_gt_path_publisher.py` (the truth line in RViz) rewritten with the scorer's anchoring
     (first real `/dlio/odometry` pose, truth interpolated at that stamp) and made cheap; the benchmark runs
     without it (`--no-truth-path`), since the score reads the bag's `ground_truth.tum`.
2. **Backend memory: +320 MB/min,** about 4× DLIO's growth. It would fill WSL's 15.9 GB in under an hour of
   driving. Low-risk half DONE 2026-10-28 (label `speed`); the rest is medium risk and not planned.
   - Found (code review, no leak): the backend held every map point in four full-resolution copies at 32 B
     each (per-submap transformed cloud and serialised message in an assembly cache, the assembled cloud,
     and a retained MapArray), plus the compressed submap file in `/dev/shm` (RAM outside RSS), the 15 cm
     timed voxel map, and the hazard structures (about 140 B per dense 5 cm cell plus about 260 B per
     observed cell). At lap end: 5.85 M points, 2.06 GB RSS.
   - Done: the assembly cache is removed and the MapArray is built only when it is published (a save or a
     full rebuild; retained only with `publish_modified_map_array_on_append: true`), so one copy of the
     points remains in the process. A `Memory stores:` log line every 30 s reports each store's size,
     including the `/dev/shm` cache and the hazard grid, which is the measurement the plan asked for.
   - Not done, deliberately: a local window or paging. The planner, the map save and the DEM need the whole
     map (see the review notes). The hazard cell shrink was also left out this round: invasive for a
     RAM-only gain.
3. **Release build check** (minutes). DONE: both already Release (DLIO's CMakeLists forces it, the Dockerfile
   builds the backend with Release and `-O2`). Fixed the tooling instead: the rebuild command above passes the
   flag explicitly, and `run_info.txt`'s `dlio_build=` line (blank before: the install is symlinked and the
   check looked for regular files) now records the binary's build time.
4. **Thread budget.** DONE as two knobs, defaults chosen without a tuning run (label `speed`):
   `odom/numThreads: 6` for DLIO's GICP and deskew (was all 12 hardware threads), `hazard/fit_threads: 4` for
   the backend's plane fits (the new parallel loop). The backend has no other OpenMP work and DLIO's submap
   rebuild is serial, so there is nothing else to cap. Target unchanged: GICP p95 and max (44 and 217 ms now).

### Step 2: hazard mapper (the biggest CPU win, and more reliable)

- **Faster layout, same math:** exact speed-ups DONE 2026-10-28 (label `speed`); the restructure is not.
  - Found: the fit already reads a dense array (289 neighbours per cell, no hash lookups) and is a 3-pass
    IRLS (passes 2 and 3 down-weight outliers), so summed-area tables would reproduce only pass 1.
  - Done, bit-identical by construction and by a gtest against the serial three-pass fit
    (`test_hazard_mapping.cpp`): cells fitted in parallel (`hazard/fit_threads`), passes 2-3 skipped when
    every residual of the first plane is inside `max_plane_error_m` (`hazard/fit_early_exit`), and the
    closed-form 3×3 eigenvalue check instead of the iterative solver. Expected: 249 ms to roughly 40-80 ms
    per update and dropped clouds near zero. Once nothing is dropped the mapper runs about six batches a
    second instead of one, so its total CPU may not fall; `hazard/batch_clouds` trades map latency for CPU.
  - Not done: a rolling grid or summed-area restructure (medium risk, see the review notes). The map stays
    global; Nav2 copies it as its costmap.
- **Footprint-based traversability** (NASA ENav/ACE style):
  - judge cells by the rover's footprint (tilt, belly clearance) instead of a point robot with inflated specks;
  - the cells the rover stands on are never lethal, which ends the "Start occupied" dead ends.
  - Earlier analysis of the old mapper: only 55 % of the lethal cells were really steeper than 14°, and 17.5 % of
    the map was lethal against 7 % truly steep.
  - Check against the world's true slopes (heights in `~/togo_assets/lunar/build/apollo16_core/heights`; reader:
    `tools/gt_check.py`, `Terrain`).
- **Nav2 after this:** a rolling local costmap (8×8 m) and incremental `map_updates`. That is small today (a few ms
  per update), but it grows with map size. Measure it on a live drive.

### Step 3: DLIO memory and latency

- **Compact covariances:** 6 floats instead of a 4×4 double, about 5× smaller per point.
- **Paging:** keep nearby keyframes in RAM and old ones on disk, for flat RAM.
- **Keyframe spacing:** try 0.5-1 m instead of 0.3 m. That gives 3.3× fewer keyframes; keep it only if accuracy
  holds.
- **Incremental target map** (small_gicp or iVox), instead of rebuilding the submap tree. Expected 1.5-2× faster
  registration with no spikes.

### Step 4: accuracy (CPU-neutral; can run as separate labels at any time)

1. **PLANE vs FROBENIUS**, the user's original question. A third option ADDED 2026-10-28, label `reg_adaptive`.
   - `ADAPTIVE` (`nano_gicp.cc`): per point, a planarity-weighted blend of PLANE's thin disc and the measured
     neighbourhood shape, normalised and floored. Flat ground gets exactly PLANE, rocks and edges keep their
     measured anisotropy, nothing to tune. PLANE and FROBENIUS are untouched; the default stays PLANE.
   - Run: `scripts/benchmark_slam.sh --label reg_adaptive --compare-to speed --config
     src/lunar_world_gen/config/slam/variants/seyond_dlio_graph_lunar__reg_adaptive.yaml` (that file differs
     from the lunar config by this one key). FROBENIUS can be run the same way with its own variant file.
   - Theory says PLANE wins on flats: FROBENIUS gives near-ground points about 37× more sideways weight.
   - RESULT: ADAPTIVE lost (zig-zag +17 %, RPE 1 m +16 %, ATE max +31 %). Measured cause (`planarity.py`, 16-NN
     patches on single 0.15 m-voxel scans of the bag): flat ground is not seen as planar by a per-scan
     neighbourhood. Ground points have planarity 0.55-0.62 median, 30-42 % below 0.5, and the patch elongation
     s2/s1 falls from 0.63 at 3-6 m to 0.18-0.44 at 10-20 m because rings are farther apart than points along a
     ring. ADAPTIVE therefore kept a line-shaped covariance for a third to a half of the ground, i.e. a constraint
     across the rings up to 3× stronger than PLANE's (across-line eigenvalue 0.32-0.87 against 1.0). That
     constraint follows the ring pattern, which moves with the rover, not the terrain: lateral jitter. On the
     plain 90 % of points are ground, so this dominates. The intended effect on rocks (planarity 0.36-0.44, shape
     kept) did happen but has little to work with there. Any retry needs a planarity that does not see the scan
     pattern (submap-side or radius-based), which is a medium-risk item, not a config switch.
   - PROBE `kcorr32` (2 runs, 2026-10-28 evening, kCorrespondences 16 -> 32): WORSE. Zig-zag +13 %, RPE 1 m +9 %,
     per-scan correction +8 %, GICP time +24 %, fitness +30 %, Hessian min eigenvalue -14 %, condition +57 %.
     Offline the 32-NN ground normals were as good as the ring-window ones (tilt 1.7-1.8 deg beyond 6 m on the
     plain vs 2.5-3.1 for 16-NN), so ground-normal tilt is NOT the lever: a longer same-ring patch smears the
     rock normals and the in-plane constraint comes from the rocks. (Host was also loaded: bag played in 377 s
     instead of 365 s, so the timing rows carry that too.)
   - CODE (since removed): a `normalSource: ring` switch fitted a plane to a 3-ring x 9-column window of the
     organised scan with a slope gate and fed the PLANE disc of source and keyframe points.
   - LABEL `ringnormals` (2 runs, 2026-10-28 21:24-21:37): CLEARLY WORSE. ATE 0.181 -> 0.257 m (+42 %), end drift
     0.042 -> 0.157 m, RPE 10 m +45 %, vertical RMSE x2, zig-zag +5 % (p95 +11 %), per-scan correction +13 %,
     fitness +22 %, Hessian min eigenvalue -22 %, condition +78 %; ring normals covered 97 % of points. Speed
     unchanged. Together with kcorr32 this refutes the normal-quality hypothesis twice: ground-normal tilt does
     not drive the zig-zag, the in-plane constraint lives in the rocks, and any neighbourhood that smooths rock
     normals (32-NN, the 27-cell ring window) loosens it. The ring switch, the ADAPTIVE option and the three
     variant configs were removed on 2026-10-28 so the code carries only what measured better;
     `tools/scan_planarity.py` (the diagnostic) stays.
   - Next bets: per-direction (information-weighted) scaling of the correction, which attacks the mechanism
     (weak directions jump to a shallow minimum) without assuming anything about the motion; the lag fix; a
     config-only counter-probe with fewer neighbours (8-10). A zero-lateral-velocity prior was considered and
     dropped: it would also damp real slip in turns and on slopes.
2. **The 47 ms pose-vs-stamp lag.**
   - Cause: `updateState()` pulls the IMU-propagated *current* state toward the GICP pose of the *older* scan
     time, and `publishPose()` stamps that with the latest IMU time
     (`lidarslam_ros2/direct_lidar_inertial_odometry/src/dlio/odom.cc` lines 588 and 3047-3067).
   - The sim LiDAR stamps were proven exact.
   - Fix: correct at the scan time and re-propagate the buffered IMU to now.
   - Targets: `heading_lag_ms` → about 0, heading error and turn spikes down. It should also clean the hazard map
     in turns.
3. **Height-scale error:** the vertical drift is +3 % of the climb (the estimate exaggerates climbs and
   descents). INVESTIGATED 2026-10-28: not a scale error. On the three baseline runs the per-0.5 m vertical
   drift rate does not follow the true slope (correlation 0.06-0.09), while a fixed-tilt model explains the
   drift as well as or better than the climb model (R² 0.67-0.81 vs 0.60-0.67): on one lap the two are
   confounded because the slope sits at one place on the route, and the lap-level gain varies 2.0-4.4 %
   between runs with the drift noise. The scorer now also reports `vertical_rate_vs_slope_pct` and its
   correlation. No config change follows, so there is no `height_scale` label. A small pitch underestimate on
   slopes (1-3 % of the slope angle, 0.2-0.6° at 20°) was seen; it does not feed the height.
4. **The structural fix for the zig-zag and drift on the plains**, CPU only:
   - feature-aware regularization (planar / line / scattered neighbourhoods);
   - a range-aware noise term;
   - then an update weighted by IMU and scan uncertainty instead of DLIO's fixed gains (iG-LIO-style iterated
     Kalman filter).
   - Targets: zig-zag (57 mm RMS), cross-track error, RPE.

## Working procedure for every change

1. One change per benchmark label: a config copy, or a C++ change plus a rebuild.
2. Run `benchmark_slam.sh --label <name> ... --compare-to baseline`.
3. Keep the change only if its target metric is *better* beyond noise and no accuracy metric is *worse*.
   Otherwise revert it.
4. Add one dated line to `src/lunar_world_gen/docs/PROGRESS.md` with the result, whether it was kept, and the
   label.
5. Keep the code change itself so the user can review it; they handle git.

Suggested order: Step 1 (items 1-3), then Step 4.1 as a quick side run, then Step 2, Step 3, Step 4.2-4.4. Re-rank
after Step 1, since the CPU split will change.
