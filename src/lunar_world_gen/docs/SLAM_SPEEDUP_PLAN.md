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

## Round 2 (from 2026-10-29): accuracy switches, measured one at a time

Agreed order with the user on 2026-10-28, after round 1 closed: (1) a config-only probe with fewer GICP
neighbours, `odom/gicp/kCorrespondences: 10` (label `kcorr10`, 2 runs against `speed`); k = 8 only if 10
improves, otherwise skipped. (2) An information-weighted correction as a runtime switch. (3) The scan-time
correction (the lag fix) as a runtime switch. Both switches sit in one build, off by default, each measured
with its own one-key variant config. (4) The full filter and voxel map (Step 4.4) last; it is not switchable.
Stop rule: if (1)-(3) bring a significant improvement, stop before (4) so the user can commit that state; if
nothing improves, continue straight on.

Removed on the user's ask, 2026-10-28: DLIO's `degeneracyProjection` (the gate-shaped ancestor of (2): it
acted only above a condition number of 30000, floored at 0.1, and belonged to the gates that never worked). The
three keys are gone from the code and from every DLIO config, including `seyond_dlio_graph_husky.yaml` and
`cfg/seyond_robin_w_dlio.yaml`, where it had been `true`. rko_lio's own eigen projection is a different
package and stays.

**(2) Information-weighted correction** (`odom/gicp/informationWeighting/enabled`, `lambda0Translation`,
`lambda0Rotation`; `odom.cc`, `information_weighted_correction`). The registration's final Hessian says how
well the scan pins each direction of the pose. Expressed in the rover's own frame (`H_b = Ad^T H Ad`, so the
translation block is along / across / up and the rotation block is about the rover rather than the world
origin, which is what nano_gicp linearises), the scan's optimum is fused with the IMU prior, which sits at zero
correction: `delta' = (H_b + L0)^-1 H_b delta`, `L0 = diag(lambda0_rot I3, lambda0_trans I3)`. A direction the
scan constrains with information `lambda` keeps `lambda / (lambda + lambda0)` of its jump. Nothing is rejected
and no threshold decides, which is the difference from every gate in the file; `lambda0 = 0` is bit-identical
to the plain correction (the block returns early). The prior's information `lambda0` is in the Hessian's own
units, which are not calibrated (PLANE covariances, no noise model), so it is set from the measured
distribution: the new diagnostics (indices 58-63 of `/dlio/frontend_diagnostics`, appended) carry the
body-frame marginal translation information (smallest and largest eigenvalue of the Schur complement),
the smallest rotation one, and the smallest translation scale applied; the scorer reports them as
`scan_matching.body_trans_info_min` (p5 / p25 / p50), `body_rot_info_min`, `iw_scale_min`. Plan: one run with
the switch on and `lambda0 = 0` (statistics only), then `lambda0_trans` near the p5-p25 of the weak direction
so the plains' cross-track jumps keep roughly a third to a half and rock-constrained directions stay above
0.95, `lambda0_rot` scaled by the scan radius squared (about 100x); labels `infoweight_<value>`, 2 runs each.

**(3) Scan-time correction** (`odom/geo/correctAtScanTime`, `odom/geo/stateHistorySec`; `propagateState()`
keeps a history of the propagated state, `updateState()` reads it). Original DLIO measures the GICP pose of
the scan's stamp against the current IMU-propagated state, tens of milliseconds later, and so pulls every
published pose back toward where the rover was: the 47 -> 38 ms heading lag of the benchmark is the scan's
age at the correction. With the switch the error is taken against the state interpolated at the scan's stamp
and applied to the current state (position offset as is, rotation error carried into the current body
frame); the fixed gains, bias updates and the IMU prior for the next scan are unchanged. Diagnostics: the scan
age at the correction (`scan_age_at_correction_ms`, the quantity the lag was) and a miss counter (no history
yet). `false` is bit-identical to the original update. Label `lagfix`, 2 runs against `speed`; targets:
`heading_lag_ms` toward 0, along-track error down, no change on the plains' zig-zag.

**Results (2 runs each against `speed`, `compare__speed__vs__<label>.md` in `~/togo_assets/lunar/bags/slam_runs/`):**

- (1) `kcorr10` (2026-10-28 22:49-23:02): WORSE, not kept; k = 8 skipped by the agreed rule. ATE 0.181 -> 0.227 m
  (+26 %, inside the spread), ATE max +17 % and RPE 1 m +19 % beyond noise, zig-zag +15 % (p95 +20 %), end
  drift x2.7, per-scan correction +7 % (p95 +11 %); GICP fitness -20 % and the world-frame Hessian's smallest
  eigenvalue +9 % look "better" but the registration got jumpier: ten neighbours make each point's covariance
  a noisier estimate, and that noise, not the amount of constraint, sets the jitter. With k = 32 worse as
  well, 16 stays.

- `iwstats` (1 run, weighting on with lambda0 = 0, 2026-10-28 23:23): the no-op control. Per-scan rows identical
  to `speed` (zig-zag 0.055, RPE 1 m 0.108, heading lag 37.8 ms, correction 105 mm); the ATE rows differ as one
  run's drift does. Body-frame marginal information of the weak translation direction: p5 1432, p50 3612 over
  the lap; the plains leg (slope -> plain) p25 1711 / p50 2160, the rocky leg p50 5390, standstill 3955; the
  strong direction (up, from the ground) 285k-960k, so the ratio is about 170; the weakest rotation direction
  1.7M-10M (rad^-2 units, lever arm). Chosen lambda0_trans = 2000: the plains' weak direction keeps about half
  of its jump, the rocks' three quarters, every strong direction more than 0.99; rotation left at 0 for the
  first label (`infoweight_t2000`).
- (3) `lagfix` (2 runs, 2026-10-28 23:29-23:42): does what it was built for and is NOT kept as is. Heading lag
  38.2 -> -5.6 ms (gone), heading error RMSE 0.619 -> 0.437 deg (-29 %), scan age at correction 40 ms p50, one
  history miss per run (the first scan). But vertical RMSE 0.019 -> 0.047 m (+147 %), zig-zag +18 % (p95 +23 %),
  RPE 1 m +10 %, ATE +13 % (inside the spread). The vertical error is a scale error, not a lag: in both runs it
  follows the climb (correlation +0.88, 3.5-4.0 % of the height change) and has no relation to the vertical
  velocity (correlation 0), while the speed runs sit at 0.3-0.8 % with weak correlation; the mean vertical
  velocity error on the slope legs is 1-2 mm/s larger than in `speed` and integrates to the 30-50 mm leg means.
  A steady-state analysis of the fixed-gain observer gives the same velocity and bias in both schemes (the fix
  only removes the v*tau position lag), so the regression sits in the loop's dynamics, which brought up the
  gains: the seyond configs set no `odom/geo/*` gain and this fork's declared defaults are 1.0 for every one of
  them (DLIO upstream: Kp 4.5, Kv 11.25, Kq 4.0, Kab 2.25, Kgb 1.0, abias_max 5.0, gbias_max 0.5). With unit
  gains the position-velocity-bias error loop has the characteristic polynomial s^3 + s^2 + s + 1 =
  (s + 1)(s^2 + 1): an undamped mode at 1 rad/s; the attitude loop s^2 + s + 1 is slow and lightly damped.
  Next probe, config only: `geogains` (the upstream gains), then `lagfix` on top of it if the gains hold up.

- (2) `infoweight_t2000` (2 runs, 2026-10-28 23:46-23:59, lambda0_trans 2000, rotation 0): the first clear
  accuracy gain of round 2, KEPT as a candidate. ATE 0.181 -> 0.138 m (-23 %; runs 0.116 and 0.161), along-track
  -23 %, cross-track -24 %, heading error -22 %, RPE 10 m 1.79 -> 1.51 % (-16 %, beyond noise), ATE slope -> plain
  0.210 -> 0.116 m (-45 %, beyond noise), per-scan correction 106 -> 75 mm (-29 %) and its p95 -27 %, rotation
  correction -9 %. Unchanged: zig-zag (0.055 -> 0.053), vertical RMSE, end drift, heading lag, GICP mean time,
  CPU. Flagged worse but not accuracy: one vertical peak (max +16 mm), GICP p95/max tails (+1.7 / +21 ms), a
  backend RAM peak (+6 %, trajectory dependent). The weak direction kept a scale of 0.645 (p50) / 0.42 (p5), as
  designed. Reading: the shallow-valley jumps were feeding the drift (RPE 10 m, ATE), not the 2 s zig-zag, which
  stays at 53 mm and must come from elsewhere (the observer's undamped loop is the open suspect, see `geogains`).
  Next values: `infoweight_t4000` (stronger) and `infoweight_t2000_r1500k` (rotation term added, lambda0_rot
  1.5e6 so the plains' weakest rotation direction keeps about half).

- `geogains` (2 runs, 2026-10-28 23:59 - 2026-10-29 00:13, config only: DLIO upstream observer gains Kp 4.5,
  Kv 11.25, Kq 4.0, Kab 2.25, Kgb 1.0, abias_max 5.0, gbias_max 0.5 instead of this fork's 1.0 everywhere): NOT
  kept. Nothing improved: ATE +14 %, RPE 10 m +15 %, zig-zag +5 %, vertical RMSE +52 % with a wide spread, heading
  lag unchanged (36 ms: the lag is the scan age, not the loop), per-scan correction unchanged, CPU +2 %. So the
  undamped unit-gain loop is not what makes the 2 s zig-zag, and it does not rescue the lag fix either; the
  combination label is dropped. The zig-zag's wavelength in the detail pictures is 0.3-0.5 m (4-6 scans), far
  from the loop's 1 rad/s mode, and it survived a 29 % cut of the registration corrections: its source is still
  open (candidates: the registration's own scatter reaching the state through the 10 %/scan position gain, or
  the deskew/prior), and it needs the registration pose next to the state in the diagnostics to be told apart.

- `infoweight_t4000` (2 runs, 2026-10-29 00:13-00:26, lambda0_trans 4000): NOT chosen. Corrections -39 %, zig-zag
  0.055 -> 0.050 (-8 %, beyond noise) and RPE 10 m -13 %, but ATE only -7 % (runs 0.131 / 0.205), end drift +61 %
  and one run with a vertical excursion (0.050 m RMSE); the weak direction kept 0.475 (p50). Too much trust in
  the prior starts to cost drift.
- `infoweight_t2000_r1500k` (2 runs, 00:26-00:40, rotation term added, lambda0_rot 1.5e6): NOT chosen. Rotation
  corrections -25 % and heading lag 38 -> 33 ms, but the gain halves (ATE -12 %, RPE 10 m -5 %), ATE max +14 %
  and the height scale 0.4 -> 1.8 % come out worse: damping the weak rotation direction (yaw on the plains) lets
  attitude drift through. The scan already pins pitch and roll through the ground, so the rotation block stays
  unweighted.
- DECISION 2026-10-29: `odom/gicp/informationWeighting/enabled: true`, `lambda0Translation: 2000.0`,
  `lambda0Rotation: 0.0` become the defaults of `seyond_dlio_graph.yaml` (and the lunar copy); the variant
  files of the sweep are deleted. A 3-run label `weighted` on the new default config is the confirmation and
  the reference for everything after.

- Lag-fix diagnosis (2026-10-29, a 200 s replay with the registration pose and the observer's reference
  position added to the diagnostics, indices 64-69): with the switch on, the published state tracks the
  registration in z to a few millimetres (no correlation with the climb), while the registration itself drifts
  by 3.4 % of the climb (correlation 0.94). So the vertical scale error sits in the scan matching, which the
  observer reaches only through the prior (previous registration pose, the state's velocity and biases, the
  IMU). The reading: the fix makes the prior good enough that GICP's stop rule (`transformationEpsilon` 5 mm,
  `rotationEpsilon` 0.005) ends the alignment after the first iteration, so every scan inherits the prior's
  small vertical bias, the keyframe carries it into the map, and the map ratchets; with the lagged observer the
  prior was worse and the alignment kept iterating. Probe: `lagfix_eps` = the lag fix with both epsilons at
  0.0005 (config only, weighting off, 2 runs against `speed`).

- `lagfix_eps` (2 runs, 2026-10-29 00:48-01:01, lag fix + both epsilons 0.0005, weighting off): NOT viable, and
  half an answer. GICP time 9.8 -> 38.9 ms (p95 99 ms), DLIO CPU 116 -> 262 %, 12 % of the scans dropped, scan
  age at correction 70 ms, zig-zag +74 %, RPE 1 m +48 %. Yet the attitude became much better (roll and pitch
  error RMSE -55 / -53 %, RPE rotation -47 / -37 %, heading error -32 %, RPE 10 m vertical -36 %) and the height
  scale fell from 3.5-4 % to 1.4-1.9 %, so the early-stop ratchet is part of the story but not all of it, and
  the dropped scans confound the rest. A side finding: with full convergence the lateral jitter grows, so the
  5 mm stop rule has been acting as a crude prior-trust in the weak directions, which the explicit weighting now
  does properly. The variant is deleted. Next and last lag-fix probe: a minimum number of re-linearisations
  (`odom/gicp/minIterations`, nano_gicp `setMinimumIterations`, default 1 = today) instead of a global
  epsilon, with the alignment's iteration count added to the diagnostics (index 70) so the probe also shows how
  many iterations the default actually runs; label `lagfix_min3` against `weighted`.

- `weighted` (3 runs, 2026-10-29 01:02-01:23, the new default config, the reference label from here on):
  the gain holds on the reliable rows and is more modest on the ATE than the 2-run label said. Beyond noise:
  ATE max 0.709 -> 0.594 m (-16 %), zig-zag 0.055 -> 0.051 m (-6 %), per-scan correction -30 % (p95 -30 %),
  rotation correction -8 %, DLIO RAM peak -12 % and growth -17 %. Inside noise: RPE 10 m 1.79 -> 1.60 % (-11 %),
  heading error -13 %, RPE 1 m -4 %; vertical 0.019 -> 0.023 m; GICP mean +0.6 ms, CPU +2 %. ATE 0.181 ->
  0.181 +- 0.067: runs 0.142, 0.143 and 0.258; the third run's error is a slow heading drift that the global
  4-DoF alignment spreads over the lap (cross-track 0.33 m already at the standstill, per-scan jitter and
  vertical normal), the kind of excursion the baseline's runs also showed (0.232 +- 0.081). Over the five runs
  of this configuration the ATE median is 0.143 m against 0.181 m (-21 %) and RPE 10 m 1.51-1.60 against 1.79 %.
  Kept: the weighting changes nothing in cost and improves the per-scan behaviour consistently; the ATE
  benefit is a median effect, not a guarantee per run.

- `lagfix_min3` (2 runs, 2026-10-29 01:30-01:43, lag fix + `odom/gicp/minIterations: 3`, weighting on, against
  `weighted`): the ratchet is tamed and the result is a TRADE, left for the user. Attitude: heading error RMSE
  0.541 -> 0.311 deg (-42 %), roll -53 %, pitch -57 %, RPE 1 m rotation -55 %, RPE 10 m rotation -44 %, heading
  lag 38 -> -3 ms, RPE 10 m vertical -48 %; vertical RMSE 0.023 -> 0.020 m, height scale 1.2-2.2 % (was 3.5-4 %
  without the minimum). ATE 0.181 -> 0.151 m (-16 %, inside the spread), along -14 %, cross -19 %. Costs, beyond
  noise: zig-zag 0.051 -> 0.061 m (+20 %, p95 +21 %), RPE 1 m translation +10 %, GICP mean 10.4 -> 11.6 ms,
  DLIO CPU +8 %, RAM +14 %. Every lag-fix variant raised the 2 s lateral jitter by about a fifth, so the lag
  itself has been smoothing the published pose laterally; why is not understood. By the accuracy guard (no
  worse verdict on zig-zag) the switch stays OFF by default; the code stays, as does
  `variants/seyond_dlio_graph_lunar__lagfix_min3.yaml`, so the user can take the trade (much better attitude and
  no lag, for a fifth more lateral jitter and a tenth more DLIO CPU) with one config change.
- Iteration counts (new diagnostic, index 70): the default config stops the alignment after one iteration on
  at least 5 % of the scans (first 80 s: p5 1, p50 3, mean 4.8); with the minimum of 3 the lap shows p5 3, p50 5,
  mean 7.0, max 46 for one extra millisecond per scan. The early stops are where a prior's bias passes through.

**Round 2 outcome (2026-10-29 01:45).** Kept: the information weighting as the default
(`odom/gicp/informationWeighting/enabled: true`, `lambda0Translation: 2000`): per-scan corrections -30 %, ATE max
-16 %, zig-zag -6 % beyond noise, RPE 10 m -11 %, ATE median over five runs 0.143 against 0.181 m, no cost.
Rejected and removed: k = 10 (k = 8 skipped), the upstream observer gains, the tight epsilons, the stronger and
the rotation-weighted variants, the degeneracy projection. Built and left as switches, off: the scan-time
correction (with `minIterations` as its companion) and the six plus one appended diagnostics. Still open: the
0.3-0.5 m lateral zig-zag of about 50 mm RMS, now shown to be neither the observer's gains nor the shallow-valley
jumps; its remaining candidates are the registration's own scatter and the deskew/prior. The full filter and
voxel map (Step 4.4) are untouched; by the stop rule, the state is handed to the user for a commit first.

More results are appended as the labels finish.


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
