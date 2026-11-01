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

## Round 3 (from 2026-10-29): new ideas as switches, one label each

The user's instruction: build the untested ideas as toggles and go down the list; no voxel-size probe; anything
that fixes the lag fix's ratchet is welcome. Switches coded 2026-10-29, all off by default, each with a one-key
variant config and a 2-run label against `weighted`: (1) `odom/gicp/informationWeighting/mode: solver`, the same
prior information added to every Gauss-Newton / LM system inside nano_gicp (`setPriorInformation`), with the LM
gain ratio comparing consistent costs; (2) `informationWeighting/scaleWithCorrespondences`, lambda0 as a ratio of
the alignment's pair count (lambda0 * n / 3600); (4) `odom/gicp/refineDistance`, a second alignment stage that
continues from the first result with a 0.5 m search radius while the solver prior stays pinned at the original
guess (`setPriorPose`); (3) `odom/keyframe/refineEpsilon`, full convergence only on the scans that become
keyframes (the keyframe decision is factored into `keyframeCriterion(p, q)`, shared with `updateKeyframes()`), so
an early-stopped alignment cannot ratchet the map; (5) `odom/geo/scanTimeHorizontalOnly`, the scan-time
correction with z referenced to the current state. Queued, not coded: (6) an iterated Kalman update in place
of the observer.

- `lagfix_min3_gc` (2 runs, 2026-10-29 02:17-02:30, the lag fix with the existing ground constraint on): WORSE on
  everything that matters and removed. The constraint engaged on about 2250 of 7000 scans; ATE 0.181 -> 0.329 m
  (+82 %), vertical RMSE 0.023 -> 0.052 m, RPE 1 m +21 %, RPE 10 m +35 %, heading error +41 %. Its design explains
  it: it anchors the ground height under the rover to one fixed value with no tracking and gates on the ground
  normal's tilt in the world frame, so on this lap it is off on the slopes and wrong wherever the plain is not at
  the start's height. Not a candidate for the ratchet.

- (1) `iw_solver` (2 runs, 2026-10-29 10:36-10:50, weighting inside the solver, lambda0 2000): KEPT, now the
  default (`informationWeighting/mode: solver`). Against `weighted`: ATE 0.181 -> 0.141 m (-22 %, spread 0.004),
  per-scan correction -17 % (p95 -14 %), pitch error -14 %, RPE 1 m rotation -8 %, heading lag 38 -> 34 ms;
  zig-zag, vertical, RPE 1 m / 10 m translation unchanged. Cost went the right way: GICP 10.4 -> 6.9 ms (-34 %,
  p95 -41 %), DLIO CPU 118 -> 96 % (-18 %), all SLAM CPU -8 %: the prior term conditions the system, so the LM
  loop rejects fewer steps. One diagnostic caveat: in solver mode the reported Hessian included the prior term
  (hence the inflated "information +138 %" row); fixed the same day so the diagnostics report the scan's
  information alone (`H_gicp` in both step functions).
- (2) `iw_perpoint` (2 runs, 10:50-11:03, lambda0 scaled by the pair count): NEUTRAL, not adopted. ATE 0.180,
  zig-zag and RPE unchanged, vertical -22 % and GICP -11 % inside noise; the factor n / 3600 sits near 1 on this
  bag (about 3570 pairs per scan), so the switch changes little here. Code to be removed at the round's cleanup.
- (4) `refine05` (2 runs, 11:03-11:17, second stage at 0.5 m): WORSE, removed. Zig-zag +15 %, GICP 10.4 -> 18.6 ms,
  DLIO CPU +44 %, attitude errors +14-19 %, overlap 0.98 -> 0.94: at 10-20 m the scan's rings sit more than
  0.5 m apart on the ground, so the tighter radius drops the ground pairs, information falls, not noise.

- (3) `kfrefine` (2 runs, 2026-10-29 11:22-11:35, full convergence on keyframe scans, on the solver-mode
  default): NOT kept on its own. 780 of 6980 scans refined (11 %); zig-zag p95 -6 % is the only row beyond noise
  in its favour; ATE 0.141 -> 0.171 m (+21 %, inside the spread), heading lag 34 -> 41 ms (+19 %, the refinement
  adds latency on those scans), GICP 6.9 -> 9.1 ms (+33 %), DLIO CPU +14 %. (The information rows of this
  comparison are not comparable: the `iw_solver` runs still reported the prior-inflated Hessian.) Its target
  was the lag fix's ratchet, which the minimum-iterations guard already tames, so it is removed unless the
  horizontal-only lag-fix run shows the vertical still needs it.
- Incident 2026-10-29: the machine suspended at about 11:40, right after the `lagfix_min3_h` run started, and
  resumed at 19:57; the run (an eight-hour gap inside a 364 s replay) was stopped by PID and deleted, the
  replay's interrupt path left the SLAM nodes alive in the container (stopped by PID too), and the 3-run
  reference `solver` plus `lagfix_min3_h` were relaunched with a keep-awake process holding the machine.

- `solver` (3 runs, 2026-10-29 20:05-20:25, the new default: weighting in the solver, lambda0 2000), against the
  round-1 `speed` label, the REFERENCE from here on. Beyond noise: RPE 1 m 0.109 -> 0.103 m (-6 %), RPE 10 m
  1.79 -> 1.52 % (-15 %), zig-zag 0.055 -> 0.050 m (-9 %, p95 -9 %), per-scan correction 106 -> 62 mm (-41 %),
  GICP 9.8 -> 7.5 ms (-23 %, p95 -29 %), DLIO CPU 116 -> 101 % (-13 %), all SLAM CPU -5 %. Inside noise: ATE
  0.181 -> 0.168 m (runs 0.181 / 0.175 / 0.149), heading error -21 %, heading lag -12 %, vertical 0.020 m.
  Flagged worse: one vertical peak (+11 mm) and fitness +1 %. The zig-zag moved for the first time: the prior
  inside every iteration damps the weak-direction scatter at its source, where the one-step fusion could not.
- (5) `lagfix_min3_h` (2 runs, 20:25-20:38, scan-time correction with z referenced to the current state, plus
  the 3-iteration minimum, on the solver-mode base, against `iw_solver`): the first lag-fix variant to pass the
  accuracy guard. Heading lag 34 -> -5 ms, heading error 0.528 -> 0.297 deg (-44 %), roll -52 %, pitch -46 %,
  RPE 1 m rotation -52 %, RPE 10 m -5 % (better), ATE max -18 %, ATE 0.141 -> 0.138, vertical equal, zig-zag
  0.050 -> 0.054 (+8 %, inside noise). Costs: RPE 1 m translation +8 %, the start -> rocks leg +24 mm, GICP
  6.9 -> 8.4 ms (the minimum iterations), DLIO CPU 96 -> 108 %. (The information rows of this comparison are
  the prior-inflated ones of the `iw_solver` runs, not comparable.) Whether the horizontal-only hedge is what
  makes it pass is measured next with the full scan-time correction on the same base.

- `lagfix_min3` on the solver base (2 runs, 2026-10-29 20:39-20:53, the full scan-time correction; the morning's
  two runs on the final-mode base were relabelled `lagfix_min3_finalbase` so the label is clean): the same
  attitude gains as the horizontal-only variant (heading -42 %, roll -43 %, pitch -52 %, lag -5 ms) but it
  fails the guard: zig-zag 0.050 -> 0.057 m (+14 %), RPE 1 m +15 %. Side by side with `lagfix_min3_h` on the
  same base: RPE 1 m 6 % worse, zig-zag +6 %, everything else equal, so keeping z on the current-state reference
  is what makes the lag fix pass.
- DECISION 2026-10-29 21:00: `odom/geo/correctAtScanTime: true`, `odom/geo/scanTimeHorizontalOnly: true` and
  `odom/gicp/minIterations: 3` become the defaults, pending a 3-run confirmation (`round3`) against `solver`;
  if the confirmation fails the guard, they go back to off and stay as switches. Removed from the code and the
  configs: the pair-count scaling, the 0.5 m second stage, the keyframe-time refinement and nano_gicp's prior
  pin (`setPriorPose`); `keyframeCriterion(p, q)` stays as a plain factoring of `updateKeyframes()`. No variant
  configs remain: every kept switch is a key in `seyond_dlio_graph.yaml`.

- `round3` (3 runs, 2026-10-29 21:00-21:20, the solver-mode default plus the horizontal-only scan-time correction
  and the 3-iteration minimum), against `solver`: the trade holds its shape with three runs and the guard now
  bites. Better beyond noise: roll error 0.146 -> 0.092 deg (-37 %), heading lag 34 -> -6 ms, RPE 1 m rotation
  -50 %, RPE 10 m rotation -34 %. Inside noise: ATE 0.168 -> 0.146 m (-13 %; runs 0.150 / 0.133 / 0.156),
  along-track -18 %, heading error -15 %, pitch -32 %, vertical -6 %, RPE 10 m -2 %. Worse beyond noise: zig-zag
  0.050 -> 0.054 m (+9 %, p95 +10 %), RPE 1 m translation 0.103 -> 0.115 m (+11 %), GICP 7.5 -> 8.7 ms, DLIO CPU
  101 -> 109 %. Against the round-1 `speed` label the same configuration is equal or better on every guarded
  row (zig-zag -1 %, RPE 1 m +5 % inside noise, RPE 10 m -16 %, ATE -19 %, attitude -37 to -53 %, GICP -12 %).

**Round 3 outcome (2026-10-29 21:30).** Kept as the default: the weighting inside the solver
(`informationWeighting/mode: solver`), which is the round-3 reference `solver`: against `speed`, RPE 1 m -6 %,
RPE 10 m -15 %, zig-zag -9 %, corrections -41 %, GICP -23 %, DLIO CPU -13 %, all beyond noise. Built, measured and
left as one config change, OFF by the accuracy guard ("unless the user accepts the trade"): the scan-time
correction with z on the current-state reference plus `minIterations: 3`, in
`config/slam/variants/seyond_dlio_graph_lunar__lagfix.yaml` (three keys). It removes the lag and halves the
attitude errors for +9 % zig-zag and +11 % RPE 1 m against the reference. Rejected and removed: the pair-count
scaling, the 0.5 m second stage, the keyframe-time refinement, the ground constraint on this terrain. Not
started: the iterated Kalman update (switch 6); it is now the only item left on the list, and both measured
mechanisms it would unify (prior-trust, scan-time attitude) are in hand. The 0.3-0.5 m lateral zig-zag is down
to about 50 mm from 55 and remains the floor set by the plains' weak-direction scatter.

## Round 4 (from 2026-10-29 evening): the error-state Kalman update as a switch

The user's decision after round 3: build the filter, as a toggle. `odom/geo/estimator: observer | iekf`
(`include/dlio/iekf.h`, hooks in `odom.cc`): the nominal state stays DLIO's own; a 15x15 error-state covariance
[attitude (body) | position | velocity | gyro bias | accel bias] is propagated on every IMU sample from the noise
densities under `odom/iekf/noise/*` (Sola's ES-EKF). Per scan, the covariance's pose block, inverted and divided by
`odom/iekf/hessianScale` (the one calibration number: information per unit of the GICP Hessian), is handed to the
solver as its prior in place of the constant lambda0, so the registration converges on the properly weighted
problem and its result is the posterior mean of the pose. The filter then conditions velocity and biases on that
pose through their cross-covariances and applies the pose shift to the current state the way the scan-time
correction does (attitude carried over the scan age, position as is), which also removes the lag. With the
switch off every line of the observer path is unchanged. The interface is generic on purpose: a second
measurement source (stereo visual odometry later) is another conditioning step on the same covariance.

- First 80 s check (scale 20, 2026-10-29 22:25): zig-zag 25.6 mm (the default's check: 26.5, the trade's: 31.5),
  heading lag -7 ms, ATE 0.056 m, RPE 10 m 0.90 %, per-scan correction 60 -> 24 mm and GICP 7.5 -> 3.2 ms because
  the solver starts from a strongly weighted prediction. The filter's own 1-sigma: about 2 mm position, 0.01 deg
  attitude, 10 mm/s velocity; the simulated IMU is near-ideal (calibrated biases about 1e-8), so the noise
  densities act as model-error knobs here and must come from the datasheet on the real rover.
- Labels `iekf` (scale 20), `iekf_s5`, `iekf_s80` (2 runs each, 2026-10-29 22:27-23:22): DIVERGED. Five of the six
  lap runs ran away to km-scale ATE (one stayed bounded at 0.49 m). Each was healthy to about 200 s (the start of
  the slope), then the filter's 1-sigma exploded (position 2 mm to metres, velocity growing linearly) while the
  per-scan shifts stayed at millimetres: the pose dragged the map along with it. Cause: overconfidence. The scan's
  rotation information, multiplied by the same scale, claimed the pitch to 0.006 deg (a ground plane supports
  about 0.06), so the prior never let the registration correct the real 0.1-0.2 deg attitude error; gravity
  leaked into the velocity on the slope and the prior pulled the map after it. Run folders deleted.
- Fix (build 2026-10-29 23:40): a scan floor (the scan's covariance is (s H + eps)^-1 plus diag(0.1 deg, 1 cm)^2
  before it is turned back into information), prior floors (0.05 deg, 5 mm: `poseInformation(floor)` and
  `floorPose()` after each conditioning), the normalised innovation squared (NIS) of every shift in the throttled
  log line (chi-square with 6 degrees of freedom: about 6 when the filter is consistent; the logged mean is
  cumulative), and a runaway guard (covariance reset when the position 1-sigma passes 1 m or a shift 0.5 m).
  Keys: `odom/iekf/priorFloor/{attitudeStd,positionStd}`, `scanFloor/{...}`, `reset/{positionStd,shift}`.
- The key measurement (`shift_stats.py` over diagnostics 64-69, 2026-10-29 23:55): in observer mode too, the
  registration lands 48-61 mm (p50; p90 130-160 mm) from the prediction horizontally on every 50 ms scan and
  5 mm vertically. That is the registration's horizontal scatter, the zig-zag seen directly, and it means the
  GICP Hessian overstates the lateral information about 100x (PLANE covariances with epsilon 1e-3 inflate the
  Mahalanobis weights). The scale therefore has to be about 0.1-2, not 5-80.
- 250 s checks (one run each, covering the slope; 2026-10-29 23:45 - 2026-10-30 00:00). Columns: ATE, zig-zag,
  heading lag, end drift and yaw, RPE 10 m, NIS cumulative mean (second-half mean in brackets):
  - scale 0.15: 0.810 m, 19.6 mm, -18 ms, 2.04 m and +3.5 deg, 3.16 %, 0.7. Stable and the lowest zig-zag seen,
    but the IMU is over-trusted: the registration is dragged along (shifts 3-12 mm) and the trajectory drifts.
  - scale 0.5: 0.240 m, 22.6 mm, -8 ms, 0.185 m and +1.3 deg, 2.31 %, 1.4 (2.4).
  - scale 2: 0.231 m, 37.7 mm, -5 ms, 0.266 m and -1.0 deg, 1.48 %, 5.1 (12).
  - scale 5: diverged at the slope (NIS 12 before it, 382 guard resets after).
  Reading: the consistent scale differs by leg, about 2 on the first half and about 1 on the slope and rocks.
  GICP time and the per-scan correction grow with the scale (4.9 to 7.2 ms, 31 to 46 mm).
- The `solver` default over the same 250 s (one run, 00:39): ATE 0.103 m, zig-zag 41.9 mm, heading lag +41 ms,
  end drift 0.027 m and +0.2 deg, RPE 10 m 1.32 %, GICP 8.3 ms, correction 56 mm. So every filter setting drifts
  5 to 17 times more in heading before the rover revisits anything, while 0.15 and 0.5 cut the zig-zag in half.
- Full-lap labels (2 runs each against the 3-run `solver`, 2026-10-30 00:00-00:39):
  - `iekf_s2` (scale 2): ATE 0.199 m (runs 0.142 / 0.255, within noise), RPE 1 m +10 % (worse), RPE 10 m
    1.82 % (+19 %, within noise: 1.63 / 2.00), zig-zag 51 mm (same), heading lag -7 ms (better), RPE 1 m and
    10 m rotation -26 / -10 %, per-scan correction -21 %, GICP and CPU the same; 3 and 1 guard resets.
  - `iekf_s05` (scale 0.5): zig-zag 29 mm (-41 %, p95 -42 %), heading lag -12 ms, GICP -19 %, DLIO CPU -6 %,
    correction -38 % (all better); but ATE 0.301 m (+79 %), RPE 10 m 2.28 % (+50 %), vertical RMSE +63 %, end
    drift +112 % and heading error RMSE 1.27 deg (+160 %), every leg's ATE worse; 0 resets.
  - `iekf_s1` (scale 1): zig-zag 38 mm (-23 %), heading lag -9 ms, RPE 1 m rotation -31 %, RPE 10 m rotation
    -22 %, GICP -13 %, correction -31 %, DLIO CPU -3 % (all better); ATE 0.174 m and RPE 1 m 0.100 m (same);
    RPE 10 m 1.78 % (+17 %, worse), end drift 0.165 m (+344 %, worse), heading error RMSE +18 %; 0 resets. The
    best setting, and it fails the guard on exactly one thing: slow drift.
- The drift is a heading wander (00:45-01:07). The filter's relative rotation over 10 m is better than the
  default's, but its absolute heading wanders by about 0.5 deg, and 0.5 deg over 10 m is 9 cm of lateral error:
  the whole RPE 10 m increase, and the end drift as well. Two explanations were tested with config-only 250 s
  checks: (1) a larger scan floor (2 and 3 cm at scale 2, 3 cm at scale 5) made it worse (22 / 296 / 982 guard
  resets): the floor only enters the filter's bookkeeping, so it inflates the posterior, loosens the next prior,
  and the solver trusts the scan more, the opposite of its intent; (2) the gyro bias frozen (`noise/gyroBias` 0,
  `init/gyroBiasStd` 1e-9) at scales 0.5 and 0.15: the same drift (yaw +1.45 / +3.33 deg against +1.27 / +3.48),
  so the filter is not learning a wrong bias. What remains is structural: the keyframes are placed at the smoothed
  pose, so their errors are correlated over the smoothing time (about 1/K scans) instead of independent, the
  submap cannot average them out, and the map's heading random-walks with the filter following it; the drift
  grows with the smoothing (scale 0.15 worst), which matches. With the observer each keyframe's error is the
  scan's own, independent, and a registration against many keyframes averages it away.
- A note on the logged NIS: it is computed on the posterior shift, and for that statistic a consistent filter
  gives about 0.4 with these settings (the sum of the squared gains), not 6. Read that way, scale 0.5 is the
  consistent setting on the quiet legs and the rocks-to-slope leg is 10 times over at every scale. The next build
  switches the statistic to the raw-innovation form, for which 6 is the healthy value.

**Round 4 outcome (2026-10-30 03:15).** The OUTPUT filter is the new default: `odom/geo/estimator:
iekf_output` and `odom/iekf/hessianScale: 0.5` in `seyond_dlio_graph.yaml` (lunar copy regenerated; the code's
own default for the estimator stays `observer`, so the husky, robin_w and live_real_flat configs are untouched;
the code default of the scale is 0.5). Against the round-3 default over three runs: zig-zag -32 %, heading lag
gone, heading error -39 %, roll and pitch errors -58 / -64 %, RPE rotation -56 % at 1 m and -47 % at 10 m, RPE
10 m -14 % and ATE -15 % within noise; GICP -10 % and DLIO CPU -5 % as measured but unexplained by the mode
(machine state). Against the 27 October baseline: zig-zag 57 ->
34 mm, heading error 0.77 -> 0.30 deg, heading lag 47 -> -3 ms, RPE 10 m 2.04 -> 1.30 %, ATE 0.232 -> 0.143 m,
GICP 18.8 -> 6.7 ms, DLIO CPU 145 -> 96 %, all SLAM CPU 490 -> 334 %. What the real rover needs before the switch
is trusted there: the IMU noise densities (`odom/iekf/noise/*`) from the datasheet and a re-check of the scale on
real data, since 0.5 was calibrated on the simulated sensor. Removed: the scratch variants of the sweep (scales
20, 5, 2, 0.5, 0.15, 0.05 of the feedback mode, the floor and frozen-bias diagnostics, the observer-gain
diagnostic, the output mode's scale-2 and 0.5 copies); kept `__iekf_s1.yaml` (the best feedback setting, for the
record), `__iekf_out_s015.yaml`, `__iekf_out_s03.yaml`, `__lagfix.yaml`. The feedback mode itself stays as a
switch (`estimator: iekf`): it fails the guard on this bag and is documented above. The earlier outcome note
(01:15) follows for the record.

**Round 4 outcome, first version (2026-10-30 01:15).** The feedback filter stays OFF by default (`odom/geo/estimator:
observer`, the observer path bit-identical); the switch, the variants (`__iekf_s05/_s1/_s2.yaml`) and the
finding stay for the real rover, where the IMU model and the bias states matter. What it proved: the GICP
Hessian overstates the lateral information about 100 times (scale 0.15-0.5 is the honest range), the rocks leg
by another factor 5, and any smoothing that feeds back into the keyframes turns into a heading random walk.
Next: an output-only mode. Registration, keyframes and map exactly as in the default; the filter fuses the
registration poses (as measurements, covariance (sH)^-1 plus floors, at the scan time) with the IMU only for the
published odometry and pose, with no feedback into the map. Expected: the default's drift with the filter's
zig-zag and lag figures. Deleted: the diverged check runs (`quick_check_iekf250_s5`, `_s2_f3`, `_s5_f3`) and
the comparison files of the diverged scale-20/5/80 labels whose run folders were already gone.

### Output-only mode (`odom/geo/estimator: iekf_output`, built 2026-10-30 01:23)

The observer, the registration, the keyframes and the map run exactly as in the default; a second nominal state
(`fstate_`, with its own covariance from the same `ErrorStateFilter`) is propagated with every IMU sample and, at
every scan, updated with the registration pose as a plain Kalman measurement at the scan time (`updatePose()`,
Joseph form; covariance (sH)^-1 plus the scan floor; the filter's own history gives the reference at the scan
stamp and the entries after it carry the correction; a runaway guard falls back to the observer's state). Only
the published odometry, pose, path and TF come from it (`publishedState()`); nothing feeds back into the
registration or the map. Diagnostics 73 (filter-to-observer offset) and 74 (NIS of the update, chi-square with
6 dof, about 6 when consistent); scorer fields `iekf_offset_mm`, `iekf_nis`; log line `iekf_output:`.

- 250 s checks (one run each, 01:23-01:38) against the default's 250 s reference (ATE 0.103 m, yaw error RMSE
  0.50 deg, heading lag +41 ms, zig-zag 41.9 mm, RPE 1 m 0.081 m / 0.45 deg, RPE 10 m 1.32 % / 0.53 deg,
  vertical 0.020 m), in that order: scale 0.5: 0.113 / 0.58 / -2 / 25.4 / 0.077 / 0.19 / 1.15 / 0.29 / 0.023;
  scale 0.15: 0.176 / 0.85 / -3 / 21.0 / 0.086 / 0.20 / 1.46 / 0.37 / 0.031; scale 2: 0.132 / 0.23 / -1 / 33.3 /
  0.080 / 0.20 / 1.12 / 0.25 / 0.013. Mechanically right (no resets, lag gone, zig-zag halved), but the heading
  still wandered at the small scales (end yaw drift +1.2 / +1.7 deg) although the map was the default's: that
  part of the drift was never the map. Per-leg NIS: about 7 on the quiet legs and 22 on rocks-to-slope at scale
  0.5 (3.4-4 and 11 at 0.15, 12-13 and 46 at 2), the same leg dependence as in the feedback mode.
- The cause (01:40): DLIO's observer updates its gyro and accelerometer biases with gain 1.0 per scan
  (`odom/geo/Kgb`, `Kab`), so its bias estimates random-walk with the registration noise (of the order of
  0.1 deg/s and 0.05 m/s^2). Harmless for the observer, which resets its attitude and velocity to the scan every
  50 ms, but the IMU samples (`imu_meas`) carry that bias, and a filter that trusts the gyro for seconds
  integrates the wander. Confirmed two ways: the observer's bias gains zeroed in the config
  (`__iekf_out_s05_kgb0.yaml`, 01:45-01:50): yaw error RMSE 0.58 -> 0.27 deg; and the fix in the code (01:51,
  build 18): the output filter takes the raw sample (`imu_meas` plus the observer's bias) less its own bias
  states, started from the calibrated values at the first registration.
- 250 s checks with the fix (01:52-02:01), same order: scale 0.5: 0.144 / 0.31 / -5 / 24.1 / 0.072 / 0.18 / 1.16
  / 0.24 / 0.011; scale 0.15: 0.111 / 0.20 / -5 / 22.1 / 0.078 / 0.19 / 1.08 / 0.23 / 0.013. Against the default:
  zig-zag -45 %, RPE 1 m -5 to -11 %, RPE 1 m rotation -59 %, RPE 10 m -12 to -18 %, RPE 10 m rotation -55 %,
  yaw error -38 to -60 %, vertical -40 %, lag gone; ATE within the single-run noise; the start-anchored end
  drift 0.17-0.22 m against 0.03 (the filter sits 5-15 cm behind the registration while the rover accelerates
  at the end of the window, NIS 10-30 there; to be judged on the full lap). Per-leg NIS at 0.15: 3.6-3.9 on the
  quiet legs, 9.1 on rocks-to-slope; at 0.5: 6-7 and 19: the honest scale is about 0.3 on the quiet legs.
- Labels (2 runs each at 02:03-02:43, a third run each at 02:44-03:04, all against the 3-run `solver`): every
  scale passes the guard with no *worse* verdict anywhere. With three runs, scale 0.5 / 0.3 / 0.15: ATE 0.143 /
  0.146 / 0.167 m (-15 / -13 / -1 %, within noise), heading error RMSE 0.30 / 0.23 / 0.34 deg (-39 / -53 / -30 %),
  roll and pitch error RMSE about 0.06 deg (-52 to -64 %, better), heading lag -2.8 ms everywhere (gone, better),
  RPE 1 m translation -4 to -6 % (within noise), RPE 1 m rotation 0.18-0.19 deg (-56 to -59 %, better), RPE 10 m
  translation 1.30 / 1.36 / 1.38 % (-14 / -10 / -9 %, within noise), RPE 10 m rotation -47 to -51 % (better),
  zig-zag RMS 34 / 34 / 27 mm (-32 / -32 / -45 %, better; p95 -32 to -44 %), end drift 0.017 / 0.032 / 0.019 m
  (-54 / -15 / -49 %), vertical RMSE -12 to -21 %, GICP time -10 to -12 %, DLIO CPU -5 %, all SLAM CPU -4 %, hazard
  update -6 % (all reported better, but this mode leaves the registration, the map and the backend untouched, so
  the speed numbers are machine state of the night runs, not a gain of the filter). Runaway resets per lap (the published pose snapped to the observer's at the return-to-start jump,
  where the registration moves more than 0.5 m in one scan): 0-1 at 0.5, 2-4 at 0.3, 0-2 at 0.15.
- Chosen: scale 0.5 (best ATE, RPE 10 m, roll/pitch and end drift, the fewest resets, and the safer side on a
  real IMU; 0.15 gives the lowest jitter, 0.3 the lowest heading error; variants `__iekf_out_s015.yaml` and
  `__iekf_out_s03.yaml` stay).


## Round 5 (from 2026-10-31): stereo visual-inertial odometry as a second measurement

The user's plan (pasted 2026-10-31) in five phases: a simulated stereo rig, stock OpenVINS scored on the lap, the
VIO as measurements in the output filter (`vio_out`), the VIO's per-scan delta in the registration's prior
(`vio_prior`), a VIO edge and a re-seed path in the backend (`vio_edge`), then hardware. Decisions: every area of
the tree may be edited for it (rover model, lidarslam_ros2, Docker image, this package); OpenVINS runs as a
separate process (GPL-3, never linked; University of Delaware, USA. Nothing Chinese-origin may be used:
VINS-Fusion is out); the work runs unattended through phase 4; DLIO keeps `imu_0`, the rig has its own IMU.

**Phase 0, the rig (2026-10-31).** `togo_description/urdf/stereo_vio_rig.urdf.xacro`: two identical pinhole
cameras (mono8, 640x480, 90 deg horizontal field of view, so f = 320 px, 20 fps, a little pixel noise) on one bar
0.20 m apart, and a 400 Hz IMU at the bar's centre with MEMS-grade noise (densities: gyro 1.7e-4, accel 2.0e-3;
bias walks 1.9e-5, 3.0e-3; per-sample sigmas computed in the xacro). Mounted in `togo_macro.xacro` on
`base_link` at (0.45, 0, 0.50) m, level, looking forward: the sensor arch's camera mount sits 0.33 m behind the
base origin and the enclosure would fill the bottom of the image. Extrinsics from the assembled model
(`tools/urdf_frame_offset.py`, new): cameras in the rig IMU at (0, +-0.10, 0) with R_CtoI = [0 0 1; -1 0 0;
0 -1 0]. Gazebo topics bridged by `togo_gz/config/bridge.yaml` (a plain `camera` sensor publishes its image on
the sensor topic itself, the camera info under `/camera_info`), ROS names `/husky/sensors/stereo_rig/{left,
right}/{image_raw,camera_info}` and `/husky/sensors/stereo_rig/imu/data_raw`; the recorder's topic list carries
them. Note for the host simulation: `$(find togo_description)` resolves to the native workspace's symlink
install of 2026-09-28, so a new file needs its symlink there (done for the rig; `bridge.yaml` is a symlink).
Gate (`record_bag.sh --check-sensors`, `tools/bag_record/check_stereo_sync.py`): both cameras at 20.0 Hz by
their stamps, 106 of 106 pairs with identical stamps, 640x480 mono8 in the optical frames, the rig IMU at
500 Hz by stamps; the simulation's real-time factor drops to about 0.4 with the two cameras rendering (wall
time only; bag time is unaffected). The images are dark (regolith at about 17 of 255 with the world's sun at
intensity 1: the renderer applies no exposure), so recordings for the VIO use `--sun-intensity 6`, a copy of
the world with a brighter sun (`tools/bag_record/world_sun.py`), the camera-exposure stand-in; LiDAR, IMUs and
physics do not see the light. `--rtf` scales the recorder's wall-clock budgets.

**Phase 1, OpenVINS (2026-10-31).** Built in the SLAM container at `~/slam_ws/src/open_vins` (not a mounted
folder; `scripts/setup_openvins.sh` reproduces clone, Jazzy header patch and build; `libceres-dev` added to the
Dockerfile), commit 69488123 of 2025-11-30; packages ov_core, ov_init, ov_msckf; the node
`run_subscribe_msckf <estimator_config.yaml>`. Configs in `config/vio/` (estimator: gravity 1.62, stereo,
calibration off, zero-velocity update on, a low drive-off threshold for the static initialisation, FAST 15,
histogram equalisation; IMU chain with the rig's densities; camera chain with the rig's extrinsics and
intrinsics). Plumbing: `seyond_dlio_slam.launch.py` arguments `vio` and `vio_config` (an `ov_msckf` node next
to DLIO), `run_live_seyond_dlio_slam.sh` env `SLAM_VIO` / `SLAM_VIO_CONFIG`, `replay_slam_bag.sh --vio
[--vio-config F]` (records `/ov_msckf/odomimu` and the feature clouds, `run_info vio=`), `benchmark_slam.sh`
passes it through, and `slam_eval.py` scores the VIO as a `base_link` path with the same scorer (per leg too),
reports feature counts per frame and leg, writes `vio_odometry.tum` and a `vio` block in `metrics.json`.

- The bag (2026-10-31 17:25-17:51): `apollo16_stereo_20261031`, the benchmark route twice with the rig, 269 m in
  675 s of simulation time (24 min of wall time at a real-time factor of 0.46), 5.6 GB; the cameras at 20 Hz, the
  rig IMU at 500 Hz (Gazebo's IMU system samples at its own step), LiDAR and `imu_0` as before. Lap 1 trimmed with
  `trim_bag.py --laps 1 --segments` into `apollo16_stereo_lap1_20261031` (351 s, the usual five legs). Missing
  from it: the rig's camera info, because Gazebo derives a camera's info topic from the parent of its image
  topic and both cameras shared one; fixed in the rig's xacro (image topics `.../left/image`) and the bridge for
  the next recording. The VIO does not need it (intrinsics come from its yaml).
- First OpenVINS replay (`vio_check1`, 17:52-18:05): the node died at start. OpenVINS reads its yaml with
  OpenCV's FileStorage, which trips over a second colon on a line, including one inside a comment; the
  comments of `config/vio/*.yaml` carry none now, and `use_mask: false` was added. Checked on the first minute
  of the bag: static initialisation during the standstill, then tracking at about 150 Hz of processing rate.
  The SLAM part of that replay ran with the default config (ATE 0.208 m, zig-zag 61 mm on the new bag).

- Reference on the new bag (`ref_stereo`, 3 runs, 18:05-18:46, the default config): ATE 0.133 m, zig-zag 54 mm,
  RPE 10 m 1.39 %, heading error 0.32 deg. A `vio_out` label (3 runs, 18:46-19:27) turned out to have run WITHOUT
  OpenVINS: `benchmark_slam.sh` initialised its argument list after parsing `--vio` and dropped it (fixed). It is
  therefore a second sample of the same config, a useful noise figure for 3-run labels on this bag: ATE 0.133
  against 0.148, zig-zag 54 against 60 mm, heading error 0.32 against 0.36 deg, all "within noise" by the scorer.
- The recording itself was flawed (19:40): under the image load the recorder stored messages up to 250 ms out of
  order (`tools/bag_record/clock_order.py`: 243 backward steps of /clock, 4 of the LiDAR, 121 of the rig IMU; the
  27 October bag has none), and their log times are the recording's wall clock at a real-time factor of 0.46.
  Played back, /clock stepped backwards 170 times per run (tf2 cleared its buffer in the backend each time), DLIO
  dropped a scan as non-increasing, and every replay crawled at the recording's pace (13 min per run). New tool
  `tools/retime_bag.py`: every message's log time becomes its own stamp (simulation time) and the messages are
  written in that order; `apollo16_stereo_lap1_20261031_rt` is the bag to use (0 backward steps, 347 s). The two
  labels above are superseded by `ref_rt` and `vio_out_rt` on it.

- On the retimed bag (`ref_rt`, 3 runs, 19:35-19:55, 6.5 min per run again): ATE 0.145 m, zig-zag 34 mm, RPE 10 m
  1.40 %, heading error 0.47 deg, the same figures as the 27 October bag gives the default. The first `vio_out_rt`
  label (19:55-20:15) had OpenVINS running through the whole lap (static initialisation in the standstill,
  zero-velocity updates, state prints) but nothing reached DLIO, the recorder or the backend: ROS 2 topic names
  are namespace-relative, and the launch gave the node a name but no namespace, so it published `/odomimu`
  while everything listens on `/ov_msckf/odomimu`. One launch line (namespace `ov_msckf`) fixes it. That label
  did show the cost of the extra process: all SLAM CPU 331 to 432 % and GICP solve time 6.5 to 9.1 ms (+39 %)
  from contention, with the accuracy within noise (no VIO data in the filter). Re-run as `vio_out_rt2`.

- `vio_out_rt2` (20:17, first run read): the VIO reached the filter at last, and OpenVINS itself had diverged
  (ATE 11 km, speed 130 m/s, zero MSCKF features over the whole lap). Diagnosis (`vio_diag.py`): the rig's own IMU
  in this bag reads a specific force of 1.36 m/s^2 at standstill instead of 1.62, with 0.34 m/s^2 on x: Gazebo's
  dynamic-bias noise model gave it a bias no MEMS unit has, OpenVINS's static initialisation turned that bias
  into a tilted gravity, the IMU prediction went wrong, and every feature update failed its chi-square test. The
  images are fine throughout (mean 31-36 of 255). Fix without re-recording: OpenVINS runs on `imu_0` (ideal in
  the simulation, 100 Hz, synchronised), with the cameras' extrinsics relative to it (p_CinI = (0.391, +-0.10,
  0.3387)); every "VIO IMU in base_link" default moved to imu_0's place (DLIO `extrinsics/baselink2vio`, the
  backend's `vio_rig_in_base_xyz`, the scorer's `--vio-imu-in-base`); the rig's xacro drops the dynamic bias for
  the next recording. A quick replay of the first 85 s: clean initialisation (zero bias, a pure yaw), the
  estimate moves. The filter's adapter also accepts a VIO message a few milliseconds ahead of its latest IMU
  sample instead of skipping it (half of the rig-rate messages were skipped before). Labels `vio_out_rt3` and
  `vio_edge` (Phase 4, below) follow in one chain after a rebuild.
- Phase 4 code (20:10, built with the above): a second between-constraint on adjacent submaps in
  `graph_based_slam` from the VIO's relative base_link pose between the two submaps' stamps (`use_vio_edge`,
  `vio_edge_info_weight_trans/rot`, its own NIS auto-scale, `vio_input` remapped to `/ov_msckf/odomimu`), and
  DLIO's `odom/gicp/recovery/reseedFromFilter`: a rejected registration restarts from the output filter's pose
  (IMU + VIO) instead of the IMU prior. Variant `__vio_edge.yaml` = VIO in the filter + the edge + the re-seed.
  The simulation never rejects a registration on this bag, so the re-seed is a no-regression check here.

- `vio_out_rt3` / the imu_0 run (21:00-21:30): OpenVINS still blew its scale up on imu_0 (18 m/s mean speed, 45 m
  from the start in 65 s against a 26 m path), so the detector, the stereo geometry (disparity 60/35/7 px near to
  far, baseline right), the intrinsics, the timing (image stamps on IMU samples) and the gates were checked one by
  one and were all fine; the state file (`save_total_state` through ROS parameters) showed the filter's roll/pitch
  walking 7 deg away from the truth within 15 s of driving with a 0.22 m/s^2 accelerometer bias absorbing the
  gravity leak, while the gyro integrated from the bag matched the truth to 0.3 deg over 90 s. Chained KLT flow
  derotated by the gyro showed the images carrying about 15 % of the rover's yaw, and the pose publisher's link
  poses in the bag settled it: **the rig, its IMU link and both camera links yawed up to 128 deg relative to
  base_link; the rig kept a constant world heading (-89 to -97 deg) through every turn.** The rig's joints carried
  `<preserveFixedJoint>` and the physics left those links free about z. Fix: all of them are plain fixed joints
  now (sdformat lumps the rig into base_link, as imu_0 is; the SDF has only the four wheel joints), and the
  recorder's gate turns the rover in place and checks the image shift against imu_0's yaw
  (`check_stereo_sync.py --spin 0.3`: 1.20 on the fixed model, the rig IMU's yaw 1.00 of imu_0's; the broken model
  would score about 0). The 2026-10-31 stereo bag is unusable for VIO (its `vio_out`/`vio_edge` numbers are void);
  `apollo16_stereo2_20261031` (1 lap, sun 6) is being recorded on the fixed rover. OpenVINS moves to the rig's
  own IMU (`config/vio/kalibr_imu_chain.yaml`, cameras at p_CinI = (0, +-0.10, 0); the imu_0 setup stays as
  `estimator_config_imu0.yaml`), every "VIO IMU in base_link" default to (0.45, 0, 0.50), and the switch variants
  are generated from the lunar config by `config/slam/make_variants.py` instead of being stale copies.
- Phase 3 code (22:30, not yet built): the VIO's motion since the last scan as the registration's prior
  (`odom/gicp/prior/vioMode`: 0 off, 1 translation only with the IMU integration's rotation, 2 full;
  `odom/gicp/prior/vioMaxGapSec` 0.08 s of velocity extrapolation beyond the VIO's last message, otherwise the
  IMU prior stays for that scan). DLIO keeps the VIO poses in its own history (under a separate lock, so the
  prior-only use needs no filter), interpolates the rig IMU's pose at both scan stamps, moves the relative motion
  into base_link with the rig extrinsics and applies it to the registered pose of the previous scan; the deskew
  frames move with the prior so the placed points stay consistent with it. Counters in the VIO log line (prior
  used / fallbacks / last delta against the IMU prior in mm). Variant `__vio_prior.yaml` = VIO in the filter +
  translation prior. The label `vio_prior_s2` runs with `vio_out_s2` and `vio_edge_s2` against `ref_s2` on the
  new bag.
- New bag validated (22:22): `apollo16_stereo2_20261031` (one lap, 134 m, sun 6) -> `apollo16_stereo2_lap1_20261031`
  (trimmed whole, `segments.json`) -> `apollo16_stereo2_lap1_20261031_rt` (simulation-time order, 0 backward
  steps: **the stereo benchmark bag**). The rig IMU is clean (|a| 1.6185 at rest, 0.04 m/s^2 noise, no bias),
  the cameras turn with the rover (chained flow derotated by the gyro leaves a few px over the 15 deg and 10 deg
  turns), and OpenVINS on the first 85 s is sane on both IMU setups: speed within 0.05 m/s of the truth, position
  error 0.2-1.6 m after 65 s of driving, tilt within about 2 deg (the rig IMU, `estimator_config.yaml`, is the
  default; `estimator_config_imu0.yaml` the imu_0 variant). Open tuning item for later: 3 of 4 MSCKF features
  fail triangulation (far regolith with a 0.2 m baseline), 0.7-1.2 px residuals on the ones used. Labels running
  (`chain21`): `ref_s2` (3 runs), `vio_out_s2`, `vio_prior_s2`, `vio_edge_s2` (3 runs each, `--vio`) on the
  `_rt` bag.
- `ref_s2` / `vio_out_s2` (23:11): the reference on the new bag scores ATE 0.139 m, zig-zag 34 mm, RPE 10 m
  1.45 % (three runs). `vio_out_s2` is WORSE (ATE +67 %, zig-zag +183 %) because OpenVINS, fine for the first
  two legs, loses every feature at the slope waypoint (sim 165-175 s: the rover crests, pitches 15 deg in 5 s,
  nearly stops and turns in place while the level cameras see three quarters sky) and never recovers: zero
  SLAM and MSCKF features for the remaining 180 s, IMU-only propagation, 4-6 m/s and an ATE of 250-470 m over
  the lap. The images are not the problem (5,000-10,000 FAST corners in every frame of the lap, mean 23-39 of
  255). The filter's soft gate only inflated R, so the diverged VIO still leaked in.
  Two consequences, both built 2026-10-31 evening: (1) **the rig's cameras pitch down 20 deg**
  (`camera_pitch` in the rig xacro; the ground stays in view over a crest and the lever arm gives parallax in
  turns in place; `config/vio/kalibr_imucam_chain.yaml` carries R_CtoI = [0 -s c; -1 0 0; 0 -c -s]) and a
  third stereo recording follows; (2) **plausibility guards** so a diverged VIO cannot hurt the SLAM: DLIO
  skips VIO messages faster than `odom/iekf/vio/maxSpeed` (2 m/s) and suspends the updates after
  `odom/iekf/vio/maxGatedStreak` (50) consecutive gated innovations until one passes, the VIO prior falls back
  to the IMU's when the implied speed is implausible, and the backend adds no VIO edge above
  `vio_edge_max_speed_mps` (2 m/s). The `*_s2` labels of the VIO switches are therefore void as measurements
  of the switches; they stand as the record of the failure.
- `vio_prior_s2` / `vio_edge_s2` (23:53, same diverged VIO, no guards yet): the VIO prior took the registration
  off the map (ATE 92 m, the prior replaced the IMU's 5,760 times with a 3 m delta at the end) and the VIO
  edges cost ATE +91 %. Both are exactly what the plausibility guards now refuse (a prior or an edge implying
  more than 2 m/s falls back / is skipped). The OpenVINS process costs +150 to +200 % CPU on this machine
  (all SLAM processes 348 -> 497-554 %): the price of the separate process, to be weighed once it helps.
  `chain22` (from 23:55): build with the guards and the prior, OpenVINS at DEBUG stage counts, the sensor gate
  with the pitched cameras, `apollo16_stereo3_20261031` recorded, trimmed, retimed, OpenVINS validated over the
  whole lap, then `ref_s3` and (if the VIO held the lap) `vio_out_s3`, `vio_prior_s3`, `vio_edge_s3`.
- `apollo16_stereo3_20261031` (2026-11-01 00:20, pitched cameras, guards built): the gate passes (spin ratio
  1.38; a camera looking down sees nearer ground in the band and the rig's lever arm adds flow, so the window is
  0.6-1.6 now), `ref_s3` scores like `ref_s2`, but OpenVINS diverges EARLIER with the pitched cameras: right
  after the first fast turn on the spot (0.56 rad/s at sim 82-88 s) its MSCKF chi-square ratios jump to 300,
  the speed doubles by 90 s and the lap ends 1.4 km off. What was checked and cleared: the camera-to-IMU time
  offset is 0 ms (image flow against the gyro over the whole lap, correlation 0.993; online calibration finds
  +2 ms); the simulated IMU includes its lever arm (the two IMUs on the body differ by the centripetal term as
  physics requires); the rover's own shadow is not in view during that turn, and a per-frame dark-region mask
  (`scripts/openvins_patch_dark_mask.py`, node parameters `dark_mask_threshold` / `dark_mask_dilate_px`, kept
  but off by default) made things worse because masking the sky also removes the horizon features that anchor
  the orientation. What the tracker's own picture shows: after the global histogram equalisation the near
  regolith is salt-and-pepper noise, on which KLT and the stereo match are fragile, while the far features
  are stable. A front-end sweep on the first 110 s (no equalisation, CLAHE, half-resolution images, both, a
  stricter detector, a longer clone window) is the next step; the labels run once a setting holds the lap.
- Front-end sweep (01:33, first 110 s of the pitched-camera bag, position error at the end): global histogram
  equalisation (the stock setting) 14.6 m; **no equalisation 0.8 m; CLAHE 0.2 m**; half-resolution images
  1.4 m; no equalisation + half resolution 0.6 m; a stricter detector or a longer clone window with the global
  equalisation still 23 m / 6.2 m. So the culprit was the global equalisation amplifying the regolith's noise
  into false corners and bad stereo matches, which the first fast turn then turned into a filter divergence.
  `config/vio/estimator_config.yaml` now uses CLAHE (`chain24`: the full lap, then `vio_out_s3`, `vio_prior_s3`,
  `vio_edge_s3` against `ref_s3`).
- CLAHE over the whole lap (01:40): the VIO holds the first two legs and the climb (0.2-2 m of error up to
  135 s of driving) and dies at the slope waypoint, where the rover turns 150 deg on the spot at up to 0.9 rad/s
  with the speed at 0.06 m/s and only 0-2 SLAM features left after the climb (17 px of image motion per frame
  for the fixed 15 px KLT window). Nothing recovers it afterwards: stock OpenVINS never re-initialises. The
  labels `vio_out_s3`, `vio_prior_s3`, `vio_edge_s3` run anyway (01:45): with the guards, a VIO that is right for
  150 s and then refused is a fair measurement of the switches on the first two legs.
- Restart on divergence, coded 02:00 (built and tested after the labels): the OpenVINS node exits when its speed
  stays above `vio_diverge_velocity_mps` (3 m/s) for a second (`scripts/openvins_patch_diverge_exit.py`), the
  launch respawns it, and it initialises again (statically at a standstill, dynamically while driving with
  `init_dyn_use`). DLIO and the backend treat a VIO pose jump above 0.5 m (or a stamp going backwards) as a new
  frame (`odom/iekf/vio/frameJumpM`, `vio_edge_frame_jump_m`): history and buffer cleared, nothing spans the
  restart. The dark-region mask stays available but off.
- **Phases 2-4 measured** (`vio_out_s3`, `vio_prior_s3`, `vio_edge_s3`, three runs each against `ref_s3`, the
  CLAHE front end, the VIO right for the first two legs and the climb, refused by the guards after the crest):

  | label | ATE | zig-zag | RPE 1 m | RPE 10 m | pitch | plain -> start leg | CPU |
  |---|---|---|---|---|---|---|---|
  | `ref_s3` | 0.129 m | 31 mm | 0.090 m | 1.23 % | 0.063 deg | 0.190 m | 343 % |
  | `vio_out_s3` (filter) | +26 % | +35 % | +29 % | +28 % | +70 % | -8 % | +42 % |
  | `vio_prior_s3` (prior) | +306 % | +32 % | +57 % | +157 % | +87 % | +111 % | +36 % |
  | `vio_edge_s3` (backend) | +5 % | +15 % | +20 % | +16 % | +104 % | -21 % | +32 % |

  None of the three improves the SLAM on this bag. The LiDAR registration against the 1 cm terrain with the
  ideal IMU is far more accurate than the stereo VIO (0.2-2 m over 135 s, velocity noise of centimetres per
  second), so feeding the VIO's velocity into the output filter adds jitter and pitch error, the VIO prior is
  worse than the IMU's and takes the registration with it (even below the 2 m/s guard), and the backend edge is
  neutral overall with a hint on the LiDAR's hardest leg (plain -> start, -21 %, within the spread). The guards
  worked (44,792 implausible and 17,182 suspended messages refused in `vio_prior_s3`; 0 implausible in the two
  others). The separate OpenVINS process costs +32-42 % CPU and +47-78 % GICP solve time from contention. By
  the guard all three switches stay off; the plumbing, the guards and the scorer's VIO block remain for the
  hardware phase, where the LiDAR will not have an ideal IMU and a 1 cm terrain.
- Restart on divergence tested (03:16, one replay with `--vio` on the `vio_out` variant): OpenVINS reported
  "estimate diverged (3.2 m/s for 1.1 s at 187.5): exiting for a respawn", the launch respawned it, DLIO logged
  4 VIO frame resets and the VIO's own path stayed within 4.7 m instead of flying off by hundreds of metres.
  OpenVINS's dynamic initialisation (starting while driving) is not usable with its stock settings on this
  rover (a probe starting the bag at 200 s initialised with a 15 deg attitude uncertainty and ran away at
  once), so `init_dyn_use` stays off and a respawned node initialises again at the next standstill. The
  respawn costs CPU while the node keeps trying to initialise (676 % all SLAM in that run). The fixed KLT
  window of OpenVINS (15 px, 5 levels) against the crest turn's 17 px per frame is the remaining lead if the
  VIO is ever to hold that waypoint.
- Textured map (03:40-04:10, built on request): the backend colours each keyframe from the left image nearest
  its stamp (depth-buffered projection, TF extrinsics), keeps the gray per keyframe point, and publishes a
  voxelised coloured map (`/modified_map_colored`, x y z rgb intensity time) next to the timed map, plus
  `map_colored.pcd` on save. No factor is added and the frontend is untouched. Switches: launch `texture_map`,
  `hazard` (false = pose graph and maps only); replay `--texture`, `--no-hazard`; `tools/colored_map_view.py`
  for a picture. Measured on the third stereo bag (one lap, `--texture --no-rviz`): 392 keyframes coloured,
  none without an image within 30 ms, about 78 % of each keyframe's points painted (the rest outside the
  camera's view or occluded), 581 k voxels at 10 cm, `map_colored.pcd` 5.8 MB; the top view shows the crater
  rims lit and the interiors shaded as the low sun does it. Cost against `ref_s3`: all SLAM processes 356 %
  against 343 % CPU, hazard update 119 against 120 ms, RAM +0.3 GB (the coloured voxels and the cached
  message), ATE within the reference spread. The frontend's numbers are unchanged. With RViz drawing the big
  clouds the same run reads 665 %, which is RViz, not the backend.

## Round 6 (from 2026-11-01 05:00): lossless CPU cuts

Principle: only the estimate runs at sensor rate; every map product runs on its own cadence, incrementally, and
never does work proportional to the whole map per update. Nothing here changes what DLIO or the graph computes,
so the accuracy labels must stay within the reference spread.

**Where the 343 % went** (reference on the third bag, per process from the resource samples, per thread from a
profiling replay): backend 212 %, DLIO 99 %, DLIO's map node 16 %, the RViz path helper 11 %. In the backend:
the DEM recorder re-rasterised the whole corrected map (5 M points) every 5 s, each pass 14 s, most of it nine
ASCII rasters of 1.8 M cells through iostream formatting (about one core); the hazard mapper at 120-150 ms per
update, saturated, keeping one scan in three (0.7 core); the timed map serialised on every submap (17 MB). In
DLIO only 13 ms per scan is the registration pipeline; the rest is spread over twenty OpenMP worker threads at
3 % each, the signature of libgomp spinning between the short parallel regions at 20 Hz.

**Switches built (05:10):** the DEM is incremental (jobs carry only the points appended to the assembled map,
a full restart when the map was rebuilt, per-cell statistics recomputed for touched cells, rasters written
every `dem/write_period_sec` = 60 s and on every save, a `to_chars` writer); `hazard/input_rate_hz` = 5 (a
designed intake rate instead of an overflowing queue; skipped scans are not drops); the timed and coloured
maps serialised at most every `modified_map_timed_min_interval_sec` = 5 s (the heartbeat republishes the
cached message); and `gomp_spincount` (launch; replay `--omp-passive`) for passive OpenMP waiting in the
three SLAM nodes. Labels: `cpu6` (the first three, as the new defaults) and `cpu6_omp` (plus passive OpenMP)
against `ref_s3`.
**`cpu6` (05:55, three runs against `ref_s3`):**

| | `ref_s3` | `cpu6` | `cpu6_omp` |
|---|---|---|---|
| all SLAM processes, CPU | 343 % | 197 % (-42 %) | 187 % (-46 %) |
| RAM peak | 2.2 GB | 1.9 GB | 1.9 GB |
| hazard update, dropped scans | 117 ms, 4276 | 93 ms, 0 | 91 ms, 0 |
| GICP solve, mean | 7.3 ms | 5.8 ms | 6.4 ms |
| ATE | 0.129 ± 0.018 m | 0.139 ± 0.025 m | 0.146 ± 0.017 m |
| zig-zag | 31 mm | 32 mm | 34 mm |
| heading error | 0.339 ± 0.085 deg | 0.332 ± 0.035 deg | 0.384 ± 0.268 deg |

`cpu6` is adopted as the default: every accuracy metric within the reference spread, 42 % less CPU, the
hazard mapper no longer saturated (0 drops instead of two scans in three), the DEM passes 40-115 ms instead
of 14 s, and the registration itself faster from the lower contention. Passive OpenMP waiting (`cpu6_omp`)
takes another 10 points but every accuracy metric drifts a little worse with wider spreads (the parallel
regions pay a wake-up latency, GICP 6.4 against 5.8 ms), so it stays a switch, off. The DEM's exactness is
proven by `test_dem_raster` (the accumulator fed in chunks against an independent one-shot rasteriser, cell
for cell; the primitives live in `dem_raster.hpp`), since the saved `map.pcd` is a downsampled copy and
cannot reproduce the DEM's input. Next label `cpu6b`: the deskewed cloud published on every second scan
(`pointcloud/publishEveryNth` 2; the backend takes one scan per keyframe, the hazard mapper its 5 Hz, the
map node the keyframe stream) and the RViz path helper only with RViz.

**`cpu6b` (06:23, three runs against `ref_s3`):** CPU 186 % (-46 %), RAM 1.86 GB (-16 %), GICP 6.0 ms, hazard
92 ms and 0 dropped; ATE 0.149 ± 0.016 m, RPE 1.40 %, zig-zag 33 mm, heading 0.323 deg, every accuracy verdict
`~`. Adopted. The DLIO deskewed cloud now goes out on every second scan (`pointcloud/publishEveryNth` 2): the
backend builds each submap from the single latest cloud once the odometry has moved its threshold, the hazard
mapper takes 5 Hz of the stream and the frontend map node is not needed, so no product loses a point; the RViz
path helper runs only with RViz. Two facts about the benchmark itself, from the nine reference / cpu6 / cpu6b
runs: the backend's map-to-odom correction is identity on this lap (no loop closure fires), so the scored
trajectory is DLIO's own and nothing the backend consumes can move the ATE; and that ATE scatters from 0.114 to
0.166 m run to run (the frontend is not deterministic under replay timing), so three runs resolve differences
of about 0.03 m and the monotone-looking 0.129 / 0.139 / 0.149 means of the three labels are inside that noise.

**`cpu6c` (06:47, three runs against `ref_s3`):** all SLAM processes 148 % (-57 %), RAM 1.69 GB (-24 %), the
backend 63 % (was 213 %), hazard updates 59 ms mean with none dropped, GICP 5.3 ms, DLIO 43 ms CPU per scan;
the hazard grid grew 40 times in the lap instead of 383 and re-inflated the whole grid once (the first
update) instead of 380 times; 47 gtests pass. Every accuracy verdict is `~` again, but the along-track RMSE
has crept over the labels (0.088, 0.092, 0.104, 0.113 m from `ref_s3` to `cpu6c`) while every registration
statistic (iterations, fitness, overlap, Hessian, scans processed) is identical, so a bisect runs before the
defaults are locked: the current defaults with every deskewed scan published (variant `every_scan`, label
`cpu6c_n1`) against three more runs of the defaults (six runs of `cpu6c`). Two more lossless cuts went in: The hazard mapper's grid grew on 383 of the 904 updates of a
cpu6 lap and each growth re-inflated the whole grid (124 ms against 69 ms for a windowed update, 57 % of the
mapper's 83 s); the cells a growth adds are unknown and, by the observation padding (slope radius on top of the
inflation radii), beyond the reach of every hazard, so growth now copies the grid and inflates nothing, and
`hazard/grow_margin_m` (2 m) grows the grid ahead of the observation so the copy is paid once per 2 m of travel
(two gtests: costs cell for cell against the full distance transform after growth, the margin's extent). And
`dlio_map_node`, the frontend map accumulator, is off by default (`frontend_map`, `--frontend-map`): nothing in
the stack reads `/dlio/map`, the backend builds the corrected maps, and it cost 15 % of a core plus memory that
grows with the map. A resource budget (`tools/slam_budget.py`, `config/slam/slam_budget.json`) now checks every
scored run against limits well above the measured defaults, so a regression trips it without a comparison.

**Where DLIO's core goes (sudo-gdb stack samples of the `cpu6c` defaults, 06:50):** of 280 busy thread
samples, 245 sit inside `libgomp` itself below `start_thread`, i.e. OpenMP workers spinning between parallel
regions (one five-worker pool per executor thread that ever ran a region, four pools), and only about 35 are
work (nanoflann searches, covariances, the cloud conversion). Yet the passive-waiting label `cpu6_omp` left
DLIO at the same 94.6 % while the backend lost 9 points, so a passive-waiting profile with the same samplers
follows the bisect to see whether those spinning samples disappear, before any intermediate spin budget is
tried as a switch.

**Bisect (11:37):** the defaults with every deskewed scan published (`cpu6c_n1`, three runs) scored ATE 0.150
± 0.071 m (0.231, 0.102, 0.117) against 0.145 ± 0.018 m for five runs of the defaults with every second scan,
along-track 0.105 against 0.104 m, every verdict `~`: the publish rate changes nothing measurable. The scatter
is the frontend's own (the two best runs of the day and the worst sit in the same three-run label; the IMU
calibration DLIO logs is bit-identical in all 17 runs, so the start is not the source; the error profile along
the lap shifts as a whole, which is what a heading drift looks like once the global 4-DoF alignment has absorbed
it). Decision: `pointcloud/publishEveryNth` goes back to 1 by default. It bought about one CPU point, nothing
measurable otherwise, and it is the one round-6 change that touches the frontend's timing at all; a switch
with no measured gain is not worth any doubt. Everything else of `cpu6c` stays. Two findings about the bench
itself: a `cpu6c` run that spanned a four-hour system sleep (07:23-11:30) was quarantined under
`slam_runs/_invalid/` before any comparison used it, and three `odom_to_path.py` helpers from runs at 23:37,
00:32 and 00:52 the night before were found still spinning at 10 % of a core each: they answered SIGINT only,
so they outlived launches that were killed harder; the helper now exits on SIGTERM too and the three were
stopped by PID. They ran through every label of the day, the reference included.

**Confirmation (12:31):** label `round6` (the defaults as they now stand, four runs) and `ref_load`
(variant `ref_load`: the pre-round-6 cadences and bounds, `--frontend-map`, three runs), both against
`ref_s3`. `ref_load` carries the old load on the new code, so it separates "the lighter load changed the
frontend's scatter" from "the reference was measured at a different time of day".

**The spinning hypothesis is wrong (passive profile, 11:46).** With `GOMP_SPINCOUNT=0` the same gdb samples
still show 263 of 278 busy DLIO thread samples "inside libgomp", and the per-thread CPU sampler (which
measures real ticks) gives the twenty workers 2.3-2.4 % each against about 3 % when they may spin; DLIO as a
whole stays at 86 % and 44 ms of CPU per scan. The explanation: libgomp issues its futex waits through an
inline `syscall` instruction, so a worker asleep in a futex shows its program counter inside the stripped
`libgomp.so.1`, not in libc, and the stack summary counted sleepers as busy. The workers' real time is real
work: the nearest-neighbour searches of the registration (source covariances once per scan, correspondences
on every iteration), spread over four worker pools because the LiDAR callback lands on any of the four
executor threads. Passive waiting therefore saves nothing in DLIO and costs the backend's fit threads a
wake-up on every update; it stays a switch, off. DLIO's remaining 85 % of a core is the registration itself
plus the IMU-rate propagation and the 100 Hz pose publisher, and none of that can go without touching the
estimate: the frontend is done for this round.

**Noted for later (not this round):** DLIO's memory grows about 80 MB per minute of driving: it keeps every
keyframe's cloud and its per-point GICP covariances (about 2.5 MB per keyframe) so that a keyframe re-entering
the submap costs nothing. Dropping the covariances of keyframes far from the rover and recomputing them on
re-entry would bound the memory of a long mission at a CPU cost only when ground is revisited; that is a
trade, not a lossless cut, so it is parked here.

### Round 6 result (2026-11-01 12:31)

| | `ref_s3` (00:28, 3 runs) | `ref_load` (12:11, 3 runs) | `round6` (11:46, 4 + 4 runs) |
|---|---|---|---|
| all SLAM processes, CPU | 343 % | 306 % | **146 %** |
| graph backend CPU | 213 % | 194 % | 62.5 % |
| RAM peak | 2.22 GB | 1.84 GB | **1.66 GB** |
| hazard update mean / p95, dropped | 117 ms, 4276 | 110 ms, 4064 | **58 / 103 ms, 0** |
| GICP solve, mean | 7.29 ms | 6.23 ms | 4.90 ms |
| DLIO CPU per scan | 51.5 ms | - | 42.3 ms |
| ATE | 0.129 ± 0.018 m | 0.161 ± 0.012 m | 0.149 ± 0.022 m (8 laps) |
| along / cross-track | 0.088 / 0.092 m | 0.102 / 0.123 m | 0.109 / 0.113 m |
| heading error | 0.339 ± 0.085 deg | 0.496 ± 0.127 deg | 0.411 ± 0.047 deg |
| zig-zag | 31 mm | 33 mm | 32 mm |
| RPE 10 m | 1.23 % | 1.38 % | 1.39 % |

`round6` is the default and the reference for the next rounds; every accuracy verdict against `ref_s3` is
`~`, every resource verdict `better`, and every run is within the budget. `ref_load` settles the question the
along-track creep raised: on the same binaries, the pre-round-6 cadences and bounds with the map node on (306 %)
score exactly like the new defaults (0.161 against 0.159 m, 0.496 against 0.411 deg, all `~`), so nothing
round 6 changed moves the estimate. What remains is that both score about 0.03 m worse than the reference
measured at 00:28, and so did the round-5 label `vio_edge_s3` at 01:23 (0.135 m): all six night runs sit at
0.11-0.15 m and all twenty day runs at 0.10-0.23 m with a mean near 0.16. The two explanations left are the
build (the reference ran on binaries from before the VIO guards of round 5 went into DLIO, code that is inert
with the VIO off; the backend cannot move the scored path at all) and the machine's state by time of day (the
replay is wall-clock timed; registration statistics are identical in every run). The decisive check costs no
code: `benchmark_slam.sh --label round6_night --repeats 3 --compare-to round6` run at night; if it comes back
at 0.13 m the benchmark needs a time-of-day control, if at 0.16 m the shift is the round-5 build and the
reference is simply `round6` from now on. Either way the frontend is untouched by this round.

**What changed in DLIO since the round-4 commit (read 2026-11-01 13:30).** The uncommitted DLIO diff (420
lines) is the round-5 VIO plumbing plus the publish switch: additions, and three substitutions. Deskew and
preprocessing multiply by a VIO prior delta that is identity whenever `odom/gicp/prior/vioMode` is 0; the output
filter's apply step moved verbatim into `applyFilterCorrection` so the VIO updates share it (the only visible
difference, which history entries get the correction when no scan-time reference is found, resolves to the
same empty set); the re-seed of a rejected registration is gated on the VIO being enabled. New parameters,
counters and two diagnostic indices (75, 76) complete the diff. With the VIO off the frontend's behaviour is,
by reading, the round-4 one. Because reading is not proof, the live script gained `SLAM_OVERLAY_SETUP`
(replay `--overlay-setup PATH`): a ROS overlay sourced after the SLAM workspace, so a package built elsewhere
replaces the workspace's one for a run. The committed DLIO was built in `~/slam_ws_r4` inside the container
(from `git archive HEAD`, exported under `slam_output/dlio_r4_src/`) and replayed for three laps as label
`dlio_r4` under the current launch, config and backend; the result is appended below.

**A/B, first pass (13:17):** `dlio_r4` (three laps, 151 % CPU, the same load as the defaults) scored ATE
0.131 ± 0.043 m (0.180, 0.100, 0.114), along-track 0.089 m, heading 0.328 deg: the night-time reference's
numbers (0.129 / 0.088 / 0.339), against 0.159 / 0.109 / 0.411 for the current DLIO (`round6`, four laps). The
registration statistics are the same (4.84 against 4.92 iterations, 5.1 against 4.9 ms). That takes the
machine's time of day off the table: the committed frontend reproduces the reference today, the current one
does not. Three against four laps with spreads of 0.04 and 0.02 m is not yet a verdict (`~`), so an
interleaved A/B follows: four more laps of each, alternating, with a Welch t-test over all runs of both labels.
If the gap holds, the three substitutions of the diff get bisected by rebuilding the current source with each
one restored; if it closes, the frontend is cleared and the scatter is the explanation.

**A/B, interleaved (14:09): the gap closes.** Four more laps of each binary, alternating: `dlio_r4` 0.157,
0.162, 0.135, 0.183 m and `round6` 0.164, 0.108, 0.131, 0.155 m. Over everything, the committed DLIO scores
0.147 ± 0.032 m (7 laps) and the current one 0.149 ± 0.022 m (8 laps), along-track 0.103 against 0.106 m,
heading 0.353 against 0.389 deg, Welch t = 0.13; against all ten laps ever run with round-4 frontend code
(mean 0.142 m) the current defaults give t = 0.62. The frontend is cleared, as the reading said. The three
reference laps of 00:28 (0.123, 0.149, 0.114 m) were a low draw from a scatter whose per-lap standard
deviation is about 0.03 m: nothing in round 5 or round 6 moved the estimate. Consequences: `round6` with its
eight laps (0.149 ± 0.022 m) is the reference from here on; and for decisions about ATE on this bag three
laps resolve differences of roughly 0.05 m only, so a claimed ATE change below that needs six to eight laps,
while zig-zag, RPE, GICP time and the resource metrics stay decisive with three. The night re-run suggested
above is no longer needed. The bisect variant `all3` (prepared, not built) is moot.

Open for later rounds: DLIO's memory growth (per-keyframe covariances), a time-of-day control in the
benchmark if the night run says so, and the hazard mapper's remaining 58 ms per update (the plane refit of
about 21 000 cells per update, already parallel and exact; the next lossless step would skip refits whose
neighbourhood heights did not change beyond the fusion noise, which needs its own exactness argument).

Results are appended as the phases land.

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

## LiDAR on a mast: a 360-degree sensor from a higher point (2026-11-01, on the user's ask)

**Question.** Does a 360-degree LiDAR mounted high do better than the front-looking Seyond (120 x 70 deg,
0.31 m above base_link) on the lap?

**Setup.** The rover model gained an optional mast at the centre of the sensor arch's top with an Ouster OS1-64
model on it (clearpath_sensors_description's macro: 1024 x 64 rays, 360 x 42.4 deg, 20 Hz, 0.3-150 m, no noise
like the Seyond's model): `xacro lidar360:=true lidar360_mast:=0.40`, sensor frame `lidar360_0_sensor_link` at
(-0.403, 0, 1.314) in base_link, i.e. 1.0 m above the Seyond and 0.4 m behind the base origin; bridged to
`/husky/sensors/ouster/points`. Both LiDARs run in one recording of the benchmark route (`record_bag.sh
--sim-args extra_xacro_args:=lidar360:=true`, with the stereo rig and the IMUs as in the stereo bag, real-time
factor about 0.3), so the two SLAM runs see the same drive. The SLAM side: replay `--lidar-topic
/husky/sensors/ouster/points --scan-period 0.05` with the variant `lidar360` (extrinsics, hazard ray-origin
frame, crop box 1.5 m because the rover's body is in a 360-degree view; everything else as `round6`). Deskew
with synthetic per-point timing stays on for both, as the benchmark runs it; note that a simulated scan is an
instantaneous snapshot, so that timing is an approximation for either sensor (something to measure on its
own some day: a `dlio_deskew:=false` label). Six laps per sensor, interleaved, on the mast bag; the Seyond laps
of that bag are also the control against the stereo bag's `round6`.

Geometry worth knowing before the numbers: the OS1's lowest ring (-21.2 deg) from 1.31 m above base_link
reaches the ground about 3.5 m from the mast, where the Seyond's -35 deg from 0.31 m reaches it at 0.5 m in
front; the mast sensor sees all around and far, the Seyond sees the ground right ahead. The hazard mapper gets
the near ground from the map's memory in the mast case.

**Result (16:31, six laps per sensor, interleaved, on the mast bag `apollo16_lidar360_lap1_20261101_rt`):**

| | Seyond, front, 0.31 m (`seyond_mb`) | OS1 on the mast, 1.31 m (`lidar360`) |
|---|---|---|
| ATE | 0.150 ± 0.024 m | **0.066 ± 0.006 m** (-56 %) |
| along / cross-track | 0.110 / 0.097 m | 0.044 / 0.045 m |
| heading error | 0.431 ± 0.161 deg | 0.093 ± 0.006 deg (-79 %) |
| RPE 10 m | 1.31 % | 0.54 % |
| zig-zag | 31 mm | 17 mm |
| ATE per leg: to rocks / rocks to slope / slope to plain / plain to start | 0.125 / 0.125 / 0.151 / 0.188 m | 0.055 / 0.037 / 0.044 / 0.060 m |
| height scale (vertical drift per metre climbed) | 2.0 % | -0.1 % |
| GICP fitness / overlap / iterations | 2.24 / 0.984 / 4.98 | 1.52 / 0.995 / 3.25 |
| GICP solve, DLIO CPU per scan | 5.0 ms, 43 ms | 22.7 ms, 217 ms |
| scans processed | 6864 | 5610 (18 % skipped: the frontend cannot keep 20 Hz at this load) |
| all SLAM CPU / RAM peak | 148 %, 1.6 GB | 578 %, 4.6 GB (DLIO +360 MB/min) |
| hazard update, dropped | 59 ms, 0 | 308 ms, 222 |

Every accuracy verdict is `better` and the mast sensor's lap-to-lap spread is a tenth of the Seyond's; the
Seyond on this bag scores exactly like `round6` on the stereo bag (0.150 against 0.149 m), so the route and
the recording are consistent and the gain is the sensor's. The gain is largest where the forward sensor has
least to see: the slope and the featureless plain (-71 %, -68 %), and the height scale error of the forward
sensor (2 % of the climb, the long-standing finding) vanishes with the all-round view. The registration itself
is healthier (fitness 1.5 against 2.2, overlap 0.995, three iterations instead of five).

The price at the untouched settings is a stack that does not fit the rover: a full circle to 150 m voxelised
at 0.15 m gives the registration four to five times the points, the frontend skips a fifth of the scans, the
hazard mapper saturates with whole-circle scans at 5 Hz, and DLIO's per-keyframe covariances grow its memory
five times faster. So the next label measures the same sensor at a compute budget near the Seyond's:
`lidar360_lean` (registration voxel 0.30 m, hazard mapper 2.5 Hz), six laps; appended below.
