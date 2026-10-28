# Gazebo validation of the final world (`rover`)

Data: rover-test agent, 2026-10-26, 23:09-23:51; Gazebo runs 23:15-23:51. It was stopped at the user's request
during its optional diagnostic phase, before it wrote a report. This file was written afterwards from its data and
logs, without running Gazebo. World: the final build `/home/nargon/togo_assets/lunar/worlds/apollo16_core/`
(`final_build.md`).

- All runs were headless, one Gazebo at a time, with `GZ_PARTITION=claude_rover` and `ROS_DOMAIN_ID=132`.
- The machine was quiet: other processes used 0.00-0.03 cores during the drives, and MemAvailable stayed at or above
  11.0 GB.
- No package file was changed, and the WSL crash-dump folder is empty.
- Raw data is in `EXP=/home/nargon/togo_assets/lunar/experiments/rover/final_validation/` (`runs/*/robot_tests.json`,
  `analysis/`, `logs/`, `scripts/`). The tools also wrote `robot_tests.json`, `robot_tests_stock.json`,
  `bench_world_flat_real.json`, `lidar/` and `lidar_checks.json` to `world_dir/reports/`.

## 1. Verdict against DESIGN 13

| area | criterion | measured | margin | verdict |
|---|---|---|---|---|
| world only | loads with no errors | 0 `[Err]`, 0 `[Wrn]` (bench and 3 capture servers) | | PASS |
| world only | D3D12 renderer | `libd3d12.so`, `libd3d12core.so`, `libdxcore.so` mapped, no llvmpipe | | PASS |
| world only | load time ≤ 30 s | first LiDAR cloud 27.48 s (captures 26.6-27.2 s) | 2.5 s | PASS |
| world only | RTF ≥ 0.95 | 0.9998 (40 s window) | 0.05 | PASS |
| world only | server RSS ≤ 5 GB | 3.69 GB peak | 1.3 GB | PASS |
| LiDAR | no returns beyond the ring | 0 of 260,369 (18 poses) | | PASS |
| LiDAR | rock detection consistent with the gazebo small-object results | 7 of 7 judged bins (18 poses); 4 of 5 on the 6 phase-3 poses alone (5.2) | | PASS |
| robot | controllers active, no spawner failure, 3 of 3 launches | 12 of 12 active, 0 spawner failures | | PASS |
| robot | RTF ≥ 0.9 while driving (headless) | 0.929-0.996 | 0.029 | PASS |
| robot | flat: steady speed ≥ 0.95 of the command | 0.997-1.005 | 0.047 | PASS |
| robot | flat: forward distance ≥ 0.93 of 3 m | 0.950-0.955 | 0.020 | PASS |
| robot | flat: in-place yaw tracking ≥ 0.85 | 0.968-1.016 | 0.118 | PASS |
| robot | flat: airborne ≤ 2 % | 0-1.67 % | 0.33 points | PASS |
| robot | slope climbed (≥ 80 % of the commanded 6.0 m, mean climb ≥ 5 deg) | 0.649 / 0.666 / 0.700; 10.9-11.5 deg | -0.10 to -0.15 | **FAIL** (section 2) |
| robot | rock traversed without stalling (≥ 80 % of 4.5 m, no 1 s window below 0.05 m/s) | 0.969-0.985; slowest 1 s 0.20-0.28 m/s | 0.169 | PASS |
| robot | turn site: in-place yaw tracking ≥ 0.85 | 0.888-0.889 | 0.038 | PASS |
| visual | camera views reviewed | not run (section 7) | | - |

The flat thresholds are the decision at the top of DESIGN 13, which supersedes the older 95 % bullet. **Overall:**
- world only and LiDAR: 7 / 7;
- robot: 9 of 10 (in `robot_tests.json` only `drive_slope` fails; it fails on the site's rough natural slope, section 2);
- visual: not run.

### 1.1 Per site

The default launch (ours, `robot_collision:=simplified`, patched Bullet 1 ms) ran 12 times: flat, slope, rock, turn,
three rounds (`EXP/runs/robot_main/`). `tools/run_robot_tests.py` starts `ros2 launch launch/apollo16_sim.launch.py
site:=<site>` with the defaults.

| site | pose (x, y, yaw) | command | READY (s after launch) | RTF driving (worst 2 s bin) | results (3 runs) |
|---|---|---|---|---|---|
| flat | -2.00, -4.50, -22.5 deg | 0.5 m/s for 6 s (3.0 m), then in place 0.6 rad/s for 6 s | 38.0-47.3 | 0.947-0.957 (0.873) | distance 0.950-0.955 (2.85-2.87 m), steady speed 0.997-1.005, yaw tracking 0.968-1.016, airborne 0-1.67 % |
| slope | 23.50, -16.50, -157.5 deg | 0.4 m/s for 15 s (6.0 m) | 38.8-40.8 | 0.929-0.990 (0.913) | 0.649-0.700 (3.9-4.2 m), climb 10.9-11.5 deg, no stall (slowest 1 s 0.11-0.18 m/s) |
| rock | -4.12, -7.41, 67.5 deg (8.7 cm rock 2 m ahead) | 0.3 m/s for 15 s (4.5 m) | 37.0-42.6 | 0.958-0.977 (0.942) | 0.969-0.985, slowest 1 s 0.20-0.28 m/s, max roll 3.7-7.1 deg, airborne 0.1-0.9 % |
| turn | -3.00, 3.00, -67.5 deg | in place 0.6 rad/s for 6 s (206 deg) | 38.1-40.8 | 0.964-0.996 (0.952) | yaw tracking 0.888-0.889 (179 deg), drift 0.54-0.55 m, airborne 0 |

- **Launch:** both controllers were active in 12 of 12 (`list_controllers`), with 0 spawner failures. The robot's LiDAR
  was advertised 32.4-38.7 s after the start. The first launch of the series was the slowest (READY at 47.3 s).
- **Spawn:** the rover was at rest at the first ground-truth sample in 12 of 12 (drop 0.0 m). The settle before that,
  during the render-scene stall, was not observed (section 7).
- **Server:** RSS was 3.74-3.79 GB with the rover. Every launch log has 0 `[Err]` and the same 39 known `[Wrn]`
  (ColladaLoader `COLOR`, `Dummy SetJointVelocity` / `SetJointPosition`, Ogre material scripts), as in e2e_launch.md 10.
- **Stop:** gz gets SIGKILL by design, so each log has one "process has died ... exit code 137" line. Stops took
  0.76-1.02 s with 0 processes left.
- **Phase 3** (another build, CAD robot collisions; gz_validation 5): slope 0.853, rock 0.997-0.998, turn site
  0.953-0.954, RTF while driving 0.19-0.92.

## 2. Slope: why it fails

All climbs use the same commands (0.4 m/s for 15 s). The smooth inclines are a diagnostic world
(`EXP/worlds/inclines/`):
- the final world's physics header (patched Bullet 1 ms, gravity 1.62);
- one ramp per angle, mu 0.4 as on the world's ground, 1,024 collision triangles of 25 x 50 cm;
- the rover spawned 0.8 m before the foot.

| drive | climbed (of 6.0 m) | mean climb angle | longitudinal slip | max pitch / roll | airborne | sideways drift / yaw change |
|---|---|---|---|---|---|---|
| slope site, 3 runs | **0.649 / 0.666 / 0.700** | 10.9-11.5 deg | 0.29-0.35 | 20.0-22.8 / 7.5-10.1 deg | 1.7-3.7 % | 0.79-1.28 m / -15.6 to -19.2 deg |
| smooth 10 deg | 0.953 | 8.7 deg | 0.044 | 10.0 / 0.2 deg | 0.1 % | 0.05 m / -1.0 deg |
| smooth 12.4 deg | 0.938 | 10.7 deg | 0.057 | 12.4 / 0.3 deg | 0.1 % | 0.05 m / -1.0 deg |
| smooth 15 deg | 0.913 | 13.0 deg | 0.080 | 15.0 / 0.1 deg | 0 | 0.02 m / -0.4 deg |
| smooth 17 deg | 0.895 | 14.7 deg | 0.097 | 17.0 / 0.4 deg | 0 | 0.08 m / -1.4 deg |
| smooth 19 deg | 0.837 | 16.2 deg | 0.155 | 19.1 / 0.7 deg | 0 | 0.08 m / -2.0 deg |
| smooth 21 deg | 0.710 (fail) | 17.4 deg | 0.280 | 21.0 / 0.8 deg | 0 | 0.07 m / -2.2 deg |

The slope site's terrain (`EXP/analysis/sites_extra.json`, 2 cm collision grid):
- The 5 m run averages 12.4 deg (rise 1.10 m), with a line-fit RMS of 57 mm.
- Its sub-slopes (site finder) range from 6.3 to 16.9 deg. The cross slope is at most 1.5 deg, and there is no rock of
  10 cm or larger in the corridor.
- Measured over the wheelbase (0.512 m), the centre line and both wheel tracks reach 19.3-20.6 deg (p90 17.9-18.9
  deg). 24-35 % of positions are steeper than 15 deg, and 17-22 % are steeper than 17 deg.
- 14 rocks of 5.1-9.4 cm lie within 0.6 m of the path.

What the data supports:
1. The wheel-ground model climbs a clean slope of the site's grade. With the same rover, plugin, friction and commands,
   a smooth 12.4 deg incline gives 0.938 with 6 % slip. Smooth inclines pass the 80 % rule up to 19 deg.
2. On the natural slope the rover slips 29-35 %, as much as on the smooth 21 deg incline (28 %). It pitches to 20-23
   deg, is airborne 1.7-3.7 % of the time (smooth inclines: at most 0.13 %) and drifts 0.8-1.3 m sideways.
3. Steepness alone does not explain the shortfall. The smooth 17-19 deg inclines, the range of the site's steeper
   sections, still give 0.84-0.90. The rest comes with the roughness.
4. Limits of the evidence:
   - The smooth ramps also have few, large collision triangles, and fix_build.md 2 measured worse climbing on finer
     collision meshes of the same relief. The data cannot separate the slope's roughness from Bullet's wheel contact
     on a finely triangulated rough mesh.
   - The stock plugin (4 ms step, different mesh friction) climbed this slope to 0.770 in 1 run, also below 0.80.

So the shortfall comes with the rough natural slope, not from a general limit of the wheel-ground physics.
- Sizeable slip on a rough 12 deg regolith slope is plausible for a real rover. The DESIGN criterion assumed a cleaner
  slope.
- Earlier builds' slope sites gave 0.83-0.90 with the shipped collision (gz_validation 5, fix_runtime 1.3, fix_build 6).
- For the user to decide: either accept the result and qualify the slope criterion for rough sites, or move the slope
  site to a smoother run (candidates in section 7, not driven).

## 3. Flat: the default site and two level spots

| spot | pose (x, y, yaw) | grade over the 3 m corridor | footprint slope along it | rocks ≥ 5 cm in the corridor | distance (of 3 m) | ratio to the traction-limited distance | steady speed | yaw tracking | airborne |
|---|---|---|---|---|---|---|---|---|---|
| default `flat`, 3 runs | -2.0, -4.5, -22.5 deg | -7.4 deg (downhill) | 4.9-9.4 deg | 2 | 0.950-0.955 | 0.999-1.005 | 0.997-1.005 | 0.968-1.016 | 0-1.67 % |
| level A, 1 run | -25.5, -4.0, 0 deg | -3.8 deg | 3.6-4.5 deg | 0 | 0.964 | 1.020 | 1.020 | 0.976 | 0.33 % |
| level B, 2 runs | -37.0, -1.5, -112.5 deg | +0.3 deg | 2.4-3.3 deg | 0 | 0.940 / 0.940 | 1.005 | 1.007-1.008 | 0.929-0.932 | 0 |

The level spots come from final_build.md 11.2, and the terrain numbers from `sites_extra.json`.
- All three spots pass every flat criterion.
- The distance follows the grade, as the traction limit predicts (mu 0.4 at 1.62 m/s²; 0.999-1.020 of the
  prediction). On level ground (B) it is 0.940, because the traction-limited distance there is 0.935 of 3 m. The 0.93
  rule therefore has only 0.010 margin on level ground. The default site passes with more margin because its corridor
  runs downhill.
- Yaw tracking is 0.93-1.02 at the three flat spots, against 0.889 at the turn site.
- No change to the flat site is needed for acceptance (the open decision in DESIGN 11 / final_build.md 11.2). Phase 3's
  0.928 failure was on a corridor rising 3.3 deg.

## 4. Stock plugin comparison

`apollo16_core_stock.sdf` (stock Bullet, 4 ms), one launch per site (`EXP/runs/robot_stock/`, role comparison):

| check | patched (default, 3 runs) | stock (1 run) |
|---|---|---|
| flat: distance / steady speed | 0.950-0.955 / 0.997-1.005 | 0.982 / 1.035 |
| flat: in-place yaw tracking (≥ 0.85) | 0.968-1.016 | **0.396** (88 of 206 deg) |
| turn site: yaw tracking (≥ 0.85) | 0.888-0.889 | **0.579** (126 deg) |
| slope (≥ 0.80) | 0.649-0.700 | 0.770 |
| rock (≥ 0.80) | 0.969-0.985 | 0.991 |
| RTF while driving | 0.929-0.996 | 1.000 (a quarter of the steps) |

- The stock plugin fails both in-place turns (0.40 / 0.58). It measured 0.45 in phase 3, and DESIGN 10 records
  0.45-0.66.
- **The patched plugin is still required.** The launch's fallback to the stock variant (DESIGN 12) keeps the world
  running but loses in-place turning.
- Stock physics also differs in step size and in mesh friction (not taken from the SDF; DESIGN 10, patch 2), so its
  slope and forward numbers are not like-for-like.

## 5. LiDAR

### 5.1 World-only bench (`tools/bench_world.py`, flat site, `real` preset, 40 s window)

| item | final world | phase 3 (gz_validation 3) | DESIGN 13 | verdict |
|---|---|---|---|---|
| sensor | (-1.630, -4.653), yaw -22.5 deg, 0.449 m above the final 1 cm heights (the resting rover's LiDAR at the flat site) | (-4.5, -0.5), yaw 135 deg | | |
| load | first `/stats` 2.16 s; first cloud **27.48 s** | 19.57 s | ≤ 30 s | PASS (2.5 s margin) |
| RTF | **0.9998** (40.2 s, 40,032 iterations), field min 0.977 | 0.9930 | ≥ 0.95 | PASS |
| server RSS | 3,559 MB at load, **3,688 MB peak** | 2,742 MB peak | ≤ 5 GB | PASS |
| renderer | `libd3d12.so`, `libd3d12core.so`, `libdxcore.so`, `libgallium-25.2.8` mapped; no llvmpipe | same | D3D12 | PASS |
| server log | 0 `[Err]`, 0 `[Wrn]` | 0 / 0 | no errors | PASS |
| GPU (whole GPU, nvidia-smi) | idle 1,717 MiB -> 3,878 MiB in the window; util 32.7 % mean | 4,288 MiB (idle 2,651) | | info |
| LiDAR | 20.0 Hz sim (19.997 Hz wall), 25,498 points, 13,739 finite | 20.0 Hz, 14,561 finite | | info |

The world now has 17.13 M visual triangles, against 9.53 M in phase 3 (DESIGN 13a).

Captures (`tools/lidar_capture.py`, `real` preset):
- 18 static poses: the 6 phase-3 poses, the rover LiDAR pose at the 4 sites, and 8 auto poses 25-35 m from the centre;
- 3 servers of 6 sensors each, 3 clouds per pose;
- first clouds after 26.6-27.2 s, server RSS 3,688-3,714 MB, 0 `[Err]` / 0 `[Wrn]`, D3D12.

### 5.2 Package checks (`lunar_world_gen.validate.lidar_checks --auto-inputs`, first cloud per pose)

| check | 18 poses (`reports/lidar_checks.json`) | 6 phase-3 poses (`EXP/analysis/lidar_checks_p3poses.json`) | phase 3 (same 6 poses) |
|---|---|---|---|
| returns (fraction of rays) | 260,369 (0.567) | 86,131 (0.563) | 86,189 (0.563) |
| returns per scan, 0-2 / 2-4 / 4-8 / 8-16 / 16-32 / 32-64 / 64-150 m | 9,170 / 2,182 / 1,016 / 610 / 578 / 457 / 453 | 9,086 / 2,132 / 1,061 / 804 / 746 / 299 / 229 | 9,075 / 2,197 / 1,033 / 785 / 746 / 300 / 229 |
| beyond the ring | 0 (farthest 167.7 m Chebyshev, ring 250 m) | 0 | 0 (167.5 m) |
| ground residual std, 0-2 / 2-4 m | 4.9 / 8.2 mm | 6.8 / 9.8 mm | 6.1 / 13.1 mm |
| below ground | 0 | 0 | 0 |
| floating | 4 (1.5e-5; limit 1e-3) | 1 | 2 |
| leak candidates | 0 | 0 | 0 |
| rock detection: judged bins consistent | 7 of 7 | **4 of 5** | 7 of 7 |
| package verdict | **5 / 5 pass** | 4 / 5 | 5 / 5 |

**Rock detection, judged bins (18 poses).** Rule: returns / ideal must lie in [0.4, 1.6], and p_detect must be at
least 0.8 where at least 5 ideal returns are expected.

| rock size | range | visible rocks | p_detect | returns / ideal |
|---|---|---|---|---|
| 3-5 cm | 0-2 m | 356 | 0.83 | 0.68 |
| 5-10 cm | 0-2 m | 108 | 0.99 | 0.58 |
| 10-20 cm | 0-2 m | 11 | 1.00 | 0.44 |
| 10-20 cm | 2-4 m | 50 | 0.92 | 0.49 |
| 20-50 cm | 2-4 m | 5 | 1.00 | 0.41 |
| 20-50 cm | 4-8 m | 24 | 0.92 | 0.59 |
| 1-3 m | 16-32 m | 6 | 1.00 | 0.63 |

- The ratios are 0.41-0.68 (phase 3: 0.53-0.69), and two of them are near the 0.4 floor.
- On the 6 phase-3 poses alone, the 10-20 cm / 0-2 m bin (6 rocks) gives 0.389 and fails the floor (phase 3: 0.529 with
  9 rocks). The gazebo cube tests gave 0.34-1.3.
- In the ray cast (5.3), 98-99 % of the beams that hit 10-20 cm rocks within 2 m return at the same range, and all 13
  such rocks are returned. The low ratio is therefore not lost returns. It points to the check's idealised rock model,
  which expects more beams than the rendered rocks present.
- Small rocks are seen as in phase 3. p_detect at 0-2 / 2-4 m on the same 6 poses:
  - 2-3 cm: 0.60 / 0.13 (phase 3: 0.58 / 0.16);
  - 3-5 cm: 0.80 / 0.27 (0.82 / 0.36);
  - 5-10 cm: 1.00 / 0.65 (0.96 / 0.70).

**Floating returns (4 of 260,369).**
- **One in capture 005** (fresh-crater pose; range 22.2 m, +0.30 m above the ground). It lies 0.36 m from the centre of
  a 0.62 m ring rock (exposed 0.37 m), just outside its radius. It is a hit on the rock's flank outside the check's
  footprint, like phase 3's two.
- **Three in capture 006** (flat-site LiDAR pose; range 7.2 m; +0.12 / +0.13 / +0.19 m). They lie within 8 cm of each
  other near (2.27, -10.65).
  - The five nearest catalog rock centres are 2-3 cm pebbles (exposed at most 1.6 cm), 0.09-0.25 m away. These are too
    small to give returns this high.
  - The cause was not determined. Whether a larger rock reaches this point from farther away was not checked.

### 5.3 Further LiDAR checks (EXP scripts, not package checks)

- **Exact ray cast** (`lidar_raycast.py`). Every beam within 6 m was cast against the rendered ground TINs and the merged
  rock meshes. A beam matches when its range is within 5 mm + 3 sigma of the depth-quantisation model.
  - Ground beams match 99.9 % (0-1 m), 99.5 % (1-2 m), 95.5 % (2-4 m) and 88.7 % (4-6 m). The range error is unbiased
    (median 0.02 sigma, 98.2 % within 3 sigma).
  - Of 216,069 ray-cast hits, 24 had no LiDAR return (0.01 %). 316 LiDAR returns hit no mesh (0.15 %).
  - Share of the rocks hit by the ray cast that the LiDAR also returned, at 0-1 / 1-2 / 2-4 / 4-6 m:
    - 2-3 cm: 99 / 91 / 64 / 43 %;
    - 3-5 cm: 100 / 97 / 78 / 52 %;
    - 5-10 cm: 100 / 98 / 91 / 70 %;
    - 10-20 cm: 100 / 100 / 100 / 91 %;
    - 20 cm and larger: all 20 returned.

    Beyond 2 m the LiDAR misses part of the small rocks that an exact ray cast hits.
- **Frame stability** (`lidar_extra.py`, 3 clouds per static pose, all 18 poses). 0 rays changed between hit and no-hit,
  and the largest range change was 0. There is no flicker and no z-fighting.
- **Tile seams.** Core ground returns within 1 cm of a 10 m tile seam have a median of +0.07 mm and a std of 3.1 mm
  (within 4 m). That is the same as away from the seams (3.0 mm). None within 0.5 m of a seam falls outside the
  threshold, so there are no cracks or steps.
- **Core/ring edge.** There is no step: band medians are -0.7 to +4.2 mm within 5 m on either side. The spread is larger
  on the ring side (std 29-71 mm), whose height source is the 25 cm ring grid.
- **Micro-relief within 4 m** (703 cells of 0.5 m). The LiDAR relief and the 1 cm heights correlate at r = 0.87. Their
  RMS is 5.6 and 5.0 mm, and the RMS of their difference is 2.8 mm.

## 6. Known issues carried over from e2e_launch.md

| issue | e2e_launch.md | this validation |
|---|---|---|
| creep at zero command | 2-3 mm/s on 4-5 deg slopes | 1 s pre-roll at zero command: 4.4 mm/s at the flat site (rover pitched 5.5 deg), 2.8 mm/s at level spot A (4.1 deg), 0.1 mm/s at level spot B; not investigated |
| stall in a tight forward pivot | one 25 s stall at v 0.26 m/s, ω -0.49 rad/s (radius 0.53 m) | not reproduced (the arcs test did not run); in-place turns track 0.89-1.02 |
| RTF with the GUI open | 0.82-0.93 (no GUI 0.93) | headless while driving 0.929-0.996: DESIGN's ≥ 0.9 holds headless, not always with the GUI open |

## 7. Not done (the user's decision: not necessary now)

- **Remaining diagnostics:**
  - CAD-collision comparison at the slope (`robot_collision:=original`): launched at 23:51 and stopped at start, no data.
  - Alternative slope sites. `sites_extra.json` found two smoother candidates, not driven:
    - (22.5, 30.5, yaw 112.5 deg): 12.5 deg over 5 m, line-fit RMS 27 mm, sub-slopes 6.2-13.8 deg;
    - (-10.0, 3.0, yaw -157.5 deg): 10.3 deg, RMS 25 mm, sub-slopes 5.2-11.8 deg.
  - A 30 s climb, to see whether the rover reaches the top of the 5 m run.
  - Spawn probes (the settle from the first pose on).
  - Arcs on level ground, including the 0.53 m pivot.
- **Visual review** (DESIGN 13 "Visual", `tools/render_views.py`): deferred until camera work starts.
- **Not produced:**
  - figures (`EXP/figs/` is empty);
  - output from the per-launch wheel-gap analysis (`analyze_robot.py`) and from `slope_analysis.py` (both written,
    never run);
  - a new `world_dir/reports/validation_report.md` (it is still the 2026-10-25 version).

  No tests were run, since no package file changed.

## 8. Reproduction

Run in WSL from `src/lunar_world_gen`. The scripts in `EXP/scripts/` set `GZ_PARTITION`, `ROS_DOMAIN_ID` and
`ROS_HOME` (`common.sh`), log the machine state, and wait until no other `gz sim` runs.

```bash
EXP=/home/nargon/togo_assets/lunar/experiments/rover/final_validation
bash $EXP/scripts/run_robot_main.sh > $EXP/logs/robot_main.log 2>&1   # 12 launches (3 per site) + stock series (4)
bash $EXP/scripts/cpu_prep.sh                                         # results -> reports/, sites_extra.py, incline world
bash $EXP/scripts/run_lidar.sh > $EXP/logs/lidar.log 2>&1             # bench, 18-pose captures, lidar_checks
bash $EXP/scripts/cpu_lidar.sh                                        # 6-pose checks, lidar_extra.py, lidar_raycast.py
STEPS="D1 D2" bash $EXP/scripts/run_diag.sh > $EXP/logs/diag.log 2>&1 # level spots A/B, smooth inclines 10-21 deg
```

The calls inside them:

```bash
SYS=/usr/bin/python3; CFG=config/apollo16_core.yaml; C="--partition claude_rover --domain 132"
W=~/togo_assets/lunar/worlds/apollo16_core; R=$W/reports
$SYS tools/run_robot_tests.py --out $EXP/runs/robot_main --world $W/apollo16_core.sdf \
    --sites flat,slope,rock,turn --repeats 12 $C
$SYS tools/run_robot_tests.py --out $EXP/runs/robot_stock --world $W/apollo16_core_stock.sdf \
    --sites flat,slope,rock,turn --repeats 4 --role comparison $C
$SYS tools/run_robot_tests.py --out $EXP/runs/flat_levelB --world $W/apollo16_core.sdf --sites flat \
    --site-pose flat:-37.0000,-1.5000,2.4293,-1.963495 --repeats 2 --role comparison $C  # A: -25.5000,-4.0000,1.5048,0.000000
$SYS tools/run_robot_tests.py --out $EXP/runs/incline_12.4 --world $EXP/worlds/inclines/inclines.sdf --sites slope \
    --site-pose slope:-0.8000,7.0000,0.1760,0.000000 --repeats 1 --role comparison $C  # ramp k at y = 7 k
$SYS tools/bench_world.py $W/apollo16_core.sdf --out $R/bench_world_flat_real.json \
    --pose-agl=-1.6305,-4.6531,0.449,-0.392699 --lidar real --window 40 --config $CFG $C  # '=': value starts with '-'
$SYS tools/lidar_capture.py $W/apollo16_core.sdf --out $R/lidar --poses-file $EXP/analysis/capture_poses.json \
    --config $CFG --lidar real --batch 6 --clouds 3 --skip 2 $C
~/.venvs/lunar_world/bin/python -m lunar_world_gen.validate.lidar_checks $R/lidar/capture_*_0.npz \
    --config $CFG --auto-inputs --out $R/lidar_checks.json
```

`$EXP/scripts/make_incline_world.py` builds the incline world, and `$EXP/scripts/sites_extra.py` produces the site
terrain numbers.
