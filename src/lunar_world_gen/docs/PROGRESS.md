
# Lunar world: progress log

The Apollo 16 lunar world is built by the generator in this folder. This log records what was done, what was measured, and the decisions made along the way. Newest entries are at the bottom.

## Ground rules (agreed with the user, 2026-10-25)

- Everything is local. No git operations of any kind.
- Existing files are not modified: the old generator, `lunar_surface.world`, the robot description, the launch files and the SLAM configs all stay as they are.
- Generated assets go to WSL's own disk at `~/togo_assets/lunar/`, not OneDrive. Only scripts, configs and docs live in this repo folder.
- This folder contains `COLCON_IGNORE`, so colcon builds in the native WSL workspace and in Docker skip it.
- The sim LiDAR stays as it is. A realistic Robin W mode comes later.
- The new world runs through the existing launcher by setting `TOGO_WORLD` to an absolute path. `start_world.launch.py` joins the path with `os.path.join`, so an absolute path is used unchanged.

## 2026-10-25: environment

- WSL venv `~/.venvs/lunar_world` contains numpy, scipy, rasterio, pyproj, trimesh, pydelatin, pyfqmr, numba, pygltflib, matplotlib, pyyaml, tqdm, pytest and rosbags. All were checked with a smoke test: reading the DTM, building a TIN with pydelatin, numba, GLB export and pyfqmr.
- Headless Blender 5.2.1 works from the command line. Cycles sees the RTX 2080 through OptiX and CUDA, and the glTF exporter is available.
- Headless Gazebo publishes GPU LiDAR point clouds at a real-time factor of about 1.0. GPU rendering needs `GALLIUM_DRIVER=d3d12`; without it, Gazebo silently falls back to llvmpipe.

## 2026-10-25: phase 1, research and capability tests (done)

The reports are in `docs/research/`. The findings that shaped the design:

**Site**
- The core is site F1 at lat -8.973448, lon 15.511126, 300 m east of the LM, with href 8.1955 m.
- It covers the west half of a degraded ~180 m bowl, with a fresh 23 m crater at its south edge.
- The DTM resolves only about 10 m. Above that the real data is used; below it the detail is synthetic.

**Science** (the generator's target numbers)
- Roughness: nu(L) = 0.069 m * L^0.8, which means RMS slope is 10.4 degrees at 1 cm, 4.0 at 1 m and 2.5 at 10 m.
- Craters: N(>=D) = 0.05 D^-2, with about 70% degraded.
- Rocks: N(>=D) = 3.78e-4 D^-2.8 per m^2, which gives about 216k rocks of 2 cm and larger in the 100 m core.
- Regolith colour (0.12, 0.107, 0.088); ground mu about 0.4; gravity 1.62 m/s^2.

**Gazebo**
- GLBs must be raw Z-up with an explicit material (metallicFactor 0).
- A single missing mesh aborts the whole world load.
- Back faces are culled for both the LiDAR and the camera.
- Coincident tile seams leak no LiDAR rays; small gaps and T-junctions leak 0.02-0.2% of ground returns.
- The gpu_lidar's range is quantised, with height errors of about 5 mm at 2-4 m and about 24 mm at 8-16 m.
- Visibility flags, PBR materials and CUSTOM_SURFACE spherical coordinates all work.

**Rover**
- The lunar turning problem is a classic-Bullet plugin bug, not real traction. `SetJointVelocityCommand` acts as a yaw damper, so in-place turns reach only 13% of the commanded rate on flat ground at 1.62 m/s^2. A 12-line patch restores 93-95%.
- Classic Bullet ignores SDF friction on mesh collisions.
- The controller spawners fail when spawning the robot stalls the sim while its render scene loads.

**Performance**
- The real-time factor stays at about 1.0 up to 40M triangles for both LiDAR settings. The cost of a bigger world is load time and memory: as tiles, 20M triangles load in about 19 s and use about 2.5 GB.
- Rocks must be merged into the tile meshes: 10k separate rock visuals drop the real-time factor to 0.03.

**Legacy**
- The old world's slopes are 1.39x too steep, its craters 2.5-4x too deep, and the heightmap script is broken.
- The `slipmap` files are not slip maps.

**Incident.** The PC went to sleep at 07:20 and was woken at 10:55. That interrupted one 40M-triangle stress benchmark; nothing was lost. Since then a temporary keep-awake process (SetThreadExecutionState, auto-expires after 18 h) and an idle WSL session keep the machine and WSL running while jobs execute. Both are stopped at the end of the work.

## 2026-10-25: design (done)

- `docs/DESIGN.md` defines:
  - the frames and the 1 cm lattice;
  - the zones (core 100 m at 1 cm, ring 500 m at 25 cm, far 2.2 km at 2 m, horizon 20 km at 20 m);
  - the single height function H = B + w_ring (coarse fBm + craters of 1 m and larger) + w_core (fine fBm + small craters) + fillets;
  - the crater profiles, rock population and placement, mesh stitching (shared boundary vertices), GLB rules, collision, the SDF, the physics plugin, the launch sequencing, and the acceptance checks.
- Foundation modules: `grid.py`, `config.py`, `layout.py` and `mesh/types.py`.
- Default config: `config/apollo16_core.yaml` (with presets `full` and `fast`).

## 2026-10-25: performance study (done, about 12:00)

`docs/research/performance_budget.md` covers 74 headless runs plus 3 full robot launches. The budgets now in DESIGN.md section 13a:

**Visual**
- Core terrain as 100 tiles of 10 m, at most 200k triangles per tile and 8-10 M in total.
- Rocks merged into the tiles: at most 6 M in total.
- Total: at most 16 M, hard cap 20 M.

**Collision**
- About 0.25-0.5 M triangles in per-tile meshes (DESIGN section 7 updated: per-tile instead of one mesh).
- Rock collision is baked into these meshes, never per-rock entities.

**Load time and memory**
- Load: first LiDAR cloud after about 2.95 + 0.81 * T s, where T is millions of triangles.
- Memory: 0.33 + 0.107 * T GB for the server. The GUI client needs the same again.

**Robot launch with the stock chain**
- Spawning stalls the sim for about 3.2 + 1.2 * T s.
- At 20 M triangles, a spawner exited with code 1 even though the controller was active.
- This confirms the need for the launch sequencing in DESIGN section 12.

## 2026-10-25: phase 2, implementation (done, about 12:25)

All five owners finished, and 117 unit tests pass (terrain 23, rocks 26, mesh 21, world 26, validation 21). The details are in `docs/impl/<owner>.md`.

**Terrain**
- H0 is one global height function. All zone boundaries are bit-identical.
- A full build takes 1:57 and 1.46 GB, and it is deterministic (identical SHA-256 on rebuild).
- `fbm_gain` was calibrated to 0.278, because craters already supply part of the small-scale roughness.
- Crater counts are within 5% of the target.

**Rocks**
- The production library `rocks_v1` has 50 prototypes x 3 LODs, a 4096^2 atlas at 90% fill, and 64-vertex hulls.
- A full core has 215,678 rocks against 216,077 expected, at 6.6 M triangles.
- On real tiles no rock floats.

**Mesh**
- The TIN is error-bounded, and stitching proves 0 mismatches, 0 T-junctions and Euler characteristic 1 on real data.
- The GLB writer has been checked with trimesh.
- Regolith textures tile seamlessly.
- In Gazebo the OpenGL normal maps are right with no flip, the SDF material overrides the GLB's own, and the 4-tile seam shows no crack.

**World**
- The SDF writer produces three variants that pass `gz sdf -k`.
- Spawn-site search works.
- The patched plugin (yaw-damper fix plus mesh friction) passes the incline test 8/8 against prediction. In-place turns at 1.62 m/s^2 track **0.973** of the command; the stock plugin at 4 ms reaches 0.66.
- The new launch file came up cleanly in 17 of 17 launches, including the old heavy world, where the stock chain failed 2 of 2.
- The run script is tested headless.

**Validation**
- The static checks, the world bench, LiDAR capture and checks, robot tests and the report are all built and cross-checked against the earlier agents' numbers.

**Contract changes (accepted)**
- B is one nested global function.
- The far field is also low-passed.
- Rock intensity is normalised to mean 1 over the core.
- Prototypes are chosen within a factor 3 of the rock size.
- Rocks are placed by contact with the ground.
- Textured materials use diffuse 1.
- A grey `rock_atlas_roughness.png` is added.
- The launch file spawns the robot at about 2 s and gates the spawners on the LiDAR topic, with timeouts of 60 s (service call) and 30 s (switch).

**Decisions** (DESIGN.md section 13, top)
- Roughness acceptance is site-aware: the real DTM governs at 10 m and above, and the synthetic part is judged at 1-3 m. The real site is rougher than average, so the detail generators were not retuned.
- The forward-drive criterion uses steady-state speed, with a distance threshold of 0.93, because the start is traction-limited at lunar gravity.
- The turn site requires no rocks of 10 cm or larger within 1.5 m.

## 2026-10-25: phase 3, integration build and validation (done, about 14:30)

**Build** (`docs/impl/integration.md`)
- `python -m lunar_world_gen.build --preset full` runs eight cached stages, 2:21 from the cached terrain.
- The world in `~/togo_assets/lunar/worlds/apollo16_core` has:
  - 9.53 M visual triangles: core ground 1.73 M at 5 mm, core rocks 6.59 M, ring/far/horizon 1.21 M;
  - 0.90 M collision triangles: core 0.487 M after raising the tolerance to 1.5 cm;
  - 215,678 core rocks and 2,946 ring rocks.
- Static checks: 8/8 pass.
- Tests: 139 pass.
- The build is deterministic: a rebuild into another folder gave identical content.

**Gazebo validation** (`docs/impl/gz_validation.md`): 23/25 checks pass.
- World only: first LiDAR cloud after 19.6-23.3 s, RTF 0.99-1.0, 2.7 GB, 0 errors, D3D12 confirmed. The Robin-W-like 920x195 preset also runs at RTF 1.0.
- LiDAR: no returns beyond the ring, 0 leaks, rock detection consistent with the gazebo tests.
- Robot:
  - 8/8 launches came up cleanly;
  - the slope was climbed (11.4 degree mean);
  - the rock was traversed at 99.7%;
  - turn tracking was 0.95.
- Failed: **RTF while driving**. RTF drops to 0.05-0.2 on slopes. The robot's CAD collision meshes (motors 46.6k triangles each, LiDAR 174k) go through GImpact tests against large tilted terrain collision triangles. With a flat GImpact grid or a box at the same site, RTF was 0.94-0.97.
- Failed: **flat drive distance**, 0.928 against 0.93. The rover reached 99.7% of the controller-limited command, so this is a criterion artefact.

**Reviews** (`docs/impl/review_{fidelity,code,usability}.md`): 44 findings, 1 high, 13 medium, 30 low.
- The high finding is U1: the DLIO gravity parameter has to change to 1.62 for this world, and nothing documented it.

**Interruption.** The user's usage limit stopped the visual stage and all 28 finding verifiers (about 14:40, reset at 15:50). The work resumed at 16:02. Instead of re-running 28 separate verifiers, the fix agents now verify each finding by reproducing it before fixing.

**Cleanup.**
- Deleted 30 GB of WSL crash dumps from `%TEMP%\wsl-crashes`: ten `ruby3.2` dumps from 13:55-14:29, all caused by gz segfaulting at SIGTERM shutdown in our test runs. The fix is to SIGKILL the gz servers; from now on tools and scripts never send SIGTERM to gz.
- Deleted two GDAL `.aux.xml` sidecars that research agents had created next to the user's DTM at 05:18 (U12). Nothing outside `src/lunar_world_gen` remains modified.

## 2026-10-25: phase 4, fix round (running)

Five work packages run in parallel, each verifying findings by reproducing them first:
- **rocks:** FID-01, 02, 04 (rock part), 09, 11, 12;
- **terrain:** FID-03, 04 (fade), 05, 06, 16, 17, CR-07;
- **build:** the RTF collision triangle-size cap, rock recontact on the rendered TIN (CR-05, FID-08, FID-10), CR-01 to CR-04, CR-06, CR-08, CR-09, CR-11 to CR-13, FID-07 with a 3 mm visual tolerance, the `inspect_stock` variant, and SIGKILL in the tools;
- **runtime:** a simplified robot-collision launch option (the primary RTF fix), U4/CR-10, U5, U6 wiring, U7, U9, U10, U11, `env.sh`, and the U3 investigation;
- **docs:** U1 (a lunar-gravity copy of the DLIO config), README, requirements, GUI config (U6), USER_GUIDE (U13), FID-13/14/15 notes.

Config changes are proposed by the agents and applied centrally.

### Results (done, about 18:10)

Every assigned finding was reproduced before it was fixed. The reports are `docs/impl/fix_{rocks,terrain,build,runtime,docs}.md`. The suite reached about 266 tests during the round.

**Rocks**
- Perched rocks now rest on a stable facet with three contacts. Rocks with a central gap above 0.1 D went from 80% to 0%.
- Debris enhancement around craters follows a smooth, size-gated profile.
- Interpenetrations involving rocks of 5 cm and larger went from 1,520 to 0.
- Fillets are wedges.
- Burial follows the young-debris calibration.
- A 6 m band of rocks under 30 cm continues outside the core (+0.53 M triangles).

**Terrain**
- The talus limiter brought the maximum slope from 47.6 to 33.5 degrees, with no area above 35 degrees.
- Rim crests are exact (0.030 D).
- Craters vary in shape.
- The fBm has two gains, 0.109 coarse and 0.578 fine; all site-aware roughness criteria pass.
- The core fade is 3 m.
- The horizon nodata fill is textured.
- `dtm_extract` no longer depends on chunk size.
- The FFT gives identical bits for any thread count.

**Build**
- Rocks are re-seated on the rendered TIN: 0 hovering and 0 hidden, where there were 11k and 7.8k.
- Caching, locking, budgets and preset directories are fixed.
- Merging rocks peaks at 28 MB instead of 812 MB.
- At 3 mm, a seam band removes the crease lines.
- An `inspect_stock` variant was added.
- The tools stop gz with SIGKILL.
- Capping the collision triangle size makes slope climbing worse (climb 0.83 fell to 0.34-0.73), so the cap is OFF.

**Runtime**
- `robot_collision:=simplified` (the default) replaces the 368k-triangle CAD collisions with 88 boxes. Slope RTF rose from 0.13 to 0.66-0.70 under heavy contention, and the build agent measured 0.90 on its world.
- Ctrl+C now takes 1.2-2.2 s and writes no crash dump.
- Added: a READY banner, a plugin dlopen check with fallback, argument validation, and `env.sh`.

**Docs**
- README, requirements file, `config/slam/seyond_dlio_graph_lunar.yaml` (the user's file with only `odom/gravity` set to 1.62), GUI config (far clip 25 km), and USER_GUIDE.
- U1 was quantified: with 9.82 in the config, DLIO stores a -8.2 m/s^2 accel bias and sees a phantom 1.4 m/s^2 horizontal acceleration at a 10 degree tilt.

**Config applied centrally (18:15)**
- Terrain: `fbm_gain` 0.109, `fbm_gain_fine` 0.578, `edge_fade_m` 3.0, talus, crater rim and shape keys, wedge fillets.
- Rocks: burial [0, 1], tilt 8 degrees, rest/settle/bedding/overlap/edge band/contact keys, new crater_enhancement.
- Mesh: 3 mm core with seam band 0.08 / 0.06; collision max edge null.
- Budgets: soft and hard limits.
- Lighting: earthshine 3e-5.
- Physics: `rock_mu` removed.

## 2026-10-25: phase 5, final integration, validation, visual review and docs

**Final integration (done, 19:10; `docs/impl/final_build.md`)**
- Cross-package fixes: ring sampling avoids core rocks; fillet rows use `fillet_mask`.
- FID-08: ring rock collisions get a footprint dilation; rocks fully covered went from 0.2% to 99.7%.
- U8: the plugin build no longer leaves files in the caller's folder.
- Bugs found and fixed: an `annulus_R` crash, and stale Gazebo results mixed into the report.
- 273 tests pass.
- The full rebuild takes 7:18, and a second rebuild is byte-identical.
- 8/8 static checks pass, and all 4 SDF variants pass `gz sdf -k`.

**The final world**
- 17.13 M visual triangles: core ground 8.74 M at 3 mm, core rocks 6.58 M, plus the ring band. This is over the 16 M soft target and under the 20 M hard cap.
- 0.81 M collision triangles.
- 215,677 core rocks and 37,029 ring rocks.
- LiDAR bench: first cloud at 27.0 s, RTF 1.0, 3.7 GB.

**Interruption.** The usage limit (about 20:00, reset at 21:00) stopped the validate, visual and docs agents.

**User's first live look (21:20-21:50)**
- *GUI did not appear.* The Gazebo window existed in both X and Windows, but WSLg was in '[WARN:COPY MODE]': WSL had been started by the background keep-alive after the morning's sleep and resume. After `wsl --shutdown`, starting the sim headless and then opening the GUI worked. The user asked for the script to do exactly that.
- *Memory leak found.* After loading, the GUI sits at about 3.5 GB and the server at about 3.7 GB, but both keep growing: about 6-10 MB/s for the GUI and about 3.5 MB/s for the server with the robot's LiDAR. After about 10 minutes WSL had 135 MB left (GUI 9.05 GB, server 6.0 GB), so the user was asked to stop. The old 30 m world stays flat at 620 MB, so the leak is triggered by something in the new world.

## 2026-10-25: phase 6, leak diagnosis and headless-first script (done, about 23:55)

**Leak (`docs/impl/leak_diagnosis.md`)**
- The cause is a bug in Mesa's d3d12 driver, the OpenGL-on-Direct3D 12 layer that WSL uses. The world itself is not the cause.
  - `d3d12_get_cmd_signature()` searches its cache with `&key` instead of `key`. Every indirect draw therefore creates one `ID3D12CommandSignature` and leaks it, about 2.7 kB of heap each.
  - The bug is in Mesa 25.2.8 (installed) and is still there in 26.2.5 (latest stable).
- Ogre-Next draws everything with `glMultiDrawElementsIndirect` whenever `GL_ARB_multi_draw_indirect` is available. In this world that is about 54 indirect draws per GUI frame. The resulting growth:
  - GUI: 5.5-6.3 MB/s, and about 10 MB/s in the user's session.
  - Server rendering the 20 Hz LiDAR: 2.7 MB/s.
  - The old one-mesh world: 0.46 MB/s. Its RSS only starts rising after about 165 s, which is why it looked flat before.
- **Fix:** `MESA_EXTENSION_OVERRIDE=-GL_ARB_multi_draw_indirect`. It was verified for 7 minutes with the server, LiDAR and GUI together:
  - GUI -0.009 MB/s and server 0.000 MB/s;
  - screenshots pixel-identical and LiDAR ranges identical to before;
  - 37-41 fps, RTF 1.0, LiDAR 20 Hz.
- **Where the fix is applied** (WSL sim only, at the user's request):
  - `scripts/env.sh`, which the wrapper and `open_gui.sh` source;
  - `tools/bench_world.py` `gz_env()`, which all the tools use;
  - the user's `src/togo_sim_wsl.sh`.

  It is not in `~/.bashrc`. The env test now covers it, and 47/47 launcher tests pass.
- **RAM after the fix** (bare world + LiDAR, default GUI config): GUI about 3.5 GB and server with LiDAR about 3.5 GB, both flat. The GUI's share by content is about 0.5 GB base, 1.0 GB ground and 2.1 GB rocks (measured with ground-only and rocks-only variants). Of the rock share, the 2-5 cm pebbles are an estimated 1.5 GB, from their roughly 70% share of the rock triangles.

**Headless-first launcher (`docs/impl/leak_script.md`)**
- `run_apollo16_sim.sh` now starts the server and robot without a GUI, waits for the launch's ready marker, then opens the GUI.
- New options: `--headless`, `--gui-early`, `--ready-timeout` and `--keep-crash-dumps`.
- New script `scripts/open_gui.sh` opens or closes only the GUI; `--close` closes it.
- At exit, the scripts delete the WSL crash dumps of their own gz and ruby processes.
- So far this has been tested only with test doubles (121 tests), not with real Gazebo.

## 2026-10-26: phase 7, final validation (done on 2026-10-27)

Started at 00:01 on 2026-10-26 and stopped at 00:11 (the user called it a day); restarted at about 21:55 and finished after midnight. Reports: `docs/impl/e2e_launch.md`, `docs/impl/final_validation.md`, `docs/impl/docs_final.md`. No package code was changed by the validation.

**Results** (final world)
- **Launcher, 22 minutes with the GUI** (driving, a camera tour 1-13 km out, close-ups 250-1270 m away): memory flat. GUI 3.69-3.76 GB (+0.4 MB), server with its nodes 3.87-3.88 GB (+2.1 MB), WSL about 8.0 GB (4.4 GB without the GUI); before the fix the GUI grew 10 MB/s. READY 37-47 s in 15 launches, GUI terrain 35-41 s after READY, Ctrl+C 2-3 s with no crash dump. `--headless` + `open_gui.sh` (refusal of a second GUI, Ctrl+C, reopen, `--close`) worked.
- **SLAM inputs:** LiDAR 20 Hz in `lidar3d_0_laser`, IMU 100 Hz in `imu_0_link` reading 1.62 m/s^2 at rest, static TF equal to the lunar DLIO config, no `odom -> base_link` from the sim. The cloud has no per-point time field and intensity 0.
- **Acceptance (DESIGN 13):** world-only and LiDAR 7 / 7 (first cloud 27.5 s, RTF 1.0, 3.69 GB; 18 LiDAR poses: 0 returns beyond the ring, 0 leaks, rock detection 7 / 7 bins). Robot 9 / 10 in 12 headless launches: controllers 12 / 12, RTF while driving 0.93-1.0, flat 0.950-0.955, rock 0.969-0.985, turn site 0.888-0.889. **The slope fails:** 0.649-0.700 against 0.80 on the rough natural slope; a smooth 12.4 deg incline with the same physics gives 0.938. The stock plugin fails the in-place turns (0.40 / 0.58), so the patched plugin stays required.
- With the GUI open the RTF is 0.82-0.92 (0.93 without): run SLAM headless.

**Cut by the user's decision (not necessary now):** the visual review (deferred until the camera work); the remaining slope diagnostics (CAD-collision run, two smoother candidate sites, a 30 s climb), spawn probes and arcs on level ground; figures and a regenerated `reports/validation_report.md`. None of them changes whether the world is ready for SLAM testing.

**Open decisions for the user**
- **Slope criterion:** accept 0.65-0.70 on the rough slope site and qualify the criterion for rough sites, or move the slope site to a smoother run (`final_validation.md` 2, 7).
- **Ground tolerance:** move it from 3 mm to 3.5 or 4 mm: about 15.1 or 13.7 M visual triangles instead of 17.13 M (`final_build.md` 5), an estimated 0.2 or 0.4 GB less per Gazebo process; 3.5 mm also meets the 16 M soft budget.
- **Pebbles:** simplify the 2-5 cm pebbles. That saves part of the roughly 1.5 GB they take in each process, but they look plainer to a camera.
- **Flat site:** no change is needed for acceptance (0.950-0.955 there, 0.940 on level ground; `final_validation.md` 3).

**Next**
1. The realistic Robin W LiDAR mode (the current sim LiDAR has no per-point time field and intensity 0).
2. SLAM testing on this world with DLIO and `config/slam/seyond_dlio_graph_lunar.yaml` (README section 5), with the simulation `--headless`.

## 2026-10-27: SLAM benchmark, ground-truth check, package reorganized

**SLAM benchmark** (README section 5):
- **Bags:** a 13-minute two-lap bag and the one-lap benchmark bag cut from it (`~/togo_assets/lunar/bags/`).
  Each has its ground truth, legs and a README.
- **Tools:**
  - `scripts/replay_slam_bag.sh` replays a bag into the Docker SLAM and scores it (`tools/slam_eval.py`: 4-DoF
    alignment, along / cross / vertical errors, pictures along the lap).
  - `scripts/benchmark_slam.sh` and `tools/slam_compare.py` compare configs, with 3 runs per label.
  - `scripts/rescore_slam_runs.sh` re-scores old runs.
- **Ground truth verified** (`tools/gt_check.py`) against the sim's IMU and LiDAR on the true terrain: median
  residual 0.1 mm, level to 0.005°, timing 0 ms.
- **Baseline** (3 runs): ATE 0.232 ± 0.081 m, vertical RMSE 0.041 m.
- **Findings** in DLIO, not acted on:
  - Its pose trails its stamp by about 47 ms, which shows as heading spikes in turns.
  - Its vertical drift is +3.1 ± 1.2 % of the climb.
  - Replays are not deterministic: identical runs gave ATE 0.17, 0.32 and 0.20 m.

**Package reorganized** (the user asked for a clearer layout):
- README section 0, *Start here*: what runs on Windows, in WSL and in the container, a task → command table,
  where results go, and the package map.
- Tests renamed by group, run by `scripts/run_tests.sh` (`quick` 1 min, `all` 5 min). The history reports were
  updated to the new names; their dated test-run tables keep the old ones:

  | old | new |
  |---|---|
  | `test_terrain`, `test_mesh`, `test_rocks`, `test_rocks_fixes`, `test_dtm_extract` | `test_world_terrain`, `test_world_mesh`, `test_world_rocks`, `test_world_rocks_fixes`, `test_world_dtm_extract` |
  | `test_world`, `test_build`, `test_fix_build`, `test_final_integration`, `test_validation` | `test_world_sdf`, `test_world_build`, `test_world_build_fixes`, `test_world_final_integration`, `test_world_validation` |
  | `test_runtime_script`, `test_runtime_launch`, `test_runtime_collision_tool`, `test_tools_stop`, `test_rover_drive_test` | `test_sim_scripts`, `test_sim_launch`, `test_sim_robot_collision`, `test_sim_gz_stop`, `test_sim_drive_test` |
  | `test_replay_slam` | `test_slam_replay` |

- The bag recorder moved from `~/togo_assets/lunar/experiments/bag_record/` into `tools/bag_record/`, with
  derived paths and `--dry-run`. Its logs now go into the bag folder. Since the move, only its arguments and dry
  run were tested.
- `tools/trim_bag.py --laps N` finds the lap end itself (the closest pass by the start).
- README section 5 now says `docker compose start dev`: `up` can recreate the container, and the SLAM build
  inside it is then lost.
- **WSL disk cleaned** at the user's request: about 19 GB of development-phase leftovers were deleted.
  - Removed: everything in `~/togo_assets/lunar/experiments/` (14 GB), `build/fix_build`, `build/fix_terrain`
    and `worlds/fix_*`. `~/togo_assets/lunar` is down from 27 GB to 7.9 GB.
  - Kept: the world, its build cache, the plugin and its build, the rock library and the bags.
  - Checked first: no code, world file, bag or shell config used the deleted folders. Some links in them pointed
    into the real world; they were removed as links first.
  - Checked after: the launcher, replay and recorder dry runs, the ground-truth check, and the full test suite.
  - The paths into `experiments/` that the dated reports in `docs/impl/` and `docs/research/` cite no longer
    exist. The reports keep their numbers and figures.
  - The cleanup also found a memory watchdog left running since the phase-7 validation, and stopped it with its
    stop file. It could only ever stop its own test simulations.
- **Next: SLAM speed-up and accuracy**, planned in `docs/SLAM_SPEEDUP_PLAN.md`, a handoff brief for a fresh
  session. The baseline adds two items the first plan lacked:
  - the path-helper scripts take about 1.6 cores, a third of all SLAM CPU;
  - the `graph_based_slam` backend's RAM grows by 315-331 MB/min.
- **2026-10-29, SLAM plan round 2** (the plan's "Round 2" section): DLIO's unused `degeneracyProjection` removed
  from the code and the configs; probe `kcorr10` (2 runs vs `speed`) worse (RPE 1 m +19 %, ATE max +17 %, zig-zag
  +15 %), not kept, k = 8 skipped by the agreed rule. Next: the information-weighting and scan-time-correction
  switches, one label each.
- **2026-10-29, round 2 continued**: switches built (information weighting, scan-time correction; both off by
  default, bit-identical when off). `lagfix` (2 runs vs `speed`): heading lag 38 -> -6 ms and heading error -29 %,
  but vertical RMSE x2.5 as a 3.5-4 % height-scale error and zig-zag +18 %; not kept as is. Found on the way: the
  observer runs with every gain at 1.0 (no `odom/geo/*` keys in the seyond configs), an undamped 1 rad/s loop;
  `geogains` probe (DLIO upstream gains, config only) queued. `iwstats` control run confirmed the weighting
  switch is a no-op at lambda0 = 0 and gave the information scale (weak direction p50 3612, strong 590k).
- **2026-10-29, information weighting works**: `infoweight_t2000` (2 runs vs `speed`): ATE -23 %, RPE 10 m -16 %
  and the slope -> plain leg -45 % beyond noise, per-scan corrections -29 %, zig-zag / vertical / CPU unchanged.
  Kept as a candidate; a stronger value and a rotation term are queued, plus the `geogains` probe.
- **2026-10-29, `geogains` probe** (DLIO upstream observer gains, config only, 2 runs vs `speed`): nothing improved
  (ATE +14 %, RPE 10 m +15 %, zig-zag +5 %), not kept; the unit-gain loop is not the zig-zag's source.
- **2026-10-29, weighting sweep**: 4000 (zig-zag -8 % but drift +61 % and a vertical excursion) and the rotation
  term (gain halved, height scale worse) both lose to 2000 translation-only, which becomes the config default;
  confirmation label `weighted` (3 runs) queued.
- **2026-10-29, lag-fix probes**: the 200 s diagnostic showed the vertical scale error sits in the registration
  (the state follows it to millimetres); `lagfix_eps` (ten times tighter GICP epsilons) halves it and halves the
  attitude errors but costs 4x GICP time and 12 % dropped scans, not viable. Last probe queued: a minimum
  number of GICP re-linearisations (`odom/gicp/minIterations`, new, default 1) with the lag fix.
- **2026-10-29, `weighted` confirmation** (3 runs of the new default vs `speed`): ATE max -16 %, zig-zag -6 %,
  corrections -30 % beyond noise; RPE 10 m -11 %; ATE median over five runs 0.143 vs 0.181 m with one drift
  excursion (0.258) of the kind the baseline also had. Kept as the reference label.
- **2026-10-29, round 2 closed**: `lagfix_min3` (lag fix + a minimum of 3 GICP iterations) halves the attitude
  errors and removes the lag but costs a fifth more lateral jitter and a tenth more DLIO CPU; left as an off
  switch with its variant config for the user's call. Kept: the information weighting as the default. The
  plan's "Round 2 outcome" paragraph has the full list; the code goes to the user for a commit before any
  full-filter work.
- **2026-10-29, round 3 opened**: new ideas built as switches (solver-side weighting, correspondence-scaled
  lambda0, two-stage correspondences), labels queued. The existing ground constraint on the lag fix: worse on
  every row (ATE +82 %), rejected; its fixed-height anchor does not fit sloped terrain.
- **2026-10-29, solver-side weighting kept**: `iw_solver` (2 runs vs `weighted`): ATE -22 % with a tight spread,
  corrections -17 %, GICP time -34 %, DLIO CPU -18 %, nothing worse; now the default (`mode: solver`). The
  pair-count scaling is neutral, the 0.5 m second stage is worse (drops ground pairs at range); both dropped.
- **2026-10-29, keyframe-time refinement**: not kept on its own (nothing better beyond noise, heading lag +19 %,
  GICP +33 %); the machine suspended for eight hours mid-chain, the broken run was discarded and the reference
  and lag-fix labels relaunched.
- **2026-10-29, new reference `solver`** (3 runs vs `speed`): RPE 1 m -6 %, RPE 10 m -15 %, zig-zag -9 %,
  corrections -41 %, GICP time -23 %, DLIO CPU -13 %, all beyond noise. The horizontal-only scan-time
  correction with a 3-iteration minimum passes the accuracy guard on this base (attitude errors halved, lag
  gone, RPE 1 m +8 %, CPU +12 %); the full version is being measured on the same base before the choice.
- **2026-10-29, lag fix adopted as a default (pending confirmation)**: the full scan-time correction fails the
  guard (zig-zag +14 %), the horizontal-only one passes with the same halved attitude errors; it becomes the
  default with the 3-iteration minimum. Rejected switches removed from the code; no variant configs remain.
- **2026-10-29, round 3 closed**: `round3` (3 runs) confirms the lag-fix trade (attitude errors -37 to -50 %, lag
  gone, zig-zag +9 % and RPE 1 m +11 % against the reference); by the guard it stays off, one config change away
  (`variants/seyond_dlio_graph_lunar__lagfix.yaml`). Default = solver-mode weighting (`solver` reference: RPE
  10 m -15 %, zig-zag -9 %, GICP -23 % vs `speed`). Rejected switches removed; the Kalman update is the only
  list item left. Tree handed to the user for a commit.
- **2026-10-29 evening, round 4 opened**: the error-state Kalman update built as a switch
  (`odom/geo/estimator: iekf`, `include/dlio/iekf.h`); first 80 s check clean with the best short-run zig-zag so
  far and no lag; the Hessian-scale sweep (5 / 20 / 80) is running as three labels against `solver`.
- **2026-10-30, round 4 measured**: the Kalman update (`odom/geo/estimator: iekf`) calibrated (the honest
  Hessian scale is 0.15-0.5, not 20) and measured on the lap: at scale 1 it removes the lag and 23 % of the
  zig-zag but adds a slow heading wander (RPE 10 m +17 %, end drift x4) because the keyframes follow the
  smoothed pose, so by the guard it stays off. Next: an output-only mode that leaves the map untouched.
- **2026-10-30, round 4 closed**: the error-state filter in its output-only form (`odom/geo/estimator:
  iekf_output`, scale 0.5) is the default: registration and map as before, the filter fuses the registration
  poses with the IMU for the published odometry only. Three runs against the round-3 default: zig-zag -32 %,
  heading lag gone, heading error -39 %, roll/pitch -58/-64 %, RPE rotation about -50 %, nothing worse (the 5-10 % lower
  GICP time and CPU of those runs are machine state: the registration is untouched). The feedback form (`iekf`) failed the guard (heading wander) and stays off; the observer's
  bias update (gain 1.0 per scan) was the cause of the output filter's first heading drift, fixed by giving the
  filter its own bias states. Tree handed to the user for a commit.
