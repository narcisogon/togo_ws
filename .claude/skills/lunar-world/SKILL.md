---
name: lunar-world
description: Orientation for the Apollo 16 lunar Gazebo world in src/lunar_world_gen (generator, launcher, tools, validation) and for running Gazebo on WSL. Use when building, regenerating, tuning, running or debugging that world (lunar_world_gen.build, config/apollo16_core.yaml, run_apollo16_sim.sh, open_gui.sh, env.sh, apollo16_sim.launch.py, the patched Bullet plugin, bench_world / lidar_capture / run_robot_tests / render_views), when validating it, or for RAM, GPU, crash-dump or GUI-window problems of Gazebo in WSL (memory growth, llvmpipe, WSLg COPY MODE).
---

# Apollo 16 lunar world (`src/lunar_world_gen`)

A generator package (`COLCON_IGNORE`, its own Python venv) that builds a Gazebo Harmonic (gz-sim 8.11) world of the
Apollo 16 landing site for the TOGO rover (Clearpath A300, Seyond LiDAR, IMU; ROS 2 Jazzy): the real LRO NAC
elevation model above 10 m wavelengths plus calibrated synthetic fBm relief, talus-limited craters and about
250 k Blender rocks merged into per-tile meshes. Zones: 100 m core at 1 cm, 500 m LiDAR ring, 2.2 km far field,
20 km horizon. Gravity 1.62 m/s², patched classic Bullet. Gazebo runs natively in WSL; SLAM and Nav2 run in the
Docker container (ROS domain 29, CycloneDDS; see the `togo-sim-slam-nav` skill). The final world (build of
2026-10-25): 17.13 M visual / 0.81 M collision triangles, 215,677 core rocks, 8/8 static checks.
**Status (2026-10-27): validated, ready for SLAM testing** (reports `e2e_launch.md` and `final_validation.md` in
`src/lunar_world_gen/docs/impl/`): all acceptance checks pass except the slope climb on the rough slope site
(0.65-0.70 against 0.80); the visual review is deferred until the camera work.
**Status, open decisions and what is still unverified: `src/lunar_world_gen/docs/PROGRESS.md` (newest at the bottom).**

## Where things are

| what | where |
|---|---|
| package | `src/lunar_world_gen`. In WSL: `REPO=/mnt/c/Users/Username/OneDrive/Desktop/togo_ws-slam-nav/togo_ws-slam-nav`, `PKG=$REPO/src/lunar_world_gen` |
| the built world | `~/togo_assets/lunar/worlds/apollo16_core/` (WSL disk): `apollo16_core{,_inspect,_stock,_inspect_stock}.sdf`, `meshes/` (324 GLBs), `textures/`, `world_info.yaml` (sites, counts), `rocks_catalog.npz/.csv`, `reports/` |
| build cache | `~/togo_assets/lunar/build/apollo16_core/` (3.2 GB, one folder per stage with `manifest.json`) |
| physics plugin | `~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so` |
| rock library | `~/togo_assets/lunar/library/rocks_v1/` (Blender 5.2 on Windows, `$PKG/blender/rock_library.py`) |
| benchmark bags, SLAM runs | `~/togo_assets/lunar/bags/<bag>/` (each with `README.md`, `ground_truth.tum`, `segments.json`, `gt_check.*`); runs in `bags/slam_runs/<run>/`, summaries `slam_runs/summary__<label>.md/.png` |
| experiments | `~/togo_assets/lunar/experiments/<label>/`, scratch for one-off runs. Big raw data goes here, never into the repo. It is empty since 2026-10-27: the development-phase scratch (14 GB), `build/fix_*` and `worlds/fix_*` were deleted at the user's request, so paths into them in `docs/impl/` and `docs/research/` no longer exist. Code that is reused belongs in the package; the bag recorder is `tools/bag_record/`. |
| Python | venv `~/.venvs/lunar_world` for the generator and tests; the system `/usr/bin/python3` (gz-transport bindings, ROS) for the launcher and every tool that talks to Gazebo or reads bags |
| docs (in `$PKG`) | `README.md`: section 0 *Start here* is the user's map (Windows / WSL / container, task → command, results, tests), then run, RAM, SLAM benchmark, troubleshooting. Also `docs/USER_GUIDE.md` (differences to `lunar_surface.world`, realism notes), `docs/DESIGN.md` (spec with "as built" notes) and `docs/PROGRESS.md` (newest at the bottom). `docs/impl/*.md` and `docs/research/` are the dated development history |

## Run

```bash
cd $REPO                                                        # repo root in WSL
./src/lunar_world_gen/scripts/run_apollo16_sim.sh               # server first, the GUI opens at READY
./src/lunar_world_gen/scripts/run_apollo16_sim.sh --headless    # no GUI; later, in a second terminal:
./src/lunar_world_gen/scripts/open_gui.sh                       # waits for READY, opens the GUI; Ctrl+C closes only it
./src/lunar_world_gen/scripts/open_gui.sh --close               # close whichever GUI is open
source src/lunar_world_gen/scripts/env.sh                       # any other terminal: domain 29, DDS, GPU env, leak fix
```

- Options: `--site flat|slope|rock|turn`, `--variant inspect|stock|inspect_stock`, `--pose X Y Z [YAW]`,
  `--robot-collision simplified|original`, `--gui-early` (old order), `--rviz`, `--dry-run` (README section 4;
  `tests/test_docs.py` fails if the README and the scripts' `--help` headers disagree).
- The launch prints `[apollo16_sim] READY at +T s` (about 35-60 s; 37-47 s measured) and writes a ready marker
  (`ready_file:=`); the wrapper opens the GUI only then. The GUI window (title `Gazebo Sim (Ubuntu)`) stays blank
  30-60 s while it loads its own copy of the world (measured: terrain 35-41 s after READY).
- Stop with Ctrl+C in the wrapper's terminal (gz gets SIGKILL first, ROS nodes SIGINT; 2-3 s, no dump); never the
  GUI's X button.
- RAM after loading, flat with the leak fix (22 min measured): GUI 3.7-3.8 GB (2.1 rocks, 1.0 ground, 0.5 base),
  server with the rover 3.7-3.9 GB; WSL about 8.0 GB in total (4.4 GB headless) of 15.9 GB. One GUI at a time.
  `--headless` + `open_gui.sh`, or RViz, saves the GUI's share.
- RTF: headless 0.93-1.0 while driving, 0.82-0.92 with the GUI open (lowest on far views). For SLAM run
  `--headless` and open the GUI only when needed.
- Known limits (README section 9): the sim LiDAR has no per-point time field and intensity 0 (a Robin W mode is
  next); rigid ground, no terramechanics; the rough `slope` site climbs only 0.65-0.70 (smooth 10-15 deg inclines
  0.91-0.95); the parked rover creeps 2-4 mm/s on 4-5.5 deg ground; no visual review yet.
- SLAM: DLIO must use gravity 1.62. In the container, from `~/slam_ws/src/lidarslam_ros2/scripts/togo`:
  `COMBINED_PARAM_FILE=/home/er4-user/ws/src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml ./run_live_seyond_dlio_slam.sh`.
  After the user edits `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml`, regenerate the copy with
  `python3 $PKG/config/slam/make_lunar_dlio_config.py` (`--check` only reports).
- Topics are those of `src/togo_sim_wsl.sh`: `/husky/sensors/seyond/points`, `/husky/sensors/imu_0/data_raw`,
  `/platform_velocity_controller/cmd_vel` (TwistStamped). Ground truth: `gz model -m togo -p` (same `GZ_PARTITION`).

## Build

```bash
cd $PKG                                                         # in WSL
PY=~/.venvs/lunar_world/bin/python
$PY -m lunar_world_gen.build --preset full                     # all 10 stages, about 7.3 min from scratch
$PY -m lunar_world_gen.build --status                          # which stages are stale, and why
$PY -m lunar_world_gen.build --stages ground,contact --force [--force-downstream]
$PY -m lunar_world_gen.build --set mesh.core_visual_max_error_m=0.0035   # one-off override; affected stages re-run
./scripts/run_tests.sh [quick|world|sim|slam|docs|all]         # quick 1 min (default), all (~340 tests) 5 min
```

- Tests are grouped by file name: `tests/test_world_*` (generator), `test_sim_*` (launcher, launch file, gz stop),
  `test_slam_*` (replay, scoring, gt check, benchmark and recorder scripts), `test_docs.py`. A new test file
  needs one of these prefixes; `test_docs.py` fails otherwise. `run_tests.sh` uses the venv and passes extra
  arguments to pytest.

- Stages: terrain, rocks (placed on H0), heights (fillets), ground (rendered TINs, stitched), contact (rocks
  re-seated on the rendered TIN), collision, meshes (collision TINs, merged rocks, budgets), export, world (sites,
  `world_info.yaml`, 4 SDFs, `gz sdf -k`), validate (static checks, report). A stage key hashes its config keys,
  code, library versions and inputs; a re-run stage invalidates everything downstream. Exit 0 ok, 1 stage failed,
  2 validation failed or a check did not run. A build lock stops a second build on the same folders.
- `--preset fast` builds into `<dir>_fast`. The rebuild is byte-deterministic.
- Never edit the world folder by hand; change the config or the code and rebuild.
- If a rebuild moves the `flat` site: `python3 $PKG/config/gui/make_apollo16_gui_config.py` (the GUI start camera).
- Terrain alone: `$PY -m lunar_world_gen.terrain.compose --all`; recalibration: `$PY -m lunar_world_gen.terrain.calibrate`
  (`--apply` rewrites the config file: use `--config` with a copy, or apply the gains by hand).

## Package map

- `lunar_world_gen/` foundation `grid.py config.py layout.py`; `build.py` (stages, caching, budgets).
  - `terrain/`: `base` (DTM via `tools/dtm_extract.py`), `fbm` (coarse and fine band, one gain each), `craters`
    (crest-exact, shaped), `talus` (32° limiter T), `compose` (H0 = B + w_ring (Fc + K_big + T) + w_core (Ff + K_small)), `calibrate`.
  - `rocks/`: `library` (Blender prototypes), `population` (SFD, density field, crater enhancement, core-edge band,
    overlap repair), `placement` (rest facets, settling, burial), `fillets` (wedges), `recontact`,
    `collision_field` (core grid; ring footprint dilation), `catalog`, `merge` (per-tile rock meshes).
  - `mesh/`: `tin` (pydelatin), `refine` (max-edge, off), `stitch` (`check_stitched`), `locate`, `normals`, `uv`, `gltf`.
  - `materials/regolith.py`; `world/`: `sdf`, `lighting`, `spawn` (test sites), `world_info`;
    `validate/`: `stats`, `static_checks`, `lidar_checks`, `report`.
- `launch/apollo16_sim.launch.py` (sequenced launch, gz guard, ready marker, plugin fallback, simplified robot
  collision).
- `scripts/`:
  - simulation: `run_apollo16_sim.sh`, `open_gui.sh`, `apollo16_lib.sh`, `env.sh`;
  - SLAM benchmark: `replay_slam_bag.sh`, `benchmark_slam.sh`, `rescore_slam_runs.sh`;
  - `run_tests.sh`.
- `config/`: `apollo16_core.yaml`; `gui/` (GUI config + camera generator); `slam/` (lunar DLIO copy + generator);
  `robot/a300_collision_simplified.yaml` (from `tools/robot_collision_simplify.py`).
- `tools/`:
  - SLAM scoring: `slam_eval.py`, `slam_compare.py`, `gt_check.py`, `trim_bag.py` (`--laps N`),
    `proc_sampler.py`, `slam_replay_check.py`;
  - bag recording: `bag_record/` (`plan_route.py`, `route.json`, `record_bag.sh`, `bag_drive.py`, `check_bag.py`);
  - validation tools, below.
- `blender/` (rock library), `tests/` (pytest, grouped as above).

SLAM benchmark (README section 5):
- **How a run is scored:** `replay_slam_bag.sh` → `slam_eval.py`. The truth is interpolated to each stamp, then
  aligned in 4-DoF. DLIO's identity placeholder poses from IMU calibration are skipped. The pictures are
  `map.png`, `profile.png` and `errors.png`.
- **Comparing:** labels of 3 runs, because replays are not deterministic (ATE spread ±0.08 m).
- **Checking a new bag:** `gt_check.py` checks its truth with the sim's own IMU and LiDAR.

Config keys that matter (`config/apollo16_core.yaml`):

- `mesh.core_visual_max_error_m` (0.003): the biggest lever on triangles and RAM. 3.5 mm gives about 15.1 M visual
  triangles, 4 mm 13.7 M (open user decision). The pebble LOD (`rocks.classes`, pebbles LOD0 = 5.28 M triangles) is the other.
- `terrain.roughness.fbm_gain` 0.109 / `fbm_gain_fine` 0.578 are calibrated together with
  `zones.core.edge_fade_m` 3.0, `terrain.talus.max_slope_deg` 32 and `terrain.craters.*`: change one, recalibrate.
- `mesh.collision.core_max_error_m` 0.015; `mesh.collision.core_max_edge_m` / `ring_max_edge_m` null (keep, lesson 8);
  `mesh.collision.ring_rock_dilation` (code default 1.0); `rocks.collision_min_D_m` 0.05.
- `physics.engine` patched|stock, `physics.step_s`, `physics.gravity_m_s2` 1.62 (the DLIO copy reads it too),
  `physics.ground_mu` 0.4; `lighting.realistic.earthshine_intensity` 3e-5.
- `budgets.*` (`*_warn_tris` soft, `*_max_tris` hard), `validation.*` (load 30 s, RTF 0.95, RSS 5 GB), `paths.*`.

## Validation tools

Gazebo-side tools run with `/usr/bin/python3` from `$PKG` in WSL, start gz with `setsid` + `timeout -s KILL`, stop
it with SIGKILL, and set the GPU environment and the leak fix (`tools/bench_world.py` `gz_env()`). Give each run
its own `--partition` / `--domain` (not 29) and a private `ROS_HOME`.

- Static: the build's `validate` stage, `reports/static_checks.json` and `reports/validation_report.md`.
- `tools/bench_world.py WORLD.sdf --out R.json --pose-agl=X,Y,0.449,YAW --lidar real --config config/apollo16_core.yaml`:
  world-only load time, RTF, RSS, VRAM, D3D12 check (the `=` matters when the value starts with `-`).
- `tools/lidar_capture.py` + `$PY -m lunar_world_gen.validate.lidar_checks DIR --config ... --auto-inputs`.
- `tools/run_robot_tests.py --out DIR --world SDF --sites flat,slope,rock,turn --repeats N`: full launches plus
  `tools/rover_drive_test.py` drives (flat standard drive, slope climb, rock traverse, in-place turn).
- `tools/render_views.py WORLD.sdf --out DIR --view NAME:X,Y,Z,YAW,PITCH --agl --config ...`: camera images.
- `$PY -m lunar_world_gen.validate.report`: `validation_report.md` from every JSON in `reports/` (results older
  than the SDF are archived by the next build).
- Others: `tools/gz_physics_bullet_lunar/incline_test.py` (mesh friction), `tools/verify_rock_glb.py` (library),
  `tools/robot_collision_simplify.py` (rerun after a Clearpath description update).

## Hard-won lessons

1. **Never send SIGTERM/SIGINT to gz.** gz-sim 8 segfaults on shutdown and WSL writes a 1.5-3.6 GB dump to
   `%TEMP%\wsl-crashes` (`/mnt/c/Users/Username/AppData/Local/Temp/wsl-crashes`; it keeps 10, once 30 GB). Start
   gz with `setsid timeout -s KILL <s> gz sim ...`, stop it with `kill -KILL -- -<pgid>`. The wrapper,
   `open_gui.sh` and the launch's gz guard do this; `src/togo_sim_wsl.sh` does not. List the crash folder before
   and after; delete only dumps of your own processes. Never `pkill -f` a pattern that also matches your own
   `bash -lc` command line.
2. **GPU:** `GALLIUM_DRIVER=d3d12` (+ `/usr/lib/wsl/lib` in `LD_LIBRARY_PATH`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`).
   Without it Mesa silently renders on llvmpipe. Check `libd3d12.so` in `/proc/<pid>/maps` or `GL_RENDERER` in
   `~/.gz/rendering/ogre2.log`.
3. **The memory leak:** every gz process that renders (the GUI, a server with sensors) needs
   `MESA_EXTENSION_OVERRIDE=-GL_ARB_multi_draw_indirect`. Mesa's d3d12 driver (25.2.8, still in 26.2.5) leaks
   one `ID3D12CommandSignature` per indirect draw (`d3d12_get_cmd_signature` searches its cache with `&key`), and
   Ogre-Next draws indirectly whenever the extension exists: +2.7-10 MB/s per process until WSL dies. With the
   override the images, LiDAR ranges and speed are unchanged. Set in `env.sh`, `bench_world.gz_env()` and the user's `src/togo_sim_wsl.sh`; not in
   `~/.bashrc` (the user wanted it for the WSL sim only). Check: `/proc/<pid>/environ`, and
   `grep -c 'rw-s.*/dev/zero' /proc/<pid>/maps` must stay constant. A future GL 4.6 context would bypass the
   override (today 3.3; `docs/impl/leak_diagnosis.md` 4.1).
4. **WSLg COPY MODE:** when WSL was booted by a hidden or background process (a keep-alive after the PC slept),
   GUI windows break and their title starts with `[WARN:COPY MODE]`. Only the user can fix it: close WSL terminals,
   run `wsl --shutdown` from a new Windows terminal, start again. Never run `wsl --shutdown` yourself (it kills
   everything, Docker too); start hidden WSL hold processes only while WSL is already running.
5. **Calling WSL from Git Bash:** `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu --exec bash -lc '...'`. MSYS rewrites
   `/paths`, and without `--exec` an extra shell layer eats `$` and backslashes. Simplest: write a script and run
   `wsl.exe -d Ubuntu --exec bash /mnt/c/.../script.sh`. `python3` is not on the Windows PATH; in `python -I` use
   `sys.stdout.reconfigure(encoding="utf-8")`.
6. **gz renders a sensor only while something subscribes to it.** An unsubscribed LiDAR costs nothing and leaks
   nothing; benchmarks keep a gz-transport subscriber (bench_world and lidar_capture do).
7. **Patched Bullet plugin:** stock classic Bullet damps yaw (in-place turns 13 % of the command at 1.62 m/s²) and
   ignores mesh friction (fixed 0.5). The SDF names the patched plugin by absolute path; the launch dlopen-checks
   it and otherwise falls back to `*_stock.sdf` (4 ms step). After a ROS/gz update rebuild it
   (`tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh`, needs gz-physics 7.6.0) and look for
   `[lunar patch] mesh collision` in the server log.
8. **Do not cap the collision triangle size** (`mesh.collision.*_max_edge_m`): it made slope climbing worse
   (0.83 to 0.34-0.73). Slow slopes came from the robot's CAD collision meshes in GImpact; the fix is
   `robot_collision:=simplified` (88 boxes, the default).
9. **DLIO gravity coupling:** the world's 1.62 must equal DLIO's `odom/gravity`. With 9.82, DLIO stores a
   -8.2 m/s² accelerometer bias (`Accel biases [xyz]:` ends near -8.2) and drifts as soon as the rover tilts. Use
   the lunar copy; never edit the user's `seyond_dlio_graph.yaml` (the old world uses 9.82 on purpose).
10. **Keep rocks merged** into per-tile meshes (10 k separate visuals: RTF 0.03). One missing mesh aborts the
    whole world load. GLBs: raw Z-up, an explicit material with metallic 0.
11. **Start-up stall:** spawning the robot stalls the simulation while its sensors build the render scene (about
    25 s here). Controller spawners must wait for it (the launch gates them on the LiDAR topic); with the stock
    chain (`togo_sim_wsl.sh`) one spawner always fails on this world.
12. **Measuring:** WSL's wall clock can step (use `time.monotonic`); the `ros2 control` CLI leaves a ros2 daemon
    behind (tools call `/controller_manager/list_controllers` through rclpy); RTF needs a quiet machine (other gz
    servers or builds can halve it); Windows sleep kills long runs (keep the PC awake).

## Rules

- No git commands: the user manages git.
- Edit only inside `src/lunar_world_gen` unless the user agrees. So far the user agreed to one change outside it:
  the `MESA_EXTENSION_OVERRIDE` line in `src/togo_sim_wsl.sh`. Offer machine-wide changes (`~/.bashrc`,
  `.wslconfig`) as suggestions.
- One Gazebo instance on the machine at a time (about 3.5 GB per process, one GPU): check `pgrep -af "gz sim"`
  first (it also lists your own `bash -lc` line), never kill a gz you did not start, abort below 3 GB `MemAvailable`.
- Generated assets, caches and raw data go under `~/togo_assets/lunar/` on the WSL disk, never into the repo
  (it lives on OneDrive).
- Isolated test runs use their own `GZ_PARTITION`, a `ROS_DOMAIN_ID` other than 29 and a private `ROS_HOME`;
  domain 29 belongs to the user's simulation and the SLAM container.
- Do not claim a fix works unless it ran. Keep README, USER_GUIDE, DESIGN and PROGRESS in step with behaviour
  changes, and write "not measured" where a result is missing.
- The user found the Windows / WSL / container split and the many tests confusing. README section 0 (*Start
  here*) is their map: when you add a script, tool or result folder, give it a row there. Reused code goes in
  the package, never only in `~/togo_assets/lunar/experiments/`.
