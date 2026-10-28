# Runtime fixes: robot collisions, launch, wrapper, environment (phase 4, `runtime`)

Owner: runtime fix agent, 2026-10-25, 16:10-18:05. Files owned: `launch/apollo16_sim.launch.py`,
`scripts/run_apollo16_sim.sh`, `scripts/env.sh` (new), `tools/robot_collision_simplify.py` (new),
`config/robot/a300_collision_simplified.yaml` (new, generated), `tests/test_runtime_*.py` (new), this file.
Every Gazebo run used `GZ_PARTITION=claude_fixruntime`, `ROS_DOMAIN_ID=94`, a private `ROS_HOME`, `setsid`,
coreutils `timeout`, and was stopped without ever sending SIGINT/SIGTERM to a gz process (except the one
deliberate U5 baseline, section 2). Scratch: `/home/nargon/togo_assets/lunar/experiments/fix_runtime/`
(`EXP` below). The build, rocks, terrain and docs agents ran Gazebo and multi-core jobs at the same time:
**every RTF below is contended and only indicative**. Variants were interleaved so that they saw the same
conditions, and every robot run records the other agents' CPU (`contention_drive`, 2 s bins).

## 0. Findings and status

| finding | verified | change | result |
|---|---|---|---|
| gz_validation 5.1, RTF while driving (primary) | reproduced: original collisions at the slope, RTF **0.122-0.137** (3 runs), main sim thread 7.4 CPU-s per simulated s | launch `robot_collision:=simplified` (default) replaces all 14 CAD collision meshes by 88 bounding boxes from `tools/robot_collision_simplify.py` (clearance kept within 15 mm) | slope RTF **0.66-0.70** (contended, same conditions as original 0.12-0.14), main thread 1.09-1.15 CPU-s per sim s; flat 0.67-0.82, rock 0.86, turn 0.80; drive metrics unchanged (1.3). RTF 0.9 not reached (1.4) |
| U4 / CR-10, option without value hangs | reproduced on the original script: `--site`, `--world`, `--variant`, `--pose 1 2` spin until `timeout 5` (rc 124, 5.00 s) | `need_val` + value checks, `--dry-run` | rc 2 in 0.02 s with `ERROR: --site needs 1 value(s)`; 17 parametrised tests |
| U5, Ctrl+C: segfault, 1.5-3.6 GB dump, 15-23 s | reproduced with the original wrapper: 3.39 GB dump, 19.1 s (section 2) | gz guard in the launch (SIGKILL), wrapper stops gz with SIGKILL first | Ctrl+C: all processes gone in **1.2-2.2 s** in 5 runs (wrapper and plain `ros2 launch`), 0 dumps |
| U7, unloadable plugin not detected | reproduced with a garbage `.so`, a missing `NEEDED` library and an undefined symbol (tests) | `plugin_loadable()`: dlopen with `RTLD_NOW` in a child python (30-70 ms), fallback to the stock variant | real plugin passes; the three broken cases fall back with the loader's message |
| U9, readiness messages | reproduced (old banner "Simulation running" at +9 s, controllers at +34-39 s, 600 s silent wait) | `wait_for_sensors` progress every 10 s; `wait_ready` (controllers active via `list_controllers` + `/clock` advancing) then `[apollo16_sim] READY at +T s` banner; spawner exit != 0 -> WARNING with the next step | READY at launch +29.9 to +43.7 s in 5 runs (other agents' simulations running) |
| U10, error messages | reproduced (world missing, no world_info hint names launch args, errors in stdout mid-banner) | messages name the next step, go to stderr, before the banner | tests |
| U11, stock fallback drops inspect lighting | by code inspection + test | inspect falls back to `<stem>_inspect_stock.sdf` when present, else `_stock` with a note; WARNING names consequences and the build command | tests (the main world has no `inspect_stock` yet: build agent) |
| U6, GUI config wiring | option name checked: `gz sim --help` lists `--gui-config [arg]` | wrapper starts `gz sim -g --gui-config <per-run copy of config/gui/apollo16.gui.config>` with the camera moved to the chosen spawn; `--gui-config FILE`, `--default-gui` | dry-run tests; GUI not started (no GUI windows) |
| U2 part, `scripts/env.sh` | - | new sourceable environment, also used by the wrapper | tests |
| U3, `togo_sim_wsl.sh` + `TOGO_WORLD` | reproduced, section 6 | none (user's files) | facts for the docs agent |

## 1. Simplified robot collisions (primary RTF fix)

### 1.1 What is replaced

The robot description is generated exactly as before (`xacro togo_gz.urdf.xacro ns:= tf_prefix:= is_sim:=true
use_fake_hardware:=false`, now `Command(...).perform()` in the launch). With `robot_collision:=simplified` (default)
the launch then substitutes the `<collision>` elements in the XML; visuals, inertials, joints, `<gazebo>` and
`<ros2_control>` are untouched (checked element by element on the real URDF with `EXP/scripts/check_simplify.py`:
36 visuals, 7 inertials, 61 joints, 14 `<gazebo>`, 1 `<ros2_control>` identical; no mesh collision left).

| collision mesh (Clearpath, `/opt/ros/jazzy/share`) | triangles | links | down axis | replacement (default) | reaches further towards the ground than the mesh (one box would) |
|---|---|---|---|---|---|
| `drivetrain/motor.stl` | 46,604 | 4 | -z | 8 horizontal slab boxes | 13.1 mm (38.0 mm) |
| `seyond_robin_w.stl` | 173,872 | 1 | +y | 6 slab boxes along mesh y | 13.4 mm (47.3 mm) |
| `drivetrain/suspension_beam.stl` | 3,004 | 2 | -z | 8 slab boxes along x | 13.9 mm (40.7 mm) |
| `drivetrain/suspension_spacer.stl` | 748 | 2 | -z | 1 box | 0.0 mm |
| `chassis_collision.stl` | 196 | 1 | -z | 16 horizontal slab boxes | 13.2 mm (118.9 mm) |
| `attachments/observer_enclosure.stl` | 140 | 1 | -z | 12 horizontal slab boxes | 14.1 mm (162.8 mm) |
| `attachments/observer_arch.stl` | 116 | 1 | -z | 2 horizontal slab boxes | 420 mm: the opening of the inverted-U arch on top of the enclosure, 0.4 m above the chassis (not a clearance) |
| `attachments/bumper_collision.stl` | 106 | 2 | -z | 1 box | 14.5 mm |

14 collision meshes, 368,456 triangles -> 88 boxes, 0 mesh triangles. Variants measured below: **original**;
**v1** = one bounding box each for motor, beam and LiDAR (7 boxes; default 16:24-17:12); **v3** = slab boxes for
those three (54 boxes; default 17:12-17:38); **v4** = all 14 meshes (88 boxes; default since 17:38, final).

- **Why slab boxes.** One bounding box fills concave outlines: the suspension beam is a bridge with an arch
  underneath (35-41 mm higher mid-span than at its end lugs), the motor's bottom edges are rounded, the chassis
  outline is stepped. v1 lowered the clearance under the beam from about 13.6 to 10.1 cm. The tool takes, per
  mesh, the candidate with the fewest boxes whose `down_gap_mm` <= 15 mm: how much further than the mesh it
  reaches in the mesh direction that points at the ground where it is mounted (from the URDF joint chain: -z for
  everything but the LiDAR, +y there), over 2 mm bins with exact triangle clipping. Candidates: the single box
  or cylinder (a cylinder only if < 0.9 x the box volume; motor 0.94, so boxes), then K slab boxes along x, y or
  z (the mesh cut into K slabs, each bounded by the box of its surface clipped to the slab; K = 2..16). The
  union of the boxes contains every vertex (asserted after 1 um rounding) and every surface point (test).
- **Why all meshes, not only those "above a few thousand triangles" (task wording).** v3 kept the 2,160
  triangles of chassis, spacers, bumpers, enclosure and arch as meshes. Thread sampling on the slope (1.4) shows
  they still cost 20-35 % of the main simulation thread (v3 1.36-1.72, v4 1.09-1.15 CPU-s per simulated s, paired
  runs). With the slab rule their geometry is kept (one box would put the chassis 119 mm lower in places).
  `--min-triangles 2000` reproduces v3.
- **No body contact on driving sites.** Along the logged poses of the flat-site in-place turn
  (`EXP/scripts/turn_clearance.py`), the lowest motor/beam point stays 66-112 mm above the 2 cm collision height
  field (the TIN follows it within 15 mm), for the meshes, v1 and v3 alike.

### 1.2 Tool and spec

- `tools/robot_collision_simplify.py` (venv: trimesh, numpy, scipy; 13 s): expands the xacro in a ROS subshell
  (or `--urdf`), resolves `package://` in the native install and `/opt/ros/jazzy/share`, fits, writes
  `config/robot/a300_collision_simplified.yaml`. Options `--min-triangles` (0), `--max-down-gap` (15 mm),
  `--cylinder-preference` (0.9), `--out`, `--save-urdf`. Needs no shapely (own slab clipping).
- The YAML records per mesh: URI, scale, file, bytes, **sha256**, triangles, links, down axes, bounds, hull
  volume, the primitive (`box`, `cylinder` or `compound` with `parts`), `down_gap_mm` (and the one-box value).
- At launch (system python, no trimesh) `simplify_collisions()` composes each part's pose with the collision's
  `<origin>` (URDF rpy, pure python), writes one `<collision>` per part (`name_k` if the original was named) and a
  comment with the original URI. A mesh whose resolved file has another size or sha256 than the spec is **kept**
  with `WARNING: kept the original collision mesh of <link>: ... re-run tools/robot_collision_simplify.py`; a
  missing or unreadable spec gives the original description with a WARNING, never a failed launch. Hashing the
  meshes costs about 40 ms.
- Launch log: `[apollo16_sim] robot collision: simplified - 14 CAD collision meshes (368,456 triangles) -> 88 boxes (spec ...)`.

### 1.3 Measurements (`tools/run_robot_tests.py`, `EXP/runs/rtf_*`, all contended)

"other" = mean CPU cores used by other agents' processes during the drive. RTF = simulated / wall time over the drive.

**Slope** (`climb 0.4 m/s 15 s`, then stop):

| run | collision | RTF | other | climb distance | climb angle (deg) | yaw drift (deg) | max roll (deg) | speed tracking | long. slip |
|---|---|---|---|---|---|---|---|---|---|
| 2 / 4 / 33 | original | **0.137 / 0.137 / 0.122** | 2.3 / 1.9 / 3.7 | 0.837 / 0.855 / 0.833 | 11.39-11.48 | -20.5 to -22.9 | 10.0-11.5 | 0.887-0.907 | 0.139-0.161 |
| 1 / 3 | v1 | 0.650 / 0.609 | 1.5 / 3.2 | 0.837 / 0.858 | 11.39-11.47 | -27.0 / -20.9 | 12.8 / 9.3 | 0.893 / 0.907 | 0.158 / 0.137 |
| 12 / 14 / 31 / 34 | v3 | 0.707 / 0.429 / 0.451 / 0.598 | 0.9 / 3.5 / 3.2 / 2.4 | 0.856-0.858 | 11.40-11.42 | -18.9 to -20.3 | 7.0-10.0 | 0.901-0.906 | 0.137-0.139 |
| 21 / 32 / 35 | **v4 (default)** | **0.441 / 0.664 / 0.695** | 4.9 / 3.2 / 1.8 | 0.842 / 0.842 / 0.853 | 11.42-11.43 | -20.6 to -20.7 | 9.8-11.5 | 0.894-0.902 | 0.141-0.153 |
| gz_validation L1/L5 | original | 0.191 / 0.198 | ~0 | 0.853 | 11.42 | -21.3 | 7.9 | 0.899 | 0.141 |

The 20 s simulated drive took 146-163 s wall with the original collisions and 28-45 s with v4. Worst 2 s bin:
original 0.023-0.027, v4 0.28-0.59.

**Rock** (0.3 m/s over the 6.1 cm rock) and **turn site** (in place 0.6 rad/s, 6 s):

| run | collision | rock: distance / v_ss / slowest 1 s (m/s) / max roll / RTF | turn: yaw tracking / dyaw / drift / RTF |
|---|---|---|---|
| gz_validation | original | 0.9966, 0.9981 / 1.018 / 0.294 / 5.6 deg / 0.92, 0.83 | 0.9545, 0.9534 / 187 deg / 0.30-0.36 m / 0.85, 0.91 |
| 6 | original | - | 0.9549 / 187.5 / 0.34 / 0.83 |
| 5 | v1 | 0.9951 / 1.017 / 0.293 / 5.4 / 0.66 | 0.9526 / 186.6 / 0.39 / 0.50 |
| 15 | v3 | 0.9963 / 1.017 / 0.292 / 5.5 / 0.65 | 0.9541 / 187.2 / 0.35 / 0.55 |
| 41 | **v4** | 0.9957 / 1.017 / 0.293 / 5.6 / **0.86** (other 0.8) | 0.9554 / 187.5 / 0.38 / **0.80** (other 1.7) |

**Flat**, `standard` preset (forward 0.5 m/s 6 s, in-place turn 0.6 rad/s 6 s):

| runs | collision | RTF | forward distance | v_ss | in-place turn tracking |
|---|---|---|---|---|---|
| gz_validation L0, L4; 6, 16, 51, 53, 55 | original | 0.29-0.57 | 0.9259-0.9317 | 1.003-1.006 | 0.883, 0.925; 0.897, 0.862, 0.850, 0.837, 0.845 (mean 0.871) |
| 5 | v1 | 0.718 | 0.9285 | 1.004 | 0.823 |
| 11, 13; 23, 24 (world copy throttled to RTF 0.45) | v3 | 0.67-0.69; 0.39-0.41 | 0.9279-0.9305 | 1.004-1.005 | 0.847, 0.854; 0.883, 0.844 |
| 22, 42, 52, 54 | **v4** | 0.67-0.82 | 0.9266-0.9289 | 1.004 | 0.869, 0.853, 0.876, 0.854 (mean 0.863) |

- Forward distance and steady-state speed are the same for all variants. The distance stays 1-4 mm short of the
  0.93 threshold for every variant (gz_validation 5.2: the corridor is not level).
- The in-place turn at the end of the flat corridor scatters 0.84-0.93 between identical runs for every variant:
  the rover rolls 6.8 deg and pitches 6-9 deg there and drifts 0.31-0.56 m while turning (0.97 and < 0.1 deg on
  gz_validation's flat control world). Wheel rates are identical in all runs (+-1.78 rad/s); only the yaw slip
  varies. Original mean 0.871 (n=7), v3+v4 0.860 (n=8): no measurable effect of the collision geometry (an
  apparent gap after the first four original runs disappeared with three more). DESIGN 13's 0.85 threshold is
  marginal on this site for every variant; the `turn` site (0.953-0.955 for all) is the robust check.
- One of 34 robot launches (run 56, v4) ended at +8.9 s before the robot existed: the gz server exited with
  `glx: failed to create drisw screen` / `X Error ... GLXBadFBConfig` while other agents' servers ran (WSLg could
  not create a GPU context). The guard passed gz's exit code 1 through and the launch shut down at once. The
  other 33 launches: both controllers active after 34-64 s, 0 spawner failures, every gz stopped by SIGKILL.

### 1.4 Remaining RTF gap

Per-thread sampling of the server during the slope drive (`EXP/scripts/thread_sampler.py`, paired runs):

| run | collision | RTF | main simulation thread (CPU-s per simulated s) | whole server | other |
|---|---|---|---|---|---|
| 33 | original | 0.127 | **7.37** (0.93 cores) | 10.35 | 3.7 |
| 31 / 34 | v3 | 0.457 / 0.598 | 1.72 / 1.36 | 4.00 / 3.26 | 3.2 / 2.4 |
| 32 / 35 | **v4** | 0.658 / 0.698 | **1.15 / 1.09** (0.76 cores) | 3.19 / 3.09 | 3.2 / 1.8 |

- The physics in the main thread is 6.5x cheaper than with the CAD meshes. With v4 it still needs about 1.1 ms of
  CPU per 1 ms step on the slope (measured with 1.8-3.2 other cores busy; CPU time per step may be lower on an
  idle machine), so RTF 0.9 on the slope is unlikely even then; on flat, rock and turn the whole server needs
  2.6-2.8 CPU-s per simulated s with v4 (original at flat 4.0-5.4).
- The server never used more than 2.3 cores, and the main thread waits about 25 % of the time (render/sensor
  synchronisation), so on a shared machine RTF also depends on free CPU: the same v4 slope drive gave 0.44 with
  4.9 other cores busy and 0.70 with 1.8.
- What is left in the main thread is the four wheel cylinders and the boxes against GImpact terrain tiles whose
  large tilted triangles have tall bounding boxes; the generator-side cap on collision triangle size (build agent)
  targets exactly that. DESIGN 13's "RTF >= 0.9 while driving" needs that cap and a quiet-machine measurement.

## 2. Shutdown without segfault or crash dump (U5)

**Baseline, reproduced** (`EXP/runs/u5_orig_flat`, original wrapper and launch kept in `EXP/orig/`, headless,
flat, one SIGINT to the wrapper's process group = Ctrl+C in its terminal): gz segfaulted about 1 s after the
SIGINT, WSL streamed a **3.39 GB** dump (`wsl-crash-1792966979-224896-_usr_bin_ruby3.2-11.dmp`, 17:22:59-17:23:17),
the log showed `[gazebo-1] Segmentation fault (core dumped)` and `process has died [... exit code 139 ...]`, and
the wrapper exited **19.1 s** after Ctrl+C. I deleted that dump (my run: ruby pid 224896 is the child of my
launch's `sh -c` process 224894; the upstream `gz_sim.launch.py` runs `ruby` under a dash that does not exec).

**Fix.** Two layers:
1. the launch's gz guard (section 5): whatever stops the launch (Ctrl+C on a plain `ros2 launch`, SIGTERM from a
   tool, `on_exit` shutdown), gz only ever gets SIGKILL from the guard or, if the guard dies, from the kernel;
2. the wrapper's stop: SIGKILL the GUI's process group; SIGINT to the guard (-> SIGKILL gz), SIGKILL to any gz
   server still in the launch group after 3 s; then SIGINT to `ros2 launch` so the ROS nodes stop normally;
   bounded waits (15 s, then SIGTERM after re-killing gz, 5 s, then SIGKILL); the stop time is printed.

**Measured after the fix** (`EXP/runs/*/result.json`; one SIGINT to the process group, as a terminal does):

| run | how started | site | READY (console s) | spawners | controllers | D3D12 | Ctrl+C -> all processes gone | dumps |
|---|---|---|---|---|---|---|---|---|
| e2e_01 | wrapper `--headless` (collision v1) | flat | 39.1 (launch +37.0) | 0 / 0 | both active | yes | **1.92 s** | 0 |
| e2e_02 | wrapper `--headless` (v3) | slope | 46.4 (launch +43.7) | 0 / 0 | both active | yes | **2.23 s** | 0 |
| e2e_03 | wrapper `--headless --variant inspect` (v3) | turn | 43.5 (launch +40.1) | 0 / 0 | both active | yes | **1.77 s** | 0 |
| direct_01 | plain `ros2 launch apollo16_sim.launch.py site:=flat` (v3) | flat | 35.0 (launch +33.4) | 0 / 0 | both active | yes | **1.17 s** | 0 |
| e2e_04 | wrapper `--headless` (final default, v4) | flat | 32.0 (launch +29.9) | 0 / 0 | both active | yes | **1.88 s** | 0 |
| u5_orig | original wrapper + launch | flat | 34.5 | 0 / 0 | both active | yes | 19.1 s | **1 (3.39 GB)** |

- In `direct_01` gz itself received the SIGINT (same process group as `ros2 launch`); the guard's SIGKILL
  (`[gz_guard] SIGINT: gz server stopped with SIGKILL after 0.63 s`, 0.63 s = kernel teardown of the 2.8 GB
  process) arrived before gz-sim's own shutdown reached the crash. No `process has died` line in any run.
- The 34 `run_robot_tests.py` launches of this task (build agent's version, which SIGKILLs gz itself) and the two
  U3 runs (stopped by SIGKILL) left no dump either. `wsl-crashes` was empty before my first run and is empty now.
- `controllers` = `list_controllers` called independently by the driver after READY.
- Server peak RSS 2.83-2.88 GB; `libd3d12.so`, `libd3d12core.so`, `libdxcore.so` mapped, no llvmpipe/swrast.
- Startup timeline of e2e_01 (console s): robot created 3.9, sensors ready 34.8 (progress lines at 15.1 and 25.1:
  `still loading the render scene (sim t = 0.00 s)`), joint_state_broadcaster 36.2, velocity controller 37.4,
  `wait_ready` 39.1 (READY). The other runs were slower because other agents' simulations were running.
- The GUI client (`gz sim -g`) was not run here. The wrapper SIGKILLs its process group; closing the GUI
  window yourself is not covered (it may segfault and dump like the server).

## 3. Wrapper `scripts/run_apollo16_sim.sh`

- **Arguments (U4 / CR-10).** `need_val <opt> <n>` checks the count and that no value starts with `--`
  (negative numbers stay valid), so `--site`, `--pose 1 2` or `--site --headless` fail at once with exit 2
  and `ERROR: --site needs 1 value(s)` / `--site is missing a value (got the option '--headless')` plus
  `See: run_apollo16_sim.sh --help`. Values are validated: `--variant` ('' | inspect | stock | inspect_stock),
  `--robot-collision` (simplified | original), `--pose` numbers, `--gui-config` file exists, not together with
  `--default-gui`, extra arguments must be `name:=value` (also accepted without `--`). `--help` prints the
  header comment only (the old one ended in a code line).
- **New options:** `--world-info PATH` (U10), `--robot-collision M`, `--gui-config FILE`, `--default-gui`,
  `--dry-run` (resolves and checks everything, prints the banner and the exact `ros2 launch` / `gz sim -g`
  commands, starts nothing; the "already running" check is skipped).
- **Environment:** `source scripts/env.sh --quiet` (section 4), so the wrapper and a second terminal cannot
  drift apart.
- **Resolution (U7, U10, U11):** the python step calls the launch file's `resolve_world(...,
  check_plugin=True)`, `resolve_pose`, `world_gravity` and `gui_config_for_pose`; the launch then gets the
  **resolved** world file (one decision for both). Errors go to stderr before any banner, with the next step:
  `world file not found: ... Build the world: cd <repo>/src/lunar_world_gen && ~/.venvs/lunar_world/bin/python
  -m lunar_world_gen.build --preset full`; missing world_info: `pass an explicit pose (wrapper: --pose X Y Z
  [YAW]; launch: robot_x/y/z:=) or the file (wrapper: --world-info PATH; launch: world_info:=)`; unknown site:
  the available sites. Fallback notes are printed as `WARNING:` lines on stderr after the banner:
  `physics plugin <so> cannot be loaded (<loader message>) -> falling back to apollo16_core_stock.sdf (stock
  classic Bullet, 4 ms step: in-place turns reach only ~45-66 % of the commanded rate and mesh friction is fixed
  at 0.5). Build the plugin once: bash <repo>/src/lunar_world_gen/tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh`.
- **Banner:** adds `Collision:`, the GUI config in use, and `Gravity: 1.62 m/s^2 -> SLAM: DLIO odom/gravity
  must be 1.62 (seyond_dlio_graph.yaml has 9.82)` (from `world_info.yaml`).
- **After start (U9):** `Starting up: wait for "[apollo16_sim] READY" (about 35-40 s ...)` instead of
  "Simulation running"; `Other terminal: source .../scripts/env.sh`; ground truth hint `gz model -m togo -p`
  (one shot) instead of the 50 Hz `gz topic -e -t /model/togo/pose`.
- **GUI (U6):** with `config/gui/apollo16.gui.config` present (docs agent), the GUI runs `gz sim -g
  --gui-config <per-run copy>` (option name from `gz sim --help`: `--gui-config [arg]`). The copy has the
  config's `<camera_pose>` (a chase view of the `flat` spawn) moved to the chosen spawn: offset turned by the yaw
  difference, height moved by the spawn-z difference, camera yaw turned, pitch kept (`gui_config_for_pose`,
  tested to keep distance, height, pitch and bearing relative to the rover's heading). `--gui-config FILE` uses a
  file as is, `--default-gui` Gazebo's own (`~/.gz/sim/8/gui.config`, far clip 1000 m). The copy is removed at
  exit. The GUI itself was not started (no GUI windows in this task); dry-run tests cover the command.
- **Stop (U5):** section 2. Traps `EXIT INT TERM HUP`; a second Ctrl+C during the stop forces SIGKILL.

## 4. `scripts/env.sh` (U2)

`source <repo>/src/lunar_world_gen/scripts/env.sh [--quiet]` gives a terminal the sim's environment: ROS Jazzy +
the native install (`TOGO_WSL_INSTALL`), `ROS_DOMAIN_ID` 29, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
`CYCLONEDDS_URI=file://<repo>/config/cyclonedds.xml`, `ROS_AUTOMATIC_DISCOVERY_RANGE`, `GZ_SIM_RESOURCE_PATH`
(togo_gz share + `/opt/ros/jazzy/share`), `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`,
`/usr/lib/wsl/lib` first in `LD_LIBRARY_PATH`, `DISPLAY`/`WAYLAND_DISPLAY`/`XDG_RUNTIME_DIR`. Values already set
win (as in `togo_sim_wsl.sh`); `GZ_PARTITION` is not touched. Safe to source twice (paths added once, tested),
keeps the caller's `set -u`, returns 1 with a message instead of exiting the shell, and refuses to run when
executed instead of sourced. The usage comment is at the top of the file.

Finding while testing: ROS Jazzy's `ros_environment` hook sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` when it is
unset, so the `${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}` default in `togo_sim_wsl.sh` (and the old wrapper)
never applies: the stock chain has always run with SUBNET. `env.sh` keeps that exact behaviour (no DDS change
against what was validated); `cyclonedds.xml` pins DDS to the loopback interface with peer 127.0.0.1 either way.

## 5. Launch file changes in detail

- **gz server under a guard (U5).** The server is no longer included through `ros_gz_sim gz_sim.launch.py`:
  the launch builds the same command and environment itself (it loads the installed `gz_sim.launch.py` as a
  module for `GazeboRosPaths.get_paths()` and `get_executable_path('gz')`): `ruby <gz> sim -r -s -v 4 <world>
  --force-version 8`, `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH` as upstream, process name `gazebo`,
  `on_exit=Shutdown()`. It runs as `python3 apollo16_sim.launch.py --gz-guard -- <command>`:
  - SIGINT / SIGTERM / SIGHUP to the guard -> SIGKILL to gz, guard exits 0 (log: `[gz_guard] SIGINT: gz server
    stopped with SIGKILL after 0.63 s`);
  - the child gets `PR_SET_PDEATHSIG=SIGKILL` (a SIGKILLed guard takes gz with it), and the guard kills gz when
    `ros2 launch` disappears (reparenting, checked every 0.5 s);
  - exit code: 0 after a requested stop, else gz's (128 + n if it died of signal n), 127 if it cannot start;
  - gz stays in the launch's process group, so tools that look for it there (`run_robot_tests.py`: RSS,
    D3D12 check) still find it.
  Blocking SIGINT/SIGTERM inside gz was tried first and does not work: ruby clears the signal mask at start
  (SigBlk 0 in all 31 server threads although the guard's child blocked both, `EXP/logs/t_guard_mask`).
  `gz_guard:=false` restores the upstream include.
- **Readiness (U9):** `wait_for_sensors` prints `still waiting for <topic> ...` / `still loading the render scene
  (sim t = 0.00 s), 22 s` every 10 s. After the velocity-controller spawner, `wait_ready` (rclpy) checks both
  controllers `active` via `list_controllers` and that `/clock` advances 0.2 s, then the launch prints a block
  starting with `[apollo16_sim] READY at +37.0 s: joint_state_broadcaster and platform_velocity_controller
  active, /clock running` (drive topic, sensor topics, `source env.sh`, DLIO gravity). On failure
  `[apollo16_sim] WARNING: NOT READY ...` with the commands to check / spawn. A spawner exit code != 0 adds
  `WARNING: <controller> spawner exited with code N; the controller may still become active - check with: ros2
  control list_controllers (after source .../env.sh)`. The lines that tools parse (`sensors ready`, `<ctrl>
  spawner exited with code N`, `WARNING: wait_for_sensors exited`) are unchanged.
- **World resolution:** variants `''`, `inspect`, `stock`, `inspect_stock`; `check_plugin:=true` (default) runs
  `plugin_loadable()`; messages as in section 3.
- **New launch arguments:** `robot_collision` (simplified), `robot_collision_spec` (''), `gz_guard` (true),
  `check_plugin` (true), `ready_timeout` (60 s).

## 6. U3: `src/togo_sim_wsl.sh` with `TOGO_WORLD` (facts for the docs agent)

Reproduced twice with the user's script unchanged (`EXP/scripts/u3_runs.sh`, `EXP/runs/u3_{default,flatpose}`):
`TOGO_WORLD=/home/nargon/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf TOGO_GZ_GUI=false
TOGO_RVIZ=false`, `GALLIUM_DRIVER=d3d12` exported as the user's `~/.bashrc` does, my partition/domain (the
script honours `ROS_DOMAIN_ID`). Stopped by SIGKILL of all its processes (its own Ctrl+C cleanup would SIGINT gz:
segfault and dump, as in section 2). Pose snapshots with `gz model -m togo -p`.

**What works.** The absolute path is used as is (`start_world.launch.py` joins it with `os.path.join`); the
patched plugin loads (`Loaded [gz::physics::bullet::Plugin] from library [...lunar-plugin.so]` at +3.2 s); D3D12
is used; both controllers are `active` at +110 s (`list_controllers`); RSS 2.98 GB.

**1. A spawner always exits 1, although both controllers end up active (timeline of run 1, s after start).**
- `sim_gz.launch.py`: `create` on a 6 s timer, `control.launch.py` (robot_state_publisher **and** both spawners,
  in parallel) on a 10 s timer (`controller_spawn_delay`). `create` waits for `/robot_description`, which only
  appears when that robot_state_publisher starts, so the robot is created at +12.9 s, together with the spawners.
- The spawners use the Jazzy defaults `--switch-timeout 5`, `--service-call-timeout 10` (3 attempts) and a
  shared file lock (`$ROS_HOME/locks/ros2-control-controller-spawner.lock`, 20 s per attempt).
- The lock winner (`joint_state_broadcaster` here) loads, configures and asks to activate at +13.5 s; the
  controller manager switches it in its next cycle (+13.9 s `Successfully switched controllers!`).
- At that moment the robot's Sensors system starts building the render scene (`Rendering Thread initialized`
  +18.3 s, LiDAR advertised +43.3 s) and the simulation does not step for ~29 s. No service response reaches the
  spawner during the stall (the switch itself is logged as done at +13.9 s; I did not trace why the reply is held
  back): it times out after 10 s and sends the request again (+23.5 s, +33.5 s: `Failed getting a result from
  calling /controller_manager/switch_controller in 10.0. (Attempt 1 of 3.)`).
- When the simulation runs again (+42.9 s) the manager processes the queued repeats: `Controller with name
  'joint_state_broadcaster' is already active.` / `Aborting, no controller is switched! (::STRICT switch)`. The
  spawner prints `Failed to activate controller : joint_state_broadcaster` and dies with exit code 1 (+43.4 s).
- The other spawner waited on the lock (`Failed to acquire lock in 20 seconds. Attempt 1 of 5`), gets it at
  +43.2 s and activates `platform_velocity_controller` at +43.9 s.
- In run 2 the velocity-controller spawner won the lock and was the one that died (exit 1 at +42.1 s); the
  broadcaster activated at +42.6 s. Which spawner fails is a race; one always does on this world.
- `controller_spawn_delay` cannot avoid it: the robot (whose sensors cause the stall) is created only when the
  same delayed include starts robot_state_publisher, i.e. together with the spawners. Our launch starts
  robot_state_publisher and `create` at t = 0 and the spawners only after the stall (`wait_for_sensors`).

**2. The default pose drops the rover 2.7 m and nothing sets the yaw.** Defaults `-2.0, 3.0, 2.0`: the rover hangs
at z = 2.000 during the stall (+15.7 s and +44.2 s snapshots), then falls 2.73 m at 1.62 m/s^2 (~1.8 s, ~3 m/s at
impact) and ends at (-1.413, 3.238, -0.755): 0.63 m from the spawn x/y, yaw -15.1 deg, roll -2.3, pitch +5.1 deg,
still creeping ~3 mm/s at +100 s. With `TOGO_ROBOT_X=-4.5 TOGO_ROBOT_Y=-0.5 TOGO_ROBOT_Z=-0.434` (the `flat` site)
it settles 6 cm lower within 3 cm of x/y, but faces east (yaw 0) instead of the site's 135 deg: neither the
script nor `sim_gz.launch.py` has a yaw input (`create` gets no `-Y`).

**3. No fallback** when the patched plugin is missing or unloadable: the SDF names it by absolute path and goes to
gz unchecked (reviewer T3: one `[Err]`, simulation without physics). Our launch falls back to the stock variant.

**4. Shutdown.** Its `cleanup` sends SIGINT to the launch's process group and then `wait`s without a limit: gz
segfaults, WSL writes a 1.5-3.6 GB dump, 15-25 s (my baseline of the same pattern: 19.1 s, 3.39 GB).

**5. Smaller differences.** RViz defaults to true; the GUI uses `~/.gz/sim/8/gui.config` (far clip 1000 m);
`GALLIUM_DRIVER` is not set by the script (it relies on `~/.bashrc`; without it Mesa may use llvmpipe); its
"already running" check only matches `sim_gz.launch.py` (our wrapper checks both); `ROS_AUTOMATIC_DISCOVERY_RANGE`
is effectively SUBNET (section 4).

Suggested README wording: "The old launcher loads the world with an absolute `TOGO_WORLD`, but on this world one of
its two controller spawners always reports `Failed to activate controller` / `exit code 1` (the controllers are
active anyway: check `ros2 control list_controllers`), it cannot set the yaw (set `TOGO_ROBOT_X/Y/Z` from
`world_info.yaml`, else the rover drops 2.7 m), it has no fallback when the patched plugin is missing, and Ctrl+C
costs a crash dump. Use `scripts/run_apollo16_sim.sh`."

Observation (not investigated, not my files): in both stock runs the parked rover crept 0.4-3 mm/s with both
controllers active (flat site: 27 mm in 50 s; default spot: 0.17 m in 50 s). I did not measure this with our
launch; worth a check by whoever owns the drive validation, since SLAM assumes a parked rover is still.

## 7. Tests

49 new tests, all passing (venv pytest in WSL; the dry-run and env.sh tests need ROS Jazzy and the native install
and are skipped without them; the dlopen tests need gcc):

- `tests/test_sim_scripts.py` (25): every missing or bad value exits 2 within the 10 s test timeout with the
  right message (17 cases: `--site`, `--world`, `--variant`, `--pose 1 2`, `--world-info`, `--robot-collision`,
  `--gui-config`, `--site --headless`, `--pose 1 --headless 2`, bad variant / mode / pose / file, both GUI
  options, unknown option, bare word, extra argument without `:=`); `--help`; dry run with a fake world and a real
  gcc-built plugin (resolved world, site pose, explicit pose with negative values and yaw, extra arguments,
  caller's `ROS_DOMAIN_ID` wins, gravity line); broken plugin + inspect -> `w_inspect_stock.sdf` with the WARNING
  on stderr, and the error when no stock sibling exists; errors with next steps (missing world, unknown site with
  the list, missing world_info); GUI config choice (`--gui-config`, `--default-gui`, packaged config with the
  camera moved); `env.sh` values, idempotence, `set -u` kept, caller's values win, errors return instead of
  exiting, refusal when executed; `bash -n`.
- `tests/test_sim_launch.py` (19): variant names incl. `inspect_stock`; inspect fallback order; garbage,
  missing-`NEEDED`-library and undefined-symbol plugins (gcc) detected by `plugin_loadable`; pose messages; rpy
  round trip incl. gimbal lock; collision substitution frames (500 points of a cylinder along the mesh y axis, placed
  by a rotated origin, lie in the substituted cylinder), scope (visuals, inertials, joints, plugins, comments,
  other scales untouched), changed/missing mesh kept; description modes; the checked-in spec is consistent and
  still bounds the installed meshes (sha256 + containment of every vertex); GUI camera keeps distance, height,
  pitch and bearing relative to the rover; gz guard: SIGINT/SIGTERM/SIGHUP -> child killed by SIGKILL within 3 s
  and never sees the signal, exit codes (3, 128+SIGUSR1, 127), child dies with a SIGKILLed guard (PDEATHSIG),
  guard and child exit when the parent `ros2 launch` is SIGKILLed; new launch arguments declared.
- `tests/test_sim_robot_collision.py` (5): minimum enclosing circle; box and cylinder fits; slab boxes bound a
  synthetic arched plate (vertices and random surface points), follow the arch (40 mm -> < 20 mm with 8 slabs,
  < 10 mm with 16), selection by the gap limit and by the down axis; down axes from a URDF joint chain (yaw only:
  -z; the LiDAR mounting: +y; `<ros2_control>` joints ignored); compound substitution in the launch.

Whole package: `pytest tests -q` -> 266 passed (5 min 7 s, niced, while simulations ran; other agents' tests
included). The existing `tests/test_world_sdf.py` launch tests pass unchanged (`resolve_world` keeps its old
behaviour unless `check_plugin=True`).

## 8. Notes for the docs agent and the orchestrator; proposed config

**Docs agent** (README / USER_GUIDE, checked against the README of 16:57):
- Troubleshooting row "Segmentation fault ... at shutdown": a plain `ros2 launch .../apollo16_sim.launch.py` also
  avoids it now (gz guard; measured 1.17 s, no dump), not only the wrapper.
- Section 7 (`TOGO_WORLD`): use the facts and the wording of section 6 here. "Spawners start on a fixed 10 s
  timer" is right but the robot is created at the same moment (+12.9 s), and *one* of the two spawners fails
  (a race on the spawner lock); `controller_spawn_delay` does not help (section 6.1).
- `config/gui/apollo16.gui.config` comment says "For another site, orbit the view": the wrapper now moves the
  camera to the chosen spawn (translation + yaw), so that hint only applies to a manual `gz sim -g --gui-config`.
- New wrapper options to document: `--robot-collision original` (CAD meshes; slow on slopes), `--world-info`,
  `--gui-config`, `--default-gui`, `--dry-run`, `--variant inspect_stock`; second Ctrl+C forces SIGKILL.
- Simplified collisions (one paragraph): 88 bounding boxes replace the robot's 14 CAD collision meshes; visuals,
  mass and inertia unchanged; the boxes reach at most 14.5 mm further towards the ground than the CAD meshes
  (the sensor arch on top excepted, where its opening is filled);
  physics about 6x cheaper per simulated second on slopes (RTF there about 0.7 instead of 0.13 on this shared
  machine); regenerate with `~/.venvs/lunar_world/bin/python tools/robot_collision_simplify.py` after a Clearpath
  update (the launch keeps a changed mesh and warns).
- `env.sh`: `ROS_AUTOMATIC_DISCOVERY_RANGE` is SUBNET in practice (ROS's hook), same as `togo_sim_wsl.sh`.
- READY appeared at launch +33.4 to +43.7 s in my runs (other agents' simulations running); "about 35-40 s" is fair.

**Orchestrator.**
- `proposed_config`: none. The runtime code reads no generator config; every new behaviour is a launch argument
  or wrapper option with a default (`robot_collision:=simplified`, `gz_guard:=true`, `check_plugin:=true`,
  `ready_timeout:=60`, spec path default `config/robot/a300_collision_simplified.yaml`).
- Robot runs by other agents with the shared launch file used simplified collisions from 16:24 (v1, 7 single
  boxes), from 17:12 (v3, 54 boxes) and from 17:38 (v4, 88 boxes, final) unless they passed
  `robot_collision:=original`; their RTF and drive numbers after 16:24 include this change.
- Deviation from the task wording: all 14 collision meshes are replaced by default, not only those above a few
  thousand triangles (measured reason in 1.1 / 1.4; `--min-triangles 2000` restores the narrower set).
- `tools/run_robot_tests.py` records `--launch-arg` values in no field: label such runs (`--label`) if the
  collision mode matters.
- The flat-site in-place turn metric (DESIGN 13, >= 0.85) spreads 0.84-0.93 between identical runs (section 1.3);
  the quiet-machine validation should repeat it several times or judge the turn on the `turn` site.
- Open (not mine): the parked rover creeps 0.4-3 mm/s in the stock chain (section 6); `inspect_stock` variant in
  the main world (build agent); RTF >= 0.9 on slopes needs the collision triangle cap and a quiet machine (1.4).

## 9. Files, reproduction, cleanliness

**Files** (all in `src/lunar_world_gen`): `launch/apollo16_sim.launch.py` (rewritten), `scripts/run_apollo16_sim.sh`
(rewritten), `scripts/env.sh`, `tools/robot_collision_simplify.py`, `config/robot/a300_collision_simplified.yaml`
(generated), `tests/test_sim_launch.py`, `tests/test_sim_scripts.py`, `tests/test_sim_robot_collision.py`,
this file. Outside the package only my scratch dir (`EXP`, 26 MB; originals of the old wrapper and launch in
`EXP/orig/`) and `~/togo_assets/lunar/worlds/fix_runtime/` (a copy of `apollo16_core.sdf` with
`real_time_factor 0.45`, `world_info.yaml`, symlinks to the main world's meshes and textures). No user file, no
main build or world file was written.

**Reproduce** (WSL, from `src/lunar_world_gen`; `E=$EXP`):

```bash
~/.venvs/lunar_world/bin/python tools/robot_collision_simplify.py              # spec, 13 s (--min-triangles 2000: v3)
~/.venvs/lunar_world/bin/python -m pytest tests/test_runtime_*.py -q           # 49 tests
scripts/run_apollo16_sim.sh --dry-run --site slope                             # resolution, commands, nothing started
source $E/scripts/rosenv.sh                                                    # own GZ_PARTITION / ROS_DOMAIN_ID / ROS_HOME
/usr/bin/python3 $E/scripts/drive_wrapper.py --out OUT --wrapper scripts/run_apollo16_sim.sh --hold 5 -- --headless --site flat
/usr/bin/python3 tools/run_robot_tests.py --out OUT --world ~/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf \
    --sites slope --repeats 1 --partition P --domain D [--launch-arg robot_collision:=original]
bash $E/scripts/thread_series.sh "1|v4|simplified|" "2|orig|original|"       # slope + per-thread CPU
bash $E/scripts/u3_runs.sh default flatpose                                    # U3 (stopped by SIGKILL)
python3 $E/scripts/analyze_rtf.py; python3 $E/scripts/cpu_per_sim.py           # tables of section 1
```

**Cleanliness.** 43 gz servers started in this task (34 robot tests, 6 wrapper/direct/baseline runs, 2 U3 runs,
1 tiny guard test); none got SIGINT/SIGTERM except the deliberate U5 baseline, whose 3.39 GB dump I deleted.
`wsl-crashes` is empty; no process with `GZ_PARTITION=claude_fixruntime` is left; no ros2 daemon was started
(controllers were listed through rclpy). No git command was run.
