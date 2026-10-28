# Usability review: running the Apollo 16 world (phase 3)

Reviewer: `review-usability` agent, 2026-10-25. This was a read-only review. No project file was edited, and this document is the only file written in the repo. Scratch files, scripts, logs and images are in `/home/nargon/togo_assets/lunar/experiments/p3_review-usability/`, called `EXP` below.

**Scope.** How the user will actually run the result:

- `scripts/run_apollo16_sim.sh`, `launch/apollo16_sim.launch.py`, `world_info.yaml` and the docs;
- the existing workflow: `src/togo_sim_wsl.sh` with `TOGO_WORLD`, the Docker SLAM stack, and DLIO gravity;
- the Gazebo GUI;
- failure messages and recovery;
- which files outside `src/lunar_world_gen` changed during this session.

**Rules followed.**

- No GUI window and no RViz were started.
- Gazebo runs used their own `GZ_PARTITION` (`claude_p3_review_usability_*`), `ROS_DOMAIN_ID=97` and a private `ROS_HOME`.
- Every run used `setsid` and `timeout`, and was stopped by process group.
- No processes are left: I checked for any process whose environment carries my partition, and found none.

## 0. Verdict

The new entry point works on this machine.

- `run_apollo16_sim.sh --headless` loaded the real world with the patched plugin and the D3D12 GPU, and spawned the rover at `flat`.
- Both controllers came up with spawner exit codes 0/0 at +33.8 s.
- The rover settled within 6.4 cm of the site pose, with a yaw error of 0.3 deg.
- It publishes the same topics as the stock chain: `/husky/sensors/seyond/points`, `/husky/sensors/imu_0/data_raw`, `/clock` and the GPS topics, all from the same `bridge.yaml`.
- It uses the same ROS domain (29), CycloneDDS file and TF publishers.

What a user hits first is mostly **documentation and workflow edges**:

1. **SLAM gravity (U1, high).** DLIO's `odom/gravity` is 9.82, but the world uses 1.62. This appears only in research reports, and DESIGN §12 says the SLAM stack is unchanged.
2. **No quick-start (U2).** There is no single first-run guide, and the venv cannot be recreated from the repo.
3. **The `TOGO_WORLD` path (U3) half-works.** Measured on this world, the stock chain gives a `joint_state_broadcaster` spawner exit 1, a 2.8 m drop from the default pose, no yaw, and no fallback when the plugin is missing.
4. **The wrapper hangs (U4)** in an infinite loop when an option is given without its value.
5. **Every Ctrl+C (U5)** makes gz segfault, as known. WSL then writes a **1.5-3.6 GB crash dump** to `%TEMP%\wsl-crashes`, which takes 15-23 s. That folder held 22 GB at the time of writing.
6. **GUI (U6).** The user's GUI config has no `camera_clip`, so the far clip is 1000 m: the 20 km horizon is never visible. The rover is also outside the initial view.

## 1. What was run

| id | what | result (details in `EXP/<id>`) |
|---|---|---|
| T1 | wrapper argument and failure cases (`t1_wrapper_args.sh`); no launch happens | `--site`, `--world` or `--pose 1 2` without values: **hang**, killed by `timeout` after 5 s (rc 124). Bad site, bad variant, unknown option and missing world give clear errors (rc 1/2). |
| T2 | plugin missing, via the launch file's own `resolve_world` on SDF copies (`t2_plugin_missing.py`) | default and `inspect` both fall back to `apollo16_core_stock.sdf` with a NOTE; with no `_stock` sibling, a clear error |
| T3 | tiny server-only worlds with a missing, a broken and the real engine path (`t3_engine_failure.sh`) | missing path: one `[Err] Failed to find plugin`. Broken library: one `[Err] Unable to load the [...] library`. In both, **sim time runs (4.3-4.4 s) but nothing moves** (box stays at z = 2.000). Real plugin: the box falls to z = 0.12. |
| T4 | GUI view rendered headless with camera sensors at the GUI camera pose (`t4_gui_views.py`), realistic and inspect lighting | first frame at +21.8 / +23.0 s, server HWM 2.65 GB, D3D12 mapped; images in `EXP/t4/{realistic,inspect}/` |
| T5 | **user's `src/togo_sim_wsl.sh`** with `TOGO_WORLD=<abs apollo16_core.sdf>`, `TOGO_GZ_GUI=false`, `TOGO_RVIZ=false`, default pose (`t56_launch_probe.py`) | see U3; console log `EXP/t5_stock_togo_world/console.log` |
| T6 | **`run_apollo16_sim.sh --headless`**, site `flat` (`t56_launch_probe.py`) | see §0 and U5; `EXP/t6_wrapper_headless/console.log` |

**Concurrency, for the orchestrator.** My Gazebo runs overlapped some of the gz-validation agent's runs. Their RTF and timing in these windows should be treated as contended. My own RTF samples are not reported as world numbers. The overlaps:

- T4 realistic (13:40:00-13:40:32) overlapped the end of their bench `flat_real` (round 2, RTF 0.993, passed) and the load phase of `crater_real` (load 21.0 s, normal).
- T4 inspect (13:44:23-13:45:00) overlapped their LiDAR capture (13:44:41-13:45:14; first LiDAR at 23.5 s, other runs about 20 s).
- T5 (13:45:32-13:47:26) overlapped their aborted robot launch `robot_aborted_1` (13:46:18-13:47:09).
- T6 (13:48:03-13:50:07) started in the same second as their `robot_patched` launch 0 (`flat`, drive_pass False). Their uncontended launch 4 at `flat` also failed the drive, so that failure is not only contention.

## 2. Findings

Severity: **H** = users will get wrong results or be blocked; **M** = significant friction or a misleading behaviour; **L** = polish; **I** = information.

### U1 (H): DLIO gravity must be 1.62, but no user-facing document says so

**Evidence**

- The world has `<gravity>0 0 -1.62</gravity>`, and `world_info.yaml` has `gravity_m_s2: 1.62`.
- `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml:70-72` sets `odom/gravity: 9.82`, with the comment "Must match lunar_surface.world. Temporarily set to Earth gravity…".
- The user's own skill note (`.claude/skills/togo-sim-slam-nav/SKILL.md`, "Gravity is coupled") says the two must change together.
- In this package the coupling is mentioned only in `docs/research/legacy_review.md` (B20, §292) and `rover_baseline.md`.
- The wrapper banner, `world.md` §3, `integration.md`, DESIGN §12 and `world_info.yaml` say nothing about it.
- DESIGN §12 even says: "Unchanged. The user's existing scripts, the SLAM stack and topic names stay the same."

**Why it matters (from DLIO's code)**

`direct_lidar_inertial_odometry/src/dlio/odom.cc:1899-1934` (`approximateGravity` and accelerometer calibration are both on in the YAML) computes `b_accel = accel_avg − normalize(accel_avg)·|odom/gravity|`.

- With the IMU reading 1.62 at rest, DLIO stores a body-frame bias of **−8.2 m/s²**. It prints `Accel biases [xyz]: ~0, ~0, -8.2`.
- Once the rover tilts by θ, that bias leaks 8.2·sin θ m/s² into the horizontal propagation: 0.72 at 5°, 1.42 at 10°, and **1.75 m/s² on the 12.3° `slope` site**.
- For comparison, the rover's real acceleration is at most 0.65 m/s².
- This was not run here, because SLAM runs in Docker.

**Fix**

1. Add the README "SLAM" section (§3 below).
2. Add a banner line to the wrapper, after the `Clock:` line:

   ```bash
   echo "Gravity:     1.62 m/s^2 -> set DLIO odom/gravity to 1.62 (seyond_dlio_graph.yaml has 9.82); see README"
   ```

   Better still, print `world_info.yaml: gravity_m_s2` from the resolution step.
3. Append to DESIGN §12 "Unchanged": "Exception: DLIO `odom/gravity` must equal the world gravity (1.62); the shared `seyond_dlio_graph.yaml` has 9.82. Use the override in README → SLAM."
4. Give the user a no-edit override. Verified: exactly one line changes, and the YAML parses with `odom/gravity = 1.62`. `run_live_seyond_dlio_slam.sh` already honours `COMBINED_PARAM_FILE`, which it uses for both `slam_param_file` and `dlio_param_file`:

   ```bash
   # inside the SLAM container (dev)
   P=/home/er4-user/slam_ws/src/lidarslam_ros2
   sed -E 's/^([[:space:]]*odom\/gravity:[[:space:]]*)[0-9.]+/\11.62/' \
       $P/lidarslam/param/seyond_dlio_graph.yaml > /tmp/seyond_dlio_graph_moon.yaml
   grep -n 'odom/gravity' /tmp/seyond_dlio_graph_moon.yaml     # 72:    odom/gravity: 1.62
   COMBINED_PARAM_FILE=/tmp/seyond_dlio_graph_moon.yaml $P/scripts/togo/run_live_seyond_dlio_slam.sh
   ```

5. Also document: **start SLAM only after the sim reports both controllers active.** DLIO calibrates the IMU on its first 3 s of data (`imu/calibration/time: 3.0`), and the rover spawns 4 cm above the ground. On the stock chain's default pose it falls 2.8 m.

### U2 (M): there is no first-run guide, and the venv cannot be reproduced

**Evidence**

- `src/lunar_world_gen` has no README, `requirements.txt` or `pyproject`.
- The run and build steps are scattered over `docs/impl/world.md` §3 (plugin and run), `integration.md` §2 (build), `validation.md` and `rocks.md` (library).
- `PROGRESS.md` lists the venv packages without versions or a creation command.
- The direct `ros2 launch …` path in `world.md` §3 and the `rover_drive_test.py` / `ros2 control list_controllers` hints assume the sim's DDS environment.
- The user's `~/.bashrc` sources ROS and sets `GALLIUM_DRIVER`, but sets no `ROS_DOMAIN_ID`, RMW or `CYCLONEDDS_URI`. A second terminal therefore talks on domain 0 with the default RMW, and `ros2 control list_controllers` finds no controller manager.

**Fix**

1. Add `src/lunar_world_gen/README.md` with the text in §3.
2. Pin the venv. These are the measured versions in `~/.venvs/lunar_world` (Python 3.12.3):

   ```text
   numpy==2.5.4 scipy==1.18.1 rasterio==1.5.2 pyproj==3.8.0 trimesh==5.2.0 pydelatin==0.4.0 pyfqmr==0.5.0
   numba==0.68.0 llvmlite==0.50.0 pygltflib==1.16.5 matplotlib==3.11.2 PyYAML==6.0.3 tqdm==4.70.1
   pytest==9.1.1 rosbags==0.11.9
   ```

3. Add a sourceable `scripts/apollo16_env.sh` with the wrapper's environment block (ROS, native install, `ROS_DOMAIN_ID=29`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `CYCLONEDDS_URI=file://$REPO/config/cyclonedds.xml`, `GALLIUM_DRIVER=d3d12`).
4. Have the wrapper banner print `In another terminal: source <repo>/src/lunar_world_gen/scripts/apollo16_env.sh`.

### U3 (M): `TOGO_WORLD` with `src/togo_sim_wsl.sh` half-works

`PROGRESS.md` says: "The new world runs through the existing launcher by setting `TOGO_WORLD` to an absolute path." That is only partly true.

**Evidence: T5, user's script unchanged, `TOGO_WORLD=/home/nargon/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf`, default pose**

- The absolute path works (`os.path.join` keeps it). The patched plugin loaded at +3.3 s, the D3D12 renderer was used, and there were 0 `[Err]` lines.
- **The spawner failure is reproduced.**
  - The spawners started on the fixed timer at +12.9 s and the robot was created at +13.0 s.
  - The render thread started at +18.4 s, and the LiDAR was advertised only at **+38.4 s**.
  - There were `switch_controller` timeouts at +24.0 and +34.0 s.
  - At +36.3 s: `[joint_state_broadcaster]: Failed to activate controller : joint_state_broadcaster`, then `[ERROR] [spawner-6]: process has died [... exit code 1 ...]`.
  - Both controllers were nevertheless `active` at +90 s (checked through the `list_controllers` service).
- **Default pose (-2, 3, 2.0).**
  - The rover dropped 2.76 m (about 1.85 s of fall, about 3 m/s at impact).
  - It ended at (-1.357, 3.252, -0.761): **0.69 m from the spawn x/y, yaw −14.5°**, roll/pitch −2.9°/4.0°.
  - The script has no yaw input (`TOGO_ROBOT_YAW` does not exist).
- **No fallback.** The world names the patched plugin by absolute path. Without it, the stock chain loads a world with no physics: T3 shows one red `[Err]` line and a frozen scene. The robot would hover and never drive.
- RViz defaults to true in `togo_sim_wsl.sh` and false in our wrapper. `togo_sim_wsl.sh` does not detect a running `run_apollo16_sim.sh` (its `pgrep` only matches `sim_gz.launch.py`), so it would start a second sim in the same partition and domain.

**Fix: replace the PROGRESS ground-rule line with**

> The new world also loads through the existing launcher with an absolute `TOGO_WORLD` (plus `TOGO_ROBOT_X/Y/Z` from `world_info.yaml`), but that chain starts the controller spawners on a fixed 10 s timer (measured on this world: `joint_state_broadcaster` spawner exit 1, both controllers still active at +36.8 s), cannot set the yaw, drops the robot from z = 2 m by default and has no stock fallback when the patched plugin is missing. `scripts/run_apollo16_sim.sh` is the supported entry point.

Add the README `TOGO_WORLD` section (§3). Optionally, the wrapper could honour `TOGO_GZ_GUI`, `TOGO_RVIZ` and `TOGO_ROBOT_X/Y/Z` as defaults, so the user's muscle memory works.

### U4 (M): the wrapper loops forever on an option without its value

**Evidence (T1).**

- `run_apollo16_sim.sh --site`, `--world` and `--pose 1 2` each printed nothing and spun until my 5 s `timeout` (rc 124).
- The cause: `shift 2` / `shift 4` fail when fewer arguments remain, nothing is shifted, and `while [[ $# -gt 0 ]]` repeats forever. `set -u` is only enabled later.

**Fix** (in the option loop; negative numbers such as `-4.5` stay valid):

```bash
need_val() {  # need_val <option> <count> <args after the option...>
  local opt="$1" n="$2"; shift 2
  if (( $# < n )); then echo "ERROR: ${opt} needs ${n} value(s)" >&2; usage >&2; exit 2; fi
  local v; for v in "${@:1:n}"; do
    [[ "${v}" != --* ]] || { echo "ERROR: ${opt} is missing a value (got '${v}')" >&2; exit 2; }
  done
}
    --world)   need_val "$1" 1 "${@:2}"; WORLD="$2"; shift 2 ;;
    --variant) need_val "$1" 1 "${@:2}"; VARIANT="$2"; shift 2 ;;
    --site)    need_val "$1" 1 "${@:2}"; SITE="$2"; shift 2 ;;
    --pose)    need_val "$1" 3 "${@:2}"; POSE=("$2" "$3" "$4"); shift 4
               if [[ $# -gt 0 && "$1" != --* ]]; then POSE+=("$1"); shift; fi ;;
```

Also `usage() { sed -n '2,21p' "$0" | sed 's/^# \{0,1\}//'; }`. Today `--help` ends with the code line `ROS_DISTRO="${ROS_DISTRO:-jazzy}"` (it prints lines 2-23).

### U5 (M): every shutdown writes a 1.5-3.6 GB crash dump to Windows and takes 15-23 s

**Evidence**

- WSL's `core_pattern` is `|/wsl-capture-crash %t %E %p %s`.
- gz-sim 8 segfaults while shutting down after SIGINT (known, documented as harmless).
- WSL streams the core of the `ruby` server process to `C:\Users\Username\AppData\Local\Temp\wsl-crashes\`. It keeps 10 files by default; there were **10 files, 22 GB** at 13:53, from all agents' runs today.
- T5: crash at 13:47:10, the 3.56 GB dump was finished at 13:47:25, so shutdown took 15.7 s.
- T6: crash at 13:49:43, the 3.64 GB dump was finished at 13:50:06. The wrapper's `stop_group` hit its 20 s limit and printed `ros2 launch: still running after 20 s, sending SIGTERM`; the sim stopped after 24.2 s.
- What the user sees: `Segmentation fault (core dumped)` or a `[gz …] exit 139`, and a long pause.
- The GUI client (`gz sim -g`) probably dumps too when closed. This is not verified, because no GUI was started.
- `ulimit -c 0` and `ulimit -c 1` do **not** stop these dumps. My shell had `ulimit -c 0` and all dumps were written. A `sleep` I killed with SIGSEGV under `ulimit -c 1` still produced `wsl-crash-…-166762-_usr_bin_sleep-11.dmp` (446 kB).

**Fix**

1. **(a) Document it.** "Shutdown takes 15-25 s and gz reports a segfault; harmless."
2. **(b) User-side.** Add `[wsl2]` `maxCrashDumpCount=2` to `%UserProfile%\.wslconfig`; this is the documented key, default 10. It caps the disk use at about 7 GB. `crashDumpFolder=` can move the dumps. Either needs `wsl --shutdown`, and it is the user's decision.
3. **(c) Proposed, not tested.** In the wrapper's `cleanup`, SIGKILL the gz server before sending SIGINT to the launch group. A SIGKILL never dumps core, and the server holds no state worth a clean exit. `on_exit_shutdown` then stops the rest. For example: `for p in $(pgrep -g "${SERVER_PID}" -f 'gz sim .* -s '); do kill -KILL "$p"; done` before `stop_group`. Re-test the shutdown time and the `wsl-crashes` folder afterwards.

**My contribution to the folder.** These dumps come from my runs: `162511` (T4 inspect, 1.7 GB), `162990` (T5, 3.6 GB), `164497` (T6, 3.6 GB) and `166762` (the 446 kB test). `161499` (T4 realistic, 2.1 GB) has already been rotated out. I did not delete anything outside my scratch dir.

### U6 (M): GUI far clip of 1000 m hides the 20 km horizon, and the rover is not in the initial view

**Evidence**

- The world SDF has no `<gui>`, so `gz sim -g` loads the user's `~/.gz/sim/8/gui.config`. That 68 kB file was customised on Jun 17, has no EntityTree or ComponentInspector, has `camera_pose -6 0 6 0 0.5 0`, and has **no `camera_clip`**.
- The gz-gui 8 `MinimalScene` defaults are near 0.01 m, **far 1000 m** and HFOV 90° (`MinimalScene.hh`, branch gz-gui8).
- T4 rendered the GUI's view with camera sensors at the same pose and field of view (16:9):
  - **Initial view:** pitch 28.6° down, so the frame is all near terrain (0 % background) and identical for far 1000 and 30000. The `flat` spawn site is 1.6 m from the camera's ground point at 76.6° depression, **below the bottom edge** (58°). The rover is not visible, and the user's config has no EntityTree to find it.
  - **Looking at the horizon** (3 m high, facing south): with far 1000 m the distant terrain is cut, and the skyline falls back to the 1 km edge. Rows 272-321 (1.8 % of the image) are terrain only with the 30 km clip; the hill on the skyline disappears. See `EXP/t4/inspect/contact_horizon_south_far1000_vs_horizon_south_far30000.png`.
  - **Brightness:** the default view averages 44.4/255 with realistic lighting and 76.5/255 with `inspect` lighting. The rover-centred view averages 53.5 and 76.4. The realistic lighting is usable, and `inspect` is about 1.7x brighter.
- Memory, measured on the server: HWM 2.65-2.76 GB without the robot (gz-validation benches) and 3.0-3.1 GB with it (T5/T6). VRAM per server process is about 1.7-2.4 GiB (bench peak minus idle). The GUI client builds its own copy, so expect about **+2.7 GB RAM and +2 GiB VRAM** (an estimate; GUI not run), with peak VRAM near 7 of 8 GiB.
- The GUI only shows the world after its own scene build, about 20 s (the server's first frame was at 21.8-23.0 s).

**Fix (wrapper).** Start the GUI with a per-world copy of the user's own config: the same plugins, plus a clip and a start view. The pose below is the tested rover view: 5.5 m east, 2 m south and 2.3 m above the site, looking at it with the sun behind the camera.

```bash
GUI_SRC="${HOME}/.gz/sim/8/gui.config"
[[ -f "${GUI_SRC}" ]] || GUI_SRC="/opt/ros/${ROS_DISTRO}/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config"
GUI_CFG="$(mktemp --suffix=_apollo16_gui.config)"
python3 - "${GUI_SRC}" "${GUI_CFG}" "${CAM_POSE}" <<'PY'
import re, sys
src, dst, pose = sys.argv[1:4]
t = re.sub(r"<camera_clip>.*?</camera_clip>", "", open(src).read(), flags=re.S)
t = re.sub(r"<camera_pose>[^<]*</camera_pose>",
           f"<camera_pose>{pose}</camera_pose><camera_clip><near>0.1</near><far>30000</far></camera_clip>", t, count=1)
open(dst, "w").write(t)
PY
setsid gz sim -g --gui-config "${GUI_CFG}" &        # --gui-config replaces the SDF/default config (gz-sim 8 Gui.cc)
```

`CAM_POSE` is (x + 5.5, y − 2.0, ground_z_visual + 2.3, roll 0, pitch 0.375, yaw 2.79), using the site's values from `world_info.yaml`. Compute it in the existing python resolution step, since bash arithmetic has no floats. For `flat` it is `1.0 -2.5 1.68 0 0.375 2.79`.

- I checked near 0.1 / far 30000 on camera sensors, with no visible z-fighting.
- The rest of the snippet is untested, because no GUI was started.
- A manual workaround: add the "View Angle" GUI plugin and raise the far clip there.

### U7 (L-M): an unloadable plugin is not detected

**Evidence (T3).** A present but broken `.so` gives one `[Err] [Physics.cc:874] Unable to load the [...] library`. Sim time advances and nothing moves. `resolve_world` only checks `os.path.isfile`, so neither the wrapper nor the launch falls back. Realistic causes:

- a distro or ROS upgrade that removes `libBulletDynamics.so.3.24`, `libgz-physics7.so.7` or `libsdformat14.so.14` (the plugin's `NEEDED` entries);
- a copy of the world to another machine.

A **minor gz-physics update** (7.6.0 → 7.x) keeps the SONAME `libgz-physics7.so.7`, so the plugin should keep loading; this is not testable here. However, `build_lunar_bullet_plugin.sh` then refuses to rebuild ("installed gz-physics is not 7.6.0"), and the mesh-friction patch relies on gz-sim calling `ConstructSdfModel` before `AttachMeshShape`.

**Fix**

1. In `resolve_world`, treat `ldd <plugin>` lines with "not found" like a missing file:

   ```python
   bad = [l.split()[0] for l in subprocess.run(["ldd", engine], capture_output=True, text=True).stdout.splitlines() if "not found" in l]
   ```

   Fall back to `_stock` with a warning that names `bad`.
2. Document the check after updates: a healthy start logs `Loaded [gz::physics::bullet::Plugin] from library [.../libgz-physics7-bullet-lunar-plugin.so]` and **122** `[lunar patch] mesh collision` lines (108 terrain + 14 robot, as in T5 and T6). If they are missing, use `--variant stock` and rebuild or port the patch (bump `VER` in the build script).
3. Optionally, have `wait_for_sensors` check that the robot's z changes after the spawn. The rover settles 6 cm in T6. If it does not move, print `ERROR: physics is not running (see [Err] lines)`.

### U8 (L): the plugin build leaves 4 dangling symlinks in the caller's directory

**Evidence.** These four links were made at 11:51:02, during the world agent's plugin build (built 11:51:17):

- `/home/nargon/libgz-physics-bullet-plugin.so -> libgz-physics7-bullet-plugin.so`
- `/home/nargon/libgz-physics-bullet-featherstone-plugin.so -> libgz-physics7-bullet-featherstone-plugin.so`
- `/home/nargon/libgz-physics-dartsim-plugin.so -> libgz-physics7-dartsim-plugin.so`
- `/home/nargon/libgz-physics-tpe-plugin.so -> libgz-physics7-tpe-plugin.so`

The cause is gz-physics 7.6.0's `bullet/CMakeLists.txt:42` (and the same lines for dartsim, tpe and bullet-featherstone): `EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink …)` with no `WORKING_DIRECTORY`, so it runs in the CWD of `cmake -S … -B …`. Following `world.md` §3 from the repo root would put the links in the OneDrive repo root.

**Fix.** In `build_lunar_bullet_plugin.sh`, run cmake from the build dir: `mkdir -p "${BUILD}" && cd "${BUILD}" && cmake -S "${SRC}" -B "${BUILD}" …`. The four links in `/home/nargon` are dangling and harmless; the user may delete them. I did not touch them.

### U9 (L): readiness and progress messages

- The wrapper prints `Simulation running. Press Ctrl+C to exit.` at +9.4 s, but the controllers became active only at +33.8 s (T6; gz-validation measured 35-39 s per launch).
- During the stall the LiDAR topic appeared at +15.2 s, but sim time resumed only at +31.6 s. The 0.5 s sim-advance check is what correctly gates the spawners.
- `wait_for_sensors` prints nothing for up to 600 s while it waits.

**Fix**

- After the platform_velocity_controller spawner exits with code 0, the launch should log a new line, keeping the existing lines that tools parse:

  ```text
  [apollo16_sim] READY at +T s: controllers active; drive /platform_velocity_controller/cmd_vel (TwistStamped); start SLAM now (DLIO odom/gravity 1.62)
  ```

  Otherwise it should log `[apollo16_sim] WARNING: … spawner exited with code N; check ros2 control list_controllers`.
- The wrapper banner should say "Ready when `[apollo16_sim] READY` appears (about 35 s)".
- `wait_for_sensors` should print a progress line every 30 s: `still waiting for <topic> (render scene loading), 60 s`.
- The ground-truth hint `gz topic -e -t /model/togo/pose` floods the terminal: the PosePublisher sends a Pose_V of every link at 50 Hz. Suggest `gz model -m togo -p` instead (one shot; used in T5/T6).

### U10 (L): error messages without a next step

- `ERROR: world file not found: …` should add: "Build it: `cd <repo>/src/lunar_world_gen && ~/.venvs/lunar_world/bin/python -m lunar_world_gen.build --preset full`".
- `no world_info.yaml at …; pass world_info:= or robot_x/y/z` names launch arguments. The wrapper's own pre-check always passes `world_info=""` and does not see `-- world_info:=…`, so in the wrapper the hint cannot work. Add `--world-info PATH` to the wrapper, or reword: "use --pose X Y Z [YAW]".
- The resolution errors print to stdout in the middle of the banner (T1). Move the `ERROR:` line after the banner, or to stderr.

### U11 (L): the stock fallback is easy to miss and drops the inspect lighting

**Evidence (T2).** `--variant inspect` with the plugin missing loads `apollo16_core_stock.sdf`, which has realistic lighting. The message is a one-line `NOTE:` inside the banner.

**Fix.** Print a `WARNING` block naming the consequences and the fix:

```text
WARNING: patched physics plugin not found -> stock Bullet (apollo16_core_stock.sdf): in-place turns reach ~66 % of the command, mesh friction fixed at 0.5, 4 ms step; inspect lighting not available. Build it once: bash <repo>/src/lunar_world_gen/tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh
```

The other option is to write an `apollo16_core_inspect_stock.sdf`.

### U12 (I): files modified outside `src/lunar_world_gen` during this session (task item 5)

I listed everything under the repo except `src/lunar_world_gen`, `build`, `install`, `log`, `.git`, `.ccache`, `bags` and `slam_output`, newer than 2026-10-25 04:20 local (CDT). I checked mtime, ctime and birth time; all three searches agree.

| path | kind | time | note |
|---|---|---|---|
| `src/togo/togo_gz/media/dem/NAC_DTM_APOLLO16.TIF.aux.xml` | **new file**, 436 B | born 05:18:30 | GDAL PAM statistics (`STATISTICS_APPROXIMATE=YES`, min −163.1, max 765.4, mean 215.6) |
| `src/togo/togo_gz/media/dem/NAC_DTM_APOLLO16_2049.tif.aux.xml` | **new file**, 363 B | born 04:38:36, modified 05:18:20 | GDAL PAM statistics (min 5830, max 52663) |
| `src/togo/togo_gz/media/dem/` | directory mtime | 05:18 | from the two files above |
| `src/` | directory mtime | 04:25 | creation of `src/lunar_world_gen` |

- **No pre-existing file was modified.** The `.aux.xml` sidecars come from a statistics computation during the phase-1 research (`gdalinfo -stats` or rasterio `statistics()`); no generator code computes statistics (grep).
- They are harmless. The user may delete them with `rm src/togo/togo_gz/media/dem/*.aux.xml`, and GDAL recreates them only when statistics are computed again.
- To avoid this in future, research tools should set `GDAL_PAM_ENABLED=NO`.
- `.claude/`, `.agents/` and the native workspace `~/togo_ws_native_clean` have no changes after 04:20.

Outside the repo (information only): the four dangling symlinks in `/home/nargon` (U8), the WSL crash dumps (U5), two apport reports in `/var/crash/` (12:15 `ros2 launch --show-args`, 13:46 `_local_setup_util.py` BrokenPipe, with `LD_PRELOAD` set; neither is mine), and a `ros2 daemon` for domain 71 seen at 13:31. That daemon is the world agent's, started at about 11:49, and was gone by 13:53.

### U13 (I): behaviour changes the user may notice

- NavSat now reports the real site (lat −8.9734, lon 15.5111, altitude about 8 m + z) instead of 0/0 in `lunar_surface.world`, which had no spherical coordinates.
- The magnetic field is 0 0 0.
- RViz is off by default; use `--rviz`. It opens only after the controllers are active (about 35 s).
- The legacy world's `<gui><camera>` (-20, -20, 25, clip 0.1-500) was a Gazebo-classic element that gz-sim ignores, so the camera behaviour is unchanged. The new world simply makes the clip limit visible.

### What works well (measured)

- **Sequencing.** The robot is created at +3.4 s and both controllers are active at +33.8 s with exit codes 0/0. gz-validation saw 35.4-38.8 s over 5+ launches, with 0 spawner failures. The stock chain gives exit 1 on the same world.
- **Resolution and messages.** Bad site, bad variant, missing world and non-numeric pose all fail before anything is launched, with exit 1/2. The bad-site message lists the available sites.
- **Stock fallback.** It works when the plugin file is absent (T2).
- **Environment.** The same topics, QoS overrides, TF and DDS environment as `togo_sim_wsl.sh`, plus `GALLIUM_DRIVER=d3d12`: D3D12 was mapped in every server process here.
- **Clean shutdown.** No processes were left in any run (T1-T6).

## 3. Proposed `src/lunar_world_gen/README.md` (exact text)

````markdown
# Apollo 16 lunar world

A 100 m Apollo 16 core (1 cm terrain, 216 k rocks) with a 500 m LiDAR ring, a 2.2 km far field and a 20 km
horizon, lunar gravity (1.62 m/s^2) and a patched classic-Bullet plugin. The simulation runs natively in WSL;
SLAM / Nav2 keep running in Docker. `REPO` = /mnt/c/Users/Username/OneDrive/Desktop/togo_ws-slam-nav/togo_ws-slam-nav

## Run (world already built on this machine)

    cd $REPO
    ./src/lunar_world_gen/scripts/run_apollo16_sim.sh                    # Gazebo GUI, site 'flat'
    ./src/lunar_world_gen/scripts/run_apollo16_sim.sh --headless         # no GUI window
    ./src/lunar_world_gen/scripts/run_apollo16_sim.sh --variant inspect  # brighter lighting for looking around
    ./src/lunar_world_gen/scripts/run_apollo16_sim.sh --site slope       # also: rock, turn (see world_info.yaml)
    ./src/lunar_world_gen/scripts/run_apollo16_sim.sh --pose X Y Z [YAW_RAD] --rviz

- Ready when the log shows `platform_velocity_controller spawner exited with code 0` (about 35 s; the
  "Simulation running" banner appears earlier). Drive with `/platform_velocity_controller/cmd_vel`
  (geometry_msgs/TwistStamped).
- Same topics (`/husky/sensors/seyond/points`, `/husky/sensors/imu_0/data_raw`, `/clock`), TF, ROS domain 29
  and CycloneDDS config as `src/togo_sim_wsl.sh`. In a second terminal set the same environment first:
  `export ROS_DOMAIN_ID=29 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST CYCLONEDDS_URI=file://$REPO/config/cyclonedds.xml`
  (after sourcing /opt/ros/jazzy and ~/togo_ws_native_clean/install).
- Ctrl+C stops everything in 15-25 s. gz-sim 8 segfaults at shutdown (exit 139, "core dumped"): harmless,
  but WSL saves a 1.5-3.6 GB dump in %TEMP%\wsl-crashes each time (keeps 10; set `maxCrashDumpCount=2`
  under `[wsl2]` in %UserProfile%\.wslconfig to cap it).
- Memory: server ~3 GB with the rover; the Gazebo GUI needs roughly another ~2.7 GB (estimate) and ~20 s
  to show the terrain.

## SLAM against this world: DLIO gravity must be 1.62

The world has lunar gravity, so the simulated IMU reads 1.62 m/s^2 at rest. `seyond_dlio_graph.yaml` has
`odom/gravity: 9.82` (for the Earth-gravity `lunar_surface.world` test); with it DLIO's start-up calibration
turns the difference into an 8.2 m/s^2 accelerometer bias (`Accel biases [xyz]: … -8.2`) that corrupts the
odometry as soon as the rover tilts. Without editing the shared YAML, in the SLAM container:

    P=/home/er4-user/slam_ws/src/lidarslam_ros2
    sed -E 's/^([[:space:]]*odom\/gravity:[[:space:]]*)[0-9.]+/\11.62/' \
        $P/lidarslam/param/seyond_dlio_graph.yaml > /tmp/seyond_dlio_graph_moon.yaml
    COMBINED_PARAM_FILE=/tmp/seyond_dlio_graph_moon.yaml $P/scripts/togo/run_live_seyond_dlio_slam.sh

Start SLAM after the sim reports both controllers active (the rover has settled; DLIO calibrates the IMU on
its first 3 s of data). Use the unmodified YAML again for `lunar_surface.world` (9.82).

## Using the old launcher instead (`TOGO_WORLD`)

    TOGO_WORLD=$HOME/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf \
    TOGO_ROBOT_X=-4.5 TOGO_ROBOT_Y=-0.5 TOGO_ROBOT_Z=-0.434 ./src/togo_sim_wsl.sh

Works, but: its controller spawners start on a fixed 10 s timer while the rover's sensors are still loading
the scene, so expect `Failed to activate controller : joint_state_broadcaster` / `exit code 1` (the
controllers still end up active, check `ros2 control list_controllers`); no yaw (the rover faces east);
without `TOGO_ROBOT_*` it is dropped from 2 m; and if the patched plugin is missing it loads a world without
physics (nothing moves; one red `[Err] Failed to find plugin` line): use `apollo16_core_stock.sdf` then.
Do not start it while `run_apollo16_sim.sh` is running (it does not detect it).

## Gazebo GUI

- `--variant inspect` adds fill light and ambient light; physics are identical.
- The GUI uses your ~/.gz/sim/8/gui.config. Its camera far clip is 1000 m, so the far field and the 20 km
  horizon are cut off; add a `<camera_clip><near>0.1</near><far>30000</far></camera_clip>` to the
  MinimalScene plugin (or a copy passed with `gz sim -g --gui-config FILE`) to see them.
- The start camera (-6, 0, 6) looks east, away from the rover at the 'flat' site (-4.5, -0.5): zoom out
  and turn around, or right-click the rover → Move to / Follow.

## One-time setup and rebuilds

1. Patched physics plugin, once per machine (~40 s; needs the ROS Jazzy gz-physics 7.6.0, cmake, g++,
   libbullet-dev and GitHub access or the cached tarball). Run it from your home directory:

       cd ~ && bash $REPO/src/lunar_world_gen/tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh

   Without it the launcher uses `apollo16_core_stock.sdf` (stock Bullet: in-place turns ~66 % of the
   command, ground friction fixed at 0.5). After ROS updates check that the server log still shows
   `Loaded [gz::physics::bullet::Plugin] from library [...lunar...]` and the `[lunar patch]` lines.
2. Python venv (only to build or validate the world, not to run it):

       python3 -m venv ~/.venvs/lunar_world
       ~/.venvs/lunar_world/bin/pip install numpy==2.5.4 scipy==1.18.1 rasterio==1.5.2 pyproj==3.8.0 \
           trimesh==5.2.0 pydelatin==0.4.0 pyfqmr==0.5.0 numba==0.68.0 pygltflib==1.16.5 \
           matplotlib==3.11.2 PyYAML==6.0.3 tqdm==4.70.1 pytest==9.1.1 rosbags==0.11.9

3. Build the world (2:21 with the cached terrain, about 4:20 from scratch; peak 1.7 GiB RAM; 3.4 GB disk):

       cd $REPO/src/lunar_world_gen && ~/.venvs/lunar_world/bin/python -m lunar_world_gen.build --preset full

   `--status` shows what is stale; `--preset fast` overwrites the same world dir (see docs/impl/integration.md).
4. Rock library, only if ~/togo_assets/lunar/library/rocks_v1 is missing: Windows Blender 5.2, see
   docs/impl/rocks.md "How to run".

## Troubleshooting

| symptom | cause / fix |
|---|---|
| `ERROR: world file not found` | build the world (step 3) |
| `NOTE: physics plugin … not found -> falling back to apollo16_core_stock.sdf` | build the plugin (step 1) |
| rover hovers, nothing moves, `[Err] … Failed to find plugin` or `Unable to load the [...] library` | plugin missing/unloadable in a world without fallback: `ldd ~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so \| grep "not found"`; rebuild, or use `--variant stock` |
| controllers not active after ~60 s | `ros2 control list_controllers`; `ros2 run controller_manager spawner platform_velocity_controller --controller-manager-timeout 60` |
| SLAM container sees no topics | ROS domain / CycloneDDS mismatch (see "Run") |
| DLIO drifts on slopes, prints `Accel biases … -8.2` | `odom/gravity` still 9.82 (see "SLAM") |
| GUI empty for ~20-30 s | the GUI builds its own copy of the 9.5 M-triangle scene |
| horizon cut off ~1 km away in the GUI | camera far clip (see "Gazebo GUI") |
````

## 4. Evidence index

| file in `EXP` | content |
|---|---|
| `t1/*.log`, `t1_wrapper_args.sh` | wrapper argument and failure cases |
| `t2_plugin_missing.py`, `t2/` | fallback resolution on SDF copies |
| `t3_engine_failure.sh`, `t3/*.log`, `t3/*_model.txt` | missing, broken and real engine on tiny worlds |
| `t4_gui_views.py`, `t4/{realistic,inspect}/*.png`, `server.log` | GUI-equivalent renders, far-clip comparison, brightness |
| `t5_stock_togo_world/{console.log,result.json}` | user's `togo_sim_wsl.sh` with `TOGO_WORLD`, time-stamped |
| `t6_wrapper_headless/{console.log,result.json}` | `run_apollo16_sim.sh --headless`, time-stamped |
| `list_controllers.py`, `t56_launch_probe.py`, `wait_gpu_free.sh` | helpers (no ros2 daemon; leftover check by partition) |
| `seyond_dlio_graph_moon_test.yaml` | output of the gravity `sed` recipe (diff = line 72 only) |
