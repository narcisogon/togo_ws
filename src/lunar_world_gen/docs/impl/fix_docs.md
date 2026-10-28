# Fix round: documentation and user-facing configs (`docs` work package)

Owner: docs fix agent, 2026-10-25, 16:15-17:10. Findings owned: U1, U2, U6, U13 (`review_usability.md`),
FID-13, FID-14, FID-15 (`review_fidelity.md`), and the DESIGN §12 SLAM wording. Every finding was reproduced or
measured before anything was written. Scratch: `/home/nargon/togo_assets/lunar/experiments/fix_docs/` (`EXP`
below; all scripts and their JSON outputs are kept there). `GZ_PARTITION=claude_fixdocs`, `ROS_DOMAIN_ID=95`,
private `ROS_HOME`; Gazebo was started only for the windowless GUI-config test (one tiny server, two GUI runs,
section 4.3), always stopped with SIGKILL.

## 1. Result

| finding | verdict | what was done | files |
|---|---|---|---|
| U1 (high) DLIO `odom/gravity` 9.82 vs world 1.62 | **confirmed** (code + numeric reproduction) | lunar copy of the user's DLIO config with only `odom/gravity` changed, generator with `--check`, usage documented, rest of the SLAM stack audited | `config/slam/seyond_dlio_graph_lunar.yaml`, `config/slam/make_lunar_dlio_config.py`, README §5 |
| U2 no first-run guide, venv not reproducible | **confirmed** | README as the single first-run guide; exact `requirements-venv.txt`; every non-GUI command run | `README.md`, `requirements-venv.txt` |
| U6 GUI far clip 1000 m, rover not in view | **confirmed** (user config has no `camera_clip`) | GUI config from the installed default: 25 km far clip, chase-view start camera at `flat`, View Angle panel; camera generator; validated statically, windowless, and geometrically on the terrain | `config/gui/apollo16.gui.config`, `config/gui/make_apollo16_gui_config.py`, README §6 |
| U13 what changed vs `lunar_surface.world` | (information) | USER_GUIDE §1, 9 subsections, sourced | `docs/USER_GUIDE.md` |
| FID-13 uniform albedo | **confirmed** | measured, documented (USER_GUIDE §2.1, README §9) | |
| FID-14 earthshine 100-1000x too bright | **confirmed**, effect measured | documented; config value proposed (`3.0e-5`) with the measured image effect | USER_GUIDE §2.2, proposed config |
| FID-15 uniform sinkage, rock friction, pebbles | **confirmed** | measured, documented (USER_GUIDE §2.3, README §9) | |
| DESIGN §12 "SLAM stack unchanged" | wording wrong | corrected wording proposed (section 8); DESIGN.md not edited | |

Tests: `tests/test_docs.py`, 16 tests (section 9).

## 2. U1: DLIO gravity

### 2.1 Verification

- **Path correction.** The task named `REPO/src/lidarslam_ros2/...`; the user's SLAM repo is at
  `REPO/lidarslam_ros2/` (mounted in the container at `/home/er4-user/slam_ws/src/lidarslam_ros2`). The config is
  `REPO/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml`, line 72 `odom/gravity: 9.82`, md5
  `8ef011d11b4773aa1c1cb069221b612e`, mtime 2026-10-04 (unchanged by this work, checked before and after).
- **Mechanism, from the code** (`direct_lidar_inertial_odometry/src/dlio/odom.cc`): with `approximateGravity`
  and accel calibration on (both on in the YAML), lines 1899-1934 set
  `grav_vec = normalize(accel_avg) * |gravity|` and `b_accel = accel_avg - grav_vec`; `propagateState` (lines
  3015-3024) integrates `q * (a - b_accel) - (0, 0, gravity)`.
- **Numeric reproduction** (`EXP/u1_dlio_bias.py`, the same formulas, IMU at rest reads 1.62, then tilted):

  | odom/gravity | stored accel bias (body) | false horizontal accel at 5 / 10 / 12.34 / 15° |
  |---|---|---|
  | 9.82 | (0, 0, **-8.20**) m/s² | 0.715 / 1.424 / **1.752** / 2.122 m/s² |
  | 1.62 | (0, 0, 0) | 0 (< 1e-16) at every tilt |

  For scale: on level ground the rover's own acceleration is traction-limited to mu g = 0.65 m/s². The resulting
  SLAM trajectory error was not
  measured (SLAM runs in Docker, not started here); GICP corrects position each scan, but every IMU prediction,
  deskew and initial guess carries this error.

### 2.2 The lunar copy

`config/slam/seyond_dlio_graph_lunar.yaml` = a 25-line comment header + the source byte for byte, except line 72
`odom/gravity: 1.62   # LUNAR COPY: ...`. It is written by `config/slam/make_lunar_dlio_config.py` (stdlib only;
gravity read from `physics.gravity_m_s2` of the generator config; `--check` exits 1 when stale; the header records
the source md5).

Checks:
- `diff source <(tail -n +26 lunar)`: one hunk, `72c72`, the gravity line only.
- PyYAML (`EXP/u1_yaml_checks.py`): 294 keys in both files; the only differing key is
  `dlio_odom_node.ros__parameters.odom/gravity` (9.82 -> 1.62).
- **ROS 2 parameter parser** (rclpy node `dlio_odom_node` with `--params-file`, domain 95): 143 parameters from
  either file, only `odom/gravity` differs, value 1.62 of type double.
- Precedence: `seyond_dlio_slam.launch.py` loads `cfg/dlio.yaml`, `cfg/params.yaml` (gravity 9.80665), then
  `dlio_param_file` ("last YAML wins"), so 1.62 is what DLIO uses.

Usage (README §5): `REPO/src` is mounted at `/home/er4-user/ws/src` in the `dev` and `hw-dev` services
(`docker-compose.yml` lines 105 and 136), so in the container

```bash
cd ~/slam_ws/src/lidarslam_ros2/scripts/togo
COMBINED_PARAM_FILE=/home/er4-user/ws/src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml \
  ./run_live_seyond_dlio_slam.sh
```

`run_live_seyond_dlio_slam.sh` lines 11-13 take `COMBINED_PARAM_FILE` and default `SLAM_PARAM_FILE` and
`DLIO_PARAM_FILE` to it. No container was running, so the in-container path was verified from the compose file
only, not with `docker exec`.

### 2.3 Other Earth-gravity assumptions in the SLAM stack (audited in source)

| component | gravity use | needs a change for this world? |
|---|---|---|
| DLIO odom (`dlio_odom_node`) | `odom/gravity` magnitude (above) | **yes**: the lunar copy |
| `dlio_map_node`, `reference_path_publisher` | none | no |
| graph backend hazard mapping (`graph_based_slam/hazard`, `graph_based_slam_optimization.cpp:799`) | direction only: `up_in_body = R_odom^T z`, i.e. DLIO's gravity-aligned odom z; heights and slopes are measured along it | no (correct as long as DLIO is) |
| RKO-LIO pipeline (`seyond_live_slam.yaml` `gravity_magnitude: 9.8107`; IMU predictor `gravity_mps2: 9.8107` in `seyond_live_slam.launch.py:117`) | magnitude | yes, if pointed at this world (old a300 topics; not used by the DLIO script) |
| lidarslam `scanmatcher` (`scanmatcher_component.cpp:2063-2065`, hard-coded 9.81 when `use_imu`) | magnitude | yes, if used with IMU on this world |
| `seyond_dlio_graph_husky.yaml` | `odom/gravity: 9.80665` | yes, if used on this world |
| real-robot DLIO configs (`*_live_real*.yaml`, `*_replay.yaml`) | 9.80665 | no (Earth) |
| `togo_deploy` robot_localization EKF (`localization.yaml`, `imu0_remove_gravitational_acceleration: True`, default 9.80665) | magnitude | no: started only by `togo_sensors.launch.py` (real rover); not in the sim chains |
| FAST-LIO pipeline | its source/config (`faster_lio/`) is not in this checkout | not checked |

## 3. U2: README and venv

### 3.1 README (`README.md`)

Sections: what the world is (zones, counts, physics, lighting, files, variants, sites; 3 images from
`docs/impl/img`), requirements, one-time setup (plugin, venv, build), running (options, start-up timeline,
robot collision, topics, second terminal, stopping), SLAM with the lunar gravity file, Gazebo GUI, the old
launcher with `TOGO_WORLD`, troubleshooting, known limitations, where the detailed docs are.

Commands run to verify it (all non-GUI ones):

| command | result |
|---|---|
| `build_lunar_bullet_plugin.sh --out EXP/... --work EXP/... --tarball <cached> --jobs 4` from a scratch CWD | rc 0 in **38 s**, 895,760 bytes (same size as the installed plugin, not byte-identical: build paths are embedded); installed plugin sha256 unchanged; **U8 reproduced**: the 4 dangling `libgz-physics-*-plugin.so` symlinks appeared in the CWD |
| `python3 -m venv` + `pip install -r requirements-venv.txt` | rc 0 in 27 s (pip cache); `pip freeze --all` identical to the project venv |
| `pytest tests -q` with that recreated venv | **142 passed** in 74 s (the suite at 16:23) |
| `python -m lunar_world_gen.build --help`, `--preset full --status` | help as documented; status rc 0, all 8 stages STALE at 16:33 ("terrain manifest missing or for another terrain key": the parallel terrain/build fixes changed code), so the README does not promise a cached build |
| `run_apollo16_sim.sh --help` and `--dry-run` with: no option, `--headless`, `--site slope`, `--variant inspect`, `--pose -4.5 -0.5 -0.43 2.36 --rviz`, `--site rock` | all rc 0, resolved world, physics plugin, pose, GUI config and launch command printed; nothing started |
| `source scripts/env.sh` | rc 0; domain 29, rmw_cyclonedds_cpp, CycloneDDS URI, GALLIUM_DRIVER=d3d12 |
| `make_lunar_dlio_config.py --check`, `make_apollo16_gui_config.py --check` | rc 0, up to date |
| `rover_drive_test.py --help` (system python, ROS sourced) | as documented |

Not run (need a window, Docker or a full simulation): the GUI, `ros2 control list_controllers` against a running
sim, the SLAM container, `togo_sim_wsl.sh` with `TOGO_WORLD` (its README section uses the U3 measurements).

### 3.2 `requirements-venv.txt`

The exact `pip freeze` of `~/.venvs/lunar_world` (45 packages, Python 3.12.3, pip 26.2.2), with the recreate
commands and the 15 direct dependencies in the header. Note: pip 26 lists `setuptools` and `wheel` in
`pip freeze`; they are included so a recreated venv is identical.

## 4. U6: Gazebo GUI config

### 4.1 Verification

- `~/.gz/sim/8/gui.config` (68 kB, the user's) has a `camera_pose` but no `camera_clip`; the world SDF has no
  `<gui>`. The gz-gui 8 MinimalScene default far clip is 1000 m (review U6, from the gz-gui source). The installed
  `libMinimalScene.so` knows `camera_clip`, `near`, `far`, `horizontal_fov`, `sky` (binary strings).
- The GUI applies the world's `<scene>` ambient and background: `libGzSceneManager.so` calls
  `RenderUtil::UpdateFromECM/Update` (nm), the same path as the sensors. So MinimalScene's ambient/background are
  only initial values; the file uses the realistic ones (0.01 / black) to avoid a grey first frame.

### 4.2 The config

`config/gui/apollo16.gui.config` = the installed default
(`/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config`, identical md5 to `/usr/share/...`) with:
`camera_clip` near 0.1 / far 25000 m (horizon corners are 14.1 km from the core centre), the chase-view
`camera_pose`, ambient/background as above, and the `ViewAngle` plugin added (runtime clip/FOV control). The
camera block is written by `config/gui/make_apollo16_gui_config.py` from the `flat` site of `world_info.yaml`
(5 m behind along the heading, 2.2 m above the visual ground, aimed 0.3 m above it):
`-0.964 -4.036 1.578 0 0.3631 2.3562` (pitch 20.8° down, 5.35 m to the rover, sun 130° right of the view).

### 4.3 Validation

- **Static** (`EXP/u6_gui_static_check.py`): wrapped XML parses; plugins = the 18 defaults + ViewAngle; all 19
  plugin libraries exist; every default element except MinimalScene is unchanged; all MinimalScene keys are known
  to the installed library. Found and fixed on the way: `--gui-config` inside an XML comment (a double hyphen is
  invalid XML).
- **Windowless run** (`EXP/gui_test/run_gui_test.sh`): a tiny world in partition `claude_fixdocs` + `gz sim -g -v 4
  --gui-config <file>` under `QT_QPA_PLATFORM=offscreen`, private `HOME`, `timeout -s KILL`. Gazebo read the file
  (`Loading config [...apollo16.gui.config]`) and loaded 17-18 of the 19 plugins (ViewAngle's QML was being
  instantiated) with no `[Err]` and no "Unable to set <near>/<far>" message; then ogre2 could not create its GLX
  render window ("Invalid parentWindowHandle"; the offscreen platform has no real window) and the GUI segfaulted
  6 s in, the same for a 1000 m control config. So the rendered picture (and the far-clip effect) could not be
  checked without a visible window; no Xvfb is installed. The far-clip effect itself was shown on ogre2 camera
  sensors by review U6 (0.1 / 30000). The two segfaults wrote two 595 MB WSL dumps (PIDs 184753, 184962,
  16:28:23 and 16:29:10), deleted at 16:30.
- **Geometry on the real terrain** (final 1 cm heights, `EXP/u6_camera_pose.py`, `u6_follow_check_rot.py`): with
  the runtime wrapper's follow transform (`launch/apollo16_sim.launch.py` `gui_config_for_pose`: offset turned
  by the yaw difference, height moved with the spawn), every site gets a chase view:

  | site | camera above terrain | min line-of-sight clearance | rover off the view axis |
  |---|---|---|---|
  | flat | 2.373 m | 0.508 m | 0.00° |
  | slope | 2.355 m | 0.513 m | 0.91° |
  | rock | 2.204 m | 0.491 m | 0.53° |
  | turn | 2.404 m | 0.489 m | 0.01° |

## 5. U13: USER_GUIDE

`docs/USER_GUIDE.md` §1 "What changed compared with `lunar_surface.world`": a 23-row summary table, then gravity
(incl. the rover's traction-limited start and bouncing), physics plugin, launch sequencing and controller timing,
lighting, materials, scale/terrain/rocks, sites and spawning, and what stayed the same. Facts were taken from the
legacy world file itself (gravity -9.82, stock Bullet, ambient 0.25, sun 1.2 at 59.3°, earthshine 0.10, fill
0.25, mu 0.8, Contact system, magnetic field 1e-9, `<gui><camera>`, only the terrain model: the "56 rocks"
comment is stale) and from the cited reports.

## 6. FID-13, FID-14, FID-15 (measured on the built world, `EXP/fid13_15_checks.py`)

- **FID-13 confirmed.** `textures/regolith_albedo.png` (2048², 2 m): linear mean (0.120, 0.107, 0.088), luminance
  relative std 4.86 %, only 1.57 % at features of 20 cm and larger; one regolith albedo map serves every zone
  (the SDF references only `regolith_albedo.png` and the rock atlas). Camera only. Documented, with the possible
  generator change; no change made (generator/material code is not this package's docs scope).
- **FID-14 confirmed and quantified.** SDF earthshine `<intensity>0.01</intensity>`; calibration 3e-5 (1e-5 to
  1e-4). In the review's realistic-lighting image of this world (`p3_review-usability/t4/realistic/
  rover_view_far30000.png`, `EXP/earthshine/shadow_levels.py`): sunlit ground median 51/255, the box's and the
  big rock's cast shadows 3/255 (linear 2.7 % of sunlit). The light geometry gives earthshine 1.67 % of the sunlit
  level-ground light, i.e. 1.8 of the 3 grey levels; at 1e-4 it would add 0.02 and at 3e-5 0.006 grey levels.
  `_fmt(3e-5)` writes `3e-05`; a scratch SDF copy with it passes `gz sdf -k` and sdformat reads 3.0e-05.
  Proposal in section 10.
- **FID-15 confirmed.** All 108 collision meshes have mu 0.4 (`physics.rock_mu` 0.6 unused); 199,185 core
  pebbles of 2-5 cm, 0 with collision; 16,493 rocks of 5 cm and larger in the collision surface;
  visual-vs-collision on rock-free ground median -20.3 mm, p1 -33.3, p99 -7.1 mm (static checks), i.e. about
  10 mm of visual sinkage everywhere after Bullet's 10 mm margin. Documented as a limitation; no change (the
  optional fixes are generator/physics work).

## 7. Cross-checks with the other fix agents (read, not edited)

- The runtime agent's wrapper (16:32 version) already passes `config/gui/apollo16.gui.config` to the GUI
  ("follow" mode), prints the gravity/DLIO line in its banner, adds `--robot-collision`, `--dry-run`,
  `--world-info`, `--gui-config`, `--default-gui`, `--variant inspect_stock`, SIGKILL shutdown, `env.sh` and the
  READY line. README §4 documents exactly those options; `test_readme_documents_the_wrapper_options` fails if
  the wrapper and the README drift apart.
- `scripts/env.sh` intends `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` as a default, but ROS Jazzy's
  `ros_environment` hook (`1.ros_discovery.sh`) sets `SUBNET` while it is sourced, so the default never applies.
  `togo_sim_wsl.sh` behaves identically, so this changes nothing for the user's working setup; noted for the
  runtime agent.
- The runtime agent's report did not exist yet at 17:10 (it was still running RTF series), so README §9 refers to
  the fix-round reports (`docs/impl/fix_*.md`) for the measured effect of the simplified robot collision, and
  README §7 (`TOGO_WORLD`) uses the U3 measurements of the usability review. Its interim, contended slope runs
  (`fix_runtime/runs/rtf_summary.json`, not quoted in the docs) showed RTF about 0.61-0.65 simplified vs 0.14
  original, with SIGKILL stops of about 1 s; the README wording ("addresses the main cause", "stops within a few
  seconds") is consistent with that. Its "crash dir changed" log lines are a false alarm (only the parent
  folder's mtime changed; `wsl-crashes` stayed empty).

## 8. DESIGN §12: proposed wording (DESIGN.md not edited)

Current: "**Unchanged.** The user's existing scripts, the SLAM stack and topic names stay the same."

Proposed:

> - **Unchanged.** The user's existing scripts, the robot description, the topic names, and the SLAM and Nav2
>   stacks. **Exception: DLIO's `odom/gravity` must equal the world gravity (1.62 m/s²).** The shared
>   `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml` keeps 9.82 for the Earth-gravity
>   `lunar_surface.world` test; this package ships an otherwise identical copy,
>   `config/slam/seyond_dlio_graph_lunar.yaml` (regenerated by `config/slam/make_lunar_dlio_config.py`), passed as
>   `COMBINED_PARAM_FILE` (README §5). With 9.82, DLIO's start-up calibration stores a -8.2 m/s² accelerometer bias
>   that leaks 8.2 sin(tilt) m/s² into its motion estimate. The graph backend's hazard mapping uses only the
>   gravity direction and needs no change; the other pipelines that hard-code Earth gravity (RKO-LIO, the lidarslam
>   scanmatcher with IMU, `seyond_dlio_graph_husky.yaml`) would need the same change before being used here.

The same section's wrapper bullet ("It also takes `--headless`, `--site`, `--variant` and `--rviz`") is now
incomplete (robot collision, GUI config, dry run, `env.sh`); that is the runtime agent's to word.

## 9. Tests (`tests/test_docs.py`, 16)

| test | checks |
|---|---|
| `test_lunar_dlio_gravity_is_world_gravity` | `odom/gravity` is a float equal to `physics.gravity_m_s2`; header and container path present |
| `test_lunar_dlio_differs_from_user_config_only_in_gravity` | parsed-YAML diff against the user's file is at most that key (skips without `lidarslam_ros2`) |
| `test_lunar_dlio_copy_is_up_to_date` | the committed copy equals the generator's output for the current user file; body = source except one line |
| `test_lunar_dlio_generator_on_synthetic_source` | one changed line, md5 in the header, float kept, errors on 0 or 2 gravity lines |
| `test_dlio_generator_reads_config_gravity` | the stdlib config reader picks `physics.gravity_m_s2`, not another section's |
| `test_requirements_fully_pinned`, `test_requirements_match_project_venv` | exact pins, no duplicates, direct deps present; versions equal the running project venv |
| `test_markdown_links_exist[README, USER_GUIDE]` | every relative link and image exists |
| `test_package_paths_in_docs_exist[README, USER_GUIDE]` | every package path named in the docs exists |
| `test_readme_documents_the_wrapper_options` | README §4 lists every wrapper option, and the README uses no option the wrapper lacks |
| `test_gui_config_structure` | valid XML (no `--` in comments), plugin set, ogre2, near clip, far clip reaches the horizon corners, pose shape |
| `test_gui_config_keeps_the_installed_defaults` | everything but MinimalScene's values equals the installed default (skips without it) |
| `test_gui_camera_generator` | chase-view geometry, idempotence, only the camera block changes |
| `test_gui_camera_looks_at_flat_site_of_world_info` | the camera frames the current `flat` site (skips without a built world) |

**Results.**
- `pytest tests/test_docs.py` (final, 17:10): **16 passed** in 1.7 s.
- Whole suite at 16:51 (project venv, niced, while other agents were editing): 205 passed, 11 failed. 10 of the
  failures are in `test_world_build.py` (3), `test_world_rocks.py` (5) and `test_world_sdf.py` (2) and come from the build and
  rocks agents' in-progress changes (a new build stage, the `inspect_stock` variant in `world_info`, the crater
  rock enhancement); none involves a file of this work package. The 11th was this file's README path check on a
  then-planned `docs/impl/fix_runtime.md` link, since reworded (now passing).
- Before any fix-round change (16:23, recreated venv): 142 passed.

## 10. Proposed config

| key | value | reason |
|---|---|---|
| `lighting.realistic.earthshine_intensity` | `3.0e-5` (now 0.01) | FID-14: the calibrated value. In Gazebo's fixed-exposure 8-bit images it renders as nothing (0.006 grey levels) instead of about 1.8 (light-geometry estimate) of the 3 grey levels measured in cast shadows now, which is also what a camera exposed for sunlit regolith records. The light keeps its direction and colour for future exposure-controlled or night rendering. Shadow fill then comes from `ambient` 0.01 (about 1 % of sunlit level ground), the stand-in for terrain-reflected light (calibration: up to 1-3 % for vertical faces). Also used by the inspect variant, where fill 0.35 and ambient 0.25 dominate. **Requires** `tests/test_world_sdf.py::test_lighting_modes` (line 88, `earth.intensity == approx(0.01)`) to compare with the config value instead. Validated: SDF writes `3e-05`, `gz sdf -k` Valid, sdformat parses 3.0e-05. |

## 11. Open points for the orchestrator

1. **After the final rebuild**, run `python3 config/gui/make_apollo16_gui_config.py --check` (and without
   `--check` if the `flat` site moved); `test_gui_camera_looks_at_flat_site_of_world_info` fails until then. The
   runtime wrapper places the camera relative to the `flat` site of `world_info.yaml`, so a stale file would aim
   at the old spot.
2. The README's "as built on 2026-10-25" numbers (9.5 M triangles, 215,678 rocks, start-up 35-40 s, RTF) and the
   images predate the fix round; refresh them from the final `world_info.yaml` and validation.
3. If the user edits `seyond_dlio_graph.yaml`, `test_lunar_dlio_copy_is_up_to_date` fails until
   `config/slam/make_lunar_dlio_config.py` is rerun (intended).
4. U8 (the plugin build's 4 dangling symlinks in the caller's directory) is reproduced but in no work package;
   README §3.1 tells the user to start the script from the build folder. The 4 links in `/home/nargon` remain.
5. `src/togo_sim_wsl.sh` has a blank line before its shebang and a stray "```" as its last line (information for
   the user; not changed).
6. When `docs/impl/fix_runtime.md` exists: point README §9 at it with its measured RTF numbers, and refresh
   README §7 if its U3 investigation found anything beyond the usability review's measurements.
7. The runtime agent was still changing `scripts/run_apollo16_sim.sh` when this work ended (16:32 version
   documented). `test_readme_documents_the_wrapper_options` fails if an option is added or removed without a
   README update.
