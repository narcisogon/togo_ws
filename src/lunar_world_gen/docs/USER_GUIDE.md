# Apollo 16 world: user guide

This guide is for people who know the old `lunar_surface.world` and want to know what is different, and for
anyone who uses the world for camera or physics experiments. To install, build and run the world, start with
`../README.md`.

Sources: the measurements come from `docs/impl/` (builds, Gazebo validation, reviews, fix round, leak diagnosis)
and `docs/research/` (calibration, legacy review). Every number is cited with its source. The world described
here is the final build of 2026-10-25 (`docs/impl/final_build.md`): 17.1 M visual triangles and new test sites.
Rover numbers marked *phase 3* were measured on the first validated world (9.5 M triangles, the old sites); the
final world's rover and LiDAR tests are in `docs/impl/final_validation.md`, its launcher run in
`docs/impl/e2e_launch.md`.

## 1. What changed compared with `lunar_surface.world`

`lunar_surface.world` is the world `src/togo_sim_wsl.sh` loads by default (`src/togo/togo_gz/worlds/`). The
Apollo 16 world replaces it for lunar work. It is started with `src/lunar_world_gen/scripts/run_apollo16_sim.sh`.

### 1.1 Summary

| topic | `lunar_surface.world` | Apollo 16 world (`apollo16_core.sdf`) |
|---|---|---|
| gravity | **-9.82 m/s²** (a temporary Earth-gravity traction A/B test) | **-1.62 m/s²** |
| IMU at rest reads | 9.82 m/s² | 1.62 m/s² |
| DLIO `odom/gravity` | 9.82 (`seyond_dlio_graph.yaml`) | **1.62**: use `config/slam/seyond_dlio_graph_lunar.yaml` |
| physics engine | stock classic Bullet, 1 ms | patched classic Bullet, 1 ms (stock variant: 4 ms) |
| in-place turns | stock Bullet damps the yaw: 13 % of the command at 1.62 m/s² (rover baseline) | 0.97-1.02 of the command at `flat`, 0.89 at the `turn` site (final world) |
| ground friction | SDF mu 0.8, ignored on meshes: effectively 0.5 | mu 0.4, honoured; rocks included |
| terrain | procedural 30 x 30 m patch, 0.97 m relief, no Apollo 16 data | the real Apollo 16 elevation model plus calibrated synthetic detail |
| extent | 30 m | 100 m driving core, 500 m LiDAR ring, 2.2 km far field, 20 km horizon |
| rocks | none (the file's "56 rocks" comment is stale) | 215,677 rocks from 2 cm in the core; around it 3,278 of 30 cm and larger, plus 33,751 smaller ones in a 6 m band along the core edge |
| collision | separate coarse mesh, 10.7 mm RMS (up to 168 mm) away from the visual | derived from the visual surface: 2 cm lower, low-passed below 25 cm, rocks of 5 cm and larger included |
| lighting | sun 1.2 at 59° elevation, earthshine 0.10, overhead fill 0.25, ambient 0.25 | sun 1.0 at 21.9° from the east (EVA-1), earthshine 3e-5 of the sun (calibrated), ambient 0.01; `inspect` variant brighter |
| materials | one Lambert colour (0.42, 0.40, 0.38) from the Collada file | PBR textures: regolith albedo, normal and roughness maps; rock atlas |
| NavSat | 0 / 0 (no spherical coordinates) | the real site, lat -8.9734, lon 15.5111 |
| magnetic field | 1e-9 | 0 |
| spawn | `TOGO_ROBOT_X/Y/Z`, default (-2, 3, 2.0): a drop; no yaw | four test sites with x, y, z and yaw in `world_info.yaml`, 4 cm above the ground |
| controller start | spawners on a fixed 10 s timer: a spawner failure in 22 of 25 old launches | spawners after the robot's sensors are up: no spawner failure in 26 launches in phase 3, 33 in the fix round and 12 on the final world |
| ready to drive | about 24 s, with a spawner failure (old world, stock chain) | about 35-60 s, announced by `[apollo16_sim] READY` (37-47 s measured on the final world) |
| robot collision | original CAD meshes | 88 boxes in place of the 14 CAD collision meshes by default (`--robot-collision original` restores them) |
| GUI camera | `<gui><camera>` (Gazebo-classic syntax, ignored by gz-sim): your `~/.gz/sim/8/gui.config`, far clip 1000 m | `config/gui/apollo16.gui.config`: far clip 25 km, start view at the rover |
| Gazebo GUI start | together with the server | the server starts without a GUI; the GUI opens after READY (`--gui-early`: the old order); `scripts/open_gui.sh` opens, closes (`--close`) or reopens it from another terminal while the simulation runs |
| memory of the Gazebo GUI | about 0.62 GB | 3.7-3.8 GB; the server with the rover 3.7-3.9 GB (both flat over 22 minutes) |
| memory growth in WSL without the leak fix | 0.46 MB/s in the GUI, starting after about 3 minutes | 5.5-10 MB/s in the GUI, 2.7-3.5 MB/s in the server; flat with the fix, which the launcher, `env.sh` and `togo_sim_wsl.sh` set (section 1.10) |
| shutdown (Ctrl+C) | gz segfaults: 15-25 s and a 1.5-3.6 GB WSL crash dump | gz is stopped with SIGKILL: 2-3 s, no dump; a dump that a crash of one of the run's gz processes left is deleted at exit |
| RViz, point fixer | started by default | off by default (`--rviz`); the point fixer only logged a FATAL because the cameras are disabled |

### 1.2 Gravity

The world uses the Moon's gravity, 1.62 m/s². `lunar_surface.world` currently uses 9.82 m/s² for a traction A/B
test, and the user's DLIO config matches that. **Everything that subtracts gravity from the IMU must use 1.62 in
this world.** For DLIO this is a ready-made config copy (README section 5). With the wrong value, DLIO stores an
accelerometer bias of -8.2 m/s² at start-up and integrates 8.2 sin(tilt) m/s² of false horizontal acceleration
(0.71 at 5°, 1.42 at 10°, about 1.8 on the 12.4° `slope` site). The README also lists the other SLAM pipelines
that hard-code Earth gravity.

Gravity also changes the rover's behaviour, as it should on the Moon:
- **Traction-limited start.** The available traction is mu g = 0.4 x 1.62 = 0.65 m/s², below the controller's
  2 m/s² acceleration limit. The rover needs about 0.74 s to reach 95 % of the commanded speed and covers 0.93-0.95
  of the nominal distance in the standard 3 m drive: 0.945 on a level test mesh (`docs/impl/world.md` 5.1), 0.928 at
  the phase-3 `flat` site, whose corridor rose 3° (`docs/impl/gz_validation.md` 5.2). On the final world:
  0.950-0.955 at `flat` (its corridor falls 7.4°, section 1.8) and 0.940 on level ground, 1.00-1.02 of the
  traction-limited prediction; steady speed 0.997-1.005 of the command (`docs/impl/final_validation.md` 3).
- **Bouncing.** At 1.62 m/s² the rover leaves the ground more easily: 12-14 % of the time airborne at 0.5 m/s on a
  gently rough test world, 0-0.33 % on the phase-3 flat site (`docs/impl/world.md` 5.3, `gz_validation.md` 5).
  On the final world: 0-1.67 % at `flat`, 0.1-0.9 % over the rock, 1.7-3.7 % on the rough slope.
- **Slopes.** The rover climbed the phase-3 slope site (12.3°; 11.4° mean climb, 85 % of the 6 m commanded) and
  the fix-round world's slope site (10.4° climb, 90 %, `docs/impl/fix_build.md` 6). The final `slope` site
  (5 m at 12.4° mean) is rougher: over the wheelbase, 24-35 % of it is steeper than 15° (up to 20.6°). There the
  rover covered only 0.649-0.700 of the 6 m commanded (criterion 0.80), with 29-35 % slip, pitching to 20-23°.
  Smooth inclines with the same rover and physics give 0.953 at 10°, 0.938 at 12.4°, 0.913 at 15° and still 0.837
  at 19° (`docs/impl/final_validation.md` 2). The rock traverse (0.969-0.985 of 4.5 m, no stall) and the turn
  site (yaw tracking 0.888-0.889, criterion 0.85) pass.

### 1.3 Physics plugin

Classic Bullet in gz-physics 7.6.0 has two defects that matter on the Moon (`docs/research/rover_baseline.md`,
`docs/impl/world.md` 4):
1. Its joint-velocity command acts as a yaw damper, so in-place turns reach only 13 % of the command at lunar
   gravity on flat ground.
2. It ignores the SDF friction of mesh collisions and uses 0.5.

The world therefore loads a patched build of the same plugin, by absolute path
(`~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so`, built once with
`tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh`). With it, in-place turns track 0.95-0.97 of the
command and the ground friction is the configured 0.4. If the plugin is missing or cannot be loaded, the launcher
falls back to `apollo16_core_stock.sdf` (stock plugin, 4 ms step) and prints a warning. Under the stock plugin,
turns reach about 0.40-0.66 of the command (0.40 / 0.58 on the final world).

### 1.4 Launch sequencing and controller timing

Loading the robot makes Gazebo build the render scene for the robot's sensors, and the simulation does not step
while it does. On the phase-3 world (9.5 M triangles) that stall lasted about 25 s (`gz_validation.md` 5); on
the final world (17.1 M triangles) a world-only test got its first LiDAR cloud 27 s after the start
(`docs/impl/final_build.md` 8). The stock chain (`sim_gz.launch.py`) starts the controller spawners on a fixed
10 s timer, during the stall, and their `switch_controller` call times out. The old world already failed this way
in 22 of 25 recorded launches.

`launch/apollo16_sim.launch.py` reproduces the stock chain (same xacro arguments, bridge, node names, topics) but
sequences it:
1. Gazebo server, clock bridge, robot_state_publisher, bridge and spawn at t = 0. The rover appears after about
   3 s. The server runs under a small guard that stops it with SIGKILL, never with SIGINT or SIGTERM (gz-sim 8
   segfaults then).
2. A small waiter holds the controllers until the robot's LiDAR topic exists and simulation time advances again.
3. `joint_state_broadcaster`, then `platform_velocity_controller`, with generous timeouts.
4. A readiness check (`list_controllers` and `/clock`), then `[apollo16_sim] READY at +T s` and a ready marker
   file (`ready_file:=`), then RViz if asked.
5. `scripts/run_apollo16_sim.sh` waits for that marker and only then opens the Gazebo GUI: in WSL the GUI worked
   reliably only when it was opened after the server had loaded the world. Its window stays blank for about
   30-60 s while it loads its own copy of the world. With `--headless` no GUI opens; `scripts/open_gui.sh` opens
   one from another terminal when you want it.

Measured on the earlier worlds: READY, or both controllers active, 30-44 s after the start in most runs and up to
64 s while other agents' simulations shared the machine (`gz_validation.md` 5, `docs/impl/fix_runtime.md` 1.3
and 2). No spawner failed in 26 launches in phase 3 (9 on that world, 17 on test worlds and a copy of the old
world, where the stock chain failed 2 of 2) and in 33 launches in the fix round; one more fix-round launch ended
before the robot existed because WSLg could not create a GPU context while other simulations ran. On the final
world READY came 37-47 s after the start (3 launcher runs and 12 robot-test launches), and both controllers were
active in 12 of 12 launches with no spawner failure (`docs/impl/e2e_launch.md` 2,
`docs/impl/final_validation.md` 1.1). **Wait for READY before driving or starting SLAM.**

### 1.5 Lighting

| | `lunar_surface.world` | Apollo 16 realistic (default) | Apollo 16 `inspect` |
|---|---|---|---|
| sun | intensity 1.2 from 59° elevation (the comment says "low grazing angle") | intensity 1.0, 21.9° elevation, azimuth 84.7° (EVA-1 start, from the east) | 33.8° elevation (EVA-2 start) |
| earthshine | 0.10 | 3e-5 of the sun, from 75° elevation, north-west, bluish (the calibrated value) | 3e-5 |
| fill / ambient | overhead fill 0.25, ambient 0.25 | no fill, ambient 0.01 | overhead fill 0.35, ambient 0.25 |
| sky | black | black | black |

The realistic lighting is dark by design: there is no sky light on the Moon, so shadows are nearly black. On the
phase-3 world, whose earthshine was still 1 % of the sun, cast shadows measured 3 of 255 grey levels in camera
images, against about 51 for sunlit ground; with the calibrated earthshine about 1 grey level is expected
(section 2.2). Use `--variant inspect` to look around; the physics are identical. The sun direction equals the
calibrated EVA-1 geometry to 0.02° (`review_fidelity.md` 2).

### 1.6 Materials

The old terrain had one Lambert colour (0.42, 0.40, 0.38) from its Collada file. The new ground uses PBR maps:
- regolith albedo with the calibrated in-situ colour (linear mean 0.120, 0.107, 0.088) and a ±12 % fine
  variation;
- a normal map with sub-centimetre micro-relief (OpenGL convention, checked in Gazebo);
- a roughness map (0.95), metalness 0.

Rocks use a shared texture atlas made in Blender (basecolor, normal, roughness); they are about 1.9 times brighter
than the soil, as in the calibration. The same albedo map repeats every 2 m in the core and ring, every 20 m in the
far field and every 200 m on the horizon.

### 1.7 Scale, terrain and rocks

The old world was a procedural 30 x 30 m patch with no Apollo 16 data, slopes 1.39 times steeper than designed
(median triangle slope 17.5°) and pit-like craters (depth/diameter 0.5-0.86; fresh lunar craters are about 0.2)
(`docs/research/legacy_review.md`).

The new world is built around the real elevation model of the landing site, which resolves about 10 m. Below
that, synthetic relief, craters and rocks are added with the measured Apollo 16 statistics (final build,
`docs/impl/final_build.md` 6):
- roughness within ±25 % of the target from 1 cm to 0.3 m (0.93-1.23 of it), and the synthetic part within ±25 %
  at 1 and 3 m;
- crater counts within 6 % of the size-frequency law from 0.2 m to 2 m;
- rock counts within sampling noise at 2, 5, 10 and 20 cm.

Crater walls steeper than 32° are slumped the way loose regolith would slump (a mass-conserving talus limiter):
the steepest ground at a 0.5 m baseline is 33.5° (it was 47.6° before the fix round, `docs/impl/fix_terrain.md`).
Craters vary in depth, rim height and outline, and fresh rims have their full calibrated height.

| zone | size | grid | contents |
|---|---|---|---|
| core | 100 m | 1 cm | everything: rocks from 2 cm, craters from 0.2 m, fine relief (fading out over the last 3 m) |
| ring | 500 m | 25 cm | rocks of 30 cm and larger, craters from 1 m; smaller rocks in a 6 m band along the core edge |
| far field | 2.2 km | 2 m | the real elevation model |
| horizon | 20 km | 20 m | the real elevation model |

Rocks are merged into one mesh per 10 m tile, because thousands of separate rock models make Gazebo unusable
(10,000 separate rocks dropped the real-time factor to 0.03). The 16,546 rocks of 5 cm and larger are part of the
collision surface; the 199,131 pebbles of 2-5 cm are visual only. Perched rocks rest on a stable face, no rock
pierces a rock of 5 cm or larger, and every rock without a regolith fillet is seated on the rendered ground: none
of those 251,001 rocks hovers, and only 5 almost fully buried ones end up just below the surface
(`docs/impl/fix_rocks.md`, `docs/impl/final_build.md` 3.2).

### 1.8 Sites and spawning

`world_info.yaml` (next to the world file) lists four test sites with position, height and heading, plus the
search criteria that chose them:

| site | position | use |
|---|---|---|
| `flat` | (-2.00, -4.50), heading -22.5° | default spawn; the standard drive test runs along its 4 m corridor |
| `slope` | (23.50, -16.50), heading -157.5° | foot of a 5 m, 12.4° slope, facing uphill |
| `rock` | (-4.12, -7.41), heading 67.5° | a lone 8.7 cm rock 2 m ahead under the left wheels |
| `turn` | (-3.00, 3.00), heading -67.5° | no rock of 10 cm or larger within 1.5 m |

The spawn height is the highest collision point under the rover's footprint plus the base height (0.136 m) plus
4 cm, so the rover never starts inside the ground and drops only a few centimetres.

The `flat` site is the nearest spot to the centre that passes the flat-site rules (footprint slope under 5°: it
has 4.85°; no rock of 5 cm or larger within 0.98 m), but its 4 m corridor is not level: it falls 7.4° over the
first 3 m, reaches a 9.4° footprint slope and holds 2 rocks of 5 cm or larger. Only 164 of 35,721 candidate spots
on this terrain pass the rules; the nearest with a rock-free corridor of at most 5° is 26 m out, at (-25.5, -4.0)
(`docs/impl/final_build.md` 11.2). Moving the site is an open decision (`docs/PROGRESS.md`, phase 7). All four
sites moved in the final build; the phase-3 sites were flat (-4.5, -0.5), slope (11.0, -4.0), rock (-5.11, -5.22)
and turn (-4.5, 0.0).

### 1.9 What stayed the same

- The robot description (the launcher only swaps the CAD collision meshes in memory, see the README), the bridge
  configuration, topic names, TF frames, the model name `togo` and the controller names.
- ROS domain 29, CycloneDDS from `REPO/config/cyclonedds.xml`, `use_sim_time`.
- The SLAM and Nav2 stacks, apart from DLIO's `odom/gravity`.
- The sim LiDAR (the robot's existing `gpu_lidar`).
- `lunar_surface.world` and the rest of the repository. The only change outside `src/lunar_world_gen` is one
  environment line in `src/togo_sim_wsl.sh`, the memory-leak fix of section 1.10, added at the user's request.

### 1.10 Memory and the WSL GPU driver

Each Gazebo process that renders (the GUI, and the server once the robot's LiDAR renders) builds its own copy of
the scene. The GUI holds 3.5-3.8 GB after loading: about 2.1 GB for the rocks, 1.0 GB for the ground and 0.5 GB
base (measured with ground-only and rock-only copies of the world, `docs/impl/leak_diagnosis.md` 5). The server
with the rover holds 3.7-3.9 GB. The old world's GUI needed 0.62 GB.

In WSL, Mesa's D3D12 driver leaks memory on every indirect draw call, and Gazebo's renderer draws that way
whenever the driver allows it. Without the fix the Apollo 16 GUI grew by 5.5-10 MB/s and the server by
2.7-3.5 MB/s; the old world leaked too, only slower. `MESA_EXTENSION_OVERRIDE=-GL_ARB_multi_draw_indirect` makes
the renderer use plain draws: memory stays flat, images and LiDAR ranges are identical, and the speed is
unchanged. `scripts/env.sh` (used by the launcher and `scripts/open_gui.sh`), the validation tools and
`src/togo_sim_wsl.sh` set it. How to use less memory and how to check the fix: README sections 4 and 8.

Measured with the launcher on the final world (`docs/impl/e2e_launch.md` 3): over 22 minutes with driving and a
camera tour 1-13 km out, the GUI held 3.69-3.76 GB and the server with its bridges and nodes 3.87-3.88 GB. They
grew by 0.4 MB and 2.1 MB in the last 20 minutes, and the leak's `rw-s /dev/zero` mappings stayed constant. WSL
used about 8.0 GB in total, 4.4 GB without the GUI.

## 2. Camera and physics realism notes

The world was built for LiDAR navigation first. These are the known gaps for camera work and for wheel-soil
physics, with what was measured and what could be done.

### 2.1 Ground albedo is uniform (FID-13; cameras only)

Measured on `textures/regolith_albedo.png`: the linear mean colour is the calibrated (0.120, 0.107, 0.088), the
luminance varies by 4.9 % overall but only by 1.6 % at features of 20 cm and larger, and every regolith material
uses the same map. Real Apollo 16 ground shows bright ejecta around fresh craters (normal albedo 0.15-0.17
against 0.11-0.15), ray patches and darker mature soil. None of that exists here.

What it means:
- Camera images show no large-scale brightness landmarks. Visual odometry and place recognition get their
  texture from rocks, shadows and the fine texture.
- The 2 m texture repeat is barely visible, but it is the same everywhere.
- The LiDAR is not affected.

Possible generator change (not made): a low-frequency albedo field (±10-20 % at 5-50 m, correlated with the rock
density) and bright halos around fresh craters, through vertex colours or a second UV set; and a second detail
map with a non-commensurate repeat.

### 2.2 Earthshine is calibrated, and therefore invisible (FID-14; cameras only)

The calibration puts earthshine at about 3e-5 of the sunlight (range 1e-5 to 1e-4), from the Earth at 75°
elevation in the north-west. Since the fix round the world uses exactly that
(`lighting.realistic.earthshine_intensity: 3.0e-5` in `config/apollo16_core.yaml`); before, it used 1e-2.

Why it renders as nothing: a camera exposed for sunlit regolith records earthshine at 3e-5 of the sun as zero,
and Gazebo's cameras have a fixed exposure. At the calibrated value earthshine adds 0.006 grey levels. (A real
camera exposed for sunlit ground would also show black shadows; detail in Apollo photographs' shadows comes mostly
from light reflected by nearby sunlit ground.) The light stays in the SDF with the correct direction and colour,
for a future camera with exposure control or a night-time lighting mode.

What changed in the images: on the phase-3 world, with earthshine at 1e-2, sunlit ground was about 51 of 255 grey
levels and cast shadows 3 (`review_usability.md` T4, rover view); from the light geometry, the earthshine
supplied about 2 of those 3 grey levels. Now the shadow fill comes only from the scene ambient (0.01), which
stands in for light reflected by sunlit terrain (the calibration gives that as up to 1-3 % of direct light for
vertical faces and about 0 for level ground): about 1 grey level is expected (an estimate, not measured: the
visual review is deferred until the camera work). `inspect` lighting is unaffected in practice (fill 0.35,
ambient 0.25).

For night scenes (sun below the horizon) earthshine would be the only light; that needs a separate lighting mode
with all intensities scaled up, which does not exist yet.

### 2.3 Rigid regolith, uniform sinkage, shared friction (FID-15)

Measured on the final world (`reports/static_checks.json`):
- **Sinkage.** The collision surface lies a uniform 20.3 mm below the visual ground (median on rock-free ground;
  1st-99th percentile 6-34 mm). With Bullet's 10 mm collision margin the wheels sink about 1 cm into the visual
  ground everywhere. That matches the average Apollo and Yutu-2 wheel sinkage, but real fresh crater rims and
  ejecta let wheels sink up to 5 cm. There is no deformation, no ruts and no slip-sinkage: the regolith is a rigid
  mesh with Coulomb friction 0.4.
- **Friction.** All 108 collision meshes use mu 0.4. The rocks are part of the ground collision (Bullet friction is
  per body), so they also use 0.4; the separate rock friction key was removed in the fix round.
- **Pebbles.** The 199,131 core pebbles of 2-5 cm have no collision: wheels pass through them in camera images.
  The 16,546 rocks of 5 cm and larger are part of the collision surface.
- **Ring rocks.** Outside the core, rocks of 30 cm and larger are part of the 25 cm ring collision grid, widened by
  one grid step so that the whole rock is covered: they collide up to 25 cm before their visual surface
  (`docs/impl/final_build.md` 3.3). The rover is meant to stay in the core.

What it means: driving dynamics are right on average (traction-limited start, correct turning), but wheel-soil
interaction is not terramechanics. Do not use the world to study sinkage, slip-sinkage or bulldozing resistance.

Possible changes (not made): a spatial ground offset (for example 4 cm lower on fresh rims and ejecta), and the
rocks as a second static body with mu 0.6 (its real-time cost needs measuring).

### 2.4 LiDAR notes

Measured on the phase-3 world (`gz_validation.md` 4):
- Gazebo's `gpu_lidar` quantises range: height errors of about 5 mm at 2-4 m and 24 mm at 8-16 m. A 2-3 cm
  pebble gets a return in about 60 % of cases within 2 m; 10-20 cm rocks are found reliably to about 4 m.
- No returns leak through tile seams, and none reach beyond the 500 m ring from inside the core.
- Keep experiments at least 10 m inside the core edge: around |x| or |y| = 50 m the small rocks and the fine
  relief stop (`review_fidelity.md` FID-04). The final world softens that edge (the fine relief fades over 3 m,
  small rocks thin out over 6 m outside the core), but the core edge is still where the detail ends.

On the final world (`docs/impl/final_validation.md` 5; 18 capture poses):
- 0 returns beyond the ring, 0 below the ground, 0 leaks through tile seams, no flicker between frames; 4
  floating returns in 260,369, 3 of them unexplained.
- Rock detection is consistent with the Gazebo small-object tests in 7 of 7 size and range bins, two of them close
  to the limit. Small rocks are seen as in phase 3 (2-3 cm: 0.60 within 2 m, 0.13 at 2-4 m).
- An exact ray cast against the rendered meshes matches 99.9 / 99.5 / 95.5 / 88.7 % of the ground beams at
  0-1 / 1-2 / 2-4 / 4-6 m, and every rock of 10 cm or larger within 4 m that the ray cast hits also returns.
- The cloud (209 x 122 points at 20 Hz in sim time) has no per-point time field, and its intensity is always 0
  (`docs/impl/e2e_launch.md` 6). A realistic Robin W mode is the planned next step.

### 2.5 Other camera notes

- The far field and the horizon cast no shadows. They could not shade the core at this sun elevation, but camera
  views toward the horizon miss the 100-250 m shadows of distant hills (`review_fidelity.md` FID-17).
- Rock shapes repeat: 50 prototypes, randomised by scale, yaw, tilt and burial.
- The close-range artefacts that `review_fidelity.md` found (perched rocks with a gap underneath, rock rings
  around fresh craters, one over-steep compound crater, rocks inside other rocks, plateau-shaped fillets, crease
  lines at the 10 m tile seams) were fixed in the fix round (`docs/impl/fix_rocks.md`,
  `docs/impl/fix_terrain.md`, `docs/impl/fix_build.md`). The visual review of the final world was not done
  (deferred until the camera work); the GUI screenshots of the end-to-end run showed the terrain, rocks, craters
  and the rover with its shadow correctly (`docs/impl/e2e_launch.md` 4).
