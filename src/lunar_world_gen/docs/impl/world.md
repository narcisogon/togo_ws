# World stage: SDF, lighting, spawn sites, world_info, physics plugin, launch

Owner: `world` agent, 2026-10-25. Implements DESIGN.md sections 8, 10, 11 and 12. Every Gazebo/ROS run used
`GZ_PARTITION=claude_world`, `ROS_DOMAIN_ID=71`, a private `ROS_HOME`, CycloneDDS with the repo
`config/cyclonedds.xml` (the user's setup), setsid + process-group stop. Scratch directory, test worlds,
logs and helper scripts: `/home/nargon/togo_assets/lunar/experiments/impl_world/` (called `EXPW` below).

## 1. What was built

| file | purpose |
|---|---|
| `lunar_world_gen/world/sdf.py` | world SDF writer: 3 variants, URI/texture checks, `gz sdf -k` validation |
| `lunar_world_gen/world/lighting.py` | sun / earthshine / overhead fill from `cfg.lighting`, direction checks |
| `lunar_world_gen/world/spawn.py` | spawn height and the flat / slope / rock / turn site search |
| `lunar_world_gen/world/world_info.py` | `world_info.yaml` writer and reader |
| `tools/gz_physics_bullet_lunar/` | `patch_bullet_lunar.py`, `bullet_velcmd.patch`, `bullet_mesh_friction.patch`, `build_lunar_bullet_plugin.sh`, `incline_test.py` |
| `launch/apollo16_sim.launch.py` | robust launch (by file path); also the `wait_for_sensors` helper when run as a script |
| `scripts/run_apollo16_sim.sh` | WSL wrapper mirroring `src/togo_sim_wsl.sh` |
| `tests/test_world_sdf.py` | 26 unit tests |

## 2. Public API

```python
# world/sdf.py
VisualSpec(name: str, uri: str, material: str, cast_shadows: bool = True, visibility_flags: int | None = None)
CollisionSpec(name: str, uri: str, mu: float, mu2: float | None = None)
MaterialDef(name, roughness, metalness=0.0, base_color=(...), albedo_map=None, normal_map=None,
            roughness_map=None, metalness_map=None, diffuse=None, specular=(0, 0, 0, 1))
  .effective_diffuse()   # (1,1,1,1) with an albedo map, else base colour
  .effective_roughness() # 1.0 with a roughness map, else roughness
default_materials(cfg) -> dict[str, MaterialDef]          # regolith_core|ring|far|horizon, rocks
write_world(cfg, layout, visuals, collisions, variant="", *, materials=None, out_path=None,
            require_textures=True, validate=False) -> Path      # variant in "", "inspect", "stock"
write_all_variants(cfg, layout, visuals, collisions, **kw) -> dict[str, Path]
physics_settings(cfg, variant) -> PhysicsSettings; find_missing(uris, base_dir) -> list[str]
validate_sdf(path) -> (ok: bool, output: str)                   # runs `gz sdf -k`
read_engine_filename(sdf_file) -> str | None

# world/lighting.py
lighting_setup(cfg, mode="realistic"|"inspection") -> LightingSetup(ambient, background, shadows, lights)
sun_light(cfg, event=None) -> LightSpec; earth_light(cfg, intensity); fill_light(intensity)
direction_from_el_az(el_deg, az_deg) -> np.ndarray; check_direction(vec, el, az) -> unit vector

# world/spawn.py
find_sites(cfg, collision: Grid, rocks=None, height_fn=None, region: Rect | None = None,
           names=None) -> SiteSearchResult(sites: dict[str, Site], missing: dict[str, str], params)
spawn_z(collision: Grid, x, y, cfg) -> float
RockTable.from_any(npz path | csv path | dict of arrays | structured array | None)
Site(name, x, y, z, yaw, description, ground_z_max, ground_z_center, ground_z_visual,
     slope_deg, roughness_m, relaxed, extra).to_dict()

# world/world_info.py
write_world_info(cfg, layout, sites, *, path=None, triangle_counts=None, rock_counts=None, extra=None) -> Path
read_world_info(path) -> dict;  site_pose(info, name) -> (x, y, z, yaw)
```

## 3. How to run

```bash
# 1. physics plugin, once per machine (~40 s, no sudo) -> ~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so
bash src/lunar_world_gen/tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh
# 2. simulation (WSL): default world ~/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf, site flat
bash src/lunar_world_gen/scripts/run_apollo16_sim.sh [--site flat|slope|rock|turn] [--variant inspect|stock]
     [--world /abs/world.sdf] [--pose X Y Z [YAW]] [--headless] [--headless-rendering] [--rviz]
# or directly (any shell with ROS + the native install sourced):
ros2 launch src/lunar_world_gen/launch/apollo16_sim.launch.py site:=flat [world:=...] [variant:=stock]
# check: ros2 control list_controllers ; drive test: python3 src/lunar_world_gen/tools/rover_drive_test.py --out DIR --gravity 1.62
# unit tests (venv, no ROS sourced):
cd src/lunar_world_gen && /home/nargon/.venvs/lunar_world/bin/python -m pytest tests/test_world_sdf.py -q
```

Launch arguments: `world`, `variant`, `world_info`, `site`, `robot_x/y/z/yaw`, `rviz` (false), `rgbd_point_fixer`
(false), `use_sim_time` (true), `tf_prefix`, `ns`, `extra_xacro_args`, `gz_verbosity` (4), `headless_rendering`
(false), `lidar_topic_gz`, `sensor_wait_timeout` (600 s), `spawner_cm_timeout` (300), `spawner_service_timeout`
(60), `spawner_switch_timeout` (30). The wrapper prints the resolved world, physics engine and spawn pose
before starting (same resolution code as the launch file).

## 4. Design details

**SDF** (section 8). Systems: Physics (`<engine><filename>`), UserCommands, SceneBroadcaster only. `physics
type="ignored"`, step `physics.step_s[engine]`, RTF 1, update rate 1/step. Patched engine by absolute path
(`cfg.paths.physics_plugin`), stock = `gz-physics-bullet-plugin` (variant `stock`: 4 ms, 250 Hz). Gravity
`0 0 -1.62`, magnetic field 0. `CUSTOM_SURFACE` 1737400/1737400, ENU, site lat/lon, elevation = href, heading 0
(`site.yaw_deg != 0` raises: the heading sign convention was not verified). Scene: black background, shadows,
no grid/origin visual. One static model `apollo16_terrain`. Every mesh URI and, by default, every texture of a
used material is checked before writing (`FileNotFoundError` lists all missing files; nothing is written).

**Lighting.** The configured sun vectors are checked against elevation/azimuth (all four events agree to
< 0.1 deg, all point down). Realistic: sun (eva1_start, el 21.94, az 84.66, intensity 1, shadows), earthshine
from el 75 / az 321 (direction (0.163, -0.201, -0.966), intensity 0.01, colour (0.55, 0.65, 0.90), no shadows),
ambient 0.01. Inspection: eva2_start sun + earthshine + overhead fill 0.35 + ambient 0.25. Light poses carry no
rotation (the `<direction>` is in the light frame).

**Spawn** (section 11). `z = max(C over r = hypot(0.55, 0.40) = 0.680 m disc) + 0.136 + 0.04`. Footprint
statistics (max, LS-plane slope, residual RMS) are exact over the 2 cm disc (3 632 nodes), evaluated for a
0.5 m candidate lattice with one matrix product per chunk. Rules as in the module docstring; additions:
flat chooses the yaw whose 4 m forward corridor has the fewest rocks >= 5 cm (the standard drive test runs
there); rock places the target on the LEFT wheel line (body y = +0.2829 m) 2 m ahead so a wheel climbs it
(centred, it would pass between the wheels under the 0.10 m clearance); the roughness limit is the median of
the slope/rock-qualified candidates unless `spawn.max_roughness_m` is set; when no candidate meets a rule
the least-violating one is used and the relaxation is recorded in the site (`relaxed`).

**Physics plugin** (section 10). Two patches of gz-physics 7.6.0 classic Bullet:
1. *velcmd* (rover baseline): `SetJointVelocityCommand` keeps the parent's angular velocity.
2. *mesh-friction*: gz-sim builds mesh shapes through `AttachMeshShape` (no SDF), but first hands the full
   model DOM to `ConstructSdfModel` -> `ConstructSdfCollision`, which reads mu/mu2 for every collision and only
   builds primitives. The patch records (mu, mu2) of mesh collisions keyed by (link id, collision name) in
   `ConstructSdfCollision` and applies them to the link's rigid body in `AttachMeshShape` (Bullet friction is
   per body; last collision wins, as for primitives). For mu == mu2 it uses the scalar
   `setFriction(mu)` (anisotropy off): product rule with the other body, exact on sloped world-frame meshes.
   The primitives' scheme `setFriction(1) + anisotropic (mu, mu2, 1)` scales the body-z component of the
   friction direction by 1 and would give 0.39 instead of 0.2 on a 20 deg world-frame mesh incline; it is
   used only for mu != mu2 (not exercised by the generator). The server log shows one
   `[lunar patch] mesh collision [<name>]: friction mu .., mu2 ..` line per mesh collision.

**Launch** (section 12). t = 0: gz server via `ros_gz_sim gz_sim.launch.py` (`-r -s -v 4 <abs world>`, same as
`start_world.launch.py`), clock bridge, robot_state_publisher (same xacro arguments as `control.launch.py`),
`sim_bridge` (same bridge.yaml and QoS overrides) and `create` (`-name togo -topic robot_description -x -y -z -Y`).
Because robot_state_publisher starts at t = 0 (not at the 10 s timer), the robot spawns as soon as the world
is up. `wait_for_sensors` (this launch file run by the system python with gz-transport) waits for
`/sensors/lidar3d_0/scan/points` to be advertised and for 0.5 s of sim time to elapse; then the
joint_state_broadcaster spawner starts, and when it exits the platform_velocity_controller spawner starts
(`--controller-manager-timeout 300 --service-call-timeout 60 --switch-timeout 30`); after that RViz / the point
fixer if requested. If the world's physics plugin path does not exist, the `_stock` sibling is loaded.

## 5. Measurements

### 5.1 Physics plugin

Build: 36-37 s (cmake + `make -j6` of one target, nice 10), 895 760 bytes, only the upstream
`unused variable 'id'` warning. Patch script verified idempotent on the real 7.6.0 source; the two unified
diffs apply with `patch -p1 --dry-run`.

**(a) Box (0.2 m, 1 kg, mu 1) on a 20 deg incline GLB in world coordinates, g 1.62, 4 s** (`incline_test.py`,
`EXPW/incline/results.md`; tan 20 deg = 0.364):

| engine | ground mu | predicted | displacement 0.5-4 s | accel pred / meas (m/s^2) | mu_eff measured |
|---|---|---|---|---|---|
| patched | 0.2 | slides | 1.95 m | 0.250 / 0.242 | 0.205 |
| patched | 0.3 | slides | 0.75 m | 0.097 / 0.092 | 0.304 |
| patched | 0.4 | sticks | 0.000 | 0 / 0 | - |
| patched | 0.8 | sticks | 0.000 | 0 / 0 | - |
| stock | 0.2, 0.3, 0.4, 0.8 | sticks (mesh fixed at 0.5) | 0.000 | 0 / 0 | - |

8/8 as predicted; two runs gave identical accelerations.

**(b) Full robot launch, g 1.62, flat GLB mesh 40 x 40 m (collision 2 cm below the visual), standard preset**
(`EXPW/runs/flat40_*`). "fwd / nominal" = ground-truth distance / 3.0 m; "fwd / limited" = / the command
after the controller's acceleration limits (2.937 m).

| run | ground mu | step | fwd (m) | fwd / nominal | fwd / limited | v_ss ratio | in-place yaw-rate ratio | turn dyaw (cmd 206 deg) | drift in turn | RTF |
|---|---|---|---|---|---|---|---|---|---|---|
| patched r1-r3 | 0.4 | 1 ms | 2.836-2.845 | 0.945-0.946 | 0.966-0.969 | 1.016 | **0.973 / 0.973 / 0.973** | 191.0-191.7 | 0.05 m | 0.85-1.00 |
| stock r1-r2 | (0.5 eff.) | 4 ms | 2.874 | 0.958 | 0.978 | 1.016 | **0.665 / 0.661** | 131.6-132.6 | 0.11-0.15 m | 0.99-1.00 |
| patched | 0.2 | 1 ms | 2.638 | 0.879 | 0.898 | 1.016 | 0.969 | 182.6 | 0.07 m | 0.98 |
| patched | 0.8 | 1 ms | 2.944 | 0.979 | 1.002 | 1.016 | 0.968 | 194.2 | 0.06 m | 0.98 |

- In-place yaw tracking >= 85 %: met (97.3 %, independent of mu, as the rover model predicts once the
  artificial damper is gone). The stock plugin at 4 ms reaches 66 %.
- Forward >= 95 %: met against the controller-limited command (96.6-96.9 %) but **not** against the nominal
  3.0 m (94.5-94.6 %) at mu 0.4. The start is traction-limited (mu g = 0.65 m/s^2 < the 2 m/s^2 controller
  limit); steady-state speed tracking is 1.016 (= wheel-radius mismatch) and steady-state slip ~0. mu 0.8 gives
  97.9 %.
- The mesh friction reaches the rover: the forward distance follows mu (0.2: 2.638 m, which equals the rover
  baseline's 2.639 m on a mu 0.2 *primitive* plane with the velcmd plugin; 0.8: 2.944 m). Under the stock
  plugin every mesh behaves as mu 0.5.

### 5.2 Launch sequencing and timelines

Seconds after `ros2 launch` (time-stamped log, `timeline_world.py`). Controllers "active" = the
`Configured and activated` line of the second spawner; `ros2 control list_controllers` confirmed both
active in every run below.

| world (runs) | launch | gz up | world init | robot spawned | LiDAR advertised | both active | spawner exit codes |
|---|---|---|---|---|---|---|---|
| rough20, 20 m, 320 k visual tris (7: r1-r3, wrapper, headless-rendering, NavSat, new materials) | new | 1.1-2.5 | 1.2-2.4 | 1.4-2.6 | 6.6-9.6 | **9.1-12.4** | 0/0 in 7/7 |
| flat40 (7) | new | 1.0-1.5 | 1.4-2.0 | 1.6-2.1 | 6.1-9.5 | **8.1-13.0** | 0/0 in 7/7 |
| old `lunar_surface.world` copy, 116 MB DAE (3) | new | 1.1-1.4 | 1.5-1.9 | 1.7-2.0 | 14.2-15.3 | **16.9-18.3** | **0/0 in 3/3** |
| same old world, control (2) | stock `sim_gz.launch.py` | 0.9-1.0 | 1.3-1.4 | 11.3-11.4 | - | 23.8-24.8 | **one spawner exit 1 in 2/2** |

- The old failure is reproduced by the stock chain in this session (2/2, at 21.8-22.0 s: `Failed getting a
  result from calling /controller_manager/switch_controller in 10.0`, `Failed to activate controller`; the
  controller still came up, as the rover found; earlier reports: 6/6 rover, 22/25 legacy) and is absent with
  the new launch (3/3 clean, controllers ~6 s earlier).
- Render-scene stall after spawn (spawn -> LiDAR advertised): 4.5-7.4 s on the test worlds, 12.2-13.3 s on the
  old world. The spawners start 0.6-0.9 s after the LiDAR appears (0.5 s sim-time check); both controllers are
  active 1.4-3.3 s later. (14 trials with the new launch on the test worlds plus 3 on the old world: 17/17
  without a spawner failure.)
- GPU: every server process mapped `libd3d12.so libd3d12core.so libdxcore.so` (`/proc/<pid>/maps`), including
  `headless_rendering:=true` with DISPLAY/WAYLAND_DISPLAY unset (controllers active 10.5 s).
- Each launch logged 15 `[lunar patch] mesh collision` lines: the terrain collision plus the robot's 14 mesh
  collisions (base_link already had friction 1 through its primitive collisions, so the robot is unchanged).
- Error lines: none from the gz server while running (`[Err]` count 0 in all 17 new-launch logs). The only
  `[ERROR]` lines come after the SIGINT at shutdown: `gz sim` exit 139 (known segfault) in every run, and once
  (flat40 mu 0.2) `parameter_bridge` aborting with "corrupted double-linked list". No process was left behind.

### 5.3 Drive test on the gently rough world (rough20, mu 0.4, g 1.62, site flat, standard preset)

| run | fwd (m) | fwd / nominal | yaw-rate ratio | airborne fraction (fwd) | RTF |
|---|---|---|---|---|---|
| r1 / r2 / r3 / wrapper | 2.732-2.738 | 0.909-0.911 | 0.897 / 0.932 / 0.925 / 0.947 | 0.12-0.14 | 0.77-0.91 |

Bouncing on ~3 cm undulations at 1.62 m/s^2 (12-14 % airborne at 0.5 m/s, as in the rover baseline) costs
forward distance; RTF was measured while other agents loaded the machine (load average 3-5).

### 5.4 Other checks

- **Material colour** (`EXPW/matcheck`, image `img/world_material_check.png`): tiles A (sRGB 0.5 map,
  diffuse 1), B (same map, diffuse = base colour), C (no map, diffuse = base), D (map = sRGB(base), diffuse 1).
  Linear B/A = (0.121, 0.104, 0.089) = base colour: ogre2 **multiplies** `<diffuse>` into the albedo map.
  C and D render identically (78, 74, 67): albedo PNGs are decoded as sRGB. Hence textured materials get
  `<diffuse>1 1 1 1</diffuse>` (the mesh owner measured the same, docs/impl/mesh.md). The trials in 5.1-5.3 ran
  with the earlier `<specular>1 1 1 1</specular>` and scalar roughness 0.95; the writer now emits specular 0
  and roughness 1 with a map (section 6.1). Neither affects physics or the LiDAR.
- **NavSat** (robot GNSS gps_0 on rough20): lat -8.9734458, lon 15.5111080, alt 9.144 m vs the position implied
  by the SDF spherical coordinates: within 0.03 m north, 0.06 m east, 0.03 m up (residual = the robot's ~5 deg
  roll, ignored in the antenna estimate).
- **`gz sdf -k`**: all written variants "Valid." (0.36-0.40 s per file).
- **Site search at full core size** (synthetic 100 m core, 2 cm C grid = 25.0 M nodes, 215 k rocks of which
  16.5 k >= 5 cm with the LM-site SFD, four crater bowls): 8.3 s, peak RSS 756 MB (incl. the grid), 35 721
  candidates. flat, slope (12.6 deg mean, 759 valid runs), rock (6.7 cm target) found without relaxation; turn
  needed one: no 1.5 m radius free of rocks >= 5 cm exists at 1.66 rocks/m^2, so the minimum (1 rock) was used.
- Test worlds (`EXPW/make_test_worlds.py`, written with this module): rough20 (320 000 visual / 80 000 collision
  triangles, sites flat = turn = (0, 0, 0.203), yaw -45 deg) and flat40; slope and rock reported missing (none
  exist), as designed.

### 5.5 Unit tests

`tests/test_world_sdf.py`: **26 passed in 4.2 s** (venv, without ROS sourced). Covers SDF structure of all variants
(systems present/absent, physics, gravity, spherical coordinates, scene, lights, materials, friction links),
URI and texture checks, lighting directions and modes, spawn height rule, all four sites on a synthetic
30 m grid with a 12.5 deg ramp and rocks, missing-site reporting, catalog inputs, world_info round trip, the
launch-file world/pose resolution and stock fallback, the patch script (synthetic and real 7.6.0 source) and
the shell scripts' syntax.

## 6. Contract changes (relative to DESIGN.md)

1. **Materials** (section 8). `<diffuse>` of a textured material is 1 1 1 1, not the base colour: ogre2
   multiplies it into the albedo map (measured here, 5.4, and independently by the mesh owner); the base
   colour would darken the ground ~8x. The base colour is `<diffuse>` only for a material without an albedo
   map. With a roughness map the scalar `<roughness>` is 1 (Ogre-Next multiplies scalar x map; the regolith map
   holds 0.95, the rock map the baked values); the configured roughness is used only without a map.
   `<specular>0 0 0 1</specular>` (no specular lobe; what the mesh owner's look tests used).
2. **One extra static link per distinct (mu, mu2)** among the collisions (section 8 says one link): Bullet
   friction is per rigid body. With the current design (all collisions use `physics.ground_mu`) there is exactly
   one link, `terrain`.
3. Mesh-friction patch: scalar friction for mu == mu2 instead of the primitives' anisotropic scheme (exact on
   slopes; see section 4).
4. `write_world` has extra keyword-only arguments (`materials`, `out_path`, `require_textures`, `validate`);
   default texture names are fixed in `sdf.py` (section 7).
5. Spawn sites: flat gets a yaw from a 4 m corridor score; the rock site is offset to the left wheel line;
   rules relax instead of failing (recorded in `relaxed`); the roughness limit defaults to the candidates'
   median.
6. Launch: robot_state_publisher and `create` start at t = 0 (spawn as soon as the world is loaded);
   `rviz` and `rgbd_point_fixer` default to false (the stock chain always starts the fixer, which logs a FATAL
   because the cameras are disabled); the spawner nodes keep the original names (`joint_state_broadcaster`,
   `platform_velocity_controller`).

## 7. What the integration stage must provide

- Meshes in `world_dir/meshes/` (DESIGN 6 names) and `VisualSpec`s with URIs relative to the SDF
  (`meshes/tile_ii_jj_ground.glb`, material `regolith_core`; `..._rocks.glb`, `rocks`; `ring_<side>_*`,
  `regolith_ring`; `far_<side>_ground.glb`, `regolith_far`; `horizon_<side>_ground.glb`, `regolith_horizon`),
  and `CollisionSpec`s (`core_collision.glb`, `ring_<side>_collision.glb`, `far_<side>_collision.glb`,
  mu = `cfg.physics.ground_mu`).
- Textures in `world_dir/textures/`: `regolith_albedo.png` (sRGB), `regolith_normal.png`,
  `regolith_roughness.png` (as written by `materials.make_regolith_textures(cfg, textures_dir)`),
  `rock_atlas_basecolor.png`, `rock_atlas_normal.png` and the grey `rock_atlas_roughness.png` (copied from the
  rock library; the ORM map is not used because ogre2 reads a roughness map's first channel) - or a
  `materials=` dict of `MaterialDef` with other names. Missing textures of a used material abort the write
  (`require_textures=False` relaxes this).
- Then: `write_all_variants(cfg, layout, visuals, collisions)`; `find_sites(cfg, C, catalog, H)` with the 2 cm
  core collision `Grid` C (the same field the collision mesh was made from), the catalog (`rocks_catalog.npz`
  path or dict with at least x, y, D, zone, collide; id and exposed_h used if present) and optionally `H(x, y)`;
  `write_world_info(cfg, layout, result, triangle_counts=..., rock_counts=...)`.
- The physics plugin built once on the machine (section 3); without it the launch uses the `_stock` world.
- Log lines other tools parse (keep stable): `[apollo16_sim] sensors ready at +T s -> spawning ...`,
  `[apollo16_sim] WARNING: wait_for_sensors exited with code N ...`,
  `[apollo16_sim] <controller> spawner exited with code N at +T s`, and `[wait_for_sensors] <topic> advertised
  after T s` (used by `tools/run_robot_tests.py` and by `EXPW/timeline_world.py`).

## 8. Known limitations / not verified

- Forward >= 95 % of the nominal command is missed by ~0.5 % at mu 0.4 (5.1); a criterion against the
  controller-limited command, or mu >= ~0.5, passes.
- Not run: the Gazebo GUI (`gz sim -g`) and RViz options (they would open windows on the user's desktop);
  they are the same commands/includes as in `togo_sim_wsl.sh` / `sim_gz.launch.py`.
- The anisotropic mesh-friction branch (mu != mu2) is untested; `site.yaw_deg` must be 0.
- The slope/rock sites, the full-size spawn search and the real world's load time were exercised on synthetic
  data only (the generated world does not exist yet); render-scene load of the real world is expected to be
  longer than 4-13 s, which the 600 s sensor wait and 300 s controller-manager timeout cover.
- RTF figures were measured while other agents used the machine (load average 2-5).
