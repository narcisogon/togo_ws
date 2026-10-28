# Legacy lunar generator and worlds: review

- Agent: `legacy` (phase 1, research). Date: 2026-10-25.
- Scope: a read-only review of:
  - `src/moon_mesh_generator/*`;
  - the worlds `lunar_surface.world`, `apollo16_test.sdf` and `flat_bullet_test.sdf`;
  - the lunar sections of `docs/LOCAL_CHANGES.md`;
  - `togo_gz/urdf/*.xacro`, expanded with xacro;
  - the rasters in `togo_gz/media/dem/`;
  - the older lunar scripts in `scripts/` and the unused `togo_gz/models/apollo16_terrain` model, both found during the review.
- Constraints: no Gazebo was started and no repo file was changed. Copies and scratch outputs are in `/home/nargon/togo_assets/lunar/experiments/legacy/` (see section 8).
- Evidence labels:
  - **[M]**: measured here with a script or command listed in section 8.
  - **[S]**: read in source code, either in this repo or in upstream gz code.
  - **[L]**: taken from the user's earlier ROS logs in `~/.ros/log`.
  - **[I]**: inference, not verified.
- Installed stack [M]: gz-sim 8.11.0, sdformat 14.9.0, gz-physics 7.6.0, gz-rendering 8.2.3, gz-sensors 8.2.2, gz-common 5.8.0, gz-math 7.6.0. The venv has Pillow 13.0.0 and numpy 2.5.4. The system python has Pillow 10.2.0 and numpy 1.26.4.

## 0. Key findings

1. **The legacy terrain does not use any Apollo 16 data.** It is a procedural 30 m x 30 m patch with 0.97 m of relief.
   - It was made from a 4096 x 4096 16-bit PNG that was designed for a 50 m x 1.2 m world. The mesh script uses 30 m x 1.0 m instead, so every slope is 1.39 times steeper than designed [M].
   - Median triangle slope of the visual mesh: 17.5° (95th percentile 36°, 99th 56°); 60 % of triangles are steeper than 15° [M].
   - Fresh craters are pits: depth/diameter is 0.50 for large craters and 0.86 for medium craters as meshed. Fresh lunar simple craters are about 0.2 [M].
   - Every crater carries a circular step of about 2 cm, because the crater profile has a discontinuity [M].
2. **The heightmap generator (v11) is broken and cannot rebuild the PNG.** Its `stamp_crater()` is a pasted patch with `...` placeholders and raises `NameError` on its first call [M]. The cached `.pyc` was compiled from the same broken source, so the working v11 code is lost [M].
3. **The visual and collision ground differ.** The 769² visual mesh and the 257² collision mesh differ by 10.7 mm RMS, by ±25 mm at the 1st and 99th percentiles, and by up to 168 mm at worst. 3.7 % of visual vertices are more than 2 cm away from the collision surface [M].
4. **Neither "slip map" is a slip map** (section 5).
   - `NAC_DTM_APOLLO16_slipmap.tif` is the 2049² DTM crop re-quantised to 8 bits: 3.68 m per digital number, matching for 99.99 % of pixels [M].
   - `slipmap-2048.tif` is an 80 m x 80 m mask of zones around rocks, centred on the LM site. It was rasterised from `models/apollo16_terrain/placed_rocks.xml`; 82 % of the rocks in its area match a blob within 0.25 m [M].
   - Neither can serve as traversability ground truth.
   - **`placed_rocks.xml` is the most valuable legacy dataset.** It holds 52,522 rocks from 0.1 to 5 m, inside ±450 m around the LM, registered to the NAC DTM [M].
5. **The 2049² DTM crop used by `apollo16_test.sdf` does not contain the landing site.**
   - It is the centre of the DTM, at 9.130°S 15.541°E. The LM lies 4.75 km north and 1.18 km west of that centre, outside the 4.1 km crop.
   - The crop shows the hilly northern flank of Stone Mountain [I]: 671 m of relief and a median slope of 11° at 2 m spacing [M].
6. **Compatibility points for the new world** (section 6).
   - The world must load Physics with classic Bullet, UserCommands and SceneBroadcaster. Contact is optional.
   - It must **not** add Sensors, Imu, NavSat or PosePublisher, because the robot URDF adds them [M].
   - It should add `<spherical_coordinates>`, preferably `CUSTOM_SURFACE` with both axes at 1,737,400 m. The `MOON_SCS` ellipsoid differs from the DTM sphere by 0.19 % north-south and 0.04 % east-west [M].
   - Gravity −1.62 needs DLIO `odom/gravity` set to 1.62. Both values are currently 9.82 [S].
7. **The launch chain was already fragile in the old lunar world.**
   - In 25 recorded launches of `lunar_surface.world`, one of the two controller spawners failed in 22 [L]: `platform_velocity_controller` 14 times and `joint_state_broadcaster` 8 times. In each case `switch_controller` did not answer within 10 s.
   - Flat and empty worlds never failed (0 of 7) [L].
   - A heavier new world risks making this worse.
8. **Classic Bullet in gz-physics 7 builds terrain meshes as `btGImpactMeshShape`** [S].
   - It has no heightmap support.
   - Of the surface parameters it reads only `<ode><mu>` and `<mu2>`. Restitution, rolling friction, `<bullet>` friction and the ODE `kp`/`kd`/`max_vel`/`min_depth` are ignored [S].
   - The number of collision triangles is therefore a direct CPU cost to benchmark.

## 1. Inventory

| Item | What it is | State |
|---|---|---|
| `moon_mesh_generator/generate_lunar_world.py` (v1 "all-in-one", 1015 lines) | 2048² heightmap for 50 m x 1.2 m, 513² Collada mesh, 56 rocks, SDF writer, auto-copy into `/home/er4-user/ws/install/...` | Parses [M]. The 16-bit save raises `ValueError` under the venv's Pillow 13 [M]. Absolute paths belong to another machine. |
| `moon_mesh_generator/generate_lunar_heightmap.py` (v11) | 4096² heightmap with min-carved craters | **Broken**: `NameError` at the first `stamp_crater()` call [M] |
| `moon_mesh_generator/generate_lunar_mesh.py` (v5) | PNG to two Collada meshes over 30 m x 1.0 m: visual 769², collision 257² | Its float path reproduces the current visual mesh exactly (max \|Δz\| = 0) [M] |
| `moon_mesh_generator/lunar_terrain.png` | 4096² `I;16` heightmap, values 75 to 65299 | Only copy of the v11-era heightmap; cannot be regenerated |
| `moon_mesh_generator/lunar_surface.world` | v1 output: 56 rocks, default DART, gravity −1.62, absolute `/home/er4-user` URIs | Superseded by the installed world |
| `moon_mesh_generator/{points_viewer.py, Points_trajectory_viewer.py, PointsData.xyz, PointsData_preview.png}` | Open3D viewers for SLAM runs; `PointsData.xyz` holds 657,721 points at km-scale coordinates | Unrelated to world generation |
| `togo_gz/media/lunar_terrain.dae` | Visual mesh: 116.5 MB, 769² vertices, 1,179,648 triangles, 3.906 cm spacing | In use [M] |
| `togo_gz/media/lunar_terrain_collision.dae` | Collision mesh: 11.8 MB, 257² vertices, 131,072 triangles, 11.72 cm spacing | In use [M] |
| `togo_gz/media/lunar_terrain_collision.png` | 1025² `I;16`, the 4096² PNG resampled bicubically (max error 2e-5) | Unused leftover, gitignored [M] |
| `togo_gz/worlds/lunar_surface.world` | Current lunar world: classic Bullet, Earth gravity 9.82 for an A/B test, no rocks | In use; the default of `togo_sim_wsl.sh` |
| `togo_gz/worlds/apollo16_test.sdf` | DEM heightmap world using the 2049² crop at 4098 x 4098 x 939 m | Will not load on this machine: absolute `/home/er4-user/ws/src/...` URIs |
| `togo_gz/worlds/flat_bullet_test.sdf` | Flat plane with μ = 0.2, classic Bullet, gravity −9.82 | Diagnostic for the skid-steer problem |
| `togo_gz/models/apollo16_terrain/` | `model.sdf` points to a missing synthetic heightmap TIF; `placed_rocks.xml` (6.1 MB, 52,522 rocks) | Not used by any world [M] |
| `scripts/make_lunar_heightmap_world.sh` | LOLA 118 m global DEM crop, then `gdal_translate -ot Byte -scale` to an 8-bit PNG, then a heightmap world | Old experiment; puts Sensors and Imu in the world |
| `scripts/make_lunar_mesh_world.py` / `.sh` | 8-bit image to an OBJ grid plus a world | Old experiment; puts Sensors and Imu in the world |
| `scripts/make_enhanced_lunar_world.py` | Pure-Python procedural OBJ with craters, mounds, ridges and a spawn pad; 4 ms physics step | Old experiment; puts Sensors and Imu in the world |
| `scripts/prepare_nac_preview_height.py` | Turns a shaded-relief preview PNG into a pseudo-height image ("not metrically true elevation") | Do not reuse |

The native install `~/togo_ws_native_clean/install/togo_gz/share/togo_gz/{worlds,media,media/dem}/*` consists of symlinks into the OneDrive repo under `/mnt/c` [M]. Gazebo therefore reads the 116.5 MB Collada file across the slow 9P bridge at every start.

## 2. (a) Algorithms worth keeping

| Function (file:lines) | What it does | Parameters | Verdict for the new generator |
|---|---|---|---|
| `fbm_terrain(size, H, rng)` (`generate_lunar_world.py:66-77`, same code in `generate_lunar_heightmap.py:103-114`) | FFT spectral synthesis of isotropic fractional Brownian motion (fBm). Complex white noise (uniform phase, Rayleigh(1) amplitude) is multiplied by \|f\|^(−β/2) with β = 2H + 2; the DC term is zeroed and `real(ifft2)` taken. The output is periodic, so tiles join seamlessly. | H = 0.80, so the 2-D power spectrum falls as f^−3.6. Frequencies are in cycles per pixel. A "tilt" of 0.04·X + 0.03·Y (X within ±0.04, Y within ±0.03) is added in raw units, then the result is min-max scaled to [0.10, 0.45]. | **Keep the method** (vectorised, seeded, tileable). Change it to physical units (cycles per metre), set the amplitude from a target power spectrum or RMS slope instead of min-max scaling, and use it only to add detail below the DTM's resolution, at wavelengths under about 4 to 8 m, so the 2 m DTM heights stay exact. |
| `bandpass_noise(size, f_lo, f_hi, rng)` (`:80-90`, `:117-127`) | Gaussian white noise times the difference of two Gaussian rings, clipped at 0. In practice this is **one Gaussian ring at f_hi with σ = 0.3·f_hi**: none of the energy lies below f_lo and about 50 % lies above f_hi [M] (pitfall B5). | Bands (0.004, 0.02), (0.02, 0.06), (0.06, 0.16), (0.16, 0.38) and (0.38, 0.5), with weights 0.018, 0.011, 0.006, 0.003, 0.002 (v1) or 0.008, 0.005, 0.003, 0.001 (v11). "Patches" use (0.02, 0.06) at 0.167/0.06 and (0.005, 0.015) at 0.10/0.04. | Replace with a proper annular band-pass, or fold into the spectrum-based fBm. |
| `add_crater(hmap, cx_n, cy_n, r_n, depth, rim_h, degraded)` (`generate_lunar_world.py:93-128`) | Additive crater profile in two variants. **Fresh**: bowl `depth·((d/0.88)^2.2 − 1)` for d < 0.88; rim `rim_h·exp(−((t−0.42)/0.22)²)` with t = (d − 0.88)/0.52 for 0.88 ≤ d < 1.40, peaking at d ≈ 1.10 r; ejecta `0.28·rim_h·(1−t)^2.5` with t = (d − 1.4)/1.6 for 1.40 ≤ d < 3.0. **Degraded**: bowl `0.40·depth·(d^2.2 − 1)`; rim `0.35·rim_h·exp(−((d−1.3)/0.35)²)` for 1 ≤ d < 2. The patch extends 3.5 r. | Large craters: 14, r_n 0.015 to 0.025, depth 0.22 to 0.32, rim 0.05 to 0.08, 35 % degraded, seed 52. Medium: 80, r_n 0.005 to 0.012, depth 0.16 to 0.26, rim 0.03 to 0.06, 40 % degraded, seed 62. Scatter layers A, B, C, D: 10000, 1000, 2000 and 3000 craters (v1); 4000, 300, 200 and 200 (v11). All in normalised units. | Keep the two profile families. Make them continuous (pitfall B4). Calibrate depth/diameter, rim height and ejecta decay to lunar values (fresh simple craters d/D ≈ 0.2 and rim height ≈ 0.04 D per Pike 1977, to be confirmed by the science report). Model degradation, for example as diffusion. Drive counts from a size-frequency distribution. |
| `stamp_crater(...)` with **min-carving** (`generate_lunar_heightmap.py:38-100`, broken; design in docstring lines 4-16, layers combined at 263-268) | Bowls go onto a separate zeroed canvas with `np.minimum`, so where craters overlap the deeper bowl wins and offsets do not add up. Rims go onto a separate additive layer, for large and medium craters only. Result: `h = base + canvas + rims`. | `bowl_power` 2.8 by default; 4 to 10 gives flat floors for small craters. | **Keep the idea**: according to the v11 docstring it fixed the "chaotic bumpy floor" caused by thousands of stacked additive stamps. It must be re-implemented because the code is lost. Better still, carve relative to the local surface before the crater rather than to a global zero canvas. |
| `add_ridge(hmap, rng)` (`generate_lunar_world.py:131-207`) | Sinuous ridge along a perturbed straight path with a Gaussian cross-section (σ = 0.7 w). Height and width vary along the ridge via smoothed noise. Returns points for placing rocks. | Width 0.008 to 0.018 and height 0.12 to 0.22 (normalised); 4 ridges. | **Drop.** Wrinkle ridges are a mare feature at km scale; Apollo 16 is highland plains. |
| `place_rocks(...)` and `rock_to_sdf(rock)` (`:556-780`) | Clusters of 4 rocks around 8 of the large craters at 1.1 to 2.0 R (ejecta), 12 rocks on ridge flanks and 12 isolated ones. Size classes, tilt ±0.3 rad, yaw 0 to π, partial burial (box centre at ground + 0.45·sz), grey 0.20 to 0.38 with ±0.03 to 0.04 jitter. | Size-class probabilities 0.4/0.4/0.2 near craters and 0.55/0.35/0.10 isolated; seed 82. | Keep the placement idea (clusters around fresh craters plus a random background), burial and tilt. Replace size classes with a size-frequency distribution, for example from `placed_rocks.xml` (section 5). Make visual and collision geometry match (pitfall B9). |
| `build_mesh(heights, world_size)` (`generate_lunar_world.py:378-419`, `generate_lunar_mesh.py:75-118`) | Regular grid with 2 triangles per cell, (tl, tr, bl) and (tr, br, bl), wound counter-clockwise so normals point up. Smooth, area-weighted vertex normals via `np.add.at`. UVs run 0 to 1 across the whole patch. | none | Keep the normal computation. Vectorise the index loop, which runs 768² = 589,824 Python iterations with two list extends each. The new terrain uses pydelatin triangulations anyway. |
| `write_collada` / `save_mesh` (`generate_lunar_mesh.py:122-230`, `generate_lunar_world.py:422-526`) | Minimal COLLADA 1.4.1 writer: Z up, metres, one Lambert material (0.42, 0.40, 0.38) with reflectivity 0.03, positions, normals and UVs. Every vertex index is written three times (VERTEX, NORMAL, TEXCOORD). | `%.6f` text | Gazebo loads it, and the world ran [L]. It costs about 99 bytes per triangle (116.5 MB for 1.18 M triangles). Keep only as a debug fallback; use GLB. |
| `sample_heightmap(path, grid_res)` (`generate_lunar_mesh.py:45-71`) | 16-bit PNG to float, bicubic resize in PIL `'F'` mode, times `MAX_HEIGHT`. Keeping floats fixed the 8-bit bug in v1. | none | No longer needed if heights stay float32 throughout. |
| Two-mesh scheme (`generate_lunar_mesh.py:234-247`) | Fine visual mesh plus coarse collision mesh, both sampled from the same height field. | 769² and 257² over 30 m | **Keep** (it is the planned design). Build the collision mesh with an explicit low-pass and an error bound, and check the vertical gap against a stated budget (legacy result: RMS 10.7 mm, maximum 168 mm). |
| `flatten_spawn(heights, size_m, radius)` (`scripts/make_enhanced_lunar_world.py:107-127`) | Blends the terrain inside the spawn radius toward the mean of the inner 0.6 R, using a smoothstep between 0.55 R and R. | radius 8 m | Optional, for a guaranteed safe spawn pad. Better: pick spawn sites by slope and roughness, and compute the spawn height (pitfall B12). |
| Portable relative mesh URIs (`scripts/make_lunar_mesh_world.py:149-151`; the installed world uses `../media/...`) | URIs are written relative to the world file. Gazebo resolves them against the world file's directory, and the installed world loads that way [L]. | none | Keep (or use absolute paths on the WSL disk). |

## 3. (b) Bugs and pitfalls

High means it affects realism or correctness of the new world if repeated. Medium means it costs time or causes confusion. Low means cosmetic.

| # | Sev. | Finding | Evidence |
|---|---|---|---|
| B1 | High | **The v11 heightmap generator is broken.** Lines 38-100 hold a pasted patch with `...` placeholders. `stamp_crater()` reads 13 names that are never defined (`d, x0, x1, y0, y1, rng, crater_canvas, micro_*, ultra_*`) and calls itself. The first call raises `NameError: name 'd' is not defined`. The `.pyc` header (mtime 19:14:43, size 14388) matches the broken source, so no working copy exists. The 4096² PNG in the repo was made by an earlier working revision and cannot be regenerated. The comments in the file also contradict themselves (a "was 200" comment next to counts of 50 and 70, while the code runs 200). | [M] `check_scripts.txt` |
| B2 | High | **The heightmap was designed for one scale and meshed at another.** It was built for 50 m x 1.2 m (`WORLD_SIZE`/`MAX_HEIGHT` in the heightmap scripts) but meshed at 30 m x 1.0 m (`generate_lunar_mesh.py:27-28`). Features in normalised coordinates shrink to 0.6x horizontally and 0.83x vertically, so slopes grow 1.39x. PNG slopes on a 4-pixel baseline: median 12.6° → 17.2°, 95th percentile 27.0° → 35.2°, 99th 46.6° → 55.8°. Visual mesh triangles: median 17.5°, 95th 36.1°, 99th 55.9°, maximum 79.3°; 60.1 % steeper than 15°, 23.2 % steeper than 25°. | [M] `legacy_analysis.txt` |
| B3 | High | **Craters are far too deep.** Depth from rim crest to floor in the PNG, as meshed: fresh large craters d/D = 0.50 (0.41 to 0.57) at D = 0.99 to 1.50 m; fresh medium craters 0.86 (0.49 to 1.34) at D = 0.32 to 0.72 m; degraded craters 0.16 and 0.26. Even at the designed 50 m scale, fresh craters reach 0.36 and 0.62. Fresh lunar simple craters are about 0.2, and small craters are usually shallower. | [M] `crater_dD.txt` |
| B4 | High | **Crater profiles have steps.** Fresh craters: the ejecta term begins at 0.28·rim_h at d = 1.40 r, where the rim Gaussian is already about 0.001·rim_h. Degraded craters: the rim term begins at 0.168·rim_h at d = 1.0 r. Measured in the PNG (median around the circle, 30 m mesh): **+20.1 mm** between r/R 1.38 and 1.42 for 9 fresh large craters, where neighbouring bins change by −1 to −4 mm; **+21.8 mm** between 0.98 and 1.02 for 5 degraded craters, against +5 to +10 mm in neighbouring bins. These are ring-shaped artefacts that LiDAR can see. | [S] `generate_lunar_world.py:111-125`; [M] `crater_profile.txt` |
| B5 | Med | **`bandpass_noise` is not a band-pass.** It is a ring at f_hi with σ = 0.3·f_hi. None of the energy is below f_lo, and 50 to 54 % is above f_hi. The (0.38, 0.50) band peaks at 0.58 cycles per pixel, above the Nyquist limit along an axis, which is pure pixel noise (v11 removed it). Weights apply to unnormalised filter output. | [M] `misc_checks.txt` §1 |
| B6 | Med | **Heights are rescaled every run.** Min-max scaling to [0, 1], then a gamma of 0.95 (v1) or 0.92 (v11), then a σ = 0.7 px blur. Absolute scale is lost, and the relief always equals `MAX_HEIGHT` whatever the features are, so one deep crater rescales everything else. `MAX_HEIGHT` travels outside the file and differs between scripts (1.2 vs 1.0). The "regional tilt" is added in raw fBm units before rescaling, so its size depends on the grid size and the random seed. | [S] |
| B7 | Med | **16-bit PNG handling.** (1) `(h*65535).astype(uint16)` truncates and stores no metadata. (2) `generate_lunar_world.generate_mesh()` turns the 16-bit PNG into 8 bits before resizing: 256 levels, or 4.7 mm terraces at 1.2 m relief (fixed in mesh v4). (3) With the venv's **Pillow 13.0.0**, `Image.fromarray(uint16, mode='I;16')` raises `ValueError: Invalid mode for data type`; the same call works with the system Pillow 10.2.0. (4) For real DTM data, 16 bits over the DTM's full range is 1.43 cm per value (`NAC_DTM_APOLLO16_2049.tif`) and 8 bits is 3.68 m. Both are coarser than a 1 cm detail target. Keep heights as float32 GeoTIFF or NPY in metres. | [M] `misc_checks.txt` §2, `legacy_analysis.txt` |
| B8 | High | **Visual and collision ground differ.** Visual 769² at 3.9 cm and collision 257² at 11.7 cm, both resampled from the PNG. Visual height minus collision height at visual vertices: RMS 10.7 mm; 1st to 99th percentile −24 to +25 mm; extremes −168 and +168 mm; 3.74 % beyond 2 cm, 0.50 % beyond 5 cm. Collision slopes are much gentler (median 5.3° against 17.5° visual). The rover drives on a different surface than the LiDAR sees, by up to 17 cm in craters. | [M] `legacy_analysis.txt` §D |
| B9 | Med | **Rocks look smaller than they collide** (v1 world). Collision is a box (sx, sy, sz), but the visual is a sphere of radius 0.55·min(sx, sy, sz). The LiDAR sees spheres of 0.18 m mean diameter; the rover hits boxes with a mean largest side of 0.25 m and a maximum of 0.64 m. 36 of the 56 rocks lie outside the ±15 m of the current mesh, because positions were computed for 50 m. Their heights come from the v1 2048² heightmap. The installed world removed the rocks but kept the "56 rocks total" comment. | [M] |
| B10 | Low | **Stale metadata.** The world comment says "Mean height: 0.082m / Robot spawn z: 1.282m", but the mesh mean is 0.624 m. Docstrings are out of date: v5 says "50m world", the mesh script says it "selects Bullet Featherstone", v11 says "GRID_RES 1025". | [M] [S] |
| B11 | Med | **Rows run the other way.** PNG row 0 is y = −15 m (south) in the Collada mesh. Regenerating without flipping matches exactly; flipping gives up to 0.78 m error. The mesh is therefore a vertical mirror of the image as viewed. Any texture, slip or rock overlay must account for this. | [M] |
| B12 | Med | **Spawn heights ignore the terrain.** The launch default (0, 0, 0.5) has terrain at 0.447 m, and within ±0.5 m the terrain spans 0.19 to 0.81 m, so the chassis can spawn inside the ground. The `togo_sim_wsl.sh` default (−2, 3, 2.0) drops 1.44 m. The old generator suggested mean height + 1.2 m, ignoring the local terrain. The new generator should publish spawn heights per site. | [M] `misc_checks.txt` §3 |
| B13 | Med | **Machine-specific absolute paths.** `generate_lunar_world.py` writes `file:///home/er4-user/ws/install/...` and copies files into install and src. `apollo16_test.sdf` and `apollo16_terrain/model.sdf` use `file:///home/er4-user/ws/src/...`, which does not exist here. | [S] |
| B14 | Med | **Heavy assets.** One 30 m mesh of 1.18 M triangles in a 116.5 MB Collada text file, read through symlinks from OneDrive on `/mnt/c`. `sim_gz.launch.py` hard-codes a 6 s wait "if terrain loads slowly". There is no tiling, LOD or culling. | [M] [S] |
| B15 | Med | **Classic Bullet specifics** (gz-physics 7 source). Terrain meshes become `btGImpactMeshShape` built from a `btTriangleMesh`, with no margin set. GImpact is meant for dynamic concave meshes; a BVH would suit static terrain better [I]. There is **no heightmap** support. Surfaces: only `<ode><mu>`/`<mu2>` are read, applied as anisotropic friction on a base friction of 1. `<bullet><friction>`, restitution, rolling friction and the ODE contact `kp`/`kd`/`max_vel`/`min_depth` are not read. Most of the legacy surface block therefore does nothing under Bullet, and the `<bullet>` block in `flat_bullet_test.sdf` has no effect. | [S] `gz-physics7/bullet/src/{SDFFeatures,ShapeFeatures}.cc` |
| B16 | Low | **The DART claim is questionable.** Comments say DART "cannot construct terrain mesh or heightmap collisions". But the gz-physics 7 DART plugin implements `AttachMeshShape` (`CustomMeshShape`) and `AttachHeightmapShape` (`CustomHeightmapShape`); only a DEM-specific path is missing. The real cause of the DART failure was never recorded. Possible causes: the 116 MB Collada file, or a GeoTIFF DEM. Not urgent while Bullet works. | [S] `gz-physics7/dartsim/src/ShapeFeatures.cc` |
| B17 | Med | **`apollo16_test.sdf` uses the wrong area.** Its crop is the DTM centre, not the landing site (section 5). It paints the 8-bit elevation "slip map" as a diffuse texture, uses dead absolute paths, and gets visuals from a DEM heightmap (Ogre2 Terra). Whether the GPU LiDAR sees Terra heightmaps was not checked. | [M] [S] |
| B18 | Low | **The `apollo16_terrain` model is inconsistent.** It points to a heightmap TIF that is not in the repo. Its size of 100 x 100 x 10 m does not match the 80 m tile implied by `slipmap-2048.tif`. It uses `slipmap-2048.tif` as both **diffuse and normal map**, but that image is not a normal map. No world uses it. | [M] [S] |
| B19 | Med | **No spherical coordinates in any lunar world.** The robot's two NavSat sensors (1 Hz) then make gz-sim print an unthrottled `gzwarn "Failed to update NavSat sensor enity [..]. Spherical coordinates not set."` whenever they are due and subscribed, about 2 lines per second. They still publish, with latitude and longitude never set. | [S] `gz-sim8/src/systems/navsat/NavSat.cc`; [M] the warning string is in the installed plugin |
| B20 | Med | **The installed lunar world uses Earth gravity** (−9.82) for a temporary traction A/B test, and DLIO `odom/gravity` is also 9.82 (`lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml:70-72`). Both must change together. | [S] |
| B21 | Low | **Lighting comments are wrong.** "SUN — low grazing angle" has direction (−0.5, 0.1, −0.86), which is **59.3° elevation**. The ambient 0.25, the overhead fill 0.25 and earthshine at 0.10 of the sun are not physical; they are fills for visibility. | [M] |
| B22 | Low | **Helper scripts have flaws.** `togo_sim_wsl.sh` has a blank first line before the shebang and a Markdown fence at the end, and passes `use_sim_time:=` although `sim_gz.launch.py` does not declare it (also noted in `.agents/.../operations.md`). The older `scripts/make_*` worlds load **Sensors and Imu at world level**, which this robot already loads (section 6). | [S] |
| B23 | Low | **gz-math's legacy `LOCAL` frame flips x and y.** +100 m local x comes out as 100 m **west**. `LOCAL2` is correct ENU. gz-sim 8 uses `LOCAL2` internally, so NavSat is correct. Any script that converts ENU to latitude/longitude with `gz.math` must also use `LOCAL2`, not `spherical_from_local_position`. | [M] `misc_checks.txt` §4; [S] `gz-sim8/src/Util.cc` |

## 4. (c) Lighting, scene and physics settings, with the stated reasons

| Setting | `lunar_surface.world` (installed) | `moon_mesh_generator/lunar_surface.world` (v1 output) | `apollo16_test.sdf` | `flat_bullet_test.sdf` |
|---|---|---|---|---|
| World name | `lunar_surface` | `lunar_surface` | `apollo16_test` | `flat_bullet_test` |
| Physics | 1 ms step, real-time factor 1, 1000 Hz update rate | same | same | same |
| Engine | classic Bullet (`gz-physics-bullet-plugin`) | DART (default) | DART (default) | classic Bullet |
| Gravity | **0 0 −9.82** (A/B test) | 0 0 −1.62 | 0 0 −1.62 | 0 0 −9.82 |
| Magnetic field | 1e-9 on each axis | same | not set (default) | 1e-9 |
| Spherical coordinates | none | none | none | none |
| World plugins | Physics, UserCommands, SceneBroadcaster, Contact | same | same | same |
| Scene | ambient 0.25, black background, shadows on, grid and origin visual off | same | ambient (0.3, 0.3, 0.35), background 0.05, shadows | ambient 0.4, background 0.15, grid on |
| Sun | directional, intensity 1.2, direction (−0.5, 0.1, −0.86): **59.3° elevation**, from bearing about 101° (east-southeast). Diffuse (1.00, 0.98, 0.95), specular 0.30, casts shadows | same | intensity 2.0, direction (−0.3, 0.1, −0.9): 70.6° elevation. The light's pose pitches it −0.3 rad, which may tilt it toward the zenith if the direction is in the light frame [I]. Specular 0.8 | intensity 1.0, (−0.5, 0.1, −0.9): 60.5° |
| Fill lights | `earthshine`: 0.10, bluish (0.55, 0.65, 0.90), direction (0.3, −0.2, −0.9), so the source is at 68.2° elevation, bearing about 304° (west-northwest), no shadows. `overhead_fill`: 0.25 straight down, no shadows | same | none | none |
| Terrain | visual 769² Collada, collision 257² Collada, relative `../media/` URIs | one Collada mesh for both, absolute URIs | DEM heightmap 4098 x 4098 x 939 m for both | plane 100 x 100 m |
| Terrain surface | `ode` mu = mu2 = 0.8, fdir1 0, slip 0. Restitution 0, threshold 1e5. ODE contact kp 1e7, kd 10, max_vel 0.01, min_depth 0. Only mu and mu2 matter under Bullet (B15) | same; rocks use mu 0.9 | none | ode mu 0.20 and bullet friction 0.20 (the bullet value is not read) |
| GUI camera | (−20, −20, 25), orbit, clip 0.1 to 500 m | same | (0, −500, 500) | none |

Other scene context:
- The robot brings 4 shadow-casting spot lights: the `driving_light_*` lights, range 20 m, 60°/120° cones [M].
- These lights reference `libAttachLightPlugin.so`, a Gazebo-classic plugin. gz-sim should fail to load it, harmlessly [I, not seen in logs].
- The terrain material in the Collada file is Lambert diffuse (0.42, 0.40, 0.38) with reflectivity 0.03. Rocks are grey at 0.20 to 0.38.
- Lighting does not matter to `gpu_lidar`. It does matter to cameras and the GUI.
- The earthshine direction is physically plausible: at 9°S 15.5°E, Earth stands about 72° high toward the north-west, ignoring libration [I].
- The sun is not low. Its intensity and the fills are tuned for visibility, not physics.

Reasons stated in the files:
- `lunar_surface.world`, Physics plugin: "DART's Gazebo 8 SDF loader cannot construct terrain mesh or heightmap collisions. Classic Bullet supports the decimated mesh and, unlike Bullet Featherstone, permits the A300 to skid-steer." (See B16 for doubts about the DART part.)
- `lunar_surface.world`, gravity: "Earth-gravity A/B test for terrain traction. Keep DLIO's odom/gravity setting matched to this value."
- `flat_bullet_test.sdf`: "Classic Bullet A/B test. The previous Bullet Featherstone run could not rotate the skid-steer rover even on this flat plane with low friction. This version changes only the Bullet implementation ..."
- `LOCAL_CHANGES.md`:
  - DART could not build the lunar mesh collision.
  - Bullet Featherstone loaded the mesh but stopped the A300 from rotating.
  - "Classic Bullet was verified to rotate correctly on a flat world and on the lunar mesh."
  - WheelSlip was removed because Featherstone lacks `SetShapeFrictionPyramidSlipCompliance`, which caused a warning every step.
  - Wheel radius is 0.1651 m physically and 0.1625 m in the controller, with a separation multiplier of 1.75.
  - Gazebo and DLIO gravity must always match.
- `generate_lunar_mesh.py:36-39`: "Keep physics substantially lighter than the render / lidar mesh ..." (it still mentions Featherstone, which is out of date).
- v1 generator world: "collision + visual both use same DAE mesh — perfect 1:1 match — rover drives on what it sees". Later replaced by the two-mesh scheme for performance.
- `sim_gz.launch.py`:
  - `robot_z` default 0.5: "Was 15.2 — robot was in freefall when controllers fired".
  - `world_load_delay = 6.0`: "tune up if terrain loads slowly".
  - `controller_spawn_delay` default 10 s: "Increase if terrain mesh is large".

## 5. (d) What the slip maps contain, and the rock catalogue

All rasters, from `gdalinfo` and statistics computed here [M]. The full DTM uses an equidistant cylindrical projection with standard parallel −9° and central meridian 180°, on a sphere of 1,737,400 m.

| File | Grid | Content |
|---|---|---|
| `NAC_DTM_APOLLO16.TIF` | 11985 x 15653 px at 2 m (23.97 x 31.31 km), float32 | Elevation in metres. Range −168.2842 to 770.8965 m (939.1807 m). 9.210 % nodata (17,277,388 px) |
| `NAC_DTM_APOLLO16_filled.tif` | same grid | Holes filled. Identical to the original wherever the original has data (checked on the 2049² window) |
| `NAC_DTM_APOLLO16_2049.tif` | 2049 x 2049 at **2 m (not downsampled)**, UInt16 | **The centre crop of the DTM**: offset col 4968, row 6802; centre 9.1302°S 15.5405°E. Each value is round((h + 168.2842)/939.1807 x 65535); 97.9 % exact, the rest ±1. **1.433 cm per value**. Crop heights −84.7 to 586.4 m (671 m of relief). Slope at 2 m: median 11.0°, 95th percentile 24.1°, 99th 30.1°, maximum 54.6°. `apollo16_test.sdf` sets size z = 939 (the full encoding range). |
| `NAC_DTM_APOLLO16_slipmap.tif` | same grid as the 2049 crop, Byte, nodata 0 | **Not a slip map: the same crop re-quantised to 8 bits.** Each value is round((h + 168.2842)/939.1807 x 255) for 99.99 % of pixels; correlation with elevation 0.99998; values 23 to 205; **3.68 m per value**. Correlation with slope is only −0.17. |
| `slipmap-2048.tif` | 2048 x 2048 RGB at **0.0390625 m** (80.0 x 80.0 m). Stereographic, centred on **−8.9734°, 15.5011°** (the LM site as used by its maker), sphere 1,737,400 m | 95.38 % of pixels are neutral grey (127, 127, 127). 4.62 % are blue only, (0, 0, B) with B from 127 to 241, forming **491 blobs**. Blob equivalent diameters: 0.26 to 1.87 m, median 0.78 m, 71 at 1 m or more. B is **random inside each blob**: its mean is flat at about 184 at every radius, and its correlation with a dome shape is −0.003. So B is not a height, normal or radial profile. |

How the blobs relate to `placed_rocks.xml` [M]:
- Of the 549 rocks inside the 80 m box, **82 %** have a blob centroid within 0.25 m (median offset 0.6 mm).
- This only holds with x = east and y = north. The three other sign combinations match 1.3 to 2.0 % of rocks.
- A blob is about **6 times the rock's diameter**: median 6.0, 5th to 95th percentile 2.8 to 7.7. Most blobs (5th to 50th percentile) are 0.76 to 0.78 m across whatever the rock size, which looks like a minimum footprint. The smallest blob (0.26 m) is cut off by the image edge at y = −39.9 m. Blob size correlates with rock diameter at r = 0.78.
- B does not track rock height (r = 0.07).
- Interpretation [I]: a mask of zones around each rock, with a random value inside, perhaps a slip or soil-class layer from a synthetic-terrain tool. Its meaning and units are not documented anywhere.

`placed_rocks.xml` [M]:
- 52,522 `<RockData insertedDiameter diameter insertedHeight x y z>` entries.
- Frame: x east and y north, in metres from the LM, the same frame as `slipmap-2048.tif`.
  - x ranges from −450.4 to 449.4 m and y from −449.3 to 450.4 m.
  - z is **absolute DTM elevation**. z − DTM(x, y) has a median of −0.58 m and a standard deviation of 0.61 m; with y flipped the standard deviation is 16.5 m.
- `insertedDiameter` is always an integer multiple of 0.0390625 m, the 80 m / 2048 px cell. The rocks were rasterised at the `slipmap-2048.tif` resolution, which suggests 2048² tiles at 3.9 cm.
- The missing `Apollo16-Origin-synthetic-curve_..._x_0_y_0_size_2048x2048.tif` was probably the origin tile [I].
- Provenance is unknown. The format matches synthetic rock generators, for example the power-law Rock Generator in the NASA JSC / JHU-APL DEMmaker suite [I].

| Rock size statistics | Value |
|---|---|
| Diameter percentiles (min / 5 / 25 / 50 / 75 / 95 / 99 / max) | 0.100 / 0.102 / 0.112 / **0.132** / 0.175 / 0.339 / 0.640 / 5.0 m |
| Exposed height `insertedHeight` | median 0.062 m; height/diameter median 0.45 (5th to 95th percentile 0.20 to 0.70) |
| Cumulative count per m², whole 0.81 km² | 0.0649 (≥ 0.1 m), 0.0238 (≥ 0.15), 0.0118 (≥ 0.2), 0.00441 (≥ 0.3), 0.00123 (≥ 0.5), 2.4e-4 (≥ 1), 4.1e-5 (≥ 2) |
| Power-law fit, 0.2 to 2 m | N(>D) = 2.26e-4 · D^−2.45 per m² |
| Extrapolation below the 0.1 m floor (uncertain) | about 0.35 per m² at ≥ 5 cm, about 3.3 per m² at ≥ 2 cm |
| **100 x 100 m box centred on the LM** | **899 rocks ≥ 0.1 m** (0.090 per m²); 52 ≥ 0.3 m; 15 ≥ 0.5 m; 1 ≥ 1 m (largest 1.19 m) |
| 200 x 200 m box | 3451 rocks; 208 ≥ 0.3 m; 57 ≥ 0.5 m; 5 ≥ 1 m (largest 1.39 m) |
| Clustering, Clark-Evans R for D ≥ 0.3 m | 0.953 (close to random, slightly clustered) |

**Are the slip maps useful as traversability ground truth?** No.
- `NAC_DTM_APOLLO16_slipmap.tif` is just elevation at 3.68 m per value, and it covers the wrong area (Stone Mountain flank, not the LM).
- `slipmap-2048.tif` is an undocumented mask of rock zones with random values. It has no physical slip units, covers only 80 m, and is redundant with `placed_rocks.xml`, which holds the exact positions and sizes.

Better ground truth comes from the generator's own geometry:
- slope over the rover footprint, from the collision height field;
- step and rock height compared with the rover's clearance;
- the exact rock list, in the hazard-map grid frame.

**`placed_rocks.xml` is worth reusing.** It gives a ready-made rock population registered to the site, and a size-frequency distribution to calibrate against. The science and site reports should decide whether its synthetic distribution is credible for Apollo 16.

**Site geometry for the site agent** [M]:
- The LM point above sits in the full DTM at col 5402.4, row 5449.8.
- DTM elevation there is 7.786 m above the 1,737.4 km sphere.
- A 120 m box around it spans 4.5 to 12.2 m of elevation.
- It lies 1180 m west and 4753 m north of the 2049 crop centre.
- The `slipmap-2048.tif` origin (−8.9734, 15.5011) should be checked against the LROC-measured LM position. A secondary wiki source gives about −8.978, 15.501.

## 6. (e) Compatibility contract for the new world

Launch chain (unchanged; must not be modified):
- `start_world.launch.py` runs `os.path.join(share/togo_gz/worlds, world)`, so an absolute path is used unchanged. It then runs `gz sim -r -s -v 4 <world>`.
- `-r` starts physics immediately, so the terrain must exist at t = 0.
- A `/clock` bridge (`/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`) uses gz's global `/clock` topic, which works for any world name.
- `ros_gz_sim create -name togo -topic robot_description` starts at about 6 s. It asks `/gazebo/worlds` for the world name and then **waits for `robot_description`**, which robot_state_publisher starts publishing at about 10.6 s after launch. In the last lunar run (2026-10-04 01:10), create started at 6.43 s and logged "Waiting messages on topic [robot_description]" every second. robot_state_publisher started at 10.65 s, and "Entity creation successful" followed at 10.70 s after launch start, 10.29 s after Gazebo started [L].
- Controllers are spawned at about 10 s (300 s timeout). See the risk below.

**World-level systems the world must list.** Because the world lists plugins, gz-sim does not add its default `server.config` systems, so every needed system must be listed. This is documented gz-sim behaviour, not re-tested here. The four below are exactly what every legacy and upstream togo world uses:

```xml
<plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'>
  <engine><filename>gz-physics-bullet-plugin</filename></engine>  <!-- classic Bullet: validated for skid-steer -->
</plugin>
<plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>   <!-- /world/<name>/create (spawn), set_pose, remove -->
<plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/> <!-- GUI scene/state -->
<plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>   <!-- optional; kept for parity, no contact sensors on the robot -->
```

**Systems the world must not duplicate.** The expanded robot URDF (`is_sim:=true`) already contains these [M]:

| System | Notes |
|---|---|
| `gz_ros2_control::GazeboSimROS2ControlPlugin` | Controller manager at 50 Hz with `gazebo_controllers.yaml` |
| `gz::sim::systems::PosePublisher` | link and nested-model poses, `Pose_V`, 50 Hz. Gives `/model/togo/pose`, the ground truth used by `lidarslam_ros2/scripts/togo/*` |
| `gz::sim::systems::Sensors` | No `<render_engine>`, so it uses ogre2 by default |
| `gz::sim::systems::Imu` | |
| `gz::sim::systems::NavSat` | |

The old `scripts/make_*_world` generators put Sensors and Imu in the world. Doing that with this robot would load a second sensor/rendering manager. At best that doubles GPU work; it may also duplicate IMU output [I]. **Do not copy that pattern.**

**Sensors the robot brings** [M]. Topic names do not depend on the world name.

| Sensor | Type and settings | Topic and bridge |
|---|---|---|
| `lidar3d_0` | `gpu_lidar`, **209 x 122** samples over ±60° x ±35° (0.01 rad), 20 Hz, 0.1 to 150 m, range resolution 0.01 m, no noise | `/sensors/lidar3d_0/scan` and `/scan/points`, bridged to `/husky/sensors/seyond/points` |
| `imu_0` | 100 Hz, no noise | `/sensors/imu_0/data` |
| `ins_0_gps_0`, `ins_0_gps_1` | NavSat, 1 Hz each | `/sensors/ins_0/gps_{0,1}/fix` |

Cameras are commented out in `togo_macro.xacro`. The LiDAR range of 150 m means far-field geometry within 150 m of any rover position is visible to the LiDAR, so the seams around the 100 m core must be clean.

**World name.** Nothing in the launch chain uses the world name `lunar_surface`.
- `create` discovers the name itself, and the bridges and ground-truth tools use sensor and model topics.
- The name matters only for `/world/<name>/{stats,control,create,set_pose,pose/info}`, which benchmarks and test tools use.
- Pick a distinct name, for example `apollo16_core`, and record it in the generator config.
- The world must not contain a model named `togo`.

**Spherical coordinates.** These are needed for the 2 NavSat sensors (B19). sdformat 14.9 parses both of the following [M]:
- `MOON_SCS`, which is an ellipsoid with axes 1,738,100 and 1,736,000 m.
- `CUSTOM_SURFACE` with explicit axes.

Recommended:

```xml
<spherical_coordinates>
  <surface_model>CUSTOM_SURFACE</surface_model>
  <surface_axis_equatorial>1737400</surface_axis_equatorial>  <!-- same sphere as the LROC DTM -->
  <surface_axis_polar>1737400</surface_axis_polar>
  <world_frame_orientation>ENU</world_frame_orientation>      <!-- +x east, +y north: same as placed_rocks.xml -->
  <latitude_deg>LAT0</latitude_deg> <longitude_deg>LON0</longitude_deg>
  <elevation>H0</elevation>   <!-- DTM elevation that maps to world z = 0 -->
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

Measured with `LOCAL2`, which is what gz-sim uses: 100 m of local x or y becomes 100.000 m east or north on the DTM sphere with `CUSTOM_SURFACE`. With `MOON_SCS` it becomes 99.957 m east and 100.193 m north. That is an error of up to 1.9 m per km across a 1 km far field [M].

**Gravity and IMU coupling.**
- Use `<gravity>0 0 -1.62</gravity>`.
- DLIO `odom/gravity` in `seyond_dlio_graph.yaml` is **9.82 now**. It must be changed to 1.62 at the same time, by the user, since that file is not in this project's scope.
- The magnetic field is irrelevant because the robot has no magnetometer; keep about 1e-9.

**Physics.**
- Keep the 1 ms step, real-time factor 1 and 1000 Hz update rate. Other rates in the chain: controller manager 50 Hz, IMU 100 Hz, LiDAR 20 Hz.
- Classic Bullet is the only engine validated for skid-steer.
- Collision must be meshes, because Bullet has no heightmaps.
- Collision triangle count is a CPU cost driver, since meshes become GImpact shapes (B15).
- Put `mu`/`mu2` in `<surface><friction><ode>`. The wheels set no friction, so they take the SDF default of 1.

**Assets and URIs.**
- Relative URIs resolve against the world file's directory, as with the installed `../media/...`.
- Absolute `file://` paths also work.
- Put assets on the WSL ext4 disk (`~/togo_assets/lunar/`), not `/mnt/c`.
- No `model://` URI is needed. If one is used, its directory must be on `GZ_SIM_RESOURCE_PATH`. `togo_sim_wsl.sh` sets that to install share plus `/opt/ros/jazzy/share`, and the `togo_description` env-hook adds share directories.
- Environment note: the `togo_description` env-hook also sets **`MESA_GL_VERSION_OVERRIDE=3.3`** if it is unset, whenever `install/setup.bash` is sourced [S]. This is relevant to rendering tests.

**Spawn.**
- The launch default is (0, 0, 0.5) and the wrapper default is (−2, 3, 2.0). Both are wrong for any new terrain (B12).
- The generator should export spawn sites with a recommended `robot_z`: the highest terrain point under the footprint, plus the offset from base_link to the wheel bottoms, plus about 5 cm. The footprint is about 0.86 x 0.68 m: chassis length 0.86 m, wheel track 0.562 m plus wheel width 0.114 m.

**Load time and controller-activation risk** [L], from `controller_activation_history.txt`:
- In 25 `lunar_surface.world` launches between 2026-09-28 and 10-04, **22 had one controller spawner exit with code 1**: `platform_velocity_controller` 14 times, `joint_state_broadcaster` 8 times, never both.
- The message is "Failed getting a result from calling /controller_manager/switch_controller in 10.0 ... Failed to activate controller".
- Flat and empty worlds: 0 failures in 7 launches.
- Gazebo start to the velocity spawner's exit: 20.0 to 27.5 s in the lunar world, against 15.1 to 20.8 s in flat and empty worlds.
- The root cause was not investigated. It is likely that the two switch requests run concurrently while the simulation steps slowly at startup [I].
- Whether the controller became active afterwards cannot be told from the logs; `LOCAL_CHANGES.md` reports successful drives in some sessions.
- Consequences for the new world:
  - The rover tests must check `ros2 control list_controllers` after launch.
  - Keep world load and startup real-time factor light. The 6 s spawn delay is hard-coded; `controller_spawn_delay` can be passed as an argument.
  - Measure the world's load time.

## 7. Recommendations for the new generator

1. Keep heights as **float32 in metres**, with georeference and vertical offset recorded in metadata. Never min-max rescale, and never store heights in 8-bit or 16-bit PNG.
2. Use the 2 m DTM as the base. Add detail only as **spectral extension** (fBm power-law noise in physical units, wavelengths below the DTM's resolution) plus explicit craters and rocks. Re-implement the **min-carve crater** idea with continuous profiles and lunar morphometry.
3. Derive the collision ground from the **same** float field: low-pass it, triangulate with an error bound, and report the visual-to-collision gap (RMS, 99th percentile, maximum). The legacy result, 10.7 mm RMS and 168 mm maximum, is the "do not repeat" baseline.
4. Generate rocks from a size-frequency distribution. Candidate source: `placed_rocks.xml`, which is site-registered (about 900 rocks ≥ 0.1 m in the central 100 x 100 m) and needs extrapolation below 0.1 m. Use **matching** visual and collision proxies.
5. Use GLB tiles on the WSL disk. Avoid Collada and `/mnt/c`.
6. Write a world with the plugin set and spherical coordinates from section 6, lunar gravity, and a distinct world name. Write a spawn table next to it.
7. Add tests for the issues seen here:
   - every crater and stamp profile is continuous;
   - histograms of depth/diameter and slope match targets;
   - visual-to-collision |Δz| stays within budget;
   - the axis convention is checked with the `placed_rocks.xml` sign test;
   - the world contains no Sensors, Imu or NavSat system.

## 8. Reproduction

All scripts and outputs are in `/home/nargon/togo_assets/lunar/experiments/legacy/` (Windows: `\\wsl.localhost\Ubuntu\home\nargon\togo_assets\lunar\experiments\legacy\`). Run them with `wsl.exe -d Ubuntu -e bash <script>`, setting `MSYS_NO_PATHCONV=1` when calling from Git Bash. All are read-only on the repo.

| Script | Output | What it does |
|---|---|---|
| `gdalinfo_all.sh` | `gdalinfo/*.txt` | `gdalinfo` for the 5 DEM rasters |
| `slipmap_stats.py` (system python, GDAL) | `slipmap_stats.txt`, `out/*.png` | Slip-map and crop statistics, slope correlation, previews |
| `legacy_analysis.py` (venv) | `legacy_analysis.txt` | §A 8-bit/16-bit encodings; §B `slipmap-2048.tif` blobs; §C rock catalogue, cross-match, z against DTM; §D meshes, slopes, visual-to-collision gap, PNG regeneration |
| `crater_dD.py` | `crater_dD.txt` | Depth/diameter of the 94 deterministic craters (seeds 52 and 62) |
| `crater_profile.py` | `crater_profile.txt` | Radial profiles showing the ring steps |
| `check_scripts.sh` | `check_scripts.txt`, `script_copies/` | AST check and `NameError` reproduction on **copies**; `.pyc` header comparison |
| `xacro_dump.sh` | `xacro_dump.txt`, `robot_sim.urdf` | Robot URDF expanded as `control.launch.py` does it; plugins, sensors, lights |
| `check_navsat_moon.sh` | `check_navsat_moon.txt`, `moon_sc_test.sdf` | `MOON_SCS` support, NavSat warning strings, sdformat parse |
| `misc_checks.sh` | `misc_checks.txt` | Band-pass shape, Pillow behaviour, spawn heights, `LOCAL` vs `LOCAL2`, rock densities near the LM |
| `controller_activation_history.sh` | `controller_activation_history.txt` | Controller-spawner outcome for each past launch, from `~/.ros/log/*/launch.log` |

Upstream sources read (branches `gz-physics7` and `gz-sim8`): `bullet/src/SDFFeatures.cc`, `bullet/src/ShapeFeatures.cc`, `dartsim/src/ShapeFeatures.cc`, `src/systems/navsat/NavSat.cc`, `src/Util.cc`.
