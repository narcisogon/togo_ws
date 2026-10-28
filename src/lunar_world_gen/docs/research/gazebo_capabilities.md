# Gazebo Harmonic capability check for the lunar world generator

Functional tests (not benchmarks) of the Gazebo features the Apollo 16 lunar world will rely on.
Run on 2026-10-25 by the `gazebo` research subagent. Every number below comes from a run that was
executed and inspected. Experiments directory (WSL):
`/home/nargon/togo_assets/lunar/experiments/gazebo/` (called `EXP` below), with `scripts/`, `worlds/`,
`meshes/`, `models/` and `results/<world>/`.

**Software:** gz-sim 8.11.0, gz-rendering 8.2.3 (ogre2), gz-sensors 8.2.2, gz-physics 7.6.0 (classic Bullet
plugin `gz-physics-bullet-plugin`), gz-common 5.8.0, sdformat 14.9.0 (SDF 1.11), assimp 5.3.1, Mesa 25.2.8,
d3d12 → RTX 2080.

**Run setup:** server only, `gz sim -s -r --headless-rendering -v 4`, with `GALLIUM_DRIVER=d3d12`,
`LD_LIBRARY_PATH=/usr/lib/wsl/lib`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `GZ_PARTITION=claude_gazebo`,
`ROS_DOMAIN_ID=62`. Each server was started with `setsid` and stopped by killing its process group.
Physics was the classic Bullet plugin with a 1 ms step and gravity 1.62 m/s², as in the lunar world. Every
world also loaded the UserCommands, SceneBroadcaster and Sensors (`ogre2`) systems. The LiDAR was the
real-sensor gpu_lidar: 210 × 122 samples, ±1.047 rad × ±0.61 rad (0.01 rad spacing), 10 Hz, no noise.
Every world with a LiDAR or camera recorded GPU evidence in `results/<world>/run.txt`: `libd3d12.so` and
`libdxcore.so` mapped into the live `gz` process (`maps pid=...` line), and the `ogre2.log` line
`GL_RENDERER = D3D12 (NVIDIA GeForce RTX 2080)`.
- Worlds without rendering sensors never start the render engine: `t4c`, `t4e`–`t4g`, `t4j`, `t4k` and
  `t8_*` (physics- or NavSat-only).
- Their `ogre2.log` copies (`t3_missing`, `t4c_friction`) were written by other agents' concurrent Gazebo
  runs, not by mine.

---

## 1. Rules the generator must follow (summary)

1. **Write GLB vertex data Z-up, in metres, exactly as authored. Do not convert to glTF's Y-up.**
   Gazebo applies no axis conversion to GLB, OBJ, STL or DAE. It also ignores DAE `<up_axis>`.
   - A spec-compliant Y-up GLB, which is Blender's default export, shows up lying on its side: authored +Z
     points along Gazebo +Y.
   - Blender: export glTF with `export_yup=False` (untick "+Y Up"). For OBJ use `forward_axis='Y', up_axis='Z'`.
   - trimesh's default GLB export is already correct (no conversion).
   - glTF node transforms are applied as-is in the Z-up frame (root rotation and translation, nested
     children) to both visuals and collisions.
2. **Always write an explicit glTF material with `metallicFactor: 0`.** A GLB without a material renders
   almost black in ogre2. So does `metallic = 1`. Measured mean RGB: 53 with no material, 46 with
   metallic = 1, 161 with metallic = 0.
3. **Use GLB.** For the same 2 M-triangle mesh it was the fastest format to load.
4. **One missing mesh file aborts the whole world.** `Parser configurations requested resolved uris ... could
   not be resolved` → `Failed to load a world` → server exits. The generator must check that every URI
   exists. Relative, absolute, `file://` and `model://` URIs all work.
5. **Collision meshes:**
   - GLB, DAE, OBJ and STL all work with classic Bullet.
   - `<submesh>` (and `<center>`) is IGNORED for collisions: the whole file is used. Give each rock
     collision its own file. Visuals do honour `<submesh>`.
   - Box and cylinder shapes rest **+10 mm above** a mesh surface (Bullet contact margin). Spheres rest
     exactly on it.
   - A body spawned inside that 10 mm band is violently ejected. **Spawn ≥ 15 mm above the collision mesh.**
   - Cylinders resting on a dense (5 cm) triangle mesh jitter by +9 to +28 mm. On a 50 cm mesh the
     jitter is about 1 mm, and on a 2-triangle quad it is 0.03 mm. This supports the planned smooth,
     coarse collision ground.
6. **Friction:**
   - Classic Bullet honours `<surface><friction><ode><mu>` per collision.
   - It combines the two surfaces as the **product** μ1·μ2. It ignores `<bullet><friction>`.
   - With no tags, μ defaults to 1.
7. **gpu_lidar sees only front faces.**
   - Back faces are culled even when glTF `doubleSided` or SDF `<double_sided>` is set. The camera
     behaves the same.
   - Meshes must be closed with outward counter-clockwise (CCW) winding.
   - An inside-out mesh is seen from behind (wrong range).
8. **Tile seams:**
   - Separate tile meshes with coincident boundary vertices leak no LiDAR rays.
   - 1–5 mm gaps leak about 0.12 % of ground returns (10–12 per scan), at specific texel/seam
     coincidences.
   - Collinear T-junctions leak 0.02 %. T-junctions whose extra vertices dip 3 mm below the neighbour's
     edge leak 0.2 %.
   - Share exact boundary vertices between tiles (or add skirts).
9. **gpu_lidar range is quantized** (the error equals an angular error with std ≈ 0.18 × the beam
   spacing):
   - Ground height noise at grazing incidence, for the real sensor 0.7 m above the ground: about 5 mm at
     2–4 m, 11 mm at 4–8 m, 24 mm at 8–16 m and 42 mm at 16–32 m. On a wall it reaches 3 to 19 mm,
     depending on incidence angle.
   - 1 cm geometric detail therefore stays above this noise only within about 4–5 m.
   - The error scales as 1/sample count: 2× oversampling halves it, 4× quarters it.
10. **Visibility flags work both ways.** A visual's `<visibility_flags>` against the lidar or camera
    `<visibility_mask>` can hide visuals from the LiDAR only, or from the camera only. Bits 30–31
    (0xC0000000) are reserved by ogre2.
11. **100 tiles** as 1 model with 100 visuals and collisions, or as 100 models: identical results and load
    times.
12. **`<surface_model>MOON_SCS</surface_model>` works** with the NavSat system. It uses the Moon ellipsoid
    (a ≈ 1738.1 km, b ≈ 1736.0 km). To match the DTM's sphere (R = 1737.4 km), use `CUSTOM_SURFACE` with
    both axes set to 1737400. That variant was verified too.

---

## 2. Results table

| # | Capability | Works? | Evidence (world → results) | Key numbers / caveats |
|---|---|---|---|---|
| 1a | GLB visual visible to gpu_lidar | **yes** | `t1a`…`t1e` | LiDAR hit bounding boxes match the authored box to within one beam spacing |
| 1b | GLB orientation | **no axis conversion** | `t1a`, `t1c` (+ camera images) | raw Z-up data shows correctly; Y-up GLB and Blender default lie on their side (authored +Z → Gazebo +Y, +Y → −Z) |
| 1c | glTF node transforms | **applied** | `t1a` rootrot / tworoots, `t1d` roottrans | a single root node rotated +90° about X shows y∈[−2,0], z∈[0,1] as expected; translations applied |
| 1d | scale | **yes** | `t1d` glb_raw_scaled | GLB = metres; SDF `<scale>0.5 0.5 0.5</scale>` → 0.25 × 0.5 × 1.0 m |
| 1e | multiple meshes / primitives / nested nodes in one GLB | **yes** | `t1d` glb_multi (+ image) | 2 primitives with 2 materials, child at +1.0 m, grandchild at +1.75 m: all present, colours correct |
| 1f | embedded PBR textures (baseColor, normal, metallicRoughness) | **yes, no log errors** | `t7_scene` image | only the harmless Ogre "vertices without bone assignments" notes in ogre2.log |
| 1g | smooth vertex normals | **yes** | `t7_scene` image | smooth vs flat (split vertices) clearly different; with no NORMAL attribute gz generates **smooth** normals |
| 1h | GLB without material | renders dark | `t1f_materials` | mean RGB 53 (no material), 46 (metallic 1), 161 (metallic 0); an OBJ without .mtl renders white (255) |
| 2a | OBJ / DAE / STL visuals | **yes** | `t1b`, `t1c` | all raw Z-up; DAE `<up_axis>Y_UP` ignored; Blender OBJ default (−Z forward, Y up) lies on its side |
| 2b | load time, 2.0 M tris (1001² grid, 100 m) | GLB fastest | `t2_big_*` | first LiDAR cloud (baseline 3.2 s): **GLB 5.8 / 6.7 s**, STL 8.9 / 8.8, OBJ 10.3 / 10.1, DAE 12.5 / 11.6 |
| 3a | relative URIs (`../`, `dir/`, `./dir/`) | **yes** | `t3_uri` | resolved relative to the SDF file that contains them (server cwd was /mnt/c/...) |
| 3b | absolute path, `file:///`, `model://` | **yes** | `t3_uri` | `model://` needs `GZ_SIM_RESOURCE_PATH` |
| 3c | `<include>` of a model directory (relative / absolute / model://) | **yes** | `t3_uri` | mesh URIs in the included model.sdf resolve relative to that file |
| 3d | missing mesh file | **world load fails** | `t3_missing` | the server shuts down 0.3 s after start; the rest of the world is not loaded |
| 4a | GLB / DAE / OBJ / STL as Bullet `<collision>` | **yes** | `t4a_formats` | sphere rests at surface + r (0.0 mm); box +10.0 mm; same values for all formats |
| 4b | 200 `<collision>` in one static link | **yes** | `t4b_convex`, `t4e_rockfield` | 100 boxes + 50 convex-hull GLB files: spheres rest at top + r (0.0 mm); physics creation < 0.1 s; RTF 1.0 |
| 4c | `<submesh>` in collision | **NO (ignored)** | `t4b_convex`, `t4h_submesh` | the collision is the union of all submeshes, uncentred; visuals honour submesh and center |
| 4d | `<mesh optimization="convex_hull">` / `"convex_decomposition"` | **yes** | `t4b_convex` (cups), `t4f`, `t4g` | hull: ball sits on the lid (z 0.480); decomposition (16 hulls): ball inside (0.185); plain: inside (0.180); a world with one optimised 60-triangle mesh loads about 1.7 s slower |
| 4e | node transforms and `<scale>` in collision | **yes** | `t4i_transforms` | rotated root node, nested nodes, uniform (2) and non-uniform (1 1 3) scale all exact |
| 4f | per-surface friction `<ode><mu>` | **yes (product)** | `t4c_friction` | 20° incline (tan 20° = 0.364): μ_eff = 0.040 (0.2·0.2), 0.250 (0.5·0.5), 0.300 (0.3·1.0), 0.000 (0·0); 0.8·0.8 and defaults stick |
| 4g | `<bullet><friction>` | **ignored** | `t4c_friction` | ode 0.1 + bullet 0.8 → μ_eff 0.010; ode 0.8 + bullet 0.1 → sticks; bullet 0.1 only → sticks (default μ 1) |
| 4h | wheel-like cylinder on a mesh | works, with caveats | `t4a`, `t4j`, `t4k` | +10 mm margin; jitter grows with triangle density; ejected (jumps up to 0.47 m, 5 of 6 tip over) if spawned 0.5 mm above the surface, i.e. inside the margin |
| 5a | back faces | **culled** (rays pass through) | `t5a_faces`, `t5a2_doublesided` | the glTF `doubleSided` flag and SDF `<double_sided>` have no effect (LiDAR and camera) |
| 5b | cracks between tiles | coincident edges: no leaks | `t5b_*` | per scan of ~8.4–8.5k ground returns: 0 (welded), 0 (separate, coincident), 10 / 10 / 12 (1 / 2 / 5 mm gap), 2 (T-junction), 18 (T-junction −3 mm), 0 (T-junction +3 mm) |
| 5c | small objects at 0.573° | probabilistic | `t5c_small` | mean returns per cube (ideal ray casting in brackets): 5 cm @ 2 m 10.0 (11.3), @ 4 m 2.3 (4.0), @ 8 m 0.67 (0.83); 2 cm @ 2 m 1.3 (1.0), @ 4 m 0.17 (0.5), @ 8 m 0.33 (0.33) |
| 5d | visual without collision seen; collision without visual not seen | **yes / yes** | `t5a_faces` | 732 hits vs 0 hits |
| 5e | visibility_flags vs visibility_mask | **yes, both ways** | `t5e_vis` (+ image) | LiDAR mask 1: sees flags {default, 1, 3}, not 2; camera mask 2: sees {default, 2, 3}, not 1 |
| 5f | range quantization | inherent | `t5f_*` | equivalent elevation error std: 1.77 mrad at 1× samples, 0.89 at 2×, 0.44 at 4× (beam spacing 10.08 mrad) |
| 6 | 100 tile visuals + collisions: 1 model vs 100 models | **both fine** | `t6a_onemodel`, `t6b_manymodels` | identical per-tile LiDAR hit maps (11006 hits, 90 tiles in view); spheres at 4-tile corners rest exactly (0.00 mm); first LiDAR cloud 4.0–4.5 s in both |
| 7 | sun with shadows, ambient 0, black background, SDF PBR maps, camera | **yes, no errors** | `t7_scene` image | albedo / normal / roughness / metalness maps, relative and absolute paths; directional shadows render headless |
| 8 | `<spherical_coordinates>` `MOON_SCS` | **yes** | `t8_moon`, `t8_custom`, `t8_earth` | 1 km north → Δlat 0.0330415°; 1 km east → Δlon 0.0333721°; altitude at 1 km = 0.288 m (Moon curvature; Earth 0.078 m) |

---

## 3. Details

### Test 1: GLB visuals (`scripts/gen_t1.py`, `gen_t1f.py`, `blender_export_t1.py`, `analyze_t1.py`)

The test object is an asymmetric box authored Z-up: x∈[0, 0.5], y∈[0, 1], z∈[0, 2] m. It was placed at
model pose (9, y_slot, 0) with slots 5 m apart. The LiDAR was at (0, 0, 1). The table gives the LiDAR hit
bounding box per cluster, relative to the model origin (`results/t1_summary.json`). The visible face is at
x = 0; hits fall short of the box extents by up to one beam spacing (about 0.1 m at 10 m).

| variant | y range | z range | reading |
|---|---|---|---|
| glb_raw (`meshlib.GLBBuilder`, Z-up data) | 0.14 … 0.99 | 0.02 … 1.98 | as authored ✔ |
| glb_trimesh (trimesh default export) | 0.14 … 0.99 | 0.02 … 1.98 | as authored ✔ |
| blender_glb_zup (`export_yup=False`) | 0.05 … 1.00 | 0.02 … 1.98 | as authored ✔ |
| glb_yup (spec Y-up = (x, z, −y)) | 0.05 … 2.00 | −0.95 … 0.01 | lying on side ✘ |
| blender_glb_yup (Blender default) | 0.13 … 2.03 | −0.96 … 0.01 | lying on side ✘ |
| glb_rootrot (single root node, +90° about X) | −2.00 … −0.05 | 0.02 … 0.95 | node rotation applied |
| glb_tworoots_rot (+ marker node at z = 3) | −2.03 … −0.14 | 0.02 … 0.95; marker 2.95 … 3.09 | node rotation and translation applied |
| glb_roottrans (root translation +0.5 z) | 0.01 … 0.98 | 0.56 … 2.52 | applied |
| glb_raw + `<scale>0.5 0.5 0.5` | 0.05 … 0.53 | 0.02 … 0.95 | scaled |
| obj_raw / stl_raw / dae_zup | ≈ 0 … 1 | ≈ 0 … 2 | as authored ✔ |
| dae_yup (Y-up data, `<up_axis>Y_UP`) | 0.01 … 1.99 | −0.99 … 0.03 | `up_axis` ignored ✘ |
| dae_yup_rawdata (Z-up data, `<up_axis>Y_UP`) | 0.14 … 0.99 | 0.02 … 1.98 | `up_axis` ignored |
| blender_obj_default (−Z forward, Y up) | 0.02 … 1.91 | −1.01 … 0.01 | lying on side ✘ |
| blender_obj_zup (Y forward, Z up) | 0.01 … 0.98 | 0.06 … 1.94 | as authored ✔ |
| glb_multi (1 mesh with 2 primitives, child, grandchild) | 3 clusters | 0.07–0.48 / 1.05–1.44 / 1.72–1.94 | all parts present, nesting applied |

Camera confirmation images (in `EXP/results/`):
- `t1c/image_0.png`: the four Blender variants.
- `t1d/image_0.png`: glb_multi with red, green and blue primitives in the right places.
- `t1f_materials/image_0.png`: material brightness.

Textures, normals and PBR: `t7_scene/image_0.png` shows:
- a GLB box with embedded checker baseColor + normal map + metallicRoughness;
- a GLB box with baseColor only;
- low-poly spheres with smooth, flat and missing normals.

The `-v 4` server log had no warnings or errors for these GLBs. The only warning in every camera world
is `Ogre2Camera::SetVisibilityMask: Mask bits c0000000 ... reserved`, caused by the default mask 0xFFFFFFFF.

### Test 2: formats and load time (`scripts/gen_t2.py`, `run_t2.sh`)

The mesh was a 1001 × 1001 vertex grid: 2,000,000 triangles over 100 m × 100 m, with positions and normals.
It was loaded as a visual only. The timing is the wall time from server start to the first LiDAR point
cloud. Each world ran twice (r1 / r2). Runs varied by up to about 1 s; other agents were using the same
GPU and CPU at the time.

| format | file size | first LiDAR cloud (s) r1 / r2 | with `GZ_MESH_FORCE_ASSIMP=true` |
|---|---|---|---|
| (baseline, LiDAR only) | – | 3.22 / 3.19 | – |
| **GLB** (assimp) | 56 MB | **5.75 / 6.69** | (always assimp) |
| STL (binary) | 100 MB | 8.92 / 8.75 | 16.90 / 18.62 (slower) |
| OBJ | 153 MB | 10.32 / 10.10 | 8.82 / 9.25 |
| DAE | 138 MB | 12.55 / 11.58 | 8.24 / 8.29 |
| GLB visual + same GLB as Bullet collision | 56 MB | 7.57 / 7.74 | rendering init delayed from about 3.0 s to 6.5 s by the physics mesh setup |

- All formats produced the identical LiDAR cloud (12183 hits).
- `GZ_MESH_FORCE_ASSIMP` must be `true`; the value `1` is silently ignored.
- Writing time with numpy: GLB 0.14 s, STL 0.6 s, OBJ 9.5 s, DAE 9.5 s.

### Test 3: mesh URIs (`scripts/gen_t3.py`, world `EXP/worlds/t3_uri.sdf`, models in `EXP/models/`)

Each slot used its own copy of the GLB, so the mesh cache could not mask a resolution failure. LiDAR hits
per slot:

| URI form | hits |
|---|---|
| `../meshes/t3/box_rel_parent.glb` | 56 |
| `t3_local/box_rel_sub.glb` | 64 |
| `./t3_local/box_rel_dot.glb` | 100 |
| absolute path | 113 |
| `file:///…` | 121 |
| `model://t3_model/meshes/box_model.glb` | 120 |
| `<include>` of `../models/t3_incmodel_rel` | 90 |
| `<include>` of an absolute path | 61 |
| `<include>` of `model://` | 40 |

- All bounding boxes were correct.
- The `model://` cases need `GZ_SIM_RESOURCE_PATH=EXP/models`.
- Missing file (`t3_missing.sdf`): `[Err] Error Code 14: ... uri [../meshes/t3/does_not_exist.glb] could not be
  resolved` → `Error Code 9: Failed to load a world` → `Shutting down gz-sim-server` at t = 0.3 s.

### Test 4: collisions with classic Bullet (`scripts/gen_t4*.py`, `analyze_t4.py`, `posetools.py`)

**t4a: resting heights** on 3 × 3 m height-field patches (5 cm grid, flat bottom offset from the patch centre),
after 6 s. Values are z − expected, in mm.

| collision source | sphere r 0.1 | box 0.2 | cylinder r 0.1651 × 0.1143 (axis horizontal), t > 3 s |
|---|---|---|---|
| GLB mesh | 0.0 | +10.0 | +9.4 … +29.8 (mean 15.2), creeping 77 mm/s |
| DAE mesh | 0.0 | +10.0 | about +9 … +20 |
| OBJ mesh | 0.0 | +10.0 | +8.8 … +20.3 (mean 10.7), creeping 47 mm/s |
| STL mesh | 0.0 | +10.0 | about +9 … +18 |
| SDF `<box>` reference | 0.0 | 0.0 | 0.0 … 0.7 (settled after about 4 s) |

The LiDAR saw the visual surfaces of the same patches at 0.301 / 0.398 / 0.501 / 0.598 m (expected
0.30–0.60; std 11–14 mm from range quantization). So visual and collision geometry coincide, apart from the
10 mm margin.

**t4j / t4k: wheel cylinder placed at rest** (5 kg, 10 s). The ground was GLB grids at 5 cm or 50 cm spacing,
a 2-triangle quad, or an SDF box. z-error is in mm after t > 2 s.

| ground | placed 0.5 mm above the surface (t4j) | placed 15 mm above the surface (t4k) |
|---|---|---|
| GLB 5 cm grid | ejected (jumps to +227 / +380); x-axis cylinder tips onto its flat face (−98) | +8.5 … +28.4, on its side |
| GLB 50 cm grid | ejected (+469 / +451); both tip onto their flat face (−98 / −95) | +9.8 … +10.9 |
| GLB 2-triangle quad | ejected (+468 / +430); tip over (−85 / −98) | +9.97 … +10.0 |
| SDF box | −0.05 … +0.13 | −0.05 … +0.13 |

A z-error of −98 mm means the cylinder lies on its flat face (half-width 57 mm) plus the 10 mm margin.
5 of the 6 cylinders spawned inside the margin tipped over. Cylinders roll slowly on every ground (3–55 cm in
10 s, neutral equilibrium). Spheres stay exactly at r (0.00 mm).

**Implications for the rover:**
- Wheel collision cylinders sit about 1 cm above a mesh ground. One option is to lower the collision ground
  by 10 mm relative to the visual ground.
- Spawn the rover with ≥ 15 mm clearance above the collision mesh.
- Prefer a coarse (smoothed) collision mesh to limit jitter.

**t4b: 200 collisions in one static link.**
- 100 `<box>` collisions and 50 separate convex-hull GLB files: every dropped sphere rests at top + r
  (error 0.0 mm; 6 + 6 drops).
- 50 hulls taken as `<submesh>` from one GLB: every sphere rests at 0.3974 m. That equals the tallest
  hull in the library (0.3484 m) + r, so the whole file is used.
- t4h settles it with a layout that separates the parts in space:
  - The visual of `sel_A` shows only partA, and `<center>true</center>` centres partB.
  - In physics, a ball over the hidden partB rests on it (z 1.100).
  - A ball over the centred partB (where only the centred copy would be) falls through to the catch
    ground (−0.900).
- Load time: the 200-collision link adds < 0.1 s (`t4e_rockfield`: first pose 2.21 s vs world start
  2.14 s).
- A world with one `optimization=` mesh (60 triangles) needed about 1.7 s more (`t4f_decomp` 3.87 s,
  `t4g_hullcup` 3.82 s). `t4b_convex`, with both cups, published its first pose 4.7 s after world start.

**t4c: friction on a 20° incline** (box on box, displacement after 1 s and 2 s, fitted μ_eff):

| incline / box setting | s(2 s) m | μ_eff | reading |
|---|---|---|---|
| ode μ 0.2 / 0.2 | 0.944 | 0.040 | product 0.04 |
| ode μ 0.5 / 0.5 | 0.331 | 0.250 | product (min or mean would stick) |
| ode μ 0.3 / 1.0 | 0.187 | 0.300 | product |
| ode μ 0 / 0 | 1.057 | 0.000 | frictionless |
| ode μ 0.8 / 0.8 | 0.0007 | sticks | 0.64 > tan 20° |
| no friction tags | 0.0007 | sticks | default μ = 1 |
| ode 0.8 + bullet 0.1 (both) | 0.0007 | sticks | `<bullet>` ignored |
| ode 0.1 + bullet 0.8 (both) | 1.030 | 0.010 | `<ode>` used |
| bullet 0.1 only | 0.0007 | sticks | `<bullet>` ignored |

### Test 5: gpu_lidar vs geometry (`scripts/gen_t5.py`, `gen_t5a2.py`, `gen_t5f.py`, `analyze_t5.py`, `analyze_t5f.py`)

**(a) Back faces** (`t5a_faces`, backstop wall at x = 12):

| object | hits | rays in its angular window reaching the backstop |
|---|---|---|
| front-facing GLB quad | 286 | 0 |
| back-facing quad | 0 | 476 (all) |
| back-facing quad, glTF doubleSided | 0 | 756 (all) |
| inside-out box | 562 | – |

The inside-out box hits are all on its far inner faces, x 6.16–7.0 instead of 6.0. In `t5a2_doublesided`,
SDF `<material><double_sided>true` on the back-facing quad gave 0 LiDAR hits and was invisible to the
camera; a front-facing quad gave 938 hits. Images: `t5a_faces/image_0.png`, `t5a2_doublesided/image_0.png`.

**(b) Tile cracks** (`t5b_<variant>`).
- Setup: ground strips with 12 seam lines at x = 2.0 … 14.9 m, a catch plane 0.3 m below, and the LiDAR
  at 0.7 m height.
- Leaked returns (points on the catch plane under the ground footprint), single scan:

  | variant | leaks | ground returns |
  |---|---|---|
  | single welded mesh | 0 | 8462 |
  | separate meshes, coincident vertices | 0 | 8462 |
  | 1 mm gap | 10 | 8452 |
  | 2 mm gap | 10 | 8444 |
  | 5 mm gap | 12 | 8446 |
  | T-junction, collinear | 2 | 8462 |
  | T-junction, extra vertices 3 mm lower on the near side | 18 | 8406 |
  | T-junction, extra vertices 3 mm higher | 0 | 8354 |

- A second cloud gave identical counts (static scene, deterministic).
- Leaks cluster at a few seams, seen at large azimuth (|az| > 50°) and range 5–6 m. The same beams leak for
  1 mm and 2 mm gaps, so leaks follow texel / seam coincidences rather than gap width.

**(c) Small cubes** (`t5c_small`, 6 instances per case with jittered angles; results in the table above).
Detection rate:
- 5 cm: 6/6 at 2 m, 6/6 at 4 m, 4/6 at 8 m.
- 2 cm: 5/6 at 2 m, 1/6 at 4 m, 2/6 at 8 m.

The counts agree roughly with ideal ray casting through the nominal beam grid.

**(d) Visual-only vs collision-only:** the visual-only box gave 732 hits; the collision-only box gave 0.

**(e) Visibility** (`t5e_vis`, image `t5e_vis/image_0.png`): LiDAR `<visibility_mask>1` and camera
`<visibility_mask>2`, against visuals with `<visibility_flags>` default / 1 / 2 / 3.

| visual flags | LiDAR hits (mask 1) | camera (mask 2) |
|---|---|---|
| default | 658 | visible |
| 1 | 866 | hidden |
| 2 | 0 | visible |
| 3 | 658 | visible |

**(f) Range quantization** (`t5f_ground_x1|x2|x4`, `t5f_wall`; `results/t5f_summary.json`).
- Returned points lie exactly on the nominal beam directions (deviation < 3 µrad). The range comes from the
  rendered depth texture.
- Ground under the sensor at 0.7 m, real FOV:

  | samples | z std 0–2 m | 2–4 m | 4–8 m | 8–16 m | 16–32 m | range std 8–16 m |
  |---|---|---|---|---|---|---|
  | 210 × 122 (real) | 2.3 mm | 5.3 mm | 10.7 mm | 23.9 mm | 42.3 mm | 0.44 m |
  | 420 × 244 | 1.1 | 2.6 | 5.6 | 10.9 | 24.0 | 0.19 m |
  | 840 × 488 | 0.6 | 1.3 | 2.8 | 5.6 | 11.0 | 0.10 m |

- Wall at 10 m, real sampling: error std 2.7 mm at 0–10° incidence, 9 mm at 20–30°, 19 mm at 45–70°;
  maximum 56 mm.
- No noise was configured. This is deterministic angular quantization: the equivalent elevation error has
  std 1.77 mrad, about 0.18 × the beam spacing.

### Test 6: many entities (`scripts/gen_t6.py`)

- Layout: 10 × 10 tiles of 2 m, each a separate GLB with 41 × 41 vertices (3200 triangles), exact shared
  boundary vertices.
- Results were identical for one model (one link, 100 visuals + 100 collisions) and for 100 models:
  - LiDAR: 11006 ground hits with identical per-tile hit maps.
  - Physics: spheres placed on 4 tile corners (where 4 meshes meet) rest at exactly −0.0500 m
    (error 0.00 mm, xy drift < 0.5 mm).
  - First LiDAR cloud: 4.51 / 3.98 s (one model) vs 4.05 / 4.38 s (100 models).
  - RTF 1.00 in both.
  - No warnings.

### Test 7: scene features (`scripts/gen_t7.py`, image `EXP/results/t7_scene/image_0.png`)

- Scene: `<ambient>0 0 0 1</ambient>`, `<background>0 0 0 1</background>`, `<shadows>true</shadows>`, and one
  directional sun with `cast_shadows` true (direction 0.75 −0.45 −0.35).
- Rendering was correct, with crisp object shadows on a ground plane carrying an SDF PBR material:
  `<pbr><metal>` with `albedo_map`, `normal_map`, `roughness_map` and relative URIs.
- Boxes with SDF PBR maps (relative and absolute paths, including `metalness_map`) and the GLB embedded
  textures also rendered.
- No errors or warnings in the server log apart from the camera mask notice.
- The camera sensor (`R8G8B8`, 1280 × 720) published its first image 4.2 s after server start (3.8 s in
  the smaller `t0_smoke` world).

### Test 8: spherical coordinates (`scripts/gen_t8.py`)

The origin is the Apollo 16 site (−8.9734°, 15.5011°), ENU. Static NavSat sensors were placed at local
(1000, 0, 0), (0, 1000, 0) and (0, 0, 100). The world loaded the `gz-sim-navsat-system` plugin.

| surface_model | Δlat for 1 km north | Δlon for 1 km east | altitude at 1 km | implied radii |
|---|---|---|---|---|
| MOON_SCS | 0.033041501° | 0.033372078° | 0.288 m | M = 1734057 m, N = 1738144 m → ellipsoid a ≈ 1738.1 km, b ≈ 1736.0 km |
| CUSTOM_SURFACE 1737400 / 1737400 | 0.032977883° | 0.033386505° | 0.288 m | sphere 1737402 m ✔ |
| EARTH_WGS84 | 0.009041488° | 0.009093721° | 0.079 m | WGS84 ✔ |

- No parser or system warnings.
- The up-100 m sensor reports altitude 100.000.
- NavSat altitude decreases with horizontal distance as d²/2R: the flat world is a tangent plane.

---

## 4. Reproduction

- Run everything with `bash EXP/scripts/run_all.sh` inside WSL (about 15 min). The Blender exports need
  `scripts/blender_export_t1.py` run from Windows Blender first (copy it to a local Windows folder: Blender
  treats `\\wsl.localhost\...` paths as blend-relative `//`).
- Single world: `bash EXP/scripts/run_capture.sh <world> --lidar /test/lidar/points --lidar-n 2 [--image /test/camera] [--poses <world> --settle 6]`.
- `scripts/run_gz.sh` contains the GPU recipe, the partition and domain isolation, `setsid` with
  process-group stop, and the renderer evidence.
- Renderer evidence: the GL_RENDERER line from a copy of `~/.gz/rendering/ogre2.log`, plus d3d12 libraries
  in `/proc/<pid>/maps`.
  - **Use the maps check.** `ogre2.log` is shared by all users of `$HOME`. Twice in this session (during
    `t3_missing` and `t4c_friction`, neither of which initialised rendering) another agent's Gazebo
    rewrote it, and my watcher copied the foreign log.
- Outputs per world in `EXP/results/<world>/`: `server.log` (timestamped, seconds since start), `run.txt`,
  `ogre2.log`, `capture.json` (first-message times), `lidar_*.npz` (x, y, z, intensity, ring as 122 × 210
  arrays in the sensor frame), `image_*.png`, `poses.json`, `stats.json`, `navsat.json`. Summaries:
  `EXP/results/t1_summary.json`, `t4_summary_*.json`, `t5_summary_*.json`, `t5f_summary.json`.
- gpu_lidar message facts:
  - Topic `<topic>/points` carries PointCloudPacked; `<topic>` itself carries LaserScan.
  - Fields x, y, z, intensity (float32, always 0 here) and ring (uint16, 0–121); no return is +inf.
  - frame_id = `<model>::<link>::<sensor>`; columns go from −60° to +60° azimuth, rows from −35° to +35°.

## 5. Not tested / open points

- Not tested:
  - Draco- or meshopt-compressed GLB, `.gltf` with an external `.bin`, KTX2 or JPEG textures, DAE `<unit>`
    scaling.
  - DART and TPE physics.
  - Bullet contact parameters (`kp`, `kd`, `min_depth`) and the `mu2`/`fdir1` anisotropy (mu2 = mu in every
    friction test).
  - Driven wheels on meshes (only resting and dropped bodies were tested). The rover team should check
    skid-steer behaviour on a mesh ground with the 10 mm margin.
- Load times were measured with other agents' Gazebo runs sharing the GPU and CPU. Treat them as relative
  only; performance and RTF with the full world belong to the perf task.
- The crack leak counts come from one sensor pose per variant (a static scene is deterministic). The exact
  count will differ at other poses; the ~0.1 % rate for mm gaps is an estimate from this one pose.
