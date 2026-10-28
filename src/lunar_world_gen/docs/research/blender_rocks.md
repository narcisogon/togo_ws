# Procedural lunar rock library: headless Blender prototype

Status: prototype working end to end and verified, 2026-10-25. Script: `blender/rock_proto.py`. Checker: `tools/verify_rock_glb.py`.
Outputs live under `/home/nargon/togo_assets/lunar/experiments/blender/` (from Windows: `//wsl.localhost/Ubuntu/...`).

## TL;DR

* One Blender run (5.2.1 LTS, headless, Cycles on the RTX 2080 via OptiX) builds a rock library from a seed. It produces:
  * a seeded high-poly shape per rock;
  * a game LOD per class;
  * per-rock UVs and a baked normal map, albedo and roughness from a procedural Apollo-16 breccia material;
  * a shared atlas (power-of-two regions sized by texel density);
  * standalone GLBs, one library GLB (one material, atlas embedded once) and convex collision hulls (≤ 64 vertices);
  * a Z-up NPZ per rock, plus per-rock and library metadata JSON;
  * Cycles previews.
* Measured on the 50-prototype production-size run (10 large, 20 medium, 20 pebbles, 4096² atlas, 53.2 % used):
  * 235 s in total;
  * per rock: large 13.7 s, medium 3.4 s, pebble 1.2 s;
  * peak extra GPU memory about 1.5 GiB;
  * `rocks_library.glb` = 20.0 MB.
  * The 9-rock test set takes 47 s.
* Output is deterministic per `--seed`: two separate runs gave identical LOD vertices and triangle counts (checked on 3 rocks).
* Axis convention: the glTF exporter writes **(x, y, z)_Blender → (x, z, −y)_glTF** (+Y up, as the spec requires); node transforms are identity.
  * This was verified numerically in two ways: npz versus GLB vertices, and `axis_marker.glb` bounds.
  * To get back to Z-up, rotate by +90° about X. If Gazebo does not convert Y-up itself, the SDF pose needs `roll = +π/2`. Another agent is testing this; `axis_marker.glb` is provided for it.
* Every GLB loads in trimesh 5.2 at metric scale. Bounding boxes match the metadata to within 5e-5 m (that is just the 4-decimal rounding of the JSON).
* Every LOD is closed after merging seam-split vertices. Every hull is convex and watertight, with ≤ 64 vertices.
* The Apollo-16 look is plausible: blocky to subangular fracture blocks, light-grey polymict breccia with angular white and dark clasts, dark glass coatings and veinlets, dust. See `docs/research/img/*.jpg` and the `preview/` folders.

## 1. Appearance reference (brief research)

* **Rock types.** Apollo 16 (Cayley plains, North Ray crater) rocks are mostly feldspathic (anorthositic) breccias.
  * Typical are light-grey friable matrix breccias with abundant white (plagioclase) clasts and dark clasts of impact melt.
  * The "dark-matrix" breccias (e.g. 67435) are medium grey, "not all that dark".
  * Some rocks are partially coated by dark glass and veined by dark glass (e.g. 67115), cut by penetrating fractures (Lunar Sample Compendium, 67016/67115/67435/66075).
* **Shape.** Impact fragments cluster around mean axial ratios a:b:c = 2 : √2 : 1, i.e. b/a ≈ 0.71 and c/a ≈ 0.5 (Fujiwara-type laboratory fragments; also Itokawa particles and Ryugu blocks < 1 m).
* **Erosion.**
  * Exposed tops are rounded by micrometeorite erosion and zap-pitted, while protected undersides stay flat or angular.
  * A sharp line separates the cratered surface from the buried part.
  * Old boulders sit in regolith fillets; freshly emplaced ones have none (69935).
* **Albedo.** Normal albedo at 0.55 µm is about 12-18 % for highlands and 6-10 % for maria.
  * I used regolith 0.14 linear.
  * Rock matrix: 0.19-0.27 (light breccia), 0.10-0.14 (dark matrix), 0.33-0.43 (anorthositic).
  * Clasts 0.05-0.55; dark glass 0.035; dust 0.15.
  * The resulting mean rock albedo is 0.15-0.36, mean 0.22.
  * These are plausible values, not photometrically calibrated, and use no Hapke/opposition model (Gazebo cannot render one anyway).

## 2. Method (what `rock_proto.py` does)

Units are metres. Modelling uses the Blender frame (right-handed, Z up).

**Classes** (`CLASS_CFG`):

| class | size a | LOD budget | high-poly | atlas region | hull |
|---|---|---|---|---|---|
| large | 0.3-3 m (log-stratified) | 2000-5000 tris, grows with size | ~0.4-1.0 M tris | 256-1024 px, target 3 mm/texel | ≤ 64 vertices |
| medium | 5-30 cm | 200-1000 tris | ~40-140 k tris | 64-512 px, target 1 mm/texel | ≤ 64 vertices |
| pebble | 2-5 cm | 16-36 tris (random) | ~5-19 k tris | 32-64 px, target 0.5 mm/texel | none (the LOD is itself convex) |

**Shape** (`build_high_poly`). Each rock gets its own seed, derived from (library seed, class, index).
1. Base: an icosphere is mapped to a **superellipsoid** (exponent 2.0-2.8). Its axes are sampled around 2 : √2 : 1 (b/a ~ N(0.72, 0.09), c/a ~ N(0.52, 0.08)), with low-frequency "lumps" of 1.5-4 % (1σ).
2. **Exact planar fracture cuts** (`bmesh.ops.bisect_plane` plus hole fill):
   * one flattened base (normal ≈ −Z, cutting 6-16 % of the height);
   * 6-12 big fractures (10-38 % of the extent along the normal; top-facing cuts × 0.4, so tops stay rounded);
   * 2-24 small edge chips.
3. **Affine rescale** back to the sampled a, b, c. Planes stay planes; without this, the cuts produce cube-like rocks.
4. **Voxel remesh** at a/n_vox, with n_vox = 120 → 60 (large) from fresh to rounded. This gives uniform topology and edges rounded at voxel scale.
5. **Height-weighted Taubin smoothing**: rounded, eroded top; angular, buried base.
6. **Catmull-Clark** subdivision, to about the class high-poly budget.
7. **Numpy displacement** along the normals: seeded Perlin fBm (σ = 0.11-0.22 % of a, starting at a/6-a/10 wavelength, so facets stay planar).
8. Optional **cellular fracture cracks**: Worley F2−F1, mildly domain-warped, gated, V-profile, half-width ≈ 0.4 % of a.
9. Optional **pits** on the exposed top.
10. Origin is moved to the **volume centroid**, then a uniform rescale makes the longest extent exactly a.

**LOD** (`make_lod`):
* Medium and large: quadric `DECIMATE/COLLAPSE` to the triangle budget, then triangulate, then sharp edges at 40° (so the low-poly normals follow the facets and the normal map only carries small deviations).
* Pebbles: a **greedy convex hull** of the high-poly with k = (tris+4)/2 vertices, giving 2k−4 triangles.
* The LOD error (signed distance of high-poly vertices to the LOD, via BVH) is measured and also sets the bake cage and ray distances.

**UV / atlas.**
* `uv.smart_project` (66°, FRACTION margin) only.
  * An extra `uv.pack_islands` (CONVEX/CONCAVE) took **7-140 s per rock** in Blender 5.2 and did not improve coverage.
* Region per rock = the smallest power of two that reaches the class texel target; it is doubled if the measured texel exceeds 1.25× the target.
* Island gap = R/128 px, clamped to 2-8 px. Bake margin = gap ("EXTEND").
* Every rock is baked into its **own** images. The atlas layout (quadtree / buddy allocation of power-of-two squares) is decided afterwards, and pixels are copied in numpy. That way no bake margin can bleed into a neighbour region.

**Bake** (`build_breccia_material`, `bake_maps`): Cycles GPU (OptiX), 8 AA samples, selected-to-active (high → low).
* The bake source is a procedural breccia: domain-warped Voronoi clast layers at 4 / 15 / 55 / 180 mm (layers finer than 3 texels are skipped).
  * A random subset of cells becomes clasts, shrunk away from the cell edges so matrix shows between them, with a lithology palette, brightness jitter and internal grain.
  * Optional dark-glass veinlets: thin noise-ridge lines, region-gated.
  * Optional glass coating: noise threshold biased to one side, roughness 0.32.
  * Dust: more on up-facing surfaces, patchy.
  * Micro-relief through a Bump node, so it ends up in the normal map.
* Maps:
  * NORMAL in tangent space, OpenGL/glTF convention.
  * DIFFUSE with pass_filter COLOR. Specular is set to 0 on the source material because Principled v2 otherwise scales the diffuse colour by ≈ 0.97.
  * ROUGHNESS.
  * ORM = (R 1, G roughness, B metallic 0), which becomes the glTF `metallicRoughnessTexture`. No occlusion texture, because Assimp maps glTF occlusion to a light map.

**Hull** (`greedy_hull`): exact hull of the LOD vertices, then greedy farthest-point selection to at most 64 vertices (124 triangles). A reflex-edge flip pass makes the triangulation strictly convex, because bmesh's hull left ~2e-5 m reflexes on near-coplanar faces.

**Export.** All exports use `export_yup=True`, apply modifiers, include normals, write MikkTSpace tangents, use PNG images, no Draco/meshopt, and backface culling (`doubleSided=false`).
* `rocks/<id>.glb` (own textures, UV 0..1);
* `rocks_library.glb` (all LODs at the origin, one material `lunar_rock_atlas`, three atlas images embedded once);
* `hulls/<id>_hull.glb` and `rocks_hulls.glb` (no material, no normals, so vertices are shared);
* `npz/<id>.npz` (Z-up `positions`, `normals`, `uv_atlas_gltf`, `indices`);
* `meta/<id>.json` and `library.json`;
* `axis_marker.glb` (+X 0.30 m red, +Y 0.20 m green, +Z 0.10 m blue);
* `normalmap_test.glb` (real relief next to a flat tile carrying its baked normal map).

**Preview** (`--preview`): re-imports `rocks_library.glb` (round-trip check: dimensions are identical), on a regolith plane with a black world (no ambient light) and a sun at 12° elevation (0.53° disc). Renders: overview, close-up, large-rock detail, a scatter field of up to 1070 instances seen from rover height, the normal-map test, and clay versions. Each render takes 2-3.5 s on the GPU.

## 3. Conventions and interfaces for the world generator

* **Rock frame:** Z up, metres, origin at the volume centroid. The rock rests on its flattened base, smallest axis vertical.
  * `bottom_offset_m` is the distance from the origin down to the lowest LOD vertex; it is about 0.52 × height (range 0.43-0.61).
  * Placement: `z_origin = ground(x, y) + s * (bottom_offset_m − f * dims_m[2])`, with burial fraction f from `burial_fraction_range`: large 0.10-0.35, medium 0.10-0.40, pebble 0.20-0.50.
  * Add random yaw and a scale s of roughly 0.7-1.3 within the class.
  * Regolith fillets belong in the terrain height field: the rock bottoms are flat and angular, as on the Moon.
* **glTF files are +Y up:** p_gltf = (x, z, −y) of p_zup.
  * `npz` positions and `meta.hull.verts_zup` are already Z-up.
  * `npz` UVs are in glTF convention (origin top-left of the PNG).
  * `meta.atlas.region_px_xywh_topleft` and `uv_rect_gltf` give each rock's atlas rectangle.
* **Library GLB:** one mesh per rock, named exactly like the rock id (also the node name), with identity node transforms. In SDF this is usable as `<mesh><uri>rocks_library.glb</uri><submesh><name>rock_large_03</name><center>false</center></submesh></mesh>` (not tested in Gazebo).
* **Per-rock metadata fields:**
  * `cls`, `lithology`, `seed`, `roundness`;
  * `dims_m`, `bbox_min_m`/`bbox_max_m`, `bottom_offset_m`, `volume_m3`, `surface_area_m2`;
  * `high_poly{hi_tris, …}`;
  * `lod{lod_tris, gltf_vertices, error{rms_mm, max_mm, max_out_mm, max_in_mm}}`;
  * `texture{region_px, texel_mm, uv_coverage, …}`;
  * `hull{vertices, triangles, volume_ratio_vs_lod, lod_max_outside_mm, verts_zup, faces, file}`;
  * `atlas{…}`, `files{…}`, `timing_s{…}`.

## 4. Results

### 4.1 Test set (`experiments/blender/test_set`, seed 1, 2048² atlas)

| id | class | lithology | dims x*y*z [m] | high tris | LOD tris | LOD err rms / max [mm] | region [px] | texel [mm] | hull v/t | hull protrusion [mm] | GLB [kB] | time total / bake [s] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rock_large_00 | large | anorthositic | 0.528 x 0.396 x 0.222 | 596800 | 2736 | 0.16 / 1.57 | 512 | 1.87 | 64/124 | 3.98 | 552.4 | 12.4 / 1.4 |
| rock_large_01 | large | light_breccia | 1.814 x 1.246 x 0.832 | 821120 | 4344 | 0.39 / 3.93 | 1024 | 2.96 | 64/124 | 11.43 | 2290.9 | 17.0 / 2.5 |
| rock_medium_00 | medium | light_breccia | 0.059 x 0.045 x 0.021 | 107168 | 270 | 0.10 / 0.60 | 128 | 0.94 | 64/124 | 0.24 | 52.2 | 3.3 / 0.9 |
| rock_medium_01 | medium | dark_matrix_breccia | 0.110 x 0.080 x 0.059 | 105552 | 550 | 0.10 / 0.65 | 256 | 0.81 | 64/124 | 0.41 | 162.9 | 3.1 / 1.0 |
| rock_medium_02 | medium | glass_coated_breccia | 0.248 x 0.191 x 0.131 | 84064 | 914 | 0.16 / 1.16 | 512 | 0.86 | 64/124 | 1.84 | 513.8 | 3.4 / 1.3 |
| rock_pebble_00 | pebble | dark_matrix_breccia | 0.022 x 0.013 x 0.012 | 5844 | 18 | 0.83 / 2.20 | 64 | 0.78 | - | - | 11.6 | 1.2 / 0.8 |
| rock_pebble_01 | pebble | glass_coated_breccia | 0.026 x 0.017 x 0.013 | 8388 | 18 | 1.16 / 3.25 | 64 | 0.87 | - | - | 14.8 | 1.2 / 0.8 |
| rock_pebble_02 | pebble | light_breccia | 0.034 x 0.020 x 0.018 | 18960 | 26 | 0.64 / 1.53 | 64 | 0.99 | - | - | 15.7 | 1.4 / 0.9 |
| rock_pebble_03 | pebble | light_breccia | 0.042 x 0.024 x 0.016 | 5588 | 18 | 0.92 / 3.16 | 64 | 1.27 | - | - | 14.1 | 1.2 / 0.8 |

* Total time is 46.6 s for the 9 rocks; atlas assembly takes 0.9 s and the library export 1.2 s.
* Sizes: `rocks_library.glb` 3.89 MB. Atlas PNGs: basecolor 1.62 MB, normal 1.83 MB, ORM 0.10 MB. Hull GLBs are 2.3 kB each.
* The atlas is 39.8 % allocated.

### 4.2 Production-size run (`experiments/blender/prod50`, seed 2, 10 / 20 / 20 rocks, 4096² atlas)

Values are mean [min..max].

| | large (n=10) | medium (n=20) | pebble (n=20) |
|---|---|---|---|
| size a [m] | 1.18 [0.32..2.90] | 0.139 [0.052..0.280] | 0.033 [0.020..0.048] |
| b/a, c/a | 0.70, 0.52 | 0.70, 0.50 | 0.74, 0.52 |
| high-poly tris | 708 k [380 k..1.03 M] | 99 k [37 k..143 k] | 6.9 k |
| LOD tris (glTF verts) | 3489 [2100..4954] (2250) | 598 [214..968] (447) | 26 [16..34] (47) |
| LOD error rms / max | 0.37 / 2.87 mm (max ≤ 5.55 mm = 0.19-0.40 % of a) | 0.12 / 0.80 mm (0.41-0.83 % of a) | 0.80 / 2.24 mm (3-13 % of a, inherent to ≤ 36 tris) |
| region / texel | 256-1024 px / 2.85 mm [1.87..5.25] | 128-512 px / 0.80 mm [0.50..1.06] | 64 px / 1.09 mm [0.61..1.61] |
| UV coverage | 0.49 | 0.45 | 0.36 |
| time per rock: total | 13.7 s [7.3..19.9] | 3.35 s | 1.19 s |
| of which: high-poly / decimate / bake | 3.1 / 7.4 / 1.8 s | 1.0 / 0.7 / 1.0 s | 0.07 / 0.02 / 0.85 s |
| standalone GLB | 1.33 MB [0.24..2.39] | 273 kB | 15 kB |
| hull | 64 v / 124 t; V_hull / V_LOD 1.004; LOD protrudes ≤ 2.5-19.5 mm (0.45-0.86 % of a) | 64 / 124; ratio 1.015; ≤ 1.8 mm | – |

* Run totals:
  * 235.4 s for 50 rocks; with all previews, 255.4 s wall time;
  * atlas 3.7 s, library export 2.7 s.
* `rocks_library.glb` = 19.97 MB. Atlas PNGs: basecolor 8.20 MB, normal 9.35 MB, ORM 0.51 MB.
* The 4096² atlas is 53.2 % allocated.
* GPU memory: baseline 3,232 MiB, peak 4,711 MiB, so +1.48 GiB (nvidia-smi at 0.5 s sampling).

### 4.3 Verification (`tools/verify_rock_glb.py`, WSL venv, trimesh 5.2.0): PASS for `test_set` and `prod50`

* Every `rocks/<id>.glb` loads with an identity node transform.
* Bounds in the **file frame** equal the Z-up metadata bbox mapped by (x, z, −y), max error 4.9e-5 m (the JSON rounding). This verifies both metric scale and the axis convention.
  * Example: `rock_large_01` file extents are [1.814, 0.832, 1.246] for metadata dims (1.814, 1.246, 0.832).
* Triangle counts match the metadata. Each mesh is closed after merging seam-split vertices.
* npz positions map onto the GLB positions exactly.
* Library: 1 material, 3 images, `doubleSided=false`, extents identical to the standalone files.
* Hulls: ≤ 64 unique vertices, convex, watertight; the worst LOD protrusion is ≤ 1 % of a.
* `axis_marker.glb` bounds are min (−0.02, −0.02, −0.20) and max (0.30, 0.10, 0.02). So Blender +X → glTF +X, Blender +Y → glTF −Z, Blender +Z → glTF +Y.

### 4.4 Visual iteration (what changed and why)

Shape iterations (`shapes_v2..v4`, clay renders):
* Ellipsoid plus heavy fBm looked like a "potato with dents", made worse by the low sun.
* Lowering the fBm amplitude and moving it to higher frequency kept the facets planar.
* Superellipsoid, more and deeper cuts, and much smaller lumps removed the scoops and the blobby pebbles.
* The affine rescale stopped the cuts from producing cubes.
* Worley cracks: the first version looked like "tape seams"; they were then warped, gated and narrowed.

Material:
* A Voronoi-edge vein network tiled the whole rock like crazy paving. It was replaced by sparse noise-ridge veinlets.
* Clasts were domain-warped, given per-clast brightness jitter, and their contrast lowered.

Final renders:
* `test_set/preview/`: overview, close-up, large_detail, scatter, normalmap_test, clay.
* `prod50/preview/`.
* Repo copies in `docs/research/img/`.

## 5. Problems found (with fixes) and open issues

1. **`uv.pack_islands` is extremely slow** in 5.2 for small meshes: 7-140 s per rock (CONVEX/CONCAVE) against 0.01 s for smart_project alone, with no coverage gain. Removed.
   * Small rocks still get low coverage, because islands plus margins dominate: pebbles 0.27-0.44, so the 64 px regions effectively give 0.6-1.6 mm texels.
2. **Bake time grew with library size**: a pebble bake took 2.0 s and then 6.7 s after 49 rocks, because per-rock images and materials accumulated in the session.
   * Fix: free them once their pixels are copied to numpy and the standalone GLB is written.
   * With the fix, a 30-pebble run stays at 0.8-0.9 s per bake; the 50-rock run dropped from 439 s to 235 s.
   * With the cleanup in place, global undo on or off made no measurable difference: the last bakes took 0.83 s with undo and 0.94 s without. Undo is disabled by default anyway; `--keep-undo` turns it back on for diagnosis.
   * Hiding finished objects from rendering did not help on its own.
3. **The DIFFUSE/COLOR bake is not the albedo** with Principled v2: it was scaled by ≈ 0.97, because the specular layer takes energy from the diffuse. Fixed with `Specular IOR Level = 0` on the bake-source material.
4. **`Image.generated_color` is written raw** into byte sRGB images (no linear→sRGB conversion), so fills must be given sRGB-encoded.
5. **Blender treats a path that starts with `//` as blend-relative.** The script normalizes `--out` with `os.path.normpath`, which turns `//wsl.localhost/...` into a backslash UNC path.
   * Writing to the WSL disk over UNC works and is fast enough here.
   * Calling `wsl.exe` from Git Bash with Linux paths needs `MSYS_NO_PATHCONV=1`.
6. **bmesh convex hull** triangulations can be slightly reflex (1.8e-5 m) on near-coplanar faces. Fixed with an edge-flip pass, applied to the final ≤ 64-vertex hulls only, because on large candidate hulls it costs seconds.
7. **Hull fidelity:** a ≤ 64-vertex hull is a subset of the exact hull, so LOD vertices protrude by up to 0.86 % of a (19.5 mm on the 2.9 m boulder). The hull also fills concavities.
   * In the **classic Bullet** plugin used by the current lunar world (gz-physics 7.6.0), `AttachMeshShape` builds a `btGImpactMeshShape` (symbol check), so *any* mesh, convex or not, becomes a GImpact triangle mesh there. The convexity buys nothing in that engine; the low triangle count (124) is what keeps it cheap.
   * bullet-featherstone does contain `btConvexHullShape`.
   * Options to evaluate in physics tests: 96-128 hull vertices for rocks > 1.5 m, or a ~300-500-triangle decimated (concave) collision mesh for large rocks, or one merged GImpact mesh per tile.
8. **Normal maps and tangents in Gazebo (not verified here; no Gazebo runs).** Maps are OpenGL/+Y with MikkTSpace tangents included in the GLB.
   * Ogre2 / gz-common may recompute tangents (not MikkTSpace) and may expect a different green convention.
   * Mitigation already in place: sharp edges at 40°, so the baked deviations are small.
   * Test with `normalmap_test.glb`. Light it from ±Y: the right (normal-mapped) tile must shade like the left (real relief). If the +Y bump looks lit from the wrong side, flip the G channel of `rock_atlas_normal.png` (one numpy line).
   * Blender's own glTF round trip is consistent (`preview_normalmap_test.png`).
9. **Texture memory and deduplication in Gazebo (open).**
   * One 4096² RGBA8 map is 64 MiB, or 85 MiB with mips, so the three atlas maps take about 256 MiB of VRAM. On a 2048² atlas the three maps take about 64 MiB.
   * Per-rock standalone GLBs each embed their own small textures. Do **not** instance many different standalone files if Gazebo does not deduplicate embedded textures by content. Prefer `rocks_library.glb` with `<submesh>`, or merged tile meshes that reference the atlas once. This needs testing on the Gazebo side.
10. **Exporter warnings, harmless** (output verified): "More than one shader node tex image used for a texture", because the ORM image feeds both roughness and metallic. `Material.use_nodes` deprecation warnings in 5.x.
11. **Look limitations:**
    * There is one shape "style": convex-ish fracture blocks, no overhangs or large spalls.
    * No zap pits at mm scale.
    * Clast palettes are plausible but uncalibrated.
    * The decimated tops of medium and large rocks show faceted shading without the normal map; this matters only if Gazebo ignores the normal map.
    * Pebble LODs are convex polyhedra with flat shading, which looks fine at 2-5 cm.

## 6. Recommended settings for the production library

* **Size:** about 50 prototypes, for example `--n-large 10 --n-medium 20 --n-pebble 20`.
  * Each class is log-stratified in size, so one seed covers the size range.
  * The world generator adds yaw, ±30 % scale and burial.
  * Lithologies cycle roughly 4 : 2 : 2 : 1 (light breccia : dark matrix : glass-coated : anorthositic).
* **Atlas:** `--atlas 4096` with the default texel targets (3 / 1 / 0.5 mm), which fills about 53 % (measured).
  * If VRAM is tight, use `--atlas 2048 --region-large 512 --region-medium 256`. Measured on the same 50 rocks (`experiments/blender/prod50_atlas2048`, verification PASS):
    * 81.6 % filled;
    * texels: large 4.45 mm mean [2.0..10.5], medium 1.11 mm [0.5..2.1], pebbles unchanged;
    * `rocks_library.glb` 9.13 MB;
    * about 64 MiB VRAM for the three maps with mips.
  * Consider dropping the ORM map (constant roughness 0.9) if VRAM is tight: roughness only varies for glass.
* **Bake:** `--bake-samples 8`. Keep the bake cage and ray distances automatic (derived from the measured LOD error).
* **Triangle budgets:** keep the defaults. LOD errors are ≤ 0.4 % (large) and ≤ 0.83 % (medium) of a, so the geometry the GPU LiDAR sees (0.573° beams, i.e. 2 cm at 2 m) is limited by the budget, not by the decimation quality.
  * About 50 % of large-rock triangles end up buried. Decimating weighted by height would recover detail if needed.
* **Collision:** convex hulls ≤ 64 vertices for rocks ≥ 5 cm, as designed, but see issue 7 for classic Bullet. Pebbles get no collision.
* **Axes:** keep `export_yup=True` (spec-compliant). Decide the SDF compensation (0 or roll +π/2) from the Gazebo `axis_marker.glb` test.
* **World generator inputs:** use `library.json` plus `npz/*.npz` (Z-up) to merge rocks into tile meshes that share the single atlas material. Use `rocks_library.glb` for individually placed models.

## 7. Reproduce

```bash
# Git Bash, from REPO/src/lunar_world_gen/blender
B="/c/Program Files/Blender Foundation/Blender 5.2/blender.exe"
"$B" -b --factory-startup -P rock_proto.py -- --out //wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/experiments/blender/test_set \
     --seed 1 --n-large 2 --n-medium 3 --n-pebble 4 --atlas 2048 --preview --preview-samples 128          # 64 s
"$B" -b --factory-startup -P rock_proto.py -- --out //wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/experiments/blender/prod50 \
     --seed 2 --n-large 10 --n-medium 20 --n-pebble 20 --atlas 4096 --preview --preview-samples 128      # 255 s
"$B" -b --factory-startup -P rock_proto.py -- --out <dir> --mode shapes   # clay-only shape iteration, ~30 s for 9 rocks
# verification (WSL)
MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu -e bash -lc '~/.venvs/lunar_world/bin/python \
  /mnt/c/Users/Username/OneDrive/Desktop/togo_ws-slam-nav/togo_ws-slam-nav/src/lunar_world_gen/tools/verify_rock_glb.py \
  /home/nargon/togo_assets/lunar/experiments/blender/test_set --json /home/nargon/togo_assets/lunar/experiments/blender/test_set/verify_report.json'
```

Helper scripts in the experiments dir: `summarize.py` (per-class statistics), `inspect_glb.py` (raw glTF structure and axis check), `table_and_images.py`.

## Sources

* Lunar Sample Compendium: [67016](https://www-curator.jsc.nasa.gov/lunar/catalogs/apollo16/part3/67016.pdf), [67115](https://www-curator.jsc.nasa.gov/lunar/catalogs/apollo16/part3/67115.pdf), [67015](https://curator.jsc.nasa.gov/lunar/catalogs/apollo16/part3/67015.pdf), [67435](https://www.nasa.gov/wp-content/uploads/static/history/alsj/a16/A16Sample67435.pdf), [66075](https://www.nasa.gov/wp-content/uploads/static/history/alsj/a16/A16LunarSample66075.pdf), [64455 (zap pits on lunar top)](https://www.nasa.gov/wp-content/uploads/static/history/alsj/a16/a16samp64455.html), [69935 (no fillet)](https://lsc.apolloinrealtime.org/apollo-16/69935/69935/)
* Impact-fragment axial ratios 2 : √2 : 1: [JpGU 2016 PPS11-P19](https://www2.jpgu.org/meeting/2016/session/PDF/P-PS11/PPS11-P19_e.pdf); [Ryugu block shapes (IAC)](https://iac.edu.es/en/science-and-technology/publications/three-axial-shape-distributions-pebbles-cobbles-and-boulders-smaller-few-meters-asteroid)
* Highland vs. mare normal albedo: [Lunar dust review, arXiv 2511.08503](https://arxiv.org/pdf/2511.08503)
* Rounded eroded tops, flat protected undersides: [Nature report](https://nature.com/articles/2231306a0.pdf); [NTRS 19710008023](https://ntrs.nasa.gov/api/citations/19710008023/downloads/19710008023.pdf)
