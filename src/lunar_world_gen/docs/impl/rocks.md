# Rocks: implementation report

Owner: rocks. Scope: DESIGN.md section 5 (rock library, population, placement, fillets, merging) and
the rock part of section 7 (collision height fields). All numbers below were measured on 2026-10-25
(WSL2, Ryzen 5 3600X, RTX 2080) and are reproducible with the commands in "How to run".

## What was built

| File | What |
|---|---|
| `blender/rock_library.py` | Production rock library (headless Blender 5.2.1, Cycles OptiX): 20 pebble + 20 medium + 10 large prototypes, LOD0/1/2, convex hulls, one shared atlas, npz/json exports, previews rebuilt from the exported files. Reuses the shape, material and bake code of `rock_proto.py`. |
| `blender/rock_proto.py` | Research prototype, still works. Only change: `sample_specs(args, class_cfg=None)` accepts a class table. |
| `lunar_world_gen/rocks/library.py` | `RockLibrary` reader. `write_synthetic_library` writes random convex rocks in the production format without Blender, for tests and dry runs. |
| `lunar_world_gen/rocks/population.py` | `sample_rocks`: broken power-law sizes, log-normal density field, fresh-crater annuli, Poisson thinning, separation rule. |
| `lunar_world_gen/rocks/placement.py` | Class and LOD, prototype, orientation, burial, contact height; quaternion helpers; ground wrappers. |
| `lunar_world_gen/rocks/catalog.py` | Catalog dtype; save/load (column-wise npz + csv); validation; N(>=D) with exact Poisson intervals; per-tile split; statistics. |
| `lunar_world_gen/rocks/fillets.py` | `apply_fillets` (numba, bit-identical across tiles). |
| `lunar_world_gen/rocks/collision_field.py` | `core_collision_grid` and `ring_collision_grid` (numba hull-top rasteriser). |
| `lunar_world_gen/rocks/merge.py` | `rocks_mesh` and `rocks_mesh_by_tile`, vectorised per (prototype, LOD) group. |
| `tests/test_world_rocks.py` | 26 tests: 25 on a synthetic library, 1 on the production library. |

The production library is at `/home/nargon/togo_assets/lunar/library/rocks_v1/` (`cfg.paths.rock_library`).

## Public API

```python
# library
RockLibrary(path)
  .prototypes -> list[Prototype]   # index, id, cls, dims (a,b,c), bottom_offset, lod_tris, lod_files, hull_file; .a, .c, .n_lods
  .by_class(cls) -> list[Prototype]                 # 'pebble' | 'medium' | 'large'
  .proto(id_or_index) -> Prototype ; .a_array() ; .dims_array()
  .lod_mesh(proto_id, lod) -> (positions (n,3) f4, normals (n,3) f4, uv (n,2) f4, indices (m,3) u4)   # cached, read-only
  .hull(proto_id) -> (k,3) f8 Z-up ; .hull_faces(proto_id) -> (t,3) i8 (outward CCW)
  .contact_points(proto_id, lod) -> unique LOD vertex positions (k,3)
  .atlas_paths -> {'basecolor', 'normal', 'orm', 'roughness': Path} ; .atlas_size ; .version ; .digest ; .summary()
write_synthetic_library(path, counts=None, seed=0, atlas_px=64) -> Path

# population and placement
sample_rocks(cfg, zone, ground, craters=None, library=None, rect=None, include_dtm_craters=True,
             field=None, return_meta=False) -> catalog | (catalog, meta)
    # zone 'core' (D 2 cm - 3 m) or 'ring' (>= 30 cm, ring square minus core square)
    # ground: object with .height(x, y) [.normal], a Grid, or a callable f(x, y)  (H0, without fillets)
    # craters: terrain CraterSet or anything with x, y, D, rho arrays (pass the same set for both zones)
    # library defaults to RockLibrary(cfg.paths.rock_library); rect restricts the region (tests, trials)
    # meta: expected_counts, intensity_integral_m2, intensity_scale, separation stats, timings, library digest
GroundGrid(grid), GroundFunction(height_fn, normal_fn=None), flat_ground(z0=0.0)
BrokenPowerLaw.from_config(cfg) -> .N(D) .inverse(n) .sample(rng, n, D_min, D_max) .count_density(D_min, D_max)
density_field(cfg) -> Grid ; place_rocks(cfg, rng, x, y, D, zone, ground, library) -> dict of catalog fields
quat_to_matrix(q) ; quat_from_axis_angle ; quat_mul ; quat_z_to(n) ; catalog_quats(rows)    # q = (w, x, y, z)

# catalog
CATALOG_DTYPE ; empty_catalog(n) ; concatenate([cats], renumber=False)
save_catalog(cat, path, meta=None, csv=True) -> [npz, csv] ; load_catalog(path) ; load_catalog_meta(path)
validate_catalog(cat, library=None, cfg=None) -> list[str] ; check_catalog(...)   # raises
cumulative_counts(cat, D_list, expected=None, conf=0.95) -> rows: N, ci95, expected, z, within_3sigma
poisson_interval(n, conf) ; split_by_tile(cat) -> {(zone, tile_i, tile_j): row indices} ; catalog_stats(cat, library, cfg)

# fillets, collision, merge
apply_fillets(cfg, grid, catalog, library=None) -> Grid          # copy: H = H0 + fillets
fillet_field(cfg, grid, catalog, library) -> ndarray
core_collision_grid(cfg, ground_2cm: Grid, catalog, library) -> Grid
ring_collision_grid(cfg, ring_ground: Grid, catalog, library) -> Grid
rocks_mesh(catalog_rows, library, lod_override=None, name='') -> mesh.types.Mesh  # lattice = -1, one primitive
rocks_mesh_by_tile(catalog, library, lod_override=None) -> {(zone, ti, tj): Mesh}  # names tile_ii_jj_rocks / ring_<side>_rocks
```

Catalog: one row per rock with the DESIGN fields `id, zone, tile_i, tile_j, proto, cls, lod, x, y, z, qw, qx,
qy, qz, scale, D, exposed_h, burial, collide`, and `p_world = (x, y, z) + scale * R(q) @ p_proto`.

## How to run

```bash
# library (Git Bash, from src/lunar_world_gen/blender); about 8.5 min including previews
B="/c/Program Files/Blender Foundation/Blender 5.2/blender.exe"
"$B" -b --factory-startup -P rock_library.py -- \
    --out //wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/library/rocks_v1 --seed 16 --atlas 4096 --preview
"$B" -b --factory-startup -P rock_library.py -- --out <dir> --mode preview      # previews only
# tests (WSL)
cd src/lunar_world_gen && ~/.venvs/lunar_world/bin/python -m pytest tests/test_world_rocks.py -q
```

Integration sketch for one build:

```python
lib = RockLibrary(cfg.paths.rock_library)
craters = terrain.compose.get_model(cfg).craters("big")      # its fresh craters (rho >= 0.73, D >= 5 m) get annuli
core = sample_rocks(cfg, "core", GroundFunction(lambda x, y: height_at(cfg, x, y, "core")), craters, lib)
ring = sample_rocks(cfg, "ring", GroundGrid(ring_H0), craters, lib)
cat = concatenate([core, ring]); save_catalog(cat, build_dir / "rocks" / "rocks_catalog", meta=...)
H_tile = apply_fillets(cfg, H0_tile, cat, lib)                   # per tile; margins are fine
meshes = rocks_mesh_by_tile(cat, lib)                            # -> tile_ii_jj_rocks.glb, ring_<side>_rocks.glb
C_core = core_collision_grid(cfg, H_2cm_with_fillets, cat, lib)  # pass >= 15 cm margin for exact edge filtering
C_ring = ring_collision_grid(cfg, H_ring, cat, lib)
```

## Measured results

### Production library (`rocks_v1`, seed 16, 4096² atlas)

| class | n | a [m] | LOD0 / LOD1 / LOD2 tris (min..max) | LOD error rms (max) [mm] | texel LOD0/1/2 mean [mm] | time / bake per rock [s] |
|---|---|---|---|---|---|---|
| pebble | 20 | 0.020-0.048 | 16..40 / 12 / 12 | 0.69 (4.2) / 1.77 (7.8) / same | 1.09 / 2.52 / 2.52 | 2.6 / 2.2 |
| medium | 20 | 0.051-0.284 | 210..974 / 62..292 / 20..96 | 0.13 (1.5) / 0.25 (3.1) / 0.70 (4.8) | 0.86 / 0.86 / 1.71 | 7.4 / 3.8 |
| large | 10 | 0.335-2.854 | 2140..4934 / 642..1478 / 214..492 | 0.34 (5.9) / 0.70 (7.9) / 1.37 (18.7) | 2.71 / 5.39 / 5.43 | 24.8 / 5.9 |

- Lithologies: 23 light breccia, 11 dark-matrix breccia, 11 glass-coated breccia, 5 anorthositic.
- LOD1/LOD2 methods: medium and large use collapse decimation of LOD0 (53 LODs); targets below 48 triangles use a greedy convex hull (47 LODs: all pebble LOD1/2 and 7 medium LOD2). Every LOD is closed after welding equal positions, and outward.
- Hulls: pebbles 10-22 vertices (the LOD0 itself); medium and large 64 vertices, strictly convex. LOD0 protrudes beyond the hull by up to 1.9 mm (medium) and 17.4 mm (2.85 m boulder, 0.6 % of a).
- Atlas: 4096², 130 regions (each distinct LOD has its own bake), 90.3 % allocated, no region halving needed. PNG sizes: basecolor 14.6 MB, normal 17.1 MB, orm 0.59 MB, roughness 0.60 MB.
- Other files: npz 1.59 MB in total, hulls 62 kB, whole directory 48 MB including previews.
- Run time: 459 s to generate plus 60 s for 11 preview renders, 519 s in total.
- Determinism:
  - Rebuilding prototypes with the same seed gives bit-identical positions, normals, indices and hulls. Checked on 3 prototypes rebuilt as a subset with a different atlas, and on a full 3-rock rebuild.
  - Baked atlases differ in 1 pixel per 10^6 by 1 LSB (GPU float noise).
- Previews (15° sun, black sky), rebuilt from the exported npz files: `img/rocks_overview.jpg`, `rocks_closeup.jpg`, `rocks_large_detail.jpg`, `rocks_scatter.jpg` (1,210 rocks drawn from the LM-site SFD with the world's class and LOD rules), `rocks_lods_{large,medium,pebble}.jpg` (LOD0/1/2 side by side), plus clay versions. They show angular to subangular light-grey breccia blocks with clasts, crack networks and dark glass splashes, and the LOD chain is visually stable because every LOD's normal map is baked from the high-poly.

### Full core and ring on flat ground (seed 16, production library)

| zone | rocks | N(>=2/5/10/20 cm) observed vs expected | z-scores |
|---|---|---|---|
| core | 215,678 | 215,678 / 16,493 / 2,421 / 322 vs 216,077 / 16,610 / 2,385 / 342 | -0.86 / -0.91 / +0.74 / -1.10 |
| ring | 3,138 (>= 30 cm) | N(>=0.3/0.5/1/2 m) 3,138 / 748 / 96 / 7 vs 3,120 / 745 / 105 / 7.2 | +0.33 / +0.13 / -0.87 / -0.07 |

- Per class in the core, with triangles at the LOD the config picks:
  - pebble 199,185 at LOD0: 5.28 M triangles;
  - small 15,063 at medium LOD2: 0.745 M;
  - medium 1,325 at LOD1: 0.245 M;
  - large 105 at LOD0: 0.319 M.
  - Core total: **6.59 M triangles**; 16,493 rocks have collide set; the largest rock is 1.43 m.
- Ring: 3,138 rocks at large LOD2, 0.919 M triangles.
- Per core tile: 23.7 k / 65.9 k / 207.7 k triangles (min / mean / max). Ring strips: S 416 k, N 332 k, W 65 k, E 106 k.
- The core intensity integral is exactly 10,000 m² (normalisation, see below; scale 0.952 with only the DTM crater, 0.873 with the real crater set).
- Separation: 2,421 rocks >= 10 cm with 8 redraws and 0 dropped; 613 small-rock redraws.
- The validation owner's `static_checks.check_rocks` passes on this catalog: all fields present, 0 tile mismatches, 0 collide-flag mismatches, all core rows within ±3σ.

| step | time | notes |
|---|---|---|
| `sample_rocks` core | 1.6 s | sizes and positions 0.37 s, separation 0.24 s, placement 0.96 s |
| `sample_rocks` ring | 0.73 s | |
| `rocks_mesh`, whole core in one mesh | **3.2 s** | 10.4 M vertices, 6.59 M triangles, RSS 1.0 GB |
| `rocks_mesh_by_tile`, 104 meshes | **3.5 s** | 100 tiles + 4 ring strips |
| `core_collision_grid`, 5001² at 2 cm | **2.1 s** | low-pass 0.77 s; raster 0.36 s for 16,493 rocks; 171 k nodes raised |
| `ring_collision_grid`, 2001² at 25 cm | 0.10 s | 3,243 rocks >= 30 cm; 7,080 nodes raised |
| `apply_fillets`, 100 tiles of 1101² | 1.2 s | 1,703 filleted rocks; first call includes 0.7 s numba JIT |
| `save_catalog` | 4.0 s | npz 18.6 MB, csv 29.2 MB |
| peak RSS of the whole benchmark | 1.5 GB | |

### Real-data trial (real terrain tiles from `build/apollo16_core/terrain/core_tiles`)

The trial used the terrain owner's cached H0 tiles (1101² at 1 cm, 0.5 m margin) through `GroundGrid`, and the real `CraterSet('big')` (14,764 craters, 60 of them fresh with D >= 5 m) plus the DTM crater.

| tile | rocks | N(>=2/5/10/20 cm) vs expected | intensity integral | fillets | collision raised nodes | mesh tris |
|---|---|---|---|---|---|---|
| tile_08_00 (SE, inside the 22.6 m fresh crater's annulus) | 5,334 | 5,334 / 390 / 62 / 12 vs 5,397 / 415 / 60 / 8.6 | 249.8 m² (2.5x nominal) | 46 rocks, max 2.1 cm | 4,249 | 168,534 |
| tile_04_05 (centre) | 1,446 | 1,446 / 116 / 21 / 3 vs 1,480 / 114 / 16 / 2.3 | 68.5 m² (0.68x) | 16 rocks, max 1.8 cm | 1,284 | 48,544 |

- All counts are within ±3σ, and the catalog validates with no problems.
- Worst lowest-vertex clearance over the real ground (2,000 largest rocks per tile): 1e-15 m, so no rock floats.
- Figures: `img/rocks_trial_tile_08_00.png` and `img/rocks_trial_tile_04_05.png` (hillshade of H0 + fillets with rock outlines; collision minus visual).

### Tests

`python -m pytest tests/test_world_rocks.py -q` → **26 passed in 7.1 s**. They cover:
- SFD: the table values to 1 %, continuity, the inverse, sampling at 2/5/10/20 cm within 3σ (2 M draws), and the full pipeline on 60 × 60 m (78 k rocks) within 3σ;
- density field statistics and determinism; crater annulus factor and measured density ratio; core intensity normalisation;
- determinism of the catalog, and a different seed giving a different catalog;
- no floating rocks on rough analytic ground and on a 1 cm `Grid` ground (lowest vertex = -burial · height, within 1e-6 m) and exposed height;
- class, LOD, burial ranges, perched fraction, tilt bound; the separation rule; ring zone geometry, ids and strips;
- quaternions;
- merge: indices, unit normals, closed and outward after welding, the exact transform, LOD override;
- collision: box top and tilted top equal to the analytic values, the core field against an independent numpy reference, the ring field;
- fillets: bit-identical across tile splits, height bound, C1 gradient bound, disabled flag;
- catalog: npz/csv round trip, column-wise keys, validation catches corrupted rows, tile split, Poisson intervals;
- empty and tiny inputs;
- the production library: counts, budgets, LOD ratios, dtypes, closed LODs, convex hulls of <= 64 vertices, atlas files.

## Contract changes and refinements (with reasons)

1. **Core intensity normalisation.**
   - What: the density field m times the crater factor e is scaled by one constant so that its mean over the core is exactly 1. The same constant is applied in the ring, so the intensity stays continuous.
   - Why: without it, a 100 m core, about 3 correlation lengths wide, shifts its totals by ±20 % per realisation (this seed's raw mean was 0.80). The 22.6 m DTM crater's annulus alone adds 5 %, and the real crater set adds 15 %.
   - Effect: the validation compares core counts with N_sfd × core area at ±3σ (σ = 0.2 % at 2 cm), which would otherwise fail. Fields and annuli now redistribute rocks; the core keeps the calibration's site-average totals.
2. **Prototype choice.** Uniform within the library class, but only among prototypes whose `a` is within a factor `rocks.proto_size_window` (default 3) of D, so the scale stays within 1/3..3. This keeps the texel size and the LOD error near their design values: a 0.33 m prototype is never blown up to 3 m.
3. **Height rule (contact rule).** The rock is lowered until its lowest visual-LOD vertex touches H0 under that vertex, then sunk by `burial × (rotated height)`.
   - On a plane with no extra tilt this is exactly DESIGN's `z = ground + s (bottom_offset - f c)`.
   - On rough or tilted ground it guarantees contact with no floating, which "lowest of 5 footprint samples" only approximates.
4. **Catalog conventions.**
   - `cls` indexes `cfg.rocks.classes` in config order.
   - Ring rocks have `tile_i` = strip index in `layout.RING_SIDES` (0 south, 1 north, 2 west, 3 east), `tile_j = -1`, and ids from 10,000,000.
   - `collide` is core-only (DESIGN 5). The ring collision rasterises rocks of 30 cm and larger by size, from any zone, so core rocks crossing the core edge also count. The core collision also includes ring rocks of 30 cm and larger that reach over the core edge.
   - The npz is column-wise (`d["x"]`, ..., plus `meta_json`), because the validation reads it that way.
5. **Library format.**
   - The npz UV key is `uv`; the reader also accepts the prototype's `uv_atlas_gltf`.
   - An extra `rock_atlas_roughness.png` (grey) is written next to the glTF ORM map, because Gazebo's `<roughness_map>` reads a single channel and would see R = 1 in the ORM map.
   - Each distinct LOD has its own unwrap and bake. Texel factors per LOD: large 1/2/2, medium 1/1/2, pebble 1/2/2.
   - Pebble LOD0 is 16-40 triangles. Pebble LOD1 = LOD2 = a 12-triangle hull: 30 % and 10 % of at most 40 triangles fall below the 12-triangle floor, so the two LODs share geometry and an atlas region.
6. **Optional config keys** (read with defaults; I did not edit the config): `rocks.proto_size_window` (3.0), `rocks.separation_min_D_m` (0.10), `rocks.crater_enhancement.min_rho` (0.73: the calibration's "fresh" class, about 10 % of craters).
7. **DTM craters.** Fresh craters that the DTM resolves are read from `site_selection.yaml`: the 22.6 m crater at (32, -60), with d/D 0.22, so rho = 1. Its 11-23 m annulus reaches into the core's south-east, as the config comment intends.
8. **Fillets.** Footprint ellipse of 0.45 × (a, b) × s, rotated by the rock's yaw. Height and width fractions come from a hash of (world seed, rock id), so tiles agree bit-for-bit. Overlapping fillets combine with max.

## Known limitations

- The rocks are placed on the 1 cm H0 grid, but the visual ground is a TIN with up to 5 mm error, so perched rocks can float or sink by up to that error.
- The core and ring populations are sampled independently: there is no separation check across the core boundary.
- The density field is isotropic: there is no elongation of rays radial to South Ray.
- Collision tops are point-sampled at the grid nodes. A 30 cm ring rock covers only a few 25 cm nodes, and nothing dilates the footprint conservatively.
- Texture density:
  - large LOD0 is 1.7-5 mm per texel, so stair-stepping of clast edges is visible closer than about 1.5 m;
  - pebble LOD0 is 0.7-1.7 mm per texel, limited by its 64 px regions.
- VRAM: the four 4096² maps are 85 MiB each with mips; the SDF needs three of them (about 256 MiB). `--atlas 2048` gives about 64 MiB at 2× the texel size.
- Not verified in Gazebo yet: the normal-map green convention (OpenGL +Y is written) and the tangents that Ogre computes for the merged GLBs, which are mesh-owner GLBs without tangents.
- The large glass-coating blotches on some glass-coated breccias look somewhat patterned up close (inherited from the prototype material).
- `PROGRESS.md` was not edited (not my file); the deviations are recorded here and in the module docstrings.
