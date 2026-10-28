# Final integration build (after the fix round)

Owner: integration agent `integrate`, 2026-10-25, 18:15-19:10. Scope: the cross-package fixes flagged in
`docs/impl/fix_*.md`, the test suite, the full rebuild of the main world with the fix-round config, the GUI
camera, static and Gazebo world-only checks. Gazebo runs used `GZ_PARTITION=claude_final_int`,
`ROS_DOMAIN_ID=101`, a private `ROS_HOME`, `setsid`, coreutils `timeout`, and SIGKILL to the process group
only. The machine was quiet (no other agent's jobs; load average 0.3-2.8). Scratch:
`/home/nargon/togo_assets/lunar/experiments/final_integrate/` (`EXP` below: scripts, logs, JSONs).
`config/apollo16_core.yaml` was not edited (proposals in section 11).

## 1. Result

| item | value |
|---|---|
| world | `~/togo_assets/lunar/worlds/apollo16_core/`: 4 SDF variants, 324 GLBs (758 MB), 6 textures, `world_info.yaml`, `rocks_catalog.npz/.csv`, `reports/` |
| build | `python -m lunar_world_gen.build --preset full`, all 10 stages from scratch (terrain re-extracted): **7:18 wall, exit 0** (first run with the same code minus the report fix: 7:38, identical world) |
| static checks | **8 / 8 pass**, `all_ran` true (section 6); `validation_report.md`: Overall PASS, 8 passed, 1 skipped (camera images, a later stage) |
| `gz sdf -k` | `apollo16_core`, `_inspect`, `_stock`, `_inspect_stock`: all `Valid.` |
| quick load (world only, headless) | **0 `[Err]`, 0 `[Wrn]`**, patched plugin loaded, 108 `[lunar patch] mesh collision` lines, sim time 1 s at 3.9 s, RTF 1.00, RSS 359 MB, stopped by SIGKILL, 0 processes left |
| LiDAR bench (extra, flat site, `real`) | first cloud **27.0 s** (limit 30 s), RTF 1.000, peak RSS 3.69 GB, VRAM peak 4.83 GiB, D3D12, 0 errors |
| visual triangles | **17,129,317**: hard cap 20 M ok, **soft target 16 M exceeded (warning)**; 16.81 M vertices |
| collision triangles | 806,122 (core 440,766 in the 0.25-0.5 M target) |
| rocks | core 215,677 (16,546 with collision), ring 37,029 (3,278 >= 30 cm + 33,751 core-edge band) |
| sites | flat (-2.00, -4.50), slope (23.50, -16.50), rock (-4.12, -7.41), turn (-3.00, 3.00); none relaxed, none missing |
| tests | **273 passed** (`pytest tests -q`, 3:52); before: 266 collected, 262 passed, 4 failed |
| determinism | rebuild: all 324 GLBs, 4 SDFs, 6 texture PNGs, the catalog CSV and every cache `.npy` identical (section 10) |
| world_built | **true** (one soft budget warning; open issues in section 11) |

## 2. Code changes

| file | change | why / finding |
|---|---|---|
| `lunar_world_gen/build.py` | stage `rocks`: ring `sample_rocks(..., avoid=core)`; results add per-zone overlap stats, `n_ring_main`, `n_ring_edge_band`, `n_perched` | (a), fix_rocks 9.1 |
| `build.py` | stage `heights`: filleted rows = `rocks.fillets.fillet_mask` (was `burial > 0`) | (b): the tiles were already right (`apply_fillets` filters again), but `filleted_rocks` counted 2,382 instead of 1,705 |
| `build.py` | stage `contact`: summary `core_perched` (`burial == 0`, now always empty: 0 of 215,677 core rows) replaced by `core_light_burial` (burial <= `terrain.fillets.min_burial`, 65,270 rows) | consequence of FID-01/12 (compliance burial of perched rocks) |
| `build.py` | stage `collision`: results report the ring dilation; `CONFIG_KEYS["collision"]` += `mesh.collision.ring_rock_dilation` | (c) |
| `build.py` | stage `validate`: new `archive_stale_results()`: Gazebo results (bench / LiDAR / robot JSONs, the LiDAR capture folder, camera images) older than this build's SDF are moved to `reports/stale_before_<time>/` before the report is built | **found here**: the rebuilt world's report included the phase-3 world's 8 Gazebo results ("Overall: FAIL", load 19.6 s measured at 9.5 M triangles, old sites); now 8 files moved, report = this world only |
| `build.py` | `STAGE_VERSION` rocks 3, heights 3, contact 2, collision 3, validate 4; module docstring | |
| `lunar_world_gen/rocks/recontact.py` | `fillet_rows()` = `rocks.fillets.fillet_mask()` | (b), fix_rocks 9.2: 677 perched rocks >= 10 cm (bedded 2-6 %, no fillet) were kept on H0 instead of being re-seated |
| `lunar_world_gen/rocks/collision_field.py` | FID-08: `_raster_max_square` (numba): exact max of each hull triangle over the square node +- w (Sutherland-Hodgman clip, linear z at the clipped vertices); `rasterize_tops(..., half_width=)`, `rock_top_grid(..., half_width=)`, `rows_near_grid(..., margin=)`; `ring_rock_dilation(cfg)` (key `mesh.collision.ring_rock_dilation`, node spacings, default `RING_ROCK_DILATION = 1.0`); `ring_collision_grid(..., dilation=)`; meta `rock_dilation_nodes`, `rock_dilation_half_width_m` | (c): with w = one node spacing the four nodes of every grid cell a rock touches are at least as high as the rock in that cell, so the collision surface covers the whole footprint; `0` = the old point samples |
| `lunar_world_gen/rocks/population.py` | `CRATER_ENHANCEMENT` default `annulus_R: [1.0, 2.0]` | real bug: the legacy `profile: annulus` raised `KeyError: 'annulus_R'` with the shipped config (the key was dropped from the YAML in the fix round) |
| `tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh` | CMake configure runs inside the build folder (`(cd "${BUILD}" && cmake ...)`) | (e) U8: gz-physics' bullet / bullet-featherstone / dartsim / tpe CMakeLists call `execute_process(create_symlink)` without `WORKING_DIRECTORY`, i.e. in the caller's directory |
| `tests/test_world_sdf.py` | earthshine compared with `lighting.realistic.earthshine_intensity` (also the inspect mode) and the SDF's `<intensity>` round trip | (d) |
| `tests/test_world_rocks.py` | `test_class_lod_burial_and_tilt`: lower bound `min(class lo, perched lo)`; the pebble perched fraction from the placement's perched count when a class range starts at 0 (burial no longer identifies perched rocks); `test_ring_collision_grid`: point samples keep the old "inside a footprint" bound, the dilated grid is within one node spacing of a footprint, never lower, and raises > 2x the nodes | config FID-12 (class burial [0, 1]); (c) |
| `tests/test_world_terrain.py` | `test_fbm_unit_fields_cached_and_deterministic`: the fine-gain case sets `fbm_gain_fine` (FID-06 split the gains); new: the coarse gain does not leak into the fine field, `fbm_gain_fine: null` falls back to `fbm_gain` | test assumed one shared gain; stronger now |
| `tests/test_world_build_fixes.py` | `test_fillet_rows_selection`: + a perched row (burial 0.04, no fillet) | (b) |
| `tests/test_world_build.py` | new `test_integration_wiring` (no ring-core conflicts, fillet counts, filleted rows kept on H0, `core_light_burial`, dilation in the collision results); new `test_stale_gazebo_results_are_archived` | (a)-(c), report fix |
| `tests/test_world_final_integration.py` | new, 5 tests: square-max raster equals an independent vertex-enumeration reference (1e-9) and is never below a 5 mm dense sample; dilated ring grid covers every rock footprint (point samples leave > 30 % of footprint points out); key 0 reproduces the point raster, monotone in w; contact keeps only `fillet_mask` rows; U8 with fake cmake/make/ldd and the real patch step (the old script fails it: the symlink lands in the caller's folder) | |
| `README.md` 3.1 | plugin build: no `cd` needed, the working-directory note updated | (e) |
| `config/gui/apollo16.gui.config` | camera block regenerated by `make_apollo16_gui_config.py` (flat site moved) | task 3 |
| outside the package | deleted the 4 dangling links `/home/nargon/libgz-physics-{bullet,bullet-featherstone,dartsim,tpe}-plugin.so` | each checked: symlink, target missing, target = the matching `libgz-physics7-*-plugin.so`, mtime 2026-10-25 11:51:02.1-.2, inside the configure step of the 11:50:42-11:51:17 plugin build (`cmake_configure.log` 11:51:02.49) |

Not changed: the flat-site rule (section 11.2), DESIGN.md, PROGRESS.md, integration.md, USER_GUIDE.md and the
README's "as built" numbers (section 11.4).

## 3. Measurements of the fixes

### 3.1 (a) Ring sampled with `avoid=core` (`EXP/scripts/a_avoid_measure.py`, final placed catalog)

| ring sampling | ring-core pairs tested | conflicts (centre inside / pierce) | ring rocks to move | time |
|---|---|---|---|---|
| without `avoid` (same streams, re-sampled) | 1,481 | **6** (5 / 5): 6 core-edge band rocks of 2.3-7.8 cm | 6 | 4.95 s |
| with `avoid=core` (build) | 1,453 | **0** | 0 | 4.49 s (`sample_ring`) |

In the build the ring repair found 7 initial conflicts and moved 7 rocks (1 round); the band 444 -> 0 (452 moved, 3 rounds); the core 2,865 -> 0 (2,919 moved). 0 dropped anywhere.

### 3.2 (b) Rows kept on H0 by the contact stage

On the placed catalog the old rule (`burial > 0`, D >= 10 cm, core) selects 2,382 rows, `fillet_mask` (burial > 0.06) 1,705; the other 677 are perched rocks bedded 2-6 % without a fillet, which are now re-seated on the rendered ground. Contact stage, final build:

| rows | n | hover > 1 mm before -> after | submerged before -> after | max hover after | dz p1 / p50 / p99 |
|---|---|---|---|---|---|
| core re-seated | 213,972 | 1,070 -> 0 | 103 -> 3 | touching (-0.001 mm) | -1.8 / +0.3 / +2.4 mm |
| of which burial <= 0.06 | 65,270 | 906 -> 0 | 23 -> 0 | touching | -1.6 / +0.4 / +2.5 mm |
| core filleted (kept on H0) | 1,705 | 0 -> 0 | 0 | -3.3 mm | 0 |
| ring incl. 33,751 band rocks | 37,029 | 11,072 -> 0 (1,804 > 20 mm) | 8,219 -> 2 | touching (was 51.0 mm) | -34.5 / +2.0 / +39.1 mm |

The 3 submerged core rocks are 2.3-2.6 cm pebbles with burial 0.76-0.85 whose top vertex lies 0.4-0.5 mm under the 3 mm TIN (`EXP/out_submerged.json`; 120 more within 2 mm): Beta(1.5, 6) on [0, 1] (FID-12) buries a few pebbles almost completely. Harmless.

### 3.3 (c) FID-08: ring collision capture (`EXP/scripts/fid08_measure.py`)

Ring rocks >= 30 cm (zone 1). Capture = the fidelity reviewer's s10 metric (max collision rise over the 25 cm nodes within centre +- D/2 divided by the visual rise + 1 cm, on the 3 cm strip TINs); coverage = fraction of the footprint (hull upper surface sampled at 2 cm, above the collision ground) where the collision TIN is >= hull top + `rock_offset_m` - 3 cm (the TIN tolerance); inflation = raised grid area / footprint area.

| data | w (node spacings) | ring collision tris | capture p10 / p50 | capture < 0.5 / < 0.25 | footprint points covered | rocks fully covered | min margin p50 | inflation p50 |
|---|---|---|---|---|---|---|---|---|
| phase-3 world (2,946 rocks; reviewer: 0.40 / 0.87, 12.0 % / 8.6 %) | 0 | 387,779 | 0.41 / 0.87 | 11.9 % / 8.5 % | 29.1 % | 0 % | -172 mm | 1.04 |
| | 0.5 | 419,764 | 0.98 / 1.02 | 0 / 0 | 98.4 % | 60.9 % | -21 mm | 3.9 |
| | **1** | 437,922 | 1.01 / 1.08 | 0 / 0 | 99.99 % | 99.9 % | -1.7 mm | 8.6 |
| final world (3,278 rocks) | 0 | 279,971 | 0.19 / 0.82 | 18.6 % / 10.9 % | 32.5 % | 0.2 % | -163 mm | 1.06 |
| | 0.5 | 314,605 | 0.98 / 1.02 | 0 / 0 | 98.5 % | 67.1 % | -16 mm | 4.0 |
| | **1 (built)** | **338,434** (338,850 stitched) | 1.01 / 1.07 | **0 / 0** | **99.95 %** | **99.7 %** | -1.5 mm | **8.8** |

- 30-50 cm rocks (2,500): capture < 0.5 24.2 % -> 0 %, footprint coverage p10 0.04 -> 1.00.
- Cost: +58.5 k ring collision triangles (+21 %), ring grid 0.18 s; ring nodes raised 6,831 -> 41,293.
- Price of "conservative": a ring rock now collides up to one node spacing (25 cm) before its visual surface (raised area 9.7x the footprint for 30-50 cm rocks, 3.0x for >= 1 m). `ring_rock_dilation: 0.5` covers 98.5 % at 4x. The rover is meant to stay in the core.
- At the core edge, collision stitching copies the core's 2 cm values onto the ring's boundary vertices. With the dilation the largest such overwrite is 0.55 m (rms 26 mm; 11 of 1,604 boundary nodes > 5 cm, point samples: 0.11 m, 2 nodes): at (-28.5, 50.0) the dilated square of a boundary node reaches the 1.52 m core boulder 0.84 m inside the core; the core's exact value wins on the boundary line, the rock lies entirely in the core (`EXP/out_edge_overwrite.json`). Stitching stays exact (0 mismatches).

### 3.4 (d) earthshine test

`test_lighting_modes` reads `lighting.realistic.earthshine_intensity` (3e-5) for both modes; the SDF writes `3e-05` and the default-variant test checks the parsed value.

### 3.5 (e) U8, plugin build from an empty working directory (`EXP/scripts/u8_plugin_build_test.sh`)

Real build with `--out`/`--work` in `EXP/u8`, the cached tarball, 4 jobs: rc 0 in 36 s, **0 entries** left in the working directory, the four unversioned symlinks in `work/build/`, plugin 895,760 bytes (same size as the installed one), installed plugin sha256 unchanged. The regression test (`test_plugin_build_leaves_nothing_in_the_callers_directory`) fails on the old script variant (symlink in the caller's folder) and passes on the new one.

### 3.6 Test suite

| run | result |
|---|---|
| baseline (before any change, 18:17) | 266 collected: 262 passed, 4 failed: `test_rocks::test_crater_enhancement` (`KeyError 'annulus_R'`, code bug), `test_rocks::test_class_lod_burial_and_tilt` (burial >= 0.02 assumed; config FID-12 gives class range [0, 1]), `test_terrain::test_fbm_unit_fields_cached_and_deterministic` (fine field scaled by `fbm_gain`; it has `fbm_gain_fine` since FID-06), `test_world::test_lighting_modes` (earthshine 0.01 hard-coded) |
| after build 1 | 271 passed, 2 failed: `test_docs::test_gui_camera_looks_at_flat_site_of_world_info` (expected until the GUI camera is regenerated) and `test_rocks::test_ring_collision_grid` (asserted the point-sample footprint; updated for the dilation) |
| final (after the rebuild and the GUI camera) | **273 passed** in 3:52, 0 skipped (`EXP/logs/pytest_final.log`) |

## 4. Build: stages, timings, memory, disk

`EXP/logs/build_full2.log` (final code), `/usr/bin/time -v`. Peak RSS = `VmHWM` of the build process per stage (GiB); the terrain's `dtm_extract` children peak at 2.72 GiB (2,782 MiB), which is also the `/usr/bin/time` maximum.

| stage | build 2 (final): s / GiB | build 1: s | main parts (build 2, s) |
|---|---|---|---|
| terrain | 111.9 / 2.39 | 116.4 | base (dtm_extract re-extraction) 46.8, fbm coarse 3.4, fine 6.0, craters 5.4, talus 15.3 (218 windows, 74,468 nodes, max \|T\| 0.470 m), tiles 32.5, zones 1.7, checks 0.5 |
| rocks | 21.6 / 1.16 | 22.1 | core 12.2, ring incl. band 4.5, save 4.1 |
| heights | 6.5 / 0.79 | 6.2 | 100 tiles, 1,705 fillets, max 46.7 mm |
| ground | 124.6 / 1.46 | 129.6 | core TIN 3 mm + seam band + normals 112.3, strips 1.9, stitch 6.6, check 3.3 |
| contact | 70.8 / 0.96 | 75.9 | clearance before 22.3, re-seat 22.2, clearance after 21.8, save 4.2 |
| collision | 3.2 / 1.11 | 3.5 | core tiles 1.6, ring + far 1.3 |
| meshes | 7.6 / 1.00 | 6.9 | collision TINs 3.3, stitch + check 0.6, rock merge 3.3 |
| export | 11.5 / 2.03 | 12.0 | GLB 2.7, regolith textures 8.7 |
| world | 9.6 / 1.03 | 9.4 | sites 6.8, `gz sdf -k` x 4 2.7 |
| validate | 67.0 / 2.17 | 71.8 | static checks 65.5 (roughness 47.1), report 1.3 |
| **total** | **437.3 s, wall 7:18.4**, CPU 587 s user + 38 s sys | 456.6 s, 7:37.6 | |

| folder | bytes | contents |
|---|---|---|
| build dir | **3.20 GB** | terrain 1.564 GB (base 0.439, fBm 0.510, talus 0.102, core tiles 0.485, zone grids 0.025), meshes cache 0.508, heights 0.485, ground 0.310, collision 0.221, contact 0.056, rocks 0.056 |
| world dir | **0.864 GB** | GLBs 758.4 MB (rocks 488.6, ground 255.1, collision 14.7), catalog csv 34.3 + npz 21.9 MB, textures 45.5 MB, reports 3.3 MB (2.6 MB of it archived phase-3 results), SDFs 4 x 0.21 MB, world_info 8.7 kB |

## 5. Counts and budgets

| zone | visual ground | visual rocks | collision | per core tile min / mean / max |
|---|---|---|---|---|
| core (100 tiles) | **8,738,711** (4.41 M vertices; seam band +208,669) | **6,583,462** (10.41 M vertices) | **440,766** | ground 65,701 / 87,387 / 95,128; rocks 23,846 / 65,835 / 118,530; collision 2,113 / 4,408 / 7,262 |
| ring (4 strips) | 131,918 | 1,488,384 (1.83 M vertices) | 338,850 | |
| far (4 strips) | 80,419 | - | 26,506 | |
| horizon (4 strips) | 106,423 | - | - | |
| **total** | **17,129,317 visual** (16.81 M vertices) | | **806,122 collision** | ground + rocks per tile max 206,806 |

| rocks | count | LOD / triangles |
|---|---|---|
| core pebble 2-5 cm | 199,131 | LOD0, 5,280,514 |
| core small 5-12 cm | 15,185 | medium LOD2, 751,032 |
| core medium 12-30 cm | 1,251 | LOD1, 234,742 |
| core large >= 30 cm | 110 (largest 1.52 m) | LOD0, 317,174 |
| core N(>=D) 2 / 5 / 10 / 20 / 50 / 100 cm | 215,677 / 16,546 / 2,382 / 332 / 26 / 5 | |
| ring >= 30 cm | 3,278 (>= 0.5 / 1 / 2 m: 778 / 101 / 7; field-aware expectation 3,260 / 778 / 110 / 7.5) | large LOD2, 958,968 |
| ring core-edge band < 30 cm | 33,751 (pebble 31,151, small 2,391, medium 209) | 529,416 |
| collision proxies | 16,546 core rocks >= 5 cm + 3,278 ring rocks (190,844 core 2 cm nodes and 41,293 ring nodes raised) | |

| budget (config `budgets`) | kind | limit | value | status |
|---|---|---|---|---|
| core tile ground | hard | 200,000 | 95,128 | ok (0 tiles capped) |
| core ground | hard | 10,000,000 | 8,738,711 | ok |
| core rocks | hard / soft | 7,500,000 / 6,600,000 | 6,583,462 | ok / ok (16.5 k spare) |
| visual total | hard / soft | 20,000,000 / 16,000,000 | 17,129,317 | ok / **WARNING (+7.1 %)** |
| core collision | hard / target | 1,000,000 / 0.25-0.5 M | 440,766 | ok / ok |
| collision total | hard / soft | 1,500,000 / 1,000,000 | 806,122 | ok / ok |

Why 17.1 M: the 3 mm core TIN (orchestrator decision) on the recalibrated terrain (fine fBm gain 0.578 instead of 0.278) needs 8.53 M triangles before the seam band, against 4.65 M on the phase-3 field. Measured on the 100 final tiles (`EXP/out_tin_scan.json`, pre-stitch):

| `core_visual_max_error_m` | 3 mm + seam band (built) | 3 mm | 3.5 mm | 4 mm | 5 mm |
|---|---|---|---|---|---|
| core ground triangles | 8,738,018 | 8,529,349 | 6,506,800 | 5,109,557 | 3,364,793 |
| max per tile | 95,115 | 93,681 | 71,151 | 55,933 | 37,641 |
| visual total (+ 8.39 M rocks and outer zones, seam band ~+0.2 M) | 17.13 M | 16.9 M | ~15.1 M | ~13.7 M | ~11.9 M |

## 6. Static checks (`reports/static_checks.json`, final build)

| check | result | measured | threshold |
|---|---|---|---|
| roughness (site-aware) | **pass** | H0 / target 0.926 / 1.028 / 1.125 / 1.230 at 0.01 / 0.03 / 0.1 / 0.3 m; H0 - B 0.892 / 0.831 at 1 / 3 m; H0 / B 1.060 at 10 m | +-25 %; +-25 %; <= 1.15 |
| craters N(>=D) | **pass** | 12,433 / 2,026 / 524 / 125 vs 12,496 / 1,996 / 496 / 121 at 0.2 / 0.5 / 1 / 2 m (ratios 0.995 / 1.015 / 1.056 / 1.032) | +-20 % |
| rocks N(>=D), core | **pass** | 215,677 / 16,546 / 2,382 / 332 vs 216,077 / 16,610 / 2,385 / 342 at 2 / 5 / 10 / 20 cm (z -0.86 / -0.50 / -0.06 / -0.56); sanity: 0 problems (33,751 band rows exempt, 0 other ring rows < D_min, quaternion norm dev 2e-16) | Poisson +-3 sigma |
| (rocks, ring, informational) | not judged | 3,278 / 778 / 101 vs the plain area model 2,639 / 630 / 89 (core-normalised intensity; field-aware expectation 3,260, z +0.32) | - |
| visual vs collision dz | **pass** | median -20.3 mm, RMS 6.0 mm, p99 15.2 mm, max 78 mm (rock-free core points, per-tile meshes) | -20 +- 3 / <= 10 / <= 30 mm |
| GLB | **pass** | 324 files load in trimesh, 0 problems, +z normals 100 % of ground vertices (every file), 0 unreferenced | >= 99.9 % |
| stitching | **pass** | 484 pairs (visual 248: tile 180, ring-tile 40, ring 4, far-ring 8, far 4, far-horizon 8, horizon 4; collision 236), 0 mismatches, 0 T-junctions, 0 height mismatches, 0 gaps, 112 / 108 pieces complete | 0 |
| SDF | **pass** | 973 / 973 / 972 / 972 references resolve (default / inspect / stock / inspect_stock), 0 missing; Physics, UserCommands, SceneBroadcaster only; gravity -1.62; 1 ms (patched) / 4 ms (stock) | all |
| world_info | **pass** | 4 sites in the core, 0 problems | - |

## 7. Stitching (build)

- Visual: 112 pieces, 248 segment + 162 point contacts, 8,415 vertices inserted, 32,986 shared heights overwritten with the canonical ones (max change 1.3e-15 m: already identical); `check_stitched`: 41,133 shared points, 0 mismatches, 0 T-junctions, 0 normal mismatches.
- Collision: 108 pieces, 941 vertices inserted, 3,039 overwritten (max 0.546 m, rms 16.5 mm; see 3.3), check: 4,035 shared points, 0 mismatches, 0 T-junctions.
- Seam band (FID-07): 100 tiles, +208,669 triangles, 4,627 flips, 144 edges marginally over 8 cm.

## 8. SDF variants and Gazebo runs (`EXP/logs/gz_checks.log`, `EXP/gz/`)

| run | result |
|---|---|
| `gz sdf -k` x 4 | `Valid.` (0.6-0.7 s each); engine: default/inspect `/home/nargon/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so`, stock/inspect_stock `gz-physics-bullet-plugin`; 216 visuals, 108 collisions |
| quick load, world only (`EXP/scripts/load_test.py`, nothing injected) | plugin loaded 0.85 s, first stats 2.25 s, sim time 1 s at 3.89 s, RTF 1.000 over 10 s, RSS 359 MB (HWM 368 MB), 0 `[Err]` / 0 `[Wrn]` (also during shutdown), 108 patched mesh-collision lines; no rendering system in the world, so no GL driver mapped (`libGLX` only), as expected |
| LiDAR bench (`tools/bench_world.py`, `real` 209 x 122 @ 20 Hz, sensor 0.449 m above the flat site, 30 s window) | world loaded 2.14 s, render init 4.17 s, LiDAR advertised 26.73 s, **first cloud 26.97 s**, RTF 1.000 (worst 2 s bin 0.83), RSS 3.57 GB at load / 3.69 GB peak, VRAM 4,834 MiB (idle 2,379), `libd3d12.so`, `libd3d12core.so`, `libdxcore.so` mapped, no software rendering, 20.0 Hz, 13,731 finite of 25,498 points, 0 errors; other processes 0.2 % CPU |

Every gz server was started under `timeout -s KILL`, stopped with SIGKILL to its process group (0 processes left); `%TEMP%\wsl-crashes` was empty before and after all runs. Phase-3 references: first cloud 19.6-21.1 s at 9.53 M triangles / 12.2 M vertices (quiet), 28.3 s at 13.5 M / 15.0 M (fix_build, 2.7 busy cores).

## 9. Spawn sites (`world_info.yaml`) and GUI camera

| site | x, y (m) | z (m) | yaw | footprint slope / roughness | notes |
|---|---|---|---|---|---|
| flat | -2.00, -4.50 | -0.485 | -22.5 deg | 4.85 deg / 5.9 mm | 0 rocks >= 5 cm within 0.98 m; 4 m corridor: 2 rocks >= 5 cm, max footprint slope 9.39 deg, grade over 3 m -7.4 deg (downhill); 164 of 35,721 candidates qualify |
| slope | 23.50, -16.50 | -3.368 | -157.5 deg | 5.78 deg / 18.0 mm | 5 m run at 12.43 deg mean (sub-slopes 6.3-16.9 deg, cross <= 1.5 deg), rise 1.10 m, 0 rocks >= 10 cm in the corridor, 2,376 valid runs |
| rock | -4.12, -7.41 | -0.387 | 67.5 deg | 2.92 deg / 6.2 mm | lone 8.65 cm rock (exposed 4.3 cm) 2 m ahead under the left wheels; path max slope 3.5 deg; 484 targets tried |
| turn | -3.00, 3.00 | -0.545 | -67.5 deg | 4.84 deg / 4.4 mm | 0 rocks >= 10 cm within 1.5 m |

All four sites moved (phase 3: flat (-4.5, -0.5), slope (11.0, -4.0), rock (-5.11, -5.22), turn (-4.5, 0.0)). The
wrapper's `--dry-run` resolves the new poses from `world_info.yaml` (flat, slope checked).

GUI: `python3 config/gui/make_apollo16_gui_config.py` rewrote the camera (`--check` before: STALE, rc 1; after: up to date, rc 0): `<camera_pose>-6.619 -2.587 1.494 0 0.3631 -0.3927</camera_pose>`, a chase view 5 m behind the flat spawn, 2.2 m above its visual ground. With the wrapper's follow transform, the camera stands 1.94 / 2.28 / 1.99 / 2.05 m above the terrain under it and the line of sight to the rover body clears the terrain by at least 0.37 / 0.42 / 0.38 / 0.39 m at flat / slope / rock / turn (`EXP/out_gui_camera.json`). At flat the view now faces east-south-east: the sun is 28 deg left of the view direction (was behind).

## 10. Determinism

Build 1 (18:32, same code except the report archive) and build 2 (18:49, final) from scratch: all 324 GLBs, the 4 SDFs, the 6 texture PNGs and `rocks_catalog.csv` are byte-identical, and so are 640 of the 642 cache `.npy`/`.npz` files. The differing files carry build metadata only: `rocks_catalog.npz` (stage key in its meta; its CSV rows are identical), `textures/regolith_textures.json`, `world_info.yaml` (time, stage keys), the lock files.

## 11. Open issues and proposals (for the orchestrator)

1. **Visual budget and load time.** 17.13 M visual triangles exceed the 16 M soft target (hard 20 M ok). First LiDAR cloud 27.0 s on a quiet machine, 3 s inside the 30 s `validation.load_time_max_s`; contention adds about 4 s (fix_build), so a busy machine will exceed it. RSS 3.69 GB (limit 5 GB). Proposal: `mesh.core_visual_max_error_m: 0.0035` (core ground ~6.7 M with the seam band, visual ~15.1 M, under 16 M; scan in section 5), then re-run ground and downstream (~5 min).
2. **Flat site.** The nearest qualified site has a 4.85 deg footprint and a rough corridor (2 rocks >= 5 cm, 9.4 deg max footprint slope, -7.4 deg grade): the standard drive test (DESIGN 13: distance >= 0.93, turn >= 0.85) is likely to be judged on non-level ground again (gz_validation 5.2). On this terrain only 164 candidates qualify at all; the nearest with a rock-free corridor of <= 5 deg max slope is (-25.5, -4.0) (25.8 m out, grade -3.8 deg), <= 4 deg (-37.0, -1.5) (37 m out, grade +0.3 deg), none at <= 3 deg (`EXP/out_flat_site.json`). A corridor requirement in `world/spawn.py` or a grade-aware drive criterion is a DESIGN decision; not changed.
3. **Ring collision inflation.** The FID-08 dilation is conservative by design: ring rocks collide up to 25 cm before their visual surface. `mesh.collision.ring_rock_dilation: 0.5` would trade full coverage (99.7 % -> 67 % of rocks fully covered, 98.5 % of footprint points) for 4x instead of 8.8x area.
4. **Stale documentation** (not edited here): README 1 / 3.3 / 4 "as built" numbers (9.5 M triangles, 215,678 rocks, 2,946 ring rocks, sites (-4.5, -0.5) etc., "Eight cached stages", 2.8 GB cache, start-up 35-40 s at 9.5 M triangles), USER_GUIDE 1.1 / 1.7 / 1.8 (rock counts, earthshine 0.01, site table), `integration.md` sections 2-3 (8 stages), DESIGN 3 / 4 / 12 (fix_terrain 6, fix_docs 8). The numbers to use are in sections 4, 5 and 9 above.
5. **Gazebo validation of this world** (robot launches at the new sites, LiDAR captures and checks, camera images) has not been run; `reports/` now holds only this build's static results, the phase-3 results are in `reports/stale_before_20261025_185520/`.
6. The 3 submerged core pebbles and 2 submerged ring rows after the contact stage are nearly fully buried by design (3.2); the build's mini-layout test still requires 0 for the core.
7. `config/apollo16_core.yaml` needs no change for this round; optional explicit key for the new default: `mesh.collision.ring_rock_dilation: 1.0`.

## 12. Reproduce (WSL, from `src/lunar_world_gen`)

```bash
PY=/home/nargon/.venvs/lunar_world/bin/python; E=/home/nargon/togo_assets/lunar/experiments/final_integrate
$PY -m pytest tests -q                                           # 273 tests, ~4 min
$PY -m lunar_world_gen.build --preset full                       # ~7.3 min from scratch
python3 config/gui/make_apollo16_gui_config.py && python3 config/gui/make_apollo16_gui_config.py --check
bash $E/scripts/gz_checks.sh                                     # gz sdf -k x 4 + quick world-only load
bash $E/scripts/run_bench.sh                                     # LiDAR bench at the flat site
PYTHONPATH=. $PY $E/scripts/fid08_measure.py --ring-height ~/togo_assets/lunar/build/apollo16_core/terrain/ring_height \
    --catalog ~/togo_assets/lunar/build/apollo16_core/contact/rocks_catalog.npz --out /tmp/fid08.json
PYTHONPATH=. $PY $E/scripts/a_avoid_measure.py /tmp/avoid.json   # also: tin_tolerance_scan.py, flat_site_analysis.py,
bash $E/scripts/u8_plugin_build_test.sh                          # submerged_check.py, gui_camera_check.py, edge_overwrite_check.py
```
