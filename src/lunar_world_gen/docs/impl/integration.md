# Integration stage: build, budgets, validation (phase 3)

Owner: integration agent, 2026-10-25. Implements `lunar_world_gen/build.py` (DESIGN.md section 14), the
post-phase-2 decisions of DESIGN section 13, and the DESIGN 13a budgets. Every number below was measured in
this session on the real data (preset `full`, seed 16) in WSL2 (12 cores, 15 GB). Gazebo runs used
`GZ_PARTITION=claude_p3int`, `ROS_DOMAIN_ID=81`, a private `ROS_HOME`, setsid and process-group stop.
Scratch: `/home/nargon/togo_assets/lunar/experiments/p3_integration/` (`EXP` below).

## 1. Result

| item | value |
|---|---|
| world | `/home/nargon/togo_assets/lunar/worlds/apollo16_core/`: `apollo16_core.sdf`, `_inspect.sdf`, `_stock.sdf`, 324 GLBs, 6 textures, `world_info.yaml`, `rocks_catalog.npz/.csv`, `reports/` |
| `gz sdf -k` | all three variants `Valid.`; 972 references per SDF (330 unique files), 0 missing; patched plugin present |
| static checks | **8 / 8 pass** (roughness site-aware, craters, rocks, visual-vs-collision, GLB, stitching, SDF, world_info) |
| quick load (world only, nothing injected) | loads with **0 `[Err]`, 0 `[Wrn]`**; world loaded 2.2-2.4 s, all 108 collision meshes built by 3.1 s, physics stepping (sim time 1 s) at 4.1 s; RTF 1.00; server RSS 371 MiB |
| visual triangles | 9.53 M (core ground 1.73 M, core rocks 6.59 M, ring/far/horizon 1.21 M) |
| collision triangles | 0.90 M (core 0.487 M, ring 0.388 M, far 0.026 M) |
| rocks | core 215,678 (16,493 with collision), ring 2,946 |
| test sites | flat, slope, rock, turn: all found, none relaxed |
| build time | 2:21 wall from the cached terrain (terrain itself 1:57 when not cached); peak process RSS 1.67 GiB |
| tests | 139 pass (`pytest tests`, 73 s); 11 new |

## 2. How to build

> **Superseded (note added 2026-10-26).** Sections 2-3 describe the phase-3 build with eight stages. Since the fix
> round the build has ten stages (`ground` and `contact` added, the visual meshes moved out of `meshes`), new
> options (`--force-downstream`, `--adopt`, `--lock-timeout`) and per-preset output folders (`--preset fast`
> builds into `<dir>_fast`). The current description is the `build.py` docstring, README section 3.3 and
> `docs/impl/final_build.md` sections 2 and 4; this report is kept as the phase-3 record.

From `src/lunar_world_gen` in WSL with the project venv:

```bash
PY=/home/nargon/.venvs/lunar_world/bin/python
$PY -m lunar_world_gen.build --preset full                    # all stages; unchanged stages are skipped
$PY -m lunar_world_gen.build --preset full --status           # which stages are up to date and why
$PY -m lunar_world_gen.build --stages meshes,export,world,validate --force   # re-run a subset
$PY -m lunar_world_gen.build --set mesh.core_visual_max_error_m=0.003       # override; affected stages re-run
```

- `--stages a,b`: run a subset. Every upstream stage outside the subset must be up to date, else the build stops
  and names the stage to add.
- `--force`: re-run the selected stages even if cached. The terrain is rebuilt only when `terrain` is named in
  `--stages` (a forced terrain rebuild re-extracts the DTM, about 2 min).
- Exit code 0 = built (and the static validation passed if it ran), 1 = a stage failed, 2 = validation failed.
- `--preset fast` uses the same `build_dir`/`world_dir` as `full` and would replace the full world. Build it
  elsewhere with `--set paths.build_dir=... --set paths.world_dir=...`.

## 3. Stages and caching

> Phase-3 stage list (eight stages); see the note at the top of section 2 for the current ten.

| stage | what it does | outputs |
|---|---|---|
| terrain | `terrain.compose.build_all(cfg)`, called only when a cached product is missing or stale. Up to date means: all 100 H0 tiles and the ring/far/horizon grids carry the current terrain key, the crater sets and base grids exist, and `terrain/manifest.json` has the key. | `terrain/` (terrain owner's caches and manifest) |
| rocks | `sample_rocks` for the core and the ring (details below), `concatenate`, `validate_catalog`, `save_catalog`, copied to the world dir | `rocks/rocks_catalog.npz/.csv`, `world_dir/rocks_catalog.npz/.csv` |
| heights | final core tiles `H = apply_fillets(H0 tile)`, 1 cm, with the 0.5 m margin; cached per tile (stage key in each tile's JSON meta) | `heights/tile_ii_jj.npy/.json` x 100 |
| collision | per tile: final H subsampled to 2 cm with its 0.5 m margin (low-pass support is 0.19 m), then `core_collision_grid` and a crop to the tile; the assembled 2 cm core grid; `ring_collision_grid` on the ring H0; far = far H0 - 0.02 m | `collision/tile_ii_jj_collision` x 100, `core_collision` (5001^2), `ring_collision`, `far_collision` |
| meshes | visual pieces: `make_piece` on each final H tile (per-tile grid normals, UV 2 m), ring/far/horizon strips; `stitch_pieces` + `check_stitched` (must be ok). Collision pieces: 100 tiles + ring/far strips, stitched and checked. `rocks_mesh_by_tile`. DESIGN 13a budgets. | `meshes/{visual,collision,rocks}/*.npz` |
| export | `write_glb` per piece (ground: regolith MaterialSpec, metallic 0, roughness 0.95; rocks: 0.85; collision: grey), stale GLBs removed, `make_regolith_textures`, rock atlas `basecolor/normal/roughness` copied as `rock_atlas_*.png` | `world_dir/meshes/*.glb` (324), `world_dir/textures/` |
| world | `find_sites` (2 cm core collision grid, catalog, visual H from the final tiles), `write_all_variants` (216 VisualSpecs, 108 CollisionSpecs at mu 0.4), `gz sdf -k` on every variant, `write_world_info` (triangle counts, rock counts, budgets, stage keys) | 3 SDFs, `world_info.yaml` |
| validate | `run_static_checks` with explicit inputs (H0 tiles, core B, crater sets, collision grid, catalog), then `build_report` | `reports/static_checks.json`, `validation_report.md` + figures |

- **Rock ground.**
  - Core: `CoreGround` is bilinear on the 1 cm H0 assembled from the 100 cached tiles including their margins (10101^2 nodes, ±50.5 m), and exact `height_at` beyond that. In the small-layout test it matches `height_at` to 0.2 mm.
  - Ring: `GroundGrid(ring H0)`.
  - Craters: annuli of 60 fresh craters at least 5 m wide, from the terrain 'big' set plus the DTM crater of the site report.
  - Both zones share one density field.
- **Stage key.** A hash of the stage version, the upstream stage keys, the stage's config sections, the source of the modules it calls and the input files it reads:
  - rock library digest;
  - `site_selection.yaml` (rocks);
  - atlas file sizes and mtimes (export);
  - presence of the physics plugin (world).

  Editing e.g. `mesh.collision.core_max_error_m` re-runs meshes, export, world and validate only (measured: rocks, heights and collision were skipped).
- **Manifest.** `build_dir/<stage>/manifest.json` records:
  - key, inputs, config-section hash;
  - outputs (path and bytes; a missing or resized output makes the stage stale);
  - timings, `peak_rss_mb`, results.

  `build_dir/build_manifest.json` records the last status of every stage.
- **Peak RSS per stage.** `VmHWM`, reset before each stage by writing 5 to `/proc/self/clear_refs`.

## 4. Timings, memory, disk

**Stage timings and peak RSS.** Run 1 is the first full run on the cached terrain. Run 2 is the rebuild after
the collision tolerance change (section 5). Run 3 is the rebuild after the export change (GLB extras, section 9).
Memory is in GiB: `VmHWM` per stage, `/usr/bin/time` maximum RSS for the whole process.

| stage | run 1: s / GiB | run 2: s / GiB | run 3: s / GiB | main parts of run 1 (s) |
|---|---|---|---|---|
| terrain | skipped (cached) | skipped | skipped | terrain owner's clean build: 1:57 wall, 2184 MiB self, 2782 MiB child (terrain manifest) |
| rocks | 10.9 / 0.75 | skipped | skipped | assemble H0 2.0, core 2.4, ring 1.9, save npz + csv 3.9 |
| heights | 2.2 / 0.29 | skipped | skipped | 100 tiles |
| collision | 2.2 / 0.52 | skipped | skipped | 100 tiles 1.7, ring + far 0.2 |
| meshes | 48.3 / 1.10 | 48.7 / 1.00 | skipped | core TIN + normals 36.9, strips 2.4, stitch 2.5, check 0.8, collision 2.3, rocks 3.6 |
| export | 11.5 / 1.58 | 11.2 / 1.49 | 11.2 / 1.34 | GLB 2.2, regolith textures 8.9 |
| world | 9.8 / 0.56 | 9.7 / 0.80 | 7.7 / 0.36 | sites 7.3, `gz sdf -k` x 3 2.3 |
| validate | 55.3 / 1.67 | 52.0 / 1.59 | 56.0 / 1.46 | roughness 42.2 (three deviograms of 10001^2), GLB 4.4, dz 1.8, stitching 1.4, report 1.3 |
| **total** | **2:21.4 / 1.67** | 2:03.0 / 1.59 | 1:16.2 / 1.46 | CPU 189 s user + 22 s sys (run 1) |

**Disk use.**

| folder | bytes | contents |
|---|---|---|
| build dir | **2.75 GB** | terrain 1.460 GB, meshes cache 0.539 GB, heights 0.485 GB, collision 0.221 GB, rocks 0.048 GB, manifests < 0.2 MB |
| world dir | **0.615 GB** | GLBs 0.521 GB (rocks 445.7 MB, ground 58.7 MB, collision 16.4 MB), textures 45.5 MB (rock atlas 32.3 MB, regolith 13.3 MB), catalog npz 18.6 MB + csv 29.1 MB, SDFs 3 x 0.21 MB, reports 0.63 MB, world_info 6.4 kB |

## 5. Triangle, rock and collision counts; budgets

| zone | visual ground | visual rocks | collision | per core tile (min / mean / max) |
|---|---|---|---|---|
| core (100 tiles) | **1,733,013** | **6,591,400** | **487,217** | ground 15,533 / 17,330 / 18,197; rocks 21,136 / 65,914 / 180,256; collision 2,114 / 4,872 / 11,533 |
| ring (4 strips) | 163,730 | 862,166 | 388,298 | |
| far (4 strips) | 80,419 | - | 26,467 | |
| horizon (4 strips) | 102,747 | - | - | |
| **total** | **9,533,475** visual | | **901,982** collision | ground + rocks per tile max 196,238 |

| budget (DESIGN 13a, config `budgets`) | limit | value | status |
|---|---|---|---|
| core terrain per tile | 200,000 | 18,197 | ok (no tile capped) |
| core terrain total | 8-10 M max | 1,733,013 | ok |
| core rocks | 6.6 M | 6,591,400 | ok (DESIGN 13a text still says 6 M) |
| total visual | 16 M | 9,533,475 | ok |
| core collision | 0.25-0.5 M (never > 1 M) | 487,217 | ok after tuning |

**Collision tuning.** The tolerance is now `mesh.collision.core_max_error_m` 0.01 -> **0.015** (recorded in the
config). The low-pass stays 0.25 m: about 70 % of the collision triangles are the 16.5 k baked rocks, which
the low-pass does not touch. Measured on the 100 real tiles (`EXP/exp_collision*.json`):

| core collision tolerance | 1.0 cm | 1.25 cm | 1.5 cm | 1.75 cm | 2.0 cm |
|---|---|---|---|---|---|
| triangles with rocks | 582,218 | 528,266 | **487,033** | 451,084 | 418,281 |
| triangles, ground only | 163,327 | - | 93,973 | - | 62,176 |

- **Rock tops survive.** Measured on 5,228 rocks in 34 tiles (`EXP/exp_rock_bumps.json`). Each rock raises the collision surface by a median 37 / 48 / 67 / 139 mm (D 5-7 cm / 7-10 cm / 10-20 cm / larger). The 1.5 cm TIN loses a median 0 mm of that rise (p90 0.13 mm, max 8.4 mm), and 90 % of rocks keep at least 99.8 % of it. pydelatin refines the rock peaks first.
- **dz cost.** Visual-vs-collision dz RMS goes from 4.3 to 5.6 mm and p99 from 10.7 to 14.2 mm; both are still well inside the limits.

**Core visual tolerance (not changed).** The configured 5 mm gives only 1.73 M triangles, because the calibrated
field (fbm_gain 0.278) is smoother than the mesh owner's projection (6 M). Measured on the 100 final tiles
(`EXP/exp_visual.json`, before stitching):

| core_visual_max_error_m | 2.0 mm | 2.5 mm | 3.0 mm | 3.5 mm | 4.0 mm | 5.0 mm |
|---|---|---|---|---|---|---|
| core ground triangles | 9,590,445 | 6,489,800 | 4,650,711 | 3,475,527 | 2,687,018 | 1,732,900 |
| max per tile | 99,027 | 67,353 | 48,378 | 36,113 | 28,022 | 18,197 |
| total visual (+ 7.80 M rocks and outer) | 17.4 M (over 16 M) | 14.3 M | 12.5 M | 11.3 M | 10.5 M | 9.5 M |

**Rocks** (catalog; triangles at the LOD the config picks):

| class | count | LOD | triangles |
|---|---|---|---|
| pebble (2-5 cm) | 199,185 | 0 | 5.28 M |
| small (5-12 cm) | 15,063 | medium LOD 2 | 0.745 M |
| medium (12-30 cm) | 1,325 | 1 | 0.245 M |
| large (>= 30 cm) | 105 | 0 | 0.319 M |
| ring (>= 30 cm) | 2,946 | large LOD 2 | 0.862 M |

- Largest core rock 1.43 m.
- Separation: core 19 big-rock redraws, 0 dropped; 631 small-rock redraws.
- Fillets: 1,703 rocks, maximum 69.3 mm.
- Collision: 203,281 core 2 cm nodes raised; ring collision rasterised 3,051 rocks of at least 30 cm.

## 6. Static checks (`reports/static_checks.json`, final run)

| check | result | measured | threshold |
|---|---|---|---|
| roughness (site-aware) | **pass** | composite H0 / target 0.806 / 0.932 / 1.085 / 1.241 at 0.01 / 0.03 / 0.1 / 0.3 m; synthetic H0 - B 0.898 / 0.835 at 1 / 3 m; composite / B 1.071 at 10 m | ±25 %; ±25 %; <= 1.15 |
| craters N(>=D) | **pass** | 12433 / 2026 / 524 / 125 vs 12496 / 1996 / 496 / 121 at 0.2 / 0.5 / 1 / 2 m | ±20 % |
| rocks N(>=D), core | **pass** | 215678 / 16493 / 2421 / 322 vs 216077 / 16610 / 2385 / 342 at 2 / 5 / 10 / 20 cm (z -0.9 / -0.9 / +0.7 / -1.1); 0 sanity problems | Poisson ±3 sigma |
| visual vs collision dz | **pass** | median -20.3 mm, RMS 5.6 mm, p99 14.2 mm, max 22.7 mm (152,582 rock-free points, per-tile meshes) | -20 ± 3 / <= 10 / <= 30 mm |
| GLB | **pass** | 324 files load in trimesh, +z normals 100 % of ground vertices, float32 / uint32, one primitive, metallic 0, 0 unreferenced | >= 99.9 % |
| stitching | **pass** | 484 pairs (visual 248, collision 236), 0 mismatches, 0 T-junctions, 0 gaps; `check_stitched` in the build: Euler 1, 0 cracks for both groups | 0 |
| SDF | **pass** | 973 / 973 / 972 references resolve | all |
| world_info | **pass** | 4 sites inside the core | - |

- **Stitching details.** Visual stitching inserted 1,827 boundary vertices, and every shared height was already bit-identical (max dz 0). Collision stitching overwrote ring boundary heights by up to 92 mm (rms 1.6 mm), because rocks of 5-30 cm raise the core collision but not the ring's.
- **Ring rocks** are reported, not judged.
  - Against the field-aware expectation of `sample_rocks` (rocks manifest) they fit: 2946 / 692 / 89 / 5 vs 2928 / 699 / 98 / 6.7 at 0.3 / 0.5 / 1 / 2 m.
  - Against the static check's plain area model they are 11.6 % high (2946 vs 2639), because the intensity is normalised to the core only (rocks.md, point 1).

## 7. Test sites (`world_info.yaml`; candidate lattice 0.5 m, 35,721 candidates in ±47 m)

| site | x, y (m) | z (m) | yaw | footprint slope / roughness | notes |
|---|---|---|---|---|---|
| flat | -4.50, -0.50 | -0.434 | 135.0° | 1.51° / 7.3 mm | 0 rocks >= 5 cm within 0.98 m; 4 m corridor: 1 rock >= 5 cm, max slope 6.4° |
| slope | 11.00, -4.00 | -2.540 | -157.5° | 6.77° / 12.1 mm | 5 m run at 12.3° mean (sub-slopes 7.0-15.2°, cross slope <= 4.4°), 0 rocks >= 10 cm in the corridor; 2,531 valid runs |
| rock | -5.11, -5.22 | -0.319 | 90.0° | 3.73° / 9.4 mm | lone 6.1 cm rock (exposed 3.1 cm) 2 m ahead on the left wheel line; 245 poses tried |
| turn | -4.50, 0.00 | -0.427 | -112.5° | 1.86° / 7.5 mm | 0 rocks >= 10 cm within 1.5 m (new rule) |

No relaxation was needed. Under the old 5 cm rule (re-run with `spawn.turn_rock_free_D_m=0.05` on the same
grid and catalog, `EXP/exp_turn_rule.py`) a 1.5 m disc free of 5 cm rocks does exist here, but only at
(-20.5, -5.0), 21 m from the centre, in a sparse patch of the log-normal density field. The new 10 cm rule puts
the turn site at (-4.5, 0.0), next to the flat site. world.md's "no such disc" holds for its uniform-density
synthetic test only.

## 8. Quick load test (world only, `EXP/load_test.py`, nothing injected)

`gz sim -s -r -v 4 apollo16_core.sdf` started under `timeout --foreground` in its own session, using
`tools/bench_world.GzServer` and `Watcher` on `/world/apollo16_core/stats`. Two runs:

| | run 1 (`EXP/load1`) | run 2 (final GLBs, `EXP/load2`) |
|---|---|---|
| physics plugin loaded | 0.94 s | 0.54 s |
| world loaded (PostUpdate threads), first stats | 2.38 s | 2.15 s |
| 108 `[lunar patch] mesh collision` lines | 2.39-3.13 s | 2.17 s + 0.74 s |
| sim time >= 1 s | 4.14 s | (wall clock stepped back 1.12 s mid-run, see 11) |
| RTF (server sim/real, 10 s) | 1.00 | 1.00 |
| server RSS / peak | 371 / 377 MiB | 371 / 377 MiB |
| `[Err]` / `[Wrn]` | 0 / 0 | 0 / 0 |
| stop | SIGTERM, group empty | SIGTERM, group empty |

The world has no rendering system (Sensors comes with the robot), so this run loads only the SDF and the
collision meshes. No GL library was mapped, as expected. Visual loading and the D3D12 renderer are exercised
by the benchmark stage (`bench_world.py` injects Sensors and a LiDAR).

## 9. Determinism and the fast preset

- **Determinism.** Everything after the terrain was rebuilt into a scratch folder (terrain cache shared read-only) and compared with the real world (`EXP/compare_worlds.py`):
  - all 324 GLBs have identical BIN chunks and JSON;
  - the catalog arrays and CSV are identical;
  - all 6 textures, the sites and the triangle counts are identical.

  The GLB `asset.extras` no longer carry the config hash, which depends on the output paths, so a rebuild elsewhere now gives byte-identical files.
- **Fast preset** (`--preset fast`, 1 cm core TIN, rocks from 3 cm / ring from 0.5 m), built into `EXP/fast_build` / `EXP/fast_world`:
  - 2:04 wall, 1.59 GiB peak;
  - visual 3.65 M: core ground 0.439 M, core rocks 2.72 M (69,205 rocks), outer 0.49 M;
  - collision 0.887 M;
  - 8 / 8 checks pass.

  The world is kept in `EXP/fast_world` (260 MB) as a light test world; its build dir was deleted.

## 10. Changes to other owners' modules, config and tests

| file | change | why |
|---|---|---|
| `lunar_world_gen/build.py` | new | the integration build |
| `mesh/stitch.py` (mesh) | `make_piece(..., max_triangles=None)`, passed to `tin_from_grid` | enforces the per-tile cap (DESIGN 13a); never triggered (max tile 18,197) |
| `world/spawn.py` (world) | the turn site uses `spawn.turn_rock_free_D_m` (default `rock_free_D_m`) over all core rocks; `_flat_like(..., index=)`; criteria report the D actually used | DESIGN 13 decision (c) |
| `validate/static_checks.py` (validation) | (1) `check_roughness` site-aware (see list below); (2) `check_visual_vs_collision` and the stitching pieces read the per-tile collision meshes `tile_ii_jj_collision.glb`, with `core_collision.glb` kept as fallback; (3) `locate_inputs` prefers `terrain/core_tiles` (H0) over other per-tile grids and finds `core_base_grid`; (4) `check_rocks` moves check sizes below the configured `D_min` up to it | DESIGN 13 decision (a); DESIGN 7 per-tile collision; the fast preset (3 cm) failed at 2 cm |
| `validate/report.py` (validation) | roughness table with the B and H0 - B columns, synthetic table, summary row "deviogram (site-aware)" | decision (a) |
| `tools/run_robot_tests.py` (validation) | `pass_fwd = v_ss / cmd >= 0.95 and distance / nominal >= 0.93`; new `pass_fwd_speed`, `pass_fwd_distance`, `fwd_rule`; constants `FWD_SPEED_MIN`, `FWD_DIST_MIN`, `TURN_YAWRATE_MIN`, `AIRBORNE_MAX`; `pass_fwd_steady_info` removed | decision (b) |
| `config/apollo16_core.yaml` | `spawn.turn_rock_free_D_m: 0.10`; new section `budgets` (DESIGN 13a numbers); `mesh.collision.core_max_error_m: 0.01 -> 0.015` | decisions; budgets |
| `tests/test_world_build.py` | new, 7 tests: full small-layout build through all stages (core 20 m, real DTM, synthetic library, about 21 s), caching and selective rebuild, upstream-stale refusal, fillets / collision tiles, CoreGround vs `height_at`, stale GLB removal | |
| `tests/test_world_validation.py` | mini world with per-tile collision; new tests: site-aware roughness (exact rows and the 3x-detail failure), legacy `core_collision.glb`, rock check D_min; the robot test covers the new forward rule (0.945 passes, 0.925 or v_ss 0.93 fails) | |
| `tests/test_world_sdf.py` | new `test_turn_site_rock_threshold` (a 7 cm rock blocks flat but not turn; 5 cm override) | |

**What `check_roughness` now judges** (DESIGN 13 decision (a)):

- the composite H0 within ±25 % at 0.01, 0.03, 0.1 and 0.3 m;
- H0 - B within ±25 % at 1 and 3 m (`rows_synthetic`; B comes from `terrain/base/core`);
- the composite at most 1.15 x B at 10 m and above (`nu_B_m`, `ratio_to_B`);
- without B, only the composite rule (`site_aware: false`).

Other owners' modules were otherwise left alone. No foundation module (`grid`, `config`, `layout`, `mesh/types`)
was changed.

## 11. Open issues and notes for the next stages

1. **Core terrain uses 1.73 M of its 8-10 M allocation** at the configured 5 mm (table in 5).
   - The performance study's design point was an error bound of about 3 mm, below the gpu_lidar's 5 mm range quantisation at 2-4 m.
   - On this field, 3 mm costs 4.65 M triangles and 2.5 mm costs 6.49 M (12.5 M / 14.3 M visual in total, both within 16 M).
   - I did not change it: the task asked only to cap. One override (`--set mesh.core_visual_max_error_m=0.003`, or a config edit) re-runs meshes to validate in about 2 min.
2. **Core rocks 6.59 M** pass the 6.6 M limit given to me with 8.6 k triangles to spare. DESIGN 13a still says "at most 6 M".
   - Pebble LOD0 alone is 5.28 M.
   - If 6 M is enforced, pebbles need a lighter LOD: LOD1 for all pebbles would give 2.39 M.
3. **WSL2 wall-clock step.** During load run 2, `time.time()` jumped back 1.12 s (a WSL time sync).
   - `tools/bench_world.py` (GzServer, Watcher) and `run_robot_tests.py` time every interval with `time.time()`, so load times measured across such a step are wrong.
   - Using `time.monotonic()` for intervals would avoid this. Not changed here: it is the validation owner's measurement tool and needs a Gazebo run to re-verify.
4. **Not exercised here:** the 216 visuals in a rendering process (D3D12, load time, VRAM), the robot launch on this world, and the LiDAR checks.
   - Predicted by the perf fits for 9.53 M triangles: first LiDAR about 10.7 s headless, server RSS about 1.35 GB, robot spawn stall about 14.6 s.
   - These are predictions, not measurements.
5. **Core-edge collision.** Collision stitching copies the core's boundary heights onto the ring strips (up to 92 mm where a 5-30 cm rock straddles the core edge). The surface stays watertight; a rock is cut at the edge with a short ramp on the ring side.
6. **World docs are partly stale.** `docs/impl/world.md` section 7 still lists a single `core_collision.glb`. The build writes per-tile `tile_ii_jj_collision.glb` (DESIGN 7), and the validation now reads both.
7. **Far and horizon visuals use `cast_shadows false`.** Terrain more than 250 m away cannot shade the core at a sun elevation of at least 21.9° (far z range -118..+44 m), and this saves shadow-pass work. Core and ring cast shadows.
8. **Validation report.** "visual / camera images" is SKIPPED (4 views + overview are a later stage's task). The static report counts it as 1 skipped.
9. The `meshes` stage keeps a 539 MB npz cache, so `export` can re-run alone. Delete `build_dir/meshes` to save disk; the next build then re-meshes (about 49 s).
