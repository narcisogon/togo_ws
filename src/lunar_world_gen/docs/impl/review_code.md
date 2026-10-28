# Code review: lunar_world_gen (phase 3)

Reviewer: `review-code` agent, 2026-10-25, 13:15-13:50. Read-only review of `src/lunar_world_gen` (build.py, every
package module, tools, launch file, run script, tests). No project file was edited; no Gazebo, ROS or git command
was run. Experiments ran in `/home/nargon/togo_assets/lunar/experiments/p3_review-code/` (`EXP` below; every
script and its `.out` are kept there), niced, with at most 3 threads, against read-only copies of the real build.

Files modified by other agents while this review ran (reviewed only lightly): `tools/bench_world.py`,
`tools/run_robot_tests.py`, `tools/lidar_capture.py`, `validate/lidar_checks.py`, `validate/report.py`,
`tests/test_world_validation.py` (13:25-13:38). `build.py` and `validate/static_checks.py` were stable since 13:08.

## 1. Result

- **Tests:** `pytest tests` = **139 passed in 76.9 s**, 0 skipped, peak RSS 0.61 GB (`EXP/pytest_full.log`;
  run with `-p no:cacheprovider`, `PYTHONDONTWRITEBYTECODE=1`, 3 threads).
- **The current world is consistent:** the real `static_checks.json` passes 8/8; frames, quaternion conventions,
  the 1 cm lattice, float32 ranges and determinism were checked and found correct (section 4).
- **13 findings, none high.** Four medium: a hard rock budget with no headroom (3 of 8 seeds fail the build), stage
  caching that ignores `paths.world_dir`, a static rock check that passes corrupted catalogs, and a per-tile triangle
  cap that can never satisfy its own budget check. The rest are low: rocks hovering above the rendered ring ground,
  dormant numerics, robustness and test gaps.

| id | sev | area | finding (confirmed by) |
|---|---|---|---|
| CR-01 | medium | config `budgets` / build meshes | `core_rocks_max_tris` 6.6 M is at the mean: seeds 17, 18, 21 give 6.68 / 6.67 / 6.62 M -> BuildError (E9) |
| CR-02 | medium | build.py caching | changing only `paths.world_dir` skips rocks/export/world; the new world dir stays empty (E6) |
| CR-03 | medium | validate/static_checks.py | `check_rocks` passes inverted collide flags, 2x quaternions, a 1 mm rock, duplicate ids (E2) |
| CR-04 | medium | build.py stage_meshes | the per-tile cap overshoots (pydelatin +1, stitching +13) -> hard budget violation (E1) |
| CR-05 | low | rocks placement (ring) | 394 of 2946 ring rocks hover above the rendered ground, 75 by > 2 cm, max 43 mm (E5) |
| CR-06 | low | concurrency | two builds into one build dir both crash; fixed temp names, no lock, non-atomic PNGs (E8) |
| CR-07 | low | tools/dtm_extract.py | vertical-iteration decided per row chunk: output depends on `--chunk-nodes` (74 mm), dormant (E4) |
| CR-08 | low | robustness | no patched plugin -> static SDF check fails, build exits 2; `--status` traceback without library (E7) |
| CR-09 | low | physics config | `physics.rock_mu` 0.6 is never applied; SDF holds only mu 0.4 (E10) |
| CR-10 | low | scripts/run_apollo16_sim.sh | an option without its value spins forever (E3) |
| CR-11 | low | cache keys | terrain/foundation code, library npz and third-party versions not in keys; `--force` does not cascade (inspection) |
| CR-12 | low | memory | merged rocks forced to float64 + an int64 `-1` lattice: 60 B/vertex instead of 32 (E10) |
| CR-13 | low | tests | none of the above is covered; one near-vacuous assertion (inspection) |

## 2. Findings

### CR-01 (medium) Rock triangle budget has no headroom
- **Evidence.** `config/apollo16_core.yaml:182` `core_rocks_max_tris: 6600000` is a hard limit
  (`build.py:914`, `_check_budgets` 952-968). The production value is 6,591,400 (integration.md section 5). DESIGN 13a
  says "at most 6 M".
- **Reproduction** (`EXP/e9_rock_budget_seeds.py`, flat ground, production library and crater set; seed 16
  reproduces the production 6,591,400 exactly):

  | seed | 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 |
  |---|---|---|---|---|---|---|---|---|
  | core rock triangles | 6,591,400 | **6,683,808** | **6,667,496** | 6,553,572 | 6,598,678 | **6,615,008** | 6,562,972 | 6,542,674 |

  Mean 6,601,951, std 51,635. Three of eight seeds raise `BuildError: budget exceeded` in the meshes stage.
- **Impact.** Any seed change, library rebuild or SFD tweak fails the build about 40 % of the time.
- **Fix.**
  - Keep a hard cap only on the visual total (16/20 M), and make the rock budget a warning.
  - Or set the limit to at least mean + 4 sigma (about 6.81 M).
  - Or cut the pebble cost: pebble LOD0 alone is 5.28 M, and LOD1 for pebbles gives 2.39 M (integration open issue 2).

### CR-02 (medium) Stage caches ignore `paths.world_dir`
- **Evidence.** The keys (`CONFIG_KEYS`, `build.py:88-99`) of `rocks` (copies the catalog into world_dir,
  622-630), `export` and `world` do not contain `paths.world_dir`. `up_to_date()` (446-463) only checks that the
  recorded output files exist, and they do: in the OLD world dir.
- **Reproduction** (`EXP/e6_world_dir_cache.py`, mini layout of tests/test_world_build.py):
  1. Build with world_dir W1: all stages built; W1 has 3 SDF, 36 GLB, 6 textures, catalog.
  2. Run again with the same build_dir and world_dir W2: terrain..world `skipped`, validate re-ran. W2 has 0 SDF, 0 GLB, 0 textures, no catalog, no world_info.
  3. The validation then reports `rocks: error` and 5 checks `skipped`.
- **Related.** `--preset fast` resolves to the same build/world dirs as `full` (E10b), so a fast build silently
  replaces the validated full world (integration.md mentions it only as a note). `summarize_results`
  (`static_checks.py:1735`) counts `skipped` as passing, so the world-level checks do not guard against an empty
  world dir.
- **Fix.**
  - Put `paths.world_dir` and `world.name` into the keys of rocks, export and world, or make `up_to_date()` reject manifests whose outputs are not under the current build/world dir.
  - Derive default dirs per preset (for example `apollo16_core_fast`), or refuse a preset that differs from the one in `build_manifest.json`.
  - In the build's validate stage, require `summary["all_ran"]`.

### CR-03 (medium) `check_rocks` passes corrupted catalogs
- **Evidence.**
  - `static_checks.py:1037-1038`: `sanity_ok` uses only the missing fields, non-finite values, centres outside the core, tile assignment and D_max.
  - `collide_flag_mismatch` (1036), `quat_norm_max_dev` (1023) and `core_below_D_min` / `ring_below_D_min` (1018-1019) are computed but never judged.
  - Duplicate ids are not computed.
  - validation.md promises that the collide flag and quaternion norms are part of the check.
- **Reproduction** (`EXP/e2_check_rocks_sanity.py`, real catalog copied into scratch). Every case gets status **pass**:
  - all collide flags inverted (218,624 mismatches);
  - all quaternions x2 (norm deviation 1.0);
  - one core rock with D = 1 mm;
  - a duplicated id.
- **Impact.** The independent acceptance gate cannot catch a catalog that disables every rock collision proxy. The build itself is mostly protected: the rocks stage runs `validate_catalog`, which catches the collide, quaternion and duplicate-id cases, but not D below D_min. Related: `check_stitching` reports `complete` (1673) but does not judge it.
- **Fix.** Add `collide_flag_mismatch == 0`, `quat_norm_max_dev <= 1e-6`, `*_below_D_min == 0`, unique ids and proto/LOD range to `sanity_ok`; fail stitching when `complete` is false; add a corrupted-catalog test.

### CR-04 (medium) Per-tile triangle cap cannot pass its own hard check
- **Evidence.**
  - `build.py:802-817`: an over-budget tile is re-meshed with `max_triangles=cap`.
  - The tiles are then stitched (830), and counts are recorded after stitching (842).
  - `_check_budgets` compares the post-stitch maximum with the same cap as a HARD limit (952-968, 914).
- **Reproduction** (`EXP/e1_cap_budget.py`):
  - pydelatin overshoots the cap: cap 1001 gives 1002 and 5000 gives 5001.
  - A tile capped at 2000 and stitched to a 25 cm neighbour ends with **2013** triangles (13 inserts).
  - `_check_budgets` then returns `ok: false`, which means BuildError.
  - On the real build, 36 core tiles receive stitch inserts (up to 9 visual and 14 collision per tile; meshes manifest), so any capped tile fails.
  - The same happens to an uncapped tile within a few triangles of the cap.
- **Impact.** Dormant at 5 mm (largest tile 18,197). It triggers below about 1.5 mm tolerance or with a lower budget: the safety mechanism of DESIGN 13a turns into a failed build.
- **Fix.** Cap with headroom (for example `0.97 * cap`), or judge the per-tile limit on the pre-stitch count. Alternatively, re-cap and re-stitch when a capped tile ends above the cap.

### CR-05 (low) Rocks hover above the rendered ring ground
- **Evidence.**
  - Ring rocks are placed on the bilinear 25 cm ring H0 grid (`build.py:605-607`, `placement._contact_heights`).
  - The rendered ring ground is its TIN, which may lie up to `ring_visual_max_error_m` = 5 cm below the grid.
- **Reproduction** (`EXP/e5_rock_float.py`). The merged rock geometry, exactly what `rocks_mesh` writes, was located in the world's own ground GLB TINs:

  | | rocks | hover > 1 mm | > 1 cm | > 2 cm | max gap |
  |---|---|---|---|---|---|
  | ring (all) | 2946 | 394 (346 of 861 perched) | 189 | **75** | **43.4 mm** (rock 10001370, D 0.32 m, x 85.6, y 22.2) |
  | core D >= 10 cm, 8 tiles | 207 | 16 | 0 | 0 | 4.3 mm (the documented 5 mm limit) |

- **Impact.** Visible gaps or shadow lines under ring boulders in close camera views. Negligible for the LiDAR beyond 50 m (beam spacing there is at least 0.5 m).
- **Fix.** Sink perched ring rocks by `ring_visual_max_error_m`, or place ring rocks on the meshed TIN (`tin_surface_on_grid`) by meshing the ring strips first. Document the core 5 mm case as is.

### CR-06 (low) No build lock; fixed temporary names
- **Evidence.**
  - Fixed temporary names:
    - `_write_json` uses `<name>.tmp` (`build.py:189`);
    - `_save_grid` and `compose._save` use `<prefix>_tmp` (`build.py:227`, `compose.py:427`);
    - `fbm._save_atomic` uses the same pattern (186);
    - `extract_base` uses `base/<zone>_tmp` (`base.py:219`).
  - No lock exists anywhere.
  - Regolith PNGs are written in place (`regolith.py:321-323`).
- **Reproduction** (`EXP/e8_concurrent_build.py`). Two builds of the same mini config were started at the same time:
  - A died with `JSONDecodeError` in `base/core_tmp.json`, written by both dtm_extract children;
  - B died with `FileNotFoundError` on `build_manifest.json.tmp`;
  - `core_tmp.npy`/`.json` were left behind.
- **Impact.** Parallel agents or users that rebuild at the same time crash and leave stray temp files. In other interleavings a file written by both processes can be installed.
- **Fix.** Take an exclusive `fcntl.flock` on `build_dir/.lock` (and `world_dir`) in `Builder.run` and `terrain.compose.build_all`. Add the pid or a uuid to temp names. Write PNGs to a temp file, then `os.replace`.

### CR-07 (low, dormant) `dtm_extract` vertical iteration depends on the chunk size
- **Evidence.** `tools/dtm_extract.py:420-436`: `shift = max|h| * sqrt(max d2) / R` and `if shift > shift_tol` are evaluated **per row chunk** (`--chunk-nodes`, default 2e6), so neighbouring chunks can disagree.
- **Reproduction** (`EXP/e4_dtm_chunk_iteration.py`). The production far command was run again with `--chunk-nodes 130100` (100 rows per chunk):
  - 1201 of 1301 rows differ by more than 1 mm, with a maximum of **74 mm** and an rms of 2.1 mm;
  - the chunk boundaries at y = 900 and 1100 m carry steps of 6-22 mm that the default output does not have.
- **Impact.** No effect today: far (1301^2) and horizon (1001^2) are single chunks, and ring and core never iterate (maximum shift 6 mm and 0.6 mm). Any extraction above 2 M nodes that iterates (a larger far field, a finer far spacing, another site) gets row seams and memory-knob-dependent heights.
- **Fix.** Decide once per extraction: compute the maximum |h| from the bounding-box sample or a coarse pre-pass, or always apply the iteration. Add a chunk-invariance test.

### CR-08 (low) Optional inputs missing
- **(a) Patched plugin not built.**
  - The default and inspect SDFs reference the absolute plugin path (correct: the launch falls back to `_stock`).
  - `check_sdf` counts the missing engine as a missing reference (`static_checks.py:1328`), so it fails and the build exits 2, although `_stock.sdf` is complete (E7: `fail`, only the engine path missing, stock 0 missing).
  - Fix: downgrade a missing patched engine to a warning when a valid `_stock` sibling exists, or let the world stage write engine `stock` into the default variant when the plugin is absent, and record that in world_info.
- **(b) Rock library missing.** `build --status` dies with an uncaught `FileNotFoundError` traceback, because the keys need the library digest. Catch it, and report "library missing".
- **(c) No `gz` CLI.** Without `gz` on PATH the world stage only logs a warning and caches an unvalidated SDF as `ok`. `/usr/bin/gz` exists here, so this does not bite on this machine. Put `sdf_validated` into the world key, or fail.

### CR-09 (low) `physics.rock_mu` is dead
- **Evidence.**
  - `config/apollo16_core.yaml:150` sets `rock_mu: 0.60`, but `build.py:1036` uses `ground_mu` for every `CollisionSpec`.
  - Rock collision proxies are baked into the ground tiles, and Bullet friction is per body.
  - `world_info.py:120` still advertises `rock_mu: 0.6`.
- **Reproduction** (E10c): the SDF has 108 collisions, all with mu 0.4.
- **Fix.** Either remove `rock_mu` from the config and world_info, or write rock proxies as separate per-tile collision meshes. `sdf.py` already puts other friction pairs on their own links, and the patched plugin applies them per body.

### CR-10 (low) Run script hangs on a missing option value
- **Evidence.** `scripts/run_apollo16_sim.sh:47-52`: `shift 2` / `shift 4` fail when too few arguments remain. Bash then leaves `$@` unchanged, and the `while` loop spins at 100 % CPU.
- **Reproduction** (`EXP/e3_run_script_args.sh`, which runs the parsing loop copied verbatim, nothing else). `--site`, `--world`, `--variant` and `--pose 1 2` are each killed by `timeout 5` (exit 124). `--pose 1 2 3` and `--site turn` exit 0.
- **Fix.** `[[ $# -ge 2 ]] || { echo "ERROR: $1 needs a value" >&2; exit 2; }` before each shift (4 for `--pose`).

### CR-11 (low, by inspection) Cache-key coverage gaps
- The following never enter a key, so edits leave stale outputs unless someone bumps `TERRAIN_VERSION` / `STAGE_VERSION`:
  - the terrain module source (`craters.py`, `fbm.py`, `compose.py`, `base.py`);
  - `grid.py`, `layout.py`, `config.py`, and `build.py` itself;
  - the numpy / scipy / pydelatin / numba versions.
- `RockLibrary.digest` hashes only `library.json` (`library.py:78`), not the LOD or hull npz files.
- Keys are input-based (`"upstream": key`), so `--force --stages meshes` re-runs meshes but never export.
- Base keys contain absolute DTM and tool paths, so moving the repo re-extracts the whole terrain. This costs time; nothing goes stale.
- **Fix.** Hash the terrain sources and the library versions into the keys. Hash the npz files (or their sizes and mtimes) into the library digest. Cascade `--force` to downstream stages, or include the upstream manifest's output digest in each key.

### CR-12 (low) Avoidable memory in the meshes stage
- **Evidence.**
  - `Mesh.__post_init__` coerces vertices to float64 (`mesh/types.py:38`), so the `dtype` argument of `rocks_mesh` has no effect (`merge.py:21`).
  - `rocks_mesh` also stores an int64 `-1` lattice (2 values per vertex, `merge.py:64`).
- **Measurement** (E10a, tile_04_05): 60 B per vertex (float64 vertices, int64 lattice, int64 triangles) against 32 B needed.
  - All 104 rock meshes are held at once: about 10.5 M vertices, roughly 0.6 GB instead of 0.34 GB.
  - Separately, `terrain.compose` process caches are never cleared by build.py. For example, the float64 coarse spline coefficients take about 205 MB.
- **Fix.**
  - Keep float32 when asked and set `lattice=None` for rocks; `write_glb` does not need it.
  - Store triangles as uint32.
  - Call `compose.clear_caches()` after the rocks stage.

### CR-13 (low) Test-quality gaps
- No test covers:
  - CR-01: a seed sweep against the budgets;
  - CR-02: path-only config changes;
  - CR-03: corrupted catalogs;
  - CR-04: a small `budgets.core_tile_visual_max_tris`;
  - CR-07: chunk invariance;
  - CR-10: missing option values (only `--help` and `bash -n` are tested).
- Rock "no floating" tests check contact against the placement ground, not the rendered TIN (CR-05).
- `test_world_validation.py:414` asserts `roughness status in ("pass", "fail")`, which is nearly vacuous.
- Otherwise the suite is strong:
  - exact lattice and bit-identity checks;
  - deliberate T-junction and gap detection;
  - brute-force crater superposition;
  - analytic hull tops;
  - SDF structure;
  - launch fallback logic.

## 3. Reproduce

```bash
E=/home/nargon/togo_assets/lunar/experiments/p3_review-code; PY=/home/nargon/.venvs/lunar_world/bin/python
cd $E && PYTHONDONTWRITEBYTECODE=1 $PY e1_cap_budget.py            # CR-04   (~5 s)
$PY e2_check_rocks_sanity.py   # CR-03 (~10 s)     bash e3_run_script_args.sh   # CR-10 (~20 s)
$PY e4_dtm_chunk_iteration.py  # CR-07 (~20 s)     $PY e5_rock_float.py         # CR-05 (~5 s)
$PY e6_world_dir_cache.py      # CR-02 (~25 s)     $PY e7_missing_plugin_and_library.py   # CR-08
$PY e9_rock_budget_seeds.py    # CR-01 (~15 s)     $PY e10_light_checks.py      # CR-09, CR-12, preset dirs
# CR-06: $PY e8_concurrent_build.py prepare; ($PY e8_concurrent_build.py A & $PY e8_concurrent_build.py B & wait); $PY e8_concurrent_build.py compare
```

## 4. Checked and found correct

- **Frames.**
  - ENU everywhere (x east, y north, z up).
  - Lighting: the sun direction from elevation/azimuth (north through east) matches the config to < 0.1 deg; `<direction>` is the propagation direction with an unrotated light pose.
  - Spawn yaw uses the ENU atan2 convention, as `ros_gz_sim create -Y` expects.
  - The dtm_extract curvature formula `z = (R+h) R / sqrt(R^2+d^2) - (R+href)` is right.
- **Z-up.** GLBs are raw Z-up with identity nodes. `static_checks._node_matrix` handles glTF (x, y, z, w) quaternions. The catalog uses (w, x, y, z), consistently in placement, merge, collision rasteriser and fillets. `rpy_matrix` is Rz Ry Rx.
- **Lattice.**
  - Tiles, strips and stitching compare integer 1 cm lattice keys (`_KEY_OFFSET` fits int64 to ±1e6).
  - Crop tolerances are robust.
  - Node coordinates are formed from integers in compose, fillets and craters, which gives bit-identical overlaps.
  - Horizon, far and ring vertices are exactly representable in float32 (20 m, 2 m and 0.25 m multiples); core vertices are within 3.8e-6 m.
  - UVs at 10 km are within 0.01 texel.
- **Numerics.**
  - pydelatin input is centred before the float32 cast (precision 3e-5 m at the horizon, against a 2 m tolerance).
  - Crater profile: no overflow at r -> 0; the smooth minimum is continuous at 2.5 R.
  - Fillet math has no division by zero.
  - The FFT filter normalisation `sqrt(S)/dx` gives exactly PSD S.
  - The B-spline sampler indices stay in bounds.
  - NaN guards: harmonic nodata fill, `tin_from_grid` and `write_glb` reject non-finite values.
- **Determinism.** Seeded per-component streams, parallel numba loops with disjoint writes, and deterministic reductions. The tests confirm it, and the integration agent's rebuild was byte-identical.
- **Physics patch and launch.**
  - The friction patch keys on (link id, collision name), and the velcmd patch sets only the relative axis component.
  - The launch falls back to `_stock` when the plugin path is missing, keeps the pose source next to the resolved world, and bounds every wait (600 s and 120 s).
  - The wrapper's `setsid` makes the PGID equal to `$!`, so stopping by process group works.

## 5. Not reviewed in depth

- `tools/bench_world.py`, `run_robot_tests.py`, `lidar_capture.py`, `validate/lidar_checks.py` and `report.py`: other agents edited them during this review.
- `blender/*` (Blender-side generation; only its conventions were checked).
- The Gazebo-side behaviour; no Gazebo was started.
