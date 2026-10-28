# Fix round: build, meshes, rock contact, validation tools (fix_build)

Owner: build fix agent, 2026-10-25, 16:08-18:10. Scope: `lunar_world_gen/build.py`, `mesh/*.py`
(new `mesh/refine.py`, `mesh/locate.py`), new `rocks/recontact.py`, `validate/static_checks.py`,
`world/{sdf,world_info}.py`, `tools/{bench_world,run_robot_tests}.py` (and `lidar_capture.py` through
`bench_world.GzServer`), tests `tests/test_world_build.py`, `tests/test_world_build_fixes.py` (new),
`tests/test_sim_gz_stop.py` (new), two assertions in `tests/test_world_sdf.py`. The config file was not
edited: every new key has a default in code, the test build used `--set`, and the proposals are in
section 9. Gazebo / ROS runs used `GZ_PARTITION=claude_fixbuild`, `ROS_DOMAIN_ID=93`, a private
`ROS_HOME`, `setsid`, coreutils `timeout`, and were stopped with SIGKILL (gz) as described in
section 7. Scratch: `/home/nargon/togo_assets/lunar/experiments/fix_build/` (`EXP` below; scripts in
`EXP/scripts` and in the session scratchpad). Test world: `/home/nargon/togo_assets/lunar/worlds/fix_build`,
build dir `/home/nargon/togo_assets/lunar/build/fix_build`.

Other agents edited terrain and rocks code during this round (TERRAIN_VERSION 3, talus, rock edge
band, burial). The test build therefore ran from a frozen copy of the package (`EXP/snap/B/pkg`,
taken 17:18, plus my `rocks/recontact.py` fix at 17:31, logged in `EXP/snap/B/PATCHES.txt`); my files
there are byte-identical to the repo.

## 1. Verdict per item

| item | reproduced? | result |
|---|---|---|
| RTF on slopes with the original robot collisions (task 1) | yes: slope RTF 0.13-0.17, climb 0.83-0.85 (2 runs) | conforming max-edge refinement implemented (`mesh/refine.py`, keys `mesh.collision.core_max_edge_m` / `ring_max_edge_m`). It raises slope RTF up to 3x (L = 0.5 m: 0.36-0.43, 0.24 with two foreign gz servers), but it **degrades the climb for both robot collision models** (L = 0.5 m: 0.34-0.73, fails 0.8; simplified robot 0.85 -> 0.61) and gives the default simplified robot no RTF gain. **Proposed: off (null)**, see 2 |
| CR-05 / FID-08 ring rocks hover above the rendered ring TIN | yes: 11,399 of 37,029 ring rocks hover > 1 mm, 2,045 > 20 mm, max 58.2 mm; 7,813 entirely below | fixed: new stages `ground` + `contact` re-seat rocks on the rendered TIN before merging and collision rasterisation: 0 hover, 0 hidden (section 3) |
| FID-10 core rocks vs the rendered TIN | yes: 480 core rocks hover > 1 mm (max 2.26 mm), 128 entirely below | fixed: 0 and 0; filleted rocks kept on H0 (2,382; max 0.32 mm above the TIN, 0 hidden) |
| FID-08 second part (ring rocks missing from the ring collision) | not addressed | owned by `rocks/collision_field.py` (rocks agent) |
| CR-01 hard rock budget at the mean | yes (review E9 data; 3 of 8 seeds > 6.6 M) | soft `*_warn_tris` vs hard `*_max_tris`; proposed hard 7.5 M rocks / 20 M visual, warnings 6.6 M / 16 M; regression test with the 8 seed totals |
| CR-02 caches ignore world_dir; fast preset shares dirs | yes (scenario test) | output paths in keys of `contact`/`export`/`world`; recorded outputs outside the current dirs make a stage stale; non-default presets build into `<dir>_<preset>`; exit 2 when a check did not run (`all_ran`). E6 rerun: W2 fully written, 8 checks ran |
| CR-03 check_rocks passes corrupted catalogs | yes (review E2 cases) | collide flags, quaternion norms, core/ring D_min (core-edge band exempt), duplicate ids, zone, proto/LOD ranges judged; stitching fails when pieces are missing |
| CR-04 tile cap vs hard check | yes (cap 2000 -> 2013 after stitching) | cap applied at 97 % (pre-stitch), hard limit judged after stitching; mini build with every tile capped passes |
| CR-06 no build lock, fixed temp names | yes (review E8) | `fcntl.flock` on `build_dir/.build.lock` and `world_dir/.build.lock`; pid+random temp names for JSON, npy, npz, GLB, SDF, world_info, catalog, PNG. E8 rerun: one build completes, the other fails at once with the holder's pid; GLBs byte-identical to a sequential build, no temp leftovers |
| CR-08a missing patched plugin fails validation | yes (review E7) | missing engine with a complete stock sibling (`_stock`, `_inspect_stock`) = warning, else fail |
| CR-08b `--status` traceback without library | yes | `--status` reports "input missing: rock library not found" per stage, exit 0 |
| CR-08c no `gz` CLI caches an unvalidated SDF | by inspection | `sdf_validator` (gz on PATH) is part of the world key; `sdf_validated` in world_info |
| CR-09 `physics.rock_mu` never applied | yes: 108 collisions, all mu 0.4 | removed from world_info (`friction_note`); propose deleting the key |
| CR-11 cache-key gaps | by inspection + tests | keys hold foundation code, terrain code (`terrain/build_record.json`), numpy/scipy/numba/pydelatin/Pillow/PyYAML versions, every rock-library file (size, mtime), upstream run ids; `--force-downstream`, `--adopt` |
| CR-12 float64/int64 rock meshes | yes: +812 MB peak for the merge | float32 vertices kept by `Mesh`, `lattice=None`, one tile at a time: +28 MB peak (real catalog, 11.1 M vertices) |
| CR-13 tests | | 35 new tests (section 8) |
| FID-07 seam creases | measured at 3 mm: seam excess crease 1.26x interior edges | optional seam-band refinement; proposed 8 cm edges within 6 cm of seams: seams at the interior level, +0.33 M triangles (section 4) |
| U11 inspect lighting lost in the stock fallback | | fourth variant `apollo16_core_inspect_stock.sdf` (stock physics, inspection lighting); the runtime agent's launch already falls back to it |
| task 6 SIGKILL in the tools | | gz is never sent SIGTERM/SIGINT by `bench_world`, `lidar_capture`, `run_robot_tests` (section 7); 0 crash dumps from my runs |

## 2. Collision max-edge refinement and the slope RTF

**Implementation** (`mesh/refine.py`, used by `make_piece(..., max_edge_m=, max_edge_band_m=)`):
rounds of Rivara longest-edge bisection with closure; new vertices are the edge midpoints snapped to
grid nodes (so they carry exact grid heights and the 1 cm lattice), shared by both triangles of an
edge (conforming); boundary vertices stay on the rectangle side; every sub-triangle is checked with
exact integer orientation tests and a snapped vertex that would invert one is moved to its next
candidate. pydelatin leaves one-node-wide strips along tile sides (vertices at i = 1 next to i = 0)
where no grid node lies strictly inside a sliver; generic split templates plus Lawson flips of long
unsplittable edges (convex quad, shorter diagonal) resolve them. On real tiles: 0-2 edges per tile
remain marginally over the limit, error stays 15-16 mm (tolerance 15 mm), fewer slivers than the
input (triangles < 5 deg: 4.8 % -> 2.5 %). After refinement the build stitches and `check_stitched`
proves the result as before (Euler 1, 0 T-junctions).

**Measurement.** Diagnostic worlds = the phase-3 world's visuals with the collision rebuilt from its
own 2 cm collision grids exactly like the meshes stage (`EXP/scripts/x10_make_col_worlds.py`; the
unrefined rebuild reproduces the shipped counts 487,217 / 901,982 exactly). Slope site (11.0, -4.0),
climb 0.4 m/s for 15 s, one launch per run, variants interleaved (`run_slope_series.sh`,
`EXP/rtf/s1|s2simp|s2orig`). Other agents ran Gazebo and builds meanwhile ("other gz" = foreign gz
servers during the drive), so RTF is indicative; the climb outcome is physics.

| collision (core) | core / total collision tris | slope-area boxes > 8 cm | robot | RTF drive | RTF climb | climb distance (>= 0.8) | climb angle | airborne |
|---|---|---|---|---|---|---|---|---|
| shipped TIN (1.5 cm) | 487,217 / 901,982 | 14.7 % | original | 0.165, 0.141 | 0.132, 0.113 | **0.853, 0.827** | 11.4, 11.5 deg | 0.9 % |
| + max edge 0.75 m | 530,332 / 945,494 | 13.5 % | original | 0.218 (1 other gz) | 0.188 | 0.806 | 10.7 | 1.2 % |
| + max edge 0.5 m | 678,377 / 1,093,864 | 8.9 % | original | 0.432, 0.359, 0.237 (0/1/2 other gz) | 0.441, 0.359, 0.236 | **0.728, 0.339 (stall), 0.714** | 11.2, 9.3, 11.0 | 2.1-4.5 % |
| + max edge 0.3 m | 1,202,132 / 1,618,312 | 2.7 % | original | 0.472 | 0.563 | 0.650 | 10.0 | 6.9 % |
| shipped TIN | | | simplified | 0.498 (1 other gz) | 0.456 | 0.851 | 11.4 | 0.5 % |
| + max edge 0.5 m | | | simplified | 0.481 | 0.506 | 0.614 | 9.9 | 3.2 % |

(max edge 1.0 m: +9 k triangles, box heights unchanged; not run. One more 0.3 m launch was invalid:
the drive tool got no odometry within 30 s wall.)

- RTF with the original CAD collisions rises 3x at 0.5 m and 3.5x at 0.3 m, far from 0.9: rock bumps
  against the 46.6 k-triangle motor meshes remain (as the 10 cm grid test of gz_validation 5.1 found).
- The climb gets worse as the triangles get smaller, for both robot collision models; slip ratio
  0.14-0.17 -> 0.27-0.66. The geometry is not the cause: along both wheel tracks the refined and
  unrefined surfaces have the same relief (detrended RMS 35 mm, 10 cm second difference 9 mm,
  `x13_path_relief.py`), while gz_validation's regular 10 cm grid that climbed well was 3x smoother
  (it under-samples the rock bumps). It is the classic-Bullet wheel contact on many small mesh
  triangles (DESIGN 7 already measured more wheel jitter on small triangles); airborne fraction
  0.9 % -> 2-7 %.
- With the simplified robot (the runtime agent's default) the refinement brings no RTF gain.

**Decision: `mesh.collision.core_max_edge_m: null` and `ring_max_edge_m: null`** (the ring would
need ~1.5-4 M triangles anyway). The option stays for users who keep the original collisions and
prefer RTF over climbing fidelity: 0.5 m costs +191 k core triangles (total 1.09 M collision).

## 3. Rocks on the rendered ground (CR-05, FID-08, FID-10)

`rocks/recontact.py`: `recontact(cfg, catalog, library, height_fn, rows=None)` recomputes `z` and
`exposed_h` with the placement contact rule (lowest visual-LOD vertex touches the surface, then
`burial x vertical extent`); with the placement ground it reproduces the placement heights to 1e-12
(test). `mesh/locate.py` (`TinLocator`, moved from `static_checks`, and `PiecewiseTinHeight`) gives
the exact height of the stitched visual pieces. Rows are processed in 10 m (core) / 50 m (ring) cells
so the locator cache works (a first version grouped by prototype and took > 6 min; now 23 s per pass).

New stage order: `terrain, rocks (placed on H0), heights (fillets), ground (rendered visual TINs,
stitched), contact (re-seat; final catalog to world_dir), collision (rock tops from the final
catalog), meshes (collision TINs, rock meshes, budgets), export, world, validate`. Rocks with a fillet
keep their H0 placement (their fillet is part of the rendered ground; re-seating would lift them onto
it). Config key `rocks.contact_on_rendered_ground` (default true).

Full test build (3 mm core TIN):

| rows | n | hover > 1 mm before -> after | > 20 mm | max hover | entirely below the TIN | z change p1 / p50 / p99 |
|---|---|---|---|---|---|---|
| core, re-seated | 213,295 | 480 -> 0 | 0 -> 0 | 2.26 mm -> touching | 128 -> 0 | -1.9 / +0.2 / +2.4 mm |
| core, filleted (kept) | 2,382 | 0 | 0 | 0.32 mm | 0 | 0 |
| ring (incl. 33,751 core-edge band rocks) | 37,029 | 11,399 -> 0 | 2,045 -> 0 | 58.2 mm -> touching | 7,813 -> 0 | -35.5 / +1.5 / +37.2 mm |

## 4. FID-07: seam creases at 3 mm

Metric (`x20_seam_crease.py`, 3 x 3 real tiles, stitched): second difference of the TIN across every
edge at its midpoint minus that of the true surface H at the same points ("excess crease"), for seam
edges and interior edges. A seam is a straight chain of edges, so comparing it with lines through
planar facets (the review's 1.92 vs 0.38 mm) overstates it; per edge:

| core visual TIN | triangles (3 x 3 tiles) | h = 1 cm seam / interior (RMS, p99) | h = 2 cm | h = 5 cm |
|---|---|---|---|---|
| 5 mm | 159,624 | 1.94, 6.0 / 1.37, 3.4 mm | 2.92 / 2.52 | 4.19 / 4.04 |
| 3 mm | 428,428 | 1.60, 4.5 / 1.27, 3.1 | 2.33 / 2.10 | 2.90 / 2.71 |
| 3 mm + seam band 5 cm edges in 10 cm | 540,336 | 1.00, 2.7 / 1.21, 3.0 | 1.40 / 2.01 | 1.53 / 2.58 |
| **3 mm + seam band 8 cm edges in 6 cm** | 455,736 (+6.4 %) | 1.21, 3.5 / 1.25, 3.1 | 1.79 / 2.07 | 2.15 / 2.67 |
| 3 mm + seam band 12 cm in 4 cm | 434,191 | 1.45, 4.0 / 1.26, 3.1 | 2.10 / 2.10 | 2.67 / 2.70 |

Shading normals of shared seam vertices are identical (0 mismatches). At 3 mm the seams are 1.1-1.3x
worse than ordinary edges; the 8 cm / 6 cm band removes the excess for +0.33 M core triangles
(full build: 4.72 -> 5.05 M) and about +0.65 s per tile of build time. Keys
`mesh.core_visual_seam_max_edge_m`, `mesh.core_visual_seam_band_m` (default off).

## 5. Full test build (`EXP/scripts/full_build.sh`, snapshot B)

`--set mesh.core_visual_max_error_m=0.003 mesh.core_visual_seam_max_edge_m=0.08
mesh.core_visual_seam_band_m=0.06` and the budget keys of section 9; collision refinement off.

| stage | s | peak GB | notes |
|---|---|---|---|
| terrain | ~194 | 2.37 (+2.40 children) | rebuilt (TERRAIN_VERSION 3) |
| rocks | 36.4 | 1.11 | 215,677 core + 37,029 ring rocks (rocks agent's edge band) |
| heights | 9.0 | 0.73 | |
| ground | 147.8 | 1.22 | core TIN + normals 126 s (3 mm + seam band); stitch 12 s |
| contact | 77.2 | 0.48 | 3 passes of 23-25 s |
| collision | 2.9 | 0.69 | |
| meshes | 9.4 | 0.58 | rock merge 4.2 s |
| export | 14.3 | 1.67 | |
| world | 16.9 | 0.64 | 4 variants `gz sdf -k` valid, sites flat (-2.0, -4.5), slope (11.0, -5.0), rock, turn |
| validate | 100.8 | 1.79 | |

| counts | value | budget (proposed) |
|---|---|---|
| core ground | 5,053,546 (per tile 47,016-55,165; seam band +333,417) | 10 M hard, tile 200 k (none capped) |
| core rocks | 6,583,546 | 7.5 M hard, 6.6 M warn (ok) |
| ring ground / rocks | 183,519 / 1,488,276 | |
| far / horizon ground | 80,419 / 110,606 | |
| **visual total** | **13,499,912** | 20 M hard, 16 M warn (ok) |
| collision core / ring / far | 460,146 / 411,472 / 26,488 | core 1 M hard |
| **collision total** | **898,106** | 1.5 M hard, 1.0 M warn (ok) |

Static checks (all 8 ran): craters, rocks, visual-vs-collision (dz median -20.2 mm, RMS 5.7, p99
14.4), GLB (324 files, 0 problems, 100 % up normals), stitching (112 visual + 108 collision pieces,
complete), SDF (4 variants, 0 missing), world_info pass; **roughness fails at 0.3 m (ratio 1.27,
limit 1.25)**: judged on the terrain's H0 tiles, unrelated to my changes (phase 3: 1.24); for the
terrain owner. Build exit code 2 accordingly.

Gazebo (indicative):

- Quick load, world only (`x40_load_test.py`): first stats 2.27 s, physics stepping at 4.48 s, RTF
  0.987, RSS 378 MB, 0 `[Err]`/`[Wrn]`, 108 patched mesh collisions, stopped by SIGKILL, 0 left.
- LiDAR bench, flat site, `real` preset, 40 s window, contended by another agent's full robot
  simulation (their gz at 172 % CPU, 5.7 busy cores): first cloud **32.6 s** (> 30 s limit), RTF
  0.968, peak RSS 3.34 GB, D3D12, 0 errors. A/B against the phase-3 world: see section 6.

## 6. A/B load bench and final slope drives

LiDAR bench back to back (`gz_final2.sh`, `real` preset at each world's flat site, 30 s window):

| world | visual tris (vertices) | first cloud | render init | RSS at load / peak | RTF | load on the machine |
|---|---|---|---|---|---|---|
| phase-3 `apollo16_core` | 9.53 M (12.2 M) | 27.1 s | 4.3 s | 2.64 / 2.71 GB | 0.989 | 2 foreign gz servers, 6.1 busy cores |
| `fix_build` (3 mm + seam band + edge-band rocks) | 13.50 M (15.0 M) | 28.3 s | 4.0 s | 3.19 / 3.32 GB | 0.988 | 2.7 busy cores |
| `fix_build`, first run (section 5) | | 32.6 s | | 3.20 / 3.34 GB | 0.968 | foreign robot sim, 5.7 busy cores |

The phase-3 world loaded in 19.6-21.1 s on a quiet machine (gz_validation 3); the new world has 23 %
more vertices, so expect roughly 24-26 s quiet, inside `validation.load_time_max_s` 30 s but with
little margin under load; RSS +0.6 GB (limit 5 GB). 0 `[Err]` in every run.

Slope drives on the final `fix_build` world (no collision refinement; slope site (11.0, -5.0)),
both runs uncontended (no foreign gz server):

| robot collisions | RTF drive | RTF climb | climb distance | climb angle | airborne | result |
|---|---|---|---|---|---|---|
| original (CAD meshes) | 0.183 | 0.158 | 0.890 | 10.4 deg | 0.7 % | pass (RTF still low) |
| simplified (launch default) | **0.901** | 0.886 | **0.900** | 10.4 deg | 0.5 % | pass |

The robot-side simplification (runtime agent) is the RTF fix; the world change for the original
collisions (section 2) is not recommended.

## 7. Stopping Gazebo without crash dumps (task 6)

- `bench_world.GzServer`: `timeout -s KILL LIMIT gz sim ...` (no `--foreground`: at the limit the
  whole group gets SIGKILL); `stop()` sends SIGKILL to the group directly. `lidar_capture.py` and
  `render_views.py` use it.
- `run_robot_tests.Launch`: `timeout -s KILL`; `stop()` SIGKILLs the gz processes of the launch group
  (`bench_world.gz_pids`: process name gz/ruby*, or `gz sim` command line), waits until they are gone,
  lets ros2 launch stop its nodes (`on_exit_shutdown` / the runtime agent's gz guard), then SIGINT to
  the remaining ROS processes (only after gz is gone) and SIGKILL after a grace period. Process deaths
  after the stop request are no longer reported as failures. The drive tool keeps its SIGINT stop.
- `tests/test_sim_gz_stop.py` runs fake `gz` processes that record SIGTERM/SIGINT: none received; the
  fake ROS node gets its graceful SIGINT.
- All my launches/benches: gz stopped by SIGKILL, 0 processes left; `%TEMP%\wsl-crashes` held no dump
  from my runs (checked 16:31, 17:36 and at the end).

## 8. Tests

`pytest tests` on snapshot B (whole package, my final files): 259 passed, 2 skipped, 3 failed in
163 s; the failures are other agents' in-progress tests (`test_rocks_fixes::test_core_edge_band`,
`test_runtime_launch::test_simplify_collisions_frames_and_scope`,
`test_runtime_script::test_env_sh_sets_the_sim_environment`). Live tree at 18:05: `test_fix_build`,
`test_tools_stop`, `test_world`, `test_mesh`, `test_validation`, `test_build`: 117 passed in 122 s.
While others edit `rocks/*.py` or `terrain/*.py`, a running `test_world_build.py` can see a stage turn stale
mid-session (the keys hash that code, by design); rerun when the tree is quiet.

New / changed: `test_world_build.py` 11 tests (10-stage build, contact results, inspect_stock, no rock_mu,
selective rebuilds incl. a collision-only change, forced upstream invalidates downstream +
`--force-downstream`, world-dir change (CR-02), preset dirs, every tile capped passes (CR-04));
`test_world_build_fixes.py` 28 tests (refinement contract at 0.15/0.3/0.6 m, one-node boundary strips,
stitching after refinement, seam band, `PiecewiseTinHeight`, recontact reproduces placement and
removes hovering, fillet rows, soft/hard budgets and the 8 seed totals, cap headroom, corrupted
catalogs (5 cases + clean), core-edge band, `check_sdf` fallback, `all_ran`, `--status` without
library, build lock, unique temp names, key coverage, float32 rocks, inspect_stock + world_info);
`test_sim_gz_stop.py` 3 tests; `test_world_sdf.py` two variant assertions.

## 9. Proposed config (not applied)

```yaml
mesh:
  core_visual_max_error_m: 0.003       # orchestrator decision, verified (core ground 4.72 M + seam band)
  core_visual_seam_max_edge_m: 0.08    # FID-07: seam band refinement
  core_visual_seam_band_m: 0.06
  collision:
    core_max_edge_m: null              # refinement implemented but OFF: worse climbing (section 2)
    ring_max_edge_m: null
budgets:
  core_rocks_max_tris: 7500000         # hard (was 6.6 M at the seed mean)
  core_rocks_warn_tris: 6600000
  visual_total_max_tris: 20000000      # hard cap of DESIGN 13a
  visual_total_warn_tris: 16000000     # DESIGN 13a target
  collision_total_max_tris: 1500000
  collision_total_warn_tris: 1000000
physics:
  rock_mu: (delete)                    # never applied: rocks are baked into the ground collision
rocks:
  contact_on_rendered_ground: true     # default in code; may be omitted
```

## 10. Notes for other owners

- Stage list changed (`ground`, `contact` added; visual meshes moved out of `meshes`); the placed
  catalog is `build_dir/rocks/rocks_catalog.npz`, the final one `build_dir/contact/` and
  `world_dir/rocks_catalog.npz`. integration.md sections 2-3 describe the old layout.
- `--preset fast` now builds into `..._fast` dirs unless paths are set.
- `validate`: a missing patched plugin with a stock sibling is a warning; ring rocks below the
  ring `D_min` are accepted inside `rocks.edge_band.width_m` of the core (rocks agent's band).
- Roughness at 0.3 m fails in the current terrain (1.27).
- Load time at 13.5 M visual triangles is around the 30 s limit (section 6).
- Not addressed (not in my task list): the flat-site corridor grade (gz_validation 5.2); FID-08's ring
  collision capture of 30-50 cm rocks.
