# Apollo 16 lunar world: design and module contracts

This spec defines how the generator builds the world and the contracts between its modules. Every module must follow it. The numbers come from the research reports in `docs/research/`; parameter values live in `config/apollo16_core.yaml`. If an implementation finds a contract unworkable, write the reason into the module's docstring and the progress log rather than silently diverging.

**As built** notes mark where the final world (build of 2026-10-25, `docs/impl/final_build.md`) differs from the original spec, mostly through the fix round (`docs/impl/fix_*.md`). Where a note and the original text disagree, the note describes the code.

## 1. Goals and constraints

- **The world.** An Apollo 16 landing-site world for LiDAR-based rover navigation in Gazebo Harmonic (gz-sim 8.11) on WSL2. It consists of:
  - a 100 x 100 m core with about 1 cm geometric detail and real pebbles (2 cm and larger) visible to `gpu_lidar`;
  - a 500 m LiDAR ring and a 2.2 km far field built from the real 2 m DTM;
  - a 20 km horizon;
  - lunar gravity, realistic lighting, and believable visuals for later camera work.
- **Real time.** The world must run at about real-time factor 1 with the rover and its GPU LiDAR, and load quickly enough for the existing launch chain (see 12).
- **Hard rules.**
  - No git commands.
  - Never modify existing files outside `src/lunar_world_gen/`. (As built, one exception at the user's request: the `MESA_EXTENSION_OVERRIDE` line in `src/togo_sim_wsl.sh`, the WSL memory-leak fix of section 12.)
  - Generated assets go under `~/togo_assets/lunar/`.
  - The sim LiDAR stays as it is.
  - The robot description is not edited.

## 2. Frames, lattice, zones, tiles

- **Frame.** Local ENU tangent plane at the core centre (site `core`: lat -8.973448, lon 15.511126): x = east, y = north, z = up, in metres. `z = 0` is at `href = 8.1955 m` above the 1737.4 km sphere, and Moon curvature is included. `tools/dtm_extract.py` implements exactly this mapping.
- **Grids.** `lunar_world_gen.grid.Grid` is node-centred, includes both edges, and has row 0 at the south edge: `z[j, i]` sits at `(x0 + i*s, y0 + j*s)`.
- **Global lattice.** Every grid origin and spacing in every zone is an integer multiple of `LATTICE_M = 0.01 m`. Vertex positions are compared through integer lattice coordinates (`grid.to_lattice`), never through float equality.
- **Zones.** Each is a square centred on the origin:

| zone | size | grid | visual source | LiDAR relevance |
|---|---|---|---|---|
| core | 100 m | 1 cm | band-limited DTM + coarse + fine synthetic detail + rocks >= 2 cm | everything |
| ring | 500 m | 25 cm | band-limited DTM + coarse synthetic detail (>= 0.5 m) + craters >= 1 m + rocks >= 30 cm | everything within 150 m of the core |
| far | 2.2 km | 2 m | band-limited DTM (deseamed) | camera / GUI only |
| horizon | 20 km | 20 m | DTM block-averaged | camera / GUI only |

- **As built (FID-04).** Rocks of 2-30 cm also continue 6 m beyond the core edge, thinning out with a raised cosine (`rocks.edge_band`, section 5), so the rock field has no cliff at the core boundary. They belong to the ring zone in the catalog and have no collision.
- **Core tiles.** The core is split into 10 x 10 tiles of 10 m (`layout.TileId(i, j)`, i from west to east, j from south to north, named `tile_ii_jj`).
- **Outer zones.** Each outer zone is four strips around the next inner square (`Layout.strip_rects`): south and north strips span the full width, and west and east strips fill the band between them.

## 3. The height function

One deterministic height function defines the visual ground everywhere:

```
H(x, y) = B(x, y)                                   band-limited DTM (Butterworth, lambda_c = 10 m, order 4)
        + w_ring(x, y) * [ F_coarse(x, y) + K_big(x, y) + T(x, y) ]
        + w_core(x, y) * [ F_fine(x, y)   + K_small(x, y) ]
        + fillets(x, y)                              core only, added after rock placement
```

H0 is H without the fillets. The talus term `T` was added in the fix round (FID-03, below).

- **`B`** comes from `tools/dtm_extract.py`: `--site core --lowpass-lambda 10` with cubic B-spline resampling. Grids extracted at different spacings agree at common nodes; the site report measured differences under 0.1 mm. B is extracted at:
  - core: 1 cm over core + margin, 101 m;
  - ring: 25 cm over 500 m;
  - far: 2 m over 2.2 km with `--deseam default`;
  - horizon: 20 m.
- **`F_coarse`** is isotropic fBm with wavelengths from `split_wavelength_m` (0.5 m) up to infinity, multiplied by the DTM power-complementary weight `W(k) = (k lambda_c)^4 / sqrt(1 + (k lambda_c)^8)`. That weight keeps out the wavelengths the DTM already contains. The field is synthesised once on a 10 cm grid covering the whole ring, and every zone samples it with cubic interpolation (`Grid.sample(order=3)`).
- **`F_fine`** is fBm with wavelengths from `min_wavelength_m` (2 cm) to 0.5 m. It is synthesised once with an FFT on a 1 cm grid covering core + margin (non-periodic: synthesise on a larger padded domain and crop), and sampled by node index.
  - Both fBm fields use the PSD `S(f) = S0 f^-(2+2H)`, with `S0 = psd_S0_m4 * gain`, `H = 0.8`, and f in cycles per metre. The gain is `terrain.roughness.fbm_gain` for F_coarse and `fbm_gain_fine` for F_fine.
  - **As built (FID-06): two gains.** One shared gain could not meet every site-aware criterion of section 13 (the best single gain, 0.222, misses 3 lags), so each band has its own: coarse 0.109, fine 0.578. The fields are cached at unit gain and scaled by sqrt(gain).
  - Split the coarse and fine bands with complementary sharp masks in the frequency domain, so they sum to the full band.
  - The FFT synthesis must give the same bits for any thread count (as built: 1-D passes over fixed 64-row blocks). The talus thresholds amplify differences of 1e-8 m to centimetres.
- **`K_big` / `K_small`** are the crater relief functions, section 4. `K_big` holds craters >= 1 m over the ring rectangle; `K_small` holds craters from 0.2 m to 1 m over the core.
- **`T`**, the talus (angle-of-repose) correction (`terrain/talus.py`, FID-03). Without it, compound craters on the DTM's own pits reached 47.6 deg at a 0.5 m baseline; walls steeper than the static angle of repose (about 35 deg) would slump.
  - Input: Hs = B + w_ring (F_coarse + K_big) + w_core K_small on the fixed 10 cm lattice of the coarse fBm field (ring square + 2 m). That is everything the 10 cm grid resolves; F_fine is excluded because the grid would only alias it.
  - Relaxation: mass-conserving explicit nonlinear diffusion (the flux across an edge is computed identically from both sides), run in windows around steep edges until every edge gradient is at most tan(`terrain.talus.max_slope_deg`). Walls slump: rims lose height, floors gain it.
  - Sampling: Catmull-Rom, exact at the 10 cm nodes, C1, and exactly 0 wherever T and its neighbours are 0, so H0 is bit-identical to the unlimited composition away from relaxed walls and every zone samples the same values.
  - Limit 32 deg: the largest limit that keeps the maximum slope at the 0.5 m baseline under 35 deg. As built: 218 windows, 74,468 changed nodes, max |T| 0.47 m; steepest core slope at 0.5 m 33.5 deg, 0 m^2 above 35 deg.
- **Fades.**
  - `w_core` = 1 inside the core, falling smoothly (raised cosine) to 0 over the last `core.edge_fade_m` inside the core boundary, so H on the core boundary equals the ring's H. **As built: 3 m** (was 1 m; FID-04: the 10-90 % transition widened from 0.8 to 2.5 m).
  - `w_ring` = 1 inside, falling to 0 over the last `ring.edge_fade_m` (20 m) inside the ring boundary, so H on the ring's outer boundary equals B and the far field.
- **Cross-zone consistency.** Any zone evaluating H at the same lattice point must get the same value to 0.1 mm. B differs between extractions by less than 0.1 mm; F_coarse, K and T are evaluated with the same functions and sample positions. Stitching (6) still makes boundary vertices bit-identical. As built: every zone boundary is bit-identical (0.0 m).
- **Calibration.** The terrain stage measures the plane-detrended directional deviogram `nu(L)` of core H, without rocks, at L = 0.01, 0.03, 0.1, 0.3, 1, 3 and 10 m, and records the result.
  - **As built: a two-gain, site-aware fit** (`python -m lunar_world_gen.terrain.calibrate [--apply]`, `fit_gains`). H0 = B + K + T + s_c Fc + s_f Ff is linear in the two band scales, so one pass over the five component fields gives nu(L) for every gain pair. The fit maximises the number of section 13 criteria met with a 0.02 margin, then minimises the sum of ln(ratio)^2. T follows the coarse gain, so the fit is repeated until the gains are stable.
  - The gains are calibrated together with the core fade, the talus limit and the crater shapes; changing any of these needs a recalibration.
  - Report the shortfall if no gain pair fits all criteria. As built: all pass; the composite reaches 0.93 / 1.03 of the target at 1 / 3 cm and 1.23 at 0.3 m (B + K + T alone are already 1.19 there).

## 4. Craters

- **Population.** `N(>=D) = C D^-b`, with C = 0.05 per m^2 and b = 2.
  - Sample D by inverse transform between `D_min` and `D_max`.
  - Centres are uniform over the zone rectangle plus a margin of 2.5 * D_max / 2, so craters near the edge still contribute.
  - Craters with D between 8 and 16 m are kept with probability falling linearly from 1 to 0, because the band-limited DTM already resolves them.
- **Freshness.** `rho = exp(-u)`, u ~ U(0, 3), gives about 10% fresh, 20% intermediate and 70% degraded.
  - Depth: `d = rho * 0.13 * D`.
  - Rim height: `h = rho * 0.03 * D`.
  - Degraded craters are slightly wider: `R_eff = R * (1 + 0.15 * (1 - rho))`.
- **Profile.** Continuous, C1 except at the rounded crest. r is the distance to the centre and R = D/2:
  - interior: `z_in(r) = -d + (d + h) * (r/R)^n`, with `n = 1 + rho` (paraboloid when fresh, cone-like when degraded);
  - exterior: `z_out(r) = h * ((r/R)^-3 - q) / (1 - q)` for `R < r < 2.5 R`, and 0 beyond, with `q = 2.5^-3`;
  - combined: `z = smin(z_in, z_out, k)`, a polynomial smooth minimum with `k = 2 * h + 1e-4`. This rounds the crest automatically, because z_in is the smaller branch inside the rim and z_out outside it.
  - **As built (FID-05): crest-exact rims.** With k = 2h the smooth minimum lowered the crest to 0.56-0.69 h. The code uses `k = rim_rounding_k_over_h * hp` (0.5) and solves the rim parameter hp per crater (numba bisection on the analytic crest location) so that the rounded crest equals h exactly; hp / h = 1.08-1.23, the crest sits at 1.02-1.03 R, and fresh rims reach 0.030 D. Crater files of version 1 (k = 2h, circular) still load.
- **Shape variation (as built, FID-16).** Each crater draws, from its own random stream (so the population draws are unchanged): depth x U(0.85, 1.15), rim height x U(0.8, 1.2), profile exponent n = 1 + rho U(0.9, 1.1), an elliptical outline (axis ratio U(0.85, 1), random orientation) and an azimuthal rim modulation 1 + a2 cos 2(az - p2) + a3 cos 3(az - p3) with a2 <= 0.2, a3 <= 0.1 (`terrain.craters.shape`). The azimuthal mean crest stays h.
- **Composition.** Craters are superposed by summation (`K = sum z_i`). This is pure superposition, so each crater contributes the same at every resolution. That is required for cross-zone consistency.
- **Implementation.** Use numba with a spatial bucket index: evaluate only the craters whose 2.5 R disc touches the queried block. Store the crater set as `.npz` with x, y, D, rho, d, h and an id.

## 5. Rocks

- **Library** (Blender, `blender/`, output in `cfg.paths.rock_library`):
  - Prototypes in three classes: about 20 pebbles, 20 medium and 10 large.
  - Each prototype has LOD0, LOD1 and LOD2 meshes.
    - LOD0 is the class budget: pebble 12-40 triangles, medium 200-1000, large 2000-5000.
    - LOD1 is about 30% of LOD0.
    - LOD2 is about 10% of LOD0, with at least 12 triangles.
  - Each prototype also has a convex hull of at most 64 vertices.
  - All rocks share one texture atlas: basecolor, normal and ORM PNGs.
  - Files: `library.json` (prototype list, class, dims_m [a, b, c], bottom_offset_m, LOD triangle counts, atlas regions, file paths), per-LOD `.npz` (Z-up metres: `positions (n,3) f4`, `normals (n,3) f4`, `uv (n,2) f4` with atlas UVs in glTF convention, v down, `indices (m,3) u4`), `hulls/*.npz`, and `atlas/*.png`.
  - The production run is deterministic per seed.
- **Population** (`rocks/population.py`):
  - Sizes come from the broken power law (config `rocks.sfd`). The core holds rocks from 2 cm to 3 m; the ring holds rocks of 30 cm and larger.
  - Positions are Poisson-distributed, modulated by a log-normal density field (sigma_ln 0.6, correlation 35 m) and by enhancement annuli around fresh craters of at least 5 m (factor 2 in 1-2 R).
  - **As built (FID-02): smooth, size-gated crater enhancement** (`rocks.crater_enhancement`, `profile: smooth`). The hard x2 annuli made visible rings of rocks. Now e(x, y) = 1 + max over fresh craters of A p(r / R), never stacking; p rises from 0.5 at the centre to 1 at the rim and follows the ejecta-thickness law to 0 at 2.5 R. Debris below 10 cm gets at most 1.5x around fresh craters of 5 m and more; rocks of 10 cm and larger are boosted only around blocky craters (weight ramping from 20 to 80 m). Each size band is normalised to mean 1 over the core, so the core totals stay nominal. `profile: annulus` keeps the old rule.
  - Rocks of 10 cm and larger are rejection-sampled against each other (`min_separation_factor`). Pebbles may overlap.
  - **As built (FID-09): overlap repair** (`rocks.overlap`). After placement every pair whose bounding discs overlap is tested on the real geometry; the smaller rock is re-drawn and placed again when its centre lies inside the larger rock's footprint, or when the larger rock is 5 cm or more and either rock pierces the other's hull. As built: 0 such pairs (1,520 before); pebble-pebble contact stays allowed.
  - **As built (FID-04): core-edge band** (`rocks.edge_band`). Rocks from the core's D_min to the ring's (2-30 cm) are also sampled outside the core with the core intensity times a raised cosine of the distance from the core edge, 1 at the edge and 0 at 6 m. They are ring rows (zone 1, `collide` false, pebbles at LOD1). As built: 33,751 rocks, 0.53 M triangles.
- **Placement** (`rocks/placement.py`):
  - Choose a size class by D (config `rocks.classes`) and a prototype uniformly within the class library.
  - Uniform scale `s = D / a_proto`, random yaw, and tilt equal to the local ground normal (plane fitted under the footprint) plus a random tilt of up to 15 degrees.
  - **As built (FID-01): rocks rest on a stable face.** Rocks placed with the ground-normal tilt often stood on one vertex with a gap underneath. Each rock now rests on a stable facet of its hull (tipping margin at least 3 deg, chosen by solid angle and flatness, `rocks.rest_face`), is settled until three LOD vertices touch the ground with the centre of mass above their triangle (`rocks.settle`), and only buried rocks then get a random tilt of up to `rocks.tilt_max_deg` (8 deg).
  - Burial fraction from Beta(1.5, 6) mapped into the class range. A `perched_fraction` of the rocks get 0.
  - **As built (FID-12): class ranges [0, 1]**, i.e. Beta(1.5, 6) directly (the calibration's young-debris burial). Perched rocks (30 %) are bedded, not floating: a compliance burial of 2-6 % of their height, more for keel-shaped facets, capped at 18 % (`rocks.perched_burial`, `rocks.bedding`).
  - z = local ground (H0 at the footprint centre, using the lowest of 5 footprint samples) + `s * (bottom_offset - f * c_proto)`.
  - **As built: contact rule.** The lowest vertex of the visual LOD touches the ground, then the rock sinks by burial x its vertical extent. The build's `contact` stage repeats this on the rendered ground TIN (section 6) for every rock without a fillet (CR-05, FID-08, FID-10), so no rock hovers above or vanishes below what Gazebo draws (as built: 251,001 rocks re-seated, 0 hover by more than 1 mm).
  - H0 is the ground without fillets.
- **Catalog** (`rocks_catalog.npz`, also written as CSV): one row per rock with fields
  - `id, zone(0 core, 1 ring), tile_i, tile_j, proto, cls, lod, x, y, z, qw, qx, qy, qz, scale, D, exposed_h, burial, collide`.
  - Tile assignment is by rock centre.
  - `collide = D >= collision_min_D_m` (core only).
- **Fillets.** Core rocks of 10 cm and larger with burial above 0 add a smooth annular ramp around their base. It is added to H only after placement. At the LM site fillets are rare and small (heights 0-15% of the exposed height).
  - **As built (FID-11): wedge fillets.** The elliptical ramp left plateaus beyond the contact. The fillet now decays as h_f (1 - smoothstep(d / w_f)) from the rock's outline cut by the local ground plane, and never rises above the rock it leans on (`terrain.fillets.profile: wedge`). Only rocks buried more than `terrain.fillets.min_burial` (0.06) get one, so bedded perched rocks have none. As built: 1,705 rocks, at most 46.7 mm.
- **Merging into tiles.** Each tile's rocks become one mesh: transformed LOD geometry, rotated normals and atlas UVs. That mesh is written as `tile_ii_jj_rocks.glb`. Never use separate Gazebo models or visuals per rock: 10k separate visuals dropped RTF to 0.03 (perf).

## 6. Meshes

- **TIN.** `pydelatin` on each tile's grid, using that tile's own node rectangle plus a 1-node shared border so neighbours share the boundary nodes, with `max_error` from the config. All vertices sit on grid nodes. The four rectangle corners are always present.
  - **As built:** core 3 mm (`mesh.core_visual_max_error_m`, was 5 mm), ring 5 cm, far 0.2 m, horizon 2 m. At 3 mm the core ground is 8.74 M triangles (95,128 in the largest tile).
  - **As built (FID-07): seam band.** Edges within `mesh.core_visual_seam_band_m` (6 cm) of a 10 m tile seam are refined to at most `mesh.core_visual_seam_max_edge_m` (8 cm), so the seams show no crease lines (+0.21 M triangles).
  - The rendered ground TINs are built before rock contact, collision and rock merging (build stage `ground`, section 14).
- **Stitching invariant.** For every pair of adjacent pieces (tile-tile, tile-ring strip, strip-strip, ring-far, far-horizon), the set of vertices on the shared boundary segment is identical in both pieces, and so are their heights. This prevents LiDAR leaks: coincident shared vertices leaked 0 rays in the gazebo tests, while T-junctions leaked 0.02-0.2%.
  - **Algorithm.** Collect the lattice positions of both pieces' boundary vertices on the shared segment. Insert the missing positions into each piece by fan-splitting the boundary triangle whose edge contains them.
  - **Heights.** Canonical heights come from the inner zone, or the lower tile index within the core. The outer piece's existing boundary heights are overwritten with the canonical values.
  - **Test.** `mesh/stitch.py` must provide `check_stitched(pieces)`, which proves the invariant (0 mismatches, 0 T-junctions).
- **Normals.** Ground vertex normals come from the height field gradient (central differences on the full-resolution grid, lightly smoothed), not from the decimated triangles, so shading keeps the fine relief. Shared boundary vertices get identical normals by using the canonical piece's grid.
- **UVs.** World-planar: `u = x / uv_tile_m[zone]`, `v = -y / uv_tile_m[zone]`, continuous across pieces.
- **GLB rules** (verified in Gazebo by the gazebo agent):
  - Positions are raw Z-up metres in world coordinates, with no Y-up conversion and no node transform.
  - float32 positions, normals and UVs; uint32 indices; CCW winding seen from above (+z normals). Gazebo culls back faces for both the LiDAR and the camera.
  - Exactly one explicit material with metallicFactor 0 and roughnessFactor from the config. Without one, GLBs render nearly black.
  - No embedded textures. Textures are attached through the SDF PBR material (section 9) so every tile shares one cached texture.
  - One primitive per file: ground and rocks are separate files.
- **Files** (in `world_dir/meshes/`):
  - visual: `tile_ii_jj_ground.glb`, `tile_ii_jj_rocks.glb`, `ring_<side>_ground.glb`, `ring_<side>_rocks.glb`, `far_<side>_ground.glb`, `horizon_<side>_ground.glb`;
  - collision: `tile_ii_jj_collision.glb` (per core tile, see section 7), `ring_<side>_collision.glb`, `far_<side>_collision.glb`.
  - Every file must exist before the SDF is written. One missing mesh aborts the whole world load.

## 7. Collision

- **Core collision height field.** A 2 cm grid:

```
C = lowpass(H, 0.25 m) + ground_offset_m (-0.02)
C = max(C, rock_top + rock_offset_m (-0.01))   for rocks with collide = true
```

  - `lowpass` is a Gaussian with `sigma = 0.25 * sqrt(ln 2 / (2 pi^2)) = 4.7 cm`, which gives 50% amplitude at 25 cm.
  - `rock_top(x, y)` is the highest point of the transformed rock hull above (x, y). Rasterise the hull triangles with numba.
  - Triangulate with pydelatin (`collision.core_max_error_m`, 1 cm). Flat ground then gets large triangles, which reduce wheel jitter: about 1 mm on 50 cm triangles versus up to 28 mm on 5 cm triangles (gazebo). **As built: 1.5 cm** (1 cm gave 582 k core triangles, about 70 % of them baked rocks; the rock tops survive the coarser tolerance).
  - **Per-tile collision** (updated from performance_budget.md). The core collision is cut into the same 100 tiles as the visuals (`tile_ii_jj_collision.glb`), stitched to each other and to the ring collision strips. Per physics step the cost is the same as one big mesh, but tiles load faster and use less RAM.
  - **Budget.** Physics step cost grows with collision triangles: 0.53 / 0.68 / 0.97 ms per 1 ms step at 0.25 / 1 / 4 M triangles. Aim for 0.25-0.5 M collision triangles in the core and never exceed 1 M. If needed, raise `collision.core_max_error_m` (1 -> 2 cm) or the low-pass, and report the count. As built: core 440,766, ring 338,850, far 26,506, total 806,122.
  - Rock collision proxies are baked into these tile meshes (as above). Never use per-rock models, links or `convex_hull` at scale: the cost is per entity, and 10k separate rock collisions give RTF 0.15-0.24.
  - **Maximum edge length: implemented, kept OFF** (`mesh.collision.core_max_edge_m` and `ring_max_edge_m`: null). Large tilted collision triangles have tall bounding boxes, which made Bullet's GImpact test the rover's CAD collision meshes on slopes (RTF 0.05-0.2). A conforming longest-edge refinement (`mesh/refine.py`) caps the triangle size. Measured on the slope site (`docs/impl/fix_build.md` 2): with a 0.5 m cap the RTF of the CAD-collision robot rose about 3x, but the climb got worse for both robot collision models (0.83-0.85 of the commanded distance fell to 0.34-0.73; with the simplified robot 0.85 fell to 0.61), and the simplified robot gained no RTF. Small mesh triangles worsen classic Bullet's wheel contact. The RTF fix is on the robot side (section 12); 0.5 m stays available for users of the original robot collisions who prefer RTF over climbing.
- **Ring collision.** The ring visual height field plus the ground offset, with rocks of 30 cm and larger rasterised. Tolerance 3 cm.
  - **As built (FID-08): footprint dilation.** Point samples of the rock tops at the 25 cm nodes missed most of a rock's footprint (32 % of footprint points covered). The rock tops are now max-pooled over +- `mesh.collision.ring_rock_dilation` node spacings (default 1, i.e. 25 cm; the exact maximum of each hull triangle over the square): 99.95 % of footprint points and 99.7 % of rocks are fully covered, for +21 % ring collision triangles. The price: ring rocks collide up to 25 cm before their visual surface. The rover is meant to stay in the core.
- **Far collision.** Tolerance 0.5 m. It only stops a runaway rover from falling. The horizon has no collision.
- **Why this works.** Classic Bullet treats every mesh as GImpact, and a single unresolvable file aborts the load. The perf tests measured 4M collision triangles at RTF about 0.96.

## 8. World SDF (`world/sdf.py`)

- **World name** `apollo16_core`. Nothing in the launch chain depends on the name.
- **Systems.** Physics (with `<engine><filename>`), UserCommands and SceneBroadcaster only. The robot itself brings Sensors, Imu, NavSat, PosePublisher and gz_ros2_control, so the world must not duplicate them.
- **Physics.** `<physics type="ignored">` with `max_step_size` = `physics.step_s[engine]`, real_time_factor 1, and real_time_update_rate = 1 / step.
  - The engine is the patched plugin, referenced by absolute path, for `engine: patched`, or `gz-physics-bullet-plugin` for `engine: stock`.
- **Gravity** `0 0 -1.62`. Magnetic field about 0.
- **Spherical coordinates.** `CUSTOM_SURFACE`, with both axes 1737400, ENU world frame orientation, the site lat/lon, elevation = href, heading 0.
- **Scene.** Background black, shadows true, no grid, ambient from the lighting mode.
  - Sun: a directional light along `lighting.sun_events[sun_event][2]`, intensity 1.0, casting shadows.
  - Earthshine: a faint directional light from the Earth's direction, no shadows. **As built (FID-14):** intensity `lighting.realistic.earthshine_intensity` = 3e-5 of the sun, the calibrated value (1e-2 before the fix round). Gazebo cameras have a fixed exposure, so it renders as nothing; it stays in the SDF with the right direction and colour for cameras with exposure control or a night mode. The realistic ambient is 0.01.
  - `inspection` mode adds an overhead fill light and ambient light, and uses the `eva2_start` sun.
  - As built: the far-field and horizon visuals do not cast shadows (they cannot shade the core at a sun elevation of 21.9 deg or more, and it saves shadow-pass work).
- **Models.**
  - One static model `apollo16_terrain` with a single link holding all visuals and collisions. 100 visuals in one model behave the same as 100 models (gazebo).
  - Visual materials: SDF `<material><pbr><metal>` with `albedo_map`, `normal_map` and `roughness_map` relative to the SDF, `metalness 0`, `roughness` from the config, plus `<diffuse>` from the base colour.
  - Collisions use `<surface><friction><ode><mu>`/`<mu2>`. These values are honoured on meshes only by the patched plugin.
- **Variants written per build.**
  - `apollo16_core.sdf`: patched physics, realistic lighting.
  - `apollo16_core_inspect.sdf`: patched physics, inspection lighting.
  - `apollo16_core_stock.sdf`: stock plugin, 4 ms step, realistic lighting. This is the fallback when the patched plugin is missing.
  - `apollo16_core_inspect_stock.sdf` (as built, U11): stock plugin, 4 ms step, inspection lighting; the fallback of the inspect variant.
- **Checks before writing.** Every URI must resolve to an existing file.

## 9. Materials

- **Regolith.** Tileable PNG maps covering `uv_tile_m`, generated by `materials/regolith.py` with numpy using periodic FFT synthesis:
  - albedo: base colour times ± 12% low-contrast variation, stored in sRGB;
  - normal: from sub-centimetre micro-relief, OpenGL convention (+Y green). Verify in Gazebo, and add a flag to flip G if needed;
  - roughness: about 0.95.
  - The far field and horizon reuse the same maps with longer UV repeats.
- **Rocks.** The library atlas maps (basecolor, normal, ORM) are copied into `world_dir/textures/`.

## 10. Physics plugin (`tools/gz_physics_bullet_lunar/`)

- Builds the classic-Bullet plugin from the gz-physics 7.6.0 source with two patches:
  1. the rover agent's `SetJointVelocityCommand` yaw-damper fix (keeps the parent body's angular velocity);
  2. mesh collisions honour SDF `<ode><mu>`, setting friction on the mesh collision object as primitives already do.
- Installs to `cfg.paths.physics_plugin`.
- Tests: a 20 degree incline with a box on a mesh at mu 0.2 and 0.8 must slide and stick as predicted, and the rover's in-place turn at 1.62 m/s^2 must track at least 85% of the command.
- `engine: stock` is the fallback when the build is unavailable.
- As built: the incline test passes 8/8 against the prediction (`tools/gz_physics_bullet_lunar/incline_test.py`); in-place turns track 0.95-0.97 of the command (stock plugin at 4 ms: 0.45-0.66). The launch checks with dlopen that the plugin loads before it starts the world, and otherwise falls back to the stock variant (section 12).

## 11. Spawn and test sites (`world/spawn.py` -> `world_info.yaml`)

- **Spawn z.** Highest point of C under the footprint (±0.55 x ±0.40 m, at any yaw: use a radius-0.68 m disc) + `base_link_height_m` (0.136) + `clearance_m` (0.04).
- **Sites.**
  - `flat`: the nearest suitable site to the core centre. Footprint slope under 5 degrees, no collide-rock within footprint + 0.3 m, low roughness.
  - `slope`: a pose facing uphill at the bottom of a 10-15 degree slope at least 5 m long.
  - `rock`: a pose 2 m before a lone 6-10 cm rock on a clear straight path.
  - `turn`: a flat site with no rocks of 5 cm or larger within 1.5 m. **As built: 10 cm** (`spawn.turn_rock_free_D_m`, decision in section 13).
- **`world_info.yaml` contents.** World name, the SDF variants, origin lat/lon/href, gravity, sites (`x, y, z, yaw` plus a description), zone sizes, triangle counts, rock counts, the build config hash, and the generator version.
- **As built** (final build): flat (-2.00, -4.50, yaw -22.5 deg), slope (23.50, -16.50, -157.5 deg; 5 m at 12.4 deg), rock (-4.12, -7.41, 67.5 deg; an 8.7 cm rock), turn (-3.00, 3.00, -67.5 deg). The `flat` rule judges only the footprint: the final flat site's 4 m drive corridor falls 7.4 deg over 3 m and holds 2 rocks of 5 cm or larger. A corridor requirement in `world/spawn.py`, or a grade-aware drive criterion, is an open decision (`docs/impl/final_build.md` 11.2).

## 12. Launch integration (`launch/apollo16_sim.launch.py`, `scripts/run_apollo16_sim.sh`)

- **Problem.** The stock chain (`togo_sim_wsl.sh` -> `sim_gz.launch.py`) works with an absolute `TOGO_WORLD`, but it starts the controller spawners at a fixed 10 s. Spawning the robot stalls the simulation while the robot's own Sensors system loads the render scene: 11-14 s on the old 30 m world. The spawner's switch call then times out (rover, legacy: 22 of 25 old lunar launches had a spawner failure).
- **Our launch file** reproduces `sim_gz.launch.py` and `togo_deploy` `control.launch.py` with the same robot_description xacro arguments, bridge config and node names, but sequences events:
  1. Start the server with the absolute world path.
  2. Spawn the robot at the `world_info.yaml` site (argument `site:=flat`), or at explicit x/y/z/yaw.
  3. Start the controller spawners only after the robot's LiDAR topic is advertised (render scene loaded), with generous timeouts.
  4. Optionally start RViz.
- **As built, the launch file also:**
  - announces readiness: `wait_ready` checks that both controllers are active (`list_controllers`) and that `/clock` advances, then the launch prints `[apollo16_sim] READY at +T s` and writes a marker file atomically (`ready_file:=PATH`; `PATH.not_ready` when the check fails). The wrapper waits for this marker;
  - runs the gz server under a guard that stops it only with SIGKILL. gz-sim 8 segfaults when it gets SIGINT or SIGTERM, and WSL then writes a 1.5-3.6 GB crash dump; with the guard, Ctrl+C takes 1.2-2.2 s and writes none (`gz_guard:=false` restores the upstream include);
  - checks with dlopen that the patched physics plugin loads, and otherwise falls back to `<world>_stock.sdf` (`<world>_inspect_stock.sdf` for the inspect variant) with a warning that names the consequences;
  - replaces the robot's 14 CAD collision meshes (368,456 triangles) in memory by 88 boxes that reach at most 15 mm further towards the ground than the meshes (`robot_collision:=simplified`, the default; spec `config/robot/a300_collision_simplified.yaml`, written by `tools/robot_collision_simplify.py`). Visuals, inertia, joints and the URDF files are unchanged. This is the RTF fix on slopes: GImpact tests of the CAD meshes against the terrain made the main thread 6.5x more expensive (slope RTF 0.12-0.14 with the meshes against 0.44-0.70 with the boxes on the same busy machine; 0.90 on a quiet one). `robot_collision:=original` keeps the meshes.
- **Wrapper script** `run_apollo16_sim.sh` mirrors `togo_sim_wsl.sh`: WSL GPU environment including `GALLIUM_DRIVER=d3d12`, ROS domain 29, CycloneDDS config from the repo, optional GUI (`gz sim -g`). It also takes `--headless`, `--site`, `--variant` and `--rviz`.
- **As built, the wrapper:**
  - takes its environment from `scripts/env.sh`, which a second terminal can source too. Besides the GPU and DDS settings it sets `MESA_EXTENSION_OVERRIDE=-GL_ARB_multi_draw_indirect`: Mesa's d3d12 driver (WSL's GPU path, Mesa 25.2.8 up to at least 26.2.5) leaks one `ID3D12CommandSignature` per indirect draw call, and Ogre-Next draws with `glMultiDrawElementsIndirect` whenever the extension is offered, so every rendering gz process grew by 2.7-10 MB/s. Without the extension Ogre-Next uses plain draws: memory flat, images pixel-identical, same speed (`docs/impl/leak_diagnosis.md`). Every process that renders with Gazebo in WSL needs it;
  - starts the simulation headless first: it starts the launch without a GUI, waits for the ready marker (at most `--ready-timeout`, 600 s) and only then starts `gz sim -g` with a per-run copy of `config/gui/apollo16.gui.config` whose camera looks at the spawn. In WSLg the GUI was reliable only when it started after the server had loaded the world. `--headless` opens no GUI; `--gui-early` keeps the old order. `scripts/open_gui.sh` opens, closes (`--close`) or reopens the GUI of a running simulation from another terminal (shared helpers in `scripts/apollo16_lib.sh`);
  - stops on Ctrl+C with SIGKILL for the GUI and the gz server first and SIGINT for the ROS nodes, deletes the WSL crash dumps of its own gz/ruby processes at exit, and warns when WSL has less than 3 GB of memory left;
  - validates its arguments and offers `--dry-run`, `--robot-collision`, `--world-info`, `--gui-config`, `--default-gui` and `--keep-crash-dumps` (README section 4).
- **Unchanged.** The user's existing scripts, the robot description, the topic names, and the SLAM and Nav2 stacks. **Exception: DLIO's `odom/gravity` must equal the world gravity (1.62 m/s²).** The shared `lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml` keeps 9.82 for the Earth-gravity `lunar_surface.world` test; this package ships an otherwise identical copy, `config/slam/seyond_dlio_graph_lunar.yaml` (regenerated by `config/slam/make_lunar_dlio_config.py`), passed as `COMBINED_PARAM_FILE` to `run_live_seyond_dlio_slam.sh` (README section 5). With 9.82, DLIO's start-up calibration stores a -8.2 m/s² accelerometer bias that leaks 8.2 sin(tilt) m/s² into its motion estimate. The graph backend's hazard mapping uses only the gravity direction. The only edit outside this package is the `MESA_EXTENSION_OVERRIDE` line in `src/togo_sim_wsl.sh` (the user's request).

## 13. Validation (acceptance criteria)

Decisions taken after phase 2. They supersede the corresponding bullets below.

- **Roughness, site-aware.** The science target nu(L) = 0.069 L^0.8 describes typical LM-area small-scale roughness. Site F1 lies inside a degraded bowl, and its real band-limited DTM alone is already 1.06 / 1.30 / 1.49 times the target at 1 / 3 / 10 m. Acceptance is therefore:
  - composite H0 within ±25% of the target at 0.01-0.3 m;
  - the synthetic part alone (H0 - B, i.e. craters + fBm) within ±25% of the target at 1 m and 3 m;
  - at 10 m and above, the composite within 1.15x of B alone. The real DTM governs there; synthetic detail must not dominate it.
  - The calibration satisfies all three. Measure H0 without rocks. (Phase 2: one gain, 0.278. As built: two gains, 0.109 coarse and 0.578 fine, section 3.)
- **Forward drive.** The start is traction-limited (mu g = 0.65 m/s^2 is below the controller's 2 m/s^2 limit), which is correct lunar behaviour. Pass on:
  - steady-state speed at least 0.95 of the command;
  - forward distance at least 0.93 of the nominal 3 m segment.
- **Turn site.** At the Apollo 16 rock density (1.66 rocks/m^2 of 5 cm and larger) no 1.5 m disc is free of 5 cm rocks. The turn site requires no rocks of 10 cm or larger within 1.5 m.
- **Materials (measured, mesh and world owners).** ogre2 multiplies `<diffuse>` into the albedo map, so a textured material uses `<diffuse>1 1 1 1</diffuse>`. With a roughness map, the scalar `<roughness>` is 1. Normal maps use the OpenGL convention with no flip, verified in Gazebo.

- **Static**, written to `reports/static_checks.json`:
  - deviogram within ±25% of target at 0.01, 0.1, 1 and 10 m;
  - crater counts N(>=D) within ±20% at D = 0.2, 0.5, 1 and 2 m;
  - rock counts N(>=D) within sampling tolerance (±3 sigma) at 2, 5, 10 and 20 cm;
  - visual vs collision on rock-free ground: median dz = -20 mm ± 3 mm, RMS of (dz - median) ≤ 10 mm, p99 ≤ 30 mm;
  - stitching check: 0 mismatches;
  - all GLBs load in trimesh with +z normals on at least 99.9% of ground vertices;
  - triangle counts and file sizes recorded.
- **Gazebo, world only** (headless, a static gpu_lidar injected into a copy of the SDF):
  - loads with no errors;
  - D3D12 renderer confirmed via `/proc/<pid>/maps`;
  - load time ≤ `validation.load_time_max_s`;
  - RTF ≥ 0.95;
  - server RSS ≤ 5 GB;
  - no LiDAR returns beyond the ring from core poses;
  - rock detection versus size and range consistent with the gazebo small-object results.
- **Gazebo, robot** (our launch, headless):
  - controllers active, verified with `list_controllers`, and no spawner failure in 3 of 3 launches;
  - RTF ≥ 0.9 while driving;
  - `tools/rover_drive_test.py` standard preset at `flat`: forward distance ≥ 95% of commanded, in-place yaw tracking ≥ 85%, airborne fraction ≤ 2%;
  - `slope` climbed;
  - `rock` traversed without stalling.
- **Visual.** Camera images from at least 4 viewpoints, plus a Blender or Gazebo overview, saved to `reports/` and reviewed by eye.

**Status of the final world** (as built; `docs/impl/final_build.md`):

- **Static:** 8 / 8 pass (roughness 0.93-1.23 of the target at 0.01-0.3 m, H0 - B 0.89 / 0.83 at 1 / 3 m, H0 / B 1.06 at 10 m; craters within 6 %; rocks within 0.9 sigma; dz median -20.3 mm, RMS 6.0 mm, p99 15.2 mm; 324 GLBs, 484 stitched pairs with 0 mismatches).
- **Gazebo, world only** (LiDAR bench at the flat site, quiet machine): 0 errors, D3D12 confirmed, first LiDAR cloud after 27.0 s (limit 30 s; a busy machine adds about 4 s), RTF 1.000, server RSS 3.69 GB peak. Final validation (`docs/impl/final_validation.md` 1, 5): bench again 27.48 s, RTF 0.9998, 3.69 GB peak; LiDAR checks on 18 poses 5 / 5 (0 returns beyond the ring, 0 below ground, 0 leaks, 4 floating of 260,369, rock detection 7 of 7 judged bins; 4 of 5 on the 6 phase-3 poses alone).
- **Gazebo, robot:** the phase-3 world passed 23 of 25 checks; it failed RTF while driving (0.05-0.2 on the slope, fixed by the simplified robot collision of section 12) and the flat-site distance (0.928 against 0.93 on a corridor rising 3 deg). The final world (12 launches, headless; `final_validation.md` 1-3): 9 of 10 pass. Controllers 12 / 12, no spawner failure; RTF while driving 0.929-0.996; flat distance 0.950-0.955 (0.940 on level ground), steady speed 0.997-1.005, yaw tracking 0.968-1.016, airborne 0-1.67 %; rock 0.969-0.985; turn site 0.888-0.889. **The slope fails:** 0.649-0.700 of 6.0 m against 0.80 on the rough natural slope (smooth 12.4 deg incline with the same physics 0.938); accepting it with a qualified criterion or moving the site is the user's decision.
- **Visual:** not run; deferred until the camera work (the user's decision).

## 13a. Performance budgets (measured, performance_budget.md)

- **Visual triangles.**
  - Core terrain: 8-10 M in total, at most 200k per 10 m tile.
  - Rocks merged into the tiles: at most 6 M in total. Pebbles about 20 triangles each, 5-20 cm rocks about 100, rocks of 20 cm and larger up to about 1500 (large boulders may use more).
  - Ring, far field and horizon: about 1 M.
  - Total: at most 16 M, hard cap 20 M.
- **Load.** Headless first LiDAR cloud arrives after about 2.95 + 0.81 * T seconds with tiles, where T is millions of visual triangles. One big mesh loads about twice as slowly.
- **Memory.**
  - Server RSS about 0.33 + 0.107 * T GB. All of it is the ogre2 render scene.
  - The GUI client builds the scene again, so budget RAM twice.
  - VRAM about 0.065-0.08 GiB per M triangles per process.
- **Robot spawn stall.** About 3.2 + 1.2 * T seconds: the robot's Sensors system builds the render scene while physics does not step. This is why section 12 gates the controller spawners on the LiDAR topic.
- **LiDAR render headroom.**
  - Current sensor at 20 Hz: 2.5-5x.
  - Robin-W-like sensor at 10 Hz: 3.6-6x.
- **Rocks.** Merged into the tile meshes they cost almost nothing (216k rocks, 5.5 M triangles: RTF 1.0, +0.6 GB). Separate visuals are disastrous: 3k gives RTF 0.36, 10k gives 0.03, and 50k crashes the server.

**As built.** The config (`budgets`, enforced by the build's `meshes` stage) splits the limits into soft targets, which only warn, and hard limits, which fail the build. The rock limit was raised because the core rock total varies with the seed (6.54-6.68 M for seeds 16-23).

| budget | soft (warning) | hard (fails the build) | final world |
|---|---|---|---|
| core ground per 10 m tile | - | 200,000 | 95,128 |
| core ground | - | 10 M | 8.74 M (3 mm tolerance + seam band) |
| core rocks | 6.6 M | 7.5 M | 6.58 M |
| visual total | 16 M | 20 M | **17.13 M: over the soft target by 7 %** |
| core collision | target 0.25-0.5 M | 1 M | 0.44 M |
| collision total | 1 M | 1.5 M | 0.81 M |

The ground is the reason for 17.1 M: the 3 mm core tolerance on the recalibrated terrain (fine gain 0.578) needs 8.53 M triangles before the seam band, against 4.65 M on the phase-3 field. 3.5 mm would give about 15.1 M visual triangles and 4 mm about 13.7 M (`docs/impl/final_build.md` 5; an open decision).

Load and memory, measured on the final world, are higher than the fits above predict for 17.1 M triangles (the fits: 16.8 s, 2.2 GB, 1.1-1.4 GiB VRAM):

- **Load:** first LiDAR cloud after 27.0 s on a quiet machine (world only, one injected LiDAR); about 4 s more on a busy machine.
- **Memory:** the server with one LiDAR 3.57 GB at load, 3.69 GB peak; with the rover about 3.7 GB. The GUI client about 3.5 GB (0.5 GB base, 1.0 GB ground, 2.1 GB rocks, measured with ground-only and rock-only copies). Budget RAM twice still holds.
- **VRAM:** the whole GPU went from 2,379 MiB at idle to 4,834 MiB with the server and one LiDAR, about 2.4 GiB.
- **Memory over time (WSL):** both numbers are flat only with `MESA_EXTENSION_OVERRIDE=-GL_ARB_multi_draw_indirect` (section 12). Without it every rendering gz process grows by 2.7-10 MB/s, in proportion to its indirect draw calls, not its triangles: about 54 per GUI frame in this world (216 visuals, 3 shadow splits).

## 14. Code layout and ownership

```
lunar_world_gen/
  grid.py config.py layout.py                (foundation, shared)
  terrain/  base.py fbm.py craters.py compose.py calibrate.py      (terrain)
  rocks/    library.py population.py placement.py fillets.py collision_field.py catalog.py   (rocks)
  mesh/     types.py tin.py stitch.py normals.py uv.py rocks_merge.py gltf.py               (mesh)
  materials/ regolith.py                                                                 (mesh)
  world/    sdf.py lighting.py spawn.py world_info.py                                    (world)
  validate/ stats.py static_checks.py lidar_checks.py report.py                          (validation)
  build.py  (integration: python -m lunar_world_gen.build --config ... --preset ... --stages ...)
blender/ rock_proto.py (prototype) rock_library.py (production, LODs)                     (rocks)
tools/   dtm_extract.py rover_drive_test.py verify_rock_glb.py
         gz_physics_bullet_lunar/ (patch + build)                                        (world)
         bench_world.py lidar_capture.py run_robot_tests.py                              (validation)
launch/  apollo16_sim.launch.py                                                          (world)
scripts/ run_apollo16_sim.sh                                                             (world)
tests/   test_*.py (pytest; each owner adds tests for its modules)
```

**As built** (`+` marks files added since the tree above was written):

```
lunar_world_gen/
  terrain/  + talus.py (T, section 3)
  rocks/    + recontact.py (contact on the rendered TIN); the per-tile rock merge is rocks/merge.py, not mesh/rocks_merge.py
  mesh/     + refine.py (conforming max-edge refinement, off by default), locate.py (exact TIN heights)
tools/   + render_views.py (camera images), robot_collision_simplify.py (robot collision boxes),
           gz_physics_bullet_lunar/incline_test.py, gz_physics_bullet_velcmd_fix/ (the rover study's first yaw patch)
config/  apollo16_core.yaml, gui/ (GUI config + camera generator), slam/ (lunar DLIO copy + generator),
         robot/ (simplified collision spec)
scripts/ + open_gui.sh, apollo16_lib.sh (shared by both scripts), env.sh (the sim environment)
```

**Build stages** (as built, `build.py` docstring; ten instead of eight): terrain; rocks (placed on H0); heights (fillets); ground (the rendered visual TINs, stitched); contact (rocks without a fillet re-seated on the rendered TIN); collision (rock tops from the final catalog); meshes (collision TINs, merged rock meshes, budgets); export (GLBs, textures); world (sites, `world_info.yaml`, four SDF variants, `gz sdf -k`); validate (static checks and report; Gazebo results older than the SDF are archived first). Each stage's key covers its config keys, its code, the foundation modules, the library versions and its input files; a re-run stage invalidates everything downstream. `--force-downstream`, `--adopt` and a build lock (`--lock-timeout`) were added; a non-default preset builds into `<dir>_<preset>`.

- Every function that takes randomness takes the config and derives its stream with `config.rng(cfg, "<component>")`.
- Every stage that writes files records a JSON manifest (inputs, config-section hash, outputs, timings) in `build_dir/<stage>/manifest.json`.
