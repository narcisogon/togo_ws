# Fidelity review of the built `apollo16_core` world (review-fidelity, phase 3)

Read-only review, 2026-10-25. It judges how faithfully the **built** world (world dir, build dir, catalogs,
height grids, GLBs, SDF) represents the Apollo 16 landing site for LiDAR-based navigation. The references are
`docs/research/lunar_calibration.*`, `docs/research/site_selection.*` and `docs/DESIGN.md`.
Gazebo was not started. Every number below comes from scripts I ran on the built products.

- **Scripts:** `/home/nargon/togo_assets/lunar/experiments/p3_review-fidelity/scripts/` (`s01`..`s10`, plus inline checks).
- **Figures:** `.../p3_review-fidelity/fig/`.
- **Raw numbers (JSON):** `.../p3_review-fidelity/out/`.
- **From Windows:** `//wsl.localhost/Ubuntu/home/nargon/togo_assets/lunar/experiments/p3_review-fidelity/`.

How the measurements were made:
- The **rendered** surfaces were rasterised from the world's own GLBs: core ground TINs at 1 cm, ring TINs at 25 cm.
- **Rocks** were rebuilt from the catalog and the library, using the same transform and LOD as the merged meshes.
- **Statements about LiDAR visibility** are inferred from the measured gpu_lidar quantisation and rock-detection tables in `validation.md`. No new captures were taken.

## 1. Verdict

At the scales the real data and the calibration control, the world is a faithful Apollo 16 site:
- **Statistics:** crater and rock size-frequency distributions, rock shapes and exposure all match.
- **Terrain:** 0.3-10 m relief and the real DTM's bowl and craters are reproduced.
- **Geometry and lighting:** the sun and Earth geometry, gravity, spherical coordinates and skyline are correct.
- **Mesh:** there are no seams or density artefacts at tile boundaries.

The problems are **local geometric artefacts of the generators** that a close-range LiDAR or a camera will see:
- rocks balanced on a single vertex with a gap underneath;
- hard-edged "donut" rings of doubled rock density around fresh craters;
- a compound crater with walls steeper than the angle of repose;
- an abrupt change of terrain and rock population at the core edge.

None of them blocks navigation testing. The four medium items should be fixed before the world is used to benchmark LiDAR obstacle detection or SLAM near craters or the core edge.

| ID | Sev. | Finding (short) |
|---|---|---|
| FID-01 | medium | 30 % of rocks are "perched" on one vertex with a randomly tilted pose: visible daylight under rocks up to 30 cm |
| FID-02 | medium | Fresh-crater rock enhancement is a hard ×2 step at R and 2R: sharp "donut" rings of rock density |
| FID-03 | medium | Synthetic fresh crater superposed on a real DTM crater: 2.55 m deep pit with 37-49° walls (above repose); no slope limit |
| FID-04 | medium | Core/ring boundary: all rocks < 30 cm, the 0.2-1 m craters and the < 0.5 m relief stop at x, y = ±50 m |
| FID-05 | low | Fresh crater rims are 31-37 % lower than designed (crest about 0.021 D, calibration 0.025-0.04 D) |
| FID-06 | low | cm-scale roughness 0.63-0.83 of target at 1-5 cm (one shared fBm gain); the 1-3 m excess is the bowl tilt |
| FID-07 | low | 5 mm visual TIN: faceted 11 cm triangles; tile seams are 10 m straight crease lines (5× curvature) |
| FID-08 | low | Ring rocks float up to 42 mm over the rendered ring TIN; 8.6 % of ring rocks are almost absent from ring collision |
| FID-09 | low | Rock-rock interpenetration: 4,604 pairs; 17.7 % of rocks ≥ 5 cm pierced by a smaller rock |
| FID-10 | low | Perched rocks float 1-3.7 mm over the rendered TIN (24 %); 271 rocks lie wholly below the rendered ground |
| FID-11 | low | Fillets are flat plateaus that can stick out beyond the rock outline (7 cm high, 0.4 m wide apron) |
| FID-12 | low | Pebble burial is about 2.5× the calibration for young debris (median 0.25 vs about 0.10) |
| FID-13 | low | Uniform ground albedo (no ejecta halos or rays); one 2 m texture tiles everywhere (camera only) |
| FID-14 | low | Earthshine at 1 % of the sun is 100-1000× too bright (camera only) |
| FID-15 | low | Sinkage is realistic on average (about 10 mm) but uniform; rocks use soil friction; pebbles have no collision |
| FID-16 | low | All 12.4 k core craters are perfect circles with one profile family (no ellipticity or rim irregularity) |
| FID-17 | low | Horizon nodata (2 %) filled with smooth harmonic rectangles 8-10 km north; far and horizon cast no shadows |

## 2. What is faithful (verified, no action)

**Sun and Earth geometry**
- SDF sun direction (-0.9236, -0.0863, -0.3736) gives elevation 21.94° and azimuth 84.66°. This equals the calibration's EVA-1 start (Meeus / ALSJ to 0.02°).
- Earthshine direction: elevation 75°, azimuth 321° (correct).
- Background black, shadows on.

**Physical constants**
- Gravity -1.62 m/s² (calibration 1.6242, -0.26 %).
- Magnetic field 0.
- CUSTOM_SURFACE sphere R = 1737400 m at the site lat/lon, elevation = href.

**Crater statistics**
- Core N(≥D) is 0.995-1.056 of the target from 0.2 to 2 m.
- Freshness mix 10.3 / 19.8 / 69.9 %.
- Profiles are C1 (checked numerically: no step or ring; the only discontinuity is a curvature jump at 1.144 R, invisible).

**Rock statistics and shapes**
- Core counts at 2 / 5 / 10 / 20 cm are within ±1.1 σ.
- 31 rocks ≥ 50 cm (expected 26) and 5 rocks ≥ 1 m (expected 3.8).
- Prototype b/a = 0.71 and c/a = 0.51 (calibration 0.7 / 0.5).
- Median exposed height / D: 0.43 for pebbles and 0.50 for rocks ≥ 5 cm (SP-8023 design value 0.5).
- Tilt from vertical: median 11°, p90 20°.

**No placement artefacts**
- No density artefact at tile seams: pebbles 19.1-19.3 /m² at every distance from 0 to 5 m from a seam; rocks ≥ 5 cm 1.56-1.75 /m².
- No artefact at the core edge from the inside: pebbles 22-24 /m² up to the edge.
- Rocks ≥ 30 cm are continuous across the core/ring boundary within Poisson noise (0.8-1.5 per 100 m² in 5 m bands).
- No rock-free halos: pebble g(r) is 1.1-1.2 from 0.75 to 3 R around rocks ≥ 10 cm, and 0 only under the rock centre, by design.
- No lattice snapping (fractional-cm χ² 24.9 for 19 dof).
- No yaw alignment (36-bin histogram within -2.4 / +2.8 %).

**Visual vs collision on rock-free ground** (static check, re-read):
- dz = z_collision − z_visual: median -20.3 mm, p1 -33.3 mm, p99 -7.1 mm.
- The 10 mm Bullet cylinder margin was measured in `gazebo_capabilities.md`. Wheels therefore sink about 10 mm visually (0 to 23 mm at p99-p1).
- On rocks (offset -10 mm) the wheel sits on the visual top.
- This matches the calibration (about 1 cm; LRV average 1.25 cm; Yutu-2 8 mm).

**Far field and horizon**
- The east strip seam at +958 m is gone: largest per-column step anomaly in the far grid is 3 mm, and -0.9 mm at the seam.
- Skyline from the core centre (eye at H(0,0) + 0.7 m), source by share of azimuths: ring 76 %, far 9 %, horizon 15 %.
- Maximum skyline elevation is 5.98° at azimuth 340.5°.
- The outer edge of the 20 km world is never the skyline.
- Real DTM terrain beyond 10 km (60 km extraction at 100 m) never rises above it (`fig/s08_skyline.png`).
- The robot LiDAR's 150 m range cannot reach beyond the ring from core poses (200 m < 250 m).

**Materials**
- Regolith albedo linear mean (0.1200, 0.1070, 0.0880) equals the calibration's in-situ colour.
- Rock atlas median luminance is 1.87× the soil (calibration: rocks 0.18-0.51, soil 0.16-0.19 normal albedo).

## 3. Findings in detail

### FID-01 (medium): perched rocks balance on one vertex with a tilted pose

**Evidence**

`s04`, `s04c`, `fig/s04_contact_hist.png`, `fig/s05_rock_sections.png`.

How placement works (`rocks/placement.py`):
- 30 % of rocks get burial 0.
- Every rock first gets a random tilt of up to 15° (uniform) on top of the ground normal.
- The rock is then lowered until its lowest vertex touches the ground.

So a perched rock rests on a single vertex or edge, not on a face. Totals: 64,397 core rocks (59,443 pebbles, 4,511 small, 405 medium, 38 large) plus about 861 ring boulders ≥ 30 cm.

Footprint fraction in contact with the rendered ground (underside within max(1 mm, 3 % of height)), median:
- perched rocks: 4.7-14 %;
- buried rocks: 51-70 %.

For core rocks ≥ 15 cm:

| | n | gap under the central 60 % of the footprint | rocks with > 0.1 D of gap there |
|---|---|---|---|
| perched | 223 | max: median 41 mm (0.18 D), p90 78 mm, worst 303 mm | 87 % |
| buried | 500 | max: median 14 mm | 37 % (mostly rocks on slopes) |

Cross-section examples:
- rock 57312 (D 0.91 m, at (37.7, -47.4)) touches at one point with about 25 cm of daylight under one side;
- rock 157955 (D 0.94 m) has 10-15 cm.

**Impact**

These are physically unstable poses that the Moon does not have. Real "perched" ejecta still rests on a face or three points and beds a few mm into loose regolith.

- **LiDAR:** at 1-20 m it gets ground returns under the overhang and sees boulders "hovering". This affects obstacle segmentation and ground-plane fitting, and creates false free space under rocks in 2.5D maps built from the lowest returns.
- **Cameras:** a lit slit under the rock and a detached shadow.

**Fix** (in `rocks/placement.py`)

1. For perched rocks, choose a stable rest face: the convex-hull face whose plane contains the projection of the centroid (quasi-static stable poses, about 20 lines on the 64-vertex hull). Rotate that face to the ground plane, then apply the yaw.
2. Allow at most about 3-5° of extra tilt for perched rocks.
3. Replace burial 0 with a small compliance burial, e.g. U(0.02, 0.06) of the height. Perched should mean "not buried", not "not in contact".
4. Alternative: settle each rock by rotating about the contact vertex toward the centroid until three hull vertices touch.

Acceptance: ≥ 90 % of perched rocks ≥ 15 cm have contact fraction ≥ 0.2 and central gap ≤ 0.05 D.

### FID-02 (medium): hard-edged rock "donuts" around fresh craters

**Evidence**

`s06`, `fig/s06_density_maps.png`, `fig/s06_annulus_profiles.png`, `out/s06_annulus.json`.

The crater enhancement multiplies rock intensity by exactly 2 on 1 R ≤ r ≤ 2 R and by 1 elsewhere. This is a step function, the same for every crater and every size class.

Four annuli intersect the core:

| crater | pebbles /m² inside and outside the annulus | in the annulus |
|---|---|---|
| synthetic 11.8 m at (-31.9, 26.0) | 10-13 | 22-25 |
| synthetic 8.9 m at (1.8, -33.6) | 13-15 | 25-29 |
| synthetic 13.4 m at (12.9, 19.5) | 22-27 | 45-50 |
| DTM 22.6 m at (32, -60) | 20-23 | 47-55 |

- Both edges are sharp within one 0.1 R bin. The rings are clearly visible in density maps of rocks ≥ 5 cm too.
- 59 synthetic fresh craters ≥ 5 m (rho ≥ 0.73) carry such rings over core and ring.
- The calibration (§2.4) says blocky craters need D ≳ 4-6 t, about 20-40 m for the about 7 m regolith. The 5-16 m craters should not excavate block fields.
- The density field is also isotropic: there is no ray elongation radial to South Ray (calibration §3.3; already listed in `rocks.md`).

**Impact**

- **LiDAR:** obstacle density (rocks ≥ 5 cm, detectable at 0-8 m) doubles across a perfect circle and halves again at 2 R. A traversability or obstacle map shows an artificial ring, the most obvious synthetic pattern at the 10-50 m scale.
- **Cameras:** the circles are visible from oblique views.

**Fix** (in `rocks/population.py`, crater factor)

- Use a smooth radial law continuous at the rim: e(r) = 1 + (f−1)·(r/R)^-3 for r ≥ R (the ejecta thickness law, calibration §2.3), and a smooth taper (0.5-1 f) over the inner wall and floor.
- Draw f per crater from 2-5 (calibration) scaled by rho.
- Enhance rocks ≥ 10 cm only for D ≳ 20 m. Smaller fresh craters get at most 1.5× for fragments < 10 cm.
- Optional: an azimuthal ray modulation of the density field (elongated toward azimuth about 200°).

### FID-03 (medium): compound crater with walls above the angle of repose

**Evidence**

`s07`, inline check in the log, `fig/s07_core_overview.png`, `fig/s07_crater_profiles_measured.png`.

The synthetic fresh crater (D 11.85 m, rho 0.99; design d 1.52 m, h 0.35 m) landed at (-31.9, 26.0). At the same spot, the band-limited DTM B already has a real depression: 1.09 m deep with a +18 cm rim at about 1.5 R (about 9 m). Pure superposition gives a pit 2.53-2.55 m deep (d/D 0.21) with these slopes:

| slope baseline | max | other measures |
|---|---|---|
| 15 cm | 48.8° | p99 40.2°; 10.4 % of the 1.3 R disc > 35° |
| 0.5 m | 40.2° | 93 cells (23 m²) > 35° |
| 1 m | 37-38° | |

For comparison:
- B alone never exceeds 21.7° anywhere in the core (0.5 m baseline).
- The calibration's fresh small-crater walls are 26-31° and the regolith repose angle is about 35°.

A second, smaller case: a 4 m fresh crater at (-6.5, -47) reaches 36.8°. Overall, synthetic detail doubles the 2 m-baseline area above 15° (2.6 % vs 1.3 % for the DTM alone in the site report).

**Impact**

A 12 m wide pit with 40°+ walls inside the operating area. It is non-traversable and unphysical (regolith would slump). A LiDAR sees an anomalously deep, sharp-walled crater, which matters for slope and hazard estimation tests.

**Fix**

1. Add a deterministic slope limiter after composition: talus relaxation or nonlinear diffusion where |∇H| > tan 34°. Apply it to K_big and K_small on the shared coarse/fine grids so that zones stay bit-consistent.
2. And/or detect DTM depressions in B (LoG, D 8-30 m) and either reject synthetic craters with D ≥ 4 m whose centre lies within 1 R of one, or scale their depth so the combined wall slope stays below 32°.

Acceptance: max slope ≤ 35° at 0.5 m baseline, and slope > 30° area ≤ the calibration's fresh-wall share.

### FID-04 (medium): abrupt terrain and population change at the core edge

**Evidence**

`s06`, `s07`, `s08`, `fig/s07_edge_fade.png`, `fig/s08_core_ring_edge.png`, `fig/s02_tile_00_04_hillshade.png`.

At |x| or |y| = 50 m:

**Rocks**
- Core: 2 cm and larger at 22-24 /m² up to the edge; ≥ 10 cm at about 0.24 /m².
- Ring: only rocks ≥ 30 cm, about 0.012 /m² (2,946 over 240,000 m²).
- Obstacles ≥ 10 cm therefore drop about 20×, and pebbles to zero, across a straight 400 m perimeter.

**Fine relief and small craters**
- The < 25 cm relief fades over the last 1 m: high-pass RMS 1.8 mm → 0.5-0.6 mm at 0.1-0.3 m from the edge.
- The 0.2-1 m craters fade the same way.
- The ring has no content below 25 cm.

**Rendered ring ground**
- 25 cm grid, 5 cm TIN, about 1.5 m² per triangle, with fan slivers along the core boundary.
- Its ν at 0.25-5 m stays within 10 % of the ring grid. Core and ring ν at 0.25 m are similar (19-25 mm vs 19-36 mm).
- So the transition affects only scales below 25 cm and the rocks below 30 cm.

**Impact**

- **LiDAR, rover within about 5-10 m of the edge:** pebbles (seen at < 3 m) and rocks of 5-30 cm (seen at 2-8 m) vanish beyond a straight line, and the ground turns into large planar facets.
- **Planners:** the ring looks trivially clear.
- **LiDAR odometry and SLAM:** feature content drops at the boundary.
- Any traverse longer than about 40 m from the centre crosses it.

**Fix**

Short term: document that tests stay at least 10 m inside the core edge.

Better:
- extend rocks of 5-30 cm (LOD2) into a 50-70 m band of the ring;
- extend pebbles to about 55 m with a 5 m taper;
- widen `zones.core.edge_fade_m` from 1 m to 5 m on a 1-2 cm skirt mesh, or tighten `ring_visual_max_error_m` to 1-2 cm within 20 m of the core (about +0.2 M triangles).

### FID-05 (low): fresh crater rims too low

**Evidence**

`s01`, inline check, `fig/s01_crater_profiles.png`, `fig/s07_crater_profiles_measured.png`.

The smooth minimum with k = 2h lowers the crest to 0.63-0.69 h. `terrain.md` states h/2 at r = R, but the maximum lies further out.

For the freshest craters the crest is about 0.021 D (e.g. D 10 m, rho 1: 0.206 m vs h 0.30 m). The calibration gives 0.025-0.04 D for fresh rims (Pike 0.036).

Measured on H:
- 8.9 m fresh crater: rim +18 cm vs +24.7 cm design;
- sub-metre fresh craters (D 0.39-0.52 m): no raised rim at all (−0.4 to +0.02 cm vs +1.2-1.4 cm design).

**Impact**

Raised rims of small fresh craters are the main LiDAR-visible crater features at range: they cast range shadows and are wheel hazards. They are underestimated by about a third.

**Fix**

In `terrain/craters.py`, use `SMIN_K_RIM_FACTOR = 0.5` (crest about 0.9 h), or pre-scale h by about 1/0.67. Acceptance: crest/D ≥ 0.027 for rho ≥ 0.9.

### FID-06 (low): roughness spectrum shape (cm-scale low)

**Evidence**

`s03`, `fig/s03_tin_deviogram.png`. Deviogram ν(L) over the 100 tiles, each detrended by its own plane (rover scale), as a fraction of the target:

| L | H | rendered TIN |
|---|---|---|
| 1 cm | 0.70× (RMS slope 7.0° vs 9.8° target) | 0.63× (6.3°) |
| 3 cm | 0.79× | |
| 10 cm | 0.89× | |
| 30 cm | 0.96× | |
| 1 m | 1.01× | |

- The DESIGN-definition 1.43× / 1.59× at 1 / 3 m (whole-core plane) comes from the bowl's tilt between tiles, i.e. real DTM.
- Because the fine and coarse fBm bands share one `fbm_gain`, the cm scales cannot reach the target without pushing 0.3 m out of tolerance (`terrain.md`).

**Impact**

Low for the LiDAR: below about 5 mm range quantisation at more than 2 m. The camera gets micro-relief from the normal map.

**Fix**

- Give the fine (≤ 0.5 m) and coarse bands separate gains: fine about 0.6-0.8, coarse kept or lowered.
- Judge roughness on 10 m tiles (tile-plane detrend) alongside the whole-core rule.

### FID-07 (low): visual TIN faceting and 10 m seam creases

**Evidence**

`s02`, `s03`, `fig/s02_tile_08_00_zoom_c.png`, `fig/s13_seam_corner.png`.

The 5 mm core TIN:
- median edge length 11.4 cm, TIN − H residual RMS 1.59 mm (max 5.0 mm);
- keeps ν within 10 % at 1 cm and within 1 % at ≥ 5 cm, but replaces the fractal texture with planar facets;
- along tile seams, the RMS second difference across the seam is 1.92 mm vs 0.38 mm elsewhere (5×). The seams are straight crease lines every 10 m, clearly visible in geometry-based shading.

Vertex normals come from the grid, so Gazebo's direct shading hides most of it.

**Impact**

Small:
- **LiDAR:** deviations ≤ 5 mm are at the quantisation floor.
- **Cameras:** a regular 10 m grid of creases can appear in self-shadowing at low sun.

**Fix**

- `mesh.core_visual_max_error_m` 0.005 → 0.003: 4.65 M ground triangles, 12.5 M total, inside the 16 M budget (`integration.md` §5).
- Optionally triangulate tiles with an overlap and clip at the seam to avoid boundary slivers.

### FID-08 (low): ring rocks vs rendered ring ground and ring collision

**Evidence**

`s10`, `fig/s10_ring_rocks.png`.

Ring rocks are placed on the 25 cm grid, but the rendered ring TIN deviates from it by |dz| p50 10.7 mm, p99 41.7 mm, max 62.6 mm. As a result:
- 9.6 % of the 2,946 ring rocks float > 5 mm above the rendered ground, 2.6 % > 20 mm, the worst 42 mm.

Ring collision (point-sampled tops on 25 cm nodes):
- captures a median 87 % of a rock's rise;
- but 12 % of ring rocks get < 50 % and 8.6 % < 25 %, i.e. almost absent;
- 15.6 % of rocks of 30-50 cm get < 50 %.

**Impact**

Only when driving outside the core: visible gaps under boulders near the edge, and wheels pass through some rocks of 30-50 cm.

**Fix**

- Re-seat ring rocks on the rendered ring surface (`mesh.tin.tin_surface_on_grid`).
- Rasterise ring rock tops at 5 cm and max-pool to the 25 cm grid (one-node dilation).

### FID-09 (low): rock-rock interpenetration

**Evidence**

`s06`; hull test: vertices of the smaller rock inside the larger rock's convex hull.

- 10,392 plan-overlapping pairs.
- 4,604 partially interpenetrating pairs (4.1 % of core rocks involved), 56 fully hidden.
- 1,520 pairs involve a rock ≥ 5 cm: 2,926 rocks ≥ 5 cm (17.7 %) have a smaller rock sticking into them.
- 156 pairs have both rocks ≥ 5 cm, 7 pairs both ≥ 10 cm.

**Impact**

Camera-visible oddities at close range. Negligible for the LiDAR.

**Fix**

Extend the redraw rule to every pebble overlapping a rock ≥ 5 cm, using the plan radius 0.5 a·s, or the hull-vertex test used here.

### FID-10 (low): rocks vs the rendered (TIN) ground

**Evidence**

`out/s04b_vertex_clearance.json` (exact, vertex-based).

- Relative to H0, no rock floats (contact rule).
- Relative to the rendered 5 mm TIN:
  - 23-24 % of perched rocks float > 1 mm, 2.3-3.0 % > 3 mm, p99 3.4-3.7 mm;
  - 271 rocks (266 pebbles, 5 small) are entirely below the rendered surface, i.e. invisible.

**Fix**

Seat rocks against the rendered surface (rasterise the tile TIN at 1 cm, about 1 s per build) or add 3 mm of minimum burial. Drop rocks whose top lies below the TIN.

### FID-11 (low): fillet plateaus extend beyond the rock

**Evidence**

`fig/s12_fillet_section.png`.

- 1,703 fillets; h_f median 4.8 mm, p90 9.8 mm, max 69 mm.
- Each fillet is a flat raised ellipse at 0.45·(a, b)·s plus a ramp.
- On the downhill side of tilted rock 80574 (D 1.41 m) a 7 cm high, about 0.4 m wide apron sticks out beyond the rock's ground-level outline.

**Fix**

Build the fillet as a wedge against the rock's actual ground-level cross-section (rasterised hull at ground height), decaying outward.

### FID-12 (low): pebble burial

**Evidence**

`s01`.
- Non-perched pebble burial: median 0.25 (class range [0.2, 0.5]).
- Calibration for young South Ray debris: 0-20 %, median about 10 %.
- Larger classes have a median of 0.14-0.15 and are fine.
- Median exposed height of pebbles is still 11 mm (p10 7.9 mm).

**Fix**

Pebble burial range [0.2, 0.5] → [0.0, 0.3].

### FID-13 (low, camera only): uniform albedo, single 2 m texture

**Evidence**

`s09`, `fig/s09_regolith_tiling.png`.
- Albedo relative std is 4.9 %; features > 20 cm carry only 1.7 %, so the 2 m repeat is barely visible.
- There is no large-scale albedo variation. Fresh crater ejecta (0.15-0.17 vs 0.11-0.15 normal albedo per the calibration), ray patches and dark mature soil are all absent.
- The same map is reused at 20 m and 200 m repeats.

**Fix**

- Add a low-frequency albedo field (±10-20 % at 5-50 m, correlated with the rock density field) and bright crater halos, through vertex colours or a second UV channel.
- Add a second detail map at a non-commensurate repeat.

### FID-14 (low, camera only): earthshine intensity

**Evidence**

SDF: earthshine intensity 0.01 relative to the sun (1.0); calibration §8: 10^-5 to 10^-4 of sunlight. It is comparable to the 0.01 ambient, so shadows get about 2 % fill.

**Fix**

`lighting.realistic.earthshine_intensity` → 1e-4. Re-check the ambient light against camera images.

### FID-15 (low): uniform sinkage, friction and pebble collision

**Evidence**

- Visual sinkage is about 10 mm everywhere. There is no extra sinkage on fresh rims and ejecta (calibration: up to 5 cm).
- Rocks are baked into the ground collision, so they use the soil's mu 0.4; `physics.rock_mu` 0.6 is unused.
- 199 k pebbles of 2-5 cm (exposed about 1.1 cm) have no collision, so wheels visibly pass through them.

**Fix (optional)**

- A spatial ground offset field (e.g. -0.04 m on rims and ejecta of rho ≥ 0.73 craters).
- Rock tops as a second static link with mu 0.6 (Bullet friction is per body; check RTF).

### FID-16 (low): crater shape uniformity

**Evidence**

Every crater (12,433 centred in the core, 14,764 in the ring set) is an exact circle with one radial profile family in (D, rho). There is no ellipticity, no azimuthal rim-height variation and no asymmetric ejecta.

**Fix**

Per-crater axis ratio 0.85-1, random orientation, and low-order azimuthal rim modulation (±20-30 % h). All of this stays deterministic.

### FID-17 (low): horizon fill and outer-zone shadows

**Evidence**

`fig/s08_far_horizon.png`.
- 20,436 nodes (2 %) are filled harmonically, as two smooth rectangles 8.1-10 km north. They lie below the skyline from the core centre.
- Far and horizon visuals have `cast_shadows false`. Distant hills cast no shadows on distant plains, so camera views toward the horizon are missing 100-250 m shadows at 22° sun.

**Fix**

Clip or fractal-fill the nodata rectangles. Keep shadows off unless camera work needs them.

## 4. Additional measurements (reference)

**Slopes**
- 15 cm baseline: median 7.1°, p95 16.7°; > 15° 7.0 %, > 20° 2.8 %, > 25° 1.3 %.
- 2 m baseline: mean 6.5°, p95 11.9°, > 15° 2.6 %. The site report's DTM-only values are 6.1°, 10.5° and 1.3 %.

**Visual TIN**
- About 17.3 k triangles per 10 m tile.
- 1.73 M of the 8-10 M budget is used (FID-07 suggests 3 mm).

**Rock statistics**
- Perched fraction: pebbles 29.8 %, small 29.9 %, medium 30.6 %, large 36.2 %, ring 29.2 %.
- Prototype reuse: up to 10,113 uses per pebble prototype; up to 516 per large prototype in the ring.
- Reuse is not a LiDAR issue: shapes are randomised by yaw, tilt, scale (0.33-3) and burial.

**Small craters**
- 30 % of 0.2-1 m craters in the core are shallower than 5 mm (degraded). They fall below the TIN tolerance and the LiDAR quantisation. This is acceptable.

## 5. Suggested order of work

1. FID-01: stable resting faces and compliance burial for perched rocks.
2. FID-02: smooth, size-gated crater rock enhancement.
3. FID-03: slope limiter or DTM-aware rejection.
4. FID-04: rock and fine-detail skirt at the core edge, or document the 10 m keep-out.
5. FID-05: rim factor.
6. FID-06: split the fBm gains.
7. The rest as convenient. FID-07 (3 mm TIN) is a one-line override within budget.

## 6. Reproduce

Run from WSL with `PY=/home/nargon/.venvs/lunar_world/bin/python`, in `.../p3_review-fidelity/scripts/`:

| script | what it does | run time |
|---|---|---|
| `s01_catalog_stats.py` | library shapes, burial / exposure / tilt, crater sets, profiles | |
| `s02_tile_hillshades.py [i_j ...]` | 1 cm hillshades of grid vs rendered tiles | |
| `s03_tin_fidelity.py` | TIN vs grid deviograms, all tiles | 44 s |
| `s04_rock_ground_contact.py` | per-rock contact, gap and visibility vs TIN / H / H0; writes the 1 cm core rasters | about 3 min |
| `s05_rock_sections.py` | cross-sections | |
| `s06_rock_spatial.py` | density maps, seam / edge profiles, g(r), interpenetration | |
| `s07_core_overview.py` | overview hillshade / slope, edge fade, crater profiles on H | |
| `s08_outer_zones.py` | ring / far / horizon, skyline | |
| `s09_textures.py` | regolith and rock textures | |
| `s10_ring_rocks_collision.py` | ring rocks vs rendered ground and collision | |

The 60 km DTM reference used by `s08` was made with `tools/dtm_extract.py --site core --size 60000 --spacing 100`, written to the scratch `dtm/` folder.
