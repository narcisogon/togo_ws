# Lunar surface calibration: Apollo 16 LM site (Cayley plains) and a rocky variant

Author: science agent (phase 1), 2026-10-25. Machine-readable companion: `lunar_calibration.yaml` (same folder).

Purpose: give the world generator measurable targets from 1 cm to about 1 km for the Apollo 16 landing site, and for a rocky variant. The targets cover craters, rocks, roughness, slopes, appearance, traction and lighting.

How the numbers were obtained:
- Primary sources (Apollo-era reports, LROC products, peer-reviewed papers) were read where they were reachable. Section 11 lists the URLs.
- Several numbers were measured directly on the repo DTM `NAC_DTM_APOLLO16.TIF` (2 m/px). The scripts are in `/home/nargon/togo_assets/lunar/experiments/science/` (Section 10).
- Sun geometry was recomputed independently and cross-checked against the Apollo Lunar Surface Journal table.

Confidence and verification labels used throughout:
- **H / M / L**: high / medium / low confidence in the value for this use.
- **[read]**: I read the number in the cited source.
- **[snippet]**: the number comes from a secondary summary or search-engine abstract of the cited source; the full text was not read.
- **[recalled]**: standard literature value from memory, not re-read in this session.
- **[measured]**: computed here from the DTM or a model; reproducible with the scripts.
- **[derived]**: my synthesis or extrapolation. It is flagged wherever it goes beyond the measured range.

---

## 0. Recommended values at a glance

| Quantity | LM site (nominal) | Rocky variant (North Ray rim, station 11) | Conf. |
|---|---|---|---|
| LM coordinates | 8.9734° S, 15.5011° E (ME/DE421), ±3 m | — | H |
| Gravity | 1.624 m/s² (1.62 acceptable) | same | H |
| Sun at landing / EVA-1 / EVA-2 / EVA-3 (start) | el 14.7° / 21.9° / 33.8° / 45.1°; az 86.0° / 84.7° / 82.0° / 78.6° (from N through E) | same | H |
| Crater SFD 0.2–50 m (equilibrium) | N(≥D) = C·D⁻² per m², D in m, C = 0.05 (0.03–0.08) | same plains; plus crater ejecta | M |
| Crater degradation mix | fresh 5 %, intermediate 25 %, degraded 70 % | younger, more fresh | M (≥35 m), L (<35 m) |
| Fresh small-crater depth/diameter | 0.12–0.15 (D 30–200 m); 0.10–0.17 assumed below 30 m | 0.2 for North Ray / South Ray themselves | M |
| Rocks per 100 m², D ≥ 2 / 5 / 10 / 20 / 50 / 100 / 200 cm | 2160 / 166 / 24 / 3.4 / 0.26 / 0.038 / 0.0033 | 5810 / 588 / 104 / 21 / 2.6 / 0.52 / 0.065 | M (LM), M–L (rocky) |
| Rock area cover ≥ 2 cm / ≥ 10 cm | 2.3 % / 0.62 % | 9.4 % / 4.3 % | M |
| Rock shape (b/a, c/a) | 0.7, 0.5 (fragments); exposed height ≈ 0.5·D | same; more rounded light-matrix breccias | M |
| RMS slope vs baseline (detrended) | 10–12° at 1 cm, ~6.7° at 10 cm, ~4° at 1 m, 2.5–2.9° at 2–10 m (DTM) | about 2–3× higher at ≥ 2 m (DTM: 8.8° at 10 m) | M |
| Hurst exponent | 0.7–0.8 (≤ 1 m); 0.83–0.89 (10–140 m); ~0.55 (140–400 m) | ~0.94 (2–40 m, DTM) | M |
| Normal albedo, visible (zero phase) | soil 0.16–0.19; rocks 0.18–0.51 | soil 0.21–0.32 | H [read] |
| Normal albedo, 905 nm / 1550 nm (zero phase, LiDAR) | ~0.25 / ~0.35 (±20 %) | ~0.30–0.45 / 0.40–0.6 | M [derived] |
| Linear-RGB albedo tint (white-balanced to sunlight) | R:G:B = 1.12 : 1 : 0.82; base colour ≈ (0.12, 0.107, 0.088) | brighter ×1.3–1.7 | M |
| Regolith density / cohesion / friction angle (top 15 cm) | 1.50 g/cm³ / 0.52 kPa / 42° | rim soils slightly looser | H [recalled, cross-checked] |
| Rigid-wheel effective Coulomb µ (longitudinal / lateral) | 0.40 (0.30–0.50) / 0.30 (0.20–0.45) | same | M–L [derived] |
| Wheel sinkage | ~1 cm (0.5–1.5 cm); up to ~5 cm on fresh crater rims | softer rims, up to ~15 cm at friable boulders | M |
| Max traversable slope | ~20° (LRV: driven on 18°; capability 19–23°) | same | M |

---

## 1. Site

### 1.1 Coordinates, reference frames and the DTM grid

| Item | Value | Source | Conf. |
|---|---|---|---|
| LM (Orion) position | **8.9734° S, 15.5011° E**, planetocentric Mean-Earth/Polar-Axis (DE421) | Wagner et al. 2017 (LROC coordinates); NSSDC table [read]; LROC 2016 table lists ±3.0 m [snippet] | H |
| ALSEP central station | 8.9759° S, 15.4986° E | NSSDC [read] | H |
| Apollo-era site coordinates (used in older tables) | 8.9914° S, 15.5144° E, about 0.7 km from the LROC position | ALSJ Scotti table header [read] | — |
| The task's 8.973 S, 15.500 E | Consistent (differs by 0.0011° in longitude, about 33 m) | — | — |
| DTM projection | Equidistant cylindrical on a 1,737,400 m sphere; **standard parallel −9°**, central meridian 180° (easting negative) | `gdalinfo` [measured] | H |
| DTM scale error at the LM | 0.007 % (scale is true at 9° S), so the projection is effectively distortion-free over 1 km | [measured] | H |
| DTM extents | 8.614–9.646° S, 15.140–15.941° E; 11985 × 15653 px at 2.0 m | `gdalinfo`; LROC README [read] | H |
| LM pixel (row, col) | (5449.8, 5402.4) | `dtm_stats.py` [measured] | H |
| DTM accuracy | SOCET-SET precision error 0.88 m; RMS vs 35 LOLA tracks 1.02 m; known artifacts: "boxes" 0.5–1 m, stereo-model seams up to 1 m, faceted (interpolated) bland or shadowed areas | NAC_DTM_APOLLO16 README [read] | H |

The DTM effective resolution is coarser than the 2 m post spacing. The deviogram has a Hurst exponent of 0.93–0.96 at 2–10 m lags, a smoothing signature, and RMS height in 10 m windows is only about 3 cm. I estimate the effective horizontal resolution at about 5–6 m [derived]. Below about 10 m, all DTM-derived roughness values are lower bounds.

### 1.2 Elevation and local relief

| Item | Value | Source | Conf. |
|---|---|---|---|
| LM elevation (DTM datum; LROC DTMs are referenced to the 1737.4 km sphere) | **+8.4 m** (median 5×5 px); radius ≈ 1737.408 km | [measured] | H (relative), M (datum) |
| ALSEP elevation, LM–ALSEP distance | +11.9 m; 107 m | [measured] | H |
| Maximum relief within 400 m of the LM | 25 m | Schaber 1981 (USGS PP 1048 D1) [read] | H |
| DTM relief, 100 m / 500 m / 1 km / 2 km box around the LM | 7.3 / 28 / 54 / 118 m | [measured] | H |
| North Ray crater (from DTM) | centre 8.8121° S, 15.4844° E; rim diameter ≈ 1.05 km; depth 233 m (d/D 0.22); 4.9 km from the LM (centre) | [measured]; literature: ~1 km, ~230–240 m deep | H |
| South Ray crater (from DTM) | centre 9.1474° S, 15.3826° E; rim diameter ≈ 690 m; depth 127 m (d/D 0.185); rim about 24 m above surroundings (0.034·D); 6.36 km from the LM | [measured]; Ulrich et al. 1981 (PP1048 G): 680 m, 135 m deep, rim 20–22 m, 6.2 km S of LM [read] | H |

### 1.3 Geology at the LM

- **Unit.** The LM landed on the Cayley plains, a light, smooth highland plain floored by impact breccias (no volcanics) [read, PP1048 D1/F]. Under a 6–12 m regolith lies a brecciated debris unit, not competent lava (active and passive seismic) [read, D1]. Magnetometer readings suggest a breccia blanket on the order of 1 km thick [read, D2].
- **Regolith thickness.**
  - Median on the Cayley plains: 6–10 m, generally about 7 m. This comes from benches in 10 concentric craters; the full range is 3.1–15 m (Freeman 1981, PP1048 F) [read].
  - 12.2 m along the active seismic line at the ALSEP (Kovach et al. 1972, via D1/F) [read]. Freeman considers this above the median because the line crossed a subdued crater's ejecta.
  - WC crater (40 m, about 700 m S of the LM) reached "bedrock" under 6.7 m of regolith [read, F].
  - Stone Mountain: 5–10 m to more than 20 m [read, F].
  - Confidence: H.
- **South Ray ray material.**
  - The LM parking sites lie at the edge of South Ray ray material. South Ray is 2 Ma (2–4 Ma) old and lies about 6 km to the SSW [read, D1/G/H].
  - LM/ALSEP rocks are "perched or only slightly buried" and many are glass-coated, so many are probably South Ray ejecta [read, D1].
  - South Ray ejecta thins from about 10–15 m at its rim to ≤ 1 cm at the southern stations and < 1 mm at stations 11/13. The preferred thickness decay is ∝ r⁻³·⁰ [read, G].
  - Rays are mainly bright fine soil plus 2–7 % rock-fragment cover, with fragments mostly < 5 cm [read, H].
- **North Ray.** North Ray is about 50 Ma old (exposure ages 30–60 Ma). Its rim has blocks up to about 25 m (House Rock and Outhouse Rock area) [read, D2].
- **Layering.** At many stations (1, 2, 4a, 5, 11, 13) a lighter subsurface soil appears 1–3 cm below a darker grey surface layer [read, F/H]. This matters for wheel tracks and disturbed soil in camera rendering.

### 1.4 Slopes at 2–100 m baselines (DTM, LM area)

Method:
- Slopes are measured on the 2 m DTM (`dtm_stats.py`, `detrended_compare.py`).
- "Gradient" is the adirectional slope from x and y forward differences over baseline L.
- "Directional RMS" is the arctan of the RMS height difference over L along x or y.
- "Detrended" means a best-fit plane was removed from each 100 m tile first.
- Values at L < 6 m are smoothed by the DTM and should be treated as lower bounds.

**LM-centred 1 km × 1 km box, not detrended** (what a rover actually drives on, including regional tilt) [measured]:

| L (m) | Dir. RMS (°) | Grad. median (°) | Grad. mean (°) | P90 (°) | P99 (°) | >10° | >15° | >20° | >25° |
|---|---|---|---|---|---|---|---|---|---|
| 2 | 3.87 | 4.08 | 4.54 | 7.7 | 15.3 | 3.7 % | 1.1 % | 0.4 % | 0.2 % |
| 4 | 3.80 | 4.07 | 4.50 | 7.6 | 14.7 | 3.5 % | 0.9 % | 0.4 % | 0.1 % |
| 6 | 3.73 | 4.06 | 4.46 | 7.5 | 14.2 | 3.2 % | 0.9 % | 0.3 % | 0.1 % |
| 10 | 3.60 | 4.01 | 4.35 | 7.3 | 13.2 | 2.6 % | 0.7 % | 0.2 % | <0.05 % |
| 20 | 3.36 | 3.89 | 4.14 | 6.8 | 11.3 | 1.7 % | 0.4 % | 0.1 % | — |
| 50 | 2.99 | 3.67 | 3.79 | 6.2 | 9.4 | 0.4 % | — | — | — |
| 100 | 2.70 | 3.37 | 3.43 | 5.5 | 7.9 | 0 | — | — | — |

Other regions [measured]:

| Region | L | Dir. RMS | Grad. median | P90 | P99 | >15° | >25° |
|---|---|---|---|---|---|---|---|
| LM 100 m box (planned core) | 2 m / 10 m | 3.5° / 3.1° | 3.9° / 3.6° | 6.7° / 6.0° | 14.7° / 10.9° | 1.0 % / 0.2 % | 0 |
| LM 2 km box (includes larger craters) | 2 m / 10 m | 5.2° / 4.8° | 4.1° / 4.0° | 10.0° / 9.3° | 25.5° / 23.7° | 4.7 % / 4.2 % | 1.1 % / 0.7 % |
| LM 1 km, 100 m tiles detrended | 2 m / 10 m | 2.9° / 2.5° | 2.4° / 2.1° | 5.6° / 4.9° | 14.7° / 11.9° | 0.9 % / 0.5 % | — |
| North Ray ejecta, 1–2 R annulus | 2 m / 10 m | 9.8° / 8.8° | 8.8° / 8.7° | 22.0° / 20.2° | 33.9° / 27.8° | 23.7 % / 21.6 % | 6.3 % / 2.8 % |
| South Ray ejecta, 1–2 R annulus | 2 m / 10 m | 5.1° / 4.8° | 5.1° / 4.9° | 11.2° / 10.8° | 18.3° / 16.3° | 3.0 % / 2.0 % | 0.1 % / 0 |

Cross-check: NASA SP-8023 (1969) "nominal (rough/hummocky upland)" design curve gives a mean slope of about 9° at 1 m, 5° at 10 m, 3° at 100 m and 1.7° at 1 km. The "minimum (smooth mare)" curve gives about 3.7° at 1 m and 1.6° at 10 m [read, digitised from Fig. 1, ±0.5°]. The LM area lies between these two curves at 10 m. SP-8023 Fig. 2 also gives a slope distribution shape: about 60 % of slopes below the mean, 90 % below 2× the mean, 99 % below about 3× the mean [read]. The DTM gives P90/mean = 1.7 and P99/mean = 3.0, in agreement.

### 1.5 Sun and Earth geometry

The sub-solar point, altitude and azimuth were recomputed with Meeus' algorithms (Sun Ch. 25, Moon Ch. 47 main terms, selenographic Sun Ch. 53; physical libration ignored) at the LROC LM position (`sun_geometry.py`). They match the ALSJ table by J. Scotti, computed from the Nautical Almanac, to within **0.02°** [measured, read]. Azimuth is measured from north through east. Event times (UTC) are from the same table.

| Event (UTC, 1972) | Sun el. | Sun az. | Sub-solar lon, lat | Earth el. / az. | Shadow length / height |
|---|---|---|---|---|---|
| Landing, 21 Apr 02:23:35 | **14.74°** | 86.00° | 90.32° E, +1.54° | 77.2° / 320° | 3.8 |
| EVA-1 start, 21 Apr 16:47:38 | 21.94° | 84.66° | 83.00, +1.55 | 76.4 / 321 | 2.5 |
| EVA-1 end, 21 Apr 23:58:40 | 25.53° | 83.93° | 79.35, +1.55 | 76.0 / 322 | 2.1 |
| EVA-2 start, 22 Apr 16:33:35 | 33.79° | 82.03° | 70.92, +1.55 | 74.9 / 322 | 1.5 |
| EVA-2 end, 22 Apr 23:56:44 | 37.45° | 81.06° | 67.17, +1.55 | 74.5 / 323 | 1.3 |
| EVA-3 start, 23 Apr 15:25:28 | 45.10° | 78.63° | 59.31, +1.55 | 73.5 / 322 | 1.0 |
| EVA-3 end, 23 Apr 21:05:31 | 47.88° | 77.56° | 56.44, +1.55 | 73.2 / 322 | 0.9 |
| Liftoff, 24 Apr 01:25:47 | 50.00° | 76.66° | 54.23, +1.55 | 73.0 / 322 | 0.8 |

- **Other quantities.**
  - Local sunrise at the LM: 1972-04-19 21:10 UTC. Sunset: 1972-05-04 14:40 UTC.
  - Maximum solar elevation: 79.5° on 27 Apr 05:50, with the Sun in the north (sub-solar latitude +1.55°).
  - Elevation rate during the EVAs: 0.50°/h. Solar angular radius: 0.265°.
- **Discrepancy.** A planning value of 11.9–12° appears in some summaries; that was the pre-mission landing sun angle. The actual landing was delayed by about 6 h, so the real landing elevation was 14.7–14.8°. The Lawrence ALSJ table lists the as-flown EVA start and end elevations as 22.2–25.7°, 34.1–37.9° and 45.8–48.7° [read].
- **Gazebo directional light** (ENU world: x east, y north, z up; `<direction>` is the propagation direction, i.e. minus the to-Sun vector):
  - Landing: (−0.965, −0.068, −0.254).
  - EVA-1 start: (−0.924, −0.086, −0.374).
  - EVA-2 start: (−0.823, −0.115, −0.556).
  - EVA-3 start: (−0.692, −0.139, −0.708).

---

## 2. Small craters (0.2–50 m)

### 2.1 Size-frequency distribution

At these sizes the Cayley plains (about 3.9 Ga) are in crater **equilibrium** (steady state). The cumulative SFD has slope −2, and the "knee" to the steeper production function lies well above 50 m on old highland plains [read: Shoemaker & Morris 1970; Gault 1970 via Minton 2019; Xiao & Werner 2015].

| Source | N(≥D) per m² (D in m) | % of geometric saturation (1.54·D⁻²) | Notes |
|---|---|---|---|
| Shoemaker et al. (Surveyor; Ranger/Trask 1966) steady state, F = 10^10.9·D⁻² per 10⁶ km² | **0.079·D⁻²** | 5.2 % | Surveyor 7: 13 cm–3 m craters follow it [snippet]. NASA SP-8023 (1969) Figs. 5–6 use the same line, N ≈ 0.075–0.09·D⁻² from about 3 cm to about 50–200 m [read/digitised] |
| Xiao & Werner 2015 (several lunar surfaces) | 0.011–0.060·D⁻² | 0.69–3.9 % | Equilibrium slope about −2 [snippet] |
| Apollo 15 site (Robbins counts; Hirabayashi et al. 2017 fit) | 4.6×10³·r⁻¹·⁸ per km² (r = radius) → 0.016·D⁻¹·⁸ | 1.6 % at 10 m | [read] |
| Minton et al. 2019 | "a few percent of geometric saturation", ∝ r⁻² | — | [read abstract] |

**Recommendation.** N(≥D) = C·D⁻² with **C = 0.05 per m²** (range 0.03–0.08) for 0.2 m ≤ D ≤ 50 m. Confidence M. The classical value 0.079 is the upper end. Modern counts are mostly 0.02–0.06. No published Apollo 16-specific count at 0.2–50 m was found. A count on the NAC DTM or ortho for 20–200 m (site agent) would pin down C. Extrapolation below about 1 m (Surveyor-only data) is **L** confidence.

Expected numbers in a 100 m × 100 m core [measured, `calib_models.py`]:

| D ≥ | 0.2 m | 0.5 m | 1 m | 2 m | 5 m | 10 m | 20 m | 50 m |
|---|---|---|---|---|---|---|---|---|
| C = 0.05 | 12,500 | 2,000 | 500 | 125 | 20 | 5 | 1.3 | 0.2 |
| C = 0.079 | 19,860 | 3,180 | 794 | 199 | 32 | 7.9 | 2.0 | 0.32 |
| C = 0.03 | 7,500 | 1,200 | 300 | 75 | 12 | 3 | 0.75 | 0.12 |

The summed crater area from 0.2 to 50 m is 43 % (C = 0.05) to 69 % (C = 0.079) of the surface. Craters overlap, as expected in equilibrium.

### 2.2 Degradation states and their fractions

- Apollo 16 (Descartes; Mahanti et al. 2018; 2,568 craters, 35–200/250 m): class A (fresh) **a little under 3 %**, class C (degraded) **just over 70 %**, the rest class B. Mean d/D < 0.1, so the landscape is dominated by shallow, inverted-cone craters. The Cayley plains have more fresh craters than Taurus-Littrow, where 88 % are class C [read, LROC summary and abstract].
- Recommended mix for all D in 0.2–50 m: **A 5 % (3–10 %), B 25 % (20–30 %), C 70 % (60–75 %)**. Confidence M for ≥ 35 m. Confidence L below 35 m, where I extrapolate assuming self-similar degradation in equilibrium.
- A simple generator rule that reproduces this mix: relative freshness ρ = d/d_fresh = exp(−u), with u ~ U(0, 3). This gives 10 % with ρ > 0.73, 20 % with ρ between 0.4 and 0.73, and 70 % with ρ < 0.4 [derived]. A diffusion-based alternative: give each crater a degradation "κt" ∝ D² × U(0, ~0.06) and diffuse its profile. Fassett & Thomson (2014) give κ ≈ 5 m²/Myr at km scale; κ is scale dependent and much smaller at metre scale [snippet].

### 2.3 Shape: d/D, rim, profile and ejecta

| Parameter | Fresh (A) | Intermediate (B) | Degraded (C) | Source / conf. |
|---|---|---|---|---|
| d/D, D 30–100 m | **0.12** | 0.06–0.10 | 0.01–0.06 | Stopar et al. 2017 (fresh: 0.12 for 30–100 m, 0.15 for 100–200 m, 0.17 for 200–400 m, 0.2 for > 400 m) [read]; Mahanti 2018 mean < 0.1 [read] — M |
| d/D, D < 30 m | 0.10–0.17 (assume 0.13) | 0.05–0.09 | 0.01–0.05 | Below NAC resolution. Lunokhod-1 area max H/D ≈ 0.14 [snippet]. Lab and strength-regime craters are deeper (≤ 0.2) — **L, extrapolated** |
| d/D (SP-8023, pre-Apollo, Moore 1968) | 0.23–0.25 | 0.17–0.19 (young), 0.11–0.13 (mature) | ~0 (old) | [read]; superseded for D < 400 m by LROC — reference only |
| Rim height / D | 0.025–0.04 (South Ray 0.03–0.034; Pike 0.036) | 0.01–0.025 | 0–0.01 | SP-8023 Table IV: fresh 0.022–0.06, young 0.016–0.045, mature 0.008–0.03 [read]; South Ray [read+measured] — M |
| Interior profile | Paraboloid bowl. Max wall slope ≈ atan(4·d/D) = 26–31° for d/D 0.12–0.15, below the regolith repose angle of about 35° | Flattened bowl, wall 12–25° | Inverted cone, walls 2–12° | Mahanti 2018 [read]; geometry [derived] — M |
| Flat floor / bench (fresh only) | D/t > 4 gives flat floors; D/t > 8–10 gives concentric (bench) craters. With regolith t ≈ 7 m: flat-floored for D ≳ 28 m, concentric for D ≳ 60 m | — | — | Quaide & Oberbeck 1968 via Prieur et al. 2018 [snippet]; Freeman 1981 used this at Apollo 16 [read] — M |
| Ejecta thickness | t(r) = t_rim·(r/R)⁻³ for r ≥ R. Continuous to 2–3 R from the centre; rays beyond (South Ray rays reach about 10 km ≈ 15 D) | same shape, subdued | absent | McGetchin et al. 1973 [snippet]; Ulrich et al. 1981 preferred exponent −3.0 for South Ray [read] — M |
| Rim-height partition | Ejecta at the rim ≈ 50–70 % of rim height; the rest is structural uplift | | | South Ray: 10–12 m ejecta on a 20–22 m rim [read] — M |

### 2.4 Which craters are blocky (excavate rocks)?

- **Evidence.**
  - WC (40 m, 700 m S of LM) reached "bedrock" under 6.7 m of regolith. It has abundant blocks, about 16 % surface cover, the densest block field at the site [read, F/G].
  - Buster (90 m) has 90 % of its floor covered by blocky debris, with blocks up to 5 m. It penetrates about 18 m into a ridge [read, D1].
  - Spook (370 m) is old and subdued, with no blocks [read, D1].
  - The 3-cm radar shows the Cayley regolith is "freer of metre-sized boulders to depths of 10–15 m" [read, F].
- **Rule** [derived, M]. A crater of diameter D is blocky if it is class A or B and D ≳ 4–6·t. With t ≈ 7 m (range 3–15 m), the probability rises from 0 at D ≈ 20 m to about 1 at D ≥ 80 m. Class-C craters have lost their blocks: about half of metre-sized boulders are destroyed in "several tens of Ma" and 90–99 % in 200–300 Ma (Basilevsky et al. 2015) [snippet]. Diviner ejecta rock abundance decays as age^−0.46 (Ghent et al. 2014) [snippet].
- **Block populations around blocky craters.** Blocks are concentrated between R and 2R. The SP-8023 "nominal blocky crater" curve gives N(≥D) ≈ 1×10⁻²·(D/0.7 m)^−2.8 per m² in that annulus [read/digitised]. That is close to the North Ray rim model in Section 3. The largest block is roughly B_max ≈ 0.25–0.3·D^0.66 (D, B in m; Bart & Melosh 2010; coefficient **not verified**). This is consistent with North Ray (D ≈ 1 km, blocks to about 25 m) and Buster (90 m, 5 m blocks) — M–L.

### 2.5 Check of the crater model against the DTM (synthetic experiment)

`crater_terrain_check.py` and `detrended_compare.py` [measured]:
- Built 100 m × 100 m crater-only height fields at 5 cm with C = 0.025–0.16, fresh d/D 0.12 or 0.18, the degradation rule above, paraboloid interiors and r⁻³ ejecta.
- Measured them at full resolution, and after a NAC-DTM emulator (2 m block mean + Gaussian σ = 1 post).

Findings:
1. With C = 0.079 and fresh d/D = 0.18, the emulated-DTM directional RMS slope is 3.1° at 2 m and 2.6° at 10 m (detrended). That is close to the real DTM (2.9° and 2.5°). C = 0.05 gives 1.8° and 1.4°.
2. But crater-only terrain has the wrong slope **distribution**: it is flat between craters with steep walls. The gradient median is 0.2–0.8° against 2.4° for the real DTM, and the fraction above 10° is 7 % against 2.3 %. The real surface also carries undulating relief from larger, degraded craters and other long-wavelength topography.
3. Results are dominated by the few largest craters (20–50 m) present, so single 100 m tiles vary strongly between seeds.

Guidance for the generator [derived, M]:
- Use the real DTM for wavelengths ≳ 6–10 m, i.e. craters ≳ 10–20 m.
- Add synthetic craters only below that size (0.2–10/20 m) with C ≈ 0.05.
- Add fractal roughness (Section 5) for what remains.
- Re-measure ν(L) of the composite and tune it to the targets in Section 5. Do not double-count craters the DTM already resolves.

---

## 3. Rocks (1 cm – 3 m)

### 3.1 Data

**Apollo 16 surface photography** (areal cover by fragments, from photogrammetric counts):

| Location | Measured cover | Notes | Source |
|---|---|---|---|
| LM/ALSEP | 2–20 cm fragments cover **1.3–6 %, average ≈ 2 %**. Rocks > 10 cm cover **0.3–0.9 %** | "Blocks as large as 0.5 m are relatively common"; largest boulder (33 m N of ALSEP) "several metres across"; rocks not deeply buried, poorly filleted, some perched | Schaber 1981 [read] |
| Station 1 (Plum crater, 1.4 km W) | > 2 cm: 0.6–1.8 %; > 10 cm: 0.2 % | more eroded, more buried, less angular | Schaber [read] |
| Station 2 (Buster/Spook) | all fragments 1.6–2.6 % (average 2.0 %), mostly 5–10 cm; > 5 cm: 0.4–1.5 % (average < 0.8 %) | angular to subangular, perched or slightly buried, fillets rare | Schaber [read] |
| South Ray rays (traverse area) | < 1 % to 7 %; mostly < 5 cm; most frequent 2–5 cm | ray and inter-ray fragment concentrations are often similar | Holt 1981 / Muehlberger et al. 1972 [read] |
| Survey Ridge (bright ray) | up to 7 % of surface covered by fragments > 2 cm | | Holt [read] |
| **North Ray rim, station 11** (within 10 m of pan 18) | fragments ≥ 10 cm cover **4.3 %**; more than 20 % of counted fragments are > 20 cm; about 70 % of fragments rounded | block concentration on the rim was "considerably lower than anticipated"; fragments from < 1 m to 25 m | Ulrich 1981 [read] |
| North Ray station 13 (0.75 km SE, outer ejecta) | ≥ 10 cm: **0.5 %**, with about one-quarter as many fragments; 10 % > 20 cm | | Ulrich [read] |
| North Ray continuous ejecta in general | < 1 % except on the rim crest | | Ulrich et al. 1981 (G) [read] |
| WC crater rim (40 m, 700 m S of LM) | blocks occupy **nearly 16 %** | highest block density at the site | Ulrich et al. 1981 (G) [read] |

**Other constraints:**
- **Shape of the size distribution.**
  - Surveyor (Shoemaker & Morris) fragment distributions are power laws N = K·D^−γ. Surveyor 3, 5 and 6 are alike; Surveyors 1 and 7 have more coarse fragments. Coarse-block abundance correlates inversely with regolith thickness [snippet].
  - Cintala & McBride (1994/1995) compared Surveyor surface counts with Lunar Orbiter block counts (≥ 2.5 m). Large blocks follow **steeper** distributions than small fragments. Extrapolating surface power laws **overestimates** large blocks at 3 of 4 sites [read].
  - SP-8023 (1969) intercrater design model, digitised: about −2.4 below 2 mm, about −2.1 from 2 mm to 0.7 m, about −3.1 above 0.7 m (nominal). The "maximum" model is −1.8 (2 mm–0.2 m) then −3.0 [read].
- **Golombek & Rapp (1997) model** [read via Golombek et al. 2021].
  - F_k(D) = k·exp(−q(k)·D), with q(k) = 1.79 + 0.152/k. F is the cumulative fractional **area** covered by rocks of diameter ≥ D (D in m); k is the total rock cover.
  - It was built for Mars and Earth analogue sites (D ≳ 0.1 m). Golombek et al. note that power laws overestimate both large and small rocks there.
  - For Apollo 16 the exponential cannot match the data. Fitting k to the measured ≥ 10 cm cover (k = 0.0176 at the LM) under-predicts the 2–20 cm population by 3×, and blocks ≥ 0.5 m by about 7×. It also predicts essentially no ≥ 1 m blocks, which contradicts "0.5 m blocks relatively common" and "several-metre boulder".
  - So I use a power law at cm–dm scales with a steeper large-block tail, which is what Cintala & McBride recommend.
- **Cross-checks not used quantitatively.**
  - Diviner rock abundance is the fractional area of rocks large enough to be thermally distinct, about ≥ 1 m; it is typically < 1 % on the Moon (Bandfield et al. 2011) [snippet].
  - Chang'e-3 and -4 rover and descent-image rock counts (Di et al. 2016; Li & Wu 2018) were located but not read.

### 3.2 Recommended model

Cumulative number per m² of rocks with diameter (longest visible axis) ≥ D, as a **broken power law**: N(≥D) = A_i·D^(−b_i) on each segment, continuous at the breaks.

| Parameter | LM site (nominal) | Rocky: North Ray rim crest (station 11) |
|---|---|---|
| Segment 1 | 0.5 cm – 1 m: b₁ = **2.8** | 0.5–10 cm: b₀ = **2.5** (assumed) ; 10 cm – 1 m: b₁ = **2.3** |
| Segment 2 | > 1 m: b₂ = **3.5** | 1–25 m: b₂ = **3.0** |
| Anchor | N(≥1 m) = A = **3.78×10⁻⁴ m⁻²** (fixed so that 2–20 cm cover = 2.0 %) | N(≥0.1 m) = **1.04 m⁻²** (fixed so that ≥ 10 cm cover = 4.3 %) |
| Implied cover ≥ 2 / 5 / 10 cm / ≥ 1 m | 2.3 / 1.1 / 0.62 / 0.07 % | 9.4 / 6.0 / 4.3 / 1.2 % |
| Checks against data | ≥ 5 cm cover 1.1 % (station 2: 0.4–1.5 %) ✓; ≥ 10 cm 0.62 % (LM: 0.3–0.9 %) ✓ | N(≥20 cm)/N(≥10 cm) = 0.20 (">20 %") ✓ |
| Largest rock in a 100 m × 100 m tile | ~2–4 m (expected count ≥ 2 m: 0.33) | ~5–8 m (expected count ≥ 5 m: 0.4) |
| Confidence | M (2–20 cm H-M, ≥ 1 m L-M) | M for 10 cm–1 m; L below 10 cm and above 2 m |

**Expected counts per 100 m²** [measured, `calib_models.py`]:

| D ≥ | 2 cm | 5 cm | 10 cm | 20 cm | 50 cm | 1 m | 2 m |
|---|---|---|---|---|---|---|---|
| **LM site, nominal** | **2160** | **166** | **23.9** | **3.4** | **0.26** | **0.038** | **0.0033** |
| LM site, low (sparse inter-ray) | 1140 | 80 | 10.7 | 1.4 | 0.10 | 0.014 | 0.001 |
| LM site, high (ray patch) | 3920 | 362 | 59.6 | 9.8 | 0.91 | 0.15 | 0.016 |
| **Rocky (North Ray rim st. 11), nominal** | **5810** | **588** | **104** | **21** | **2.6** | **0.52** | **0.065** |
| Rocky, low (≥ 10 cm cover 2 %, b₁ 2.6) | 3590 | 363 | 64 | 10.6 | 0.98 | 0.16 | 0.020 |
| Rocky, high (≥ 10 cm cover 8 %, b₁ 2.1) | 8620 | 872 | 154 | 36 | 5.3 | 1.2 | 0.15 |
| Extreme block field (WC rim, 16 % cover) | ~21,600 | ~2190 | ~390 | ~79 | ~9.6 | ~1.9 | ~0.24 |
| North Ray station 13 (outer ejecta) | 2280 | 175 | 25 | 2.6 | 0.12 | 0.013 | 0.001 |
| *Compare:* Golombek model fitted to ≥ 10 cm cover, LM (k = 0.0176) | 659 | 147 | 33 | 3.9 | 0.04 | 1×10⁻⁴ | 0 |
| *Compare:* Golombek, rocky (k = 0.065) | 1290 | 387 | 130 | 33 | 2.4 | 0.09 | 0.0004 |
| *Compare:* SP-8023 1969 nominal intercrater | 1340 | 191 | 44 | 10 | 1.4 | 0.23 | 0.027 |

Where sources disagree: below 10 cm the rocky-variant numbers rest on an assumed slope. With b₀ = 2.8 (the LM-site value) the ≥ 2 cm count would be 9,400 per 100 m²; with b₀ = 2.3 it would be about 4,200. Above 2 m all lunar surface-photo power laws are extrapolations; orbital data say the tail is steeper.

**Totals in a 100 m × 100 m core** (performance planning): LM site ≈ 216,000 rocks ≥ 2 cm, 16,600 ≥ 5 cm, 2,390 ≥ 10 cm, 343 ≥ 20 cm, 26 ≥ 50 cm, 3.8 ≥ 1 m. Rocky variant ≈ 581,000 / 59,000 / 10,400 / 2,110 / 257 / 52. Pebbles will need to be merged into tile meshes or instanced, not spawned as individual models.

### 3.3 Spatial arrangement

- **LM site.** Mostly random (Poisson) placement, modulated by ray patchiness. Measured cover varies 1–7 % between photos, so multiply the local density by a log-normal field with σ_ln ≈ 0.6 and a correlation length of about 20–50 m [derived]. Rays are elongated radial to South Ray (azimuth from the LM ≈ 200°, i.e. bands trending NNE–SSW).
- **Craters.** Around each blocky crater (Section 2.4), add blocks between R and 2R with about 2–5× the local intercrater density and the largest block from B_max. Bias blocks toward the rim crest and the floor (Buster: floor 90 % covered). Class-C craters carry no extra blocks.
- **North Ray rim.** Blocks are clustered: the House Rock and Outhouse Rock cluster, the White breccia boulder group, and boulder-free gaps. SP-8023 notes paths ≥ 50 m wide that are relatively block-free even in dense block fields [read].

---

## 4. Rock shape and setting

| Property | Recommended | Source / conf. |
|---|---|---|
| Axial ratios (impact fragments) | **b/a ≈ 0.7, c/a ≈ 0.5** (a:b:c ≈ 2:√2:1); spread b/a 0.5–0.9, c/a 0.3–0.7 | Fujiwara 1978 (0.73, 0.50); Capaccioni 1984/86 (0.7, 0.5); Michikami 2016 (0.7, 0.5 for disruption; c/a 0.2 for cratering spalls) [snippet] — M |
| Longest/shortest dimension | 1:1 to 1:5 | SP-8023 "standard lunar block" [read] — M |
| Exposed height | **≈ 0.5·D** as a design value (SP-8023 mobility rule). Demidov & Basilevsky (2014) measured H/D on Lunokhod and Apollo panoramas; their value was not verified here | [read] / [not verified] — M |
| Angularity | LM/ALSEP: "highly angular to subround", many glass-coated. Station 2: angular to subangular. Station 1 (older): less angular, more eroded. North Ray rim: ~70 % rounded; dark-matrix breccias angular and jointed, light-matrix breccias rounded and crudely jointed | Schaber; Ulrich [read] — H |
| Burial (fraction of rock height below the surface) | Young South Ray-derived rocks (LM, station 2): 0–20 %, median ~10 %, many perched. Older rocks (station 1, inter-ray): 20–60 %. Recommend sampling burial from Beta(1.5, 6) for young, Beta(3, 4) for old | Schaber "not deeply buried … some perched" [read]; distributions [derived] — L |
| Fillets | LM area: poorly filleted or none. Old and friable rocks: fillet height up to ~0.3–0.5 of exposed height and width ~0.5–1× height. North Ray friable light-matrix boulders: regolith thickens to **15 cm or more** in fillets. Dark-matrix boulders: few fillets | Schaber; Ulrich [read] — M |
| Clustering | See Section 3.3: rays, crater annuli, boulder clusters | — |
| Rock types and colours | Light-grey to white anorthositic and light-matrix breccias dominate: B1 25 % and C2 metaclastic 20 % of LM/ALSEP samples; 70–90 % of rocks at North Ray are light-coloured. Dark-matrix breccias and impact melts (dark grey to black) make up 10–30 % (North Ray), e.g. House/Outhouse Rock and the 5 m Shadow Rock. Black glass coats and splashes are common on LM-area rocks and boulder faces (shiny, specular) | Schaber Table 1; Ulrich [read] — H |
| Rock normal albedo (visible) | 0.18–0.51; light-matrix breccias are brightest | Holt 1981 [read] — H |
| Rock bulk density | **2.4 g/cm³** (2.2–2.9; feldspathic breccias 2.2–2.6 with 10–20 % porosity; crystalline/melt about 2.8–2.9) | Kiefer et al. 2012 [snippet]; SP-8023 2.8–2.9 [read] — M |
| Surface texture | Zap pits (µm–cm), rounded tops on older rocks, spalled faces, jointing and fractures | Apollo PP1048 [read] — M |

---

## 5. Regolith micro-topography and roughness

### 5.1 Measurements

| Scale | Value | Site / method | Source |
|---|---|---|---|
| 85 µm – 8.5 cm | RMS slope at 1 mm: **16° ± 4° (mare), 25° ± 1° (Fra Mauro highland ejecta)**. At 1 cm: **8.1° ± 2.4° (mare), 12.5° ± 2.0° (Fra Mauro)**. Fractal dimension 2.31 ± 0.06, i.e. **H ≈ 0.69** | Apollo 11/12/14 closeup stereo (ALSCC) | Helfenstein & Shepard 1999 [snippet] |
| 7 mm – dm | Bidirectional slope at 7 mm: median ≈ 10°, can exceed 40°. RMS height in 125 mm windows: median ≈ 4 mm (1–18 mm). **H ≈ 0.6–0.85** | Chang'e-4, 5 mm DTM | Guo et al. 2021 [read abstract] |
| 2–400 m | Deviogram ν(L) (LM 1 km box): 0.135 m at 2 m, 0.63 m at 10 m, 2.16 m at 40 m, 4.7 m at 100 m, 10.7 m at 400 m. **H = 0.96 (2–10 m, smoothing artefact), 0.89 (10–40 m), 0.83 (40–140 m), 0.54 (140–400 m)**. Detrended RMS height (median over windows): 10 m 2.9 cm (lower bound), 20 m 7.9 cm, 50 m 24 cm, 100 m 54 cm | NAC DTM | [measured] |
| 17 m – 2.7 km | Median Hurst exponent: **0.95 (highlands), 0.76 (maria)**. Transitions typically near 1 km | LOLA | Rosenburg et al. 2011 [snippet] |
| 1 m – 10 km | Design "mean slope": nominal (rough/hummocky upland) ~9° at 1 m, ~5° at 10 m, ~3° at 100 m | pre-Apollo model | SP-8023 [read] |

The Apollo 16 Cayley plains are smoother than average highlands at 100–400 m (H ≈ 0.54 against 0.95 in the highland median). The LROC/LOLA highland medians include rugged massifs.

### 5.2 Recommended targets (LM site, plane-detrended, directional)

A single self-affine law fits from 1 cm to about 40 m within the uncertainties:

**ν(L) = 0.069 m × (L / 1 m)^0.8**, where ν is the RMS height difference at lag L. Equivalently RMS slope = atan(ν/L).

| L | 1 cm | 3 cm | 10 cm | 30 cm | 1 m | 3 m | 10 m | 20 m |
|---|---|---|---|---|---|---|---|---|
| RMS slope (target) | 10–12° | 8.7° | 6.7° | 5.3° | 4.0° | 2.9° | 2.5° | 2.2° |
| Check | Fra Mauro 12.5°, mare 8.1°; CE-4 ~10° at 7 mm | — | — | — | — | — | DTM detrended 2.49° | DTM detrended 2.07° |

- Hurst exponent: **H = 0.8** (0.7–0.85) for 1 cm–40 m. H ≈ 0.55 at 100–400 m (DTM). Confidence M.
- RMS height (detrended) in a window W: **≈ 2.4–3.5 mm at W = 12.5 cm, 1.3–1.6 cm at 1 m, ~9 cm at 10 m** (fBm with these parameters, `fbm_check.py`). These match Guo et al.'s 4 mm median at 12.5 cm within a factor of 1.6. They exceed the DTM's 2.9 cm at 10 m because the DTM is smoothed.
- Spectral synthesis (2-D isotropic PSD, f in cycles/m): **S(f) = 2.5×10⁻⁵ m⁴ · f^−3.6** for H = 0.8. For H = 0.7 use S₀ = 3.6×10⁻⁵ and exponent −3.4. This is the *total* roughness target. When craters (0.2–10 m) and rocks are added explicitly, reduce the fractal amplitude: crater-only terrain with C = 0.05 already gives 4.3° at 1 m and 5.7° at 10 cm at full resolution. Tune the composite until ν(L) matches.
- **Rocky variant** (North Ray ejecta, DTM): directional RMS slope 9.8° at 2 m and 8.8° at 10 m (not detrended), H ≈ 0.93–0.94 at 2–40 m. Use about 2.5× the LM ν at L ≥ 2 m. Assume about 1.5× at cm scales (L, no data).
- **Collision ground.** The planned 20–30 cm low-pass is consistent with the wheel. A rigid wheel of r = 0.165 m bridges a 1 cm-deep pit narrower than 2·√(2rδ) ≈ 11 cm [derived]. Features under about 5 cm mainly cause vibration, not traction changes.

### 5.3 Small-scale features worth modelling

- **Small fresh craters** down to cm size, continuing the D⁻² equilibrium; Surveyor 7 counted 13 cm–3 m craters on that line. Fresh ones have sharp raised rims and sometimes glass-lined pits.
- **Rock fragments and pebbles** (Section 3). Young ones are perched; older ones are partly buried with fillets.
- **Soil clods**: weak aggregates of about 0.5–5 cm, common on fresh crater rims and ejecta [recalled, L].
- **Light subsurface soil** 1–3 cm below darker grey soil, exposed by any disturbance: wheel tracks, footprints, small fresh craters, the LM blast zone [read].
- **Elephant-hide / patterned ground** (cm-relief lineations) on slopes ≳ 10–15°, e.g. Stone Mountain [recalled, L].
- **Tracks.** LRV tracks are 0–5 cm deep (average 1.25 cm) with crisp chevron imprints. Footprints are a few cm deep [read, Mitchell et al. 1972].

---

## 6. Photometry and appearance (for later camera and LiDAR-intensity work)

### 6.1 Albedo and spectral reflectance

| Quantity | Value | Source / conf. |
|---|---|---|
| Normal albedo (visible, zero phase): LM, stations 1, 2, 6, 8, 9 | **0.16–0.19** | Holt 1981 (down-sun Hasselblad photometry) [read] — H |
| Normal albedo: North Ray ejecta, stations 11/13 | 0.21–0.32 (rim deposits 0.25–0.30) | Holt [read] — H |
| Normal albedo: Stone Mountain, stations 4/5 | 0.14–0.15 | Holt [read] — H |
| Normal albedo: rocks | 0.18–0.51 (brightest: North Ray light-matrix breccias) | Holt [read] — H |
| Normal albedo: brightest fresh crater walls | 0.52 (North Ray), 0.60 (South Ray rim) | Holt [read] — H |
| Mature regional soil (500 km² map) | 0.11–0.15 over > 58 % of area; 0.15–0.17 near young craters; 0.17–0.25 rays | Holt [read] — H |
| LOLA 1064 nm normal albedo (zero phase) | typical highlands **≈ 0.30**, maria ≈ 0.15. V-band/1064 nm ratio 0.583 (Clementine proxy) | Lucey et al. 2014 [read] — M (absolute calibration from Kaguya MI) |
| Apollo 16 standard soil 62231, lab bidirectional reflectance factor (i = 30°, e = 0°) | 415 nm 0.108 · 550 nm 0.142 · 650 nm 0.161 · 753 nm 0.178 · 899 nm 0.189 · 1000 nm 0.200 · 1064 nm ~0.209 · 1550 nm ~0.265 · 2500 nm ~0.345 | Pieters 1999 Table 2 [read]; > 1 µm digitised from Fig. 1 — H (lab) |
| Spectral ratios (62231) | R905/R550 = **1.33**; R1550/R905 = **1.40**; R905/R1064 = 0.906 | [measured from the above] — M |
| Lab against in-situ | Lab 62231 is about 33 % brighter than what fits Diviner in situ (compaction, maturity). An ASTER study finds the site 0.5–0.55× the lab sample | [snippet] — M |

**LiDAR-relevant zero-phase normal albedo** [derived, M]:
- **905 nm ≈ 0.25** (0.21–0.29) for LM-area soil. This is 0.175 × 1.33, and also 0.30 × 0.906 = 0.27 from LOLA.
- **1550 nm ≈ 0.35** (0.29–0.41).
- North Ray ejecta soil: ≈ 0.30–0.45 at 905 nm.
- Rocks: ≈ 0.22–0.6 at 905 nm. Rocks have weaker opposition surges, so their contrast against soil at zero phase is smaller than at 30–50° phase.
- The Robin W is a **905 nm** sensor (datasheet; 70 m range at 10 % Lambertian reflectivity) [read].

### 6.2 Colour (approximate linear RGB)

The 62231 spectrum (380–780 nm) was integrated with CIE 1931 colour-matching functions (Wyman et al. 2013 analytic fit) under a 5778 K blackbody Sun, converted to linear sRGB and white-balanced so a flat spectrum is neutral (`calib_models.py`) [measured]:
- **Tint R:G:B = 1.12 : 1.00 : 0.82**: a slightly warm, reddish grey.
- Lab reflectance-factor colour: (0.159, 0.141, 0.116).
- **In-situ diffuse base colour** (≈ 0.75 × lab, at i = 30°, e = 0°): **(0.12, 0.107, 0.088)** linear, ≈ sRGB (97, 92, 84)/255. Use this for a Lambert/PBR renderer.
- Zero-phase brightness corresponds to (0.20, 0.175, 0.144).
- North Ray ejecta and light breccias: ×1.3–1.7 brighter and a little less red. Dark-matrix breccias and glass: ×0.4–0.6.

### 6.3 Photometric behaviour (Hapke-type summary)

- **Strong backscatter and an opposition surge.** Rays and light soils brighten most toward zero phase. Rock fragments behave more like Lambertian surfaces; relative to soil they are brightest at 30–50° phase, and they reduce the opposition effect [read, Holt].
- **No limb darkening at full phase.** A Lommel–Seeliger-like disc function means that at zero phase brightness is nearly independent of incidence angle [recalled, H].
- **Hapke parameters** (for an advanced shader; values via a secondary table):
  - Moon average terrain, LROC WAC: w ≈ 0.36, B_S0 ≈ 2.0, h_S ≈ 0.08 (Sato et al. 2014). The WAC maps fix θ̄ = 23.657°, B_C0 = 0 and φ = 1 [README read; values snippet].
  - Helfenstein & Veverka 1987: w ≈ 0.21, h ≈ 0.07, θ̄ ≈ 20° [snippet].
  - Confidence M–L.
- **Polarisation** up to 30 % near 120° phase for some rocks (SP-8023) [read].

### 6.4 Implications for LiDAR simulation

The LiDAR is monostatic, so it always observes at zero phase. Regolith returns therefore do **not** fall off as cos θ.
- At a 70° incidence (ground about 2 m ahead of a sensor about 0.7 m high), lunar soil returns about A_n ≈ 0.25.
- A Lambertian of the same normal albedo would return 0.25·cos 70° ≈ 0.09.
- So, compared with an Earth-calibrated Lambertian assumption, grazing returns are about 3× brighter [derived, M].
- No atmosphere means no range attenuation and no backscatter from air, but the solar background is about 40 % above Earth sea level at 905 nm [recalled, M].

---

## 7. Regolith mechanics (for a rigid-body simulator)

### 7.1 Soil properties

| Property | Value | Source / conf. |
|---|---|---|
| Bulk density | **1.50 ± 0.05 g/cm³ (0–15 cm)**, 1.58 (0–30 cm), 1.74 (30–60 cm). A newer fit gives ~1.30 at the surface rising to 1.92 below 1 m | Carrier et al. 1991 (Lunar Sourcebook) [recalled]; Connolly & Carrier 2023 [snippet] — H |
| Apollo 16 densities | 1.40–1.80 g/cm³ (penetrometer, cores). Footprint porosity: 45.0 % inter-crater, 46.1 % crater rims, 43.1 % LM area | Mitchell et al. 1972 (A16 PSR §8) [read] — H |
| Cohesion | **0.52 kPa (0.44–0.62) at 0–15 cm**; 0.90 kPa at 0–30 cm; 3.0 kPa at 30–60 cm. Apollo 16 range 0.1–1.0 kPa. Station 4: 0.6 kPa | Sourcebook [recalled]; Mitchell 1972 [read]; Yutu-2 reference value c = 520 Pa [read] — H |
| Internal friction angle | **42° (41–43) at 0–15 cm**; 46° at 0–30 cm; 54° at 30–60 cm. Apollo 16 range 30–50°; station 4: 46.5° | same — H |
| Bekker parameters (LRV design soil) | n = 1.0, k_φ ≈ 820 kN/m^(n+2), k_c ≈ 1.4 kN/m^(n+1), c ≈ 0.17 kPa, φ ≈ 35° | Costes et al. 1972 [recalled]; Yutu-2 used N = 1, K_s = 820 kPa/m^N [read] — M |
| Yutu-2 (Chang'e-4, far side) | φ 21.5–42°, c 0.52–3.15 kPa, wheel–soil external friction angle 8.3–16.5°, bearing capacity ≈ 4 kPa | Ding et al. 2024 [read] — M |
| Lunokhod-1 | bearing strength 0.2–1.0 kg/cm² (mode 0.34); vane shear 0.02–0.09 kg/cm² (mode 0.048) | Soviet results via NASA reports [snippet] — M |
| Cone-penetration gradient from LRV tracks | ≈ 840 kN/m²/m (shallow) | Mitchell 1972 [read] — M |

### 7.2 Observed wheel behaviour

| Vehicle | Sinkage | Slip and slopes | Source |
|---|---|---|---|
| Apollo 16 LRV | tracks **0–5 cm, average 1.25 cm**; deepest on rims of small fresh craters | "excellent traction", sharp chevron imprints, slip "minimal". Driven on slopes up to **18°**; tested capability 19–23° (crew felt near the limit on the station 4 slope). Max speed 17 km/h | Mitchell et al. 1972 [read] |
| Yutu-2 | average **≈ 8 mm**, ≤ 15 mm (≥ 5 mm elsewhere) | mostly *skid* (slip −0.075 to 0); side-slip 0.5–3°. A separate study reports slip 10–20 % at three test sites | Ding et al. 2024 [read]; Jilin Univ. study [snippet] |
| North Ray rim (astronauts on foot) | "sinking in about six inches" on friable rim slopes; ≤ 1 cm elsewhere | — | Ulrich 1981 [read] |

**Prediction for the A300** [derived, M]. Assume about 100 kg, so W ≈ 40 N per wheel at 1.62 m/s². With b = 0.114 m, D = 0.33 m and the LRV Bekker soil (n = 1, k_eq ≈ 832 kN/m³):
- Sinkage z ≈ 1.1 cm.
- Compaction (rolling) resistance ≈ 5 N per wheel, i.e. R/W ≈ 0.13. A simpler estimate, √(z/D)/(n + 1), gives about 0.09.

### 7.3 Low gravity

- **Same wheel load, different gravity.** At an identical wheel load, sinkage under lunar g is predicted 3.6–7.3× larger than on Earth for n = 0.4–0.9 (Wong 2012) [read]. Earth tests at 1/6 load therefore underpredict lunar sinkage.
- **Measured at 1/6 g.** Parabolic-flight tests with GRC-1 found drawbar pull 24–44 % *higher* at 1 g than at 1/6 g after matching cone index, so Earth tests overpredict traction (Daca et al. 2022) [snippet].
- **Same mass, DEM.** A 2025 DEM study found the traction *coefficient* (DP/W) at lunar g about equal to Earth's for the same mass, with forces scaling by the gravity ratio [snippet].
- **Conclusion.** Use traction coefficients no higher than Earth's for loose dry sand. Absolute traction forces are 1/6 of Earth's.

### 7.4 Recommended rigid-body contact parameters (Bullet)

| Parameter | Value | Rationale / conf. |
|---|---|---|
| Longitudinal Coulomb µ (`friction`) | **0.40** (0.30–0.50) | Peak net traction coefficient ≈ tan(19–23°) = 0.34–0.42 from LRV slope capability; regolith φ ≈ 42° but net DP/W is lower — M–L |
| Lateral µ (`friction2`) | **0.30** (0.20–0.45) | Yutu-2 wheel–soil external friction 8–16.5° (tan 0.15–0.30) plus bulldozing; lower lateral µ eases skid-steer turning — L |
| Rolling resistance coefficient | 0.08 (0.05–0.13), if implementable | Bekker compaction estimate — M |
| Expected sinkage (for visual offset) | ~1 cm; up to 5 cm on fresh crater rims | LRV, Yutu-2 — M |
| Max traversable slope target | ~20° | LRV — M |

The current repo worlds use µ = 0.8 (`lunar_surface.world`, Earth-g A/B test) and µ = 0.2 (`flat_bullet_test.sdf`). 0.8 is above any plausible effective regolith traction coefficient. The rover agent should validate turning and slope climbing with the values above.

---

## 8. Gravity and other facts a lunar navigation simulation should get right

| Item | Value / note | Conf. |
|---|---|---|
| Gravity at the LM | **g = 1.6242 m/s²** (GM = 4902.800 km³/s², r = 1737.408 km). Centrifugal correction −1.2×10⁻⁵. Local gravity anomalies are ≲ ±0.003 m/s². SP-8023 lists 1.62. **Use 1.62 or 1.624**; the 0.25 % difference is negligible | H [measured/recalled GM] |
| Lunar rotation | 2.66×10⁻⁶ rad/s (0.55°/h). Gyrocompassing is impractical; heading needs sun or star sensors or map matching | H |
| Magnetic field | No global field; crustal fields are a few hundred nT at most (North Ray recorded the largest values of Apollo 14–16). **Do not fuse a magnetometer for heading.** No GNSS | H |
| Sun motion | Elevation changes about 0.5°/h (12°/Earth day). Static lighting is fine for hour-scale runs | H |
| Sun disc | 0.53° diameter, so penumbrae are about 9 mm wide per metre of occluder–ground distance | H |
| Ambient light | No sky. Shadow fill comes only from sunlit terrain (≲ 1–3 % of direct for vertical faces, ~0 for horizontal surfaces on flat ground) and earthshine (~10⁻⁵–10⁻⁴ of sunlight; Earth at about 73–77° elevation, NW). Matters for cameras and VO, not for the LiDAR | M |
| Curvature | Height drop d²/2R: 2.9 mm at 100 m, 6.5 mm at 150 m (LiDAR max), 7 cm at 500 m, **29 cm at 1 km**. Apply it to the 1 km far-field ring if the DTM is mapped to a flat world. Geometric horizon for a 0.7 m sensor: 1.56 km | H |
| LiDAR sampling (current sim, 0.01 rad, sensor assumed ~0.7 m above ground) | Grazing angle and along-range ground spacing: 2 m → 19°, 6 cm; 5 m → 8°, 36 cm; 10 m → 4°, 1.4 m; 20 m → 2°, 5.7 m. Cross-range spacing 1 cm per metre of range. **1 cm detail is visible only within ~2–3 m and 2–5 cm pebbles within ~5 m.** Beyond ~10 m the grazing angle falls below the local RMS slopes at 0.1–1 m (4–7°), so returns come mainly from facets facing the sensor, rocks and crater rims, with long range shadows behind them | H [derived] |
| Dynamics at low g | Froude similarity: 0.8 m/s on the Moon behaves like about 2.0 m/s on Earth. Expect more wheel hop and pitch over 5–20 cm obstacles, longer braking, and lower absolute traction | H |
| DTM artefacts | "Boxes" and seams up to ~1 m and faceted patches in the NAC DTM create false steps and slopes. Filter them before tiling | H |
| Optional human artefacts | The LM descent stage, ALSEP, LRV, tracks and the bright LM blast zone exist at the site today. Omit them for a "natural" world | — |

---

## 9. Where sources disagree, and where values are extrapolated

1. **Crater density.** The classical steady state (0.079·D⁻²) is about 1.3–7× above modern counts (0.011–0.060). No Apollo 16 count exists at 0.2–50 m, so the choice C = 0.05 is a compromise. Below 1 m, only Surveyor data support D⁻².
2. **Fresh d/D of small craters.** Pre-Apollo models used 0.23–0.25. LROC measurements give 0.12–0.17 for D < 400 m. Below 30 m there are no orbital measurements; I extrapolated 0.10–0.17.
3. **Degradation fractions** below 35 m are extrapolated from Mahanti et al. (35–250 m).
4. **Rock SFD.**
   - The LM model rests on areal-cover data (2–20 cm and > 10 cm) and one qualitative statement about 0.5 m blocks.
   - The slope 2.8 is derived from cover ratios, not from published counts.
   - Golombek's exponential, a power law and SP-8023 differ by 2–3× at 2 cm and by orders of magnitude at ≥ 1 m.
   - Above 2 m every value is an extrapolation steepened per Cintala & McBride.
   - The rocky variant below 10 cm assumes b₀ = 2.5, uncertain by about ±0.3 (factor ~2 at 2 cm).
5. **Burial and fillet distributions** are judgement calls constrained only by qualitative field descriptions.
6. **Roughness.**
   - The cm-scale values come from Apollo 11/12/14 and Chang'e-4, not from Apollo 16 (the closeup camera did not fly on Apollo 16).
   - The 1 cm–40 m single power law is a fit through two anchors.
   - DTM values below about 6–10 m are smoothed.
7. **Photometry.**
   - Lab 62231 against in-situ brightness differ by about 25–50 %.
   - The LOLA absolute calibration depends on Kaguya MI.
   - The 905/1550 nm normal albedos are spectral-ratio extrapolations.
   - The Hapke values are from secondary listings.
8. **Mechanics.**
   - The Lunar Sourcebook values were recalled from memory, but the Yutu-2 paper's reference values match them (c = 520 Pa, φ = 42°, K_s = 820).
   - The effective rigid-body µ is a modelling surrogate with no direct measurement.
   - Gravity scaling of traction is disputed: Wong 2012, Daca 2022 and the 2025 DEM study disagree.
9. **Landing sun angle.** 14.7° is the as-flown value; 11.9–12° was the planned value.
10. **Largest-block relation.** The coefficient (0.25–0.3·D^0.66) was not verified; it is consistent with North Ray and Buster.

---

## 10. Reproducibility

Scripts and outputs are in `/home/nargon/togo_assets/lunar/experiments/science/`. Run them from WSL with `/home/nargon/.venvs/lunar_world/bin/python` unless noted.

| Script | What it does | Outputs |
|---|---|---|
| `dtm_stats.py` | LM pixel and elevation, gravity, slope statistics at 2–100 m, deviogram, Hurst fits, RMS heights for LM boxes (100 m–2 km), crater centre and rim search, ejecta-annulus statistics | `dtm_stats.json` (run 18.7 s) |
| `detrended_compare.py` | 100 m-tile plane-detrended DTM statistics; synthetic crater terrains through a NAC-DTM emulator | `detrended_compare.json` / `.txt` |
| `crater_terrain_check.py` | Synthetic crater-only terrains (C = 0.025–0.079, fresh d/D 0.12/0.18), full-resolution and DTM-emulated slopes | `crater_terrain_check.json` / `.txt` |
| `calib_models.py` | Rock broken-power-law models, Golombek comparisons, SP-8023 digitised curves, crater counts, 62231 colour | `calib_models.json` / `.txt`, `calib_rocky_final.txt` |
| `fbm_check.py` | fBm calibration: PSD amplitude for ν(1 m) = 0.069 m; RMS slopes 1 cm–10 m; RMS height in windows | `fbm_check.txt` |
| `sun_geometry.py` (system `/usr/bin/python3`) | Meeus Sun/Moon/selenographic geometry; ALSJ comparison; sunrise, sunset and noon | console output, reproduced in Section 1.5 |
| `pdftext2.py`, `pdfimages.py` | Text and scanned-page extraction used to read PDFs (no external dependencies) | — |

South Ray morphometry used an inline variant of `dtm_stats.py` with centre (−9.1474, 15.3826). No Gazebo or ROS processes were started. No packages were installed.

---

## 11. References (with URLs)

**Apollo 16 site, geology and field data**
- Wagner R.V. et al. (2017) Coordinates of anthropogenic features on the Moon. *Icarus* 283, 92–103. NSSDC table: https://nssdc.gsfc.nasa.gov/planetary/lunar/lunar_sites.html ; LROC table: https://www.lroc.asu.edu/data/support/downloads/2016_LROC_Coordinates_of_Human_Features.pdf
- Ulrich G.E., Hodges C.A., Muehlberger W.R. (eds.) (1981) *Geology of the Apollo 16 area, central lunar highlands*. USGS Prof. Paper 1048. https://pubs.usgs.gov/publication/pp1048 ; chapters at https://www.apollojournals.org/alsj/a16/a16profpaper.htm
  - D1 Schaber G.G., Field geology of the central region: https://www.apollojournals.org/alsj/a16/D1_field_geo.pdf
  - D2 Ulrich G.E., Geology of North Ray crater: https://www.apollojournals.org/alsj/a16/D2_geology_north_ray.pdf
  - F Freeman V.L., Regolith of the Apollo 16 site: https://www.apollojournals.org/alsj/a16/A16PP-F-Regolith.pdf
  - G Ulrich, Moore, Reed, Wolfe, Larson, Ejecta distribution model, South Ray crater: https://www.apollojournals.org/alsj/a16/A16PP-G-SouthRayEjecta.pdf
  - H Holt H.E., Optical properties at the Apollo 16 landing site: https://www.apollojournals.org/alsj/a16/A16PP-H-Optical%20Properties.pdf
- Mitchell J.K. et al. (1972) Soil mechanics. *Apollo 16 Preliminary Science Report*, NASA SP-315, §8. https://an.rsl.wustl.edu/apollo/data/a16/station4/srp/docs/soilmechanics_psr.pdf
- LROC NAC_DTM_APOLLO16 README: https://pds.lroc.im-ldi.com/data/LRO-L-LROC-5-RDR-V1.0/LROLRC_2001/DATA/SDP/NAC_DTM/APOLLO16/NAC_DTM_APOLLO16_README.TXT ; Henriksen et al. (2016) *Icarus*, doi:10.1016/j.icarus.2016.05.012
- ALSJ sun and Earth angles (J. Scotti): https://www.nasa.gov/wp-content/uploads/static/history/alsj/Scotti_AltAz.txt ; B. Lawrence sun-angle table: https://www.apollojournals.org/alsj/sunangles.html
- Meeus J. (1998) *Astronomical Algorithms*, 2nd ed. (Ch. 25, 47, 53), Willmann-Bell.

**Craters**
- NASA SP-8023 (1969) *Lunar Surface Models*. https://ntrs.nasa.gov/api/citations/19700009596/downloads/19700009596.pdf
- Shoemaker E.M., Morris E.C. (1970) Physical characteristics of the lunar regolith determined from Surveyor television observations. *Radio Science* 5, 129. https://pubs.usgs.gov/publication/70227167
- Xiao Z., Werner S.C. (2015) Size-frequency distribution of crater populations in equilibrium on the Moon. *JGR Planets* 120, 2277. doi:10.1002/2015JE004860
- Minton D.A. et al. (2019) The equilibrium SFD of small craters … *Icarus* 326, 63. https://arxiv.org/abs/1902.07746
- Hirabayashi M. et al. (2017) An analytical model of crater count equilibrium. https://ar5iv.arxiv.org/html/1701.00471
- Mahanti P. et al. (2018) Small lunar craters at the Apollo 16 and 17 landing sites – morphology and degradation. *Icarus* 299, 475. doi:10.1016/j.icarus.2017.08.018 ; LROC summary: https://lroc.im-ldi.com/images/1009
- Stopar J.D. et al. (2017) Relative depths of simple craters and the nature of the lunar regolith. *Icarus* 298, 34. doi:10.1016/j.icarus.2017.05.022 ; LROC summary: https://lroc.im-ldi.com/images/975
- Moore H.J., Boyce J.M., Hahn D.A. (1980) Small impact craters in the lunar regolith. *Moon & Planets* 23, 231. https://pubs.usgs.gov/publication/70012230
- Prieur N.C. et al. (2018) *JGR Planets* (crater morphology and regolith thickness; Quaide & Oberbeck 1968 thresholds). https://www.duo.uio.no/bitstream/10852/67686/2/Prieur_et_al-2018-Journal_of_Geophysical_Research__Planets.pdf
- McGetchin T.R., Settle M., Head J.W. (1973) Radial thickness variation in impact crater ejecta. *EPSL* 20, 226.
- Fassett C.I., Thomson B.J. (2014) Crater degradation on the lunar maria. *JGR Planets* 119, 2255.
- Bart G.D., Melosh H.J. (2010) Distributions of boulders ejected from lunar craters. *Icarus* 209, 337.

**Rocks**
- Cintala M.J., McBride K.M. (1994) Block distributions on the lunar surface … LPSC XXV, 261. https://ntrs.nasa.gov/api/citations/19940030913/downloads/19940030913.pdf ; NASA TM-104804 (1995): https://ntrs.nasa.gov/citations/19960011631
- Golombek M.P., Rapp D. (1997) *JGR* 102, 4117, doi:10.1029/96JE03319 ; model restated in Golombek M. et al. (2021) *Earth & Space Science*, doi:10.1029/2021EA001959
- Bandfield J.L. et al. (2011) Lunar surface rock abundance … Diviner. *JGR* 116, E00H02, doi:10.1029/2011JE003866
- Ghent R.R. et al. (2014) Constraints on the recent rate of lunar ejecta breakdown. *Geology* 42, 1059. https://repository.si.edu/items/edc681df-3456-497b-a26b-12924914fd2f
- Basilevsky A.T. et al. (2015) Survival times of meter-sized rock boulders on the surface of airless bodies (NTRS record: https://ntrs.nasa.gov/citations/20150001903)
- Di K. et al. (2016) Rock size-frequency distribution analysis at the Chang'E-3 landing site. *PSS* 120, 103. doi:10.1016/j.pss.2015.11.012
- Li Y., Wu B. (2018) Analysis of rock abundance on lunar surface … *JGR Planets* 123, 1061. doi:10.1029/2017JE005496
- Michikami T. et al. (2016) Fragment shapes in impact experiments … *Icarus*. https://oro.open.ac.uk/44943 ; Fujiwara A. et al. (1978) *Nature* 272, 602.
- Kiefer W.S. et al. (2012) The density and porosity of lunar rocks. *GRL* 39, L07201.

**Roughness**
- Helfenstein P., Shepard M.K. (1999) Submillimeter-scale topography of the lunar regolith. *Icarus* 141, 107. doi:10.1006/icar.1999.6160
- Guo D., Fa W., Wu B., Li Y., Liu Y. (2021) Millimeter- to decimeter-scale surface roughness of the Moon at the Chang'E-4 exploration region. *GRL* 48, e2021GL094931. https://research.polyu.edu.hk/en/publications/millimeter-to-decimeter-scale-surface-roughness-of-the-moon-at-th/
- Rosenburg M.A. et al. (2011) Global surface slopes and roughness of the Moon from LOLA. *JGR* 116, E02001. https://pgda.gsfc.nasa.gov/products/70
- Shepard M.K. et al. (2001) The roughness of natural terrain. *JGR* 106, 32777. doi:10.1029/2000JE001429

**Photometry**
- Lucey P.G. et al. (2014) The global albedo of the Moon at 1064 nm from LOLA. *JGR Planets* 119, 1665. https://ntrs.nasa.gov/citations/20140017658
- Pieters C.M. (1999) The Moon as a spectral calibration standard enabled by lunar samples: the Clementine example. https://sites.brown.edu/relab/files/2022/02/Pieters_NV99_8025.pdf
- Sato H. et al. (2014) Resolved Hapke parameter maps of the Moon. *JGR Planets* 119, 1775. doi:10.1002/2013JE004580 ; README: https://pds.lroc.im-ldi.com/data/LRO-L-LROC-5-RDR-V1.0/LROLRC_2001/DATA/SDP/WAC_HAPKEPARAMMAP/WAC_HAPKEPARAMMAP_README.TXT
- Helfenstein P., Veverka J. (1987) Photometric properties of lunar terrains derived from Hapke's equation. *Icarus* 72, 342. (NTRS: https://ntrs.nasa.gov/citations/19870013999)
- Wyman C., Sloan P.-P., Shirley P. (2013) Simple analytic approximations to the CIE XYZ colour matching functions. *JCGT* 2(2).
- Seyond Robin W1G datasheet: https://irp.cdn-website.com/237e2515/files/uploaded/Seyond_Robin+W1G+Datasheet_EN_Public_V2.1.3_ITS.pdf

**Mechanics**
- Carrier W.D. III, Olhoeft G.R., Mendell W. (1991) Physical properties of the lunar surface. In *Lunar Sourcebook*, Ch. 9, Cambridge Univ. Press.
- Connolly J., Carrier W.D. (2023) LPSC abstract: https://ntrs.nasa.gov/api/citations/20220018853/downloads/LPSC%202023%20Abstract_Connolly_Carrier_v2.pdf
- Ding L. et al. (2024) Lunar rock investigation and tri-aspect characterization of lunar farside regolith by a digital twin. *Nat. Commun.* 15, 2098. https://pmc.ncbi.nlm.nih.gov/articles/PMC11258293/
- Wong J.Y. (2012) Predicting the performances of rigid rover wheels on extraterrestrial surfaces based on test results obtained on earth. *J. Terramechanics* 49, 49–61. doi:10.1016/j.jterra.2011.11.002
- Daca A., Skonieczny K. (2022) ASTRA 2022: https://robotics.estec.esa.int/ASTRA/Astra2022/24_Skonieczny-Krzysztof-24-109-Skonieczny-Krzysztof.pdf
- Costes N.C., Farmer J.E., George E.B. (1972) Mobility performance of the Lunar Roving Vehicle: terrestrial studies – Apollo 15 results. NASA TR R-401 (not read here).
