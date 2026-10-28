# Mesh owner: TIN, stitching, normals, UVs, GLB, regolith textures

Implements DESIGN.md sections 6 and 9. Owner: `mesh`. Status 2026-10-25: all modules built, 21 tests pass,
the real-data trial passes, and the Gazebo visual check is done (results in section 4).

Scratch/experiments: `/home/nargon/togo_assets/lunar/experiments/impl_mesh/` (called `EXP` below).

## 1. What was built

| file | content |
|---|---|
| `lunar_world_gen/mesh/tin.py` | `tin_from_grid` (pydelatin, vertices = grid nodes with float64 node heights, `Mesh.lattice`, corners, CCW), plus `tin_surface_on_grid` / `tin_max_error` (numba rasteriser that independently checks the error bound at every node) |
| `lunar_world_gen/mesh/stitch.py` | `Piece`, `make_piece`, `stitch_pieces` (fan-split insertion plus canonical heights and normals), `check_stitched` (proves the invariant), `union_topology` (cracks, manifoldness, orientation, Euler characteristic), `tile_priority` / `strip_priority` |
| `lunar_world_gen/mesh/normals.py` | `GridNormalField` (central differences plus Gaussian smoothing at levels 1/2/4/8 nodes), `vertex_normals` / `assign_normals` (adaptive smoothing by local vertex spacing), `triangle_normals` |
| `lunar_world_gen/mesh/uv.py` | `planar_uv` (u = x / tile, v = -y / tile), `assign_uvs`, `zone_uv_tile_m` |
| `lunar_world_gen/mesh/gltf.py` | `write_glb` (direct binary writer: one primitive, one material, float32 attributes, uint32 indices, no textures, no node transform), `read_glb`, `read_glb_json`, `verify_glb` (own parser plus trimesh), `planar_tangents` |
| `lunar_world_gen/materials/regolith.py` | `make_regolith_textures` / `synthesize_regolith`: periodic albedo (sRGB), normal (OpenGL), roughness maps; helpers `height_to_normal_map`, `periodic_noise`, `seam_ratio`, `linear_to_srgb` |
| `lunar_world_gen/materials/__init__.py` | re-exports `make_regolith_textures`, `synthesize_regolith` |
| `tests/test_world_mesh.py` | 21 tests (section 3) |

The foundation files (`grid.py`, `config.py`, `layout.py`, `mesh/types.py`, `mesh/__init__.py`) are unchanged.

## 2. Public API

```python
# mesh/tin.py
tin_from_grid(grid: Grid, max_error: float, name: str = "", *, max_triangles: int | None = None,
              check: bool = True) -> Mesh            # meta: max_error, tin_error, grid_shape, spacing_m, n_nodes, time_s
tin_surface_on_grid(mesh: Mesh, grid: Grid) -> np.ndarray   # TIN height at every grid node (NaN = uncovered)
tin_max_error(mesh: Mesh, grid: Grid) -> dict               # {max_abs_m, rms_m, uncovered_nodes, n_nodes}
grid_node_index(grid: Grid, lattice) -> (i, j, on_grid)

# mesh/normals.py
GridNormalField(grid: Grid, levels=(1.0, 2.0, 4.0, 8.0))    # callable (x, y) -> (n,3) float32; .gradient(x, y, sigma_nodes) / .normals(...)
vertex_normals(mesh, field: GridNormalField | Grid, *, adaptive=True, sigma_nodes=1.0, alpha=0.375) -> (n,3) float32
assign_normals(mesh, field, **kw) -> Mesh;  vertex_spacing(mesh) -> (n,);  triangle_normals(mesh) -> (n,3)

# mesh/uv.py
planar_uv(x, y, tile_m) -> (n,2) float32;  assign_uvs(mesh, tile_m) -> Mesh;  zone_uv_tile_m(cfg, zone) -> float

# mesh/stitch.py
@dataclass Piece(mesh: Mesh, rect: Rect, priority: int, height_fn=None, normal_fn=None, uv_tile_m=None, name="")
tile_priority(t: TileId, n_tiles: int) -> int           # j * n + i  (lower wins)
strip_priority(zone: str, side: str) -> int             # zone rank * 1e6 + index in (south, north, west, east)
make_piece(zone_grid, rect, max_error, priority, *, name, uv_tile_m=None, normals=True,
           normal_margin_m=0.0, adaptive_normals=True) -> Piece
find_contacts(pieces) -> list[Contact]                  # segment and corner-point contacts; overlap -> ValueError
stitch_pieces(pieces, *, copy_normals=True) -> dict      # in place; report (inserted, n_overwritten, max_abs_dz_overwritten_m, ...)
check_stitched(pieces, *, topology=True, compare_normals=True) -> dict
      # {ok, mismatches, t_junctions, normal_mismatches, n_shared_points, max_abs_dz_m, topology{cracks, euler, ...}}
union_topology(pieces) -> dict

# mesh/gltf.py
write_glb(path, mesh: Mesh, material: MaterialSpec, *, node_name=None, tangents=None, extras=None) -> dict
read_glb(path) -> Mesh;  read_glb_json(path) -> dict
verify_glb(path, expected: Mesh | None = None, *, require_up=False, use_trimesh=True) -> dict   # {ok, errors, ...}
planar_tangents(normals, handedness=1.0) -> (n,4)       # optional; not needed by Gazebo

# materials/regolith.py
make_regolith_textures(cfg, out_dir, *, px=None, flip_green=None, prefix="regolith",
                       write_flipped=False) -> dict[str, str]   # keys albedo, normal, roughness, meta[, normal_flipped]
synthesize_regolith(cfg, *, px=None, flip_green=None) -> dict  # arrays + stats, no I/O
```

### Recommended use (integration / world stages)

```python
lay = Layout(cfg)
pieces  = [make_piece(core_H, lay.tile_rect(t), cfg.mesh.core_visual_max_error_m, tile_priority(t, lay.n_tiles),
                      name=t.name, uv_tile_m=cfg.mesh.uv_tile_m.core, normal_margin_m=cfg.zones.core.margin_m)
           for t in lay.tiles()]
pieces += [make_piece(ring_H, r, cfg.mesh.ring_visual_max_error_m, strip_priority("ring", s), name=f"ring_{s}",
                      uv_tile_m=cfg.mesh.uv_tile_m.ring, normal_margin_m=1.0) for s, r in lay.strip_rects("ring").items()]
# ... far (2 m) and horizon (20 m) strips the same way
rep = stitch_pieces(pieces); chk = check_stitched(pieces); assert chk["ok"]
for p in pieces:   # tiles: lay.tile_mesh_name(t, "ground"); strips: lay.strip_mesh_name(zone, side, "ground")
    write_glb(lay.meshes_dir / f"{p.name}_ground.glb", p.mesh, MaterialSpec("regolith", (*base, 1.0), 0.0, rough))
```

- **Order.** TIN, then normals and UVs, then stitch, then write. `make_piece` does the first three.
- **Memory.** `height_fn` and `normal_fn` only hold a reference to the zone grid, so pass a memory-mapped grid (`Grid.load(..., mmap=True)`). Otherwise the grids stay alive with the pieces.
- **Collision pieces.** Use `make_piece(C, rect, cfg.mesh.collision.core_max_error_m, prio, name=..., normals=False)`. They are stitched the same way.
- **Rectangles.** Every rectangle edge must be a node line of its grid (`make_piece` checks this). The zone boundaries ±50 / ±250 / ±1100 m are on the 25 cm, 2 m and 20 m lattices.

SDF material for ground visuals (as tested; see the contract changes in section 6):

```xml
<material><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><pbr><metal>
  <albedo_map>textures/regolith_albedo.png</albedo_map><normal_map>textures/regolith_normal.png</normal_map>
  <roughness_map>textures/regolith_roughness.png</roughness_map><roughness>1.0</roughness><metalness>0.0</metalness>
</metal></pbr></material>
```

## 3. How to run

```bash
cd src/lunar_world_gen
~/.venvs/lunar_world/bin/python -m pytest tests/test_world_mesh.py -q        # 21 passed in ~8-10 s
# experiments (PYTHONPATH=$PWD, venv python): EXP/real_trial.py, realistic_tile.py, realistic_stitch.py,
# bench_tin.py, bench_glb_strip.py, make_tex.py, gz_build.py; Gazebo: EXP/run_gz.sh + gzcap_multi.py (system python3)
```

The tests cover:
- the pydelatin convention probe;
- the TIN error bound at 3 tolerances (max |z_grid - z_mesh| over every node ≤ max_error + 2e-6), vertices on nodes with exact float64 node heights, corners, CCW triangles that tile the rectangle, an off-lattice grid rejected, and a flat grid giving 2 triangles;
- planar normals exact (adaptive and fixed), grid normals keeping more relief than triangle normals, and planar UVs;
- stitching on 2×2 tiles (1 cm) + 4 ring strips (25 cm) + 4 outer strips (2 m): 0 mismatches, 0 T-junctions, 0 normal mismatches, 0 cracks, Euler 1, union boundary edges only on the outer square (trimesh), canonical core heights on the ring boundary, and planar UVs of inserted vertices;
- 3- and 4-piece corners taking the lowest priority's height, with deliberately offset heights per piece;
- idempotence and determinism, insertion without `height_fn` leaving the canonical surface unchanged (≤ 1e-12 m), `make_piece` layouts and the alignment check, and overlapping pieces rejected;
- a GLB round trip (header, one primitive, uint32 / float32, material, no textures or transforms, float32-exact positions, trimesh), the TANGENT option, and empty meshes rejected;
- textures: seams (wrap-around ratio < 1.3; a non-periodic crop scores > 3), calibrated mean colour (±2 %), variation band, 12° ± 0.3° RMS slope, roughness 0.95, determinism, G flip, PNG output, and the normal-map axis convention.

## 4. Measurements

### TIN (`tin_from_grid`, 1001 × 1001 nodes = one 10 m tile at 1 cm)

| input | 5 mm | 1 cm |
|---|---|---|
| synthetic fBm H 0.8, λ ≥ 2 cm, ν(1 cm) = 1.70 mm | 54,406 tris / 27,363 verts, 0.18 s, max err 4.99982 mm | 13,575 tris, 0.13 s |
| **realistic**: real DTM tile + calibrated fBm (W(k) complement) + DESIGN §4 craters (~3,100 per tile window); composite RMS slope at 1 cm 10.2-11.9° (target 10.4°) | **59,202-59,846 tris, 0.15-0.19 s** | 15,109-15,696 tris, 0.08-0.11 s |
| real band-limited DTM only (`core_1cm`, 100 tiles) | 39-622 tris (mean 109, total 10,881), 0.027 s mean | – |

- **Projection for the full core.** About 6.0 M ground triangles at 5 mm (~1.67 MB GLB per tile, ~167 MB total), or 1.5 M at 1 cm.
- **Per-tile cost.** TIN ~0.2 s plus grid normals 0.29-0.5 s (4 Gaussian levels on the 1101² window with margin). The 100 real tiles took 32.5 s in total, with 605 MB RSS (core grid memory-mapped).
- **Error bound.** Checked at every node by the independent rasteriser (first call 0.69 s including numba JIT, then 0.04 s per tile). The worst overshoot was 1.3e-8 m above 5 mm (pydelatin works in float32).

Ring strip 2001 × 801 nodes at 25 cm:
- Synthetic coarse fBm (λ ≥ 0.5 m, calibrated PSD): 82,881 triangles at 5 cm (0.31 s, normals + UV 0.45 s); 203,413 at 3 cm (0.76 s).
- Real band-limited DTM only: south 6,623, north 5,924, west (801 × 401) 1,276, east 1,124.

### Stitching

| case | before | after | stitch / check time |
|---|---|---|---|
| synthetic test layout (12 pieces, 1 cm / 25 cm / 2 m) | 240 T-junctions, 31 height mismatches, 276 cracks, Euler −20 | 0 / 0 / 0, Euler 1 (234 vertices inserted) | 17 ms / – |
| **real**: 100 tiles from `core_1cm` + 4 ring strips from a separate 25 cm extraction (`dtm_extract.py --size 500 --spacing 0.25`) | 430 T-junctions, 9 height mismatches (max 5.3 µm), 61 normal mismatches; 386 contacts (224 segments, 162 corner points) | 0 mismatches, 0 T-junctions, 0 normal mismatches, 0 cracks / non-manifold / orientation conflicts, V 13,212, E 39,431, F 26,220, **Euler 1** | 2.53 s* / 0.09 s |
| realistic 3 × 3 tiles (59.2-59.8k tris each, one composite field) | 0 T-junctions, 0 height mismatches, 1,077 normal mismatches | all 0, Euler 1 | 0.11 s / 0.35 s |

\* Re-timed after making `height_fn` / `normal_fn` run once per piece: still 2.53 s, with identical results (deterministic). The cost is one lazy normal-field build (crop of 1101² from the memory map + gradient + Gaussian, ~0.1 s) per canonical piece that receives inserted points (about 25 here). Meshing the 100 real tiles took 32.5-48 s depending on machine load.

- **Real trial details.**
  - 392 vertices inserted: 1-89 per piece, 57-89 per ring strip.
  - 1,131 existing boundary heights overwritten. Max |dz| 5.28 µm, rms 0.54 µm: this is the core-vs-ring extraction disagreement. Sampled every 25 cm on the core boundary it is max 5.3 µm, rms 2.2 µm, far inside the 0.1 mm contract.
  - Max normal change at shared vertices 2.6°.
  - Tile error after stitching: max 4.99894 mm, still within 5 mm.
  - 104 GLBs written in 0.034 s and verified in 0.62 s (own parser + trimesh), 0 failures.
  - Total 39 s, peak RSS 0.7 GB.
- **Observation.** For tiles cut from one field, pydelatin's boundary vertex sets coincided. Inserts are needed between different grids (core ↔ ring) and when inputs differ. The normals still differ (adaptive smoothing uses each piece's own vertex spacing, up to 7.2° apart) and are unified by copying.

### GLB

- **Write speed.** 1 M vertices / 2 M triangles (POSITION + NORMAL + TEXCOORD_0, uint32) is 56.1 MB, written in 0.21-0.23 s (≈ 250 MB/s). `read_glb` takes 0.11 s; `verify_glb` with trimesh 1.7 s, OK.
- **Merged rock test mesh** (Gazebo check): 8,807 prod50 rocks from the SFD over 20 × 20 m (8,125 pebbles, 677 medium, 5 large), 625,188 triangles, 29.6 MB, verify OK.

### Regolith textures (2048², covering 2 m, 0.977 mm per texel)

| | |
|---|---|
| time | synthesis 7.0 s, PNG writing 3.7 s |
| sizes | albedo 3.93 MB, normal 8.37 MB, roughness 0.96 MB |
| albedo | linear mean (0.12000, 0.10700, 0.08800); relative variation p1 / p99 −11.0 % / +11.2 %, std 4.75 % |
| normal map | directional RMS slope 12.0°; micro-relief RMS 0.79 mm; 10,032 clods (3-25 mm, 6 % cover) |
| roughness | mean 0.950 |
| seams | ratio 1.008 / 1.007 (albedo), 1.079 / 1.019 (normal), where 1 = seamless |

Preview: `docs/impl/img/mesh_regolith_shaded_preview.png`.

### Gazebo visual check

Setup:
- headless gz-sim 8, ogre2, `GZ_PARTITION=claude_mesh`, `ROS_DOMAIN_ID=74`;
- D3D12 confirmed in `/proc/<pid>/maps` (libd3d12, libdxcore);
- classic Bullet, SceneBroadcaster, UserCommands, Sensors;
- ambient 0.01, black background, shadows on.

The server logs show only the known `SetVisibilityMask` reserved-bits warning. First image 4.2 s after start; all captures done in 7.8-11.2 s.

**Normal-map and material test** (`EXP/gz/mesh_normaltest.sdf`, sun from the NE at 22°, nadir camera; image `docs/impl/img/mesh_normaltest_panels.png`). The bump numbers are luminance differences between half-discs. Sun from the NE means the true relief has N > S and E > W.

| panel | N − S | E − W | reading |
|---|---|---|---|
| real geometric bumps (reference) | +25.4 | +26.4 | truth |
| flat quad + our normal map (OpenGL, G = +y) | **+27.1** | **+27.5** | matches the true relief on both axes |
| flat quad + G-flipped map | −27.5 | +27.4 | north/south inverted: wrong |
| our map + GLB TANGENT (w = +1) | +27.4 | +27.8 | same as without: tangents not needed |

| material test (mean RGB of the quad) | |
|---|---|
| `<diffuse>1 1 1</diffuse>` + albedo map | (50.2, 47.0, 42.6) |
| `<diffuse>` = base colour + albedo map | **(12.4, 10.0, 7.0)**: ogre2 multiplies diffuse × map |
| `<diffuse>` = base colour, no map | (50.0, 47.0, 42.0) |
| GLB with a red material, no SDF material | (140, 0, 0) |
| the same GLB + SDF regolith material | (48.8, 45.7, 41.4): the SDF material overrides the GLB material |

- **(a)** The SDF `<material>` overrides the GLB material, and the PBR maps are applied (relative paths).
- **(b)** Our normal map convention is correct as generated: OpenGL, G = +y = image up = north. No flip is needed.
- **UV orientation.** An arrow/marker texture appears upright and unmirrored: image up = north, right = east, with u = x / tile and v = −y / tile.

**Look test** (`EXP/gz/mesh_look.sdf`):
- **Scene.**
  - The 2 × 2 real stitched tiles with the most relief: `tile_05_07`, `tile_06_07`, `tile_05_08`, `tile_06_08`, i.e. x 0-20, y 20-40, 4.7 m relief.
  - One merged rock GLB with the prod50 atlas through an SDF PBR material.
  - The eva1 sun, 21.9° elevation from the east.
- **Images.** `docs/impl/img/mesh_look_{rover,close,oblique,seam_nadir}.png`.
- **Overall look.** Plausible:
  - grey-brown regolith with mm-scale grain and clods visible up to a few metres;
  - rocks textured from the atlas, so its UVs are right, with lit east faces and westward shadows;
  - the 2 m texture repeat is not noticeable in the views.
- **Tiles.** These DTM-only tiles are smooth at the metre scale, as expected; fBm and craters come from the terrain stage.
- **Seams.** A nadir view over the 4-tile corner shows no crack. The centre row and column luminance is within 1.2 % of the median. The longest dark run on the seam lines is 2 px (column) and 8 px (row, one pebble shadow).
- **G-flipped variant under this east sun.** Mean |Δ| 0.1-1.4 grey levels (p99 ≤ 8). East light barely exercises G, which is why the NE-lit bump test is the decisive one.

## 5. Known limitations

- **Fan slivers.** Fan splitting can create long sliver triangles in a coarse piece next to a dense boundary: one ring-strip edge receives the core boundary's 1 cm-lattice vertices. The result is watertight and valid; on real data it was 57-89 inserts per ring strip.
- **Normals of inserted canonical vertices.** They use the finest smoothing level (σ = 1 node), not the adaptive one. A non-canonical piece's boundary normals are the canonical piece's, which differ by up to 7° from what the piece would compute itself. This is identical across the seam, so there is no shading crack.
- **float32 in pydelatin.** The error bound holds to about 1e-8 m. `tin_from_grid` requires grids on the 1 cm lattice and raises otherwise.
- **Regolith map.** It is generic micro-relief that repeats every 2 m in the core, every 20 m in the far field and every 200 m on the horizon.
- **Roughness map.** That ogre2 multiplies `<roughness>` × `roughness_map` comes from the Ogre-Next PBS shader, not from a measurement. `<roughness>1.0</roughness>` with the map is safe either way.
- **Rock atlas normal map.** It renders plausibly, but its convention was not tested separately. It is the blender agent's MikkTSpace/OpenGL map.
- **Empty meshes.** `write_glb` refuses them. A tile without rocks must get no rocks visual, rather than an empty file.
- **Not in scope.** `rocks_merge.py` (listed under `mesh/` in DESIGN §14) is not in this task's file list. An experiment-only merger is in `EXP/gz_build.py::build_rocks`: it transforms the prod50 npz positions and normals and copies the atlas UVs.

## 6. Contract changes and notes for other owners

1. **World SDF (§8).** When `albedo_map` is set, use `<diffuse>1 1 1 1</diffuse>`, not the base colour. Measured: base colour × map renders 4× too dark. The albedo map's mean already is the calibrated base colour.
2. **World SDF (§8).** Use `<roughness>1.0</roughness>` together with `roughness_map`; the map holds 0.95. Ogre multiplies the two (from the shader, unverified). `<specular>0 0 0 1</specular>` was used in all tests.
3. **Normal map.** Keep the OpenGL convention: verified, no flip. The optional config key `materials.regolith.normal_flip_green` (default false) is read but not needed; it is not in the YAML.
4. **`Piece` fields.** Beyond the suggested `(mesh, rect, priority, height_fn)`, `Piece` has `normal_fn`, `uv_tile_m` and `name`, plus the `make_piece` helper.
5. **Normals (§6).** "Light smoothing" defaults to *adaptive* smoothing: σ = 0.375 × local vertex spacing, clipped to 1-8 nodes. `adaptive=False` gives the plain σ = 1 node rule.
6. **GLB writer.** It never writes TANGENT unless asked; Gazebo does not need it. One primitive and one material, as specified.
