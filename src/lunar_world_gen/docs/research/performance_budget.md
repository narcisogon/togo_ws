# Gazebo performance budget for the lunar world generator

Benchmarks of the RTX 2080 / WSL2 machine that will run the Apollo 16 world: how much visual geometry,
how many rocks and how much collision geometry Gazebo Harmonic can take while the GPU LiDAR runs in real time.
Work of the `perf` research subagents, 2026-10-25. Session 1 ran 06:19–07:19; it stopped when the PC went to
sleep during `S_real_200_40M_tiles`. Session 2 (`perf-finish`, 11:11–11:49) filled the gaps and ran a
full-robot launch check. Every number below comes from a run that was executed and inspected.

Experiments directory (WSL): `/home/nargon/togo_assets/lunar/experiments/perf/` (called `EXP` below).
All runs are merged in `EXP/results/summary_final.csv` (`.json`); per-run data is in
`EXP/results/<config>/result.json` and `server.log`.

**Software:** gz-sim 8.11.0, gz-rendering 8.2.3 (ogre2) through Mesa 25.2.8 d3d12 on the RTX 2080 (8 GB),
gz-physics 7.6.0 classic Bullet plugin, ROS 2 Jazzy. **Machine:** Ryzen 5 3600X (6 cores, 12 threads),
WSL2 Ubuntu 24.04 with 12 CPUs and 15.6 GB RAM, Windows 11 power plan "Balanced".

---

## 0. Budget at a glance

| item | budget | why (section) |
|---|---|---|
| Layout | **100 tiles of 10 m** for the core; never one big mesh | One mesh: 2× load time, 2.6–2.9× RAM. 400 tiles: +31 % LiDAR frame time (2.1, 2.4) |
| Triangles per tile | **≤ 200 k**, preferably 80–160 k | 400 k tiles build 28 % slower per triangle (2.4) |
| Core terrain | **8–10 M triangles** | Max error 3.5–2.9 mm on the calibrated field, below the LiDAR's 5–11 mm range noise beyond 2 m (1.2) |
| Total visual (terrain + merged rocks + far field) | **≤ 16 M**, hard cap **20 M** | Headless load ≈ 2.95 + 0.81·T s. The robot launch stalls 3.2 + 1.2·T s at spawn (2.3, 2.9) |
| Total visual if the launch file stays unchanged | **≤ 5.7 M** | Keeps the spawn stall under the spawners' 10 s service timeout (2.9) |
| Rocks (visual) | **merge every rock into its tile mesh**; ≤ ~200 separate rock visuals | 216 k merged rocks: RTF 1.0. Separate: 3 k → RTF 0.36, 10 k → 0.03, 50 k → server abort (2.6) |
| Rock triangles | **≤ ~6 M** in the core: pebbles (2–5 cm) ≤ 20, 5–20 cm rocks ≤ 100, rocks ≥ 20 cm ≤ 1 500 | Fits the 16 M total with the LM-site counts (4.3) |
| Server RAM | **0.33 + 0.107 GB per M triangles** (tiles): 2.0 GB at 16 M. Budget **×2** with the GUI client | All of it is the render scene; the GUI builds its own copy (2.7, 4.2) |
| VRAM | **+0.065–0.08 GiB per M triangles** per rendering process | Windows itself held 2.0–3.3 GiB of the 8 GiB (4.2) |
| Collision ground | **0.25 M triangles** (28.6 cm grid) in 100 tiles; ≤ 1 M | 4 M: RTF 0.95–0.965 (2.8) |
| Rock collisions (16.6 k rocks ≥ 5 cm) | **bake them into the collision tile meshes**. No per-rock entities, no `convex_hull` | 10 k baked: no extra step cost. 10 k boxes: RTF 0.24. Hulls: +10 ms of load each (2.8) |
| LiDAR headroom at 16–20 M | real sensor 20 Hz: **2.5–3.1×**; Robin-W-like 10 Hz: **3.6–4.0×** | Render cost per frame (2.5) |
| Launch | the robot spawns at ~11–12 s, then the sim stalls **~3.2 + 1.2·T s**; controllers are active 0.6–1.4 s after the stall. Raise spawner timeouts or accept exit 1 | Measured with `sim_gz.launch.py` (2.9, 4.6) |

---

## 1. Method

### 1.1 Run setup

- **Server.** One server per run: `gz sim -s -r --headless-rendering -v 4 <world>`.
  - Environment: `GALLIUM_DRIVER=d3d12`, `LD_LIBRARY_PATH=/usr/lib/wsl/lib`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`,
    `GZ_PARTITION=claude_perf`, `ROS_DOMAIN_ID=63`.
  - Each server ran in its own session/process group and was stopped by group (TERM, then KILL).
  - In session 2 every server also ran under `timeout --foreground -k 10 <limit>`, and every measurement under
    `timeout` (`EXP/scripts/finish/bench_t.py` = `bench.py` plus that limit; the measurement code is unchanged).
- **GPU check.** Every rendering run had `libd3d12.so`/`libdxcore.so` mapped into the live server process
  (column `d3d12` in `summary_final.csv`).
- **World template.** The physics block of `lunar_surface.world`: classic Bullet, `max_step_size` 1 ms, RTF 1,
  1000 Hz. Gravity −1.62. Systems UserCommands, SceneBroadcaster and Sensors (ogre2). A directional sun with
  shadows, ambient 0.25.
- **LiDAR rig.** A static `gpu_lidar` 0.6 m above the terrain at (0, 0), looking +x, FOV 120° × 70°,
  range 0.1–150 m, no noise. The benchmark subscribes to it, so it really renders.
  - *Real sensor:* 209 × 122 = 25 498 rays at 20 Hz, as generated from the URDF.
  - *Robin-W-like:* 920 × 195 = 179 400 rays at 10 Hz.
  - About 50 % of the rays hit the ground (12 770 / 25 498 and 89 858 / 179 400 returns); the rest is sky.

### 1.2 Test assets

- **Visual terrain (A, B, S, T).** A synthetic lunar-like height field (`scripts/hfield.py`): fBm with
  H = 0.8 calibrated to ν(L) = 0.069 m·(L/1 m)^0.8 from `lunar_calibration.md`, plus power-law craters
  0.5–14 m, sampled every 1 cm.
  - The 100 × 100 m core is 10 × 10 tiles of 10 m. Each tile was triangulated with pydelatin to an exact
    budget of T/100 triangles, so triangles follow the local roughness as they will in the generator.
  - `single` holds the same triangles concatenated into one GLB primitive.
  - GLB contents: positions, normals, UVs, uint32 indices, an explicit material (metallic 0), Z-up.
  - Geometric error of each budget on this field (median / max over the 100 tiles of the per-tile max error):

| triangles (core) | 1 M | 3 M | 10 M | 20 M | 40 M |
|---|---|---|---|---|---|
| max error, median / worst tile | 9.7 / 18.5 mm | 5.6 / 11.2 mm | 2.9 / 5.6 mm | 1.9 / 3.2 mm | 1.2 / 2.0 mm |
| GLB size (all tiles) | 28 MB | 85 MB | 281 MB | 561 MB | 1.1 GB |

- **Tile-size variants (T).** The same 10 M triangles as 25 tiles of 20 m (400 k each) or 400 tiles of 5 m
  (25 k each), from `gen_retile.py`.
- **Rocks (C).**
  - Library: deformed icospheres with 20 / 80 / 320 / 1280 triangles for D < 5 cm / 5–15 / 15–50 / ≥ 50 cm,
    2 variants each.
  - Fields: N(≥D) ∝ D^−2.8. 1 k, 10 k and 50 k rocks with D ≥ 5 cm, and 216 k rocks with D ≥ 2 cm (the
    LM-site count for 100 × 100 m from `lunar_calibration.md`, 5.53 M rock triangles).
  - `inst`: one `<visual>` per rock referencing a library GLB with `<scale>`, in 100 static models.
  - `merged`: the rocks baked into the 10 M terrain tiles as a second primitive (still 100 visuals).
- **Collision (D, D2).**
  - Ground: low-passed regular grids of the same field, positions only: 0.25 M triangles (28.6 cm cells),
    1 M (14.3 cm) and 4 M (7.1 cm), as 100 tiles or one mesh.
  - Rock collisions (all D ≥ 5 cm rocks of a field): boxes D × 0.7D × 0.5D, the library meshes (GImpact) or
    `optimization="convex_hull"`. The D2 layouts are in 2.8.
  - Load generator: a 4-wheel skid-steer stand-in for the A300 (wheel r 0.1651 m, 60 kg box chassis, gz DiffDrive
    driving 0.5 m/s at 0.15 rad/s after load) plus 12 passive spheres, boxes and wheel-like cylinders.
  - D worlds are physics only (no Sensors system).

### 1.3 What was measured

| quantity | definition |
|---|---|
| first LiDAR (load) | wall time from server start to the first point cloud received; for physics-only worlds, to sim time > 0.1 s ("sim running") |
| scene build | server-log time of `Lidar points for …` minus `Rendering Thread initialized`: building the render scene, i.e. loading every visual mesh into ogre2 |
| RTF | Δsim / Δreal from `/world/<w>/stats` over a 30 s window after a 5 s warm-up; "min" is the lowest 0.1 s field value in the window |
| ms / frame (S) | 1000 / (LiDAR frames per wall second) with the LiDAR at 10× its real rate (200 or 100 Hz). The simulation then waits for every frame, so this is the full per-frame cost; headroom = real period / frame cost |
| CPU | server process CPU as % of one core; "busiest thread" is the hottest thread (the simulation thread in D) |
| physics ms per step | busiest-thread % / (100 · RTF): CPU time of the simulation thread per 1 ms step |
| RSS / peak | VmRSS at the end of the window / VmHWM |
| VRAM | whole-GPU memory increase over an idle baseline taken 3 s before the start (`nvidia-smi`) |

### 1.4 Sessions, conditions, repeatability

- **Session 1 (06:19–07:19, 46 runs).** The PC was otherwise idle.
- **Session 2 (11:11–11:49, 28 runs + 3 robot launches, 31 min of benchmark time).** The PC was in
  interactive use: VS Code, Chrome, Task Manager and audio were running, and Windows CPU load was ~37 %.
  - A temporary Windows keep-awake request was held during session 2 so the PC could not idle-sleep again. It
    was released afterwards; no setting was changed.
- **Cross-session repeats:**
  - LiDAR frame cost: 14.5 → 14.9 ms (`S_real_200_10M_tiles` vs `_r2`). **Render throughput is reproducible
    to 3 %.**
  - First LiDAR: 10.5 → 11.0 s (`A_10M_tiles` vs `_r2`). Session-2 loads were 4–16 % slower.
  - Physics step: 0.53 → 0.64 ms ground only, 0.93 → 1.28 ms with 1 k rock boxes (`D_*_r2`). **The busy
    machine costs the simulation thread 20–40 %**, so the D2 comparisons use only same-session references.
- **Within a session:** the same world measured with A and B gave load times within 0.0–1.6 s up to 20 M
  (e.g. 20 M tiles 19.1 / 19.4 s; 3.5 s apart for the 40 M single mesh) and RSS within 4 %.

---

## 2. Results

### 2.1 Suite A — visual triangles, real LiDAR (209 × 122 @ 20 Hz), session 1

| config | tris | first LiDAR s | scene build s | RTF mean / min | LiDAR Hz | server CPU % | busiest thread % | RSS GB (peak) | VRAM +GiB | GPU W (idle 22) |
|---|---|---|---|---|---|---|---|---|---|---|
| `A_flat` | 12 | 3.6 | 0.0 | 1.000 / 1.00 | 19.99 | 60 | 23 | 0.30 (0.30) | 0.10 | 23 |
| `A_1M_tiles` | 1M | 4.1 | 1.2 | 1.000 / 0.98 | 19.99 | 64 | 25 | 0.43 (0.43) | 0.14 | 23 |
| `A_1M_single` | 1M | 4.2 | 1.3 | 1.000 / 0.99 | 20.02 | 60 | 23 | 0.50 (0.62) | 0.15 | 24 |
| `A_3M_tiles` | 3M | 5.4 | 2.5 | 1.000 / 0.98 | 20.02 | 65 | 25 | 0.63 (0.63) | 0.29 | 26 |
| `A_3M_single` | 3M | 6.8 | 3.9 | 1.000 / 0.99 | 20.02 | 60 | 23 | 1.00 (1.32) | 0.31 | 26 |
| `A_10M_tiles` | 10M | 10.5 | 7.6 | 1.000 / 0.99 | 19.99 | 65 | 25 | 1.37 (1.37) | 0.67 | 42 |
| `A_10M_single` | 10M | 18.0 | 15.0 | 1.000 / 0.29 | 19.99 | 60 | 23 | 3.44 (3.75) | 0.78 | 48 |
| `A_20M_tiles` | 20M | 19.1 | 16.1 | 1.000 / 1.00 | 20.02 | 65 | 26 | 2.48 (2.48) | 1.39 | 47 |
| `A_20M_single` | 20M | 35.2 | 32.0 | 1.000 / 0.99 | 20.02 | 60 | 23 | 6.55 (7.22) | 1.45 | 94 |
| `A_40M_tiles` | 40M | 42.0 | 38.7 | 1.000 / 0.98 | 19.99 | 65 | 25 | 4.53 (4.53) | 2.44 | 67 |
| `A_40M_single` | 40M | 69.0 | 64.9 | 0.973 / 0.21 | 19.45 | 58 | 22 | 12.82 (14.17) | 2.80 | 164 |

**Steady-state rendering is not the limit:** RTF stays 1.0 up to 40 M triangles in tiles. The costs are load
time, RAM and VRAM. One 40 M mesh is the only case below RTF 1. In every A run the server process uses ~0.6
cores.

### 2.2 Suite B — Robin-W-like LiDAR (920 × 195 @ 10 Hz), session 1

Paired cells read "tiles / single".

| config | first LiDAR s | scene build s | RTF mean | LiDAR Hz | server CPU % | RSS GB (peak) | VRAM +GiB | GPU W |
|---|---|---|---|---|---|---|---|---|
| `B_flat` | 4.6 | 0.0 | 1.000 | 9.99 | 56 | 0.31 | 0.14 | 23 |
| `B_1M_tiles` / `single` | 4.1 / 4.2 | 1.1 / 1.2 | 1.000 / 1.000 | 10.02 / 10.03 | 60 / 56 | 0.44 / 0.50 (0.62) | 0.18 / 0.19 | 23 / 23 |
| `B_3M_tiles` / `single` | 5.5 / 6.9 | 2.5 / 3.9 | 1.000 / 0.998 | 9.99 / 9.96 | 59 / 56 | 0.64 / 1.00 (1.32) | 0.31 / 0.35 | 24 / 28 |
| `B_10M_tiles` / `single` | 10.5 / 17.3 | 7.5 / 14.3 | 1.000 / 1.000 | 9.99 / 9.99 | 59 / 54 | 1.37 / 3.44 (3.75) | 0.70 / 0.81 | 32 / 43 |
| `B_20M_tiles` / `single` | 19.4 / 33.6 | 16.2 / 30.5 | 1.000 / 1.000 | 9.99 / 9.99 | 58 / 55 | 2.40 / 6.55 (7.22) | 1.42 / 1.48 | 42 / 50 |
| `B_40M_tiles` / `single` | 40.4 / 72.5 | 37.3 / 67.0 | 1.000 / 1.000 | 10.03 / 9.99 | 59 / 55 | 4.53 / 12.79 (14.12) | 2.47 / 2.38 | 48 / 112 |

Same picture as A: the 7× larger point cloud at half the rate keeps RTF 1.0 up to 40 M in tiles. The lowest
0.1 s RTF value was ≥ 0.73 in every B run except one hiccup to 0.03 in `B_3M_single`. Loads and memory are
those of the meshes.

### 2.3 Load-time and memory model (least squares, 100 tiles vs one mesh, T in millions of triangles)

From A + B + S runs of session 1 (11 points each up to 20 M), `EXP/scripts/finish/fit.py`:

| quantity | 100 tiles | one mesh |
|---|---|---|
| scene build | **0.81·T s** (max residual 1.0 s; session 2: −0.5 + 0.95·T) | 1.61·T − 0.8 s |
| first LiDAR (headless, world has its own LiDAR) | **2.95 + 0.81·T s** (session 2: 2.57 + 0.95·T) | 2.10 + 1.62·T s |
| server RSS | **0.33 + 0.107·T GB** (max residual 0.06 GB; session 2: 0.34 + 0.106·T); peak = RSS | 0.15 + 0.32·T GB; peak ~10 % higher at ≥ 10 M |
| VRAM | **0.08 + 0.065·T GiB** (session 2: 0.082·T) | 0.11 + 0.068·T GiB |

Triangles allowed for a load-time target (100 tiles, headless first LiDAR):

| target | session 1 (idle PC) | session 2 (busy PC) | per tile (100 tiles) |
|---|---|---|---|
| ≤ 10 s | ≤ 8.7 M | ≤ 7.8 M | ≤ 78–87 k |
| ≤ 20 s | ≤ 21.0 M | ≤ 18.3 M | ≤ 183–210 k |

Beyond 20 M the cost per triangle rises: 40 M tiles (400 k per tile) cost 0.97 s/M.

### 2.4 Suite T — tile size at constant 10 M triangles, session 2

| config | tiles | tris / tile | first LiDAR s | scene build s | RSS GB | ms / LiDAR frame (200 Hz stress) | server CPU % at 20 Hz |
|---|---|---|---|---|---|---|---|
| `T_10M_t25` | 25 (20 m) | 400 k | 11.8 | 10.2 | 1.41 | 16.3 | 66 |
| `A_10M_tiles_r2` | 100 (10 m) | 100 k | 11.0 | 8.0 | 1.36 | 14.9 | 66 |
| `T_10M_t400` | 400 (5 m) | 25 k | 11.2 | 8.1 | 1.41 | 19.6 | 90 |
| `A_10M_single` (session 1) | 1 | 10 M | 18.0 | 15.0 | 3.44 | 15.9 | 60 |

- **Large tiles** load slower: 400 k-triangle meshes build 28 % slower than 100 k. Coarse tiles also cull
  worse: +9 % frame time at 20 m tiles.
- **Many small tiles** cost per object: 400 tiles add 31 % to the LiDAR frame time and 24 % of a core to the
  server at 20 Hz. That is ~16 µs per extra visual per frame, because the gpu_lidar renders several cube-map
  faces.
- **10 m tiles are the best of the three on every measure.**

### 2.5 Suite S — LiDAR render throughput and headroom

The LiDAR rate was raised 10× so that rendering limits the simulation. Frames per wall second then give the
per-frame cost.

| config | rays | frames / wall s | ms / frame | headroom at real rate | busiest thread % | GPU util % (idle) | GPU W | session |
|---|---|---|---|---|---|---|---|---|
| `S_real_200_flat` | 25 498 | 104.8 | 9.6 | 5.2× (20 Hz) | 51 | 29 (28) | 24 | 1 |
| `S_real_200_1M_tiles` | 25 498 | 86.7 | 11.5 | 4.3× | 47 | 32 (29) | 29 | 1 |
| `S_real_200_1M_single` | 25 498 | 93.5 | 10.7 | 4.7× | 46 | 27 (27) | 36 | 1 |
| `S_real_200_10M_tiles` | 25 498 | 68.9 | 14.5 | 3.4× | 38 | 39 (25) | 57 | 1 |
| `S_real_200_10M_tiles_r2` | 25 498 | 66.9 | 14.9 | 3.3× | 40 | 35 (2) | 56 | 2 |
| `S_real_200_10M_t25` | 25 498 | 61.5 | 16.3 | 3.1× | 41 | 38 (10) | 66 | 2 |
| `S_real_200_10M_t400` | 25 498 | 51.1 | 19.6 | 2.6× | 51 | 32 (10) | 48 | 2 |
| `S_real_200_10M_single` | 25 498 | 62.9 | 15.9 | 3.1× | 31 | 48 (25) | 126 | 1 |
| `S_real_200_20M_tiles` | 25 498 | 62.2 | 16.1 | 3.1× | 34 | 44 (30) | 115 | 1 |
| `S_real_200_20M_tiles_corner` | 25 498 | 50.3 | **19.9** | **2.5×** | 35 | 47 (3) | 118 | 2 |
| `S_real_200_20M_single` | 25 498 | 37.5 | 26.7 | 1.9× | 19 | 69 (28) | 152 | 1 |
| `S_robinw_100_flat` | 179 400 | 60.1 | 16.6 | 6.0× (10 Hz) | 69 | 14 (9) | 39 | 2 |
| `S_robinw_100_1M_tiles` | 179 400 | 57.7 | 17.3 | 5.8× | 68 | 16 (9) | 40 | 2 |
| `S_robinw_100_10M_tiles` | 179 400 | 43.2 | 23.1 | 4.3× | 53 | 34 (9) | 52 | 2 |
| `S_robinw_100_20M_tiles` | 179 400 | 39.5 | 25.3 | 4.0× | 49 | 37 (9) | 74 | 2 |
| `S_robinw_100_20M_tiles_corner` | 179 400 | 36.4 | **27.5** | **3.6×** | 45 | 38 (10) | 99 | 2 |

`_corner` runs put the LiDAR at (−45, −45) m looking across the whole core diagonally (worst-case culling).
All other S runs are at the centre looking +x.

- **Per-frame cost is mostly fixed:** 9.6 ms for the real sensor and 16.6 ms for the Robin-W-like one on a
  flat world. Each million triangles in tiles adds ~0.25 ms (real) or ~0.4 ms (Robin-W).
- **Neither the GPU nor any single thread is saturated** (GPU ≤ 70 % busy, busiest thread ≤ 70 %). The cost is
  latency in the render/readback path through d3d12.
- **The Robin-W-like mode has more headroom at 10 Hz than the real sensor at 20 Hz.** At 20 Hz it would have
  1.8× (20 M, worst case).
- **The run interrupted in session 1, `S_real_200_40M_tiles`, was not repeated** (> 20 M).

### 2.6 Suites C and C2 — rocks as separate visuals vs merged (10 M terrain tiles, real LiDAR)

| config | rocks (rock tris) | status | first LiDAR s | RTF mean / min | LiDAR Hz | server CPU % | busiest threads % | RSS GB | session |
|---|---|---|---|---|---|---|---|---|---|
| `A_10M_tiles` (no rocks) | 0 | ok | 10.5 | 1.000 / 0.99 | 19.99 | 65 | 25/12/7 | 1.37 | 1 |
| `C_merged_1k` | 1 k (92 k) | ok | 9.9 | 1.000 / 0.97 | 20.02 | 65 | 25/12/7 | 1.39 | 1 |
| `C_merged_10k` | 10 k (0.93 M) | ok | 10.4 | 1.000 / 0.98 | 20.02 | 66 | 25/12/7 | 1.45 | 1 |
| `C_merged_50k` | 50 k (4.6 M) | ok | 14.0 | 1.000 / 0.99 | 20.02 | 66 | 25/13/7 | 1.83 | 1 |
| `C_merged_216k` | 216 k (5.5 M) | ok | 13.7 | 1.000 / 0.99 | 20.02 | 65 | 25/12/7 | 1.99 | 1 |
| `C_inst_1k` | 1 k separate | ok | 10.5 | 0.999 / 0.26 | 19.98 | **132** | 57/36/14 | 1.86 | 1 |
| `C2_inst_3k` | 3 k separate (1 model) | ok | 13.4 | **0.364** / 0.01 | 7.60 | 146 | 92/32/9 | 2.04 | 2 |
| `C_inst_10k` | 10 k separate (100 models) | ok | 15.4 | **0.034** / 0.00 | 0.67 | 119 | 96/15/3 | 3.61 | 1 |
| `C2_inst_10k_onemodel` | 10 k separate (1 model) | ok | 27.4 | 0.028 / 0.00 | 0.56 | 118 | 94/10/7 | 3.58 | 2 |
| `C2_inst_10k_nosb` | as above, no SceneBroadcaster | ok | 21.0 | 0.031 / 0.00 | 0.63 | 126 | 99/18/4 | 3.56 | 2 |
| `C2_inst_10k_nolidar` | as above, no Sensors/LiDAR | ok | 8.9 (sim running) | 0.193 / 0.06 | – | 101 | 99/1/0 | 2.23 | 2 |
| `C_inst_50k` | 50 k separate | **server aborted** | – | – | – | – | – | – | 1 |

- **Merged rocks are almost free.** 216 k rocks (5.5 M triangles) on top of 10 M terrain: RTF 1.0, normal CPU,
  +0.6 GB RAM. The load (13.7 s) is below the 15.5 s the triangle model predicts for 15.5 M.
- **Separate rock visuals cost per entity on every simulation step, and the cost grows faster than linearly.**
  1 k add ~0.3 ms of simulation-thread time per step. With 3 k a step takes 2.75 ms of wall time, with 10 k
  29–36 ms.
  - Grouping them into one model does not help (onemodel vs 100 models).
  - The SceneBroadcaster is not the cause (`nosb`).
  - Without rendering, 10 k still give RTF 0.19 (5 ms per step of entity work); the render scene multiplies it.
  - RAM: ~0.2–0.5 MB per separate visual.
- **`C_inst_50k` aborted after 43 s** inside the Sensors render thread while creating visuals:
  `Ogre::ItemIdentityException … HlmsManager::addReference` while creating a material. This is an ogre2
  material/block limit, not RAM. The hard ceiling lies between 10 k and 50 k separate visuals.

### 2.7 Suite E — what the server memory is

| config | sim running s | RSS GB | session |
|---|---|---|---|
| `E_20M_tiles_nolidar` (20 M tiles, no rendering sensor) | 2.3 | **0.14** | 2 |
| `A_20M_tiles` (same meshes, LiDAR) | 19.2 | 2.48 | 1 |

Without a rendering sensor the server never loads the visual meshes. All of the RSS growth with triangles,
2.3 GB at 20 M, is the ogre2 render scene and its mesh copies.

### 2.8 Suites D and D2 — collision (physics only, rover stand-in driving + 12 passive bodies)

Session 1, idle PC:

| config | sim running s | RTF mean / min | busiest thread % | physics ms / step | RSS GB |
|---|---|---|---|---|---|
| `D_flat` (box ground) | 2.3 | 1.000 / 0.99 | 38 | 0.38 | 0.17 |
| `D_col_0.25M_tiles` | 2.4 | 1.000 / 0.94 | 53 | 0.53 | 0.23 |
| `D_col_0.25M_single` | 2.4 | 1.000 / 0.97 | 51 | 0.51 | 0.24 |
| `D_col_1M_tiles` | 2.8 | 1.000 / 0.98 | 68 | 0.68 | 0.33 |
| `D_col_1M_single` | 3.2 | 1.000 / 0.99 | 68 | 0.68 | 0.45 |
| `D_col_4M_tiles` | 4.6 | 0.954 / 0.66 | 92 | 0.97 | 0.75 |
| `D_col_4M_single` | 6.9 | 0.965 / 0.71 | 93 | 0.96 | 1.33 |
| `D_rocks_1k_box` (0.25 M tiles + 1 k rock boxes in 100 models) | 2.4 | 0.981 / 0.61 | 91 | 0.93 | 0.74 |
| `D_rocks_1k_mesh` (same, library meshes) | 2.5 | 0.985 / 0.60 | 91 | 0.92 | 0.77 |
| `D_rocks_1k_hull` (same, `convex_hull`) | **12.9** | 0.980 / 0.52 | 90 | 0.92 | 1.09 |

Session 2, busy PC, all on the 0.25 M collision tiles. D2 separates per-entity from per-shape cost:

| config | rock collision layout | sim running s | RTF mean / min | physics ms / step | RSS GB |
|---|---|---|---|---|---|
| `D_col_0.25M_tiles_r2` | none (reference) | 2.5 | 0.976 / 0.14 | 0.64 | 0.23 |
| `D2_r1k_mergedcol` | 1 k rocks baked into the 100 collision tile GLBs | 2.6 | 0.992 / 0.31 | **0.60** | 0.25 |
| `D2_r1k_box_inground` | 1 k boxes as extra collisions of the ground link | 2.5 | 1.000 / 0.89 | 0.75 | 0.51 |
| `D2_r1k_box_onelink` | 1 k boxes in one extra static link | 2.5 | 0.998 / 0.68 | 0.79 | 0.51 |
| `D_rocks_1k_box_r2` | 1 k boxes in 100 models (as in D) | 2.7 | 0.700 / 0.05 | 1.28 | 0.74 |
| `D2_r1k_box_models` | 1 k boxes, one model each | **8.9** | **0.191** / 0.06 | 4.9 | **2.82** |
| `D2_r10k_mergedcol` | 10 k rocks baked into the collision tiles (+0.93 M tris) | 3.1 | **0.993** / 0.01 | **0.62** | 0.39 |
| `D2_r10k_box_inground` | 10 k boxes in the ground link | 11.0 | 0.246 / 0.10 | 3.95 | 3.02 |
| `D2_r10k_box_onelink` | 10 k boxes in one link | 11.6 | 0.240 / 0.07 | 4.0 | 3.02 |
| `D_rocks_10k_box` | 10 k boxes in 100 models | 8.5 | 0.151 / 0.08 | 6.4 | 3.24 |
| `D_rocks_10k_mesh` | 10 k meshes (GImpact) in 100 models | 12.4 | 0.141 / 0.06 | 6.9 | 3.51 |

- **Ground mesh.** The step cost grows with the triangles under the contacts: 0.25 M → 0.53 ms, 1 M → 0.68 ms,
  4 M → 0.97 ms (RTF < 1). Tiles and one mesh cost the same per step, but one mesh loads slower (4 M: 6.9 vs
  4.6 s) and uses more RAM (1.33 vs 0.75 GB).
- **Rock collisions are paid per entity, not per shape.** Box, GImpact mesh and hull cost the same per step
  at 1 k (0.92–0.93 ms).
  - Each extra static model costs ~4.3 µs per step and ~2.6 MB of RAM.
  - Collisions inside one link cost ~0.1–0.15 µs per step each at 1 k, but ~0.33 µs each at 10 k
    (superlinear), and ~0.29 MB of RAM each.
- **Baking rocks into the collision tile meshes costs nothing measurable per step** (0.60–0.62 vs 0.64 ms)
  and only +0.16 GB RAM and +0.6 s load for 10 k rocks.
- **`convex_hull` adds ~10 ms of load per collision** (+10.4 s for 1 k rocks). Extrapolated to the 16.6 k
  rocks ≥ 5 cm of the LM-site field, that would be ~170 s.

### 2.9 Full robot launch on a generator-sized world (session 2)

Three real launches: `ros2 launch togo_gz sim_gz.launch.py rviz:=false use_sim_time:=true` with the rover
agent's headless recipe (`EXP/launch/run_launch.sh`; WSLg display for ogre2, d3d12 verified,
`GZ_PARTITION=claude_perf`, `ROS_DOMAIN_ID=63`, private `ROS_HOME`).

- **Worlds.** `EXP/worlds/L_<T>M_tiles_robot.sdf` mirrors `lunar_surface.world`: Physics (classic Bullet,
  1 ms), UserCommands, SceneBroadcaster and Contact, and **no Sensors system** (the robot's
  `sensors_gz.xacro` brings it). Gravity 1.62. The visual terrain is the A suite's 1 M or 20 M tiles; the
  collision ground is the 0.25 M tiles.
- **Spawn.** At (0, 0, −0.052): terrain maximum under the footprint + 0.136 + 0.05 m. The robot stood still.
- **Timing.** Times are seconds after `ros2 launch`. "Stall" is wall time with no sim progress, from a
  `/stats` log.

| event | 1 M tiles (`L1M_1`) | 20 M tiles (`L20M_2`) | 20 M tiles (`L20M_3`) |
|---|---|---|---|
| world initialized (no rendering yet) | 1.5 | 1.6 | 2.1 |
| robot_state_publisher + spawners started (`controller_spawn_delay` 10 s) | 11.2 | 11.3 | 11.7 |
| robot spawned (`Entity creation successful`) | 11.4 | 11.5 | 11.9 |
| ogre2 loading (robot's Sensors system) | 12.5 | 14.4 | 15.1 |
| LiDAR advertised = render scene built | 15.6 | **37.5** | **39.8** |
| simulation stalled around the spawn | **4.4 s** | **26.4 s** | **28.2 s** |
| joint_state_broadcaster | active 15.8 | activated at 14.4, but its spawner got no reply within 10 s (twice), then "already active" → **exit 1** | same, **exit 1** |
| platform_velocity_controller active | 17.1 | 38.4 | 40.6 |
| server RSS | 0.88 GB | 2.91 GB | 2.95 GB |
| RTF after start-up (40 s, robot still) | 0.80 | – | 0.91 |

- **The world itself loads in ~2 s.** The whole render scene is built after the spawn, and the simulation
  stalls meanwhile. The stall grows from 4.4 s to 26–28 s, about **3.2 + 1.2 s per million visual
  triangles**.
  - The scene build here (23–25 s for 20 M, including ~0.7 s of ogre2 start-up) is ~15–25 % slower than in
    the headless benchmark of the same session (19.4 s). It includes the robot's own meshes, and the server
    runs without `--headless-rendering`.
- **Both controllers end up active, but the joint_state_broadcaster spawner exits with code 1 in 2 of 2
  20 M launches.** It is the same failure mode the rover agent saw on the 30 m world. The velocity-controller
  spawner was blocked on the shared spawner lock during the stall, so its own calls did not time out.
- **The steady RTF with the full robot (LiDAR, RGBD camera, NavSat, IMU, robot lights) was 0.80–0.91 on both
  worlds.** It was not lower with 20 M visual triangles, so in this busy session the RTF limit came from the
  robot and the machine load, not the world's visuals. The rover agent measured 0.97–1.00 with a quiet PC.

---

## 3. Interpretation

1. **Rendering cost per frame is small and mostly fixed.** That is why the A and B suites hold RTF 1.0 up to
   40 M triangles. What grows with triangles is the one-off scene build (~0.8 s per million in tiles) and
   memory (~0.11 GB RAM and ~0.07 GiB VRAM per million, per rendering process).
2. **Per-entity costs dominate everything else.** Every visual or collision entity costs time on every 1 ms
   step and RAM, and the step cost grows faster than linearly at 10 k entities (C2, D2). Meshes are cheap per
   triangle. The generator should therefore put geometry into a few large mesh entities:
   - ~100 terrain tiles with their rocks for visuals
   - ~100 collision tiles with their rocks for physics
   - not thousands of models, visuals or collisions
3. **The mesh-size optimum is ~100 k triangles per entity.** Larger meshes load slower per triangle (one
   mesh: 2×; 400 k tiles: 1.28×). Smaller and more numerous meshes cost per-object time in every LiDAR face
   render and in the server.
4. **The robot launch is where load time hurts.** Without a rendering sensor in the world, the scene is built
   when the robot spawns, and the simulation does not step until it is done (2.9).

---

## 4. Budgets for the world generator

### 4.1 Visual triangles and tiles

- **Layout.** The core as 100 tiles of 10 m. The far-field ring as its 64 tiles (~0.5 M triangles, adds
  ~0.4 s and ~0.05 GB by the model; not benchmarked here). Never one mesh for the core.
- **Per tile.** ≤ 200 k triangles of terrain + rocks; 80–160 k is ideal.
- **Load-time targets.** Headless server with its own LiDAR, fit 2.3:

| target | total visual triangles | per tile (100 tiles) |
|---|---|---|
| first LiDAR ≤ 10 s | ≤ 7.8–8.7 M | ≤ 78–87 k |
| first LiDAR ≤ 20 s | ≤ 18–21 M | ≤ 183–210 k |
| robot-launch spawn stall ≤ 10 s (no launch change) | ≤ 5.7 M | ≤ 57 k |
| robot-launch spawn stall ≤ 20 s | ≤ 14 M | ≤ 140 k |

- **Recommended:** a core terrain of 8–10 M triangles. That gives an error bound of ~3–3.5 mm on the
  calibrated field (1.2), below the gpu_lidar's own range quantisation of 5 mm at 2–4 m and 11 mm at 4–8 m
  (`gazebo_capabilities.md`).
- **Total visual ≤ 16 M** (hard cap 20 M): 8–10 M terrain + ≤ 6 M merged rocks + ~0.5 M far field. That
  means ~16 s headless load, ~22 s spawn stall and 2.0 GB server RAM. LiDAR headroom is ≥ 3.1× (real) /
  ≥ 4.0× (Robin-W) at the centre, 2.5× / 3.6× in the worst case.

### 4.2 Memory

- **Server.** RSS ≈ 0.33 + 0.107·T GB with tiles: 1.4 GB at 10 M, 2.0 GB at 16 M, 2.5 GB at 20 M. The robot
  adds ~0.4 GB (2.9 GB in the 20 M launch). One mesh is 0.15 + 0.32·T GB, with peaks 10 % higher.
- **GUI client.** All of the server's mesh memory is the render scene (2.7). The GUI client (`gz sim -g`,
  started by `togo_sim_wsl.sh`) builds its own copy from the same files.
  - Budget **another ~0.11 GB RAM and ~0.07 GiB VRAM per million triangles**, plus its own scene-build time
    before it shows the world.
  - Not measured: the GUI was not opened because it would appear on the user's desktop.
- **Limits.** WSL has 15.6 GB, shared with ROS, RViz and the SLAM container. At 16 M, server + GUI is about
  4.5–5 GB of RAM and 2.2–2.6 GiB of VRAM.
  - Windows itself held 2.0–3.3 GiB of the 8 GiB VRAM during these tests.
  - 40 M in tiles (4.5 GB per process) or any single-mesh world ≥ 20 M (6.5–7.2 GB) is too much with the GUI.

### 4.3 Rocks (visual)

- **Merge every rock into its terrain tile mesh** (a second primitive/material per tile is fine). 216 k
  merged rocks (5.5 M triangles) ran at RTF 1.0 with normal CPU and +0.6 GB.
- **Separate rock visuals** are the measured limit:
  - 1 k still hold RTF ~1.0, but cost +0.67 cores and +0.5 GB.
  - 3 k → RTF 0.36; 10 k → RTF 0.03; 50 k → server abort.
  - Allow at most a few hundred (e.g. hero boulders that need their own pose); none for pebbles.
- **Rock triangle budget.** With the LM-site counts for the 100 × 100 m core (`lunar_calibration.md`:
  216 k ≥ 2 cm, 16.6 k ≥ 5 cm, 2.4 k ≥ 10 cm, 343 ≥ 20 cm, 26 ≥ 50 cm), keep merged rocks ≤ ~6 M triangles:
  - pebbles 2–5 cm: ≤ 20 triangles on average (199 k → ≤ 4.0 M; the Blender pebble LODs, 16–36 with mean 26,
    are close)
  - 5–20 cm: ≤ 100 triangles (16.3 k → ≤ 1.6 M). At 0.573° beam spacing a 10 cm rock gets 2–3 beams across
    at 4 m. The Blender library's medium LODs (5–30 cm) average 598 triangles and would add ~9.8 M, so they
    need a coarser LOD for scattering.
  - ≥ 20 cm: ≤ 1 500 triangles (343 rocks → ≤ 0.5 M)

### 4.4 Collision

- **Ground.** The smooth collision ground as 100 tiles at ~0.25 M triangles (28.6 cm cells, low-passed),
  ≤ 1 M at most. 0.25 M gave 0.53 ms per step with the driving rover stand-in and 12 bodies. 4 M already
  drops RTF to 0.95–0.965.
- **Rocks ≥ 5 cm** (16.6 k at the LM site): **bake their collision geometry into the collision tile meshes.**
  Use a low-poly proxy per rock, e.g. the ≤ 64-vertex hull mesh or ≤ 30–80 triangles. 10 k baked rocks cost
  no measurable step time, +0.16 GB and +0.6 s load.
- **Never one model per rock** (1 k models → RTF 0.19, +2.6 GB, +6.4 s load). Do not put thousands of
  primitives in a link either (10 k → RTF 0.24, +2.8 GB, +9 s load). No `optimization="convex_hull"` at
  scale (~10 ms of load each).
- **Friction caveat** (rover agent): classic Bullet ignores SDF friction on mesh collisions (μ fixed at
  0.5 × the other surface). If rock friction must differ from the ground's, make only the few large rocks
  (e.g. ≥ 20 cm, ~343 in the core) primitives in **one** link. That costs an estimated +0.05 ms per step and
  +0.1 GB, from the 1 k one-link run.

### 4.5 LiDAR headroom

| world (100 tiles) | real 209×122 @ 20 Hz: ms/frame (headroom) | Robin-W-like 920×195 @ 10 Hz: ms/frame (headroom) |
|---|---|---|
| flat | 9.6 (5.2×) | 16.6 (6.0×) |
| 1 M | 11.5 (4.3×) | 17.3 (5.8×) |
| 10 M | 14.5–14.9 (3.3–3.4×) | 23.1 (4.3×) |
| 20 M, centre | 16.1 (3.1×) | 25.3 (4.0×) |
| 20 M, whole core in view (worst case) | 19.9 (2.5×) | 27.5 (3.6×) |

- **Headroom** = how many times the sensor's frame budget the current render cost fits. At 16–20 M, the real
  sensor uses ≤ 40 % of its 50 ms budget and the Robin-W-like mode ≤ 28 % of 100 ms. The Robin-W-like mode
  at 20 Hz would still have 1.8×.
- **What eats into it** (not measured here): other rendering sensors on the robot (the RGBD camera), the
  robot's 4 shadow-casting spot lights, and the GUI client rendering on the same GPU.

### 4.6 Robot launch timing (`sim_gz.launch.py`: spawn at t = 6 s, controllers at `controller_spawn_delay` = 10 s)

What actually happens (2.9, and `rover_baseline.md` §3):

1. **`world_load_delay` (6 s) is not the gate.** `create` waits for `robot_description`, which
   robot_state_publisher only publishes after `controller_spawn_delay`, so the robot spawns at ~11–12 s.
   `create` retries the world service without limit.
2. **A generated world without its own rendering sensor initialises in ~2 s.** The visual load happens at the
   spawn and stalls the simulation for **~3.2 + 1.2·T s** (T = millions of visual triangles in 10 m tiles):
   4.4 s at 1 M, 26–28 s at 20 M.
3. **Above ~5.7 M triangles the stall exceeds the spawners' 10 s service-call timeout.** At 20 M the
   joint_state_broadcaster spawner exited with code 1 in both launches, although the controller was active.
   Both controllers are active 0.6–1.4 s after the stall ends: 17 s at 1 M, 38–41 s at 20 M.

Recommendations:

- **Keep `controller_spawn_delay` at 10 s.** Raising it only delays the spawn: it starts robot_state_publisher
  and therefore the spawn, and the stall comes after the spawn anyway. `world_load_delay` can stay at 6 s.
- **For worlds above ~5 M visual triangles**, give both spawners in `togo_deploy/launch/control.launch.py`
  `--service-call-timeout` and `--switch-timeout` (Jazzy defaults 10 s and 5 s) larger than the expected
  stall plus margin, e.g. 60 s at 16–20 M. Not tested here; the options exist in the installed
  `controller_manager/spawner.py`.
  - The `--controller-manager-timeout 300` and the spawner lock's ~115 s retry budget are not limiting below
    ~2 min of stall.
- **Do not judge start-up by spawner exit codes.** Check `/controller_manager/list_controllers`, as the rover
  test tool does.
- **Gate SLAM, navigation and test scripts on events, not timers:** the first LiDAR message on
  `/husky/sensors/seyond/points` and `/clock` advancing. Expect the first cloud ~15.6 s after launch at 1 M
  and ~38–40 s at 20 M; add ~1.2 s per extra million triangles.
- **Untested option:** build the scene before the spawn by giving the world its own Sensors system plus a
  tiny rendering sensor. The stall would then fall into the world load, before the controllers start. Risk:
  the robot's URDF also loads a Sensors system, which may duplicate the render scene. This needs a test
  before use.
- **The GUI client loads the same scene a second time.** `togo_sim_wsl.sh` starts it 8 s after the launch,
  so with a large world it shows the terrain only after its own build of roughly the same length.

---

## 5. Limitations and open items

- **Session 2 ran on a busy PC.** Physics step times were 20–40 % higher and loads 5–17 % slower than in
  session 1. The D2 conclusions rest on same-session references, and the ratios are large (4–10×), so they
  hold. Absolute session-2 numbers are pessimistic.
- **Synthetic stand-ins.** The terrain was the calibrated synthetic field, not the DTM-based generator output.
  The rocks were icosphere stand-ins, not the Blender library, and the rover was a stand-in. Mesh files had
  no textures; baked normal maps and the regolith material are not included, and their texture memory comes
  on top.
- **Not benchmarked:**
  - the combined candidate world (`scripts/gen_worlds_f.py`: 10 M core + 216 k merged rocks + far field +
    rover LiDAR); a quick follow-up, preferably with baked rock collisions instead of `--rockcol box`
  - the far-field ring on its own
  - shadows on/off
  - the GUI client
  - other robot sensors at stress rates
- **Not repeated:** configs above 20 M. `S_real_200_40M_tiles` and `S_*_40M_single` were dropped by
  instruction.
- **Spawn-stall model.** Fitted to two triangle counts (1 M and 20 M) in one machine state. The 20 M point was
  reproduced (26.4 / 28.2 s).

## 6. Files

- **Existing data, unchanged.** Session 1's suites and logs, including `results/summary.csv` and
  `logs/runs.log`.
- **Session 2 scripts** (`EXP/scripts/finish/`):
  - `bench_t.py` and `make_bench_t.py`
  - `run_one.sh` and `run_list.sh`: no overwrite, hard limits, logs in `logs/runs_finish.log`
  - `collect.py`: writes `results/summary_final.csv` / `.json` with all 74 runs
  - `tables.py`, `fit.py`
- **Session 2 configs:** `worlds/configs_finish.json` (repeats and corner worlds),
  `worlds/S_*_20M_tiles_corner.sdf`, and `worlds/configs_t.json` + `meshes/terrain_10M_{t25,t400,pos,nouv}`
  (from `gen_retile.py 10`).
- **Robot launch:** `EXP/launch/` (`make_world.py`, `run_launch.sh`, `statslog.py`; runs in
  `launch/runs/{L1M_1,L20M_2,L20M_3}` with `launch.log`, `timeline.json`, `stats.csv`), worlds
  `worlds/L_{1,20}M_tiles_robot.sdf`.
- **Reproduce one run:** `bash EXP/scripts/finish/run_one.sh <config>`. It refuses to overwrite an existing
  `results/<config>`.
