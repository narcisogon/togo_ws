"""
generate_lunar_world.py  (v1 — All-in-One)
═══════════════════════════════════════════
Generates everything needed for a lunar Gazebo Jazzy world:
  1. lunar_terrain.png  — 16-bit heightmap
  2. lunar_terrain.dae  — Collada mesh (visual + collision)
  3. lunar_surface.world — complete SDF with rocks, ridges, lighting

Then auto-copies all three to the install directory.

Requires: numpy, Pillow, scipy
  pip install numpy Pillow scipy

Usage:
  python3 generate_lunar_world.py
"""

import numpy as np
from PIL import Image
from numpy.fft import fft2, ifft2, fftfreq
from scipy.ndimage import gaussian_filter
import os
import shutil

# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

# World
WORLD_SIZE   = 50.0    # metres
MAX_HEIGHT   = 1.2     # metres
SEED         = 42

# Heightmap
HM_SIZE      = 2048    # pixels (2^n for Ogre2)
HM_OUTPUT    = "lunar_terrain.png"

# Mesh
MESH_RES     = 513     # vertices per side (50/512 = 0.098m spacing)
MESH_OUTPUT  = "lunar_terrain.dae"
MESH_COLOUR  = (0.42, 0.40, 0.38, 1.0)   # lunar regolith grey

# SDF
SDF_OUTPUT   = "lunar_surface.world"
ROBOT_X      = 15.0
ROBOT_Y      = 15.0

# Rock clusters
N_CLUSTERS       = 8     # clusters of rocks near crater rims
ROCKS_PER_CLUSTER = 4    # rocks per cluster
N_ISOLATED       = 12    # isolated rocks scattered across map

# Ridges
N_RIDGES = 4             # sinuous wrinkle ridges

# Auto-copy destination
INSTALL_MEDIA = "/home/er4-user/ws/install/togo_gz/share/togo_gz/media"
INSTALL_WORLD = "/home/er4-user/ws/install/togo_gz/share/togo_gz/worlds"
SRC_WORLD     = "/home/er4-user/ws/src/togo/togo_gz/worlds"


# ══════════════════════════════════════════════════════════════════════════════
# PART 1 — HEIGHTMAP
# ══════════════════════════════════════════════════════════════════════════════

def fbm_terrain(size: int, H: float, rng: np.random.Generator) -> np.ndarray:
    phase          = rng.uniform(0, 2 * np.pi, (size, size))
    amplitude      = rng.rayleigh(1.0, (size, size))
    white          = amplitude * np.exp(1j * phase)
    fx             = fftfreq(size).reshape(1, -1)
    fy             = fftfreq(size).reshape(-1, 1)
    freq           = np.sqrt(fx**2 + fy**2)
    freq[0, 0]     = 1.0
    beta           = 2.0 * H + 2.0
    pf             = freq ** (-beta / 2.0)
    pf[0, 0]       = 0.0
    return np.real(ifft2(white * pf))


def bandpass_noise(size: int, f_lo: float, f_hi: float,
                   rng: np.random.Generator) -> np.ndarray:
    noise  = rng.normal(0.0, 1.0, (size, size))
    F      = fft2(noise)
    fx     = fftfreq(size).reshape(1, -1)
    fy     = fftfreq(size).reshape(-1, 1)
    freq   = np.sqrt(fx**2 + fy**2)
    band   = (np.exp(-0.5 * ((freq - f_hi) / (0.3 * f_hi))**2)
              - np.exp(-0.5 * ((freq - f_lo) / (0.3 * f_lo))**2))
    band   = np.clip(band, 0, None)
    return np.real(ifft2(F * band))


def add_crater(hmap, cx_n, cy_n, r_n, depth, rim_h,
               degraded=False):
    n  = hmap.shape[0]
    cx = cx_n * n
    cy = cy_n * n
    r  = r_n  * n

    margin = int(r * 3.5) + 3
    x0 = max(0, int(cx - margin));  x1 = min(n, int(cx + margin) + 1)
    y0 = max(0, int(cy - margin));  y1 = min(n, int(cy + margin) + 1)

    xs = np.arange(x0, x1)
    ys = np.arange(y0, y1)
    xx, yy = np.meshgrid(xs, ys)
    d  = np.sqrt((xx - cx)**2 + (yy - cy)**2) / max(r, 1e-6)

    patch = hmap[y0:y1, x0:x1].copy()

    if degraded:
        m = d < 1.0
        patch[m] += depth * (d[m] ** 2.2 - 1.0) * 0.40
        m = (d >= 1.0) & (d < 2.0)
        t = (d[m] - 1.0) / 1.0
        patch[m] += rim_h * 0.35 * np.exp(-((t - 0.3) / 0.35) ** 2)
    else:
        m = d < 0.88
        patch[m] += depth * ((d[m] / 0.88) ** 2.2 - 1.0)
        m = (d >= 0.88) & (d < 1.40)
        t = (d[m] - 0.88) / 0.52
        patch[m] += rim_h * np.exp(-((t - 0.42) / 0.22) ** 2)
        m = (d >= 1.40) & (d < 3.00)
        t = (d[m] - 1.40) / 1.60
        patch[m] += rim_h * 0.28 * (1.0 - t) ** 2.5

    hmap[y0:y1, x0:x1] = patch
    return hmap


def add_ridge(hmap: np.ndarray,
              rng: np.random.Generator) -> tuple:
    """
    Add one sinuous wrinkle ridge.
    Returns list of (cx_norm, cy_norm) world positions along the ridge
    so rocks can be placed near it.

    Ridge is defined by a random spline path across the map.
    Height varies along the ridge — uneven, realistic.
    """
    n = hmap.shape[0]

    # Random start/end points — ridge crosses the map
    angle  = rng.uniform(0, 2 * np.pi)
    cx0    = rng.uniform(0.15, 0.85)
    cy0    = rng.uniform(0.15, 0.85)
    length = rng.uniform(0.25, 0.55)   # fraction of map

    # Control points for sinuous path
    n_ctrl  = rng.integers(3, 6)
    t_vals  = np.linspace(0, 1, n_ctrl)
    offsets = rng.uniform(-0.08, 0.08, n_ctrl)  # lateral wander

    # Parametric path
    t_fine  = np.linspace(0, 1, 500)
    lateral = np.interp(t_fine, t_vals, offsets)

    path_x = cx0 + t_fine * length * np.cos(angle) \
             - lateral * np.sin(angle)
    path_y = cy0 + t_fine * length * np.sin(angle) \
             + lateral * np.cos(angle)

    # Ridge width and height vary along path
    base_width = rng.uniform(0.008, 0.018)   # normalised
    base_h     = rng.uniform(0.12, 0.22)     # normalised height

    # Height envelope — uneven bumpy ridge
    h_noise = rng.uniform(0.5, 1.5, len(t_fine))
    h_noise = gaussian_filter(h_noise, sigma=20)  # smooth variation
    h_noise /= h_noise.mean()                      # normalise around 1.0

    width_noise = rng.uniform(0.6, 1.4, len(t_fine))
    width_noise = gaussian_filter(width_noise, sigma=15)
    width_noise /= width_noise.mean()

    # Stamp ridge onto heightmap
    for i, (px, py) in enumerate(zip(path_x, path_y)):
        if not (0.01 < px < 0.99 and 0.01 < py < 0.99):
            continue

        cpx = px * n
        cpy = py * n
        w   = base_width * width_noise[i] * n
        h   = base_h * h_noise[i]

        margin = int(w * 3) + 2
        x0 = max(0, int(cpx - margin));  x1 = min(n, int(cpx + margin) + 1)
        y0 = max(0, int(cpy - margin));  y1 = min(n, int(cpy + margin) + 1)

        xs = np.arange(x0, x1)
        ys = np.arange(y0, y1)
        xx, yy = np.meshgrid(xs, ys)
        d = np.sqrt((xx - cpx)**2 + (yy - cpy)**2) / max(w, 1e-6)

        patch = hmap[y0:y1, x0:x1]
        m = d < 2.5
        # Gaussian cross-section — smooth sides, rounded top
        patch[m] += h * np.exp(-0.5 * (d[m] / 0.7) ** 2)
        hmap[y0:y1, x0:x1] = patch

    # Return sample points along ridge for rock placement
    stride = max(1, len(path_x) // 8)
    ridge_pts = [(path_x[i], path_y[i])
                 for i in range(0, len(path_x), stride)
                 if 0.05 < path_x[i] < 0.95
                 and 0.05 < path_y[i] < 0.95]
    return hmap, ridge_pts


def build_heightmap(size: int, rng: np.random.Generator):
    print(f"\n{'='*60}")
    print(f" PART 1 — Heightmap  ({size}×{size}px, "
          f"{WORLD_SIZE}×{WORLD_SIZE}m, max {MAX_HEIGHT}m)")
    print(f"{'='*60}")

    # ── 1. fBm base ───────────────────────────────────────────────────────
    print("  [1/7] fBm base terrain …")
    base   = fbm_terrain(size, H=0.80, rng=rng)
    xc     = np.linspace(-0.04, 0.04, size)
    yc     = np.linspace(-0.03, 0.03, size)
    X, Y   = np.meshgrid(xc, yc)
    base  += 0.04 * X + 0.03 * Y
    lo, hi = base.min(), base.max()
    hmap   = 0.10 + 0.35 * (base - lo) / (hi - lo)

    # ── 2. Roughness layers ───────────────────────────────────────────────
    print("  [2/7] Roughness layers …")
    hmap += 0.018 * bandpass_noise(size, 0.004, 0.020, rng)
    hmap += 0.011 * bandpass_noise(size, 0.020, 0.060, rng)
    hmap += 0.006 * bandpass_noise(size, 0.060, 0.160, rng)
    hmap += 0.003 * bandpass_noise(size, 0.160, 0.380, rng)
    hmap += 0.002 * bandpass_noise(size, 0.380, 0.500, rng)

    # ── 2b. Regional elevation patches (~0.2m variation) ──────────────────
    print("  [2b/7] Regional elevation patches …")
    patch_n        = bandpass_noise(size, 0.02, 0.06, rng)
    lo_p, hi_p     = patch_n.min(), patch_n.max()
    patch_norm     = (patch_n - lo_p) / (hi_p - lo_p)
    hmap          += 0.167 * patch_norm

    broad_n        = bandpass_noise(size, 0.005, 0.015, rng)
    lo_b, hi_b     = broad_n.min(), broad_n.max()
    broad_norm     = (broad_n - lo_b) / (hi_b - lo_b)
    hmap          += 0.10 * broad_norm

    # ── 3. Large craters (randomised) ────────────────────────────────────
    print("  [3/7] Large craters …")
    rng_l   = np.random.default_rng(seed=SEED + 10)
    n_large = 14
    large_cx  = rng_l.uniform(0.10, 0.90, n_large)
    large_cy  = rng_l.uniform(0.10, 0.90, n_large)
    large_r   = rng_l.uniform(0.015, 0.025, n_large)
    large_dep = rng_l.uniform(0.22, 0.32, n_large)
    large_rim = rng_l.uniform(0.05, 0.08, n_large)
    large_deg = rng_l.random(n_large) < 0.35

    large_crater_positions = list(zip(large_cx, large_cy,
                                      large_r, large_dep,
                                      large_rim, large_deg))
    for cx, cy, r, depth, rim_h, deg in large_crater_positions:
        hmap = add_crater(hmap, cx, cy, r, depth, rim_h,
                          degraded=bool(deg))

    # ── 4. Medium craters (randomised) ───────────────────────────────────
    print("  [4/7] Medium craters …")
    rng_m   = np.random.default_rng(seed=SEED + 20)
    n_med   = 80
    med_cx  = rng_m.uniform(0.04, 0.96, n_med)
    med_cy  = rng_m.uniform(0.04, 0.96, n_med)
    med_r   = rng_m.uniform(0.005, 0.012, n_med)
    med_dep = rng_m.uniform(0.16, 0.26, n_med)
    med_rim = rng_m.uniform(0.03, 0.06, n_med)
    med_deg = rng_m.random(n_med) < 0.40
    for cx, cy, r, depth, rim_h, deg in zip(
            med_cx, med_cy, med_r, med_dep, med_rim, med_deg):
        hmap = add_crater(hmap, cx, cy, r, depth, rim_h,
                          degraded=bool(deg))

    # ── 5. Sinuous wrinkle ridges ─────────────────────────────────────────
    print(f"  [5/7] {N_RIDGES} sinuous wrinkle ridges …")
    rng_r       = np.random.default_rng(seed=SEED + 30)
    ridge_points = []
    for i in range(N_RIDGES):
        hmap, pts = add_ridge(hmap, rng_r)
        ridge_points.extend(pts)
        print(f"         Ridge {i+1}/{N_RIDGES} done")

    # ── 6. Procedural scatter ─────────────────────────────────────────────
    print("  [6/7] Procedural crater scatter …")

    # 0.5m grid coverage
    print("        Layer A: 0.5m grid (10000) …")
    n_grid = 10000
    grid_r = rng.uniform(0.002, 0.004, n_grid)
    cells  = np.array([(i, j)
                       for i in range(100)
                       for j in range(100)])
    rng.shuffle(cells)
    for idx, (ci, cj) in enumerate(cells[:n_grid]):
        cx    = (ci + rng.uniform(0.05, 0.95)) / 100.0
        cy    = (cj + rng.uniform(0.05, 0.95)) / 100.0
        r     = grid_r[idx]
        depth = r * rng.uniform(4.0, 7.0)
        rim_h = depth * rng.uniform(0.06, 0.10)
        deg   = rng.random() < 0.70
        hmap  = add_crater(hmap, cx, cy, r, depth, rim_h,
                           degraded=bool(deg))

    print("        Layer B: small scatter (1000) …")
    n_small   = 1000
    small_r   = rng.power(0.28, n_small) * 0.003 + 0.003
    small_cx  = rng.uniform(0.01, 0.99, n_small)
    small_cy  = rng.uniform(0.01, 0.99, n_small)
    small_deg = rng.random(n_small) < 0.55
    for cx, cy, r, deg in zip(small_cx, small_cy, small_r, small_deg):
        depth = r * rng.uniform(8.0, 14.0)
        rim_h = depth * rng.uniform(0.10, 0.16)
        hmap  = add_crater(hmap, cx, cy, r, depth, rim_h,
                           degraded=bool(deg))

    print("        Layer C: micro (2000) …")
    n_micro  = 2000
    micro_r  = rng.uniform(0.001, 0.002, n_micro)
    micro_cx = rng.uniform(0.01, 0.99, n_micro)
    micro_cy = rng.uniform(0.01, 0.99, n_micro)
    for cx, cy, r in zip(micro_cx, micro_cy, micro_r):
        depth = r * rng.uniform(3.0, 6.0)
        rim_h = depth * rng.uniform(0.05, 0.08)
        hmap  = add_crater(hmap, cx, cy, r, depth, rim_h,
                           degraded=True)

    print("        Layer D: ultra-micro (3000) …")
    n_ultra  = 3000
    ultra_r  = rng.uniform(0.0005, 0.001, n_ultra)
    ultra_cx = rng.uniform(0.01, 0.99, n_ultra)
    ultra_cy = rng.uniform(0.01, 0.99, n_ultra)
    for cx, cy, r in zip(ultra_cx, ultra_cy, ultra_r):
        depth = r * rng.uniform(2.0, 4.0)
        rim_h = depth * 0.05
        hmap  = add_crater(hmap, cx, cy, r, depth, rim_h,
                           degraded=True)

    # ── 7. Normalise ──────────────────────────────────────────────────────
    print("  [7/7] Normalising …")
    lo, hi = hmap.min(), hmap.max()
    hmap   = (hmap - lo) / (hi - lo)
    hmap   = np.clip(hmap, 0.0, 1.0)
    hmap   = np.power(hmap, 0.95)

    print(f"  ✓ Heightmap complete")
    return hmap, large_crater_positions, ridge_points


def save_heightmap(hmap: np.ndarray, path: str) -> None:
    img_array = (hmap * 65535).astype(np.uint16)
    img = Image.fromarray(img_array, mode='I;16')
    img.save(path)
    print(f"  ✓ Saved: {path}")


def get_terrain_height(hmap: np.ndarray,
                       cx_norm: float,
                       cy_norm: float) -> float:
    """
    Sample normalised heightmap at (cx_norm, cy_norm)
    and return world-space Z in metres.
    """
    n  = hmap.shape[0]
    px = int(np.clip(cx_norm * n, 0, n - 1))
    py = int(np.clip(cy_norm * n, 0, n - 1))
    return float(hmap[py, px]) * MAX_HEIGHT


# ══════════════════════════════════════════════════════════════════════════════
# PART 2 — MESH
# ══════════════════════════════════════════════════════════════════════════════

def build_mesh(heights: np.ndarray, world_size: float):
    n    = heights.shape[0]
    half = world_size / 2.0
    print(f"  Building {n}×{n} mesh ({(n-1)**2*2:,} triangles) …")

    xv     = np.linspace(-half, half, n)
    yv     = np.linspace(-half, half, n)
    XX, YY = np.meshgrid(xv, yv)
    verts  = np.stack([XX.ravel(),
                       YY.ravel(),
                       heights.ravel()], axis=1).astype(np.float32)

    uu  = (XX - XX.min()) / (XX.max() - XX.min())
    vv  = (YY - YY.min()) / (YY.max() - YY.min())
    uvs = np.stack([uu.ravel(), vv.ravel()], axis=1).astype(np.float32)

    # CCW winding — normals point UP
    idx = []
    for r in range(n - 1):
        for c in range(n - 1):
            tl = r * n + c
            tr = tl + 1
            bl = tl + n
            br = bl + 1
            idx.extend([tl, tr, bl])
            idx.extend([tr, br, bl])
    indices = np.array(idx, dtype=np.int32)

    normals = np.zeros_like(verts)
    v0 = verts[indices[0::3]]
    v1 = verts[indices[1::3]]
    v2 = verts[indices[2::3]]
    fn = np.cross(v1 - v0, v2 - v0)
    np.add.at(normals, indices[0::3], fn)
    np.add.at(normals, indices[1::3], fn)
    np.add.at(normals, indices[2::3], fn)
    lengths  = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths  = np.where(lengths == 0, 1, lengths)
    normals /= lengths
    normals  = normals.astype(np.float32)

    return verts, normals, uvs, indices


def save_mesh(path, verts, normals, uvs, indices, colour):
    n_verts = verts.shape[0]
    n_tris  = indices.shape[0] // 3
    r, g, b, a = colour

    def fa(arr):
        return " ".join(f"{v:.6f}" for v in arr.ravel())

    p = []
    for i in range(0, len(indices), 3):
        for j in range(3):
            vi = indices[i + j]
            p += [str(vi), str(vi), str(vi)]
    p_str = " ".join(p)

    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <created>2026-07-27</created>
    <modified>2026-07-27</modified>
    <unit name="metre" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="regolith_fx">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>{r} {g} {b} {a}</color></diffuse>
            <reflectivity><float>0.03</float></reflectivity>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="regolith_mat" name="regolith">
      <instance_effect url="#regolith_fx"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="terrain_mesh" name="lunar_terrain">
      <mesh>
        <source id="pos">
          <float_array id="pos_arr" count="{n_verts * 3}">{fa(verts)}</float_array>
          <technique_common>
            <accessor source="#pos_arr" count="{n_verts}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="nrm">
          <float_array id="nrm_arr" count="{n_verts * 3}">{fa(normals)}</float_array>
          <technique_common>
            <accessor source="#nrm_arr" count="{n_verts}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="uvs">
          <float_array id="uvs_arr" count="{n_verts * 2}">{fa(uvs)}</float_array>
          <technique_common>
            <accessor source="#uvs_arr" count="{n_verts}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="verts">
          <input semantic="POSITION" source="#pos"/>
        </vertices>
        <triangles count="{n_tris}" material="regolith_mat">
          <input semantic="VERTEX"   source="#verts" offset="0"/>
          <input semantic="NORMAL"   source="#nrm"   offset="1"/>
          <input semantic="TEXCOORD" source="#uvs"   offset="2" set="0"/>
          <p>{p_str}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="terrain_node" name="lunar_terrain" type="NODE">
        <instance_geometry url="#terrain_mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="regolith_mat"
                                 target="#regolith_mat"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(dae)
    size_mb = os.path.getsize(path) / 1024 / 1024
    print(f"  ✓ Saved: {path}  ({size_mb:.1f} MB, {n_tris:,} triangles)")


def generate_mesh(hmap: np.ndarray) -> None:
    print(f"\n{'='*60}")
    print(f" PART 2 — Mesh  (grid={MESH_RES}×{MESH_RES})")
    print(f"{'='*60}")

    # Sample heightmap to mesh resolution
    img = Image.open(HM_OUTPUT)
    if img.mode in ('I;16', 'I'):
        arr = np.array(img, dtype=np.float64) / 65535.0
        img = Image.fromarray((arr * 255).astype(np.uint8), mode='L')
    else:
        arr = np.array(img, dtype=np.float64)
        if arr.max() > 1.0:
            arr /= 255.0
        img = Image.fromarray((arr * 255).astype(np.uint8), mode='L')

    img_r   = img.resize((MESH_RES, MESH_RES), Image.LANCZOS)
    heights = np.array(img_r, dtype=np.float64) / 255.0 * MAX_HEIGHT

    verts, normals, uvs, indices = build_mesh(heights, WORLD_SIZE)
    save_mesh(MESH_OUTPUT, verts, normals, uvs, indices, MESH_COLOUR)


# ══════════════════════════════════════════════════════════════════════════════
# PART 3 — ROCK PLACEMENT
# ══════════════════════════════════════════════════════════════════════════════

def place_rocks(hmap: np.ndarray,
                large_craters: list,
                ridge_points: list,
                rng: np.random.Generator) -> list:
    """
    Place rocks as SDF model entries.
    Returns list of rock dicts:
      { name, x, y, z, rx, ry, rz, sx, sy, sz, r, g, b }

    Placement strategy:
- Clusters near large crater rims (ejecta deposits)
      - Some along ridge flanks
      - Isolated rocks scattered across surface
    """
    rocks = []
    rock_id = 0

    half = WORLD_SIZE / 2.0

    # ── Clusters near large crater rims ───────────────────────────────────
    rng_rock = np.random.default_rng(seed=SEED + 40)

    # Pick N_CLUSTERS random large craters to place rocks near
    crater_indices = rng_rock.choice(len(large_craters),
                                     size=min(N_CLUSTERS, len(large_craters)),
                                     replace=False)

    for ci in crater_indices:
        cx_n, cy_n, r_n, _, _, _ = large_craters[ci]

        # Rocks land just outside the rim (1.1–2.0 crater radii away)
        for _ in range(ROCKS_PER_CLUSTER):
            angle  = rng_rock.uniform(0, 2 * np.pi)
            dist_n = rng_rock.uniform(1.1, 2.0) * r_n
            rx_n   = cx_n + dist_n * np.cos(angle)
            ry_n   = cy_n + dist_n * np.sin(angle)

            # Keep within map bounds
            rx_n = np.clip(rx_n, 0.02, 0.98)
            ry_n = np.clip(ry_n, 0.02, 0.98)

            # World XY centred at origin
            wx = rx_n * WORLD_SIZE - half
            wy = ry_n * WORLD_SIZE - half

            # Get exact terrain height at this position
            tz = get_terrain_height(hmap, rx_n, ry_n)

            # Rock size — near craters tend to be medium/large ejecta
            size_class = rng_rock.choice(['small', 'medium', 'large'],
                                         p=[0.40, 0.40, 0.20])
            if size_class == 'small':
                sx = rng_rock.uniform(0.08, 0.18)
                sy = rng_rock.uniform(0.08, 0.18)
                sz = rng_rock.uniform(0.06, 0.14)
            elif size_class == 'medium':
                sx = rng_rock.uniform(0.20, 0.40)
                sy = rng_rock.uniform(0.18, 0.38)
                sz = rng_rock.uniform(0.14, 0.28)
            else:
                sx = rng_rock.uniform(0.42, 0.65)
                sy = rng_rock.uniform(0.38, 0.60)
                sz = rng_rock.uniform(0.28, 0.45)

            # Slight random tilt — rocks don't sit perfectly flat
            tilt_x = rng_rock.uniform(-0.3, 0.3)
            tilt_y = rng_rock.uniform(-0.3, 0.3)
            rot_z  = rng_rock.uniform(0, 3.14159)

            # Lunar rock colour — dark grey with slight variation
            base_c = rng_rock.uniform(0.22, 0.38)
            r_col  = base_c + rng_rock.uniform(-0.03, 0.03)
            g_col  = base_c + rng_rock.uniform(-0.03, 0.03)
            b_col  = base_c + rng_rock.uniform(-0.03, 0.03)

            rocks.append({
                'name':  f'rock_{rock_id}',
                'x':     wx,
                'y':     wy,
                'z':     tz + sz * 0.45,   # half-buried look
                'rx':    tilt_x,
                'ry':    tilt_y,
                'rz':    rot_z,
                'sx':    sx,
                'sy':    sy,
                'sz':    sz,
                'r':     r_col,
                'g':     g_col,
                'b':     b_col,
            })
            rock_id += 1

    # ── Rocks along ridge flanks ───────────────────────────────────────────
    if ridge_points:
        n_ridge_rocks = min(12, len(ridge_points))
        chosen_pts    = rng_rock.choice(len(ridge_points),
                                        size=n_ridge_rocks,
                                        replace=False)
        for pi in chosen_pts:
            px_n, py_n = ridge_points[pi]

            # Offset slightly from ridge centre
            offset = rng_rock.uniform(0.01, 0.025)
            angle  = rng_rock.uniform(0, 2 * np.pi)
            rx_n   = np.clip(px_n + offset * np.cos(angle), 0.02, 0.98)
            ry_n   = np.clip(py_n + offset * np.sin(angle), 0.02, 0.98)

            wx = rx_n * WORLD_SIZE - half
            wy = ry_n * WORLD_SIZE - half
            tz = get_terrain_height(hmap, rx_n, ry_n)

            # Ridge rocks tend to be smaller
            sx = rng_rock.uniform(0.08, 0.25)
            sy = rng_rock.uniform(0.08, 0.22)
            sz = rng_rock.uniform(0.06, 0.18)

            tilt_x = rng_rock.uniform(-0.25, 0.25)
            tilt_y = rng_rock.uniform(-0.25, 0.25)
            rot_z  = rng_rock.uniform(0, 3.14159)

            base_c = rng_rock.uniform(0.22, 0.35)
            rocks.append({
                'name': f'rock_{rock_id}',
                'x':    wx,
                'y':    wy,
                'z':    tz + sz * 0.45,
                'rx':   tilt_x,
                'ry':   tilt_y,
                'rz':   rot_z,
                'sx':   sx,
                'sy':   sy,
                'sz':   sz,
                'r':    base_c,
                'g':    base_c - 0.02,
                'b':    base_c - 0.03,
            })
            rock_id += 1

    # ── Isolated scattered rocks ───────────────────────────────────────────
    for _ in range(N_ISOLATED):
        rx_n = rng_rock.uniform(0.05, 0.95)
        ry_n = rng_rock.uniform(0.05, 0.95)
        wx   = rx_n * WORLD_SIZE - half
        wy   = ry_n * WORLD_SIZE - half
        tz   = get_terrain_height(hmap, rx_n, ry_n)

        # Isolated rocks — mix of all sizes
        size_class = rng_rock.choice(['small', 'medium', 'large'],
                                     p=[0.55, 0.35, 0.10])
        if size_class == 'small':
            sx = rng_rock.uniform(0.06, 0.15)
            sy = rng_rock.uniform(0.06, 0.15)
            sz = rng_rock.uniform(0.05, 0.12)
        elif size_class == 'medium':
            sx = rng_rock.uniform(0.18, 0.35)
            sy = rng_rock.uniform(0.16, 0.32)
            sz = rng_rock.uniform(0.12, 0.24)
        else:
            sx = rng_rock.uniform(0.38, 0.55)
            sy = rng_rock.uniform(0.35, 0.50)
            sz = rng_rock.uniform(0.25, 0.40)

        tilt_x = rng_rock.uniform(-0.3, 0.3)
        tilt_y = rng_rock.uniform(-0.3, 0.3)
        rot_z  = rng_rock.uniform(0, 3.14159)

        base_c = rng_rock.uniform(0.20, 0.36)
        r_col  = base_c + rng_rock.uniform(-0.04, 0.04)
        g_col  = base_c + rng_rock.uniform(-0.04, 0.04)
        b_col  = base_c + rng_rock.uniform(-0.04, 0.04)

        rocks.append({
            'name': f'rock_{rock_id}',
            'x':    wx,
            'y':    wy,
            'z':    tz + sz * 0.45,
            'rx':   tilt_x,
            'ry':   tilt_y,
            'rz':   rot_z,
            'sx':   sx,
            'sy':   sy,
            'sz':   sz,
            'r':    r_col,
            'g':    g_col,
            'b':    b_col,
        })
        rock_id += 1

    print(f"  ✓ Placed {len(rocks)} rocks total")
    return rocks


# ══════════════════════════════════════════════════════════════════════════════
# PART 4 — SDF WORLD FILE
# ══════════════════════════════════════════════════════════════════════════════

def rock_to_sdf(rock: dict) -> str:
    """Convert a rock dict to an SDF model block."""
    return f"""
    <model name='{rock['name']}'>
      <static>true</static>
      <pose>{rock['x']:.4f} {rock['y']:.4f} {rock['z']:.4f} {rock['rx']:.4f} {rock['ry']:.4f} {rock['rz']:.4f}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <box><size>{rock['sx']:.4f} {rock['sy']:.4f} {rock['sz']:.4f}</size></box>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.9</mu><mu2>0.9</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <sphere><radius>{min(rock['sx'], rock['sy'], rock['sz']) * 0.55:.4f}</radius></sphere>
          </geometry>
          <material>
            <ambient>{rock['r']:.3f} {rock['g']:.3f} {rock['b']:.3f} 1</ambient>
            <diffuse>{rock['r']:.3f} {rock['g']:.3f} {rock['b']:.3f} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def generate_sdf(hmap: np.ndarray,
                 rocks: list,
                 mean_height: float) -> None:
    print(f"\n{'='*60}")
    print(f" PART 4 — SDF World File")
    print(f"{'='*60}")

    robot_z = mean_height + 1.2   # spawn above mean terrain height

    rock_sdf = "\n".join(rock_to_sdf(r) for r in rocks)

    sdf = f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='lunar_surface'>

    <!-- ══ PHYSICS ══ -->
    <physics name='1ms' type='ignored'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin name='gz::sim::systems::Physics'
            filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands'
            filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster'
            filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Contact'
            filename='gz-sim-contact-system'/>

    <!-- Lunar gravity and near-zero magnetic field -->
    <gravity>0 0 -1.62</gravity>
    <magnetic_field>1e-09 1e-09 1e-09</magnetic_field>

    <!-- ══ SCENE ══ -->
    <scene>
      <ambient>0.25 0.25 0.25 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- ══ SUN — low grazing angle, harsh unfiltered ══ -->
    <light name='sun' type='directional'>
      <pose>0 0 100 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.2</intensity>
      <direction>-0.5 0.1 -0.86</direction>
      <diffuse>1.00 0.98 0.95 1</diffuse>
      <specular>0.30 0.28 0.25 1</specular>
      <attenuation>
        <range>100000</range>
        <linear>0.0</linear>
        <constant>1.0</constant>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

    <!-- Earthshine — faint blue fill, no shadows -->
    <light name='earthshine' type='directional'>
      <pose>0 0 50 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>0.10</intensity>
      <direction>0.3 -0.2 -0.9</direction>
      <diffuse>0.55 0.65 0.90 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <attenuation>
        <range>100000</range>
        <linear>0.0</linear>
        <constant>1.0</constant>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

    <!-- Overhead fill — fixes black top-down view -->
    <light name='overhead_fill' type='directional'>
      <pose>0 0 100 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>0.25</intensity>
      <direction>0.0 0.0 -1.0</direction>
      <diffuse>0.70 0.68 0.65 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <attenuation>
        <range>100000</range>
        <linear>0.0</linear>
        <constant>1.0</constant>
        <quadratic>0.0</quadratic>
      </attenuation>
    </light>

    <!-- ══ LUNAR TERRAIN ══
         collision + visual both use same DAE mesh
         perfect 1:1 match — rover drives on what it sees
         Mean height: {mean_height:.3f}m
         Robot spawn z: {robot_z:.3f}m                    -->
    <model name='lunar_terrain'>
      <static>true</static>
      <link name='link'>

        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>file://{INSTALL_MEDIA}/lunar_terrain.dae</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
                <fdir1>0 0 0</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>10000000</kp>
                <kd>10</kd>
                <min_depth>0.0</min_depth>
                <max_vel>0.01</max_vel>
              </ode>
            </contact>
          </surface>
        </collision>

        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>file://{INSTALL_MEDIA}/lunar_terrain.dae</uri>
            </mesh>
          </geometry>
        </visual>

      </link>
    </model>

    <!-- ══ ROCKS ══
         {len(rocks)} rocks total
         Clustered near crater rims + ridge flanks + isolated scatter
         Static — collision boxes, sphere visuals              -->
{rock_sdf}

    <!-- ══ GUI CAMERA ══ -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-20 -20 25 0 0.70 0.80</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
        <clip>
          <near>0.1</near>
          <far>500</far>
        </clip>
      </camera>
    </gui>

  </world>
</sdf>
"""

    with open(SDF_OUTPUT, "w") as f:
        f.write(sdf)
    print(f"  ✓ Saved: {SDF_OUTPUT}  ({len(rocks)} rocks embedded)")
    print(f"  ✓ robot_z suggestion: {robot_z:.3f}m")


# ══════════════════════════════════════════════════════════════════════════════
# PART 5 — AUTO COPY TO INSTALL
# ══════════════════════════════════════════════════════════════════════════════

def auto_copy() -> None:
    print(f"\n{'='*60}")
    print(f" PART 5 — Auto Copy to Install")
    print(f"{'='*60}")

    os.makedirs(INSTALL_MEDIA, exist_ok=True)
    os.makedirs(INSTALL_WORLD, exist_ok=True)
    os.makedirs(SRC_WORLD,     exist_ok=True)

    copies = [
        (HM_OUTPUT,   os.path.join(INSTALL_MEDIA, HM_OUTPUT)),
        (MESH_OUTPUT, os.path.join(INSTALL_MEDIA, MESH_OUTPUT)),
        (SDF_OUTPUT,  os.path.join(INSTALL_WORLD, SDF_OUTPUT)),
        (SDF_OUTPUT,  os.path.join(SRC_WORLD,     SDF_OUTPUT)),
    ]

    for src, dst in copies:
        shutil.copy2(src, dst)
        print(f"  ✓ {src}  →  {dst}")


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("\n🌕 Lunar World Generator — All-in-One")
    print("="*60)

    rng = np.random.default_rng(seed=SEED)

    # ── 1. Heightmap ──────────────────────────────────────────────────────
    hmap, large_craters, ridge_points = build_heightmap(HM_SIZE, rng)
    save_heightmap(hmap, HM_OUTPUT)

    # ── 2. Mesh ───────────────────────────────────────────────────────────
    generate_mesh(hmap)

    # ── 3. Rock placement ─────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f" PART 3 — Rock Placement")
    print(f"{'='*60}")
    rocks = place_rocks(hmap, large_craters, ridge_points, rng)

    # ── 4. SDF ────────────────────────────────────────────────────────────
    mean_h = float(hmap.mean()) * MAX_HEIGHT
    generate_sdf(hmap, rocks, mean_h)

    # ── 5. Copy to install ────────────────────────────────────────────────
    auto_copy()

    print(f"\n{'='*60}")
    print(f"✅ Done! Run:")
    print(f"   bash togo_sim.sh")
    print(f"   robot_z should be ≈ {mean_h + 1.2:.2f}m")
    print(f"{'='*60}\n")
