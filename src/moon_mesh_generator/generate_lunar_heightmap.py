"""
generate_lunar_heightmap.py  (v11 — 50m Dense Lunar Surface)

Root cause fix (v11):
  The additive stamping approach caused every crater bowl + rim + ejecta
  to raise intercrater terrain, stacking thousands of positive offsets
  into a chaotic bumpy floor. No amount of count/depth tuning fixes this.

New approach:
  - Craters are stamped onto a SEPARATE flat canvas using np.minimum
    for bowls (can only carve DOWN, never push up)
  - Only large + medium crater rims are additive (physically justified)
  - Crater canvas is combined with base terrain via np.minimum blend
  - fBm roughness layers significantly reduced
  - SIZE 2048→4096 for higher source detail
  - GRID_RES 513→1025 for 0.049m mesh vertex spacing (was 0.098m)

Requires: numpy, Pillow, scipy
  pip install numpy Pillow scipy
"""

import numpy as np
from PIL import Image
from numpy.fft import fft2, ifft2, fftfreq
from scipy.ndimage import gaussian_filter
from pathlib import Path

# ── Configuration ─────────────────────────────────────────────────────────────
SIZE        = 4096          # was 2048 — 4x more heightmap detail
OUTPUT_PATH = str(Path(__file__).resolve().parent / "lunar_terrain.png")
WORLD_SIZE  = 50.0
MAX_HEIGHT  = 1.2
SEED        = 42


# ── Crater stamp — MIN-based carving ─────────────────────────────────────────
# ── Crater stamp — change signature ──────────────────────────────────────────
def stamp_crater(canvas: np.ndarray,
                 rim_layer: np.ndarray,
                 cx_n: float, cy_n: float,
                 r_n: float,
                 depth: float,
                 rim_h: float,
                 degraded: bool = False,
                 has_rim: bool = False,
                 bowl_power: float = 2.8) -> None:   # ← NEW parameter
    ...
    if degraded:
        m = d < 1.0
        if np.any(m):
            bowl_vals = depth * (d[m] ** bowl_power - 1.0) * 0.25
            canvas[y0:y1, x0:x1][m] = np.minimum(
                canvas[y0:y1, x0:x1][m], bowl_vals)
        ...
    else:
        m = d < 0.88
        if np.any(m):
            bowl_vals = depth * ((d[m] / 0.88) ** bowl_power - 1.0)
            canvas[y0:y1, x0:x1][m] = np.minimum(
                canvas[y0:y1, x0:x1][m], bowl_vals)
        ...


# ── Layer A — grid bowls ──────────────────────────────────────────────────────
        depth = r * rng.uniform(1.5, 2.5)
        deg   = rng.random() < 0.90
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=bool(deg), has_rim=False,
                     bowl_power=5.0)               # ← flat floor


# ── Layer B — small scatter ───────────────────────────────────────────────────
        depth = r * rng.uniform(4.0, 7.0)
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=bool(deg), has_rim=False,
                     bowl_power=4.0)               # ← gentler than default


# ── Layer C — micro pockmarks ─────────────────────────────────────────────────
    n_micro  = 50                                 # was 200
    ...
    for cx, cy, r in zip(micro_cx, micro_cy, micro_r):
        depth = r * rng.uniform(0.1, 0.16)         # was 0.8–1.5
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=True, has_rim=False,
                     bowl_power=7.0)               # ← very flat


# ── Layer D — ultra-micro specks ──────────────────────────────────────────────
    n_ultra  = 70                                 # was 200
    ...
    for cx, cy, r in zip(ultra_cx, ultra_cy, ultra_r):
        depth = r * rng.uniform(0.1, 0.15)         # was 0.5–1.0
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=True, has_rim=False,
                     bowl_power=10.0)              # ← pure dimple

# ── fBm terrain ───────────────────────────────────────────────────────────────
def fbm_terrain(size: int, H: float, rng: np.random.Generator) -> np.ndarray:
    phase          = rng.uniform(0, 2 * np.pi, (size, size))
    amplitude      = rng.rayleigh(1.0, (size, size))
    white          = amplitude * np.exp(1j * phase)
    fx             = fftfreq(size).reshape(1, -1)
    fy             = fftfreq(size).reshape(-1, 1)
    freq           = np.sqrt(fx**2 + fy**2)
    freq[0, 0]     = 1.0
    beta           = 2.0 * H + 2.0
    power_filter   = freq ** (-beta / 2.0)
    power_filter[0, 0] = 0.0
    return np.real(ifft2(white * power_filter))


def bandpass_noise(size: int, f_lo: float, f_hi: float,
                   rng: np.random.Generator) -> np.ndarray:
    noise = rng.normal(0.0, 1.0, (size, size))
    F     = fft2(noise)
    fx    = fftfreq(size).reshape(1, -1)
    fy    = fftfreq(size).reshape(-1, 1)
    freq  = np.sqrt(fx**2 + fy**2)
    band  = (np.exp(-0.5 * ((freq - f_hi) / (0.3 * f_hi))**2)
             - np.exp(-0.5 * ((freq - f_lo) / (0.3 * f_lo))**2))
    band  = np.clip(band, 0, None)
    return np.real(ifft2(F * band))


# ── Build heightmap ───────────────────────────────────────────────────────────
def build_heightmap(size: int) -> np.ndarray:
    print(f"Building {size}×{size} lunar heightmap ({WORLD_SIZE}×{WORLD_SIZE}m) …")
    rng = np.random.default_rng(seed=SEED)

    # ── 1. fBm base terrain ───────────────────────────────────────────────
    print("  [1/5] fBm base terrain (H=0.80) …")
    base   = fbm_terrain(size, H=0.80, rng=rng)
    xc     = np.linspace(-0.04, 0.04, size)
    yc     = np.linspace(-0.03, 0.03, size)
    X, Y   = np.meshgrid(xc, yc)
    base  += 0.04 * X + 0.03 * Y
    lo, hi = base.min(), base.max()
    hmap   = 0.10 + 0.35 * (base - lo) / (hi - lo)

    # ── 2. Roughness — reduced weights vs v10 ────────────────────────────
    # v11: all weights roughly halved — fBm was contributing to bumpiness
    print("  [2/5] Roughness layers (reduced) …")
    hmap += 0.008 * bandpass_noise(size, 0.004, 0.020, rng)   # was 0.018
    hmap += 0.005 * bandpass_noise(size, 0.020, 0.060, rng)   # was 0.011
    hmap += 0.003 * bandpass_noise(size, 0.060, 0.160, rng)   # was 0.006
    hmap += 0.001 * bandpass_noise(size, 0.160, 0.380, rng)   # was 0.003
    # Highest freq band removed entirely — pure pixel noise at this scale

    # ── 2b. Regional patch elevation ──────────────────────────────────────
    print("  [2b] Regional elevation patches …")
    patch_noise  = bandpass_noise(size, 0.02, 0.06, rng)
    lo_p, hi_p   = patch_noise.min(), patch_noise.max()
    hmap        += 0.06 * (patch_noise - lo_p) / (hi_p - lo_p)  # was 0.083

    broad_noise  = bandpass_noise(size, 0.005, 0.015, rng)
    lo_b, hi_b   = broad_noise.min(), broad_noise.max()
    hmap        += 0.04 * (broad_noise - lo_b) / (hi_b - lo_b)  # was 0.05

    # ── 3+4. Craters — separate canvas + rim layer ────────────────────────
    # v11: all craters go onto a zeroed canvas using MIN-based carving.
    #      Rims go onto a separate rim_layer (additive) for large/medium only.
    #      Final combination: hmap = hmap + crater_canvas + rim_layer
    print("  [3/5] Large craters (14) …")
    crater_canvas = np.zeros((size, size), dtype=np.float64)
    rim_layer     = np.zeros((size, size), dtype=np.float64)

    rng_l   = np.random.default_rng(seed=SEED + 10)
    n_large = 14
    large_cx  = rng_l.uniform(0.10, 0.90, n_large)
    large_cy  = rng_l.uniform(0.10, 0.90, n_large)
    large_r   = rng_l.uniform(0.015, 0.025, n_large)
    large_dep = rng_l.uniform(0.22, 0.32, n_large)
    large_rim = rng_l.uniform(0.05, 0.08, n_large)
    large_deg = rng_l.random(n_large) < 0.35

    for cx, cy, r, depth, rim_h, deg in zip(
            large_cx, large_cy, large_r,
            large_dep, large_rim, large_deg):
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, rim_h,
                     degraded=bool(deg), has_rim=True)

    print("  [4/5] Medium craters (80) …")
    rng_m  = np.random.default_rng(seed=SEED + 20)
    n_med  = 80
    med_cx  = rng_m.uniform(0.04, 0.96, n_med)
    med_cy  = rng_m.uniform(0.04, 0.96, n_med)
    med_r   = rng_m.uniform(0.005, 0.012, n_med)
    med_dep = rng_m.uniform(0.16, 0.26, n_med)
    med_rim = rng_m.uniform(0.03, 0.06, n_med)
    med_deg = rng_m.random(n_med) < 0.40

    for cx, cy, r, depth, rim_h, deg in zip(
            med_cx, med_cy, med_r,
            med_dep, med_rim, med_deg):
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, rim_h,
                     degraded=bool(deg), has_rim=True)

    # ── 5. Procedural scatter — all has_rim=False ────────────────────────
    print("  [5/5] Procedural scatter …")

    # Layer A — 4000 small bowls, no rim
    print("        Layer A: grid bowls (4000) …")
    n_grid = 4000
    grid_r = rng.uniform(0.002, 0.004, n_grid)
    cells  = np.array([(i, j)
                       for i in range(100)
                       for j in range(100)])
    rng.shuffle(cells)
    for idx, (ci, cj) in enumerate(cells[:n_grid]):
        cx    = (ci + rng.uniform(0.05, 0.95)) / 100.0
        cy    = (cj + rng.uniform(0.05, 0.95)) / 100.0
        r     = grid_r[idx]
        depth = r * rng.uniform(1.5, 2.5)
        deg   = rng.random() < 0.90
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=bool(deg), has_rim=False)

    # Layer B — 300 small scatter, no rim
    print("        Layer B: small scatter (300) …")
    n_small  = 300
    small_r  = rng.power(0.28, n_small) * 0.003 + 0.003
    small_cx = rng.uniform(0.01, 0.99, n_small)
    small_cy = rng.uniform(0.01, 0.99, n_small)
    small_deg = rng.random(n_small) < 0.55
    for cx, cy, r, deg in zip(small_cx, small_cy, small_r, small_deg):
        depth = r * rng.uniform(4.0, 7.0)
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=bool(deg), has_rim=False)

    # Layer C — 200 micro pockmarks
    print("        Layer C: micro pockmarks (200) …")
    n_micro  = 200
    micro_r  = rng.uniform(0.001, 0.002, n_micro)
    micro_cx = rng.uniform(0.01, 0.99, n_micro)
    micro_cy = rng.uniform(0.01, 0.99, n_micro)
    for cx, cy, r in zip(micro_cx, micro_cy, micro_r):
        depth = r * rng.uniform(0.8, 1.5)
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=True, has_rim=False)

    # Layer D — 200 ultra-micro specks
    print("        Layer D: ultra-micro specks (200) …")
    n_ultra  = 200
    ultra_r  = rng.uniform(0.0005, 0.001, n_ultra)
    ultra_cx = rng.uniform(0.01, 0.99, n_ultra)
    ultra_cy = rng.uniform(0.01, 0.99, n_ultra)
    for cx, cy, r in zip(ultra_cx, ultra_cy, ultra_r):
        depth = r * rng.uniform(0.5, 1.0)
        stamp_crater(crater_canvas, rim_layer,
                     cx, cy, r, depth, 0.0,
                     degraded=True, has_rim=False)

    # ── Combine layers ────────────────────────────────────────────────────
    # crater_canvas values are 0 or negative (MIN-carved bowls only)
    # rim_layer values are 0 or positive (additive rims, large/med only)
    # hmap stays as the rolling base — craters carve into it
    print("  Combining layers …")
    hmap = hmap + crater_canvas + rim_layer

    # ── Normalise ──────────────────────────────────────────────────────────
    print("  Normalising …")
    lo, hi = hmap.min(), hmap.max()
    hmap   = (hmap - lo) / (hi - lo)
    hmap   = np.clip(hmap, 0.0, 1.0)
    hmap   = np.power(hmap, 0.92)

    # Light blur — sub-pixel smoothing only
    print("  Gaussian anti-aliasing blur (sigma=0.7) …")
    hmap = gaussian_filter(hmap, sigma=0.7)
    hmap = np.clip(hmap, 0.0, 1.0)

    return hmap


# ── Save 16-bit PNG ───────────────────────────────────────────────────────────
def save_heightmap(hmap: np.ndarray, path: str) -> None:
    img_array = (hmap * 65535).astype(np.uint16)
    img = Image.fromarray(img_array, mode='I;16')
    img.save(path)
    n_total = 14 + 80 + 4000 + 300 + 200 + 200
    print(f"\n✓ Saved : {path}  ({hmap.shape[0]}×{hmap.shape[1]} px, 16-bit)")
    print(f"  World  : {WORLD_SIZE}×{WORLD_SIZE}m")
    print(f"  Height : 0.0 – {MAX_HEIGHT:.1f} m")
    print(f"  Craters: ~{n_total:,} total")
    print(f"  Density: ~{n_total / (WORLD_SIZE**2):.1f} craters/m²")
    print(f"  Approach: MIN-carve bowls + additive rims (large/med only)")


if __name__ == "__main__":
    hmap = build_heightmap(SIZE)
    save_heightmap(hmap, OUTPUT_PATH)
    print("\nDone. Run generate_lunar_mesh.py next.")
