"""Tileable regolith PBR texture maps (docs/DESIGN.md section 9).

``make_regolith_textures(cfg, out_dir)`` writes three seamless PNG maps that cover one UV repeat
(``cfg.mesh.uv_tile_m.core`` = 2 m with ``texture_px`` = 2048, i.e. ~1 mm per texel):

* ``regolith_albedo.png`` - sRGB 8-bit. Linear base colour ``base_color_linear`` (0.12, 0.107,
  0.088) times (1 + v), where v is a low-contrast variation (99 % of texels within
  +-``albedo_variation``, rare specks clipped at 1.5x that): 10 cm - 1 m mottling, 1-5 cm
  patches, ~1 mm grain, soil clods (3-25 mm, ~6 % cover, slightly lighter or darker) and sparse
  bright (plagioclase) / dark (glass bead) specks, plus a +-1.5 % warm/cool tint. The texture
  mean is the base colour exactly (in linear space).
* ``regolith_normal.png`` - tangent-space normal map, linear 8-bit, OpenGL convention: R = +x
  (image right = east), G = +y (image up = north), B = +z. It is computed with periodic central
  differences from a sub-centimetre micro-relief (fBm with the terrain Hurst exponent between
  2 texels and 4 cm, plus the clods as smooth bumps) scaled to ``normal_detail_rms_slope_deg``
  (12 deg) directional RMS slope. ``flip_green=True`` (or config
  ``materials.regolith.normal_flip_green``) writes the DirectX convention (G = -y) instead.
* ``regolith_roughness.png`` - 0.95 +- 0.02, stored as grey RGB (R = G = B, so it does not
  matter which channel the renderer samples).

All components are synthesised periodically (FFT filtering of white noise, wrap-around
stamping and filtering), so every map tiles seamlessly; the synthesis is deterministic through
``config.rng(cfg, "materials.regolith")``.

UV orientation: ground UVs are ``u = x / tile``, ``v = -y / tile`` (``mesh.uv``); with v = 0 at the
top image row the map lies unmirrored on the ground with image up = north.

Verified in Gazebo Harmonic / ogre2 (docs/impl/mesh.md, experiments/impl_mesh/gz):

* the OpenGL normal map (G = +y) shades like the true relief on both axes, the G-flipped map
  inverts north/south, and a GLB TANGENT attribute changes nothing -> keep ``flip_green=False``;
* the UV orientation test texture appears upright and unmirrored (image up = north);
* ogre2 MULTIPLIES the SDF ``<diffuse>`` colour with ``albedo_map`` (diffuse = base colour plus
  this map rendered ~4x too dark): use ``<diffuse>1 1 1 1</diffuse>`` with these maps, whose
  mean already is the calibrated base colour;
* the Ogre-Next PBS shader also multiplies ``<roughness>`` with ``roughness_map`` (from the
  shader, not measured): use ``<roughness>1.0</roughness>`` with the roughness map.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

from ..config import rng as config_rng

__all__ = [
    "linear_to_srgb",
    "srgb_to_linear",
    "periodic_noise",
    "height_to_normal_map",
    "synthesize_regolith",
    "make_regolith_textures",
    "seam_ratio",
]


# ---------------------------------------------------------------------------------------------
# colour helpers
# ---------------------------------------------------------------------------------------------
def linear_to_srgb(c: np.ndarray) -> np.ndarray:
    """IEC 61966-2-1 sRGB encoding of linear values in [0, 1]."""
    c = np.clip(np.asarray(c, dtype=np.float64), 0.0, 1.0)
    return np.where(c <= 0.0031308, 12.92 * c, 1.055 * np.power(c, 1.0 / 2.4) - 0.055)


def srgb_to_linear(s: np.ndarray) -> np.ndarray:
    """Inverse of ``linear_to_srgb``."""
    s = np.clip(np.asarray(s, dtype=np.float64), 0.0, 1.0)
    return np.where(s <= 0.04045, s / 12.92, np.power((s + 0.055) / 1.055, 2.4))


# ---------------------------------------------------------------------------------------------
# periodic synthesis
# ---------------------------------------------------------------------------------------------
def _radial_freq(n: int, pixel_m: float) -> np.ndarray:
    fy = np.fft.fftfreq(n, d=pixel_m)[:, None]
    fx = np.fft.rfftfreq(n, d=pixel_m)[None, :]
    return np.hypot(fx, fy)  # cycles per metre, rfft2 layout (n, n // 2 + 1)


def periodic_noise(n: int, pixel_m: float, rng: np.random.Generator, amplitude_fn) -> np.ndarray:
    """Zero-mean, unit-std periodic Gaussian field: white noise filtered by ``amplitude_fn(f)``.

    ``amplitude_fn`` maps the radial frequency f (cycles per metre) to the filter amplitude
    (the square root of the PSD shape); the field is periodic over n * pixel_m in x and y.
    """
    white = rng.standard_normal((n, n))
    spec = np.fft.rfft2(white)
    f = _radial_freq(n, pixel_m)
    amp = np.asarray(amplitude_fn(np.maximum(f, 1e-12)), dtype=np.float64)
    amp[0, 0] = 0.0
    field = np.fft.irfft2(spec * amp, s=(n, n))
    field -= field.mean()
    std = field.std()
    return field / std if std > 0 else field


def _band(f, lam_min: float, lam_max: float, order: int = 4) -> np.ndarray:
    """Smooth band-pass weight: Butterworth-like high-pass at 1/lam_max, low-pass at 1/lam_min."""
    hp = (f * lam_max) ** order / np.sqrt(1.0 + (f * lam_max) ** (2 * order))
    lp = 1.0 / np.sqrt(1.0 + (f * lam_min) ** (2 * order))
    return hp * lp


def _wrap_gradient(h: np.ndarray, pixel_m: float) -> tuple[np.ndarray, np.ndarray]:
    """(dh/dx, dh/dy) in world axes with periodic central differences.

    Image layout: column c -> x = c * pixel (east), row r -> y = -r * pixel (row 0 = north edge).
    """
    dh_dc = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) / (2.0 * pixel_m)
    dh_dr = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) / (2.0 * pixel_m)
    return dh_dc, -dh_dr


def height_to_normal_map(h: np.ndarray, pixel_m: float, *, flip_green: bool = False) -> np.ndarray:
    """uint8 (n, n, 3) tangent-space normal map of a periodic height field (OpenGL: G = +y = up)."""
    gx, gy = _wrap_gradient(np.asarray(h, dtype=np.float64), pixel_m)
    inv = 1.0 / np.sqrt(gx * gx + gy * gy + 1.0)
    nx, ny, nz = -gx * inv, -gy * inv, inv
    if flip_green:
        ny = -ny
    rgb = np.stack([nx, ny, nz], axis=-1) * 0.5 + 0.5
    return np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)


def _stamp_periodic(target: np.ndarray, patch: np.ndarray, r0: int, c0: int) -> None:
    """target[(r0 + i) % n, (c0 + j) % n] += patch[i, j] (in place)."""
    n = target.shape[0]
    rows = (r0 + np.arange(patch.shape[0])) % n
    cols = (c0 + np.arange(patch.shape[1])) % n
    target[np.ix_(rows, cols)] += patch


def _clods(n: int, pixel_m: float, rng: np.random.Generator, coverage: float, d_min: float, d_max: float,
           b: float = 2.0) -> tuple[np.ndarray, np.ndarray, int]:
    """Periodic field of soil clods: (relief in metres, albedo offset, count)."""
    relief = np.zeros((n, n), dtype=np.float64)
    albedo = np.zeros((n, n), dtype=np.float64)
    area_total = coverage * (n * pixel_m) ** 2
    area = 0.0
    count = 0
    ratio_b = (d_min / d_max) ** b
    while area < area_total and count < 200000:
        u = rng.random()
        d = d_min * (1.0 - u * (1.0 - ratio_b)) ** (-1.0 / b)  # truncated power law N(>D) ~ D^-b
        a = 0.5 * d / pixel_m  # semi-major axis in pixels
        q = rng.uniform(0.55, 1.0)  # axis ratio
        th = rng.uniform(0.0, np.pi)
        height = rng.uniform(0.25, 0.5) * 0.5 * d  # 25-50 % of the radius
        dalb = rng.uniform(-0.05, 0.07)
        cr, cc = rng.uniform(0.0, n, size=2)
        half = int(np.ceil(a)) + 1
        r0 = int(np.floor(cr)) - half
        c0 = int(np.floor(cc)) - half
        rr = r0 + np.arange(2 * half + 1)[:, None] - cr
        cc_ = c0 + np.arange(2 * half + 1)[None, :] - cc
        ct, st = np.cos(th), np.sin(th)
        u_ = (cc_ * ct + rr * st) / a
        v_ = (-cc_ * st + rr * ct) / (a * q)
        r2 = u_ * u_ + v_ * v_
        inside = r2 < 1.0
        shape = np.where(inside, (1.0 - np.minimum(r2, 1.0)) ** 1.5, 0.0)
        _stamp_periodic(relief, height * shape, r0, c0)
        _stamp_periodic(albedo, dalb * np.where(inside, np.sqrt(1.0 - np.minimum(r2, 1.0)), 0.0), r0, c0)
        area += np.pi * (0.5 * d) ** 2 * q
        count += 1
    return relief, albedo, count


def seam_ratio(img: np.ndarray, axis: int) -> float:
    """Mean |difference| across the wrap-around seam / mean |difference| between interior neighbours.

    About 1 for a seamless (periodic) texture; much larger when the opposite edges do not match.
    """
    a = np.asarray(img, dtype=np.float64)
    if a.ndim == 2:
        a = a[..., None]
    d_int = np.abs(np.diff(a, axis=axis)).mean()
    first = np.take(a, 0, axis=axis)
    last = np.take(a, -1, axis=axis)
    d_seam = np.abs(first - last).mean()
    return float(d_seam / max(d_int, 1e-12))


# ---------------------------------------------------------------------------------------------
# synthesis
# ---------------------------------------------------------------------------------------------
def _params(cfg) -> dict:
    reg = cfg["materials"]["regolith"]
    tile_m = float(cfg["mesh"]["uv_tile_m"]["core"])
    hurst = float(((cfg.get("terrain") or {}).get("roughness") or {}).get("hurst", 0.8))
    return {
        "base_color_linear": [float(c) for c in reg["base_color_linear"]],
        "albedo_variation": float(reg.get("albedo_variation", 0.12)),
        "roughness": float(reg.get("roughness", 0.95)),
        "texture_px": int(reg.get("texture_px", 2048)),
        "normal_detail_rms_slope_deg": float(reg.get("normal_detail_rms_slope_deg", 12.0)),
        "normal_flip_green": bool(reg.get("normal_flip_green", False)),
        "uv_tile_m": tile_m,
        "hurst": hurst,
    }


def synthesize_regolith(cfg, *, px: int | None = None, flip_green: bool | None = None) -> dict:
    """Synthesise the regolith maps in memory.

    Returns dict(albedo (n, n, 3) uint8 sRGB, normal (n, n, 3) uint8, roughness (n, n, 3)
    uint8, height (n, n) float32 micro-relief in metres, stats dict, params dict).
    """
    t0 = time.perf_counter()
    prm = _params(cfg)
    n = int(px or prm["texture_px"])
    if n < 16 or n % 2:
        raise ValueError("texture size must be an even number >= 16")
    flip = prm["normal_flip_green"] if flip_green is None else bool(flip_green)
    tile_m = prm["uv_tile_m"]
    pix = tile_m / n
    rng = config_rng(cfg, "materials.regolith")
    H = prm["hurst"]
    beta = 2.0 + 2.0 * H  # height PSD exponent, as for the terrain fBm

    # ---- micro-relief (normal map): fBm between 2 texels and 4 cm + soil clods
    lam_lo = 2.0 * pix
    fbm = periodic_noise(n, pix, rng, lambda f: f ** (-beta / 2.0) * _band(f, 0.75 * lam_lo, 0.04, order=3))
    clod_relief, clod_albedo, n_clods = _clods(n, pix, rng, coverage=0.06, d_min=0.003, d_max=0.025)
    # relative weight of clods vs fBm, measured in slope: clods carry ~45 % of the slope variance
    gx_f, gy_f = _wrap_gradient(fbm, pix)
    gx_c, gy_c = _wrap_gradient(clod_relief, pix)
    s_f = np.sqrt(0.5 * (np.mean(gx_f ** 2) + np.mean(gy_f ** 2)))
    s_c = np.sqrt(0.5 * (np.mean(gx_c ** 2) + np.mean(gy_c ** 2)))
    w_c = 0.45
    h = fbm * (np.sqrt(1.0 - w_c) / s_f) + (clod_relief * (np.sqrt(w_c) / s_c) if s_c > 0 else 0.0)
    gx, gy = _wrap_gradient(h, pix)
    s_now = np.sqrt(0.5 * (np.mean(gx ** 2) + np.mean(gy ** 2)))
    target = np.tan(np.radians(prm["normal_detail_rms_slope_deg"]))
    h *= target / s_now
    normal = height_to_normal_map(h, pix, flip_green=flip)
    gx, gy = _wrap_gradient(h, pix)
    slope_rms_dir = float(np.degrees(np.arctan(np.sqrt(0.5 * (np.mean(gx ** 2) + np.mean(gy ** 2))))))

    # ---- albedo variation (relative, linear)
    mottle = periodic_noise(n, pix, rng, lambda f: f ** (-beta / 2.0) * _band(f, 0.10, 1.0))
    patches = periodic_noise(n, pix, rng, lambda f: f ** (-beta / 2.0) * _band(f, 0.01, 0.05))
    grain = ndimage.gaussian_filter(rng.standard_normal((n, n)), 0.7, mode="wrap")
    grain /= grain.std()
    specks = np.zeros((n, n))
    n_bright = int(0.0015 * n * n)
    n_dark = int(0.0020 * n * n)
    idx_b = rng.integers(0, n, size=(n_bright, 2))
    idx_d = rng.integers(0, n, size=(n_dark, 2))
    np.add.at(specks, (idx_b[:, 0], idx_b[:, 1]), rng.uniform(0.8, 1.6, n_bright))
    np.add.at(specks, (idx_d[:, 0], idx_d[:, 1]), -rng.uniform(0.8, 1.6, n_dark))
    specks = ndimage.gaussian_filter(specks, 0.6, mode="wrap") * (2.0 * np.pi * 0.36)  # unit peak
    v = 0.022 * mottle + 0.020 * patches + 0.028 * grain + clod_albedo
    var = prm["albedo_variation"]
    q99 = float(np.percentile(np.abs(v), 99.0))
    if q99 > 0:
        v *= var / q99  # 99 % of texels within +-variation
    v = v + 0.9 * var * specks  # rare 1-2 texel specks, clipped below
    v = np.clip(v, -1.5 * var, 1.5 * var)
    v -= v.mean()
    tint = periodic_noise(n, pix, rng, lambda f: _band(f, 0.05, 1.0))
    base = np.asarray(prm["base_color_linear"], dtype=np.float64)
    lin = base[None, None, :] * (1.0 + v[..., None]) * (1.0 + 0.015 * tint[..., None] * np.array([1.0, 0.0, -1.0]))
    srgb = linear_to_srgb(lin)
    # triangular dither of +-1 LSB keeps the slow mottling free of 8-bit banding
    dith = (rng.random((n, n, 1)) - rng.random((n, n, 1))) / 255.0
    albedo = np.clip(np.rint((srgb + dith) * 255.0), 0, 255).astype(np.uint8)

    # ---- roughness
    rough_noise = periodic_noise(n, pix, rng, lambda f: _band(f, 0.02, 0.5))
    rough_f = prm["roughness"] + 0.02 * np.clip(rough_noise / 2.5, -1.0, 1.0)
    rough8 = np.clip(np.rint(np.clip(rough_f, 0.0, 1.0) * 255.0), 0, 255).astype(np.uint8)
    roughness = np.repeat(rough8[..., None], 3, axis=2)

    lin_back = srgb_to_linear(albedo.astype(np.float64) / 255.0)
    stats = {
        "texture_px": n,
        "pixel_mm": pix * 1000.0,
        "uv_tile_m": tile_m,
        "albedo_mean_linear": [float(c) for c in lin_back.reshape(-1, 3).mean(axis=0)],
        "albedo_rel_p01_p99": [float(np.percentile(v, 1)), float(np.percentile(v, 99))],
        "albedo_rel_std": float(v.std()),
        "normal_rms_slope_dir_deg": slope_rms_dir,
        "normal_flip_green": flip,
        "micro_relief_rms_mm": float(h.std() * 1000.0),
        "n_clods": int(n_clods),
        "roughness_mean": float(rough8.mean() / 255.0),
        "seam_ratio_albedo": [seam_ratio(albedo, 0), seam_ratio(albedo, 1)],
        "seam_ratio_normal": [seam_ratio(normal, 0), seam_ratio(normal, 1)],
        "time_s": round(time.perf_counter() - t0, 3),
    }
    return {"albedo": albedo, "normal": normal, "roughness": roughness, "height": h.astype(np.float32),
            "stats": stats, "params": prm}


def make_regolith_textures(cfg, out_dir: str | Path, *, px: int | None = None, flip_green: bool | None = None,
                           prefix: str = "regolith", write_flipped: bool = False) -> dict[str, str]:
    """Write the tileable regolith maps into ``out_dir`` and return their paths.

    Returns dict(albedo, normal, roughness, meta[, normal_flipped]) of absolute path strings.
    ``write_flipped`` additionally writes the normal map with the other green convention
    (``<prefix>_normal_flipg.png``) for renderer checks.
    """
    from PIL import Image

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    res = synthesize_regolith(cfg, px=px, flip_green=flip_green)
    paths = {
        "albedo": out / f"{prefix}_albedo.png",
        "normal": out / f"{prefix}_normal.png",
        "roughness": out / f"{prefix}_roughness.png",
        "meta": out / f"{prefix}_textures.json",
    }
    t0 = time.perf_counter()
    Image.fromarray(res["albedo"], "RGB").save(paths["albedo"])
    Image.fromarray(res["normal"], "RGB").save(paths["normal"])
    Image.fromarray(res["roughness"], "RGB").save(paths["roughness"])
    if write_flipped:
        flipped = res["normal"].copy()
        flipped[..., 1] = 255 - flipped[..., 1]
        paths["normal_flipped"] = out / f"{prefix}_normal_flipg.png"
        Image.fromarray(flipped, "RGB").save(paths["normal_flipped"])
    stats = dict(res["stats"])
    stats["write_s"] = round(time.perf_counter() - t0, 3)
    stats["bytes"] = {k: p.stat().st_size for k, p in paths.items() if k != "meta"}
    meta = {"params": res["params"], "stats": stats,
            "files": {k: p.name for k, p in paths.items() if k != "meta"},
            "convention": {"uv": "u = x / uv_tile_m, v = -y / uv_tile_m (v = 0 at the top image row)",
                           "normal": "DirectX (G = -y)" if stats["normal_flip_green"] else "OpenGL (G = +y = image up = north)",
                           "albedo": "sRGB 8-bit", "roughness": "linear grey RGB"}}
    paths["meta"].write_text(json.dumps(meta, indent=2))
    return {k: str(p) for k, p in paths.items()}
