"""Synthetic fractional-Brownian-motion roughness: F_coarse and F_fine (docs/DESIGN.md section 3).

Spectrum
    Isotropic two-sided 2-D PSD in cycles per metre (variance = integral of S over the plane)::

        S(f) = S0 * fbm_gain * f^-(2 + 2H)       S0 = terrain.roughness.psd_S0_m4, H = hurst

    split into two complementary bands with sharp radial masks (they sum to the full band):

    * ``F_coarse``: ``0 < f < f_split`` (wavelength >= split_wavelength_m = 0.5 m), times the DTM
      power-complementary amplitude weight ``W(f) = (f lc)^n / sqrt(1 + (f lc)^(2n))`` (lc = 10 m,
      n = 4: the Butterworth order of the DTM low-pass), so ``|H_dtm|^2 + W^2 = 1`` and no wavelength
      is counted twice. Synthesised once on a 10 cm grid over the ring square + COARSE_MARGIN_M; every
      zone samples it with cubic B-spline interpolation (``CubicSampler`` = ``Grid.sample(order=3)``).
    * ``F_fine``: ``f_split <= f <= 1 / min_wavelength_m`` (0.5 m .. 2 cm). Synthesised once on the
      1 cm grid over core + core.margin_m (101 m) and sampled by node index.

Synthesis
    Unit-variance white noise ``w`` (float32, ``config.rng(cfg, "terrain.fbm.<band>")``) on a padded
    square of ``N`` nodes (``scipy.fft.next_fast_len``), ``F = irfft2(rfft2(w) * sqrt(S(f)) / dx)``,
    then the centre is cropped. The padding (COARSE_PAD_M, FINE_PAD_M on each side) makes the field
    non-periodic over the cropped domain. With this filter the field's PSD is exactly S(f)
    (``Var = sum S df^2``). Everything runs in float32/complex64; the 2-D transforms are done as blocked
    1-D passes so the result is bitwise independent of the thread count (``synthesize``).

Caching and the gains
    The fields are cached at unit gain (``fbm_gain = 1``) in ``build_dir/terrain/fbm/<band>_unit.npy``
    (+ .json with the cache key). The gains multiply the PSD, so the amplitude scales with
    ``sqrt(gain)``; ``coarse_field(cfg)`` / ``fine_field(cfg)`` return ``sqrt(gain) * unit field``.
    Changing a gain therefore never re-synthesises (same noise, exactly scaled), which is what the
    calibration relies on. (The config comment calls fbm_gain an amplitude multiplier; DESIGN section 3
    and the task define it on the PSD. This module follows DESIGN: S0 = psd_S0_m4 * fbm_gain.)

    FID-06: the two bands have separate gains. ``terrain.roughness.fbm_gain`` is the coarse band's
    (wavelengths >= split_wavelength_m) and ``terrain.roughness.fbm_gain_fine`` the fine band's
    (2 cm - 0.5 m); a missing or null ``fbm_gain_fine`` means "same as fbm_gain" (the old single-gain
    behaviour). One shared gain could not reach the target at 1-3 cm without pushing 0.3 m out of the
    tolerance (crater relief and the DTM already supply most of the 0.1-0.3 m roughness).
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Callable

import numba as nb
import numpy as np
import scipy.fft as sfft
from scipy import ndimage

from ..config import Config
from ..config import rng as cfg_rng
from ..grid import Grid, to_lattice
from ..layout import Layout

FBM_VERSION = "2"          # 2: blocked FFT, bitwise independent of the thread count
COARSE_MARGIN_M = 2.0    # coarse field covers the ring square + this border (cubic support, ring margin)
COARSE_PAD_M = 64.0      # synthesis padding per side (dominant coarse wavelengths are ~10 m)
FINE_PAD_M = 2.0         # synthesis padding per side (fine band has wavelengths <= 0.5 m)
BANDS = ("coarse", "fine")


# ---------------------------------------------------------------------------------------- spectra
def fbm_psd(f: np.ndarray, S0: float, hurst: float) -> np.ndarray:
    """S(f) = S0 f^-(2+2H) for f > 0 (0 at f = 0)."""
    f = np.asarray(f, dtype=np.float64)
    safe = np.where(f > 0, f, 1.0)
    return np.where(f > 0, S0 * safe ** (-(2.0 + 2.0 * hurst)), 0.0)


def dtm_weight(f: np.ndarray, lambda_c: float, order: int = 4) -> np.ndarray:
    """Power-complementary amplitude weight W(f) = (f lc)^n / sqrt(1 + (f lc)^(2n)) of the DTM low-pass."""
    a = (np.asarray(f, dtype=np.float64) * lambda_c) ** order
    return a / np.sqrt(1.0 + a * a)


def band_params(cfg: Config, band: str) -> dict:
    """Everything that defines a unit-gain band field (also its cache key)."""
    if band not in BANDS:
        raise ValueError(f"band must be one of {BANDS}")
    r = cfg.terrain.roughness
    lay = Layout(cfg)
    if band == "coarse":
        spacing = float(r.coarse_field_spacing_m)
        half = lay.zone_rect("ring").x1 + COARSE_MARGIN_M
        pad = COARSE_PAD_M
    else:
        spacing = float(cfg.zones.core.spacing_m)
        half = lay.zone_rect("core").x1 + float(cfg.zones.core.margin_m)
        pad = FINE_PAD_M
    n = int(round(2 * half / spacing)) + 1
    to_lattice(half)        # raises if the extent is off the 1 cm lattice
    to_lattice(spacing)
    return {
        "version": FBM_VERSION, "band": band, "seed": int(cfg.world.seed), "S0_m4": float(r.psd_S0_m4),
        "hurst": float(r.hurst), "f_split": 1.0 / float(r.split_wavelength_m),
        "f_max": 1.0 / float(r.min_wavelength_m), "lambda_c_m": float(cfg.dtm.lowpass_lambda_m),
        "order": int(cfg.dtm.lowpass_order), "spacing_m": spacing, "half_m": half, "n": n,
        "pad_m": pad, "n_fft": int(sfft.next_fast_len(n + 2 * int(math.ceil(pad / spacing)), real=True)),
        "rng_stream": f"terrain.fbm.{band}",
    }


def band_transfer(p: dict, f: np.ndarray, gain: float = 1.0) -> np.ndarray:
    """Amplitude of the band at radial frequency f: sqrt(S(f) * gain) * mask(f) [* W(f) for coarse]."""
    f = np.asarray(f, dtype=np.float64)
    amp = np.sqrt(fbm_psd(f, p["S0_m4"] * gain, p["hurst"]))
    if p["band"] == "coarse":
        return np.where((f > 0) & (f < p["f_split"]), amp * dtm_weight(f, p["lambda_c_m"], p["order"]), 0.0)
    return np.where((f >= p["f_split"]) & (f <= p["f_max"]), amp, 0.0)


def band_psd(p: dict, f: np.ndarray, gain: float = 1.0) -> np.ndarray:
    """Target PSD of the band (the square of band_transfer)."""
    return band_transfer(p, f, gain) ** 2


# ---------------------------------------------------------------------------------------- synthesis
def _workers() -> int:
    env = os.environ.get("LUNAR_WORKERS")
    return int(env) if env else (os.cpu_count() or 1)


FFT_BLOCK = 64          # rows / columns per single-threaded 1-D FFT call (fixed: results do not depend on threads)


def _map(fn, items) -> None:
    """Run fn over items on LUNAR_WORKERS threads (scipy.fft releases the GIL); order-independent writes."""
    nw = max(1, _workers())
    if nw == 1:
        for it in items:
            fn(it)
        return
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(nw) as ex:
        list(ex.map(fn, items))


def synthesize(rng: np.random.Generator, n_fft: int, spacing: float,
               transfer: Callable[[np.ndarray], np.ndarray], crop: tuple[int, int] | None = None,
               row_block: int = 512) -> np.ndarray:
    """Filtered white noise on an n_fft x n_fft periodic square, optionally centre-cropped.

    ``transfer(f)`` is the amplitude ``sqrt(S(f))`` (radial f in cycles/m); the result has PSD S(f).
    crop = (n_rows, n_cols) of the centred window to return. float32 throughout.

    The 2-D real FFT is done as 1-D passes (rfft along x, fft along y, and back) over fixed blocks of
    ``FFT_BLOCK`` rows / columns, each block single-threaded; blocks run on LUNAR_WORKERS threads. The
    result is therefore bitwise identical for any thread count (``rfft2(workers=n)`` was not: its output
    changed by up to 6e-8 m between 1, 4, 8 and 12 workers, which the talus limiter's thresholds can
    amplify).
    """
    n = int(n_fft)
    nc = n // 2 + 1
    w = rng.standard_normal((n, n), dtype=np.float32)
    spec = np.empty((n, nc), dtype=np.complex64)
    rows = [(r, min(n, r + FFT_BLOCK)) for r in range(0, n, FFT_BLOCK)]
    cols = [(c, min(nc, c + FFT_BLOCK)) for c in range(0, nc, FFT_BLOCK)]

    def rfft_rows(a):
        spec[a[0]:a[1]] = sfft.rfft(w[a[0]:a[1]], axis=1, workers=1)

    def fft_cols(a):
        spec[:, a[0]:a[1]] = sfft.fft(spec[:, a[0]:a[1]], axis=0, workers=1)

    def ifft_cols(a):
        spec[:, a[0]:a[1]] = sfft.ifft(spec[:, a[0]:a[1]], axis=0, workers=1)

    _map(rfft_rows, rows)
    del w
    _map(fft_cols, cols)
    fy_all = sfft.fftfreq(n, d=spacing)
    fx = sfft.rfftfreq(n, d=spacing)
    for r0 in range(0, n, row_block):
        fy = fy_all[r0:r0 + row_block]
        t = transfer(np.hypot(fy[:, None], fx[None, :])) / spacing
        spec[r0:r0 + row_block] *= t.astype(np.float32)
    _map(ifft_cols, cols)
    z = np.empty((n, n), dtype=np.float32)

    def irfft_rows(a):
        z[a[0]:a[1]] = sfft.irfft(spec[a[0]:a[1]], n=n, axis=1, workers=1)

    _map(irfft_rows, rows)
    del spec
    if crop is not None:
        ny, nx = crop
        oy, ox = (n - ny) // 2, (n - nx) // 2
        z = np.ascontiguousarray(z[oy:oy + ny, ox:ox + nx])
    return z


def synthesize_band(cfg: Config, band: str) -> Grid:
    """Synthesise the unit-gain band field of a config (no caching)."""
    p = band_params(cfg, band)
    t0 = time.time()
    z = synthesize(cfg_rng(cfg, p["rng_stream"]), p["n_fft"], p["spacing_m"],
                   lambda f: band_transfer(p, f, 1.0), crop=(p["n"], p["n"]))
    meta = {"cache_key": _hash(p), "params": p, "gain": 1.0, "elapsed_s": round(time.time() - t0, 2),
            "rms_m": float(np.sqrt(np.mean(z.astype(np.float64) ** 2)))}
    return Grid(z, -p["half_m"], -p["half_m"], p["spacing_m"], meta)


def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


def field_prefix(cfg: Config, band: str) -> Path:
    return Layout(cfg).stage_dir("terrain") / "fbm" / f"{band}_unit"


def unit_field(cfg: Config, band: str, rebuild: bool = False, mmap: bool = True) -> Grid:
    """Cached unit-gain band field (synthesised and saved on first use or when parameters change)."""
    p = band_params(cfg, band)
    prefix = field_prefix(cfg, band)
    key = _hash(p)
    if not rebuild and prefix.with_suffix(".json").exists() and prefix.with_suffix(".npy").exists():
        try:
            info = json.loads(prefix.with_suffix(".json").read_text())
            if info.get("meta", {}).get("cache_key") == key:
                return Grid.load(prefix, mmap=mmap)
        except (OSError, json.JSONDecodeError):
            pass
    g = synthesize_band(cfg, band)
    _save_atomic(g, prefix)
    del g
    return Grid.load(prefix, mmap=mmap)


def _save_atomic(g: Grid, prefix: Path) -> None:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    tmp = prefix.with_name(prefix.name + "_tmp")
    g.save(tmp)
    os.replace(tmp.with_suffix(".npy"), prefix.with_suffix(".npy"))
    os.replace(tmp.with_suffix(".json"), prefix.with_suffix(".json"))


def fbm_gains(cfg: Config) -> dict[str, float]:
    """PSD gains per band: {'coarse': fbm_gain, 'fine': fbm_gain_fine (default fbm_gain)} (FID-06)."""
    r = cfg.terrain.roughness
    gc = float(r.fbm_gain)
    gf = r.get("fbm_gain_fine")
    gf = gc if gf is None else float(gf)
    for name, g in (("fbm_gain", gc), ("fbm_gain_fine", gf)):
        if g < 0:
            raise ValueError(f"terrain.roughness.{name} must be >= 0")
    return {"coarse": gc, "fine": gf}


def fbm_scale(cfg: Config, band: str = "coarse") -> float:
    """Amplitude factor sqrt(gain) of a band applied to its unit field."""
    if band not in BANDS:
        raise ValueError(f"band must be one of {BANDS}")
    return math.sqrt(fbm_gains(cfg)[band])


def coarse_field(cfg: Config, unit: bool = False) -> Grid:
    """F_coarse on its 10 cm grid (ring square + margin). unit=True: cached unit-gain field (mmap)."""
    g = unit_field(cfg, "coarse")
    if unit:
        return g
    s = np.float32(fbm_scale(cfg, "coarse"))
    return Grid(np.asarray(g.z) * s, g.x0, g.y0, g.spacing, dict(g.meta, gain=fbm_gains(cfg)["coarse"]))


def fine_field(cfg: Config, unit: bool = False) -> Grid:
    """F_fine on the 1 cm grid over core + margin. unit=True: cached unit-gain field (mmap)."""
    g = unit_field(cfg, "fine")
    if unit:
        return g
    s = np.float32(fbm_scale(cfg, "fine"))
    return Grid(np.asarray(g.z) * s, g.x0, g.y0, g.spacing, dict(g.meta, gain=fbm_gains(cfg)["fine"]))


# ---------------------------------------------------------------------------------------- sampling
@nb.njit(cache=False, inline="always")
def _bspline3_at(coef, r, c):
    i = int(math.floor(c))
    j = int(math.floor(r))
    tx = c - i
    ty = r - j
    sx = 1.0 - tx
    sy = 1.0 - ty
    wx0 = sx * sx * sx / 6.0
    wx1 = (3.0 * tx * tx * tx - 6.0 * tx * tx + 4.0) / 6.0
    wx2 = (-3.0 * tx * tx * tx + 3.0 * tx * tx + 3.0 * tx + 1.0) / 6.0
    wx3 = tx * tx * tx / 6.0
    wy0 = sy * sy * sy / 6.0
    wy1 = (3.0 * ty * ty * ty - 6.0 * ty * ty + 4.0) / 6.0
    wy2 = (-3.0 * ty * ty * ty + 3.0 * ty * ty + 3.0 * ty + 1.0) / 6.0
    wy3 = ty * ty * ty / 6.0
    acc = 0.0
    for a in range(4):
        wy = wy0 if a == 0 else (wy1 if a == 1 else (wy2 if a == 2 else wy3))
        row = j - 1 + a
        v = (wx0 * coef[row, i - 1] + wx1 * coef[row, i] + wx2 * coef[row, i + 1] + wx3 * coef[row, i + 2])
        acc += wy * v
    return acc


@nb.njit(cache=False, parallel=True)
def _bspline3_kernel(coef, rows, cols, out):
    for p in nb.prange(rows.size):
        out[p] = _bspline3_at(coef, rows[p], cols[p])


class CubicSampler:
    """Cubic B-spline interpolation of a Grid, numerically identical to ``Grid.sample(order=3)``.

    ``Grid.sample(order=3)`` calls ``scipy.ndimage.map_coordinates(..., order=3, mode='nearest')``,
    which edge-pads the array by 12 nodes, runs the spline prefilter (mode 'nearest') and evaluates
    the cubic B-spline. This class does the same prefilter once (coefficients cached, float64) and
    evaluates with a numba-parallel kernel, so repeated sampling of a large grid does not re-filter.
    Points outside the grid are clamped to the edge (as Grid.sample). Interpolating: node positions
    return the node values (to ~1e-15 relative).
    """

    PAD = 12

    def __init__(self, grid: Grid):
        self.x0, self.y0, self.spacing = grid.x0, grid.y0, grid.spacing
        self.nx, self.ny = grid.nx, grid.ny
        padded = np.pad(np.asarray(grid.z, dtype=np.float64), self.PAD, mode="edge")
        self.coef = ndimage.spline_filter(padded, order=3, output=np.float64, mode="nearest")

    def __call__(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = np.broadcast_shapes(x.shape, y.shape)
        col = np.clip((np.broadcast_to(x, shape).ravel() - self.x0) / self.spacing, 0.0, self.nx - 1) + self.PAD
        row = np.clip((np.broadcast_to(y, shape).ravel() - self.y0) / self.spacing, 0.0, self.ny - 1) + self.PAD
        out = np.empty(col.size, dtype=np.float64)
        if col.size:
            _bspline3_kernel(self.coef, np.ascontiguousarray(row), np.ascontiguousarray(col), out)
        return out.reshape(shape)


# ---------------------------------------------------------------------------------------- analysis
def radial_psd(z: np.ndarray, spacing: float, nbins: int = 60, window: bool = True,
               fmin: float | None = None, fmax: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Radially averaged two-sided 2-D periodogram (same normalisation as S(f)), Hann-windowed.

    Returns (f_centres [cycles/m], psd [m^2/(cycles/m)^2]) over log-spaced bins; empty bins are dropped.
    """
    a = np.asarray(z, dtype=np.float64)
    a = a - a.mean()
    ny, nx = a.shape
    if window:
        w2 = np.outer(np.hanning(ny), np.hanning(nx))
        a = a * w2
        norm = (w2 ** 2).sum()
    else:
        norm = float(nx * ny)
    P = np.abs(np.fft.rfft2(a)) ** 2 * spacing ** 2 / norm
    fy = np.fft.fftfreq(ny, d=spacing)
    fx = np.fft.rfftfreq(nx, d=spacing)
    f = np.hypot(fy[:, None], fx[None, :])
    lo = fmin if fmin is not None else 2.0 / (min(nx, ny) * spacing)
    hi = fmax if fmax is not None else 0.5 / spacing
    edges = np.geomspace(lo, hi, nbins + 1)
    idx = np.digitize(f.ravel(), edges) - 1
    ok = (idx >= 0) & (idx < nbins)
    s = np.bincount(idx[ok], weights=P.ravel()[ok], minlength=nbins)
    c = np.bincount(idx[ok], minlength=nbins)
    fc = np.sqrt(edges[:-1] * edges[1:])
    good = c > 0
    return fc[good], s[good] / c[good]


def loglog_slope(f: np.ndarray, p: np.ndarray, fmin: float, fmax: float) -> float:
    """Least-squares slope of log10(p) vs log10(f) for fmin <= f <= fmax."""
    m = (f >= fmin) & (f <= fmax) & (p > 0)
    return float(np.polyfit(np.log10(f[m]), np.log10(p[m]), 1)[0])
