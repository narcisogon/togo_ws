"""Surface statistics used by the acceptance checks (docs/DESIGN.md section 13).

Everything here is plain numpy/scipy, works with numpy 1.26 (system python, ROS side) and
numpy 2.x (the generator venv), and accepts arrays far larger than memory-friendly sizes by
processing them in row blocks (the 1 cm core grid has 10001 x 10001 nodes).

Roughness definitions (these MATCH the science agent's estimators, so measured values are
directly comparable with the targets in docs/research/lunar_calibration.md section 5.2 and
``config terrain.roughness``):

* **Plane-detrended directional deviogram** ``nu(L)``.
  1. The analysis window (default: the whole grid; ``window_m`` splits the grid into
     non-overlapping square windows, like ``detrended_compare.py`` did with 100 m DTM tiles)
     has its least-squares plane ``a x + b y + c`` removed, fitted over the finite nodes.
  2. For a lag L the node offset is ``k = max(1, round(L / spacing))``; the lag actually
     used is ``k * spacing`` (reported as ``lag_used_m``).
  3. Forward differences of the detrended heights are taken along x
     (``r[j, i+k] - r[j, i]``) and along y (``r[j+k, i] - r[j, i]``) for every pair of finite
     nodes inside the same window.
  4. ``nu(L) = sqrt(mean(d^2))`` over the POOLED x and y differences (each direction weighted
     by its number of pairs), exactly the concatenation used by ``fbm_check.nu()`` and by the
     "directional RMS" of ``detrended_compare.py``. ``nu_x`` / ``nu_y`` are also reported.
  5. RMS slope at baseline L is ``atan(nu(L) / L_used)``.
  Removing the plane before differencing is equivalent to subtracting ``a * k * spacing``
  (resp. ``b * k * spacing``) from every raw difference, which is how it is computed here
  (no detrended copy of the grid is ever materialised).
  Target: ``nu(L) = nu1 * (L / 1 m) ** H`` (``nu1 = 0.069 m``, ``H = 0.8``).

* **Radially averaged PSD.** Two-sided 2-D periodogram normalised so that
  ``sum(S) * df_x * df_y = variance`` (m^4, f in cycles per metre) - the convention of the
  synthesis ``S(f) = S0 f^-(2+2H)`` in ``fbm_check.py`` and DESIGN section 3. Each (Welch)
  segment is plane-detrended and tapered with a separable Hann window (power normalised by
  ``sum(w^2)``), periodograms are averaged, then averaged in log-spaced annuli. A log-log
  fit gives the slope ``beta = -(2 + 2H)`` and ``S0 = S(1 cycle/m)``.

* **Size-frequency distributions.** Cumulative counts ``N(>=D)`` with exact (Garwood)
  Poisson confidence intervals, model curves (single and broken power law, the crater
  population with the DESIGN section 4 fade), and the central Poisson acceptance range used
  for "within sampling tolerance (+-3 sigma)".
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Callable, Iterable, Sequence

import numpy as np

# ----------------------------------------------------------------------------------------------
# small helpers

DEFAULT_LAGS_M = (0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0)
CHECK_LAGS_M = (0.01, 0.1, 1.0, 10.0)


def nu_target(L: float | np.ndarray, nu1_m: float = 0.069, hurst: float = 0.8) -> np.ndarray:
    """Target deviogram ``nu1 * (L / 1 m) ** H``."""
    return nu1_m * np.power(np.asarray(L, dtype=np.float64), hurst)


def rms_slope_deg(nu: float | np.ndarray, L: float | np.ndarray) -> np.ndarray:
    """RMS slope ``atan(nu / L)`` in degrees."""
    return np.degrees(np.arctan(np.asarray(nu, dtype=np.float64) / np.asarray(L, dtype=np.float64)))


def _as_array(z) -> np.ndarray:
    """Accept a numpy array, a memmap or a ``Grid``-like object with ``.z``."""
    return getattr(z, "z", z)


def _spacing_of(z, spacing: float | None) -> float:
    if spacing is not None:
        return float(spacing)
    s = getattr(z, "spacing", None)
    if s is None:
        raise ValueError("spacing is required when z is a plain array")
    return float(s)


# ----------------------------------------------------------------------------------------------
# plane fit (block-wise, NaN aware)

@dataclass
class Plane:
    """z = a * (x - xc) + b * (y - yc) + c, x/y in metres relative to the window's node (0, 0)."""

    a: float
    b: float
    c: float
    xc: float
    yc: float
    n: int

    def slope_deg(self) -> float:
        return float(np.degrees(np.arctan(np.hypot(self.a, self.b))))


def plane_fit(z: np.ndarray, spacing: float, block_rows: int = 1024) -> Plane:
    """Least-squares plane over the finite nodes of ``z`` (any size; row blocks)."""
    z = _as_array(z)
    ny, nx = z.shape
    xc = 0.5 * (nx - 1) * spacing
    yc = 0.5 * (ny - 1) * spacing
    X = np.arange(nx, dtype=np.float64) * spacing - xc
    S = np.zeros(9)  # n, sx, sy, sz, sxx, syy, sxy, sxz, syz
    for j0 in range(0, ny, block_rows):
        j1 = min(ny, j0 + block_rows)
        blk = np.asarray(z[j0:j1], dtype=np.float64)
        Y = (np.arange(j0, j1, dtype=np.float64) * spacing - yc)[:, None]
        m = np.isfinite(blk)
        if m.all():
            nrow = j1 - j0
            zz = blk
            n = nrow * nx
            sx = X.sum() * nrow
            sy = Y.sum() * nx
            sxx = (X * X).sum() * nrow
            syy = (Y * Y).sum() * nx
            sxy = X.sum() * Y.sum()
            sz = zz.sum()
            sxz = (zz * X[None, :]).sum()
            syz = (zz * Y).sum()
        else:
            zz = np.where(m, blk, 0.0)
            mf = m.astype(np.float64)
            n = int(m.sum())
            Xb = np.broadcast_to(X[None, :], blk.shape)
            Yb = np.broadcast_to(Y, blk.shape)
            sx = (mf * Xb).sum()
            sy = (mf * Yb).sum()
            sxx = (mf * Xb * Xb).sum()
            syy = (mf * Yb * Yb).sum()
            sxy = (mf * Xb * Yb).sum()
            sz = zz.sum()
            sxz = (zz * Xb).sum()
            syz = (zz * Yb).sum()
        S += np.array([n, sx, sy, sz, sxx, syy, sxy, sxz, syz], dtype=np.float64)
    n, sx, sy, sz, sxx, syy, sxy, sxz, syz = S
    if n < 3:
        return Plane(0.0, 0.0, float(sz / n) if n else float("nan"), xc, yc, int(n))
    A = np.array([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]])
    rhs = np.array([sxz, syz, sz])
    try:
        a, b, c = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError:
        a, b, c = np.linalg.lstsq(A, rhs, rcond=None)[0]
    return Plane(float(a), float(b), float(c), xc, yc, int(n))


# ----------------------------------------------------------------------------------------------
# deviogram

@dataclass
class DeviogramResult:
    lags_m: list[float]
    lag_used_m: list[float]
    nu_m: list[float]
    nu_x_m: list[float]
    nu_y_m: list[float]
    n_pairs: list[int]
    rms_slope_deg: list[float]
    spacing_m: float
    detrend: str
    window_m: float | None
    n_windows: int
    stride: int
    plane_slope_deg: list[float] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    def nu_at(self, L: float) -> float:
        i = int(np.argmin(np.abs(np.asarray(self.lags_m) - L)))
        return self.nu_m[i]


def _diff_sums_x(z: np.ndarray, k: int, offset: float, rows: np.ndarray, block_rows: int) -> tuple[float, int]:
    """Sum of squared (z[j, i+k] - z[j, i] - offset) over finite pairs, for the given rows."""
    tot, cnt = 0.0, 0
    for r0 in range(0, len(rows), block_rows):
        rr = rows[r0:r0 + block_rows]
        if len(rr) == 0:
            continue
        if rr[-1] - rr[0] + 1 == len(rr):  # contiguous -> cheap slice
            blk = np.asarray(z[rr[0]:rr[-1] + 1], dtype=np.float64)
        else:
            blk = np.asarray(z[rr], dtype=np.float64)
        d = blk[:, k:] - blk[:, :-k]
        if offset:
            d -= offset
        m = np.isfinite(d)
        if m.all():
            tot += float(np.einsum("ij,ij->", d, d))
            cnt += d.size
        else:
            dd = d[m]
            tot += float(dd @ dd)
            cnt += dd.size
    return tot, cnt


def _diff_sums_y(z: np.ndarray, k: int, offset: float, col_stride: int, block_rows: int) -> tuple[float, int]:
    """Sum of squared (z[j+k, i] - z[j, i] - offset) over finite pairs (every col_stride-th column)."""
    ny = z.shape[0]
    tot, cnt = 0.0, 0
    for j0 in range(0, ny - k, block_rows):
        j1 = min(ny - k, j0 + block_rows)
        a = np.asarray(z[j0 + k:j1 + k, ::col_stride], dtype=np.float64)
        b = np.asarray(z[j0:j1, ::col_stride], dtype=np.float64)
        d = a - b
        if offset:
            d -= offset
        m = np.isfinite(d)
        if m.all():
            tot += float(np.einsum("ij,ij->", d, d))
            cnt += d.size
        else:
            dd = d[m]
            tot += float(dd @ dd)
            cnt += dd.size
    return tot, cnt


def _windows(ny: int, nx: int, spacing: float, window_m: float | None) -> list[tuple[int, int, int, int]]:
    """Non-overlapping windows (j0, j1, i0, i1) in node indices (end exclusive)."""
    if window_m is None:
        return [(0, ny, 0, nx)]
    w = int(round(window_m / spacing))
    if w < 2:
        raise ValueError("window_m must span at least 2 nodes")
    wins = []
    for j0 in range(0, ny - w + 1, w):
        for i0 in range(0, nx - w + 1, w):
            wins.append((j0, j0 + w, i0, i0 + w))
    if not wins:
        raise ValueError(f"grid {ny}x{nx} is smaller than one {window_m} m window")
    return wins


def deviogram(z, lags_m: Sequence[float] = DEFAULT_LAGS_M, spacing: float | None = None, *,
              detrend: str = "plane", window_m: float | None = None, stride: int = 1,
              block_rows: int = 512) -> DeviogramResult:
    """Plane-detrended directional deviogram (see module docstring for the exact estimator).

    Args:
        z: 2-D height array (row 0 = south, as ``Grid``), a memmap, or a ``Grid``.
        lags_m: lags L in metres.
        spacing: node spacing in metres (taken from ``z.spacing`` for a Grid).
        detrend: ``"plane"`` (default, the acceptance definition) or ``"none"``.
        window_m: detrend window size in metres (None = whole grid).
        stride: use every ``stride``-th row (x differences) / column (y differences) to save
            time on huge grids; 1 = all pairs.
        block_rows: rows per processing block (memory control).
    """
    spacing = _spacing_of(z, spacing)
    arr = _as_array(z)
    if arr.ndim != 2:
        raise ValueError("z must be 2-D")
    ny, nx = arr.shape
    lags = [float(L) for L in lags_m]
    wins = _windows(ny, nx, spacing, window_m)
    sums_x = np.zeros(len(lags))
    sums_y = np.zeros(len(lags))
    cnt_x = np.zeros(len(lags), dtype=np.int64)
    cnt_y = np.zeros(len(lags), dtype=np.int64)
    plane_slopes = []
    for (j0, j1, i0, i1) in wins:
        sub = arr[j0:j1, i0:i1] if (j0, j1, i0, i1) != (0, ny, 0, nx) else arr
        if detrend == "plane":
            pl = plane_fit(sub, spacing, block_rows=max(block_rows, 256))
            a, b = pl.a, pl.b
            plane_slopes.append(pl.slope_deg())
        elif detrend in ("none", None):
            a = b = 0.0
        else:
            raise ValueError(f"unknown detrend {detrend!r}")
        wy, wx = sub.shape
        rows = np.arange(0, wy, max(1, stride))
        for li, L in enumerate(lags):
            k = max(1, int(round(L / spacing)))
            if k < wx:
                s, c = _diff_sums_x(sub, k, a * k * spacing, rows, block_rows)
                sums_x[li] += s
                cnt_x[li] += c
            if k < wy:
                s, c = _diff_sums_y(sub, k, b * k * spacing, max(1, stride), block_rows)
                sums_y[li] += s
                cnt_y[li] += c
    nu, nux, nuy, used, npairs, slope = [], [], [], [], [], []
    for li, L in enumerate(lags):
        k = max(1, int(round(L / spacing)))
        Lu = k * spacing
        n = int(cnt_x[li] + cnt_y[li])
        v = math.sqrt((sums_x[li] + sums_y[li]) / n) if n else float("nan")
        nu.append(v)
        nux.append(math.sqrt(sums_x[li] / cnt_x[li]) if cnt_x[li] else float("nan"))
        nuy.append(math.sqrt(sums_y[li] / cnt_y[li]) if cnt_y[li] else float("nan"))
        used.append(Lu)
        npairs.append(n)
        slope.append(float(rms_slope_deg(v, Lu)) if n else float("nan"))
    return DeviogramResult(lags, used, nu, nux, nuy, npairs, slope, spacing,
                           str(detrend), window_m, len(wins), int(stride),
                           [float(s) for s in plane_slopes[:16]])


def compare_deviogram(result: DeviogramResult, nu1_m: float, hurst: float, tolerance_frac: float,
                      check_lags: Iterable[float] = CHECK_LAGS_M) -> list[dict]:
    """Rows comparing a deviogram with the target law; ``pass`` only set for ``check_lags``."""
    check = [float(c) for c in check_lags]
    rows = []
    for L, Lu, v, vx, vy, n, sl in zip(result.lags_m, result.lag_used_m, result.nu_m, result.nu_x_m,
                                       result.nu_y_m, result.n_pairs, result.rms_slope_deg):
        tgt = float(nu_target(Lu, nu1_m, hurst))
        ratio = v / tgt if tgt > 0 and np.isfinite(v) else float("nan")
        checked = any(abs(L - c) <= 1e-9 * max(1.0, c) for c in check)
        row = {"L_m": L, "L_used_m": Lu, "nu_m": v, "nu_x_m": vx, "nu_y_m": vy, "n_pairs": n,
               "target_nu_m": tgt, "ratio": ratio, "rms_slope_deg": sl,
               "target_rms_slope_deg": float(rms_slope_deg(tgt, Lu)), "checked": checked}
        if checked:
            row["pass"] = bool(np.isfinite(ratio) and abs(ratio - 1.0) <= tolerance_frac)
        rows.append(row)
    return rows


def fit_hurst(lags: Sequence[float], nu: Sequence[float]) -> tuple[float, float]:
    """Least-squares log-log fit nu = nu1 * L^H; returns (H, nu1)."""
    L = np.asarray(lags, dtype=np.float64)
    v = np.asarray(nu, dtype=np.float64)
    ok = np.isfinite(v) & (v > 0) & (L > 0)
    if ok.sum() < 2:
        return float("nan"), float("nan")
    H, c = np.polyfit(np.log(L[ok]), np.log(v[ok]), 1)
    return float(H), float(np.exp(c))


# ----------------------------------------------------------------------------------------------
# adirectional slope distribution (as in the science report tables)

def slope_distribution(z, baseline_m: float, spacing: float | None = None, *, detrend: str = "plane",
                       block_rows: int = 512, bin_deg: float = 0.02) -> dict:
    """Gradient-magnitude slope statistics over baseline L (forward differences in x and y).

    ``g = atan(hypot(dz_x, dz_y) / L)`` per node, as in ``detrended_compare.grad_stats``.
    Percentiles come from a histogram with ``bin_deg`` bins (exact to half a bin).
    """
    spacing = _spacing_of(z, spacing)
    arr = _as_array(z)
    ny, nx = arr.shape
    k = max(1, int(round(baseline_m / spacing)))
    if k >= min(nx, ny):
        raise ValueError("baseline larger than the grid")
    Lu = k * spacing
    a = b = 0.0
    if detrend == "plane":
        pl = plane_fit(arr, spacing)
        a, b = pl.a, pl.b
    edges = np.arange(0.0, 90.0 + bin_deg, bin_deg)
    hist = np.zeros(len(edges) - 1, dtype=np.int64)
    s1 = 0.0
    n = 0
    for j0 in range(0, ny - k, block_rows):
        j1 = min(ny - k, j0 + block_rows)
        blk = np.asarray(arr[j0:j1 + k], dtype=np.float64)
        dx = (blk[:j1 - j0, k:] - blk[:j1 - j0, :-k]) / Lu - a
        dy = (blk[k:, :nx - k] - blk[:j1 - j0, :nx - k]) / Lu - b
        g = np.degrees(np.arctan(np.hypot(dx, dy)))
        g = g[np.isfinite(g)]
        hist += np.histogram(g, bins=edges)[0]
        s1 += float(g.sum())
        n += g.size
    if n == 0:
        return {"baseline_m": Lu, "n": 0}
    cdf = np.cumsum(hist) / n
    centers = 0.5 * (edges[:-1] + edges[1:])

    def pct(p):
        return float(centers[min(np.searchsorted(cdf, p), len(centers) - 1)])

    return {"baseline_m": Lu, "n": n, "mean_deg": s1 / n, "median_deg": pct(0.5), "p90_deg": pct(0.9),
            "p99_deg": pct(0.99),
            "frac_gt_10deg": float(hist[edges[:-1] >= 10.0].sum() / n),
            "frac_gt_15deg": float(hist[edges[:-1] >= 15.0].sum() / n),
            "frac_gt_20deg": float(hist[edges[:-1] >= 20.0].sum() / n),
            "frac_gt_25deg": float(hist[edges[:-1] >= 25.0].sum() / n)}


# ----------------------------------------------------------------------------------------------
# radially averaged PSD

def _hann(n: int) -> np.ndarray:
    if n < 2:
        return np.ones(n)
    return 0.5 - 0.5 * np.cos(2.0 * np.pi * np.arange(n) / (n - 1))


def radial_psd(z, spacing: float | None = None, *, segment: int | None = None, overlap: float = 0.5,
               detrend: str = "plane", window: str = "hann", n_bins: int = 40, max_segments: int = 64,
               f_min: float | None = None) -> dict:
    """Radially averaged two-sided 2-D PSD (m^4, f in cycles/m); see module docstring.

    Args:
        segment: Welch segment size in nodes (square). None = the largest square that fits,
            capped at 4096 nodes; bigger grids are covered with ``overlap``-overlapping segments
            (at most ``max_segments``, evenly spread).
    Returns a dict with ``f`` (bin centres, geometric mean of the member frequencies), ``S``
    (mean PSD in the bin), ``n_modes`` (frequency samples per bin) and segment info.
    """
    spacing = _spacing_of(z, spacing)
    arr = _as_array(z)
    ny, nx = arr.shape
    if segment is None:
        segment = min(ny, nx, 4096)
    segment = int(min(segment, ny, nx))
    if segment < 8:
        raise ValueError("grid too small for a PSD")
    step = max(1, int(round(segment * (1.0 - overlap))))
    starts_j = list(range(0, ny - segment + 1, step)) or [0]
    starts_i = list(range(0, nx - segment + 1, step)) or [0]
    starts = [(j, i) for j in starts_j for i in starts_i]
    if len(starts) > max_segments:
        idx = np.linspace(0, len(starts) - 1, max_segments).round().astype(int)
        starts = [starts[k] for k in sorted(set(idx.tolist()))]
    w1 = _hann(segment) if window == "hann" else np.ones(segment)
    w2 = np.outer(w1, w1)
    wsum = float((w2 * w2).sum())
    fy = np.fft.fftfreq(segment, d=spacing)
    fx = np.fft.rfftfreq(segment, d=spacing)
    F = np.hypot(fy[:, None], fx[None, :])
    # rfft2 holds half the plane: interior columns stand for two (+f and -f) modes
    weight = np.full(F.shape, 2.0)
    weight[:, 0] = 1.0
    if segment % 2 == 0:
        weight[:, -1] = 1.0
    acc = np.zeros(F.shape)
    nseg = 0
    for (j, i) in starts:
        sub = np.asarray(arr[j:j + segment, i:i + segment], dtype=np.float64)
        if not np.all(np.isfinite(sub)):
            continue
        if detrend == "plane":
            pl = plane_fit(sub, spacing)
            X = np.arange(segment) * spacing - pl.xc
            Y = np.arange(segment) * spacing - pl.yc
            sub = sub - (pl.a * X[None, :] + pl.b * Y[:, None] + pl.c)
        else:
            sub = sub - sub.mean()
        Z = np.fft.rfft2(sub * w2)
        acc += (Z.real ** 2 + Z.imag ** 2)
        nseg += 1
    if nseg == 0:
        raise ValueError("no finite PSD segment")
    P = acc / nseg * spacing * spacing / wsum
    f_lo = f_min if f_min is not None else 1.0 / (segment * spacing)
    f_hi = 0.5 / spacing
    edges = np.geomspace(f_lo, f_hi, n_bins + 1)
    Ff = F.ravel()
    Pf = P.ravel()
    Wf = weight.ravel()
    ok = (Ff >= edges[0]) & (Ff <= edges[-1])
    bins = np.clip(np.searchsorted(edges, Ff[ok], side="right") - 1, 0, n_bins - 1)
    wsum_b = np.bincount(bins, weights=Wf[ok], minlength=n_bins)
    s_b = np.bincount(bins, weights=Wf[ok] * Pf[ok], minlength=n_bins)
    lf_b = np.bincount(bins, weights=Wf[ok] * np.log(Ff[ok]), minlength=n_bins)
    good = wsum_b > 0
    f_c = np.exp(lf_b[good] / wsum_b[good])
    S_c = s_b[good] / wsum_b[good]
    df = 1.0 / (segment * spacing)
    return {"f": f_c.tolist(), "S": S_c.tolist(), "n_modes": wsum_b[good].astype(int).tolist(),
            "segment_nodes": segment, "segment_m": segment * spacing, "n_segments": nseg,
            "spacing_m": spacing, "detrend": detrend, "window": window,
            "variance_m2": float((P * weight).sum() * df * df)}


def fit_power_law(f: Sequence[float], S: Sequence[float], f_min: float | None = None,
                  f_max: float | None = None) -> dict:
    """Fit log10 S = log10 S0 + beta log10 f over [f_min, f_max]; H = (-beta - 2) / 2."""
    f = np.asarray(f, dtype=np.float64)
    S = np.asarray(S, dtype=np.float64)
    ok = np.isfinite(S) & (S > 0) & (f > 0)
    if f_min is not None:
        ok &= f >= f_min
    if f_max is not None:
        ok &= f <= f_max
    if ok.sum() < 3:
        return {"n": int(ok.sum()), "beta": float("nan"), "S0": float("nan"), "H": float("nan")}
    lx, ly = np.log10(f[ok]), np.log10(S[ok])
    beta, c = np.polyfit(lx, ly, 1)
    pred = beta * lx + c
    ss_res = float(((ly - pred) ** 2).sum())
    ss_tot = float(((ly - ly.mean()) ** 2).sum())
    return {"n": int(ok.sum()), "beta": float(beta), "S0": float(10 ** c), "H": float((-beta - 2.0) / 2.0),
            "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
            "f_min": float(f[ok].min()), "f_max": float(f[ok].max())}


# ----------------------------------------------------------------------------------------------
# size-frequency distributions

def cumulative_counts(sizes: np.ndarray, D_values: Sequence[float]) -> np.ndarray:
    """N(>=D) for each D (counts, not densities)."""
    s = np.sort(np.asarray(sizes, dtype=np.float64).ravel())
    D = np.asarray(D_values, dtype=np.float64)
    return (len(s) - np.searchsorted(s, D, side="left")).astype(np.int64)


def _sigma_to_alpha(sigma: float) -> float:
    """Two-sided tail probability of a +-sigma Gaussian interval (sigma=1 -> 0.3173)."""
    return math.erfc(sigma / math.sqrt(2.0))


def poisson_ci(n: int | np.ndarray, sigma: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    """Exact (Garwood) two-sided confidence interval of a Poisson mean given n observed events.

    The coverage equals a +-sigma Gaussian interval (sigma=1: 68.27 %, 3: 99.73 %).
    """
    from scipy.stats import chi2
    n = np.asarray(n, dtype=np.float64)
    alpha = _sigma_to_alpha(sigma)
    lo = np.where(n > 0, 0.5 * chi2.ppf(alpha / 2.0, 2.0 * np.maximum(n, 1e-300)), 0.0)
    hi = 0.5 * chi2.ppf(1.0 - alpha / 2.0, 2.0 * n + 2.0)
    return lo, hi


def poisson_range(lam: float | np.ndarray, sigma: float = 3.0) -> tuple[np.ndarray, np.ndarray]:
    """Central acceptance range [lo, hi] (integers) of Poisson(lam) at +-sigma coverage.

    An observed count n is "within sampling tolerance" when lo <= n <= hi.
    """
    from scipy.stats import poisson
    lam = np.asarray(lam, dtype=np.float64)
    alpha = _sigma_to_alpha(sigma)
    lo = poisson.ppf(alpha / 2.0, lam)
    hi = poisson.isf(alpha / 2.0, lam)
    return np.nan_to_num(lo).astype(np.int64), np.nan_to_num(hi).astype(np.int64)


def power_law_N(D: float | np.ndarray, C: float, b: float) -> np.ndarray:
    """N(>=D) = C D^-b (per m^2)."""
    return C * np.power(np.asarray(D, dtype=np.float64), -b)


def broken_power_law_N(D: float | np.ndarray, A_at_anchor: float, segments: Sequence[Sequence[float]],
                       anchor_D: float = 1.0) -> np.ndarray:
    """Continuous broken power law N(>=D) per m^2 (config ``rocks.sfd``).

    ``segments`` = [[D_lo, D_hi, b], ...] in increasing D; ``A_at_anchor`` = N(>=anchor_D).
    Outside the segments the first/last exponent is extended.
    """
    segs = sorted([(float(a), float(b), float(c)) for a, b, c in segments])
    breaks = [segs[0][0]] + [s[1] for s in segs]

    def seg_b(d):
        for lo, hi, b in segs:
            if d <= hi:
                return b
        return segs[-1][2]

    # N at each break, propagated from the anchor
    def N_scalar(d: float) -> float:
        if d <= 0:
            return float("inf")
        # walk from anchor to d through the break points
        x, N = anchor_D, A_at_anchor
        if d >= x:
            pts = [p for p in breaks if x < p < d] + [d]
            for p in pts:
                b = seg_b(0.5 * (x + p) if p != x else p)
                N *= (p / x) ** (-b)
                x = p
        else:
            pts = [p for p in sorted(breaks, reverse=True) if d < p < x] + [d]
            for p in pts:
                b = seg_b(0.5 * (x + p))
                N *= (p / x) ** (-b)
                x = p
        return N

    D_arr = np.asarray(D, dtype=np.float64)
    out = np.vectorize(N_scalar, otypes=[np.float64])(D_arr)
    return out if out.ndim else out[()]


def crater_expected_N(D: float | np.ndarray, C: float, b: float, D_min: float, D_max: float,
                      fade: Sequence[float] | None = (8.0, 16.0)) -> np.ndarray:
    """Expected N(>=D) per m^2 of the DESIGN section 4 crater population.

    Sizes follow N(>=D) = C D^-b truncated to [D_min, D_max] (inverse transform), and craters
    with D in [fade0, fade1] are kept with probability falling linearly from 1 to 0.
    """
    from scipy.integrate import quad

    def keep(x):
        if fade is None:
            return 1.0
        f0, f1 = fade
        if x <= f0:
            return 1.0
        if x >= f1:
            return 0.0
        return (f1 - x) / (f1 - f0)

    def dens(x):  # differential density -dN/dD
        return C * b * x ** (-b - 1.0) * keep(x)

    def N_scalar(d):
        lo = max(d, D_min)
        if lo >= D_max:
            return 0.0
        pts = [p for p in (fade or ()) if lo < p < D_max]
        val, _ = quad(dens, lo, D_max, points=pts or None, limit=200)
        return val

    D_arr = np.asarray(D, dtype=np.float64)
    out = np.vectorize(N_scalar, otypes=[np.float64])(D_arr)
    return out if out.ndim else out[()]


def sfd_rows(sizes: np.ndarray, area_m2: float, D_values: Sequence[float],
             expected_per_m2: Callable[[np.ndarray], np.ndarray] | Sequence[float], *,
             sigma: float = 3.0, frac_tol: float | None = None) -> list[dict]:
    """Compare observed N(>=D) with a model.

    Pass rule: ``frac_tol`` given -> |n / lambda - 1| <= frac_tol (crater rule, +-20 %);
    otherwise the observed n must lie in the central Poisson(lambda) range at +-sigma
    (rock rule, "within sampling tolerance (+-3 sigma)").
    """
    D = np.asarray(D_values, dtype=np.float64)
    n = cumulative_counts(sizes, D)
    lam = (np.asarray(expected_per_m2(D)) if callable(expected_per_m2)
           else np.asarray(expected_per_m2, dtype=np.float64)) * area_m2
    lo, hi = poisson_range(lam, sigma)
    ci_lo, ci_hi = poisson_ci(n, 1.0)
    rows = []
    for k in range(len(D)):
        lam_k = float(lam[k])
        ratio = float(n[k] / lam_k) if lam_k > 0 else float("nan")
        z = (n[k] - lam_k) / math.sqrt(lam_k) if lam_k > 0 else float("nan")
        row = {"D_m": float(D[k]), "observed": int(n[k]), "expected": lam_k, "ratio": ratio, "z_poisson": z,
               "poisson_range": [int(lo[k]), int(hi[k])], "ci68_observed": [float(ci_lo[k]), float(ci_hi[k])],
               "density_per_m2": float(n[k] / area_m2), "expected_density_per_m2": lam_k / area_m2}
        if frac_tol is not None:
            row["rule"] = f"|ratio-1| <= {frac_tol}"
            row["pass"] = bool(np.isfinite(ratio) and abs(ratio - 1.0) <= frac_tol)
        else:
            row["rule"] = f"poisson +-{sigma} sigma"
            row["pass"] = bool(lo[k] <= n[k] <= hi[k])
        rows.append(row)
    return rows


def fit_cumulative_slope(sizes: np.ndarray, D_min: float, D_max: float | None = None) -> dict:
    """Maximum-likelihood exponent b of N(>=D) ~ D^-b for sizes in [D_min, D_max) (truncated Pareto)."""
    s = np.asarray(sizes, dtype=np.float64)
    s = s[(s >= D_min) & ((s < D_max) if D_max else True)]
    n = len(s)
    if n < 5:
        return {"n": n, "b": float("nan")}
    lsum = float(np.log(s / D_min).sum())
    b_hill = n / lsum if lsum > 0 else float("nan")
    if not D_max:
        return {"n": n, "b": b_hill, "b_err": b_hill / math.sqrt(n)}
    r = D_max / D_min
    from scipy.optimize import brentq

    def score(b):  # d/db log-likelihood of the truncated Pareto
        rb = r ** (-b)
        return n / b - lsum - n * rb * math.log(r) / (1.0 - rb)

    try:
        b = brentq(score, 1e-3, 20.0)
    except ValueError:
        b = b_hill
    return {"n": n, "b": float(b), "b_err": float(b / math.sqrt(n)), "b_hill_untruncated": b_hill}
