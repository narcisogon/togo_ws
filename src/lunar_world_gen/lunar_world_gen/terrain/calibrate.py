"""Roughness calibration of the composed core H0 (docs/DESIGN.md section 3 "Calibration" and the
site-aware acceptance of section 13).

Estimator: plane-detrended directional deviogram nu(L)
    Input: H0 (no rocks, no fillets) on the core nodes, |x|, |y| <= core half (100 m square), 1 cm.
    1. Fit the least-squares plane a + b*x + c*y to all nodes (equal weights) and subtract it.
    2. For a lag L = k * dx (k integer), over all node pairs inside the square:
           Dx(L) = mean_{i,j} (r[j, i+k] - r[j, i])^2        (pairs along x)
           Dy(L) = mean_{i,j} (r[j+k, i] - r[j, i])^2        (pairs along y)
    3. nu_x = sqrt(Dx), nu_y = sqrt(Dy), nu(L) = sqrt((Dx + Dy) / 2); RMS slope(L) = atan(nu / L).
    All sums are accumulated in float64. Because the residual of a plane is linear, the lag differences
    of the detrended field are the raw differences minus the plane's constant step (b*L or c*L), so no
    detrended copy is needed.

Exact two-gain fit (FID-06)
    H0 = B + K + T + s_c F_coarse_unit + s_f w_core F_fine_unit with K = K_big + w_core K_small, T the
    talus term, s_c = sqrt(fbm_gain), s_f = sqrt(fbm_gain_fine) (inside the core w_ring = 1). Detrending
    and differencing are linear, so for any linear combination sum_c a_c comp_c of the five component
    fields (B, K, T, Fc, Ff)
        D(L) = sum_{c,d} a_c a_d S_cd(L)
    where S_cd are the cross sums of the detrended lag differences. One pass over the fields gives S_cd
    for all lags; nu(L; g_c, g_f) is then exact for every pair of gains (T is computed from the coarse
    gain of the config, so a changed coarse gain needs a second round, see ``run``).

    The gains are chosen on a grid (coarse 0.005 over [0, G_MAX_COARSE] x [0, G_MAX], refined to 0.001)
    against the site-aware criteria of DESIGN 13 (the same rules as validate.static_checks):
        * composite H0 within +-tolerance_frac of target(L) = nu1 (L / 1 m)^H at 0.01, 0.03, 0.1, 0.3 m;
        * the synthetic part H0 - B (craters, talus, fBm) within +-tolerance_frac at 1 m and 3 m;
        * the composite at most 1.15 x B alone at L >= 10 m;
    with the lexicographic objective 1) maximise the number of criteria met with a margin of 0.02 to
    spare (ratios within 1 +- 0.23; composite <= 1.13 x B), 2) minimise the sum of the squared log
    ratios over all ratio criteria (this centres every lag; a min-max objective left the cm scale at the
    lower edge to gain 0.006 at 0.3 m). The single-gain fit ``fit_gain`` (phase-2 rule, composite at
    every lag) is still available (``--single-gain``) and reported for reference.

    After rebuilding with the new gains the deviogram is measured again on the core assembled from the
    cached H0 tiles (verification).

CLI
    python -m lunar_world_gen.terrain.calibrate [--apply] [--no-rebuild] [--single-gain] [--preset ...]
                                                 [--set key=value ...] [--figure PNG]
    writes build_dir/terrain/calibration.json and the figure (default docs/impl/img/terrain_deviogram.png);
    with --apply both gains are written into the config file (only the values; comments kept; a missing
    fbm_gain_fine line is inserted after fbm_gain) and the H0 tiles and zone grids are rebuilt and
    verified (up to ``rounds`` fit/apply rounds until the gains are stable, because T follows the coarse
    gain).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

import numba as nb
import numpy as np

from ..config import DEFAULT_CONFIG, PACKAGE_ROOT, Config, load_config
from ..layout import Layout
from . import compose as tcompose

LAGS_M = (0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0)
COMPONENTS = ("B", "K", "T", "Fc", "Ff")
G_MAX = 4.0
G_MAX_COARSE = 2.0
G_STEP = 1e-3
#: site-aware acceptance (DESIGN 13; same constants as validate.static_checks.check_roughness)
COMPOSITE_LAGS_M = (0.01, 0.03, 0.1, 0.3)
SYNTHETIC_LAGS_M = (1.0, 3.0)
DTM_MIN_LAG_M = 10.0
DTM_RATIO_MAX = 1.15
FIGURE = PACKAGE_ROOT / "docs" / "impl" / "img" / "terrain_deviogram.png"
ESTIMATOR = "plane-detrended directional deviogram v2 (components B, K, T, Fc, Ff)"


# ---------------------------------------------------------------------------------------- kernels
@nb.njit(cache=False, parallel=True)
def _plane_moments_rows(comps):
    """Per component and row: sum z, sum i*z, j*sum z (i = column, j = row index)."""
    nc, ny, nx = comps.shape
    out = np.zeros((nc, ny, 3))
    for j in nb.prange(ny):
        for c in range(nc):
            s0 = 0.0
            si = 0.0
            for i in range(nx):
                v = np.float64(comps[c, j, i])
                s0 += v
                si += i * v
            out[c, j, 0] = s0
            out[c, j, 1] = si
            out[c, j, 2] = j * s0
    return out


@nb.njit(cache=False, parallel=True)
def _lag_cross_sums_rows(comps, k, axis, step):
    """Per-row cross sums S[c, d] of detrended lag differences (lag k nodes along axis 0=x, 1=y).

    step[c] is the plane's change over k nodes along that axis (subtracted from every difference).
    The caller sums the rows (numpy pairwise summation, deterministic).
    """
    nc, ny, nx = comps.shape
    nrows = ny if axis == 0 else ny - k
    part = np.zeros((nrows, nc, nc))
    for j in nb.prange(nrows):
        d = np.empty(nc)
        acc = np.zeros((nc, nc))
        if axis == 0:
            for i in range(nx - k):
                for c in range(nc):
                    d[c] = np.float64(comps[c, j, i + k]) - np.float64(comps[c, j, i]) - step[c]
                for c in range(nc):
                    for e in range(c, nc):
                        acc[c, e] += d[c] * d[e]
        else:
            for i in range(nx):
                for c in range(nc):
                    d[c] = np.float64(comps[c, j + k, i]) - np.float64(comps[c, j, i]) - step[c]
                for c in range(nc):
                    for e in range(c, nc):
                        acc[c, e] += d[c] * d[e]
        for c in range(nc):
            for e in range(c, nc):
                part[j, c, e] = acc[c, e]
                part[j, e, c] = acc[c, e]
    return part


# ---------------------------------------------------------------------------------------- estimator
def plane_fit(comps: np.ndarray, spacing: float) -> np.ndarray:
    """Least-squares plane per component: array (nc, 3) of (a, b, c) with z ~ a + b*x + c*y (index units)."""
    nc, ny, nx = comps.shape
    m = _plane_moments_rows(comps).sum(axis=1)
    i = np.arange(nx, dtype=np.float64)
    j = np.arange(ny, dtype=np.float64)
    N = float(nx * ny)
    Si, Sj = ny * i.sum(), nx * j.sum()
    A = np.array([[N, Si, Sj], [Si, ny * (i * i).sum(), i.sum() * j.sum()], [Sj, i.sum() * j.sum(), nx * (j * j).sum()]])
    return np.array([np.linalg.solve(A, m[c]) for c in range(nc)])


def cross_stats(comps: np.ndarray, spacing: float, lags_m=LAGS_M) -> dict:
    """Cross statistics of the plane-detrended lag differences of a stack of fields (nc, ny, nx).

    Returns {"lags_m", "k", "S": (nL, 2, nc, nc) mean products [m^2] (axis 0 = x, 1 = y), "npairs",
    "plane": (nc, 3)}.
    """
    comps = np.ascontiguousarray(comps)
    if comps.ndim == 2:
        comps = comps[None]
    nc, ny, nx = comps.shape
    plane = plane_fit(comps, spacing)
    ks = [int(round(L / spacing)) for L in lags_m]
    S = np.zeros((len(ks), 2, nc, nc))
    npairs = np.zeros((len(ks), 2), dtype=np.int64)
    for a, k in enumerate(ks):
        if k < 1 or k >= min(nx, ny):
            raise ValueError(f"lag {lags_m[a]} m is not representable on this grid")
        for axis in (0, 1):
            step = plane[:, 1 + axis] * k
            n = (nx - k) * ny if axis == 0 else nx * (ny - k)
            S[a, axis] = _lag_cross_sums_rows(comps, k, axis, step).sum(axis=0) / n
            npairs[a, axis] = n
    return {"lags_m": list(lags_m), "k": ks, "S": S, "npairs": npairs, "plane": plane, "spacing": spacing,
            "shape": [ny, nx]}


def nu_of(stats: dict, coef) -> dict:
    """nu, nu_x, nu_y [m] of the field sum_c coef[c] * comp_c."""
    a = np.asarray(coef, dtype=np.float64)
    D = np.einsum("c,lkcd,d->lk", a, stats["S"], a)
    D = np.maximum(D, 0.0)
    return {"nu": np.sqrt(D.mean(axis=1)), "nu_x": np.sqrt(D[:, 0]), "nu_y": np.sqrt(D[:, 1])}


def deviogram(z: np.ndarray, spacing: float, lags_m=LAGS_M) -> dict:
    """Plane-detrended directional deviogram of a single field (see module docstring)."""
    st = cross_stats(np.asarray(z, dtype=np.float32)[None], spacing, lags_m)
    r = nu_of(st, [1.0])
    L = np.asarray(lags_m)
    return {"lags_m": list(lags_m), "nu_m": r["nu"].tolist(), "nu_x_m": r["nu_x"].tolist(),
            "nu_y_m": r["nu_y"].tolist(), "rms_slope_deg": np.degrees(np.arctan(r["nu"] / L)).tolist()}


def target_nu(cfg: Config, lags_m=LAGS_M) -> np.ndarray:
    r = cfg.terrain.roughness
    return float(r.nu1_m) * np.asarray(lags_m, dtype=np.float64) ** float(r.hurst)


def _n_comp(stats: dict) -> int:
    return int(np.asarray(stats["S"]).shape[-1])


def fbm_coef(g: float, n: int = 4, g_fine: float | None = None, base: float = 1.0) -> list[float]:
    """Coefficients of the composite: ``base`` for the n - 2 non-fBm components (B, K[, T]), then
    sqrt(g) for F_coarse and sqrt(g_fine) (default g) for F_fine."""
    s = math.sqrt(max(g, 0.0))
    sf = s if g_fine is None else math.sqrt(max(g_fine, 0.0))
    return [base] * (n - 2) + [s, sf]


def synthetic_coef(gc: float, gf: float, n: int) -> list[float]:
    """Coefficients of the synthetic part H0 - B (B is component 0)."""
    c = fbm_coef(gc, n, gf)
    c[0] = 0.0
    return c


def fit_gain(stats: dict, target: np.ndarray, tol: float, g_max: float = G_MAX, step: float = G_STEP) -> dict:
    """Single shared gain (both bands), grid search against the composite target at every lag.

    Objective (lexicographic): 1) maximise the number of lags with |nu/target - 1| <= tol;
    2) minimise the largest |ln(nu/target)| over those in-tolerance lags (centres them in the band,
    so a re-measurement cannot tip a marginal lag out); 3) minimise sum ln(nu/target)^2 over them.
    Lags no gain can bring into tolerance (e.g. scales dominated by the real DTM) do not pull the fit.
    The plain all-lag least-squares optimum is reported as well.
    """
    n = _n_comp(stats)
    gs = np.round(np.arange(0.0, g_max + step / 2, step), 6)
    best = None
    best_ls = None
    for g in gs:
        nu = nu_of(stats, fbm_coef(g, n))["nu"]
        lr = np.log(np.maximum(nu, 1e-300) / target)
        within = np.abs(nu / target - 1.0) <= tol + 1e-12
        worst = float(np.max(np.abs(lr[within]))) if within.any() else float("inf")
        cand = (-int(within.sum()), round(worst, 12), float(np.sum(lr[within] ** 2)), float(g))
        if best is None or cand[:3] < best[:3]:
            best = cand
        ls = float(np.sum(lr ** 2))
        if best_ls is None or ls < best_ls[0]:
            best_ls = (ls, float(g))
    n_in, worst, ssq, g = -best[0], best[1], best[2], best[3]
    n_out = len(target) - n_in
    # per-lag gain that would hit the target exactly: Syy s^2 + 2 Sxy s + Sxx - T^2 = 0, s = sqrt(g)
    S = stats["S"].mean(axis=1)                     # average of the x and y directions
    X = np.array([1.0] * (n - 2) + [0.0, 0.0])
    Y = np.array([0.0] * (n - 2) + [1.0, 1.0])
    req = []
    for a, T in enumerate(target):
        sxx, sxy, syy = X @ S[a] @ X, X @ S[a] @ Y, Y @ S[a] @ Y
        disc = sxy * sxy - syy * (sxx - T * T)
        s = (-sxy + math.sqrt(disc)) / syy if disc >= 0 and syy > 0 else float("nan")
        req.append(s * s if s == s and s >= 0 else None)
    return {"gain": g, "n_outside_tolerance": n_out, "max_abs_log_ratio_in_tolerance": worst,
            "sum_sq_log_ratio_in_tolerance": ssq, "tolerance_frac": tol,
            "objective": "max #lags within tolerance, then min max |ln(nu/target)| over those lags, "
                         "then min sum ln(nu/target)^2 over them",
            "least_squares_all_lags_gain": best_ls[1], "search": {"g_min": 0.0, "g_max": g_max, "step": step},
            "gain_hitting_target_per_lag": req}


def _quad(S: np.ndarray, a0: np.ndarray, n: int) -> tuple[np.ndarray, ...]:
    """Quadratic form of nu^2 in (s_c, s_f) for coefficients a0 + s_c e_Fc + s_f e_Ff: per lag
    (Q00, Q0c, Q0f, Qcc, Qcf, Qff) with nu^2 = Q00 + 2 s_c Q0c + 2 s_f Q0f + s_c^2 Qcc + 2 s_c s_f Qcf
    + s_f^2 Qff (S averaged over the two directions)."""
    Sm = S.mean(axis=1)
    ec = np.zeros(n)
    ec[n - 2] = 1.0
    ef = np.zeros(n)
    ef[n - 1] = 1.0
    q = lambda u, v: np.einsum("c,lcd,d->l", u, Sm, v)  # noqa: E731
    return q(a0, a0), q(a0, ec), q(a0, ef), q(ec, ec), q(ec, ef), q(ef, ef)


def _site_aware_eval(stats: dict, target: np.ndarray, tol: float, gc: np.ndarray, gf: np.ndarray,
                     margin: float = 0.0) -> dict:
    """Vectorised criteria for gain arrays gc (column) x gf (row); a criterion counts as met when it
    holds with ``margin`` to spare (ratio within 1 +- (tol - margin), composite <= (1.15 - margin) B)."""
    n = _n_comp(stats)
    L = np.asarray(stats["lags_m"], dtype=np.float64)
    sc = np.sqrt(np.maximum(gc, 0.0))[:, None]
    sf = np.sqrt(np.maximum(gf, 0.0))[None, :]
    a_comp = np.array(fbm_coef(0.0, n))                     # B, K, T = 1; fBm terms via s
    a_syn = a_comp.copy()
    a_syn[0] = 0.0
    qc = _quad(stats["S"], a_comp, n)
    qs = _quad(stats["S"], a_syn, n)
    nuB = np.sqrt(np.maximum(nu_of(stats, [1.0] + [0.0] * (n - 1))["nu"] ** 2, 0.0))
    shape = (sc.shape[0], sf.shape[1])
    n_met = np.zeros(shape, np.int32)
    worst = np.zeros(shape)
    ssq = np.zeros(shape)
    n_crit = 0
    for i, Lv in enumerate(L):
        comp_lag = any(abs(Lv - c) < 1e-9 for c in COMPOSITE_LAGS_M)
        syn_lag = any(abs(Lv - c) < 1e-9 for c in SYNTHETIC_LAGS_M)
        dtm_lag = Lv >= DTM_MIN_LAG_M - 1e-9
        if not (comp_lag or syn_lag or dtm_lag):
            continue
        q = qs if syn_lag else qc
        nu2 = q[0][i] + 2 * sc * q[1][i] + 2 * sf * q[2][i] + sc * sc * q[3][i] + 2 * sc * sf * q[4][i] + sf * sf * q[5][i]
        nu = np.sqrt(np.maximum(nu2, 0.0))
        n_crit += 1
        if dtm_lag and not (comp_lag or syn_lag):
            n_met += (nu <= (DTM_RATIO_MAX - margin) * nuB[i]).astype(np.int32)
            continue
        lr = np.log(np.maximum(nu, 1e-300) / target[i])
        ok = np.abs(nu / target[i] - 1.0) <= tol - margin + 1e-12
        n_met += ok.astype(np.int32)
        worst = np.maximum(worst, np.abs(lr))
        ssq += lr * lr
    return {"n_met": n_met, "worst": worst, "ssq": ssq, "n_criteria": n_crit}


def fit_gains(stats: dict, target: np.ndarray, tol: float, gc_max: float = G_MAX_COARSE, gf_max: float = G_MAX,
              coarse_step: float = 0.005, step: float = G_STEP, margin: float = 0.02) -> dict:
    """Two gains (coarse, fine) against the site-aware criteria (module docstring), grid search
    coarse-to-fine. Objective (lexicographic): 1) maximise the number of criteria met with ``margin``
    to spare (ratio within 1 +- (tol - margin); the margin absorbs the small change of the talus term
    with the coarse gain), 2) minimise sum ln(nu/target)^2 over all ratio criteria (centres every lag,
    including the cm scale, instead of only the worst one). Deterministic: ties go to the smaller coarse
    gain, then the smaller fine gain."""
    def search(gc, gf):
        ev = _site_aware_eval(stats, target, tol, gc, gf, margin)
        n_met, ssq = ev["n_met"], ev["ssq"]
        best_n = n_met.max()
        s = np.where(n_met == best_n, ssq, np.inf)
        j, i = np.unravel_index(int(np.argmin(s)), s.shape)       # argmin -> first (smallest gc, gf)
        return float(gc[j]), float(gf[i]), int(best_n), float(ev["worst"][j, i]), float(s[j, i]), ev["n_criteria"]

    gc0 = np.round(np.arange(0.0, gc_max + coarse_step / 2, coarse_step), 6)
    gf0 = np.round(np.arange(0.0, gf_max + coarse_step / 2, coarse_step), 6)
    gc, gf, *_ = search(gc0, gf0)
    w = 4 * coarse_step
    gc1 = np.round(np.arange(max(0.0, gc - w), min(gc_max, gc + w) + step / 2, step), 6)
    gf1 = np.round(np.arange(max(0.0, gf - w), min(gf_max, gf + w) + step / 2, step), 6)
    gc, gf, n_met, worst, ssq, n_crit = search(gc1, gf1)
    return {"gain": gc, "gain_fine": gf, "n_criteria": n_crit, "n_criteria_met": n_met,
            "max_abs_log_ratio": worst, "sum_sq_log_ratio": ssq, "tolerance_frac": tol, "margin": margin,
            "criteria": {"composite_lags_m": list(COMPOSITE_LAGS_M), "synthetic_lags_m": list(SYNTHETIC_LAGS_M),
                         "dtm_min_lag_m": DTM_MIN_LAG_M, "dtm_ratio_max": DTM_RATIO_MAX},
            "objective": "max #site-aware criteria met with the margin to spare, then min sum ln(nu/target)^2 "
                         "over the ratio criteria",
            "search": {"coarse_step": coarse_step, "step": step, "gc_max": gc_max, "gf_max": gf_max}}


# ---------------------------------------------------------------------------------------- components
def core_components(cfg: Config, log=print) -> np.ndarray:
    """Stack (5, n, n) float32 of the core-square fields B, K (= K_big + w_core K_small), T (talus),
    Fc (unit gain) and Ff (unit gain, x w_core) at 1 cm (|x|, |y| <= core half)."""
    lay = Layout(cfg)
    core = lay.zone_rect("core")
    sp = float(cfg.zones.core.spacing_m)
    n = int(round(core.width / sp)) + 1
    out = np.empty((len(COMPONENTS), n, n), dtype=np.float32)
    spec = {"B": (("B",), None), "K": (("Kb", "Ks"), None), "T": (("T",), None), "Fc": (("Fc",), (1.0, 0.0)),
            "Ff": (("Ff",), (0.0, 1.0))}
    for c, name in enumerate(COMPONENTS):
        t0 = time.time()
        parts, scale = spec[name]
        out[c] = tcompose.core_height(cfg, core.x0, core.x1, core.y0, core.y1, parts=parts, fbm_scale=scale).z
        log(f"[calibrate] component {name}: {time.time() - t0:.1f} s")
    return out


def summarize(stats: dict, cfg: Config, g: float, g_fine: float | None = None) -> dict:
    """Composite, synthetic and per-component nu at gains (g, g_fine) (g_fine None: shared gain)."""
    n = _n_comp(stats)
    gf = g if g_fine is None else g_fine
    L = np.asarray(stats["lags_m"])
    T = target_nu(cfg, L)
    tol = float(cfg.terrain.roughness.tolerance_frac)
    r = nu_of(stats, fbm_coef(g, n, gf))
    nu = r["nu"]
    syn = nu_of(stats, synthetic_coef(g, gf, n))["nu"]
    nuB = nu_of(stats, [1.0] + [0.0] * (n - 1))["nu"]
    names = list(COMPONENTS) if n == len(COMPONENTS) else ["B", "K", "Fc", "Ff"]
    comp = {}
    for i, name in enumerate(names):
        a = [0.0] * n
        a[i] = 1.0 if name not in ("Fc", "Ff") else math.sqrt(g if name == "Fc" else gf)
        comp[name if name in ("B", "K", "T") else f"{name}_at_gain"] = nu_of(stats, a)["nu"].tolist()
    comp["B+K+T"] = nu_of(stats, fbm_coef(0.0, n, 0.0))["nu"].tolist()
    comp["F_at_gain"] = nu_of(stats, [0.0] * (n - 2) + [math.sqrt(g), math.sqrt(gf)])["nu"].tolist()
    comp["F_unit"] = nu_of(stats, [0.0] * (n - 2) + [1.0, 1.0])["nu"].tolist()
    comp["synthetic"] = syn.tolist()
    rows = []
    for i, Lv in enumerate(L):
        row = {"L_m": float(Lv), "target_m": float(T[i]), "nu_m": float(nu[i]), "ratio": float(nu[i] / T[i]),
               "nu_synthetic_m": float(syn[i]), "ratio_synthetic": float(syn[i] / T[i]),
               "nu_B_m": float(nuB[i]), "ratio_to_B": float(nu[i] / nuB[i]) if nuB[i] > 0 else None}
        if any(abs(Lv - c) < 1e-9 for c in COMPOSITE_LAGS_M):
            row.update(rule="composite within tolerance", pass_=bool(abs(nu[i] / T[i] - 1) <= tol + 1e-12))
        elif any(abs(Lv - c) < 1e-9 for c in SYNTHETIC_LAGS_M):
            row.update(rule="synthetic H0 - B within tolerance", pass_=bool(abs(syn[i] / T[i] - 1) <= tol + 1e-12))
        elif Lv >= DTM_MIN_LAG_M - 1e-9:
            row.update(rule=f"composite <= {DTM_RATIO_MAX} x B", pass_=bool(nu[i] <= DTM_RATIO_MAX * nuB[i]))
        rows.append(row)
    return {"gain": g, "gain_fine": gf, "lags_m": L.tolist(), "target_nu_m": T.tolist(), "nu_m": nu.tolist(),
            "nu_x_m": r["nu_x"].tolist(), "nu_y_m": r["nu_y"].tolist(), "ratio": (nu / T).tolist(),
            "within_tolerance": (np.abs(nu / T - 1) <= tol + 1e-12).tolist(),
            "rms_slope_deg": np.degrees(np.arctan(nu / L)).tolist(),
            "target_rms_slope_deg": np.degrees(np.arctan(T / L)).tolist(), "components_nu_m": comp,
            "site_aware_rows": rows, "site_aware_pass": all(r_.get("pass_", True) for r_ in rows)}


# ---------------------------------------------------------------------------------------- config edit
def _value_re(key: str) -> re.Pattern:
    return re.compile(rf"^(?P<pre>[ \t]*{re.escape(key)}:[ \t]*)(?P<val>[-+0-9.eE]+|null|~)(?P<post>[^\r\n]*)(?=\r?\n|\Z)",
                      re.M)


def _fmt_gain(gain: float) -> str:
    new = f"{gain:.3f}".rstrip("0").rstrip(".") if gain != 0 else "0.0"
    if "." not in new:
        new += ".0"
    return new


def write_gain_to_config(path: str | Path, gain: float, key: str = "fbm_gain") -> str | None:
    """Replace only the value of ``terrain.roughness.<key>`` in a YAML file (comment column kept).

    For ``fbm_gain_fine`` a missing line is inserted right after the ``fbm_gain`` line (same indent,
    comment aligned). Returns the old value (None if the line was inserted)."""
    path = Path(path)
    raw = path.read_bytes().decode("utf-8")
    matches = list(_value_re(key).finditer(raw))
    new = _fmt_gain(gain)
    if not matches and key == "fbm_gain_fine":
        anchor = list(_value_re("fbm_gain").finditer(raw))
        if len(anchor) != 1:
            raise ValueError(f"expected exactly one 'fbm_gain:' line in {path}, found {len(anchor)}")
        a = anchor[0]
        post = a.group("post")
        ccol = len(a.group("pre")) + len(a.group("val")) + (len(post) - len(post.lstrip(" "))) if post.strip() else 0
        indent = a.group("pre")[: len(a.group("pre")) - len(a.group("pre").lstrip(" \t"))]
        line = f"{indent}fbm_gain_fine: {new}"
        comment = "# fine-band (< split_wavelength_m) fBm PSD gain; calibrated with fbm_gain (FID-06)"
        line = line + " " * max(1, ccol - len(line)) + comment if ccol else line + "  " + comment
        nl = "\r\n" if "\r\n" in raw else "\n"
        raw2 = raw[:a.end()] + nl + line + raw[a.end():]
        path.write_bytes(raw2.encode("utf-8"))
        return None
    if len(matches) != 1:
        raise ValueError(f"expected exactly one '{key}:' line in {path}, found {len(matches)}")
    m = matches[0]
    # keep the comment column: pad/trim the spaces after the value
    post = m.group("post")
    width = len(m.group("val")) + (len(post) - len(post.lstrip(" "))) if post.strip() else 0
    post_new = (" " * max(1, width - len(new)) + post.lstrip(" ")) if post.strip() else post
    raw2 = raw[:m.start()] + m.group("pre") + new + post_new + raw[m.end():]
    path.write_bytes(raw2.encode("utf-8"))
    return m.group("val")


# ---------------------------------------------------------------------------------------- figure
def plot(cal: dict, path: Path = FIGURE, dpi: int = 110) -> Path:
    """Three panels: nu/target with the tolerance band (composite and synthetic part), absolute nu with
    its components, RMS slope.

    Colours follow the entity (dataviz reference palette, fixed slot order, validated for adjacent
    CVD separation): H0 calibrated = slot 1 blue, H0 before = slot 2 orange, DTM B = slot 3 aqua,
    craters + talus K = slot 4 yellow, fBm F = slot 5 magenta; the target is reference chrome.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    px = 72.0 / dpi                                     # points per pixel
    surface, ink, ink2, muted, grid_c, axis_c = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
    c_post, c_pre, c_B, c_K, c_F = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"
    lw, ms, ring = 2 * px, 8 * px, 2 * px
    L = np.asarray(cal["lags_m"])
    T = np.asarray(cal["target_nu_m"])
    tol = cal["tolerance_frac"]
    nu1, H = cal["nu1_m"], cal["hurst"]
    Lf = np.geomspace(L.min() * 0.8, L.max() * 1.25, 200)
    pre = cal["pre"]
    post = cal.get("verified") or cal["predicted"]
    post_kind = "measured" if cal.get("verified") else "predicted"
    comp = post.get("components_nu_m") or cal["predicted"]["components_nu_m"]

    def glabel(s):
        gf = s.get("gain_fine", s["gain"])
        return f"gains {s['gain']:g} / {gf:g}" if gf != s["gain"] else f"fbm_gain {s['gain']:g}"

    plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "axes.edgecolor": axis_c,
                         "axes.labelcolor": ink2, "xtick.color": muted, "ytick.color": muted,
                         "text.color": ink, "axes.titlecolor": ink})
    fig, ax = plt.subplots(1, 3, figsize=(16.0, 5.0), facecolor=surface)

    def line(axx, x, y, color, label, z=3, ls="-", marker="o"):
        axx.plot(x, y, color=color, lw=lw, ls=ls, solid_capstyle="round", solid_joinstyle="round", marker=marker,
                 ms=ms, mfc=color, mec=surface, mew=ring, label=label, zorder=z)

    # (a) ratio to the target with the acceptance band
    a = ax[0]
    a.axhspan(1 - tol, 1 + tol, color=ink2, alpha=0.08, lw=0, label=f"tolerance ±{tol:.0%}")
    a.axhline(1.0, color=ink2, lw=1 * px * 1.5, zorder=2)
    line(a, L, np.asarray(pre["nu_m"]) / T, c_pre, f"H0, {glabel(pre)} (before)")
    line(a, L, np.asarray(post["nu_m"]) / T, c_post, f"H0, {glabel(post)} ({post_kind})", z=4)
    syn = cal["predicted"]["components_nu_m"].get("synthetic")
    if syn is not None:
        line(a, L, np.asarray(syn) / T, c_K, "synthetic H0 - B (predicted)", z=3, ls="--", marker="s")
    a.set_xscale("log")
    a.set_xlabel("lag L [m]")
    a.set_ylabel("ν(L) / target")
    a.set_title("Deviogram relative to the target", loc="left", fontsize=11)
    a.legend(fontsize=8, frameon=False, loc="upper left")

    # (b) absolute deviogram and its components at the calibrated gains
    b = ax[1]
    b.fill_between(Lf, nu1 * Lf ** H * (1 - tol), nu1 * Lf ** H * (1 + tol), color=ink2, alpha=0.08, lw=0)
    b.plot(Lf, nu1 * Lf ** H, color=ink2, lw=1.5 * px * 1.5, label=f"target ν1·L^H (ν1 = {nu1} m, H = {H})", zorder=2)
    series = [("H0", post["nu_m"], c_post, f"H0 ({post_kind})", ink, 4)]
    series += [(name, comp[key], color, name, ink2, 3) for key, color, name in (
        ("B", c_B, "DTM base B"), ("K", c_K, "craters K"), ("F_at_gain", c_F, "fBm F"))]
    for _, y, color, label, _, z in series[1:] + series[:1]:
        line(b, L, y, color, label, z=z)
    placed: list[float] = []
    for text, y, _, _, tcolor, _ in series:
        ly = math.log10(y[-1])
        if all(abs(ly - p) > 0.12 for p in placed):
            b.annotate(text, (L[-1], y[-1]), xytext=(6, 0), textcoords="offset points", va="center",
                       fontsize=8, color=tcolor)
            placed.append(ly)
    b.set_xscale("log")
    b.set_yscale("log")
    b.set_xlim(L.min() * 0.7, L.max() * 2.2)
    b.set_xlabel("lag L [m]")
    b.set_ylabel("ν(L) [m]  (plane-detrended, directional)")
    b.set_title(f"Components at {glabel(post)}", loc="left", fontsize=11)
    b.legend(fontsize=8, frameon=False, loc="upper left")

    # (c) RMS slope
    c = ax[2]
    c.plot(Lf, np.degrees(np.arctan(nu1 * Lf ** H / Lf)), color=ink2, lw=1.5 * px * 1.5, label="target atan(ν/L)",
           zorder=2)
    tl = cal.get("targets_rms_slope_deg") or {}
    if tl:
        c.plot([float(k) for k in tl], list(tl.values()), "D", color=ink2, ms=ms * 0.9, mfc=surface, mew=1.5 * px * 1.5,
               label="config targets_rms_slope_deg", zorder=3)
    line(c, L, pre["rms_slope_deg"], c_pre, f"{glabel(pre)} (before)")
    line(c, L, post["rms_slope_deg"], c_post, f"{glabel(post)} ({post_kind})", z=4)
    c.set_xscale("log")
    c.set_xlabel("baseline L [m]")
    c.set_ylabel("RMS slope [deg]")
    c.set_title("RMS slope vs baseline", loc="left", fontsize=11)
    c.legend(fontsize=8, frameon=False, loc="upper right")

    for axx in ax:
        axx.set_facecolor(surface)
        axx.grid(True, which="major", color=grid_c, lw=px, ls="-")
        axx.set_axisbelow(True)
        for s in ("top", "right"):
            axx.spines[s].set_visible(False)
    rows = cal["predicted"].get("site_aware_rows") or []
    checked = [r for r in rows if "pass_" in r]
    n_ok = sum(1 for r in checked if r["pass_"])
    sub = (f"Core H0 (100 m, 1 cm, no rocks), {glabel(post)}: site-aware criteria met {n_ok} of {len(checked)} "
           f"(composite ±{tol:.0%} at 0.01-0.3 m, H0 - B ±{tol:.0%} at 1-3 m, composite <= {DTM_RATIO_MAX} x B at >= 10 m)")
    fig.suptitle(sub, x=0.01, ha="left", fontsize=10, color=ink2)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=dpi, facecolor=surface)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------------------- driver
def stats_key(cfg: Config, lags_m) -> str:
    """Key of the cross statistics (components incl. the talus field, fades, geometry, lags)."""
    m = tcompose.get_model(cfg)
    z = cfg.zones
    return tcompose._hash({"components": m.component_keys(), "talus": m.talus_key,
                           "terrain_version": tcompose.TERRAIN_VERSION,
                           "zones": {k: dict(z[k]) for k in ("core", "ring")}, "lags_m": list(lags_m),
                           "estimator": ESTIMATOR})


def _load_stats(path: Path, key: str) -> dict | None:
    try:
        old = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    st = old.get("stats") or {}
    if old.get("stats_key") != key or "S" not in st:
        return None
    return {"lags_m": old["lags_m"], "k": [int(round(L / st["spacing_m"])) for L in old["lags_m"]],
            "S": np.asarray(st["S"], dtype=np.float64), "npairs": np.asarray(st["npairs"], dtype=np.int64),
            "plane": np.asarray(st["plane"], dtype=np.float64), "spacing": st["spacing_m"], "shape": st["shape"]}


def _gains_of(cfg: Config) -> tuple[float, float]:
    r = cfg.terrain.roughness
    gc = float(r.fbm_gain)
    gf = r.get("fbm_gain_fine")
    return gc, (gc if gf is None else float(gf))


def _stats_for(cfg: Config, tdir: Path, lags_m, reuse: bool, log) -> tuple[dict, float, float]:
    skey = stats_key(cfg, lags_m)
    stats = _load_stats(tdir / "calibration.json", skey) if reuse else None
    t_comp = t_stats = 0.0
    if stats is None:
        tcompose.build_all(cfg, stages=("talus",), log=log)
        t0 = time.time()
        comps = core_components(cfg, log=log)
        t_comp = time.time() - t0
        t0 = time.time()
        stats = cross_stats(comps, float(cfg.zones.core.spacing_m), lags_m)
        t_stats = time.time() - t0
        del comps
    else:
        log("[calibrate] reusing the cross statistics stored in calibration.json (components unchanged)")
    return stats, t_comp, t_stats


def run(cfg: Config, apply: bool = False, rebuild: bool = True, config_path: Path | None = None,
        figure: Path | None = FIGURE, log=print, lags_m=LAGS_M, reuse_stats: bool = True,
        single_gain: bool = False, rounds: int = 3) -> dict:
    """Measure, fit, (apply + rebuild + verify) and record the calibration; see the module docstring.

    Two gains (coarse, fine) against the site-aware criteria by default; ``single_gain`` fits one shared
    gain against the composite target at every lag (the phase-2 rule). With apply=True the gains are
    written to ``config_path`` (default: the file the config was loaded from); because the talus term
    follows the coarse gain, fit and apply repeat (at most ``rounds`` times) until the gains are stable,
    then the H0 tiles / zone grids are rebuilt and the deviogram is re-measured on the assembled core.
    """
    t_start = time.time()
    lags_m = tuple(float(v) for v in lags_m)
    lay = Layout(cfg)
    tdir = lay.stage_dir("terrain")
    tcompose.build_all(cfg, stages=("base", "fbm", "craters", "talus"), log=log)
    rough = cfg.terrain.roughness
    T = target_nu(cfg, lags_m)
    tol = float(rough.tolerance_frac)
    g0c, g0f = _gains_of(cfg)
    cpath = Path(config_path or cfg.get("_source", {}).get("config_file") or DEFAULT_CONFIG)
    history = []
    cfg_cur = cfg
    timings = {"components": 0.0, "cross_stats": 0.0}
    pre = None
    for rnd in range(max(1, rounds)):
        stats, t_comp, t_stats = _stats_for(cfg_cur, tdir, lags_m, reuse_stats, log)
        stats_cfg = cfg_cur
        timings["components"] += round(t_comp, 1)
        timings["cross_stats"] += round(t_stats, 1)
        gc_cur, gf_cur = _gains_of(cfg_cur)
        if pre is None:
            pre = summarize(stats, cfg_cur, gc_cur, gf_cur)
        if single_gain:
            fit = fit_gain(stats, T, tol)
            g_new = (float(f"{fit['gain']:.3f}"),) * 2
        else:
            fit = fit_gains(stats, T, tol)
            g_new = (float(f"{fit['gain']:.3f}"), float(f"{fit['gain_fine']:.3f}"))
        history.append({"round": rnd, "gains_in": [gc_cur, gf_cur], "gains_fit": list(g_new),
                        "talus_key": tcompose.get_model(cfg_cur).talus_key})
        stable = abs(g_new[0] - gc_cur) < 5e-4 and abs(g_new[1] - gf_cur) < 5e-4
        if not apply or stable:
            break
        write_gain_to_config(cpath, g_new[0], "fbm_gain")
        if not single_gain or cfg_cur.terrain.roughness.get("fbm_gain_fine") is not None:
            write_gain_to_config(cpath, g_new[1], "fbm_gain_fine")
        src = cfg.get("_source", {})
        cfg_cur = load_config(cpath, preset=src.get("preset"), overrides=src.get("overrides"))
        if _gains_of(cfg_cur) != tuple(g_new):
            raise RuntimeError(f"config gains were not updated (overrides of fbm_gain in --set win over the file?)")
        tcompose.build_all(cfg_cur, stages=("talus",), log=log)
    cal = {"estimator": "plane-detrended directional deviogram over the core square at 1 cm "
                        "(see lunar_world_gen/terrain/calibrate.py docstring)",
           "lags_m": list(lags_m), "nu1_m": float(rough.nu1_m), "hurst": float(rough.hurst),
           "tolerance_frac": tol, "target_nu_m": T.tolist(),
           "targets_rms_slope_deg": {str(k): v for k, v in dict(rough.targets_rms_slope_deg).items()},
           "stats_key": stats_key(stats_cfg, lags_m),
           "stats": {"S": stats["S"].tolist(), "npairs": stats["npairs"].tolist(), "plane": stats["plane"].tolist(),
                     "components": list(COMPONENTS), "spacing_m": float(cfg.zones.core.spacing_m),
                     "shape": stats["shape"],
                     "note": "S[lag][axis x|y][c][d]: mean product of plane-detrended lag differences, m^2"},
           "pre": pre, "fit": fit, "fit_mode": "single gain" if single_gain else "two gains, site-aware",
           "rounds": history, "predicted": summarize(stats, stats_cfg, g_new[0], g_new[1]),
           "timings_s": timings}
    if not single_gain:
        cal["fit_single_gain_reference"] = {k: v for k, v in fit_gain(stats, T, tol).items()
                                            if k in ("gain", "n_outside_tolerance", "gain_hitting_target_per_lag")}
    if apply:
        cal["applied"] = {"config": str(cpath), "old_values": [g0c, g0f], "new_values": list(g_new),
                          "stable": bool(stable)}
        if rebuild:
            t0 = time.time()
            tcompose.build_all(cfg_cur, stages=("talus", "tiles", "zones", "checks"), log=log)
            cal["timings_s"]["rebuild"] = round(time.time() - t0, 1)
            t0 = time.time()
            core = tcompose.assemble_core(cfg_cur)
            st2 = cross_stats(np.asarray(core.z)[None], float(cfg.zones.core.spacing_m), lags_m)
            del core
            v = summarize_single(st2, cfg_cur)
            v["components_nu_m"] = cal["predicted"]["components_nu_m"]
            v["max_abs_rel_diff_vs_predicted"] = float(np.max(np.abs(np.asarray(v["nu_m"]) /
                                                                     np.asarray(cal["predicted"]["nu_m"]) - 1)))
            cal["verified"] = v
            cal["timings_s"]["verify"] = round(time.time() - t0, 1)
    cal["timings_s"]["total"] = round(time.time() - t_start, 1)
    cal["peak_rss"] = tcompose._peak_rss_mb()
    (tdir / "calibration.json").write_text(json.dumps(cal, indent=1, default=str))
    if figure:
        plot(cal, Path(figure))
        cal["figure"] = str(figure)
        (tdir / "calibration.json").write_text(json.dumps(cal, indent=1, default=str))
    return cal


def summarize_single(stats: dict, cfg: Config) -> dict:
    L = np.asarray(stats["lags_m"])
    T = target_nu(cfg, L)
    tol = float(cfg.terrain.roughness.tolerance_frac)
    r = nu_of(stats, [1.0])
    nu = r["nu"]
    gc, gf = _gains_of(cfg)
    return {"gain": gc, "gain_fine": gf, "lags_m": L.tolist(), "target_nu_m": T.tolist(),
            "nu_m": nu.tolist(), "nu_x_m": r["nu_x"].tolist(), "nu_y_m": r["nu_y"].tolist(),
            "ratio": (nu / T).tolist(), "within_tolerance": (np.abs(nu / T - 1) <= tol + 1e-12).tolist(),
            "rms_slope_deg": np.degrees(np.arctan(nu / L)).tolist(),
            "target_rms_slope_deg": np.degrees(np.arctan(T / L)).tolist()}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Calibrate the fBm gains (terrain.roughness.fbm_gain and "
                                             "fbm_gain_fine) against the site-aware roughness criteria.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--preset", default=None)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--apply", action="store_true", help="write the fitted gains into the config and rebuild")
    ap.add_argument("--no-rebuild", action="store_true")
    ap.add_argument("--single-gain", action="store_true", help="one shared gain (phase-2 rule)")
    ap.add_argument("--figure", default=str(FIGURE), help="output PNG ('' to skip)")
    ap.add_argument("--lags", default=None, help="comma list of lags in m (default 0.01,...,10)")
    ap.add_argument("--recompute", action="store_true", help="do not reuse stored cross statistics")
    args = ap.parse_args(argv)
    cfg = load_config(args.config, preset=args.preset, overrides=args.set)
    lags = tuple(float(v) for v in args.lags.split(",")) if args.lags else LAGS_M
    cal = run(cfg, apply=args.apply, rebuild=not args.no_rebuild,
              config_path=Path(args.config) if args.config else None,
              figure=Path(args.figure) if args.figure else None, lags_m=lags, reuse_stats=not args.recompute,
              single_gain=args.single_gain)
    out = {k: cal[k] for k in ("fit", "rounds", "timings_s", "peak_rss") if k in cal}
    for k in ("pre", "predicted", "verified"):
        if k in cal:
            out[k] = {kk: cal[k].get(kk) for kk in ("gain", "gain_fine", "nu_m", "ratio", "within_tolerance",
                                                    "rms_slope_deg", "site_aware_rows", "site_aware_pass")}
    print(json.dumps(out, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
