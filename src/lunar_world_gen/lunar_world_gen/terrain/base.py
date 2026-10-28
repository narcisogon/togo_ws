"""Band-limited DTM base field B(x, y) for every zone (docs/DESIGN.md sections 2-3).

B comes from ``tools/dtm_extract.py`` (run as a subprocess with the current interpreter) around the
config site (``cfg.site``: lat/lon/href/yaw, identical to site_selection.yaml key ``core``), in the
local ENU frame with Moon curvature, cubic B-spline resampling (``cfg.dtm.interp``) and:

=========  ========  ==================================  =====================================================
zone       spacing   cached extraction (square)          processing
=========  ========  ==================================  =====================================================
core       1 cm      core + 2 x core.margin_m (101 m)    Butterworth low-pass (lambda_c 10 m, order 4)
ring       25 cm     ring + 2 x RING_MARGIN_M (540 m)    Butterworth low-pass
far        2 m       far + 2 x FAR_SKIRT_M (2.6 km)      Butterworth low-pass + ``--deseam`` (zones.far.deseam)
horizon    20 m      horizon (20 km)                     block average + auto anti-alias (DESIGN: "DTM
                                                         block-averaged") + ``--deseam``; NaN filled
=========  ========  ==================================  =====================================================

Why these choices
    * The far field uses the same Butterworth low-pass as the ring so that B agrees across the ring's
      outer boundary (where w_ring = 0 and H = B); without it the far field would carry the DTM's
      sub-10 m content and differ from the ring by decimetres.
    * Every extraction is wider than its zone (core by core.margin_m, ring by RING_MARGIN_M, far by
      FAR_SKIRT_M). compose nests them into ONE global B: inside a zone's square B is that zone's
      extraction (selected exactly), and over the extra border it blends (raised cosine) into the next
      outer extraction. So every zone sees bit-identical B at shared points, which also absorbs the
      remaining differences between extractions: dtm_extract applies its vertical-sampling iteration
      only when h*d/R exceeds a spacing-dependent tolerance, which the 2.6 km far extraction does and
      the ring does not (about 1 mm at 250 m), and the horizon is block-averaged (up to 2.4 m off the
      far field at 1.1 km). ``base_grid(cfg, 'far')`` crops the zone out of the wider extraction.
      The horizon also uses ``--deseam`` so that the feathered east seam (which crosses the far
      field's north and south edges) continues consistently.
    * Horizon nodata (2 % beyond 8.1 km N-NE, also absent from the filled DTM) is filled by harmonic
      (Laplace) interpolation, which gives the large-scale shape, plus a textured residual (FID-17,
      ``fill_nodata_textured``): fBm noise with the Hurst exponent and amplitude fitted to the valid DTM
      within 2 km of the holes (deviogram at 1-10 nodes), high-passed above 1 km so the harmonic fill
      keeps the large-scale shape, smoothed at the node scale like the block-averaged DTM (Gaussian,
      1.2 nodes), and tapered in from 0 at the hole boundary over 10 nodes (C0 continuous with the real
      data). Deterministic (``config.rng(cfg, 'terrain.base.<zone>_fill')``).
      Without it the holes were smooth rectangles with 2.5x less relief at 20-200 m lags. The mask is
      kept.

Cache
    ``build_dir/terrain/base/<zone>.npy`` + ``<zone>.json`` in ``Grid.save`` format (``Grid.load``
    works; the 400 MB core grid is opened with mmap). ``meta`` holds the cache key, the dtm_extract
    metadata (``meta['dtm_extract']``), timings and the child's peak RSS. The key covers the site, the
    DTM files (path + size), the zone parameters, the dtm settings and a hash of dtm_extract.py.
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from ..config import Config
from ..grid import Grid, from_lattice, to_lattice
from ..layout import Layout

BASE_VERSION = "1"
FILL_VERSION = "textured-3"   # nodata fill of the zones with fill_nodata (far, horizon); FID-17
FILL_TAPER_NODES = 10         # texture fades in from 0 at the hole boundary over this many nodes
FILL_HIGHPASS_M = 1000.0      # texture keeps wavelengths below this (the harmonic fill carries the rest)
FILL_LOCAL_NODES = 100        # roughness is fitted on valid nodes within this distance of the holes
FILL_SMOOTH_NODES = 1.2       # Gaussian sigma [nodes] applied to the texture: the block-averaged, anti-aliased
                              # DTM carries little node-scale curvature (horizon: Laplacian RMS 1.54 m in the fill
                              # vs 1.58 m in the valid 10-node ring around it; 0.6 nodes gave 2.36 m)
RING_MARGIN_M = 20.0     # ring extraction border beyond the 500 m square (ring -> far blend band, see compose)
FAR_SKIRT_M = 200.0      # far extraction border beyond the 2.2 km square (far -> horizon blend band)
BASE_ZONES = ("core", "ring", "far", "horizon")


@dataclass(frozen=True)
class BaseSpec:
    zone: str
    spacing_m: float
    half_m: float          # half width of the cached extraction
    zone_half_m: float     # half width of the zone grid (base_grid default; core includes its margin)
    lowpass: bool
    deseam: str | None
    fill_nodata: bool


def base_spec(cfg: Config, zone: str) -> BaseSpec:
    if zone not in BASE_ZONES:
        raise ValueError(f"zone must be one of {BASE_ZONES}, got {zone!r}")
    z = cfg.zones
    deseam = z.far.get("deseam") or None
    if zone == "core":
        half = z.core.size_m / 2 + float(z.core.margin_m)
        spec = BaseSpec("core", float(z.core.spacing_m), half, half, True, None, False)
    elif zone == "ring":
        spec = BaseSpec("ring", float(z.ring.spacing_m), z.ring.size_m / 2 + RING_MARGIN_M, z.ring.size_m / 2,
                        True, None, False)
    elif zone == "far":
        spec = BaseSpec("far", float(z.far.spacing_m), z.far.size_m / 2 + FAR_SKIRT_M, z.far.size_m / 2,
                        True, deseam, True)
    else:
        spec = BaseSpec("horizon", float(z.horizon.spacing_m), z.horizon.size_m / 2, z.horizon.size_m / 2,
                        False, deseam, True)
    for v in (spec.spacing_m, spec.half_m, spec.zone_half_m):
        to_lattice(v)
    if abs(round(spec.half_m / spec.spacing_m) - spec.half_m / spec.spacing_m) > 1e-9:
        raise ValueError(f"{zone}: half extent {spec.half_m} is not a multiple of the spacing {spec.spacing_m}")
    return spec


def _file_sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


def base_inputs(cfg: Config, zone: str) -> dict:
    """Everything that determines a cached extraction (hashed into its cache key)."""
    spec = base_spec(cfg, zone)
    tool = Path(cfg.paths.dtm_extract)
    files = {}
    for key in ("dtm", "dtm_filled"):
        p = Path(cfg.paths[key])
        files[key] = {"path": str(p), "bytes": p.stat().st_size if p.exists() else None}
    out = {
        "version": BASE_VERSION, "spec": asdict(spec),
        "site": {k: cfg.site[k] for k in ("lat_deg", "lon_deg", "href_m", "yaw_deg")},
        "dtm": {"lowpass_lambda_m": float(cfg.dtm.lowpass_lambda_m), "lowpass_order": int(cfg.dtm.lowpass_order),
                "interp": str(cfg.dtm.interp)},
        "files": files, "tool": {"path": str(tool), "sha": _file_sha(tool) if tool.exists() else None},
    }
    if spec.fill_nodata:
        out["fill"] = {"version": FILL_VERSION, "taper_nodes": FILL_TAPER_NODES, "highpass_m": FILL_HIGHPASS_M,
                       "local_nodes": FILL_LOCAL_NODES, "smooth_nodes": FILL_SMOOTH_NODES, "seed": int(cfg.world.seed)}
    return out


def base_prefix(cfg: Config, zone: str) -> Path:
    return Layout(cfg).stage_dir("terrain") / "base" / zone


def _dtm_extract_cmd(cfg: Config, spec: BaseSpec, out: Path) -> list[str]:
    s = cfg.site
    cmd = [sys.executable, str(cfg.paths.dtm_extract),
           "--dtm", str(cfg.paths.dtm), "--fill-dtm", str(cfg.paths.dtm_filled),
           "--lat", repr(float(s.lat_deg)), "--lon", repr(float(s.lon_deg)),
           "--href", repr(float(s.href_m)), "--yaw-deg", repr(float(s.yaw_deg)),
           "--size", repr(2 * spec.half_m), "--spacing", repr(spec.spacing_m),
           "--interp", str(cfg.dtm.interp), "--out", str(out)]
    if spec.lowpass:
        cmd += ["--lowpass-lambda", repr(float(cfg.dtm.lowpass_lambda_m)),
                "--lowpass-order", str(int(cfg.dtm.lowpass_order))]
    if spec.deseam:
        cmd += ["--deseam", str(spec.deseam)]
    return cmd


def _run_child(cmd: list[str], log_path: Path) -> tuple[int, float, float]:
    """Run a command, logging stdout/stderr; returns (exit code, elapsed s, child peak RSS MB)."""
    t0 = time.time()
    with open(log_path, "w") as fh:
        fh.write(" ".join(cmd) + "\n")
        fh.flush()
        p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT)
        _, status, ru = os.wait4(p.pid, 0)
        p.returncode = os.waitstatus_to_exitcode(status)
    return p.returncode, time.time() - t0, ru.ru_maxrss / 1024.0


def fill_nodata_harmonic(z: np.ndarray) -> tuple[np.ndarray, int]:
    """Fill NaN nodes by solving Laplace's equation (4-neighbour) with the valid nodes as boundary values.

    Grid edges are natural (zero-flux) boundaries. NaN components with no valid neighbour are set to the
    global mean. Returns (filled float32 array, number of filled nodes).
    """
    import scipy.sparse as sp
    from scipy import ndimage
    from scipy.sparse.linalg import spsolve

    z = np.array(z, dtype=np.float64)
    bad = ~np.isfinite(z)
    n = int(bad.sum())
    if n == 0:
        return z.astype(np.float32), 0
    lab, nlab = ndimage.label(bad)
    touch = ndimage.binary_dilation(~bad) & bad
    has_ok = np.zeros(nlab + 1, dtype=bool)
    has_ok[np.unique(lab[touch])] = True
    orphan = bad & ~has_ok[lab]
    if orphan.any():
        z[orphan] = np.nanmean(z[~bad])
        bad &= ~orphan
    ny, nx = z.shape
    idx = -np.ones(z.shape, dtype=np.int64)
    jj, ii = np.nonzero(bad)
    m = jj.size
    idx[jj, ii] = np.arange(m)
    rows, cols, vals = [], [], []
    rhs = np.zeros(m)
    diag = np.zeros(m)
    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        j2, i2 = jj + dj, ii + di
        inside = (j2 >= 0) & (j2 < ny) & (i2 >= 0) & (i2 < nx)
        diag += inside
        k = np.nonzero(inside)[0]
        nb_idx = idx[j2[k], i2[k]]
        unknown = nb_idx >= 0
        rows.append(k[unknown])
        cols.append(nb_idx[unknown])
        vals.append(-np.ones(int(unknown.sum())))
        known = ~unknown
        np.add.at(rhs, k[known], z[j2[k[known]], i2[k[known]]])
    A = sp.csr_matrix((np.concatenate(vals + [diag]),
                       (np.concatenate(rows + [np.arange(m)]), np.concatenate(cols + [np.arange(m)]))),
                      shape=(m, m))
    z[jj, ii] = spsolve(A.tocsc(), rhs)
    return z.astype(np.float32), n


def _deviogram_masked(z: np.ndarray, ok: np.ndarray, lags: tuple[int, ...]) -> np.ndarray:
    """nu(k) = sqrt(mean of squared lag differences) over pairs with both nodes in ``ok`` (x and y)."""
    out = []
    for k in lags:
        dx = (z[:, k:] - z[:, :-k])[ok[:, k:] & ok[:, :-k]]
        dy = (z[k:, :] - z[:-k, :])[ok[k:, :] & ok[:-k, :]]
        d = np.concatenate([dx, dy])
        d = d - d.mean()                                  # remove the mean regional gradient
        out.append(float(np.sqrt(np.mean(d * d))) if d.size else float("nan"))
    return np.asarray(out)


def fill_nodata_textured(z: np.ndarray, spacing: float, rng: np.random.Generator,
                         taper_nodes: int = FILL_TAPER_NODES, highpass_m: float = FILL_HIGHPASS_M,
                         local_nodes: int = FILL_LOCAL_NODES,
                         smooth_nodes: float = FILL_SMOOTH_NODES) -> tuple[np.ndarray, int, dict]:
    """Harmonic fill plus a tapered fBm texture matched to the surrounding valid data (FID-17).

    1. ``fill_nodata_harmonic`` gives the large-scale shape (exact at the hole boundary).
    2. The deviogram nu(L) of the valid nodes within ``local_nodes`` of the holes at L = 1, 2, 5, 10
       nodes gives a Hurst exponent H and amplitude (log-log fit; H clipped to [0.3, 1.0]).
    3. fBm noise with PSD ~ f^-(2+2H), high-passed with the power-complementary Butterworth weight at
       ``highpass_m`` and smoothed with a Gaussian of ``smooth_nodes`` (the real coarse grids are
       block-averaged and anti-aliased, so they carry little power at the node scale), is scaled so that
       its nu at the reference lag (2 nodes) inside the holes equals the fitted nu there, and added with a
       raised-cosine taper that is 0 on the hole boundary and 1 from ``taper_nodes`` inside (continuous
       with the real data).
    Returns (filled float32 array, number of filled nodes, info).
    """
    from scipy import ndimage

    bad = ~np.isfinite(np.asarray(z))
    filled, n = fill_nodata_harmonic(z)
    info: dict = {"method": "harmonic + tapered fBm texture", "n_filled": n}
    if n == 0:
        return filled, 0, info
    zz = np.asarray(filled, dtype=np.float64)
    valid = ~bad
    near = ndimage.binary_dilation(bad, iterations=local_nodes) & valid
    lags = (1, 2, 5, 10)
    nu = _deviogram_masked(zz, near, lags)
    L = np.asarray(lags, float) * spacing
    good = np.isfinite(nu) & (nu > 0)
    if good.sum() < 2:
        info["texture"] = "skipped (not enough valid data near the holes)"
        return filled, n, info
    H, lognu0 = np.polyfit(np.log(L[good]), np.log(nu[good]), 1)
    H = float(np.clip(H, 0.3, 1.0))
    ny, nx = zz.shape
    nf = max(ny, nx)
    w = rng.standard_normal((nf, nf))
    fy = np.fft.fftfreq(nf, d=spacing)
    fx = np.fft.rfftfreq(nf, d=spacing)
    f = np.hypot(fy[:, None], fx[None, :])
    with np.errstate(divide="ignore"):
        amp = np.where(f > 0, f ** (-(1.0 + H)), 0.0)
    a = (f * highpass_m) ** 4
    amp *= a / np.sqrt(1.0 + a * a)                       # keep wavelengths below highpass_m
    if smooth_nodes > 0:
        amp *= np.exp(-2.0 * (np.pi * f * spacing * smooth_nodes) ** 2)     # Gaussian, sigma = smooth_nodes
    noise = np.fft.irfft2(np.fft.rfft2(w) * amp, s=(nf, nf))[:ny, :nx]
    k_ref = 2
    nu_noise = _deviogram_masked(noise, bad, (k_ref,))[0]
    target = float(np.exp(lognu0) * (k_ref * spacing) ** H)
    scale = target / nu_noise if nu_noise > 0 else 0.0
    dist = ndimage.distance_transform_edt(bad)            # >= 1 inside the holes
    t = np.clip((dist - 0.5) / float(taper_nodes), 0.0, 1.0)
    taper = 0.5 * (1.0 - np.cos(np.pi * t))
    zz[bad] += (scale * noise * taper)[bad]
    after = _deviogram_masked(zz, bad, lags)
    info.update({"hurst_fit": H, "nu_valid_near_m": dict(zip([f"{v:g}" for v in L], nu.tolist())),
                 "nu_fill_after_m": dict(zip([f"{v:g}" for v in L], after.tolist())),
                 "texture_scale": float(scale), "taper_nodes": taper_nodes, "highpass_m": highpass_m,
                 "local_nodes": local_nodes, "smooth_nodes": smooth_nodes})
    return zz.astype(np.float32), n, info


def extract_base(cfg: Config, zone: str, rebuild: bool = False) -> Grid:
    """The cached full extraction of a zone (runs dtm_extract on a cache miss)."""
    spec = base_spec(cfg, zone)
    prefix = base_prefix(cfg, zone)
    inputs = base_inputs(cfg, zone)
    key = _hash(inputs)
    jpath, npath = prefix.with_suffix(".json"), prefix.with_suffix(".npy")
    if not rebuild and jpath.exists() and npath.exists():
        try:
            if json.loads(jpath.read_text()).get("meta", {}).get("cache_key") == key:
                return Grid.load(prefix, mmap=(zone == "core"))
        except (OSError, json.JSONDecodeError):
            pass
    prefix.parent.mkdir(parents=True, exist_ok=True)
    tmp = prefix.with_name(prefix.name + "_tmp")
    cmd = _dtm_extract_cmd(cfg, spec, tmp)
    code, elapsed, rss_mb = _run_child(cmd, prefix.with_name(prefix.name + "_dtm_extract.log"))
    if code != 0:
        raise RuntimeError(f"dtm_extract failed for zone {zone} (exit {code}); see "
                           f"{prefix.with_name(prefix.name + '_dtm_extract.log')}")
    dmeta = json.loads(Path(str(tmp) + ".json").read_text())
    g = dmeta["grid"]
    nx, ny = int(g["nx"]), int(g["ny"])
    n_filled = 0
    fill_info = None
    tnpy = Path(str(tmp) + ".npy")
    if spec.fill_nodata:
        z = np.load(tnpy)
        if not np.isfinite(z).all():
            from ..config import rng as cfg_rng
            z, n_filled, fill_info = fill_nodata_textured(z, spec.spacing_m, cfg_rng(cfg, f"terrain.base.{zone}_fill"))
            np.save(tnpy, z)
        del z
    os.replace(tnpy, npath)
    tmask = Path(str(tmp) + "_mask.npy")
    if tmask.exists():
        os.replace(tmask, prefix.with_name(prefix.name + "_mask.npy"))
    Path(str(tmp) + ".json").unlink()
    meta = {"cache_key": key, "inputs": inputs, "zone": zone, "elapsed_s": round(elapsed, 2),
            "child_peak_rss_mb": round(rss_mb, 1), "nodata_filled_nodes": n_filled, "nodata_fill": fill_info,
            "command": " ".join(cmd[1:]), "dtm_extract": dmeta}
    # origin/spacing snapped to the exact 1 cm lattice values (dtm_extract computes -0.5*(n-1)*s in float)
    info = {"x0": float(from_lattice(to_lattice(g["x_min"]))), "y0": float(from_lattice(to_lattice(g["y_min"]))),
            "spacing": float(from_lattice(to_lattice(g["spacing_m"]))), "nx": nx, "ny": ny, "row0": "south",
            "meta": meta}
    jpath.write_text(json.dumps(info, indent=1, default=str))
    return Grid.load(prefix, mmap=(zone == "core"))


def crop_view(grid: Grid, x_min: float, x_max: float, y_min: float, y_max: float) -> Grid:
    """Node-aligned sub-grid like Grid.crop, but a view (no copy; stays memory-mapped)."""
    i0 = int(np.ceil((x_min - grid.x0) / grid.spacing - 1e-9))
    i1 = int(np.floor((x_max - grid.x0) / grid.spacing + 1e-9))
    j0 = int(np.ceil((y_min - grid.y0) / grid.spacing - 1e-9))
    j1 = int(np.floor((y_max - grid.y0) / grid.spacing + 1e-9))
    i0, j0 = max(i0, 0), max(j0, 0)
    i1, j1 = min(i1, grid.nx - 1), min(j1, grid.ny - 1)
    if i1 < i0 or j1 < j0:
        raise ValueError("crop window does not overlap the grid")
    return Grid(grid.z[j0:j1 + 1, i0:i1 + 1], grid.x0 + i0 * grid.spacing, grid.y0 + j0 * grid.spacing,
                grid.spacing, dict(grid.meta))


def base_grid(cfg: Config, zone: str, margin_m: float | None = None) -> Grid:
    """B for a zone at its spacing over the zone square (+ margin_m), from the cache.

    Defaults: core = 1 cm over core + 2 x core.margin_m (101 m); ring = 25 cm over 500 m;
    far = 2 m over 2.2 km; horizon = 20 m over 20 km. ``margin_m`` (if given) replaces the core margin
    or adds a border to the other zones; it must stay inside the cached extraction (core <= margin_m,
    ring <= RING_MARGIN_M, far <= FAR_SKIRT_M, horizon 0). The core grid is memory-mapped (read-only).
    """
    spec = base_spec(cfg, zone)
    full = extract_base(cfg, zone)
    if zone == "core":
        half = cfg.zones.core.size_m / 2 + (float(cfg.zones.core.margin_m) if margin_m is None else margin_m)
    else:
        half = spec.zone_half_m + (0.0 if margin_m is None else margin_m)
    if half > spec.half_m + 1e-9:
        raise ValueError(f"{zone}: requested half extent {half} m exceeds the cached extraction ({spec.half_m} m)")
    if abs(half - spec.half_m) < 1e-9:
        return full
    g = crop_view(full, -half, half, -half, half)
    return g if zone == "core" else Grid(np.array(g.z), g.x0, g.y0, g.spacing, g.meta)


def build_all_bases(cfg: Config, rebuild: bool = False) -> dict[str, dict]:
    """Extract (or validate the cache of) all four base grids; returns per-zone summaries."""
    out = {}
    for zone in BASE_ZONES:
        t0 = time.time()
        g = extract_base(cfg, zone, rebuild=rebuild)
        dm = g.meta.get("dtm_extract", {})
        out[zone] = {"spacing_m": g.spacing, "nx": g.nx, "ny": g.ny, "x0": g.x0, "y0": g.y0,
                     "elapsed_s": round(time.time() - t0, 2), "extract_elapsed_s": g.meta.get("elapsed_s"),
                     "child_peak_rss_mb": g.meta.get("child_peak_rss_mb"),
                     "nodata_filled_nodes": g.meta.get("nodata_filled_nodes"),
                     "stats": dm.get("stats"), "href_m": dm.get("frame", {}).get("href_m"),
                     "vertical_iteration": dm.get("frame", {}).get("vertical_iteration"),
                     "deseam": dm.get("processing", {}).get("deseam")}
    return out
