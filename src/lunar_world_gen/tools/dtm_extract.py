#!/usr/bin/env python3
"""Extract a local east-north-up (ENU) height grid from the LROC NAC Apollo 16 DTM.

The LROC DTM (NAC_DTM_APOLLO16.TIF, v1.9) is posted at 2 m in an Equidistant Cylindrical
projection on a sphere (R = 1737400 m, standard parallel -9 deg, central meridian 180 deg,
false easting/northing 0); heights are metres above the 1737.4 km sphere. This tool resamples
a region around a latitude/longitude into a regular grid in a local tangent-plane frame:

    x = east [m], y = north [m], z = up [m] relative to a reference height (href),

with the origin on the reference sphere point below the centre, raised to href. Each grid node
(x, y) is mapped through the gnomonic (tangent-plane) projection to lat/lon, then to DTM pixel
coordinates, and the height is interpolated (cubic B-spline by default, C2-smooth, no facets or
terraces). The Moon's curvature is included (z drops by ~d^2/2R: 0.3 m at 1 km, 7 m at 5 km), and
for large extents one fixed-point iteration samples the terrain on the local vertical rather than
the radial line (matters only at km distances in high terrain). Whether to iterate is decided ONCE per
extraction (max |h| of the DTM window over the grid's pixel box x max node distance / R against
max(0.05 * spacing, 5 mm)), so the output never depends on --chunk-nodes; --vertical-iteration
always/never overrides the decision.

Grid layout (default --row0 south):
    z[j, i] is the height at x = x_min + i*spacing, y = y_min + j*spacing, i = 0..nx-1, j = 0..ny-1,
    x_min = -size_x/2, y_min = -size_y/2 (node-centred; both edges included: nx = round(size_x/spacing)+1).
    Row 0 is the SOUTH edge. With --row0 north the rows are flipped (image layout, row 0 = north).
    If --yaw-deg is given, x/y are the grid axes rotated CCW by yaw from east/north.

Outputs (for --out PREFIX):
    PREFIX.npy        float32 heights [m] (NaN where neither DTM has data)
    PREFIX_mask.npy   uint8 per node: 1 = original DTM, 2 = hole-filled DTM used, 0 = no data
                      (written only when some node is not 1, unless --always-mask)
    PREFIX.json       metadata: centre lat/lon, projected x/y, pixel col/row, frame, href, grid,
                      processing parameters, statistics, conversion constants, command line
    PREFIX.png        optional hillshade quick-look (--png)

Usage (run with the project venv, e.g. ~/.venvs/lunar_world/bin/python):
    # 100 x 100 m core at 5 cm: DTM band-limited to its effective resolution (Butterworth, 10 m,
    # order 4) then C2 cubic-spline resampled; href = mean height of the 100 m square (auto)
    dtm_extract.py --lat -8.97370 --lon 15.51015 --size 100 --spacing 0.05 \
                   --lowpass-lambda 10 --out /home/nargon/togo_assets/lunar/dtm/core_5cm
    # same centre / href / yaw as the chosen site in docs/research/site_selection.yaml
    dtm_extract.py --site core --size 100 --spacing 0.05 --lowpass-lambda 10 --out ...
    # simpler Gaussian alternative (broader crossover, see site_selection.md)
    dtm_extract.py --site core --size 100 --spacing 0.05 --lowpass-sigma 1.9 --out ...
    # 2.2 x 2.2 km far field at native 2 m (no low-pass) with the core's href; feather the
    # 0.5-0.9 m strip seam ~960 m east of the core
    dtm_extract.py --site core --size 2200 --spacing 2 --deseam default --out .../far_2m
    # 20 km horizon at 20 m (auto block-average decimation + anti-alias low-pass)
    dtm_extract.py --site core --size 20000 --spacing 20 --out .../horizon_20m
    # self-test of the coordinate conversions on the real file
    dtm_extract.py --selftest

Options (see --help): --dtm, --fill-dtm/--no-fill, --size W [H], --spacing, --yaw-deg,
--href (number | auto), --href-box, --lowpass-sigma (m | auto), --lowpass-lambda (m),
--lowpass-order, --deseam (path | default), --deseam-taper, --interp {cubic,linear},
--no-curvature, --no-decimate, --row0 {south,north}, --chunk-nodes, --vertical-iteration
{auto,always,never}, --png, --site-yaml, --site.

Coordinate conversion (identical to pyproj for this CRS; verified by --selftest):
    K = R cos(lat_ts);   x_proj = x_0 + K * wrap180(lon - lon_0) [rad];   y_proj = y_0 + R * lat [rad]
    col = (x_proj - X0) / DX ;  row = (y_proj - Y0) / DY      (GDAL corner convention: pixel
    (c, r) covers [c, c+1) x [r, r+1), its centre is (c + 0.5, r + 0.5); X0, Y0, DX, DY from the
    GeoTIFF transform; for NAC_DTM_APOLLO16: X0 = -4937550.0000015, Y0 = -261204.00000008,
    DX = 2.0000000000006, DY = -2.0000000000006).

Notes:
  * The DTM contains sub-DTM (stereo-strip) seams with 0.3-0.9 m steps along N-S lines (e.g.
    ~1.26 km east and ~2.44 km west of the LM). Keep high-detail cores away from them; for the
    far field, --deseam default feathers the two traced seams near the LM (cosine taper, +-300 m;
    trace valid within +-3 km of the LM latitude), see docs/research/site_selection.md.
  * The DTM resolves features of roughly 8-10 m and larger (craters D >= ~12-16 m keep their depth).
    For the high-detail core use --lowpass-lambda 10 (Butterworth order 4: |H| = 0.998 at 20 m,
    0.707 at 10 m, 0.13 at 6 m) and add synthetic detail with the power-complementary weight
    sqrt(1 - H^2) (the transfer is written to the JSON), see site_selection.md.

Requires: numpy, scipy, rasterio (pyyaml for --site, matplotlib for --png).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

__version__ = "1.0"

_HERE = Path(__file__).resolve()
_SRC = _HERE.parents[2]  # .../src
DEFAULT_DTM = _SRC / "togo" / "togo_gz" / "media" / "dem" / "NAC_DTM_APOLLO16.TIF"
DEFAULT_FILL = _SRC / "togo" / "togo_gz" / "media" / "dem" / "NAC_DTM_APOLLO16_filled.tif"
DEFAULT_SITE_YAML = _HERE.parents[1] / "docs" / "research" / "site_selection.yaml"
DEFAULT_SEAMS = _HERE.parents[1] / "docs" / "research" / "dtm_seams_apollo16.json"


def wrap180(a):
    return (np.asarray(a, dtype=np.float64) + 180.0) % 360.0 - 180.0


class EqcDTM:
    """Thin wrapper around a GeoTIFF DTM in an equidistant-cylindrical (eqc) spherical CRS."""

    def __init__(self, path):
        import rasterio
        self.path = str(path)
        self.ds = rasterio.open(self.path)
        p = self.ds.crs.to_dict()
        if p.get("proj") != "eqc":
            raise ValueError(f"{path}: expected an eqc (equirectangular) CRS, got {p}")
        if float(p.get("lat_0", 0.0)) != 0.0:
            raise ValueError("lat_0 != 0 is not supported")
        self.R = float(p.get("R", p.get("a")))
        if "b" in p and abs(float(p["b"]) - self.R) > 1e-6:
            raise ValueError("ellipsoidal eqc is not supported (sphere expected)")
        self.lat_ts = float(p.get("lat_ts", 0.0))
        self.lon0 = float(p.get("lon_0", 0.0))
        self.xoff = float(p.get("x_0", 0.0))
        self.yoff = float(p.get("y_0", 0.0))
        self.K = self.R * math.cos(math.radians(self.lat_ts))
        t = self.ds.transform
        if abs(t.b) > 0 or abs(t.d) > 0:
            raise ValueError("rotated geotransforms are not supported")
        self.X0, self.DX, self.Y0, self.DY = t.c, t.a, t.f, t.e
        self.width, self.height = self.ds.width, self.ds.height
        self.nodata = self.ds.nodata

    # --- conversions -------------------------------------------------------------------------
    def ll2xy(self, lat, lon):
        x = self.xoff + self.K * np.radians(wrap180(np.asarray(lon) - self.lon0))
        y = self.yoff + self.R * np.radians(np.asarray(lat, dtype=np.float64))
        return x, y

    def xy2ll(self, x, y):
        lon = wrap180(np.degrees((np.asarray(x) - self.xoff) / self.K) + self.lon0)
        lat = np.degrees((np.asarray(y) - self.yoff) / self.R)
        return lat, lon

    def xy2px(self, x, y):
        return (np.asarray(x) - self.X0) / self.DX, (np.asarray(y) - self.Y0) / self.DY

    def px2xy(self, col, row):
        return self.X0 + np.asarray(col) * self.DX, self.Y0 + np.asarray(row) * self.DY

    def constants(self):
        return dict(proj="eqc (Equidistant Cylindrical, sphere)", R_m=self.R, lat_ts_deg=self.lat_ts,
                    lon_0_deg=self.lon0, x_0_m=self.xoff, y_0_m=self.yoff, K_m_per_rad=self.K,
                    X0=self.X0, Y0=self.Y0, DX=self.DX, DY=self.DY, width=self.width, height=self.height,
                    nodata=None if self.nodata is None else float(self.nodata),
                    formulas=["x = x_0 + K*rad(wrap180(lon - lon_0)), K = R*cos(lat_ts)", "y = y_0 + R*rad(lat)",
                              "col = (x - X0)/DX, row = (y - Y0)/DY (GDAL corner convention; pixel centre = +0.5)"])

    # --- reading ------------------------------------------------------------------------------
    def read(self, col0, row0, ncol, nrow, dec=1):
        """Read window (may extend outside the raster: outside -> NaN). With dec > 1 the window is
        block-averaged by GDAL (nodata-aware) to (ncol//dec, nrow//dec)."""
        import rasterio
        from rasterio.enums import Resampling
        from rasterio.windows import Window
        out_w, out_h = ncol // dec, nrow // dec
        full = np.full((out_h, out_w), np.nan, dtype=np.float64)
        # intersect with raster bounds, aligned to dec blocks
        c0 = max(col0, col0 + dec * math.ceil((0 - col0) / dec)) if col0 < 0 else col0
        r0 = max(row0, row0 + dec * math.ceil((0 - row0) / dec)) if row0 < 0 else row0
        c1 = min(col0 + out_w * dec, col0 + dec * ((self.width - col0) // dec))
        r1 = min(row0 + out_h * dec, row0 + dec * ((self.height - row0) // dec))
        if c1 <= c0 or r1 <= r0:
            return full
        w = Window(c0, r0, c1 - c0, r1 - r0)
        if dec == 1:
            a = self.ds.read(1, window=w, masked=True)
        else:
            a = self.ds.read(1, window=w, masked=True, out_shape=((r1 - r0) // dec, (c1 - c0) // dec),
                             resampling=Resampling.average)
        a = np.ma.filled(a.astype(np.float64), np.nan)
        a[~np.isfinite(a) | (a < -1e30)] = np.nan
        oi, oj = (r0 - row0) // dec, (c0 - col0) // dec
        full[oi:oi + a.shape[0], oj:oj + a.shape[1]] = a
        return full


# ---------------------------------------------------------------------------------------------
def enu_basis(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    U = np.array([math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)])
    E = np.array([-math.sin(lo), math.cos(lo), 0.0])
    N = np.array([-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)])
    return E, N, U


def gnomonic_to_ll(ge, gn, R, E, N, U):
    """Tangent-plane (distance R from centre) coordinates -> lat/lon (deg) of the radial line."""
    px = R * U[0] + ge * E[0] + gn * N[0]
    py = R * U[1] + ge * E[1] + gn * N[1]
    pz = R * U[2] + ge * E[2] + gn * N[2]
    lat = np.degrees(np.arctan2(pz, np.hypot(px, py)))
    lon = np.degrees(np.arctan2(py, px))
    return lat, lon


def apply_deseam(win, col0, row0, dec, pix, seam_file, taper_m):
    """Remove the step of traced N-S seams in-place with a cosine feather of half-width taper_m.
    seam_file: JSON with seams[*] = {row_first, cols[k], steps[k]}: for DTM row row_first + k the seam lies
    between pixel columns cols[k]-1 and cols[k]; steps[k] = z(east) - z(west) [m].
    For a pixel at signed east distance d [m] from the seam: z -= sign(d) * step/2 * 0.5*(1 + cos(pi*|d|/L)), |d| < L.
    Outside the traced row range the correction fades out over L (cosine) in the row direction."""
    with open(seam_file) as f:
        doc = json.load(f)
    nr, nc = win.shape
    rows = row0 + (np.arange(nr) + 0.5) * dec       # pixel-centre rows of window cells
    cols = col0 + (np.arange(nc) + 0.5) * dec
    info = []
    L = float(taper_m)
    for sm in doc.get("seams", []):
        if sm.get("orientation", "NS") != "NS":
            continue
        tc = np.asarray(sm["cols"], dtype=np.float64)
        ts = np.asarray(sm["steps"], dtype=np.float64)
        r_lo, r_hi = float(sm["row_first"]), float(sm["row_first"] + len(tc))
        Lpx = L / pix
        if rows.max() < r_lo - Lpx or rows.min() > r_hi + Lpx:
            continue
        k = np.clip(np.floor(rows - r_lo).astype(int), 0, len(tc) - 1)  # trace row of each window row
        cs = tc[k]
        st = ts[k]
        out_r = np.maximum(r_lo - rows, rows - r_hi).clip(min=0) * pix  # metres beyond traced range
        fade = np.where(out_r < L, 0.5 * (1 + np.cos(np.pi * np.minimum(out_r, L) / L)), 0.0)
        d = (cols[None, :] - cs[:, None]) * pix             # signed east distance [m]
        if np.abs(d).min() > L:
            continue
        w = np.where(np.abs(d) < L, 0.5 * (1 + np.cos(np.pi * np.minimum(np.abs(d), L) / L)), 0.0)
        corr = -np.sign(d) * (st * fade)[:, None] / 2.0 * w
        win += corr
        info.append(dict(name=sm.get("name"), max_abs_correction_m=float(np.nanmax(np.abs(corr))), taper_m=L))
    return info


def load_site(yaml_path, site):
    import yaml
    with open(yaml_path) as f:
        y = yaml.safe_load(f)
    if site not in y:
        raise KeyError(f"site '{site}' not in {yaml_path} (have {list(y)})")
    s = y[site]
    c = s["centre"]
    return dict(lat=float(c["lat_deg"]), lon=float(c["lon_deg"]), href=s.get("href_m"),
                yaw=float(s.get("yaw_deg", 0.0)), size=s.get("size_m"))


def selftest(dtm):
    ok = True
    print(f"[selftest] {dtm.path}")
    print(f"  CRS constants: {json.dumps(dtm.constants(), indent=None)[:400]}...")
    try:
        from pyproj import CRS, Transformer
        crs = CRS.from_wkt(dtm.ds.crs.to_wkt())
        tr = Transformer.from_crs(crs.geodetic_crs, crs, always_xy=True)
        lats = np.array([-8.9734, -8.61, -9.64, -9.0]); lons = np.array([15.5011, 15.15, 15.93, 15.5])
        x, y = dtm.ll2xy(lats, lons)
        xp, yp = tr.transform(lons, lats)
        err = max(np.abs(x - xp).max(), np.abs(y - yp).max())
        print(f"  ll2xy vs pyproj max |diff| = {err:.3e} m")
        ok &= err < 1e-3
    except Exception as e:  # pragma: no cover
        print(f"  pyproj check skipped: {e}")
    lat, lon = dtm.xy2ll(*dtm.ll2xy(-8.9734, 15.5011))
    rt = max(abs(float(lat) + 8.9734), abs(float(lon) - 15.5011))
    print(f"  round trip lat/lon error {rt:.2e} deg")
    ok &= rt < 1e-10
    col, row = dtm.xy2px(*dtm.ll2xy(-8.9734, 15.5011))
    a = dtm.read(int(col) - 2, int(row) - 2, 5, 5)
    from scipy.ndimage import map_coordinates
    h = map_coordinates(a, [[float(row) - (int(row) - 2) - 0.5], [float(col) - (int(col) - 2) - 0.5]], order=1)[0]
    print(f"  LM (LROC 2016: -8.9734, 15.5011): col {float(col):.3f} row {float(row):.3f}, DTM height {h:.2f} m (LROC table: 8 +- 3 m)")
    ok &= abs(h - 8.0) < 3.0
    print("  SELFTEST", "PASSED" if ok else "FAILED")
    return ok


# ---------------------------------------------------------------------------------------------
def extract(args):
    t_start = time.time()
    dtm = EqcDTM(args.dtm)
    fill = None
    if not args.no_fill and args.fill_dtm and Path(args.fill_dtm).exists():
        fill = EqcDTM(args.fill_dtm)
        if (fill.width, fill.height, fill.X0, fill.Y0) != (dtm.width, dtm.height, dtm.X0, dtm.Y0):
            raise ValueError("fill DTM grid differs from main DTM grid")
    R = dtm.R
    lat0, lon0 = args.lat, args.lon
    E, N, U = enu_basis(lat0, lon0)
    sx, sy = (args.size[0], args.size[0]) if len(args.size) == 1 else (args.size[0], args.size[1])
    sp = float(args.spacing)
    nx, ny = int(round(sx / sp)) + 1, int(round(sy / sp)) + 1
    x_min, y_min = -0.5 * (nx - 1) * sp, -0.5 * (ny - 1) * sp
    yaw = math.radians(args.yaw_deg)
    cy, syw = math.cos(yaw), math.sin(yaw)

    def node_en(i, j):  # grid indices -> east/north (m)
        x = x_min + i * sp
        y = y_min + j * sp
        return x * cy - y * syw, x * syw + y * cy

    # --- pixel bounding box of all nodes (sampled on the grid outline + interior lattice) ----
    ii = np.unique(np.r_[np.linspace(0, nx - 1, 65), [0, nx - 1]])
    jj = np.unique(np.r_[np.linspace(0, ny - 1, 65), [0, ny - 1]])
    I, J = np.meshgrid(ii, jj)
    e_, n_ = node_en(I, J)
    la, lo = gnomonic_to_ll(e_, n_, R, E, N, U)
    c_, r_ = dtm.xy2px(*dtm.ll2xy(la, lo))
    pix = abs(dtm.DX)
    # decimation (block average) for coarse output spacings
    dec = 1 if args.no_decimate else max(1, int(math.floor(sp / pix + 1e-9)))
    if dec < 2:
        dec = 1
    # low-pass: Gaussian sigma [m] and/or Butterworth cut-off wavelength [m]
    lp_lambda = float(args.lowpass_lambda) if args.lowpass_lambda else 0.0
    if args.lowpass_sigma == "auto":
        sig_m = 0.0 if (sp <= 2 * pix or lp_lambda > 0) else 0.4 * sp
    else:
        sig_m = float(args.lowpass_sigma)
    sig_px = sig_m / (pix * dec)
    margin = int(math.ceil(4 * sig_px * dec + 6 * dec + 4 + 8 * lp_lambda / pix))
    # vertical-iteration may shift sampling by up to |h|*d/R -> add margin for that too
    dmax = math.hypot(max(abs(x_min), abs(x_min + (nx - 1) * sp)), max(abs(y_min), abs(y_min + (ny - 1) * sp)))
    margin += int(math.ceil(1000.0 * dmax / R / pix)) + 1
    col0 = int(math.floor(c_.min())) - margin
    row0 = int(math.floor(r_.min())) - margin
    ncol = int(math.ceil(c_.max())) + margin - col0
    nrow = int(math.ceil(r_.max())) + margin - row0
    ncol += (-ncol) % dec
    nrow += (-nrow) % dec
    win = dtm.read(col0, row0, ncol, nrow, dec)
    code = np.where(np.isfinite(win), 1, 0).astype(np.uint8)
    if fill is not None and (code == 0).any():
        wf = fill.read(col0, row0, ncol, nrow, dec)
        use = (code == 0) & np.isfinite(wf)
        win[use] = wf[use]
        code[use] = 2
    n_src_nodata = int((code == 0).sum())
    if n_src_nodata == code.size:
        raise RuntimeError("no DTM data in the requested region")
    # optional seam feathering (sub-DTM strip boundaries with 0.3-0.9 m steps)
    deseam_info = []
    if args.deseam:
        deseam_info = apply_deseam(win, col0, row0, dec, pix, args.deseam, float(args.deseam_taper))
    # replace remaining NaN by nearest valid value (keeps splines finite; those nodes masked later)
    if n_src_nodata:
        from scipy.ndimage import distance_transform_edt
        idx = distance_transform_edt(code == 0, return_distances=False, return_indices=True)
        win = win[tuple(idx)]
    from scipy import ndimage as ndi
    if sig_px > 0:
        win = ndi.gaussian_filter(win, sig_px, mode="nearest", truncate=4.0)
    if lp_lambda > 0:
        # Butterworth low-pass in the Fourier domain (reflect-padded):
        #   H(k) = 1 / sqrt(1 + (k * lambda_c)^(2n)),  |H| = 0.707 at wavelength lambda_c
        dpx = pix * dec
        # remove the best-fit plane first (a plane passes the low-pass unchanged) so that the
        # reflect padding does not create roof-shaped creases at the window border
        ii_, jj_ = np.mgrid[0:win.shape[0], 0:win.shape[1]]
        A_ = np.c_[np.ones(win.size), ii_.ravel(), jj_.ravel()]
        pl_, *_ = np.linalg.lstsq(A_, win.ravel(), rcond=None)
        plane = (A_ @ pl_).reshape(win.shape)
        pad = int(math.ceil(4 * lp_lambda / dpx)) + 8
        wp = np.pad(win - plane, pad, mode="reflect")
        ky = np.fft.fftfreq(wp.shape[0], d=dpx)
        kx = np.fft.rfftfreq(wp.shape[1], d=dpx)
        kr = np.hypot(ky[:, None], kx[None, :])
        hb = 1.0 / np.sqrt(1.0 + (kr * lp_lambda) ** (2 * int(args.lowpass_order)))
        win = np.fft.irfft2(np.fft.rfft2(wp) * hb, s=wp.shape)[pad:-pad, pad:-pad] + plane
    order = 3 if args.interp == "cubic" else 1
    coef = ndi.spline_filter(win, order=3, mode="mirror") if order == 3 else win
    # fractional array index of a pixel coordinate (corner convention) in the decimated window
    def px2idx(c, r):
        return (r - row0) / dec - 0.5, (c - col0) / dec - 0.5

    all_orig = bool((code == 1).all())

    def sample(e, n):
        la, lo = gnomonic_to_ll(e, n, R, E, N, U)
        c, r = dtm.xy2px(*dtm.ll2xy(la, lo))
        fi, fj = px2idx(c, r)
        h = ndi.map_coordinates(coef, [fi, fj], order=order, mode="mirror", prefilter=False)
        if all_orig:  # fast path: every source pixel in the window is original DTM data
            return h, np.ones(h.shape, dtype=np.uint8)
        m = ndi.map_coordinates(code.astype(np.float32), [fi, fj], order=0, mode="nearest")
        ok_orig = ndi.map_coordinates((code == 1).astype(np.float32), [fi, fj], order=1, mode="nearest") > 0.999
        mm = np.where(ok_orig, 1, np.where(m == 0, 0, 2)).astype(np.uint8)
        # nodes whose interpolation support touches no-data -> 0
        nod = ndi.map_coordinates((code == 0).astype(np.float32), [fi, fj], order=1, mode="nearest") > 0
        mm[nod] = 0
        return h, mm

    # --- reference height ----------------------------------------------------------------------
    if args.href == "auto":
        hb = float(args.href_box) if args.href_box else min(sx, sy)
        nh = int(np.clip(hb / 0.5, 100, 400))
        g = -hb / 2 + (np.arange(nh) + 0.5) * hb / nh   # cell-centred (midpoint rule) -> unbiased area mean
        Gx, Gy = np.meshgrid(g, g)
        hh, mm = sample(Gx * cy - Gy * syw, Gx * syw + Gy * cy)  # square aligned with the grid axes
        href = float(np.mean(hh[mm > 0])) if (mm > 0).any() else float(np.nanmean(hh))
        href_method = (f"area mean of the DTM height (after the same low-pass/interp, midpoint rule {nh}x{nh}) "
                       f"over a {hb:g} m square at the centre (x/y axes of the grid)")
    else:
        href = float(args.href)
        href_method = "given"
    curv = not args.no_curvature

    # --- vertical-sampling iteration: ONE decision for the whole extraction (CR-07) ----------------
    # Sampling on the radial line instead of the local vertical shifts a node by about |h| * d / R.
    # The decision whether to apply the fixed-point iteration used to be taken per row chunk, so the
    # output depended on --chunk-nodes (74 mm steps on the production far field with 100-row chunks).
    # It is now taken once from a chunk-independent bound: the largest |h| of the processed DTM window
    # over the pixel box of all grid nodes (+2 px of spline support) times the largest node distance.
    fi_box, fj_box = px2idx(np.array([c_.min(), c_.max()]), np.array([r_.min(), r_.max()]))
    r_lo = max(int(math.floor(min(fi_box))) - 2, 0)
    r_hi = min(int(math.ceil(max(fi_box))) + 3, win.shape[0])
    c_lo = max(int(math.floor(min(fj_box))) - 2, 0)
    c_hi = min(int(math.ceil(max(fj_box))) + 3, win.shape[1])
    h_abs_max_est = float(np.abs(win[r_lo:r_hi, c_lo:c_hi]).max())
    shift_tol = max(0.05 * sp, 0.005)
    shift_est = h_abs_max_est * dmax / R
    mode = getattr(args, "vertical_iteration", "auto") or "auto"
    if not curv or mode == "never":
        iterate = False
    elif mode == "always":
        iterate = True
    else:
        iterate = shift_est > shift_tol
    iteration_decision = dict(mode=mode, h_abs_max_est_m=h_abs_max_est, d_max_m=dmax, shift_est_m=shift_est,
                              shift_tol_m=shift_tol, iterate=bool(iterate),
                              rule="iterate if max|h| (DTM window over the grid's pixel box) * max node distance "
                                   "/ R > max(0.05 * spacing, 5 mm); decided once per extraction")

    # --- main loop over row chunks ---------------------------------------------------------------
    out_dir = Path(args.out).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    zpath = str(args.out) + ".npy"
    Z = np.lib.format.open_memmap(zpath, mode="w+", dtype=np.float32, shape=(ny, nx))
    M = np.empty((ny, nx), dtype=np.uint8)
    max_shift = 0.0
    chunk = max(1, int(args.chunk_nodes) // nx)
    iv = np.arange(nx, dtype=np.float64)
    for j0 in range(0, ny, chunk):
        j1 = min(ny, j0 + chunk)
        Jg, Ig = np.meshgrid(np.arange(j0, j1, dtype=np.float64), iv, indexing="ij")
        e, n = node_en(Ig, Jg)
        h, mm = sample(e, n)
        d2 = e * e + n * n
        if curv:
            # pointwise shift (its maximum over all nodes does not depend on the chunking)
            max_shift = max(max_shift, float((np.abs(h) * np.sqrt(d2)).max() / R))
            if iterate:
                # one fixed-point iteration: sample on the local vertical through (e, n)
                s = R / (R + h - d2 / (2 * R))
                e2, n2 = e * s, n * s
                h, mm = sample(e2, n2)
                d2 = e2 * e2 + n2 * n2
            z = (R + h) * R / np.sqrt(R * R + d2) - (R + href)
        else:
            z = h - href
        z = z.astype(np.float32)
        z[mm == 0] = np.nan
        rows = slice(j0, j1)
        if args.row0 == "north":
            rows = slice(ny - j1, ny - j0)
            z = z[::-1]
            mm = mm[::-1]
        Z[rows] = z
        M[rows] = mm
    Z.flush()
    zs = np.asarray(Z)
    finite = np.isfinite(zs)
    # area-weighted (trapezoid) mean: the plain node mean over-weights the edge rows/columns
    wx = np.ones(nx); wx[[0, -1]] = 0.5
    wy = np.ones(ny); wy[[0, -1]] = 0.5
    zw = np.where(finite, zs, 0.0)
    wsum = float((finite * wy[:, None] * wx[None, :]).sum())
    z_area_mean = float((zw * wy[:, None]).sum(axis=0) @ wx / wsum) if wsum > 0 else None
    mpath = None
    if args.always_mask or (M != 1).any():
        mpath = str(args.out) + "_mask.npy"
        np.save(mpath, M)
    # centre info
    cx, cy_ = dtm.ll2xy(lat0, lon0)
    ccol, crow = dtm.xy2px(cx, cy_)
    hc, _ = sample(np.array([0.0]), np.array([0.0]))
    meta = dict(
        format="lunar_world_gen.dtm_extract/1", tool_version=__version__,
        created_utc=_dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        command=" ".join([os.path.basename(sys.argv[0])] + sys.argv[1:]),
        source=dict(dtm=str(Path(args.dtm).resolve()), dtm_bytes=os.path.getsize(args.dtm),
                    fill_dtm=None if fill is None else str(Path(args.fill_dtm).resolve()),
                    crs_proj4=dtm.ds.crs.to_proj4(), pixel_size_m=pix,
                    height_reference=f"metres above a sphere of radius {R:.0f} m"),
        conversion_constants=dtm.constants(),
        centre=dict(lat_deg=lat0, lon_deg=lon0, proj_x_m=float(cx), proj_y_m=float(cy_),
                    pixel_col=float(ccol), pixel_row=float(crow),
                    pixel_convention="GDAL corner: pixel (c, r) spans [c, c+1) x [r, r+1); array index = floor",
                    h_abs_m=float(hc[0])),
        frame=dict(type=("local tangent-plane ENU at centre (gnomonic), x=east, y=north, z=up" if args.yaw_deg == 0 else
                         "local tangent-plane ENU at centre (gnomonic); grid x/y = east/north rotated CCW by yaw_deg; z=up"),
                   yaw_deg=args.yaw_deg, href_m=href, href_method=href_method,
                   z_definition=("z = (R+h)*R/sqrt(R^2+d^2) - (R+href)  (curvature included)" if curv
                                 else "z = h - href (no curvature)"),
                   vertical_iteration=bool(iterate and curv), max_radial_vs_vertical_shift_m=max_shift,
                   vertical_iteration_decision=iteration_decision,
                   moon_radius_m=R, ecef_axes=dict(E=E.tolist(), N=N.tolist(), U=U.tolist())),
        grid=dict(nx=nx, ny=ny, spacing_m=sp, x_min=x_min, y_min=y_min,
                  x_max=x_min + (nx - 1) * sp, y_max=y_min + (ny - 1) * sp, row0=args.row0,
                  layout=("z[j, i] at x = x_min + i*spacing, y = y_min + j*spacing; row 0 = SOUTH edge"
                          if args.row0 == "south" else
                          "z[j, i] at x = x_min + i*spacing, y = y_max - j*spacing; row 0 = NORTH edge"),
                  dtype="float32", units="m"),
        processing=dict(interp=args.interp, lowpass_sigma_m=sig_m,
                        lowpass_butterworth=(dict(lambda_c_m=lp_lambda, order=int(args.lowpass_order),
                                                  transfer="H(k) = 1/sqrt(1 + (k*lambda_c)^(2*order)), k in cycles/m")
                                             if lp_lambda > 0 else None),
                        decimation_block=dec, deseam=deseam_info or None,
                        read_window=dict(col0=col0, row0=row0, ncol=ncol, nrow=nrow),
                        source_nodata_px_in_window=n_src_nodata, curvature=curv,
                        elapsed_s=round(time.time() - t_start, 2)),
        stats=dict(z_min=float(np.nanmin(zs)) if finite.any() else None,
                   z_max=float(np.nanmax(zs)) if finite.any() else None,
                   z_mean=float(np.nanmean(zs)) if finite.any() else None, z_area_mean=z_area_mean,
                   z_std=float(np.nanstd(zs)) if finite.any() else None,
                   n_nodes=int(zs.size), n_nan=int((~finite).sum()),
                   frac_from_filled=float((M == 2).mean()), frac_nodata=float((M == 0).mean())),
        files=dict(z=os.path.basename(zpath), mask=None if mpath is None else os.path.basename(mpath)),
    )
    with open(str(args.out) + ".json", "w") as f:
        json.dump(meta, f, indent=1)
    if args.png:
        quicklook(zs, sp, str(args.out) + ".png", args.row0, meta)
    print(json.dumps(dict(out=zpath, nx=nx, ny=ny, href=href, stats=meta["stats"],
                          elapsed_s=meta["processing"]["elapsed_s"]), indent=1))
    return meta


def quicklook(z, sp, path, row0, meta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    step = max(1, int(max(z.shape) // 2000))
    zz = np.asarray(z[::step, ::step], dtype=np.float64)
    if row0 == "south":
        zz = zz[::-1]
    zf = np.where(np.isfinite(zz), zz, np.nanmean(zz))
    gy, gx = np.gradient(zf, sp * step)
    nx_, ny_ = -gx, gy  # image rows go south
    az, alt = math.radians(90.0), math.radians(20.0)
    l = np.array([math.cos(alt) * math.sin(az), math.cos(alt) * math.cos(az), math.sin(alt)])
    hs = (nx_ * l[0] + ny_ * l[1] + l[2]) / np.sqrt(nx_ ** 2 + ny_ ** 2 + 1)
    g = meta["grid"]
    ext = [g["x_min"], g["x_max"], g["y_min"], g["y_max"]]
    fig, ax = plt.subplots(1, 2, figsize=(14, 6.5))
    ax[0].imshow(np.clip(hs, 0, 1), cmap="gray", extent=ext)
    ax[0].set_title("hillshade (sun E, 20 deg)")
    im = ax[1].imshow(zz, cmap="terrain", extent=ext)
    plt.colorbar(im, ax=ax[1], shrink=0.8, label="z [m]")
    ax[1].set_title(f"z (href {meta['frame']['href_m']:.2f} m)")
    for a in ax:
        a.set_xlabel("x east [m]")
        a.set_ylabel("y north [m]")
    plt.tight_layout()
    plt.savefig(path, dpi=80)
    plt.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--dtm", default=str(DEFAULT_DTM), help="main DTM GeoTIFF (default: %(default)s)")
    ap.add_argument("--fill-dtm", default=str(DEFAULT_FILL), help="hole-filled DTM used where the main one has nodata")
    ap.add_argument("--no-fill", action="store_true", help="do not use the filled DTM")
    ap.add_argument("--lat", type=float, help="centre latitude [deg, planetocentric, ME frame]")
    ap.add_argument("--lon", type=float, help="centre longitude [deg east]")
    ap.add_argument("--site", help="take centre (and href, yaw) from this key of --site-yaml (e.g. core, rocky_alternative)")
    ap.add_argument("--site-yaml", default=str(DEFAULT_SITE_YAML))
    ap.add_argument("--size", type=float, nargs="+", help="grid extent [m]: W or W H (default: site size or 100)")
    ap.add_argument("--spacing", type=float, default=1.0, help="node spacing [m] (default 1.0)")
    ap.add_argument("--yaw-deg", type=float, default=None, help="grid rotation CCW from east (default 0 / site value)")
    ap.add_argument("--href", default=None, help="reference height [m above sphere] or 'auto' (default: site value or auto)")
    ap.add_argument("--href-box", type=float, default=None, help="square size [m] used for href=auto (default: grid size)")
    ap.add_argument("--lowpass-sigma", default="auto",
                    help="Gaussian low-pass sigma [m] applied on the DTM before interpolation; 'auto' = 0 for "
                         "spacing <= 2 px, else 0.4*spacing (anti-alias). Core recommendation: 1.9")
    ap.add_argument("--lowpass-lambda", type=float, default=None,
                    help="Butterworth low-pass cut-off wavelength [m] (|H| = 0.707 there), applied on the DTM grid "
                         "in the Fourier domain. Core recommendation: 10 (with --lowpass-order 4)")
    ap.add_argument("--lowpass-order", type=int, default=4, help="Butterworth order n (default 4)")
    ap.add_argument("--deseam", default=None,
                    help="feather traced sub-DTM seams: path to a seam JSON, or 'default' = "
                         "docs/research/dtm_seams_apollo16.json (valid within +-3 km of the LM latitude)")
    ap.add_argument("--deseam-taper", type=float, default=300.0, help="seam feather half-width [m] (default 300)")
    ap.add_argument("--interp", choices=["cubic", "linear"], default="cubic")
    ap.add_argument("--no-curvature", action="store_true", help="do not apply the Moon-curvature drop")
    ap.add_argument("--no-decimate", action="store_true", help="never block-average the source for coarse spacings")
    ap.add_argument("--row0", choices=["south", "north"], default="south")
    ap.add_argument("--chunk-nodes", type=int, default=2_000_000,
                    help="grid nodes per processing chunk (memory only: the output does not depend on it)")
    ap.add_argument("--vertical-iteration", choices=["auto", "always", "never"], default="auto",
                    help="sample on the local vertical (one fixed-point iteration): auto = when the estimated "
                         "radial-vs-vertical shift exceeds max(0.05*spacing, 5 mm), decided once per extraction")
    ap.add_argument("--always-mask", action="store_true")
    ap.add_argument("--png", action="store_true", help="write a hillshade quick-look PNG")
    ap.add_argument("--out", help="output prefix (writes PREFIX.npy, PREFIX.json, ...)")
    ap.add_argument("--selftest", action="store_true", help="check coordinate conversions on the DTM and exit")
    args = ap.parse_args(argv)
    if args.selftest:
        return 0 if selftest(EqcDTM(args.dtm)) else 1
    if args.site:
        s = load_site(args.site_yaml, args.site)
        args.lat = s["lat"] if args.lat is None else args.lat
        args.lon = s["lon"] if args.lon is None else args.lon
        if args.href is None and s["href"] is not None:
            args.href = s["href"]
        if args.yaw_deg is None:
            args.yaw_deg = s["yaw"]
        if args.size is None and s["size"] is not None:
            args.size = [float(v) for v in (s["size"] if isinstance(s["size"], list) else [s["size"]])]
    if args.lat is None or args.lon is None or not args.out:
        ap.error("--lat/--lon (or --site) and --out are required")
    if args.deseam == "default":
        args.deseam = str(DEFAULT_SEAMS)
    args.size = args.size or [100.0]
    args.yaw_deg = 0.0 if args.yaw_deg is None else args.yaw_deg
    args.href = "auto" if args.href is None else args.href
    extract(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
