"""Validation report: ``world_dir/reports/validation_report.md`` from the JSON results.

Inputs (all optional; found in the reports dir by their ``"kind"`` field, or passed explicitly):

* ``static_checks.json``   - ``validate/static_checks.py``  (kind ``static_checks``)
* ``bench_world*.json``    - ``tools/bench_world.py``       (kind ``bench_world``)
* ``lidar_checks*.json``   - ``validate/lidar_checks.py``   (kind ``lidar_checks``)
* ``robot_tests*.json``    - ``tools/run_robot_tests.py``   (kind ``robot_tests``)
* camera images            - any other ``*.png`` / ``*.jpg`` in the reports dir (or ``images/``,
  ``camera/``), shown in the "Visual" section for review by eye.

Figures (PNG, written next to the report): deviogram vs target, crater and rock cumulative
SFDs vs model, visual-collision dz histogram, LiDAR returns per range bin, rock detection
probability vs range. Colours follow the dataviz reference palette (measured = blue
``#2a78d6``, target/model = orange ``#eb6834``); every figure has a table view in the report.

Run:  python -m lunar_world_gen.validate.report [--config CFG] [--reports DIR] [--json FILE ...]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence

# palette (dataviz reference instance, light mode; slots 1-3 validated)
C_MEAS = "#2a78d6"
C_MODEL = "#eb6834"
C_THIRD = "#1baf7a"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SURFACE = "#fcfcfb"

IMAGE_EXT = (".png", ".jpg", ".jpeg")


# ----------------------------------------------------------------------------------------------
# small formatting helpers

def fmt(v: Any, nd: int = 3, unit: str = "") -> str:
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int,)) and not isinstance(v, bool):
        return f"{v}{unit}"
    if isinstance(v, float):
        if not math.isfinite(v):
            return "-"
        a = abs(v)
        if a != 0 and (a < 10 ** -nd or a >= 1e6):
            return f"{v:.{max(nd - 1, 1)}e}{unit}"
        return f"{v:.{nd}f}{unit}"
    return f"{v}{unit}"


def verdict(p: Any) -> str:
    if p is True or p == "pass":
        return "PASS"
    if p is False or p == "fail":
        return "FAIL"
    if p == "error":
        return "ERROR"
    return "SKIPPED" if p in (None, "skipped") else str(p).upper()


def table(header: Sequence[str], rows: Iterable[Sequence[Any]]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for r in rows:
        out.append("| " + " | ".join(str(c).replace("|", "/") for c in r) + " |")
    return "\n".join(out)


# ----------------------------------------------------------------------------------------------
# figures

def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK, "axes.titlesize": 11,
        "axes.labelsize": 9.5, "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5, "grid.color": GRID, "grid.linewidth": 0.6, "axes.grid": True,
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False, "legend.fontsize": 8.5,
        "font.family": "sans-serif", "lines.linewidth": 1.6, "lines.markersize": 6, "axes.axisbelow": True,
    })
    return plt


def _save(fig, path: Path) -> str:
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    import matplotlib.pyplot as plt
    plt.close(fig)
    return path.name


def fig_deviogram(rough: dict, path: Path) -> str | None:
    rows = rough.get("rows") or []
    rows = [r for r in rows if r.get("nu_m") is not None]
    if not rows:
        return None
    plt = _mpl()
    L = [r["L_used_m"] for r in rows]
    nu = [r["nu_m"] for r in rows]
    tgt = [r["target_nu_m"] for r in rows]
    tol = rough.get("target", {}).get("tolerance_frac", 0.25)
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    import numpy as np
    Lf = np.geomspace(min(L) * 0.8, max(L) * 1.25, 100)
    nu1, H = rough.get("target", {}).get("nu1_m", 0.069), rough.get("target", {}).get("hurst", 0.8)
    ax.fill_between(Lf, nu1 * Lf ** H * (1 - tol), nu1 * Lf ** H * (1 + tol), color=C_MODEL, alpha=0.12, lw=0,
                    label=f"target +-{int(tol * 100)} %")
    ax.plot(Lf, nu1 * Lf ** H, color=C_MODEL, lw=1.6, label=f"target {nu1:g} L^{H:g}")
    ax.plot(L, nu, color=C_MEAS, lw=1.6, marker="o", label="measured (core H, no rocks)")
    for r in rows:
        if r.get("checked"):
            ax.annotate("PASS" if r.get("pass") else "FAIL", (r["L_used_m"], r["nu_m"]), textcoords="offset points",
                        xytext=(6, -12), fontsize=7.5, color=INK2)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("lag L [m]")
    ax.set_ylabel("nu(L) [m]  (plane-detrended, directional)")
    ax.set_title("Deviogram of the core height field")
    ax.legend(loc="upper left")
    return _save(fig, path)


def fig_sfd(curve: dict, obs_key: str, exp_key: str, title: str, path: Path, area: float | None = None) -> str | None:
    if not curve or not curve.get("D_m"):
        return None
    import numpy as np
    plt = _mpl()
    D = np.asarray(curve["D_m"], float)
    n = np.asarray(curve[obs_key], float)
    e = np.asarray(curve[exp_key], float)
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ok = e > 0
    ax.plot(D[ok], e[ok], color=C_MODEL, lw=1.6, label="model")
    m = n > 0
    err = np.sqrt(np.maximum(n[m], 1.0))
    ax.errorbar(D[m], n[m], yerr=[np.minimum(err, n[m] * 0.999), err], color=C_MEAS, lw=1.6, marker="o",
                capsize=0, label="observed (+-1 sigma Poisson)")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("diameter D [m]")
    ax.set_ylabel("cumulative count N(>=D)" + (f" in {area:g} m^2" if area else ""))
    ax.set_title(title)
    ax.legend(loc="upper right")
    return _save(fig, path)


def fig_dz(vc: dict, path: Path) -> str | None:
    h = vc.get("hist")
    if not h:
        return None
    import numpy as np
    plt = _mpl()
    edges = np.asarray(h["edges_m"]) * 1000
    cnt = np.asarray(h["counts"], float)
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.bar(edges[:-1], cnt, width=np.diff(edges) * 0.9, align="edge", color=C_MEAS, label="dz samples")
    th = vc.get("thresholds", {})
    t = th.get("median_target_m", -0.02) * 1000
    tol = th.get("median_tol_m", 0.003) * 1000
    ax.axvspan(t - tol, t + tol, color=C_MODEL, alpha=0.15, lw=0, label=f"target median {t:g} +-{tol:g} mm")
    if vc.get("median_m") is not None:
        ax.axvline(vc["median_m"] * 1000, color=INK, lw=1.2, label=f"median {vc['median_m'] * 1000:.1f} mm")
    ax.set_xlabel("dz = collision - visual [mm]")
    ax.set_ylabel("samples per 1 mm bin")
    ax.set_title("Visual vs collision ground (rock-free core)")
    ax.legend(loc="upper left")
    return _save(fig, path)


def fig_lidar_range(rh: dict, path: Path) -> str | None:
    if not rh or not rh.get("edges_m"):
        return None
    import numpy as np
    plt = _mpl()
    edges = np.asarray(rh["edges_m"])
    per = np.asarray(rh["per_scan"], float)
    labels = [f"{edges[i]:g}-{edges[i + 1]:g}" for i in range(len(per))]
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    x = np.arange(len(per))
    ax.bar(x, per, color=C_MEAS, width=0.8)
    for xi, v in zip(x, per):
        ax.annotate(f"{v:.0f}", (xi, v), textcoords="offset points", xytext=(0, 3), ha="center", fontsize=7.5,
                    color=INK2)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel("range bin [m]")
    ax.set_ylabel("returns per scan")
    ax.set_title(f"LiDAR returns per range bin ({rh.get('n_scans')} scans)")
    ax.grid(axis="x", visible=False)
    return _save(fig, path)


def fig_rock_detection(rd: dict, path: Path) -> str | None:
    rows = [r for r in (rd.get("rows") or []) if r.get("p_detect") is not None]
    if not rows:
        return None
    import numpy as np
    plt = _mpl()
    groups = [("D < 5 cm", 0.0, 0.05, C_MEAS), ("5-20 cm", 0.05, 0.20, C_MODEL), (">= 20 cm", 0.20, 99.0, C_THIRD)]
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    any_line = False
    for name, lo, hi, col in groups:
        sel = [r for r in rows if lo <= r["D_m"][0] < hi]
        if not sel:
            continue
        by_r: dict = {}
        for r in sel:
            k = tuple(r["range_m"])
            n, d = by_r.get(k, (0, 0.0))
            by_r[k] = (n + r["n_visible"], d + r["p_detect"] * r["n_visible"])
        ks = sorted(by_r)
        xm = [math.sqrt(max(k[0], 0.5) * k[1]) for k in ks]
        p = [by_r[k][1] / by_r[k][0] if by_r[k][0] else np.nan for k in ks]
        ax.plot(xm, p, color=col, lw=1.6, marker="o", label=name)
        any_line = True
    if not any_line:
        plt.close(fig)
        return None
    ax.set_xscale("log")
    ax.set_ylim(-0.03, 1.05)
    ax.set_xlabel("horizontal range [m]")
    ax.set_ylabel("detection probability (>= 1 return)")
    ax.set_title("Rock detection vs size and range")
    ax.legend(loc="lower left")
    return _save(fig, path)


# ----------------------------------------------------------------------------------------------
# sections

def _static_section(st: dict, figdir: Path, figures: bool) -> tuple[list[str], list[list]]:
    md: list[str] = []
    summ: list[list] = []
    ch = st.get("checks", {})
    md.append("## 1. Static checks\n")
    md.append(f"Source: `static_checks.json` ({st.get('created')}, config hash `{st.get('config_hash')}`).\n")
    r = ch.get("roughness")
    if r:
        md.append(f"### Roughness - {verdict(r.get('status'))}\n")
        if r.get("rows"):
            md.append(f"Grid: `{r.get('source')}`. {r.get('definition', '')}\n")
            if r.get("acceptance"):
                md.append(f"Acceptance: {r['acceptance']}.\n")
            md.append(table(["L [m]", "nu [m]", "target [m]", "ratio", "nu B [m]", "nu / nu B", "RMS slope [deg]",
                             "target slope [deg]", "check"],
                            [[fmt(x["L_m"], 2), fmt(x["nu_m"], 4), fmt(x["target_nu_m"], 4), fmt(x["ratio"], 3),
                              fmt(x.get("nu_B_m"), 4), fmt(x.get("ratio_to_B"), 3),
                              fmt(x["rms_slope_deg"], 2), fmt(x["target_rms_slope_deg"], 2),
                              verdict(x.get("pass")) if x.get("checked") else "-"] for x in r["rows"]]))
            if r.get("rows_synthetic"):
                md.append("\nSynthetic part H0 - B (craters + fBm):\n")
                md.append(table(["L [m]", "nu [m]", "target [m]", "ratio", "check"],
                                [[fmt(x["L_m"], 2), fmt(x["nu_m"], 4), fmt(x["target_nu_m"], 4), fmt(x["ratio"], 3),
                                  verdict(x.get("pass")) if x.get("checked") else "-"] for x in r["rows_synthetic"]]))
            elif r.get("site_aware") is False:
                md.append(f"\nSite-aware parts not evaluated: {r.get('reason', 'no DTM base grid')}.\n")
            fit = r.get("fit", {})
            md.append(f"\nLog-log fit: H = {fmt(fit.get('H'), 3)}, nu1 = {fmt(fit.get('nu1_m'), 4)} m.")
            psd = r.get("psd", {}).get("fit_0p1_20")
            if psd:
                md.append(f" PSD fit (0.1-20 cycles/m): slope {fmt(psd.get('beta'), 2)} (target "
                          f"{fmt(r['psd'].get('target_beta'), 2)}), S0 {fmt(psd.get('S0'), 3)} m^4 (synthesis target "
                          f"{fmt(r['psd'].get('target_S0_m4'), 3)}; the composite includes the DTM and craters).")
            md.append("\n")
            if figures:
                f = fig_deviogram(r, figdir / "fig_deviogram.png")
                if f:
                    md.append(f"![deviogram]({f})\n")
            sd = [x for x in r.get("slope_distribution", []) if "median_deg" in x]
            if sd:
                md.append(table(["baseline [m]", "median [deg]", "mean", "p90", "p99", ">10 deg", ">15 deg"],
                                [[fmt(x["baseline_m"], 2), fmt(x["median_deg"], 2), fmt(x["mean_deg"], 2),
                                  fmt(x["p90_deg"], 2), fmt(x["p99_deg"], 2), fmt(x["frac_gt_10deg"], 4),
                                  fmt(x["frac_gt_15deg"], 4)] for x in sd]))
                md.append("")
            parts = []
            for x in r["rows"]:
                if x.get("checked"):
                    if x.get("ratio_to_B") is not None and x["L_m"] >= 10.0 - 1e-9:
                        parts.append(f"{x['L_m']:g} m: H0/B {fmt(x.get('ratio_to_B'), 2)}")
                    else:
                        parts.append(f"{x['L_m']:g} m: {fmt(x.get('ratio'), 2)}")
            for x in r.get("rows_synthetic") or []:
                if x.get("checked"):
                    parts.append(f"{x['L_m']:g} m (H0-B): {fmt(x.get('ratio'), 2)}")
            summ.append(["static", "deviogram (site-aware)", verdict(r.get("status")), "nu/target " + ", ".join(parts),
                         "H0 0.75-1.25 at 0.01-0.3 m; H0-B 0.75-1.25 at 1, 3 m; H0 <= 1.15 B at 10 m"])
        else:
            md.append(f"{r.get('reason', '')}\n")
            summ.append(["static", "deviogram (site-aware)", verdict(r.get("status")), r.get("reason", ""), ""])
    for key, title, rows_key, rule in (("craters", "Craters", "rows", "+-20 % at 0.2/0.5/1/2 m"),
                                       ("rocks", "Rocks (core)", "rows_core", "Poisson +-3 sigma at 2/5/10/20 cm")):
        c = ch.get(key)
        if not c:
            continue
        md.append(f"### {title} - {verdict(c.get('status'))}\n")
        rows = c.get(rows_key) or []
        if rows:
            md.append(table(["D >= [m]", "observed", "expected", "ratio", "z (Poisson)", "accept range", "check"],
                            [[fmt(x["D_m"], 2), x["observed"], fmt(x["expected"], 1), fmt(x["ratio"], 3),
                              fmt(x["z_poisson"], 2), f"{x['poisson_range'][0]}-{x['poisson_range'][1]}"
                              if key == "rocks" else "+-20 %", verdict(x.get("pass"))] for x in rows]))
            md.append("")
            if key == "rocks" and c.get("rows_ring"):
                md.append("Ring rocks (informational):\n")
                md.append(table(["D >= [m]", "observed", "expected", "ratio", "check"],
                                [[fmt(x["D_m"], 2), x["observed"], fmt(x["expected"], 1), fmt(x["ratio"], 3),
                                  verdict(x.get("pass"))] for x in c["rows_ring"]]))
                md.append("")
            if key == "rocks" and c.get("sanity"):
                s = c["sanity"]
                md.append("Catalog sanity: " + ", ".join(f"{k} = {v}" for k, v in s.items()) + "\n")
            if figures:
                if key == "craters":
                    f = fig_sfd(c.get("curve"), "observed", "expected", "Craters centred in the core",
                                figdir / "fig_sfd_craters.png", c.get("area_m2"))
                else:
                    f = fig_sfd(c.get("curve"), "observed_core", "expected_core", "Rocks in the core",
                                figdir / "fig_sfd_rocks.png", c.get("area_core_m2"))
                if f:
                    md.append(f"![{key} SFD]({f})\n")
            nums = ", ".join(f"{x['D_m']:g} m: {x['observed']}/{x['expected']:.0f}" for x in rows)
            summ.append(["static", f"{key} N(>=D)", verdict(c.get("status")), nums, rule])
        else:
            md.append(f"{c.get('reason', '')}\n")
            summ.append(["static", f"{key} N(>=D)", verdict(c.get("status")), c.get("reason", ""), rule])
    vc = ch.get("visual_vs_collision")
    if vc:
        md.append(f"### Visual vs collision - {verdict(vc.get('status'))}\n")
        if vc.get("n"):
            md.append(f"Source: {vc.get('source', {}).get('kind')}; {vc.get('n')} rock-free samples. "
                      f"{vc.get('definition', '')}\n")
            md.append(table(["median [mm]", "RMS about median [mm]", "p99 abs dev [mm]", "max abs dev [mm]",
                             "p1 / p99 [mm]"],
                            [[fmt(vc["median_m"] * 1000, 2), fmt(vc["rms_about_median_m"] * 1000, 2),
                              fmt(vc["p99_abs_dev_m"] * 1000, 2), fmt(vc["max_abs_dev_m"] * 1000, 2),
                              f"{vc['p1_m'] * 1000:.1f} / {vc['p99_m'] * 1000:.1f}"]]))
            md.append("")
            if figures:
                f = fig_dz(vc, figdir / "fig_dz_hist.png")
                if f:
                    md.append(f"![dz histogram]({f})\n")
            summ.append(["static", "visual vs collision dz", verdict(vc.get("status")),
                         f"median {vc['median_m'] * 1000:.1f} mm, RMS {vc['rms_about_median_m'] * 1000:.1f} mm, "
                         f"p99 {vc['p99_abs_dev_m'] * 1000:.1f} mm", "-20+-3 / <=10 / <=30 mm"])
        else:
            md.append(f"{vc.get('reason', '')}\n")
            summ.append(["static", "visual vs collision dz", verdict(vc.get("status")), vc.get("reason", ""), ""])
    g = ch.get("glb")
    if g:
        md.append(f"### GLB files - {verdict(g.get('status'))}\n")
        s = g.get("summary") or {}
        if s.get("by_kind"):
            md.append(table(["kind", "files", "triangles", "vertices", "MB"],
                            [[k, v["files"], f"{v['triangles']:,}", f"{v['vertices']:,}", fmt(v["bytes"] / 1e6, 1)]
                             for k, v in sorted(s["by_kind"].items())]))
            md.append(f"\nTotal {s.get('total_triangles', 0):,} triangles, {fmt(s.get('total_bytes', 0) / 1e6, 1)} MB; "
                      f"ground +z normals {fmt(s.get('ground_up_normals_frac_all'), 5)} (min per file "
                      f"{fmt(s.get('ground_up_normals_frac_min_file'), 5)}); trimesh failures "
                      f"{s.get('trimesh_failures')}.\n")
        for p in (g.get("problems") or [])[:15]:
            md.append(f"- {p}")
        md.append("")
        summ.append(["static", "GLBs load, Z-up, +z normals", verdict(g.get("status")),
                     g.get("reason") or f"{s.get('files', 0)} files, {g.get('n_problems', 0)} problems",
                     ">= 99.9 % +z normals"])
    sti = ch.get("stitching")
    if sti:
        md.append(f"### Stitching - {verdict(sti.get('status'))}\n")
        tot = sti.get("totals")
        if tot:
            md.append(table(list(tot.keys()), [list(tot.values())]))
            md.append(f"\nPieces: {sti.get('n_pieces_visual')} visual (expected {sti.get('expected_visual_pieces')}), "
                      f"{sti.get('n_pieces_collision')} collision. Pairs by type: {sti.get('pairs_by_type')}.\n")
            for b in (sti.get("failing_pairs") or [])[:10]:
                md.append(f"- {b['a']} / {b['b']}: only_a {b['only_a']}, only_b {b['only_b']}, "
                          f"T-junctions {b['t_junctions']}, height mismatches {b['height_mismatch']}, "
                          f"uncovered {b['uncovered']}")
            md.append("")
            summ.append(["static", "stitching", verdict(sti.get("status")),
                         f"{tot.get('pairs')} pairs, {tot.get('mismatches')} mismatches, "
                         f"{tot.get('t_junctions')} T-junctions", "0 mismatches"])
        else:
            md.append(f"{sti.get('reason', '')}\n")
            summ.append(["static", "stitching", verdict(sti.get("status")), sti.get("reason", ""), "0 mismatches"])
    for key, title in (("sdf", "SDF references"), ("world_info", "world_info.yaml")):
        c = ch.get(key)
        if c:
            md.append(f"### {title} - {verdict(c.get('status'))}\n")
            if key == "sdf" and c.get("sdfs"):
                md.append(table(["SDF", "refs", "missing", "Sensors system in world"],
                                [[n, v.get("n_refs"), v.get("n_missing"), fmt(v.get("world_has_sensors_system"))]
                                 for n, v in c["sdfs"].items()]))
                md.append("")
            elif key == "world_info" and c.get("sites"):
                md.append(table(["site", "x", "y", "z", "yaw", "description"],
                                [[n, fmt(s["x"], 2), fmt(s["y"], 2), fmt(s["z"], 3), fmt(s["yaw"], 3),
                                  s.get("description", "")] for n, s in c["sites"].items()]))
                md.append("")
            else:
                md.append(f"{c.get('reason', '')}\n")
            summ.append(["static", title, verdict(c.get("status")), c.get("reason", "") or "", ""])
    return md, summ


def _bench_section(benches: list[dict]) -> tuple[list[str], list[list]]:
    md = ["## 2. Gazebo, world only\n"]
    summ = []
    if not benches:
        md.append("No `bench_world` result.\n")
        return md, summ
    rows = []
    for b in benches:
        c = b.get("checks", {})
        label = f" [{b['label']}]" if b.get("label") else ""
        rows.append([Path(b.get("world_sdf", "?")).name + label, b.get("lidar_preset"), b.get("status"),
                     fmt(b.get("load", {}).get("first_lidar_s"), 2), fmt(b.get("window", {}).get("rtf_mean"), 4),
                     fmt(b.get("window", {}).get("wall_s"), 1), fmt(b.get("cpu", {}).get("server_pct"), 1),
                     fmt(b.get("mem", {}).get("hwm_mb"), 0), fmt((b.get("gpu_window") or {}).get("util_mean"), 1),
                     fmt(b.get("renderer", {}).get("d3d12")), b.get("log", {}).get("n_err"), verdict(b.get("pass"))])
        summ.append(["world", f"bench {Path(b.get('world_sdf', '?')).name} ({b.get('lidar_preset')}){label}",
                     verdict(b.get("pass")),
                     f"load {fmt(b.get('load', {}).get('first_lidar_s'), 1)} s, RTF {fmt(b.get('window', {}).get('rtf_mean'), 3)}, "
                     f"peak RSS {fmt(b.get('mem', {}).get('hwm_mb'), 0)} MB, errors {b.get('log', {}).get('n_err')}, "
                     f"d3d12 {fmt(b.get('renderer', {}).get('d3d12'))}",
                     f"load <= {b.get('thresholds', {}).get('load_time_max_s')} s, RTF >= {b.get('thresholds', {}).get('rtf_min')}, "
                     f"RSS <= {b.get('thresholds', {}).get('server_rss_max_gb')} GB"])
        busy = (b.get("machine_before") or {}).get("perf_benchmark_procs")
        if busy:
            md.append(f"Note: {busy} perf-benchmark processes were running when `{Path(b.get('world_sdf', '?')).name}` "
                      "was measured.\n")
    md.append(table(["world", "lidar", "status", "load [s]", "RTF", "window [s]", "CPU %", "peak RSS [MB]",
                     "GPU %", "d3d12", "[Err]", "verdict"], rows))
    md.append("")
    for b in benches:
        errs = b.get("log", {}).get("errors") or []
        if errs:
            md.append(f"Server-log errors of `{Path(b.get('world_sdf', '?')).name}` (first {min(5, len(errs))}):\n")
            md.extend(f"    {e}" for e in errs[:5])
            md.append("")
    return md, summ


def _lidar_section(lids: list[dict], figdir: Path, figures: bool) -> tuple[list[str], list[list]]:
    md = ["## 3. LiDAR checks\n"]
    summ = []
    if not lids:
        md.append("No `lidar_checks` result.\n")
        return md, summ
    for k, L in enumerate(lids):
        ch = L.get("checks", {})
        md.append(f"Captures: {len(L.get('captures', []))} poses; beam direction max deviation "
                  f"{fmt(L.get('beam_direction_max_dev_deg'), 4)} deg.\n")
        e = ch.get("edge", {})
        md.append(f"- Edge: {verdict(e.get('status'))} - returns per zone {e.get('returns_per_zone')}, "
                  f"max Chebyshev distance from the centre {fmt(e.get('max_cheb_from_centre_m'), 1)} m "
                  f"(ring half-size {fmt(e.get('ring_half_m'), 0)} m).")
        g = ch.get("ground_residuals", {})
        md.append(f"- Ground residuals: {verdict(g.get('status'))} - {g.get('n')} returns, below ground "
                  f"{g.get('n_below')} ({fmt(g.get('frac_below'), 6)}), floating {g.get('n_floating')}, "
                  f"median residual {fmt((g.get('median_m') or 0) * 1000, 2)} mm.")
        m = ch.get("missing_returns", {})
        if m:
            md.append(f"- Missing returns (leaks): {verdict(m.get('status'))} - {m.get('leak_candidates')} of "
                      f"{m.get('expected_ground_hits')} expected ground hits ({fmt(m.get('frac'), 6)}).")
        rd = ch.get("rock_detection", {})
        md.append(f"- Rock detection: {verdict(rd.get('status'))} - {rd.get('n_judged_bins')} judged bins.\n")
        if g.get("bins"):
            md.append(table(["range [m]", "returns", "median [mm]", "p1 [mm]", "p99 [mm]", "below", "above"],
                            [[f"{b['range_m'][0]:g}-{b['range_m'][1]:g}", b["n"], fmt((b.get("median_m") or 0) * 1000, 1),
                              fmt((b.get("p1_m") or 0) * 1000, 1), fmt((b.get("p99_m") or 0) * 1000, 1), b["below"],
                              b["above"]] for b in g["bins"] if b["n"]]))
            md.append("")
        if rd.get("rows"):
            md.append(table(["D [m]", "range [m]", "visible", "occluded", "p_detect", "mean returns", "ideal", "ratio"],
                            [[f"{r['D_m'][0]:g}-{r['D_m'][1]:g}", f"{r['range_m'][0]:g}-{r['range_m'][1]:g}",
                              r["n_visible"], r["n_occluded"], fmt(r.get("p_detect"), 2), fmt(r.get("mean_returns"), 2),
                              fmt(r.get("mean_ideal_returns"), 2), fmt(r.get("ratio_to_ideal"), 2)]
                             for r in rd["rows"]]))
            md.append("\nGazebo small-object reference (cubes): " + "; ".join(
                f"{x['size_m'] * 100:g} cm @ {x['range_m']:g} m: {x['mean_returns']} returns (ideal {x['ideal']})"
                for x in rd.get("gazebo_reference", [])) + "\n")
        if figures:
            f = fig_lidar_range(ch.get("range_histogram", {}), figdir / f"fig_lidar_range{'' if k == 0 else k}.png")
            if f:
                md.append(f"![returns per range]({f})\n")
            f = fig_rock_detection(rd, figdir / f"fig_rock_detection{'' if k == 0 else k}.png")
            if f:
                md.append(f"![rock detection]({f})\n")
        for name in ("edge", "ground_residuals", "missing_returns", "rock_detection"):
            c = ch.get(name)
            if c:
                summ.append(["world", f"lidar {name}", verdict(c.get("status")), _lidar_key(name, c), ""])
    return md, summ


def _lidar_key(name: str, c: dict) -> str:
    if name == "edge":
        return f"beyond ring {c.get('beyond_ring_from_core_sensors')}, max {fmt(c.get('max_cheb_from_centre_m'), 1)} m"
    if name == "ground_residuals":
        return f"below {c.get('n_below')}, floating {c.get('n_floating')} of {c.get('n')}"
    if name == "missing_returns":
        return f"{c.get('leak_candidates')} leak candidates ({fmt(c.get('frac'), 6)})"
    return f"{c.get('n_judged_bins')} judged bins"


def _robot_section(robots: list[dict]) -> tuple[list[str], list[list]]:
    md = ["## 4. Gazebo, robot\n"]
    summ = []
    if not robots:
        md.append("No `robot_tests` result.\n")
        return md, summ
    for R in robots:
        comparison = R.get("role") == "comparison"
        label = R.get("label") or ""
        md.append(f"World `{Path(R.get('world', '?')).name}`, launch `{R.get('launch_kind')}`, "
                  f"{len(R.get('launches', []))} launches" + (f" ({label})" if label else "")
                  + (" - comparison run, reported for information, not judged." if comparison else ".") + "\n")
        rows = []
        for L in R.get("launches", []):
            ev = (L.get("drive") or {}).get("evaluation", {})
            tl = L.get("timeline_s", {})
            air = ev.get("airborne_frac_fwd") if ev.get("airborne_frac_fwd") is not None else ev.get("airborne_frac")
            main_cpu = ((L.get("thread_cpu_drive") or {}).get("drive") or {}).get("main_thread_cpu_s_per_sim_s")
            rows.append([L.get("index"), L.get("site"), fmt(L.get("controllers", {}).get("active")),
                         fmt(L.get("controllers", {}).get("t_s"), 1), fmt(tl.get("entity_created"), 1),
                         fmt(tl.get("lidar_advertised"), 1), fmt(L.get("ready_banner_launch_s"), 1),
                         len(L.get("spawner_failures", [])),
                         fmt(ev.get("rtf_driving"), 3), fmt(main_cpu, 2),
                         fmt(ev.get("fwd_dist_ratio") or ev.get("dist_ratio"), 3),
                         fmt(ev.get("fwd_speed_tracking") or ev.get("speed_tracking"), 3),
                         fmt(ev.get("turn_yawrate_tracking"), 3), fmt(ev.get("climb_angle_deg"), 1), fmt(air, 4),
                         verdict(ev.get("pass")) if ev else "-"])
        md.append(table(["#", "site", "controllers", "active at [s]", "spawned [s]", "lidar adv. [s]", "READY [s]",
                         "spawner fails", "RTF driving", "main thread CPU-s / sim-s", "dist ratio", "v_ss / cmd",
                         "yaw tracking", "climb [deg]", "airborne", "drive"], rows))
        md.append("\n`READY [s]`: the launch's own `[apollo16_sim] READY at +T s` banner (seconds after its start-up); "
                  "`main thread CPU-s / sim-s`: CPU time of the gz server's main (simulation) thread per simulated "
                  "second while driving (1 ms steps: 1.0 means one full core for real time).\n")
        md.append("")
        for k, c in (R.get("checks") or {}).items():
            if comparison:
                summ.append(["robot (comparison)", f"{label or R.get('launch_kind')}: {k}",
                             "INFO" if c.get("pass") is not None else "SKIPPED", _robot_key(k, c), "not judged"])
            else:
                summ.append(["robot", k, verdict(c.get("pass")), _robot_key(k, c), ""])
    return md, summ


def _robot_key(k: str, c: dict) -> str:
    def lst(v, nd=3):
        return "[" + ", ".join(fmt(x, nd) for x in (v or [])) + "]"

    if k == "controllers_active":
        return f"{c.get('active')}/{c.get('launches')} launches, at {lst(c.get('t_active_s'), 1)} s"
    if k == "no_spawner_failure":
        return f"{c.get('failures')} failures"
    if k == "three_launches":
        return f"{c.get('launches')} launches"
    if k == "ready_banner":
        return f"READY at {lst(c.get('launch_s'), 1)} s (launch clock)"
    if k == "rtf_driving":
        ps = c.get("per_site") or {}
        if ps:
            return "; ".join(f"{s} {lst(v)}" for s, v in ps.items())
        return f"RTF {lst(c.get('values'))}"
    if k.startswith("drive_"):
        keys = {"fwd_dist_ratio": "fwd", "fwd_speed_tracking": "v_ss", "turn_yawrate_tracking": "yaw",
                "airborne_frac_fwd": "airborne", "dist_ratio": "dist", "speed_tracking": "v_ss",
                "climb_angle_deg": "climb deg", "airborne_frac": "airborne (info)"}
        runs = c.get("runs", [])
        parts = [f"{short} {lst([r.get(name) for r in runs])}" for name, short in keys.items()
                 if any(r.get(name) is not None for r in runs)]
        return "; ".join(parts)
    return ""


def _visual_section(reports: Path) -> tuple[list[str], list[list]]:
    imgs = []
    for d in (reports, reports / "images", reports / "camera"):
        if d.exists():
            imgs += [p for p in sorted(d.iterdir()) if p.suffix.lower() in IMAGE_EXT and not p.name.startswith("fig_")]
    md = ["## 5. Visual\n"]
    if not imgs:
        md.append("No camera images in the reports folder yet (DESIGN 13: at least 4 viewpoints plus an overview, "
                  "reviewed by eye).\n")
        return md, [["visual", "camera images", "SKIPPED", "0 images", ">= 4 viewpoints + overview"]]
    for p in imgs:
        rel = p.relative_to(reports)
        md.append(f"![{p.stem}]({rel.as_posix()})\n")
    return md, [["visual", "camera images", "REVIEW", f"{len(imgs)} images", ">= 4 viewpoints + overview"]]


# ----------------------------------------------------------------------------------------------
# driver

def collect(reports: Path, extra: Iterable[str | Path] = ()) -> dict[str, list[dict]]:
    found: dict[str, list[dict]] = {"static_checks": [], "bench_world": [], "lidar_checks": [], "robot_tests": []}
    files = list(reports.glob("*.json")) if reports.exists() else []
    files += [Path(p) for p in extra]
    seen = set()
    for f in files:
        f = f.resolve()
        if f in seen:
            continue
        seen.add(f)
        try:
            d = json.loads(f.read_text())
        except Exception:  # noqa: BLE001
            continue
        if isinstance(d, dict) and d.get("kind") in found:
            d["_file"] = str(f)
            found[d["kind"]].append(d)
    for k in found:
        found[k].sort(key=lambda d: d.get("created", ""))
    return found


def build_report(reports_dir: str | Path, *, extra_json: Iterable[str | Path] = (), out_md: str | Path | None = None,
                 figures: bool = True, title: str | None = None) -> Path:
    """Write the markdown report (and figures) into ``reports_dir``; returns its path."""
    reports = Path(reports_dir)
    reports.mkdir(parents=True, exist_ok=True)
    found = collect(reports, extra_json)
    out = Path(out_md) if out_md else reports / "validation_report.md"
    figdir = out.parent
    md: list[str] = []
    summ: list[list] = []
    static = found["static_checks"][-1] if found["static_checks"] else None
    world = (static or {}).get("world") or (title or "world")
    parts = []
    if static:
        s_md, s_sum = _static_section(static, figdir, figures)
        parts += s_md
        summ += s_sum
    else:
        parts += ["## 1. Static checks\n", "No `static_checks.json` yet.\n"]
    for fn, key in ((_bench_section, "bench_world"),):
        b_md, b_sum = fn(found[key])
        parts += b_md
        summ += b_sum
    l_md, l_sum = _lidar_section(found["lidar_checks"], figdir, figures)
    r_md, r_sum = _robot_section(found["robot_tests"])
    v_md, v_sum = _visual_section(reports)
    parts += l_md + r_md + v_md
    summ += l_sum + r_sum + v_sum
    n_fail = sum(1 for r in summ if r[2] in ("FAIL", "ERROR"))
    n_pass = sum(1 for r in summ if r[2] == "PASS")
    n_skip = sum(1 for r in summ if r[2] == "SKIPPED")
    md.append(f"# Validation report: {world}\n")
    md.append(f"Generated {time.strftime('%Y-%m-%d %H:%M:%S')} by `lunar_world_gen.validate.report`. "
              f"Acceptance criteria: docs/DESIGN.md section 13.\n")
    md.append(f"**Overall: {'FAIL' if n_fail else ('PASS' if n_pass else 'INCOMPLETE')}** - {n_pass} passed, "
              f"{n_fail} failed or errored, {n_skip} skipped.\n")
    md.append("## Summary\n")
    md.append(table(["area", "check", "result", "measured", "threshold"], summ))
    md.append("")
    md += parts
    md.append("## Inputs\n")
    for k, lst in found.items():
        for d in lst:
            md.append(f"- {k}: `{d['_file']}` ({d.get('created')})")
    md.append("")
    out.write_text("\n".join(md) + "\n")
    return out


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build reports/validation_report.md from the validation JSONs")
    ap.add_argument("--config")
    ap.add_argument("--preset")
    ap.add_argument("--reports", help="reports dir (default world_dir/reports from the config)")
    ap.add_argument("--json", action="append", default=[], help="extra result JSON files")
    ap.add_argument("--out", help="output markdown path")
    ap.add_argument("--no-figures", action="store_true")
    a = ap.parse_args(argv)
    if a.reports:
        reports = Path(a.reports)
    else:
        from ..config import load_config
        from ..layout import Layout
        reports = Layout(load_config(a.config, a.preset)).world_dir / "reports"
    p = build_report(reports, extra_json=a.json, out_md=a.out, figures=not a.no_figures)
    print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
