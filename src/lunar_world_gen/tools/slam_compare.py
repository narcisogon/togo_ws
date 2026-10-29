#!/usr/bin/env python3
"""Summarise and compare SLAM benchmark runs (metrics.json of scripts/replay_slam_bag.sh runs).

Usage: slam_compare.py --baseline LABEL [--candidate LABEL ...] [--root DIR] [--out FILE.md] [--plot FILE.png]

Runs are grouped by the label in their run_info.txt (scripts/benchmark_slam.sh --label). For each metric the
table shows mean ± standard deviation over the group's runs. With --candidate, also the change against the
baseline and a verdict: 'better' / 'worse' only when the change is larger than twice the larger of the two
groups' run-to-run spreads (or a small absolute floor), else '~' (within noise).
Signed metrics (vertical drift, heading lag) are judged by their size: closer to 0 is better.
--plot draws, per label, the errors along the lap (errors_vs_distance.csv of each run, 1 m bins, mean over the
runs, band = min..max): position error, cross-track, zig-zag RMS and vertical error, plus the vertical drift;
every run replays the same bag, so the same distance is the same place on the ground.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

ROOT = Path.home() / "togo_assets/lunar/bags/slam_runs"

# (key path, name, unit, lower_is_better, noise floor); lower_is_better "abs": closer to 0 is better (signed
# metrics), None: shown without a verdict
METRICS = [
    ("accuracy.ate_rmse_m", "ATE RMSE (4-DoF aligned)", "m", True, 0.005),
    ("accuracy.ate_max_m", "ATE max", "m", True, 0.01),
    ("accuracy.along_rmse_m", "along-track RMSE", "m", True, 0.005),
    ("accuracy.cross_rmse_m", "cross-track RMSE", "m", True, 0.005),
    ("accuracy.vertical_rmse_m", "vertical RMSE", "m", True, 0.005),
    ("accuracy.vertical_max_m", "vertical error, max", "m", True, 0.01),
    ("accuracy.yaw_err_rmse_deg", "heading error RMSE", "deg", True, 0.02),
    ("accuracy.roll_err_rmse_deg", "roll error RMSE", "deg", True, 0.02),
    ("accuracy.pitch_err_rmse_deg", "pitch error RMSE", "deg", True, 0.02),
    ("accuracy.drift_end_m", "end drift (start-anchored)", "m", True, 0.01),
    ("accuracy.drift_end_pct", "end drift / path", "%", True, 0.01),
    ("accuracy.drift_end_z_m", "end drift, vertical", "m", "abs", 0.01),
    ("accuracy.drift_max_z_m", "largest vertical drift", "m", "abs", 0.01),
    ("accuracy.vertical_drift_per_100m", "vertical drift per 100 m", "m", "abs", 0.01),
    ("accuracy.vertical_vs_climb_pct", "vertical drift per metre climbed (height scale)", "%", "abs", 0.5),
    ("accuracy.drift_end_yaw_deg", "end drift, yaw", "deg", "abs", 0.05),
    ("accuracy.heading_lag_ms", "heading lag in turns (pose behind its stamp)", "ms", "abs", 5.0),
    ("accuracy.ate_rmse_6dof_m", "ATE RMSE, 6-DoF aligned (reference)", "m", True, 0.005),
    ("accuracy.rpe_1m_mean_m", "RPE 1 m, translation", "m", True, 0.001),
    ("accuracy.rpe_1m_rot_mean_deg", "RPE 1 m, rotation", "deg", True, 0.01),
    ("accuracy.rpe_10m_mean_pct", "RPE 10 m, translation", "%", True, 0.02),
    ("accuracy.rpe_10m_rot_mean_deg", "RPE 10 m, rotation", "deg", True, 0.02),
    ("accuracy.rpe_10m_vertical_mean_m", "RPE 10 m, vertical", "m", True, 0.005),
    ("accuracy.zigzag_rms_m", "zig-zag RMS (lateral jitter)", "m", True, 0.001),
    ("accuracy.zigzag_p95_m", "zig-zag p95", "m", True, 0.002),
    ("accuracy.segments.to_rocks.ate_rmse_m", "ATE, start -> rocks", "m", True, 0.005),
    ("accuracy.segments.rocks_to_slope.ate_rmse_m", "ATE, rocks -> slope", "m", True, 0.005),
    ("accuracy.segments.slope_to_plain.ate_rmse_m", "ATE, slope -> plain", "m", True, 0.005),
    ("accuracy.segments.plain_to_start.ate_rmse_m", "ATE, plain -> start", "m", True, 0.005),
    ("scan_matching.gicp_ms.mean", "GICP solve time, mean", "ms", True, 0.2),
    ("scan_matching.gicp_ms.p95", "GICP solve time, p95", "ms", True, 0.5),
    ("scan_matching.gicp_ms.max", "GICP solve time, max", "ms", True, 2.0),
    ("scan_matching.correction_mm.mean", "per-scan correction, mean", "mm", True, 0.1),
    ("scan_matching.correction_mm.p95", "per-scan correction, p95", "mm", True, 0.3),
    ("scan_matching.correction_deg.mean", "per-scan rotation correction", "deg", True, 0.002),
    ("scan_matching.fitness.mean", "GICP fitness, mean", "", True, 1e-4),
    ("scan_matching.overlap.mean", "overlap, mean", "", False, 0.002),
    ("scan_matching.hessian_min_eig.p5", "Hessian min eigenvalue, p5", "", False, 1.0),
    ("scan_matching.hessian_condition.p95", "Hessian condition, p95", "", True, 1.0),
    ("scan_matching.body_trans_info_min.p5", "body-frame translation information, min, p5", "", False, 1.0),
    ("scan_matching.iw_scale_min.p50", "information weighting, min scale, p50", "", None, 0.01),
    ("scan_matching.scan_age_at_correction_ms.p50", "scan age at correction, p50", "ms", True, 0.5),
    ("scan_matching.gicp_iterations.mean", "GICP iterations per scan, mean", "", None, 0.1),
    ("scan_matching.corrections_rejected", "rejected corrections", "", True, 1),
    ("scan_matching.stale_scan_drops", "stale scans dropped", "", True, 1),
    ("scan_matching.scans_processed", "scans processed", "", False, 1),
    ("scan_matching.filtered_points.mean", "points per scan after filter", "", None, 10),
    ("resources.dlio_cpu_ms_per_scan", "DLIO CPU time per scan", "ms", True, 0.5),
    ("resources.processes.dlio_odom_node.cpu_pct_mean", "dlio_odom_node CPU", "%", True, 2),
    ("resources.processes.dlio_odom_node.rss_peak_mb", "dlio_odom_node RAM peak", "MB", True, 10),
    ("resources.processes.dlio_odom_node.rss_growth_mb_per_min", "dlio_odom_node RAM growth", "MB/min", True, 1),
    ("resources.processes.dlio_map_node.rss_peak_mb", "dlio_map_node RAM peak", "MB", True, 10),
    ("resources.processes.dlio_map_node.cpu_pct_mean", "dlio_map_node CPU", "%", True, 2),
    ("resources.processes.graph_based_slam_node.rss_peak_mb", "graph backend RAM peak", "MB", True, 10),
    ("resources.processes.graph_based_slam_node.cpu_pct_mean", "graph backend CPU", "%", True, 2),
    ("resources.total_cpu_pct_mean", "all SLAM processes, CPU", "%", True, 3),
    ("resources.total_rss_peak_mb", "all SLAM processes, RAM peak", "MB", True, 20),
    ("hazard.update_ms.mean", "hazard map update, mean", "ms", True, 5),
    ("hazard.update_ms.p95", "hazard map update, p95", "ms", True, 10),
    ("hazard.dropped_clouds", "hazard clouds dropped", "", True, 5),
    ("loop_closure.max_correction_m", "loop-closure correction (0 = off)", "m", True, 0.001),
]


def get(d: dict, path: str):
    for k in path.split("."):
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d if isinstance(d, (int, float)) and not isinstance(d, bool) else None


def run_dirs(root: Path, label: str) -> list[Path]:
    out = []
    for info in sorted(root.glob("*/run_info.txt")):
        kv = dict(l.split("=", 1) for l in info.read_text().splitlines() if "=" in l)
        if kv.get("label") == label and (info.parent / "metrics.json").is_file():
            out.append(info.parent)
    return out


def load_runs(root: Path, label: str) -> list[dict]:
    return [json.loads((d / "metrics.json").read_text()) for d in run_dirs(root, label)]


def run_marks(run_dir: Path) -> list[tuple[str, float]]:
    """(place, distance along the lap) where the route's legs start: the replayed bag's segments.json, placed with
    the run's errors_vs_distance.csv (stamp -> distance). Empty when either is missing."""
    import numpy as np
    from slam_eval import place_marks
    kv = dict(l.split("=", 1) for l in (run_dir / "run_info.txt").read_text().splitlines() if "=" in l)
    seg = Path(kv["bag"]) / "segments.json" if kv.get("bag") else None
    f = run_dir / "errors_vs_distance.csv"
    if not seg or not seg.is_file() or not f.is_file():
        return []
    rows = list(csv.DictReader(open(f)))
    t, s = (np.array([float(r[k]) for r in rows]) for k in ("t", "s"))
    return [(name, sk) for name, sk, _ in place_marks(json.loads(seg.read_text()), t, s)]


def plot_along_lap(root: Path, labels: list[str], out: Path, bin_m: float = 1.0) -> None:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = [("err", "position error, aligned (m)"), ("cross", "cross-track error (m)"),
            ("cross_hp", "zig-zag RMS: cross-track minus 2 s average (m)"), ("vertical", "vertical error, aligned (m)"),
            ("drift_z", "vertical drift, start-anchored (m)")]
    # per 1 m bin: mean, except the zig-zag (zero-mean by construction), whose size is its RMS
    agg = {"err": lambda x: np.mean(np.abs(x)), "cross_hp": lambda x: np.sqrt(np.mean(x ** 2))}
    fig, ax = plt.subplots(len(cols), 1, figsize=(13, 3.0 * len(cols)), sharex=True)
    for li, label in enumerate(labels):
        curves = []
        for d in run_dirs(root, label):
            f = d / "errors_vs_distance.csv"
            if not f.is_file():
                continue
            rows = list(csv.DictReader(open(f)))
            if not rows:
                continue
            s = np.array([float(r["s"]) for r in rows])
            curves.append((s, {c: np.array([float(r[c]) for r in rows]) for c, _ in cols}))
        if not curves:
            continue
        edges = np.arange(0, max(c[0][-1] for c in curves) + bin_m, bin_m)
        mid = (edges[:-1] + edges[1:]) / 2
        for k, (c, name) in enumerate(cols):
            per_run = []
            reduce = agg.get(c, np.mean)
            for s, v in curves:
                b = np.digitize(s, edges) - 1
                per_run.append([reduce(v[c][b == i]) if np.any(b == i) else np.nan for i in range(len(mid))])
            arr = np.array(per_run)
            color = f"C{li}"
            ax[k].plot(mid, np.nanmean(arr, 0), color=color, lw=1.2, label=f"{label} (n={len(curves)})")
            if len(curves) > 1:
                ax[k].fill_between(mid, np.nanmin(arr, 0), np.nanmax(arr, 0), color=color, alpha=0.2, lw=0)
            ax[k].set_ylabel(name, fontsize=8)
    marks = next((m for label in labels for d in run_dirs(root, label) if (m := run_marks(d))), [])
    for a in ax:
        a.axhline(0, color="k", lw=0.5)
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper left")
        for _, sk in marks:
            a.axvline(sk, color="k", lw=0.6, alpha=0.4)
    for name, sk in marks:
        ax[0].text(sk, 1.0, name, transform=ax[0].get_xaxis_transform(), fontsize=8, ha="center", va="bottom")
    ax[-1].set_xlabel("distance along the lap (m): the same distance is the same place in every run")
    ax[0].set_title("errors along the lap: mean over each label's runs, band = min..max", pad=16)
    fig.savefig(out, dpi=100, bbox_inches="tight")
    plt.close(fig)


def mean_sd(vals: list[float]) -> tuple[float, float] | None:
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    m = sum(vals) / len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1)) if len(vals) > 1 else float("nan")
    return m, sd


def fmt(x: float) -> str:
    if x is None or not math.isfinite(x):
        return "-"
    a = abs(x)
    return f"{x:.0f}" if a >= 100 else f"{x:.1f}" if a >= 10 else f"{x:.2f}" if a >= 1 else f"{x:.3f}" if a >= 0.01 else f"{x:.4f}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--candidate", action="append", default=[])
    ap.add_argument("--root", type=Path, default=ROOT)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--plot", type=Path, help="PNG of the errors along the lap, one curve per label")
    a = ap.parse_args()
    groups = [a.baseline] + a.candidate
    runs = {g: load_runs(a.root, g) for g in groups}
    for g, r in runs.items():
        if not r:
            raise SystemExit(f"no runs with label '{g}' under {a.root}")
    head = ["metric", "unit"] + [f"{g} (n={len(runs[g])})" for g in groups]
    if a.candidate:
        head += [f"change {c}" for c in a.candidate] + ["verdict"]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for path, name, unit, lower, floor in METRICS:
        stats = {g: mean_sd([get(r, path) for r in runs[g]]) for g in groups}
        if all(s is None for s in stats.values()):
            continue
        row = [name, unit]
        for g in groups:
            s = stats[g]
            row.append("-" if s is None else fmt(s[0]) + ("" if not math.isfinite(s[1]) else f" ± {fmt(s[1])}"))
        if a.candidate:
            b = stats[a.baseline]
            verdicts = []
            for c in a.candidate:
                s = stats[c]
                if b is None or s is None:
                    row.append("-")
                    verdicts.append("-")
                    continue
                d = s[0] - b[0]
                pct = f" ({d / b[0] * 100:+.0f} %)" if b[0] else ""
                row.append(f"{'+' if d >= 0 else ''}{fmt(d)}{pct}")
                spread = max([x for x in (b[1], s[1]) if math.isfinite(x)], default=0.0)
                if lower is None:
                    verdicts.append("info")
                elif abs(d) <= max(2 * spread, floor):
                    verdicts.append("~")
                elif lower == "abs":
                    verdicts.append("better" if abs(s[0]) < abs(b[0]) else "worse")
                else:
                    verdicts.append("better" if (d < 0) == lower else "worse")
            row.append(", ".join(verdicts))
        lines.append("| " + " | ".join(row) + " |")
    text = "\n".join(lines)
    print(text)
    if a.out:
        a.out.write_text(text + "\n")
    if a.plot:
        try:
            plot_along_lap(a.root, groups, a.plot)
            print(f"Saved: {a.plot}")
        except Exception as e:  # noqa: BLE001 - the table matters more than the picture
            print(f"(plot skipped: {e})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
