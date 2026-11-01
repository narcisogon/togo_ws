#!/usr/bin/env python3
"""Resource budget of a SLAM replay: checks a run's metrics.json (tools/slam_eval.py) against the limits in
config/slam/slam_budget.json and prints one line per limit. Exit 0 when every limit holds, 1 when one is exceeded.
A metric the run does not have (the hazard block of a --no-hazard run, say) is reported as n/a and does not fail.

usage: slam_budget.py RUN/metrics.json [--budget config/slam/slam_budget.json] [--quiet]

The limits are not targets: they sit well above the measured defaults (round 6 of the speed-up plan, 2026-11-01) so
that only a real regression trips them, with the benchmark comparison (slam_compare.py) as the finer judge.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_BUDGET = Path(__file__).resolve().parents[1] / "config" / "slam" / "slam_budget.json"


def lookup(tree: dict, dotted: str):
    node = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node if isinstance(node, (int, float)) and not isinstance(node, bool) else None


def check(metrics: dict, budget: dict) -> tuple[list[dict], bool]:
    rows, exceeded = [], False
    for limit in budget["limits"]:
        value = lookup(metrics, limit["key"])
        if value is None:
            rows.append({**limit, "value": None, "state": "n/a"})
            continue
        over = value > limit["max"]
        exceeded |= over
        rows.append({**limit, "value": value, "state": "OVER" if over else "ok"})
    return rows, exceeded


def format_rows(rows: list[dict]) -> str:
    width = max(len(r["name"]) for r in rows)
    lines = []
    for r in rows:
        value = "n/a" if r["value"] is None else f"{r['value']:g}"
        lines.append(f"  {r['name']:<{width}}  {value:>8} {r['unit']:<6} limit {r['max']:g} {r['unit']:<6} {r['state']}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("metrics", type=Path, help="metrics.json of a replay run")
    ap.add_argument("--budget", type=Path, default=DEFAULT_BUDGET)
    ap.add_argument("--quiet", action="store_true", help="only the verdict line")
    a = ap.parse_args(argv)
    metrics = json.loads(a.metrics.read_text())
    budget = json.loads(a.budget.read_text())
    rows, exceeded = check(metrics, budget)
    over = [r["name"] for r in rows if r["state"] == "OVER"]
    if not a.quiet:
        print(format_rows(rows))
    print("budget: " + ("EXCEEDED (" + ", ".join(over) + ")" if over else "within limits"))
    return 1 if exceeded else 0


if __name__ == "__main__":
    sys.exit(main())
