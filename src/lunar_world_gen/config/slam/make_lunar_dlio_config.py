#!/usr/bin/env python3
"""Regenerate ``seyond_dlio_graph_lunar.yaml`` from the user's DLIO + graph-SLAM config.

Why (review_usability.md U1): the Apollo 16 world runs at lunar gravity (1.62 m/s^2), but
``lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml`` sets ``odom/gravity: 9.82`` (it is
matched to the Earth-gravity A/B test in ``lunar_surface.world``). DLIO subtracts
``odom/gravity`` from the IMU, so the value must equal the world's gravity.

The lunar copy is the source file byte for byte, except
  1. a comment header (this script's ``HEADER``), and
  2. the single ``odom/gravity:`` line, whose value becomes the world gravity.

Usage (any python3, stdlib only; run from anywhere)::

    python3 src/lunar_world_gen/config/slam/make_lunar_dlio_config.py           # (re)write the copy
    python3 src/lunar_world_gen/config/slam/make_lunar_dlio_config.py --check   # exit 1 if stale

The gravity defaults to ``physics.gravity_m_s2`` of ``config/apollo16_core.yaml``.
"""
from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent                      # src/lunar_world_gen/config/slam
PKG_DIR = HERE.parents[1]                                    # src/lunar_world_gen
REPO_ROOT = PKG_DIR.parents[1]                               # repository root
SOURCE_REL = "lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml"
DEFAULT_SOURCE = REPO_ROOT / SOURCE_REL
DEFAULT_OUTPUT = HERE / "seyond_dlio_graph_lunar.yaml"
DEFAULT_CONFIG = PKG_DIR / "config" / "apollo16_core.yaml"
CONTAINER_PATH = "/home/er4-user/ws/src/lunar_world_gen/config/slam/seyond_dlio_graph_lunar.yaml"
FALLBACK_GRAVITY = 1.62

GRAVITY_LINE = re.compile(r"^(?P<indent>[ \t]*)odom/gravity:(?P<sp>[ \t]*)(?P<value>[-+0-9.eE]+)(?P<rest>[^\n]*)$",
                          re.MULTILINE)

HEADER = """\
# ═════════════════════════════════════════════════════════════════════════════
# LUNAR-GRAVITY COPY of {source_rel}
# for the Apollo 16 world built by src/lunar_world_gen (apollo16_core*.sdf).
#
# The ONLY change against the source is dlio_odom_node odom/gravity:
#   {old} -> {new} m/s^2 (the world's <gravity>; world_info.yaml gravity_m_s2).
# Everything below this header is the source file byte for byte apart from
# that one line (source md5 {md5}).
# The comment "Must match lunar_surface.world" above odom/gravity is the
# source's own: lunar_surface.world (the Earth-gravity A/B test world) still
# needs the unmodified source file.
#
# Why: the simulated IMU reads {new} m/s^2 at rest. With odom/gravity {old},
# DLIO's start-up calibration (approximateGravity + accel calibration, both on
# below) stores an accelerometer bias of {bias:.2f} m/s^2 along body z, which
# then leaks |bias|*sin(tilt) into the horizontal IMU propagation:
# {e5:.2f} m/s^2 at 5 deg, {e10:.2f} at 10 deg, {e12:.2f} on the 12.3 deg 'slope' site.
#
# Use (inside the SLAM dev container; ./src is mounted at /home/er4-user/ws/src):
#   COMBINED_PARAM_FILE={container_path} \\
#     ~/slam_ws/src/lidarslam_ros2/scripts/togo/run_live_seyond_dlio_slam.sh
# (or DLIO_PARAM_FILE=... alone: only dlio_odom_node reads odom/gravity).
# Regenerate after editing the source:
#   python3 src/lunar_world_gen/config/slam/make_lunar_dlio_config.py [--check]
# ═════════════════════════════════════════════════════════════════════════════
"""


def config_gravity(config: Path = DEFAULT_CONFIG) -> float:
    """``physics.gravity_m_s2`` from the generator config (stdlib only, no PyYAML needed)."""
    try:
        lines = config.read_text(encoding="utf-8").splitlines()
    except OSError:
        return FALLBACK_GRAVITY
    in_physics = False
    for line in lines:
        if line and not line[0].isspace() and not line.startswith("#"):
            in_physics = line.split("#")[0].strip() == "physics:"
            continue
        if in_physics:
            m = re.match(r"^[ \t]+gravity_m_s2:[ \t]*([0-9.eE+-]+)", line)
            if m:
                return float(m.group(1))
    return FALLBACK_GRAVITY


def fmt_gravity(g: float) -> str:
    s = f"{g:.6g}"
    return s if any(c in s for c in ".eE") else s + ".0"      # keep it a YAML float (DLIO declares a double)


def make_lunar(source_text: str, gravity: float, source_md5: str) -> str:
    """The lunar copy of ``source_text`` (header + one replaced value)."""
    matches = list(GRAVITY_LINE.finditer(source_text))
    if len(matches) != 1:
        raise ValueError(f"expected exactly one 'odom/gravity:' line in the source, found {len(matches)}")
    m = matches[0]
    old = float(m.group("value"))
    new_value = fmt_gravity(gravity)
    line = (f"{m.group('indent')}odom/gravity:{m.group('sp')}{new_value}{m.group('rest')}"
            f"   # LUNAR COPY: apollo16_core world gravity (source file: {m.group('value')}), see header")
    body = source_text[:m.start()] + line + source_text[m.end():]
    bias = gravity - old                                    # accel_avg - normalize(accel_avg)*|old|, at rest
    e = {k: abs(bias) * math.sin(math.radians(d)) for k, d in (("e5", 5.0), ("e10", 10.0), ("e12", 12.34))}
    header = HEADER.format(source_rel=SOURCE_REL, old=m.group("value"), new=new_value, md5=source_md5,
                           bias=bias, container_path=CONTAINER_PATH, **e)
    return header + body


def build(source: Path = DEFAULT_SOURCE, gravity: float | None = None) -> str:
    raw = source.read_bytes()
    text = raw.decode("utf-8")
    return make_lunar(text, config_gravity() if gravity is None else gravity, hashlib.md5(raw).hexdigest())


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help=f"default: <repo>/{SOURCE_REL}")
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument("--gravity", type=float, default=None,
                    help="m/s^2 (default: physics.gravity_m_s2 of config/apollo16_core.yaml)")
    ap.add_argument("--check", action="store_true", help="only compare; exit 1 if the output is stale")
    a = ap.parse_args(argv)
    if not a.source.is_file():
        print(f"ERROR: source not found: {a.source}", file=sys.stderr)
        return 2
    text = build(a.source, a.gravity)
    current = a.output.read_text(encoding="utf-8") if a.output.is_file() else None
    if a.check:
        if current == text:
            print(f"up to date: {a.output}")
            return 0
        print(f"STALE: {a.output} differs from what {a.source} gives; rerun without --check", file=sys.stderr)
        return 1
    if current == text:
        print(f"unchanged: {a.output}")
        return 0
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with open(a.output, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(f"wrote {a.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
