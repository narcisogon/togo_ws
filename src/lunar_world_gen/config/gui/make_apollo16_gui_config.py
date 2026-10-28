#!/usr/bin/env python3
"""Set the start camera of ``apollo16.gui.config`` from ``world_info.yaml`` (review_usability U6).

The camera is a chase view of one spawn site (default ``flat``): ``BEHIND`` m behind the spawn point
along the rover's heading, ``HEIGHT`` m above the site's visual ground, aimed at the rover body
(``AIM_DZ`` above the ground). ``scripts/run_apollo16_sim.sh`` shifts this view to the chosen spawn by
(spawn - flat spawn), so the file must follow the ``flat`` site whenever a rebuild moves it.

Only the block between the ``START CAMERA`` comment and ``</camera_pose>`` is rewritten. Usage (any
python3 with PyYAML, e.g. /usr/bin/python3 or the project venv)::

    python3 src/lunar_world_gen/config/gui/make_apollo16_gui_config.py            # rewrite the camera
    python3 src/lunar_world_gen/config/gui/make_apollo16_gui_config.py --check    # exit 1 if stale
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "apollo16.gui.config"
DEFAULT_WORLD_INFO = Path("~/togo_assets/lunar/worlds/apollo16_core/world_info.yaml")
BEHIND, HEIGHT, AIM_DZ = 5.0, 2.2, 0.3

BLOCK = re.compile(r"(?P<indent>[ \t]*)<!-- START CAMERA.*?</camera_pose>", re.S)


def chase_pose(site: dict, behind: float = BEHIND, height: float = HEIGHT, aim_dz: float = AIM_DZ
               ) -> tuple[float, float, float, float, float, float]:
    """gz ``camera_pose`` (x, y, z, roll, pitch, yaw; pitch > 0 looks down) for a world_info site."""
    yaw = float(site["yaw"])
    ground = float(site.get("ground_z_visual", float(site["z"]) - 0.176))
    x = float(site["x"]) - behind * math.cos(yaw)
    y = float(site["y"]) - behind * math.sin(yaw)
    z = ground + height
    pitch = math.atan2(height - aim_dz, behind)
    return (x, y, z, 0.0, pitch, math.atan2(math.sin(yaw), math.cos(yaw)))


def format_pose(p) -> str:
    return f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} 0 {p[4]:.4f} {p[5]:.4f}"


def camera_block(info: dict, site_name: str = "flat", indent: str = "  ") -> str:
    site = info["sites"][site_name]
    p = chase_pose(site)
    dist = math.hypot(BEHIND, HEIGHT - AIM_DZ)
    light = info.get("lighting") or {}
    sun = ""
    if "sun_azimuth_deg" in light:
        view_az = math.degrees(math.atan2(math.cos(p[5]), math.sin(p[5]))) % 360.0   # from north through east
        rel = (float(light["sun_azimuth_deg"]) - view_az + 540.0) % 360.0 - 180.0
        side = "right" if rel > 0 else "left"
        sun = (f";\n{indent}     sun (azimuth {float(light['sun_azimuth_deg']):.1f} deg) {abs(rel):.0f} deg to the {side} of "
               f"the view direction")
    g = float(site.get("ground_z_visual", float(site["z"]) - 0.176))
    return (f"{indent}<!-- START CAMERA (written by make_apollo16_gui_config.py from world_info.yaml):\n"
            f"{indent}     chase view of site '{site_name}' (x {float(site['x']):.3f}, y {float(site['y']):.3f}, "
            f"heading {math.degrees(float(site['yaw'])):.1f} deg,\n"
            f"{indent}     visual ground z {g:.3f}): {BEHIND:g} m behind it, {HEIGHT:g} m above the ground, aimed at\n"
            f"{indent}     the rover body (pitch {math.degrees(p[4]):.1f} deg down, {dist:.2f} m away){sun}.\n"
            f"{indent}     x y z roll pitch yaw; pitch > 0 looks down -->\n"
            f"{indent}<camera_pose>{format_pose(p)}</camera_pose>")


def apply(text: str, info: dict, site_name: str = "flat") -> str:
    matches = list(BLOCK.finditer(text))
    if len(matches) != 1:
        raise ValueError(f"expected one START CAMERA block ending with </camera_pose>, found {len(matches)}")
    m = matches[0]
    return text[:m.start()] + camera_block(info, site_name, m.group("indent")) + text[m.end():]


def read_pose(text: str) -> list[float]:
    m = re.search(r"<camera_pose>\s*([^<]*?)\s*</camera_pose>", text)
    if m is None:
        raise ValueError("no <camera_pose>")
    return [float(v) for v in m.group(1).split()]


def main(argv: list[str] | None = None) -> int:
    import yaml

    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--world-info", type=Path, default=DEFAULT_WORLD_INFO)
    ap.add_argument("--site", default="flat")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--check", action="store_true", help="only compare; exit 1 if the camera is stale")
    a = ap.parse_args(argv)
    wi = a.world_info.expanduser()
    if not wi.is_file():
        print(f"ERROR: {wi} not found (build the world, or pass --world-info)", file=sys.stderr)
        return 2
    info = yaml.safe_load(wi.read_text()) or {}
    if a.site not in (info.get("sites") or {}):
        print(f"ERROR: site '{a.site}' not in {wi}", file=sys.stderr)
        return 2
    text = a.config.read_text(encoding="utf-8")
    new = apply(text, info, a.site)
    if a.check:
        if new == text:
            print(f"up to date: {a.config} (site '{a.site}' of {wi})")
            return 0
        print(f"STALE: the camera of {a.config} does not match site '{a.site}' of {wi}; rerun without --check",
              file=sys.stderr)
        return 1
    if new == text:
        print(f"unchanged: {a.config}")
        return 0
    with open(a.config, "w", encoding="utf-8", newline="\n") as f:
        f.write(new)
    print(f"wrote {a.config}: camera_pose {format_pose(read_pose(new))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
