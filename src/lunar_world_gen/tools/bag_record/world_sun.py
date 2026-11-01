#!/usr/bin/env python3
"""Copy of a world SDF with the sun's intensity changed: the camera-exposure stand-in for recordings with the
stereo rig. The renderer applies no exposure, so with the world's sun at 1 the sunlit regolith lands at about
17 of 255 in the rig's images (a real camera would expose for it); at 6 it sits near 100. The LiDAR, the IMUs
and the physics do not see the light. The copy goes next to the original (the world's mesh URIs are relative).

usage: world_sun.py WORLD.sdf INTENSITY      -> prints the path of the copy (<stem>_sunINTENSITY.sdf)
"""
import re
import sys
from pathlib import Path


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    src = Path(sys.argv[1])
    intensity = float(sys.argv[2])
    text = src.read_text(encoding="utf-8")
    pattern = re.compile(r'(<light\s+name="sun"[^>]*>.*?<intensity>)([^<]*)(</intensity>)', re.S)
    new, n = pattern.subn(lambda m: f"{m.group(1)}{intensity:g}{m.group(3)}", text, count=1)
    if n != 1:
        raise SystemExit(f"no <light name=\"sun\"> with an <intensity> in {src}")
    tag = f"{intensity:g}".replace(".", "p")
    dst = src.with_name(f"{src.stem}_sun{tag}{src.suffix}")
    dst.write_text(new, encoding="utf-8")
    print(dst)


if __name__ == "__main__":
    main()
