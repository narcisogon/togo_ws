#!/usr/bin/env python3
"""Prepare a small cleaned lunar height image from NAC_DTMS.png preview imagery."""

from pathlib import Path
import argparse
from PIL import Image, ImageFilter, ImageOps


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='/mnt/c/Users/Username/Downloads/NAC_DTMS.png')
    parser.add_argument('--out-dir', default='/mnt/c/Users/Username/OneDrive/Desktop/husky/lunar_nac_small_01')
    parser.add_argument('--x', type=int, default=500)
    parser.add_argument('--y', type=int, default=420)
    parser.add_argument('--size', type=int, default=420)
    parser.add_argument('--autocontrast-cutoff', type=float, default=1.0)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rgba = Image.open(args.input).convert('RGBA')
    w, h = rgba.size
    x = max(0, min(args.x, w - args.size))
    y = max(0, min(args.y, h - args.size))
    crop = rgba.crop((x, y, x + args.size, y + args.size))

    gray = crop.convert('L')
    median = gray.filter(ImageFilter.MedianFilter(size=7))

    pix = crop.load()
    gpix = gray.load()
    mpix = median.load()
    for yy in range(crop.height):
        for xx in range(crop.width):
            r, g, b, a = pix[xx, yy]
            is_pink = r > 160 and b > 140 and g < 170 and (r - g) > 35 and (b - g) > 20
            if is_pink:
                gpix[xx, yy] = mpix[xx, yy]

    # Convert shaded relief into a usable synthetic height field. This is not
    # metrically true elevation, but it preserves crater/ridge texture for SLAM testing.
    gray = ImageOps.autocontrast(gray, cutoff=args.autocontrast_cutoff)
    heightmap = gray.resize((513, 513), Image.Resampling.BICUBIC)
    height_path = out_dir / 'nac_preview_height.png'
    crop_path = out_dir / 'nac_preview_crop.png'
    crop.save(crop_path)
    heightmap.save(height_path)

    print(f'Crop preview: {crop_path}')
    print(f'Height image:  {height_path}')
    print('Next:')
    print(f'  python3 scripts/make_lunar_mesh_world.py --image {height_path} --out-dir {out_dir} --size-m 120 --height-m 12 --resolution 65 --z-offset -4')


if __name__ == '__main__':
    main()