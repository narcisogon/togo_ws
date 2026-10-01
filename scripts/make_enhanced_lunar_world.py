#!/usr/bin/env python3
"""Generate a Gazebo-friendly lunar terrain from a DEM-like image plus procedural detail."""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

from PIL import Image


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge0 == edge1:
        return 0.0
    t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def load_base_heights(image_path: Path, resolution: int, height_m: float) -> list[list[float]]:
    img = Image.open(image_path).convert('L').resize((resolution, resolution), Image.Resampling.BILINEAR)
    pix = list(img.getdata())
    min_v = min(pix)
    max_v = max(pix)
    denom = max(max_v - min_v, 1)
    heights = [[0.0 for _ in range(resolution)] for _ in range(resolution)]
    for y in range(resolution):
        for x in range(resolution):
            value = pix[y * resolution + x]
            heights[y][x] = ((value - min_v) / denom) * height_m
    return heights


def add_crater(heights: list[list[float]], size_m: float, cx: float, cy: float, radius: float, depth: float, rim: float) -> None:
    resolution = len(heights)
    step = size_m / (resolution - 1)
    half = size_m / 2.0
    for y in range(resolution):
        wy = half - y * step
        for x in range(resolution):
            wx = x * step - half
            r = math.hypot(wx - cx, wy - cy)
            if r > radius * 1.45:
                continue
            bowl = -depth * math.exp(-((r / max(radius * 0.62, 0.01)) ** 2))
            rim_ring = rim * math.exp(-(((r - radius) / max(radius * 0.18, 0.01)) ** 2))
            ejecta = 0.18 * rim * math.exp(-(((r - radius * 1.2) / max(radius * 0.35, 0.01)) ** 2))
            heights[y][x] += bowl + rim_ring + ejecta


def add_mound(heights: list[list[float]], size_m: float, cx: float, cy: float, radius: float, height: float) -> None:
    resolution = len(heights)
    step = size_m / (resolution - 1)
    half = size_m / 2.0
    for y in range(resolution):
        wy = half - y * step
        for x in range(resolution):
            wx = x * step - half
            r = math.hypot(wx - cx, wy - cy)
            if r > radius:
                continue
            heights[y][x] += height * (1.0 - smoothstep(0.0, radius, r))


def add_fine_texture(heights: list[list[float]], amplitude: float, passes: int = 3) -> None:
    """Add low-cost lunar regolith roughness without separate texture files."""
    resolution = len(heights)
    for _ in range(passes):
        noise = [[random.uniform(-amplitude, amplitude) for _ in range(resolution)] for _ in range(resolution)]
        for y in range(1, resolution - 1):
            for x in range(1, resolution - 1):
                local = (
                    noise[y][x] * 0.45
                    + (noise[y - 1][x] + noise[y + 1][x] + noise[y][x - 1] + noise[y][x + 1]) * 0.10
                    + (noise[y - 1][x - 1] + noise[y - 1][x + 1] + noise[y + 1][x - 1] + noise[y + 1][x + 1]) * 0.01875
                )
                heights[y][x] += local
        amplitude *= 0.55


def add_ridge(heights: list[list[float]], size_m: float, cx: float, cy: float, angle: float, length: float, width: float, height: float) -> None:
    resolution = len(heights)
    step = size_m / (resolution - 1)
    half = size_m / 2.0
    ca = math.cos(angle)
    sa = math.sin(angle)
    for y in range(resolution):
        wy = half - y * step
        for x in range(resolution):
            wx = x * step - half
            dx = wx - cx
            dy = wy - cy
            along = dx * ca + dy * sa
            across = -dx * sa + dy * ca
            if abs(along) > length / 2.0 or abs(across) > width * 2.5:
                continue
            taper = 1.0 - smoothstep(length * 0.35, length * 0.5, abs(along))
            profile = math.exp(-((across / max(width, 0.01)) ** 2))
            heights[y][x] += height * taper * profile


def flatten_spawn(heights: list[list[float]], size_m: float, radius: float) -> None:
    resolution = len(heights)
    step = size_m / (resolution - 1)
    half = size_m / 2.0
    samples = []
    for y in range(resolution):
        wy = half - y * step
        for x in range(resolution):
            wx = x * step - half
            if math.hypot(wx, wy) <= radius * 0.6:
                samples.append(heights[y][x])
    target = sum(samples) / max(len(samples), 1)
    for y in range(resolution):
        wy = half - y * step
        for x in range(resolution):
            wx = x * step - half
            r = math.hypot(wx, wy)
            if r > radius:
                continue
            blend = 1.0 - smoothstep(radius * 0.55, radius, r)
            heights[y][x] = heights[y][x] * (1.0 - blend) + target * blend


def normalize_floor(heights: list[list[float]], z_offset: float) -> None:
    min_h = min(min(row) for row in heights)
    for y, row in enumerate(heights):
        for x, h in enumerate(row):
            row[x] = h - min_h + z_offset


def save_height_preview(heights: list[list[float]], path: Path) -> None:
    flat = [h for row in heights for h in row]
    lo = min(flat)
    hi = max(flat)
    denom = max(hi - lo, 1e-6)
    img = Image.new('L', (len(heights), len(heights)))
    img.putdata([int(clamp((h - lo) / denom, 0.0, 1.0) * 255) for h in flat])
    img.save(path)


def write_obj(heights: list[list[float]], obj_path: Path, size_m: float) -> None:
    resolution = len(heights)
    step = size_m / (resolution - 1)
    half = size_m / 2.0

    def normal_at(x: int, y: int) -> tuple[float, float, float]:
        xl = max(x - 1, 0)
        xr = min(x + 1, resolution - 1)
        yd = max(y - 1, 0)
        yu = min(y + 1, resolution - 1)
        dzdx = (heights[y][xr] - heights[y][xl]) / max((xr - xl) * step, step)
        dzdy = (heights[yd][x] - heights[yu][x]) / max((yu - yd) * step, step)
        nx, ny, nz = -dzdx, -dzdy, 1.0
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        return nx / length, ny / length, nz / length

    with obj_path.open('w', encoding='ascii') as f:
        f.write('# Enhanced lunar terrain mesh\n')
        f.write('o enhanced_lunar_patch\n')
        for y in range(resolution):
            for x in range(resolution):
                wx = x * step - half
                wy = half - y * step
                f.write(f'v {wx:.5f} {wy:.5f} {heights[y][x]:.5f}\n')
        for y in range(resolution):
            for x in range(resolution):
                nx, ny, nz = normal_at(x, y)
                f.write(f'vn {nx:.6f} {ny:.6f} {nz:.6f}\n')
        for y in range(resolution - 1):
            for x in range(resolution - 1):
                i0 = y * resolution + x + 1
                i1 = i0 + 1
                i2 = i0 + resolution
                i3 = i2 + 1
                f.write(f'f {i0}//{i0} {i2}//{i2} {i1}//{i1}\n')
                f.write(f'f {i1}//{i1} {i2}//{i2} {i3}//{i3}\n')


def write_world(obj_path: Path, world_path: Path, use_ogre2_sensors: bool) -> None:
    render_engine = 'ogre2' if use_ogre2_sensors else 'ogre'
    world_path.write_text(f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="enhanced_lunar_test">
    <physics name="fast_stable" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.33 0.33 0.34 1</ambient>
      <background>0.025 0.025 0.035 1</background>
      <grid>true</grid>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="low_sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 60 0 0 0</pose>
      <diffuse>0.72 0.70 0.66 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <intensity>2.8</intensity>
      <direction>-0.65 0.25 -0.72</direction>
    </light>

    <model name="enhanced_lunar_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <mesh><uri>file://{obj_path}</uri></mesh>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <mesh><uri>file://{obj_path}</uri></mesh>
          </geometry>
          <material>
            <ambient>0.28 0.28 0.27 1</ambient>
            <diffuse>0.42 0.41 0.38 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
''', encoding='ascii')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, help='Input grayscale-ish DEM/height image.')
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--size-m', type=float, default=120.0)
    parser.add_argument('--base-height-m', type=float, default=8.0)
    parser.add_argument('--resolution', type=int, default=97)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--craters', type=int, default=9)
    parser.add_argument('--rocks', type=int, default=36)
    parser.add_argument('--ridges', type=int, default=8)
    parser.add_argument('--rock-radius-min', type=float, default=0.18)
    parser.add_argument('--rock-radius-max', type=float, default=0.75)
    parser.add_argument('--rock-height-min', type=float, default=0.08)
    parser.add_argument('--rock-height-max', type=float, default=0.45)
    parser.add_argument('--fine-texture-m', type=float, default=0.16)
    parser.add_argument('--spawn-radius-m', type=float, default=8.0)
    parser.add_argument('--z-offset', type=float, default=-2.5)
    parser.add_argument('--ogre2-sensors', action='store_true')
    args = parser.parse_args()

    random.seed(args.seed)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    heights = load_base_heights(Path(args.image), args.resolution, args.base_height_m)
    margin = args.size_m * 0.12
    limit = args.size_m / 2.0 - margin

    for _ in range(args.craters):
        cx = random.uniform(-limit, limit)
        cy = random.uniform(-limit, limit)
        if math.hypot(cx, cy) < args.spawn_radius_m * 1.6:
            continue
        radius = random.uniform(args.size_m * 0.035, args.size_m * 0.11)
        depth = random.uniform(0.25, 1.2)
        rim = random.uniform(0.15, 0.75)
        add_crater(heights, args.size_m, cx, cy, radius, depth, rim)

    add_fine_texture(heights, args.fine_texture_m)

    for _ in range(args.ridges):
        cx = random.uniform(-limit, limit)
        cy = random.uniform(-limit, limit)
        angle = random.uniform(0, math.tau)
        length = random.uniform(args.size_m * 0.18, args.size_m * 0.55)
        width = random.uniform(0.8, 2.8)
        height = random.uniform(0.25, 1.0)
        add_ridge(heights, args.size_m, cx, cy, angle, length, width, height)

    for _ in range(args.rocks):
        cx = random.uniform(-limit, limit)
        cy = random.uniform(-limit, limit)
        if math.hypot(cx, cy) < args.spawn_radius_m * 1.25:
            continue
        radius = random.uniform(args.rock_radius_min, args.rock_radius_max)
        height = random.uniform(args.rock_height_min, args.rock_height_max)
        add_mound(heights, args.size_m, cx, cy, radius, height)

    flatten_spawn(heights, args.size_m, args.spawn_radius_m)
    normalize_floor(heights, args.z_offset)

    obj_path = out_dir / 'enhanced_lunar_terrain.obj'
    preview_path = out_dir / 'enhanced_lunar_height.png'
    world_path = out_dir / 'enhanced_lunar.world.sdf'
    save_height_preview(heights, preview_path)
    write_obj(heights, obj_path, args.size_m)
    write_world(obj_path, world_path, args.ogre2_sensors)

    print(f'Height preview: {preview_path}')
    print(f'OBJ mesh:       {obj_path}')
    print(f'World:          {world_path}')
    print(f'Try:            gz sim -r {world_path}')


if __name__ == '__main__':
    main()