#!/usr/bin/env python3
"""Convert a grayscale DEM image into an OBJ terrain mesh and Gazebo world."""

from pathlib import Path
import argparse
import math
import os
from PIL import Image


def write_obj(image_path: Path, obj_path: Path, size_m: float, height_m: float, resolution: int):
    img = Image.open(image_path).convert('L')
    img = img.resize((resolution, resolution), Image.Resampling.BILINEAR)
    pix = list(img.getdata())
    min_v = min(pix)
    max_v = max(pix)
    denom = max(max_v - min_v, 1)

    step = size_m / (resolution - 1)
    half = size_m / 2.0
    heights = [[0.0 for _ in range(resolution)] for _ in range(resolution)]
    for y in range(resolution):
        for x in range(resolution):
            value = pix[y * resolution + x]
            heights[y][x] = ((value - min_v) / denom) * height_m

    def normal_at(x: int, y: int):
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
        f.write('# Lunar DEM mesh generated from LOLA heightmap\n')
        f.write('o lunar_dem_patch\n')
        for y in range(resolution):
            for x in range(resolution):
                z = heights[y][x]
                wx = x * step - half
                wy = half - y * step
                f.write(f'v {wx:.5f} {wy:.5f} {z:.5f}\n')
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


def write_world(mesh_uri: str, world_path: Path, z_offset: float):
    world_path.write_text(f'''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="lunar_mesh_test">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.02 0.02 0.025 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun_low_angle">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0.4 0.25 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.65 0.25 -0.72</direction>
    </light>

    <model name="lunar_mesh_patch">
      <pose>0 0 {z_offset:.3f} 0 0 0</pose>
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.45 0.43 0.39 1</ambient>
            <diffuse>0.62 0.60 0.55 1</diffuse>
            <specular>0.03 0.03 0.03 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
''', encoding='ascii')


def main():
    repo_root = Path(__file__).resolve().parent.parent
    package_dir = repo_root / 'src' / 'togo' / 'togo_gz'

    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True, help='Grayscale source image or DEM preview')
    parser.add_argument('--mesh-dir', type=Path, default=package_dir / 'media')
    parser.add_argument('--world-dir', type=Path, default=package_dir / 'worlds')
    parser.add_argument('--mesh-name', default='lunar_terrain.obj')
    parser.add_argument('--world-name', default='lunar_mesh.world.sdf')
    parser.add_argument('--size-m', type=float, default=350.0)
    parser.add_argument('--height-m', type=float, default=25.0)
    parser.add_argument('--resolution', type=int, default=129)
    parser.add_argument('--z-offset', type=float, default=-8.0)
    args = parser.parse_args()

    image_path = Path(args.image).expanduser().resolve()
    if not image_path.is_file():
        parser.error(f'image does not exist: {image_path}')

    mesh_dir = args.mesh_dir.expanduser().resolve()
    world_dir = args.world_dir.expanduser().resolve()
    mesh_dir.mkdir(parents=True, exist_ok=True)
    world_dir.mkdir(parents=True, exist_ok=True)
    obj_path = mesh_dir / args.mesh_name
    world_path = world_dir / args.world_name

    # Keep the SDF portable after colcon installs the package. The default
    # media/ and worlds/ directories are siblings in both source and install.
    mesh_uri = Path(os.path.relpath(obj_path, world_path.parent)).as_posix()

    write_obj(image_path, obj_path, args.size_m, args.height_m, args.resolution)
    write_world(mesh_uri, world_path, args.z_offset)
    print(f'OBJ mesh: {obj_path}')
    print(f'World:    {world_path}')
    print(f'Try:      gz sim -r {world_path}')


if __name__ == '__main__':
    main()
