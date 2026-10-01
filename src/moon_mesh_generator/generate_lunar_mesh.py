"""
generate_lunar_mesh.py  (v5 — 50m world, high resolution)

v5 changes:
  - GRID_RES 513→1025  (0.049m vertex spacing, was 0.098m)
  - Heightmap loaded at full 4096 float precision before resize
  - All v4 fixes retained (16-bit float path, BICUBIC resize)

Warning: DAE file will be ~4x larger than v4 (~400–600MB).
If file size is a concern drop GRID_RES back to 769 (0.065m spacing).

Requires: numpy, Pillow
"""

import numpy as np
from PIL import Image
import os
from pathlib import Path

# ── Configuration ─────────────────────────────────────────────────────────────
SCRIPT_DIR    = Path(__file__).resolve().parent
PACKAGE_MEDIA = SCRIPT_DIR.parent / "togo" / "togo_gz" / "media"
HEIGHTMAP_PNG = SCRIPT_DIR / "lunar_terrain.png"
OUTPUT_DAE    = PACKAGE_MEDIA / "lunar_terrain.dae"
COLLISION_DAE = PACKAGE_MEDIA / "lunar_terrain_collision.dae"

WORLD_SIZE    = 30.0
MAX_HEIGHT    = 1.0

# Resolution options:
#   513  → 0.098m spacing, ~150MB DAE  (original)
#   769  → 0.065m spacing, ~330MB DAE  (good balance)
#   1025 → 0.049m spacing, ~590MB DAE  (full detail)
GRID_RES      = 769    

# Keep physics substantially lighter than the render / lidar mesh. Gazebo
# Harmonic's default DART SDF loader cannot create mesh or heightmap collision
# shapes, so lunar_surface.world selects Bullet Featherstone and uses this DAE.
COLLISION_GRID_RES = 257

COLOUR        = (0.42, 0.40, 0.38, 1.0)


# ── Load and sample heightmap ─────────────────────────────────────────────────
def sample_heightmap(path: str, grid_res: int) -> np.ndarray:
    """
    v4: 16-bit PNG decoded to float32, resized in float space (no uint8 cast).
    v5: Same path, GRID_RES doubled for finer mesh.
    """
    print(f"Loading heightmap: {path}")
    img = Image.open(path)

    if img.mode in ('I;16', 'I'):
        arr = np.array(img, dtype=np.float64) / 65535.0
    elif img.mode == 'L':
        arr = np.array(img, dtype=np.float64) / 255.0
    else:
        arr = np.array(img.convert('L'), dtype=np.float64) / 255.0

    # Resize in float32 — no uint8 intermediate
    img_f       = Image.fromarray(arr.astype(np.float32), mode='F')
    img_resized = img_f.resize((grid_res, grid_res), Image.BICUBIC)
    arr_resized = np.array(img_resized, dtype=np.float64)
    arr_resized = np.clip(arr_resized, 0.0, 1.0)

    relief = (arr_resized.max() - arr_resized.min()) * MAX_HEIGHT
    spacing = WORLD_SIZE / (grid_res - 1)
    print(f"  Vertex spacing : {spacing:.4f}m")
    print(f"  Total relief   : {relief:.4f}m")

    return arr_resized * MAX_HEIGHT


# ── Build mesh ────────────────────────────────────────────────────────────────
def build_mesh(heights: np.ndarray, world_size: float):
    n    = heights.shape[0]
    half = world_size / 2.0
    n_tris = (n - 1) ** 2 * 2
    print(f"Building {n}×{n} mesh ({n_tris:,} triangles) …")

    xv     = np.linspace(-half, half, n)
    yv     = np.linspace(-half, half, n)
    XX, YY = np.meshgrid(xv, yv)
    verts  = np.stack([XX.ravel(),
                       YY.ravel(),
                       heights.ravel()], axis=1).astype(np.float32)

    uu  = (XX - XX.min()) / (XX.max() - XX.min())
    vv  = (YY - YY.min()) / (YY.max() - YY.min())
    uvs = np.stack([uu.ravel(), vv.ravel()], axis=1).astype(np.float32)

    # CCW winding — normals point UP
    idx = []
    for r in range(n - 1):
        for c in range(n - 1):
            tl = r * n + c
            tr = tl + 1
            bl = tl + n
            br = bl + 1
            idx.extend([tl, tr, bl])
            idx.extend([tr, br, bl])
    indices = np.array(idx, dtype=np.int32)

    # Smooth per-vertex normals
    normals = np.zeros_like(verts)
    v0 = verts[indices[0::3]]
    v1 = verts[indices[1::3]]
    v2 = verts[indices[2::3]]
    fn = np.cross(v1 - v0, v2 - v0)
    np.add.at(normals, indices[0::3], fn)
    np.add.at(normals, indices[1::3], fn)
    np.add.at(normals, indices[2::3], fn)
    lengths  = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths  = np.where(lengths == 0, 1, lengths)
    normals /= lengths
    normals  = normals.astype(np.float32)

    return verts, normals, uvs, indices


# ── Write Collada ─────────────────────────────────────────────────────────────
def write_collada(path, verts, normals, uvs, indices, colour):
    print(f"Writing Collada: {path} …")
    n_verts = verts.shape[0]
    n_tris  = indices.shape[0] // 3
    r, g, b, a = colour

    def fa(arr):
        return " ".join(f"{v:.6f}" for v in arr.ravel())

    p = []
    for i in range(0, len(indices), 3):
        for j in range(3):
            vi = indices[i + j]
            p += [str(vi), str(vi), str(vi)]
    p_str = " ".join(p)

    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <created>2026-07-29</created>
    <modified>2026-07-29</modified>
    <unit name="metre" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="regolith_fx">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>{r} {g} {b} {a}</color></diffuse>
            <reflectivity><float>0.03</float></reflectivity>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="regolith_mat" name="regolith">
      <instance_effect url="#regolith_fx"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="terrain_mesh" name="lunar_terrain">
      <mesh>
        <source id="pos">
          <float_array id="pos_arr" count="{n_verts * 3}">{fa(verts)}</float_array>
          <technique_common>
            <accessor source="#pos_arr" count="{n_verts}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="nrm">
          <float_array id="nrm_arr" count="{n_verts * 3}">{fa(normals)}</float_array>
          <technique_common>
            <accessor source="#nrm_arr" count="{n_verts}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="uvs">
          <float_array id="uvs_arr" count="{n_verts * 2}">{fa(uvs)}</float_array>
          <technique_common>
            <accessor source="#uvs_arr" count="{n_verts}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="verts">
          <input semantic="POSITION" source="#pos"/>
        </vertices>
        <triangles count="{n_tris}" material="regolith_mat">
          <input semantic="VERTEX"   source="#verts" offset="0"/>
          <input semantic="NORMAL"   source="#nrm"   offset="1"/>
          <input semantic="TEXCOORD" source="#uvs"   offset="2" set="0"/>
          <p>{p_str}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="terrain_node" name="lunar_terrain" type="NODE">
        <instance_geometry url="#terrain_mesh">
          <bind_material>
            <technique_common>
              <instance_material symbol="regolith_mat"
                                 target="#regolith_mat"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(dae)

    size_mb = os.path.getsize(path) / 1024 / 1024
    print(f"✓ Saved: {path}  ({size_mb:.1f} MB, {n_tris:,} triangles)")


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    PACKAGE_MEDIA.mkdir(parents=True, exist_ok=True)
    collision_heights = sample_heightmap(HEIGHTMAP_PNG, COLLISION_GRID_RES)
    collision_mesh = build_mesh(collision_heights, WORLD_SIZE)
    write_collada(COLLISION_DAE, *collision_mesh, COLOUR)

    heights = sample_heightmap(HEIGHTMAP_PNG, GRID_RES)
    verts, normals, uvs, indices = build_mesh(heights, WORLD_SIZE)
    write_collada(OUTPUT_DAE, verts, normals, uvs, indices, COLOUR)
    spacing = WORLD_SIZE / (GRID_RES - 1)
    print(f"\n  Vertex spacing : {spacing:.4f}m")
    print(f"  Grid resolution: {GRID_RES}×{GRID_RES}")
    print(f"  Triangles      : {(GRID_RES-1)**2 * 2:,}")
    print("\nDone!")
