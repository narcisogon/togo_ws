#!/usr/bin/env python3
"""
transform_and_view.py

Reads poses.csv, applies each pose to the matching frame_XXXXXX.xyz,
combines into a single point cloud, colours by frame index (time),
and renders it with Open3D.

Usage:
    python3 transform_and_view.py
    python3 transform_and_view.py --xyz_dir run_20260812_142514 \
                                  --poses   run_20260812_142514/poses.csv \
                                  --save    run_20260812_142514/combined_transformed.pcd \
                                  --color   time      # or: submap
"""

import argparse
import csv
import glob
import os
import sys
import numpy as np
import open3d as o3d


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def quat_to_rot(qw, qx, qy, qz):
    """Unit-quaternion (w, x, y, z) → 3x3 rotation matrix."""
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def load_poses(csv_path):
    """Return dict  cloud_id (int) → (R 3x3, t 3,)."""
    poses = {}
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            cid = int(row['cloud_id'])
            t   = np.array([float(row['px']),
                             float(row['py']),
                             float(row['pz'])])
            R   = quat_to_rot(float(row['qw']),
                               float(row['qx']),
                               float(row['qy']),
                               float(row['qz']))
            poses[cid] = (R, t)
    return poses


def load_xyz(path):
    """
    Load an .xyz file.  Accepts:  x y z  /  x y z i  /  x y z r g b
    Returns float64 array of shape (N, 3).
    """
    pts = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = line.split()
            if len(vals) < 3:
                continue
            pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
    return np.array(pts, dtype=np.float64)


def frame_index_from_name(filename):
    """frame_000042.xyz → 42,  None if pattern doesn't match."""
    stem = os.path.splitext(os.path.basename(filename))[0]
    if not stem.startswith('frame_'):
        return None
    try:
        return int(stem.split('_')[1])
    except (IndexError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Colormap utilities
# ---------------------------------------------------------------------------

def turbo(t):
    """
    Google Turbo colormap (black-free, perceptually uniform).
    t : float array in [0, 1]
    Returns (N, 3) float64 RGB in [0, 1].
    """
    t = np.clip(t, 0.0, 1.0)
    r = ( 0.1357 + t * ( 4.5974 + t * (-42.3277 + t * ( 130.5887 + t * (-150.5799 + t *  52.3148)))))
    g = ( 0.0914 + t * ( 2.1856 + t * (  4.8052 + t * ( -14.0741 + t * (  14.3765 + t *  -5.9244)))))
    b = ( 0.1071 + t * (12.5925 + t * (-60.1097 + t * ( 109.0528 + t * ( -88.5015 + t *  26.9475)))))
    return np.clip(np.stack([r, g, b], axis=1), 0.0, 1.0)


# How many distinct colours to cycle through for submap mode
SUBMAP_SIZE = 30    # frames per submap  ← tweak to taste


def colorize_by_time(frame_ids):
    """
    Smooth gradient across all frames: early = cool (purple/blue),
    late = warm (yellow/red) using the Turbo colormap.
    frame_ids : 1-D int array, one entry per point.
    """
    ids = frame_ids.astype(np.float64)
    t   = (ids - ids.min()) / (ids.max() - ids.min() + 1e-12)
    return turbo(t)


def colorize_by_submap(frame_ids, submap_size=SUBMAP_SIZE):
    """
    Each group of `submap_size` consecutive frames gets a distinct colour.
    Colours cycle through the Turbo colormap.
    frame_ids : 1-D int array, one entry per point.
    """
    submap_idx = (frame_ids // submap_size).astype(np.float64)
    n_submaps  = int(submap_idx.max()) + 1
    t = (submap_idx % n_submaps) / max(n_submaps - 1, 1)
    return turbo(t)


# ---------------------------------------------------------------------------
# Build combined point cloud
# ---------------------------------------------------------------------------

def build_combined_pcd(xyz_dir, poses_path):
    """Returns (points Nx3 float64, frame_ids N int64)."""
    poses = load_poses(poses_path)
    print(f"Loaded {len(poses)} poses from {poses_path}")

    xyz_files = sorted(glob.glob(os.path.join(xyz_dir, 'frame_*.xyz')))
    if not xyz_files:
        sys.exit(f"ERROR: no frame_*.xyz files found in {xyz_dir}")
    print(f"Found {len(xyz_files)} xyz files — transforming...")

    all_pts = []
    all_ids = []
    skipped = 0

    for xyz_path in xyz_files:
        cid = frame_index_from_name(xyz_path)
        if cid is None or cid not in poses:
            skipped += 1
            continue

        pts = load_xyz(xyz_path)
        if pts.size == 0:
            skipped += 1
            continue

        R, t = poses[cid]
        all_pts.append((R @ pts.T).T + t)           # p_world = R @ p_local + t
        all_ids.append(np.full(len(pts), cid, dtype=np.int64))

        if cid % 50 == 0:
            print(f"  frame {cid:06d}: {len(pts):>7,} pts")

    if not all_pts:
        sys.exit("ERROR: no frames were successfully transformed.")

    points    = np.vstack(all_pts)
    frame_ids = np.concatenate(all_ids)
    print(f"\nCombined: {len(points):,} points  ({skipped} frames skipped)\n")
    return points, frame_ids


# ---------------------------------------------------------------------------
# Render  (no auto-spin)
# ---------------------------------------------------------------------------

def render(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="PointCloud Viewer", width=1280, height=720)
    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.background_color = np.array([0, 0, 0])
    opt.point_size = 1.0

    # Plain event loop — mouse / trackpad to navigate
    vis.run()
    vis.destroy_window()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Transform xyz frames by poses and view with Open3D")
    parser.add_argument('--xyz_dir', default='.',
                        help="Directory with frame_*.xyz files  (default: .)")
    parser.add_argument('--poses',   default='poses.csv',
                        help="Path to poses.csv  (default: poses.csv)")
    parser.add_argument('--save',    default=None,
                        help="Optional: save combined PCD to this path before viewing")
    parser.add_argument('--color',   default='time',
                        choices=['time', 'submap'],
                        help="Colour mode: 'time' gradient or 'submap' blocks  (default: time)")
    parser.add_argument('--submap_size', type=int, default=SUBMAP_SIZE,
                        help=f"Frames per submap when --color submap  (default: {SUBMAP_SIZE})")
    args = parser.parse_args()

    # ── Build ────────────────────────────────────────────────────────────────
    points, frame_ids = build_combined_pcd(args.xyz_dir, args.poses)

    # ── Colorize ─────────────────────────────────────────────────────────────
    if args.color == 'time':
        print("Colouring by time (frame index)…")
        colors = colorize_by_time(frame_ids)
    else:
        n_sub = int(frame_ids.max() // args.submap_size) + 1
        print(f"Colouring by submap  ({args.submap_size} frames/submap → {n_sub} submaps)…")
        colors = colorize_by_submap(frame_ids, args.submap_size)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # ── Optionally save ──────────────────────────────────────────────────────
    if args.save:
        o3d.io.write_point_cloud(args.save, pcd)
        print(f"Saved → {args.save}")

    # ── Render ───────────────────────────────────────────────────────────────
    print("Launching viewer  (close window to exit)")
    print("  Mouse: left-drag = rotate | right-drag = pan | scroll = zoom")
    render(pcd)


if __name__ == '__main__':
    main()
