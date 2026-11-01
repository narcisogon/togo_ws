#!/usr/bin/env python3
"""Fixed-joint offset between two links of a URDF: the transform of TO expressed in FROM (xyz in m, rpy in rad and
deg, and the 4x4), e.g. the pose of a sensor mount in base_link, to fill extrinsics in a SLAM config.

usage: urdf_frame_offset.py URDF FROM TO [TO ...]
       xacro robot.urdf.xacro | urdf_frame_offset.py - base_link stereo_rig_imu_link

Only fixed joints are followed (a moving joint on the path is an error). Pure stdlib.
"""
import math
import sys
import xml.etree.ElementTree as ET


def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_to_rpy(m):
    p = -math.asin(max(-1.0, min(1.0, m[2][0])))
    r = math.atan2(m[2][1], m[2][2])
    y = math.atan2(m[1][0], m[0][0])
    return r, p, y


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def pose_matrix(xyz, rpy):
    m = rpy_to_matrix(*rpy)
    return [[m[0][0], m[0][1], m[0][2], xyz[0]], [m[1][0], m[1][1], m[1][2], xyz[1]],
            [m[2][0], m[2][1], m[2][2], xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def invert(t):
    r = [[t[j][i] for j in range(3)] for i in range(3)]
    p = [-sum(r[i][k] * t[k][3] for k in range(3)) for i in range(3)]
    return [[r[0][0], r[0][1], r[0][2], p[0]], [r[1][0], r[1][1], r[1][2], p[1]],
            [r[2][0], r[2][1], r[2][2], p[2]], [0.0, 0.0, 0.0, 1.0]]


def load(path):
    text = sys.stdin.read() if path == "-" else open(path, encoding="utf-8").read()
    root = ET.fromstring(text)
    joints = {}
    for j in root.iter("joint"):
        if j.find("parent") is None or j.find("child") is None:
            continue   # <joint> entries of <ros2_control> / <gazebo> blocks are not kinematic joints
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        joints[child] = (parent, j.get("type"), pose_matrix(xyz, rpy))
    return joints


def chain(joints, link):
    """transform of link in the root frame, following parents up to the root"""
    t = pose_matrix([0, 0, 0], [0, 0, 0])
    while link in joints:
        parent, jtype, tj = joints[link]
        if jtype != "fixed":
            raise SystemExit(f"joint to {link} is {jtype}, not fixed")
        t = mul(tj, t)
        link = parent
    return t, link


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(2)
    joints = load(sys.argv[1])
    src = sys.argv[2]
    t_src, root_src = chain(joints, src)
    for dst in sys.argv[3:]:
        t_dst, root_dst = chain(joints, dst)
        if root_src != root_dst:
            raise SystemExit(f"{src} and {dst} do not share a root ({root_src} vs {root_dst})")
        t = mul(invert(t_src), t_dst)
        r, p, y = matrix_to_rpy(t)
        print(f"{dst} in {src}:")
        print(f"  xyz {t[0][3]:.6f} {t[1][3]:.6f} {t[2][3]:.6f}")
        print(f"  rpy {r:.6f} {p:.6f} {y:.6f} rad  ({math.degrees(r):.3f} {math.degrees(p):.3f} {math.degrees(y):.3f} deg)")
        print("  R   " + "  ".join(f"{t[i][j]:9.6f}" for i in range(3) for j in range(3)))


if __name__ == "__main__":
    main()
