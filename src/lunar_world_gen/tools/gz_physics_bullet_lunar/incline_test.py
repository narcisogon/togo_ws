#!/usr/bin/env python3
"""Friction test for MESH collisions: a box on a 20 deg incline mesh (DESIGN section 10).

For every (engine, ground mu) pair the script writes a world with
  * a 6 x 3 m incline, a GLB written in world coordinates (z = -tan(theta) x, so the box
    slides toward +x), carrying <surface><friction><ode><mu>/<mu2> = ground mu;
  * a 0.2 m, 1 kg box (primitive, mu 1) resting on it 15 mm above the surface (outside the
    10 mm Bullet margin), aligned with the slope;
runs ``gz sim -s -r`` headless for --duration sim seconds, records the box pose from the
SceneBroadcaster's ``/world/<name>/dynamic_pose/info`` and fits s(t) = s0 + v0 t + a t^2 / 2
along the slope after a short settling window.

Prediction (Coulomb, product rule mu_eff = mu_ground * mu_box = mu_ground):
  slides iff mu_eff < tan(theta) (0.364 at 20 deg) with a = g (sin theta - mu_eff cos theta);
  the measured mu_eff = tan(theta) - a / (g cos theta).
Stock classic Bullet ignores the mesh's SDF friction (body default 0.5): it sticks for every mu.

usage (system python3 with ROS 2 Jazzy sourced, for gz-transport):
  incline_test.py --out DIR [--plugin /abs/libgz-physics7-bullet-lunar-plugin.so]
                  [--mu 0.2 0.3 0.4 0.8] [--engines patched stock] [--duration 4] [--gravity 1.62]
Runs use their own GZ_PARTITION (default claude_world), setsid + process-group stop, and a
hard timeout per run. Writes DIR/results.json and DIR/results.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_PLUGIN = os.path.expanduser("~/togo_assets/lunar/plugins/libgz-physics7-bullet-lunar-plugin.so")
STOCK = "gz-physics-bullet-plugin"
BOX = 0.2
GAP = 0.015


# ----------------------------------------------------------------------------- GLB writer
def write_glb(path: Path, vertices: list[tuple[float, float, float]], normals, triangles,
              color=(0.5, 0.5, 0.5, 1.0)) -> None:
    """Minimal glTF 2.0 binary: one mesh/primitive, float32 positions + normals, uint32
    indices, one material with metallicFactor 0 (Z-up metres, no node transform)."""
    pos = b"".join(struct.pack("<3f", *v) for v in vertices)
    nrm = b"".join(struct.pack("<3f", *n) for n in normals)
    idx = b"".join(struct.pack("<3I", *t) for t in triangles)
    blob = pos + nrm + idx
    n = len(vertices)
    mins = [min(v[k] for v in vertices) for k in range(3)]
    maxs = [max(v[k] for v in vertices) for k in range(3)]
    gltf = {
        "asset": {"version": "2.0", "generator": "incline_test.py"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2, "material": 0}]}],
        "materials": [{"name": "ground", "pbrMetallicRoughness": {
            "baseColorFactor": list(color), "metallicFactor": 0.0, "roughnessFactor": 0.9}}],
        "buffers": [{"byteLength": len(blob)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos), "byteLength": len(nrm), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos) + len(nrm), "byteLength": len(idx), "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": n, "type": "VEC3", "min": mins, "max": maxs},
            {"bufferView": 1, "componentType": 5126, "count": n, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5125, "count": 3 * len(triangles), "type": "SCALAR"},
        ],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    blob += b"\0" * ((4 - len(blob) % 4) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob))
    out += struct.pack("<II", len(js), 0x4E4F534A) + js + struct.pack("<II", len(blob), 0x004E4942) + blob
    path.write_bytes(out)


def incline_mesh(theta: float, x0=-3.0, x1=3.0, y0=-1.5, y1=1.5, step=0.5):
    t = math.tan(theta)
    nx = int(round((x1 - x0) / step)) + 1
    ny = int(round((y1 - y0) / step)) + 1
    verts, norms, tris = [], [], []
    nrm = (math.sin(theta), 0.0, math.cos(theta))
    for j in range(ny):
        for i in range(nx):
            x = x0 + i * step
            y = y0 + j * step
            verts.append((x, y, -t * x))
            norms.append(nrm)
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            b, c, d = a + 1, a + nx, a + nx + 1
            tris += [(a, b, d), (a, d, c)]          # CCW seen from above
    return verts, norms, tris


# ----------------------------------------------------------------------------- world
def world_sdf(name: str, engine: str, mu: float, theta: float, gravity: float, mesh_uri: str) -> str:
    t = math.tan(theta)
    xs = -1.5
    zs = -t * xs
    n = (math.sin(theta), 0.0, math.cos(theta))
    off = BOX / 2 + GAP
    bx, by, bz = xs + n[0] * off, 0.0, zs + n[2] * off
    I = 1.0 * BOX * BOX / 6.0
    return f"""<?xml version="1.0"?>
<sdf version="1.11">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"><engine><filename>{engine}</filename></engine></plugin>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <gravity>0 0 -{gravity}</gravity>
    <model name="incline">
      <static>true</static>
      <link name="link">
        <collision name="incline_collision">
          <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="incline_visual"><geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry></visual>
      </link>
    </model>
    <model name="box">
      <pose>{bx:.6f} {by:.6f} {bz:.6f} 0 {theta:.8f} 0</pose>
      <link name="link">
        <inertial><mass>1</mass><inertia><ixx>{I:.6f}</ixx><iyy>{I:.6f}</iyy><izz>{I:.6f}</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry>
          <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry></visual>
      </link>
    </model>
  </world>
</sdf>
"""


# ----------------------------------------------------------------------------- run
def run_case(world: Path, name: str, duration: float, log: Path, timeout_s: float) -> list[tuple[float, float, float, float]]:
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V

    samples: list[tuple[float, float, float, float]] = []

    def on_pose(msg):
        for p in msg.pose:
            if p.name == "box":
                t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                samples.append((t, p.position.x, p.position.y, p.position.z))
                break

    node = Node()
    topic = f"/world/{name}/dynamic_pose/info"
    node.subscribe(Pose_V, topic, on_pose)
    with open(log, "w") as lf:
        proc = subprocess.Popen(["timeout", "-k", "5", str(int(timeout_s)), "gz", "sim", "-s", "-r", "-v", "4", str(world)],
                                stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        t0 = time.time()
        try:
            while time.time() - t0 < timeout_s:
                if samples and samples[-1][0] >= duration:
                    break
                if proc.poll() is not None:
                    break
                time.sleep(0.1)
        finally:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
    node.unsubscribe(topic)
    return samples


def analyse(samples, theta: float, gravity: float, settle: float = 0.5) -> dict:
    import numpy as np

    if len(samples) < 10:
        return {"status": "no data", "n": len(samples)}
    a = np.array(samples)
    t, x, z = a[:, 0], a[:, 1], a[:, 3]
    s = x * math.cos(theta) - z * math.sin(theta)     # distance down the slope
    m = t >= settle
    tt, ss = t[m] - t[m][0], s[m] - s[m][0]
    disp = float(ss[-1]) if ss.size else 0.0
    out = {"n": int(len(samples)), "t_end": round(float(t[-1]), 3), "displacement_m": round(disp, 4)}
    if ss.size > 5:
        A = np.column_stack([np.ones_like(tt), tt, 0.5 * tt ** 2])
        coef, *_ = np.linalg.lstsq(A, ss, rcond=None)
        acc = float(coef[2])
        out["accel_m_s2"] = round(acc, 4)
        out["mu_eff_measured"] = round(math.tan(theta) - acc / (gravity * math.cos(theta)), 4)
    out["slides"] = bool(disp > 0.02)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--plugin", default=DEFAULT_PLUGIN)
    ap.add_argument("--mu", type=float, nargs="+", default=[0.2, 0.3, 0.4, 0.8])
    ap.add_argument("--engines", nargs="+", default=["patched", "stock"], choices=["patched", "stock"])
    ap.add_argument("--theta-deg", type=float, default=20.0)
    ap.add_argument("--gravity", type=float, default=1.62)
    ap.add_argument("--duration", type=float, default=4.0, help="sim seconds per case")
    ap.add_argument("--timeout", type=float, default=60.0, help="wall seconds per case (hard limit)")
    args = ap.parse_args()
    os.environ.setdefault("GZ_PARTITION", "claude_world")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    theta = math.radians(args.theta_deg)
    mesh = out / "incline.glb"
    write_glb(mesh, *incline_mesh(theta))
    if "patched" in args.engines and not os.path.isfile(args.plugin):
        print(f"patched plugin not found: {args.plugin}", file=sys.stderr)
        return 2
    results = []
    for engine in args.engines:
        for mu in args.mu:
            name = f"incline_{engine}_mu{mu:g}".replace(".", "p")
            world = out / f"{name}.sdf"
            world.write_text(world_sdf(name, args.plugin if engine == "patched" else STOCK, mu, theta,
                                       args.gravity, str(mesh)))
            t0 = time.time()
            samples = run_case(world, name, args.duration, out / f"{name}.log", args.timeout)
            res = analyse(samples, theta, args.gravity)
            log = re.sub(r"\x1b\[[0-9;]*m", "", (out / f"{name}.log").read_text(errors="replace"))
            res.update({"engine": engine, "ground_mu": mu, "wall_s": round(time.time() - t0, 1),
                        "plugin_loaded": ("lunar" in log and "Loaded [gz::physics::bullet::Plugin]" in log)
                        if engine == "patched" else None,
                        "lunar_patch_lines": log.count("[lunar patch] mesh collision"),
                        "predicted_slides": (mu if engine == "patched" else 0.5) < math.tan(theta),
                        "predicted_accel_m_s2": round(max(0.0, args.gravity * (math.sin(theta) - (mu if engine == "patched" else 0.5) * math.cos(theta))), 4)})
            res["as_predicted"] = res.get("slides") == res["predicted_slides"]
            print(json.dumps(res), flush=True)
            results.append(res)
    (out / "results.json").write_text(json.dumps(results, indent=2))
    lines = [f"# incline test: {args.theta_deg:g} deg mesh incline, g {args.gravity}, box mu 1, tan = {math.tan(theta):.3f}", "",
             "| engine | ground mu | predicted | measured displacement (m) | accel (m/s2) pred / meas | mu_eff meas | as predicted |",
             "|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| {r['engine']} | {r['ground_mu']} | {'slides' if r['predicted_slides'] else 'sticks'} | "
                     f"{r.get('displacement_m')} | {r['predicted_accel_m_s2']} / {r.get('accel_m_s2')} | "
                     f"{r.get('mu_eff_measured') if r.get('slides') else '-'} | {r['as_predicted']} |")
    (out / "results.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    sys.stdout.flush()
    os._exit(0 if all(r["as_predicted"] for r in results) else 1)


if __name__ == "__main__":
    main()
