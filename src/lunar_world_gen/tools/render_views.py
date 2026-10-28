#!/usr/bin/python3
"""Render camera views of a world SDF headless in Gazebo Harmonic and save one PNG per camera.

Run with the SYSTEM python3 (gz-transport13 / gz-msgs10 bindings and PIL), from WSL, in
``src/lunar_world_gen``:

    /usr/bin/python3 tools/render_views.py WORLD.sdf --out DIR \
        [--view NAME:X,Y,Z,YAW,PITCH[,FOV[,WIDTH,HEIGHT]]]... [--views-file views.json] \
        [--agl] [--config CFG | --grid PREFIX | --ground-from-sdf] \
        [--width 1280] [--height 720] [--fov 70] [--near 0.05] [--far 30000] \
        [--rate 2] [--skip 3] [--stable-tol 0.25] [--max-frames 12] [--batch 8] [--timeout 300] \
        [--light-intensity NAME=VALUE]... [--ambient V] [--partition P] [--domain D] \
        [--sheet] [--dry-run]

Conventions. Angles are degrees. ``yaw`` is the heading of the optical axis measured from +x (east)
toward +y (north): 0 = east, 90 = north, 180 = west, -90 = south. ``pitch`` is POSITIVE DOWN
(Gazebo link pitch: a camera looks along its link +x, and +pitch turns +x toward -z; 90 = nadir).
``fov`` is the horizontal field of view. With ``--agl`` (or ``"agl": true`` in a views file) z is a
height above the ground under the camera, taken from the generator build named by ``--config`` (the
final 1 cm core tiles + the 25 cm ring grid), a height grid (``--grid``) or the world's own static GLB
visuals (``--ground-from-sdf``). A views-file entry may give ``look_at: [x, y, z]`` (with
``look_at_agl: true``, z above the ground there) instead of yaw/pitch.

Views file: a JSON list of objects with keys ``name`` (letters, digits, _), ``x``, ``y``, ``z``,
optional ``agl``, ``yaw``, ``pitch``, ``fov``, ``width``, ``height``, ``near``, ``far``,
``look_at``, ``look_at_agl`` and a free-text ``note``.

What it does, per batch of ``--batch`` cameras:
  1. writes a copy of the SDF (``DIR/run_kkk/views_<world>.sdf``): relative URIs made absolute, the
     Sensors system (ogre2) added if the world lacks it (generated worlds leave it to the robot), one
     static model with one ``camera`` sensor per view (image topic ``/render_views/<name>/image``),
     and the optional light / ambient overrides;
  2. starts ``timeout --foreground -k 10 LIMIT gz sim -s -r --headless-rendering -v 4 COPY`` in its
     own session (setsid) with the WSL GPU environment (GALLIUM_DRIVER=d3d12,
     LD_LIBRARY_PATH=/usr/lib/wsl/lib, MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA), its own GZ_PARTITION /
     ROS_DOMAIN_ID and a private ROS_HOME (``tools/bench_world.py`` helpers);
  3. subscribes to every camera, skips the first ``--skip`` frames, then keeps receiving until two
     consecutive frames differ by at most ``--stable-tol`` grey levels (mean absolute difference: a
     static camera in a static world repeats its image once all meshes and textures are loaded) or
     ``--max-frames`` frames were seen; the last frame is written to ``DIR/<name>.png``;
  4. checks the renderer in /proc/<pid>/maps (libd3d12 = GPU through D3D12; llvmpipe/swrast =
     software), records [Err]/[Wrn] lines, load times and peak RSS, and always stops the process
     group (SIGTERM, wait, SIGKILL). It never signals any other process.

``DIR/render_run.json`` holds the resolved poses, intrinsics, frame counts, frame-to-frame
differences, image statistics (sRGB luma percentiles, crushed and clipped fractions) and the
renderer evidence; ``--sheet`` adds ``DIR/contact_sheet.png``. Exit code 0 when every view was saved
from a stable image on the GPU renderer, 1 otherwise. ``--dry-run`` writes the SDF copies and the
resolved views without starting Gazebo.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import bench_world as bw  # noqa: E402

DEFAULT_PARTITION = "lunar_render_views"
DEFAULT_DOMAIN = 76
TOPIC_PREFIX = "/render_views"
NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")

# gz.msgs.PixelFormatType values (gz-msgs10 image.proto)
PF_L_INT8, PF_RGB_INT8, PF_RGBA_INT8, PF_BGRA_INT8, PF_BGR_INT8 = 1, 3, 4, 5, 8


# ==============================================================================================
# views

@dataclass
class View:
    name: str
    x: float
    y: float
    z: float
    agl: bool = False
    yaw_deg: float | None = None
    pitch_deg: float | None = None
    fov_deg: float = 70.0
    width: int = 1280
    height: int = 720
    near: float = 0.05
    far: float = 30000.0
    look_at: tuple | None = None
    look_at_agl: bool = False
    note: str = ""
    # filled by resolve_view
    pose: tuple | None = None          # SDF pose x, y, z, roll, pitch, yaw (rad)
    ground_z: float | None = None
    target: tuple | None = None

    def record(self) -> dict:
        return {"name": self.name, "request": {"x": self.x, "y": self.y, "z": self.z, "agl": self.agl,
                                               "look_at": list(self.look_at) if self.look_at else None,
                                               "look_at_agl": self.look_at_agl},
                "pose_sdf": [round(t, 6) for t in self.pose] if self.pose else None,
                "camera_xyz": [round(t, 4) for t in self.pose[:3]] if self.pose else None,
                "yaw_deg": round(self.yaw_deg, 3) if self.yaw_deg is not None else None,
                "pitch_down_deg": round(self.pitch_deg, 3) if self.pitch_deg is not None else None,
                "ground_z": None if self.ground_z is None else round(self.ground_z, 4),
                "target": [round(t, 4) for t in self.target] if self.target else None,
                "fov_deg": self.fov_deg, "width": self.width, "height": self.height,
                "near": self.near, "far": self.far, "intrinsics": intrinsics(self), "note": self.note}


def intrinsics(v: View) -> dict:
    fx = v.width / 2.0 / math.tan(math.radians(v.fov_deg) / 2.0)
    return {"fx": round(fx, 3), "fy": round(fx, 3), "cx": v.width / 2.0, "cy": v.height / 2.0,
            "vfov_deg": round(math.degrees(2.0 * math.atan(v.height / 2.0 / fx)), 3)}


def parse_view_arg(text: str, defaults: dict, agl: bool) -> View:
    """``NAME:X,Y,Z,YAW,PITCH[,FOV[,WIDTH,HEIGHT]]`` (degrees; pitch positive down)."""
    name, sep, rest = text.partition(":")
    if not sep:
        raise ValueError(f"--view {text!r}: expected NAME:X,Y,Z,YAW,PITCH[,FOV[,W,H]]")
    vals = [float(t) for t in rest.replace(" ", "").split(",") if t != ""]
    if len(vals) < 5:
        raise ValueError(f"--view {text!r}: needs at least X,Y,Z,YAW,PITCH")
    v = View(name=name.strip(), x=vals[0], y=vals[1], z=vals[2], agl=agl, yaw_deg=vals[3], pitch_deg=vals[4],
             **defaults)
    if len(vals) >= 6:
        v.fov_deg = vals[5]
    if len(vals) >= 8:
        v.width, v.height = int(vals[6]), int(vals[7])
    return v


def view_from_dict(d: dict, defaults: dict, agl: bool) -> View:
    kw = dict(defaults)
    for key, alias in (("fov_deg", "fov"), ("width", "width"), ("height", "height"), ("near", "near"),
                       ("far", "far")):
        if alias in d:
            kw[key] = d[alias]
        elif key in d:
            kw[key] = d[key]
    look = d.get("look_at")
    v = View(name=str(d["name"]), x=float(d["x"]), y=float(d["y"]), z=float(d.get("z", d.get("h", 0.0))),
             agl=bool(d.get("agl", agl)), yaw_deg=d.get("yaw", d.get("yaw_deg")),
             pitch_deg=d.get("pitch", d.get("pitch_deg")),
             look_at=tuple(float(t) for t in look) if look is not None else None,
             look_at_agl=bool(d.get("look_at_agl", False)), note=str(d.get("note", "")), **kw)
    v.fov_deg, v.width, v.height = float(v.fov_deg), int(v.width), int(v.height)
    v.near, v.far = float(v.near), float(v.far)
    if v.look_at is not None and len(v.look_at) != 3:
        raise ValueError(f"view {v.name}: look_at needs [x, y, z]")
    if v.look_at is None and (v.yaw_deg is None or v.pitch_deg is None):
        raise ValueError(f"view {v.name}: give yaw and pitch, or look_at")
    return v


def _ground(hfn, x: float, y: float, what: str) -> float:
    g = hfn(x, y) if hfn is not None else float("nan")
    if not math.isfinite(g):
        raise ValueError(f"{what}: no ground height at ({x:.3f}, {y:.3f}) (give --config, --grid or "
                         f"--ground-from-sdf, or use absolute z)")
    return float(g)


def resolve_view(v: View, hfn) -> View:
    """Absolute camera position and SDF pose (roll 0, pitch positive down, yaw from +x)."""
    z = v.z
    if v.agl:
        v.ground_z = _ground(hfn, v.x, v.y, f"view {v.name}")
        z = v.ground_z + v.z
    if v.look_at is not None:
        tx, ty, tz = v.look_at
        if v.look_at_agl:
            tz = _ground(hfn, tx, ty, f"view {v.name} look_at") + tz
        dx, dy, dz = tx - v.x, ty - v.y, tz - z
        if math.hypot(dx, dy) < 1e-9 and abs(dz) < 1e-9:
            raise ValueError(f"view {v.name}: look_at equals the camera position")
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else 0.0
        pitch = math.atan2(-dz, math.hypot(dx, dy))
        v.target = (tx, ty, tz)
    else:
        yaw, pitch = math.radians(float(v.yaw_deg)), math.radians(float(v.pitch_deg))
    v.yaw_deg, v.pitch_deg = math.degrees(yaw), math.degrees(pitch)
    v.pose = (v.x, v.y, z, 0.0, pitch, yaw)
    return v


def camera_model_xml(v: View, model_name: str, topic: str, rate: float, anti_aliasing: int) -> str:
    """A static model with one camera sensor at its link origin (optical axis = link +x)."""
    m = ET.Element("model", {"name": model_name})
    ET.SubElement(m, "static").text = "true"
    ET.SubElement(m, "pose").text = " ".join(f"{t:.6f}" for t in v.pose)
    link = ET.SubElement(m, "link", {"name": "link"})
    s = ET.SubElement(link, "sensor", {"name": "camera", "type": "camera"})
    ET.SubElement(s, "topic").text = topic
    ET.SubElement(s, "update_rate").text = f"{rate:g}"
    ET.SubElement(s, "always_on").text = "1"
    ET.SubElement(s, "visualize").text = "false"
    cam = ET.SubElement(s, "camera")
    ET.SubElement(cam, "horizontal_fov").text = repr(math.radians(v.fov_deg))
    im = ET.SubElement(cam, "image")
    ET.SubElement(im, "width").text = str(int(v.width))
    ET.SubElement(im, "height").text = str(int(v.height))
    ET.SubElement(im, "format").text = "R8G8B8"
    ET.SubElement(im, "anti_aliasing").text = str(int(anti_aliasing))
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = f"{v.near:g}"
    ET.SubElement(clip, "far").text = f"{v.far:g}"
    return ET.tostring(m, encoding="unicode")


def apply_overrides(sdf_path: Path, light_intensity: dict[str, float], ambient: float | None) -> dict:
    """Edit the copy in place: light intensities by name, scene ambient (grey). Returns what changed."""
    if not light_intensity and ambient is None:
        return {}
    tree = ET.parse(sdf_path)
    world = tree.getroot().find("world")
    done: dict = {}
    for light in world.iter("light"):
        name = light.get("name")
        if name in light_intensity:
            el = light.find("intensity")
            if el is None:
                el = ET.SubElement(light, "intensity")
            done[f"light:{name}"] = {"from": el.text, "to": light_intensity[name]}
            el.text = f"{light_intensity[name]:g}"
    missing = sorted(set(light_intensity) - {k.split(":", 1)[1] for k in done})
    if missing:
        raise ValueError(f"--light-intensity: no light named {missing} in {sdf_path}")
    if ambient is not None:
        scene = world.find("scene")
        if scene is None:
            scene = ET.SubElement(world, "scene")
        el = scene.find("ambient")
        if el is None:
            el = ET.SubElement(scene, "ambient")
        done["ambient"] = {"from": el.text, "to": ambient}
        el.text = f"{ambient:g} {ambient:g} {ambient:g} 1"
    tree.write(sdf_path, encoding="unicode", xml_declaration=True)
    return done


# ==============================================================================================
# images

def decode_image(msg) -> np.ndarray:
    """gz.msgs.Image -> (h, w, 3) uint8 RGB."""
    w, h, fmt = int(msg.width), int(msg.height), int(msg.pixel_format_type)
    buf = np.frombuffer(msg.data, np.uint8)
    step = int(msg.step) or (len(buf) // max(h, 1))
    if h <= 0 or w <= 0 or len(buf) < h * step:
        raise ValueError(f"image {w}x{h} step {step}: {len(buf)} bytes")
    rows = buf[: h * step].reshape(h, step)
    if fmt == PF_RGB_INT8:
        return rows[:, : w * 3].reshape(h, w, 3).copy()
    if fmt == PF_BGR_INT8:
        return rows[:, : w * 3].reshape(h, w, 3)[:, :, ::-1].copy()
    if fmt == PF_RGBA_INT8:
        return rows[:, : w * 4].reshape(h, w, 4)[:, :, :3].copy()
    if fmt == PF_BGRA_INT8:
        return rows[:, : w * 4].reshape(h, w, 4)[:, :, 2::-1].copy()
    if fmt == PF_L_INT8:
        return np.repeat(rows[:, :w][:, :, None], 3, axis=2)
    raise ValueError(f"unsupported pixel format {fmt}")


def image_stats(img: np.ndarray) -> dict:
    """sRGB-coded luma (Rec. 709 weights on the 8-bit values) and colour statistics."""
    f = img.astype(np.float32)
    y = 0.2126 * f[..., 0] + 0.7152 * f[..., 1] + 0.0722 * f[..., 2]
    p = np.percentile(y, [1, 5, 25, 50, 75, 95, 99])
    lit = y > 8
    mean_rgb_lit = [round(float(f[..., k][lit].mean()), 2) for k in range(3)] if lit.any() else None
    return {"luma_mean": round(float(y.mean()), 2),
            "luma_p1_p5_p25_p50_p75_p95_p99": [round(float(t), 1) for t in p],
            "frac_le_2": round(float((y <= 2).mean()), 4), "frac_le_8": round(float((y <= 8).mean()), 4),
            "frac_ge_253": round(float((y >= 253).mean()), 5),
            "mean_rgb": [round(float(f[..., k].mean()), 2) for k in range(3)],
            "mean_rgb_where_luma_gt_8": mean_rgb_lit}


def save_png(path: Path, img: np.ndarray) -> int:
    from PIL import Image as PILImage
    PILImage.fromarray(np.ascontiguousarray(img)).save(path, optimize=True)
    return path.stat().st_size


def contact_sheet(items: list[tuple[str, Path]], path: Path, thumb_w: int = 420, cols: int = 3) -> str | None:
    from PIL import Image as PILImage, ImageDraw
    thumbs = []
    for name, p in items:
        try:
            im = PILImage.open(p).convert("RGB")
        except OSError:
            continue
        h = max(1, round(im.height * thumb_w / im.width))
        thumbs.append((name, im.resize((thumb_w, h), PILImage.LANCZOS)))
    if not thumbs:
        return None
    rows = math.ceil(len(thumbs) / cols)
    cell_h = max(t.height for _, t in thumbs) + 18
    sheet = PILImage.new("RGB", (cols * (thumb_w + 6) + 6, rows * cell_h + 6), (40, 40, 40))
    draw = ImageDraw.Draw(sheet)
    for k, (name, t) in enumerate(thumbs):
        x0, y0 = 6 + (k % cols) * (thumb_w + 6), 6 + (k // cols) * cell_h
        sheet.paste(t, (x0, y0 + 16))
        draw.text((x0 + 2, y0 + 2), name, fill=(255, 220, 120))
    sheet.save(path, optimize=True)
    return str(path)


class Grabber:
    """Keeps the newest frame per camera topic until two consecutive frames agree."""

    def __init__(self, topics: list[str], skip: int, stable_tol: float, max_frames: int):
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node
        self.lock = threading.Lock()
        self.skip, self.tol, self.max_frames = int(skip), float(stable_tol), int(max_frames)
        self.topics = list(topics)
        self.st = {t: {"n": 0, "first": None, "last": None, "stamp": None, "wall": None, "diffs": [],
                       "done": False, "stable": False, "errors": []} for t in topics}
        self.node = Node()
        for t in topics:
            if not self.node.subscribe(Image, t, self._cb(t)):
                raise RuntimeError(f"cannot subscribe to {t}")

    def _cb(self, topic: str):
        def cb(msg) -> None:
            w = bw.clock()
            st = self.st[topic]
            with self.lock:
                st["n"] += 1
                n = st["n"]
                if st["first"] is None:
                    st["first"] = w
                if st["done"] or n <= self.skip:
                    return
                prev = st["last"]
            try:
                img = decode_image(msg)
            except ValueError as e:
                with self.lock:
                    st["errors"].append(str(e))
                    st["done"] = True
                return
            d = None
            if prev is not None and prev.shape == img.shape:
                d = float(np.abs(img.astype(np.int16) - prev.astype(np.int16)).mean())
            with self.lock:
                if d is not None:
                    st["diffs"].append(round(d, 4))
                    if d <= self.tol:
                        st["stable"] = True
                        st["done"] = True
                st["last"] = img
                st["stamp"] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
                st["wall"] = w
                if n - self.skip >= self.max_frames:
                    st["done"] = True
        return cb

    def all_done(self) -> bool:
        with self.lock:
            return all(s["done"] for s in self.st.values())

    def any_frame(self) -> bool:
        with self.lock:
            return any(s["n"] > 0 for s in self.st.values())

    def close(self) -> None:
        for t in self.topics:
            try:
                self.node.unsubscribe(t)
            except Exception:  # noqa: BLE001
                pass


# ==============================================================================================
# runs

def run_batch(a: argparse.Namespace, world: str, views: list[View], k0: int, out: Path) -> dict:
    work = out / f"run_{k0:03d}"
    copy = work / f"views_{Path(world).stem}.sdf"
    topics = [f"{TOPIC_PREFIX}/{v.name}/image" for v in views]
    models = [camera_model_xml(v, f"render_views_{v.name}", t, a.rate, a.anti_aliasing) for v, t in zip(views, topics)]
    rec: dict = {"views": [v.name for v in views], "status": "init", "sdf_copy": str(copy)}
    rec["sdf_edit"] = bw.prepare_world(world, copy, [], extra_models_xml=models)
    rec["overrides"] = apply_overrides(copy, a.light_intensity_map, a.ambient)
    if a.dry_run:
        rec["status"] = "dry_run"
        return rec
    grab = Grabber(topics, a.skip, a.stable_tol, a.max_frames)
    ros_home = Path(a.ros_home) if a.ros_home else work / "ros_home"
    env = bw.gz_env(a.partition, a.domain, ros_home)
    srv = bw.GzServer(copy, work / "server.log", env, limit_s=int(a.timeout + 60))
    renderer = None
    try:
        srv.start()
        rec["pgid"] = srv.pgid
        tend = srv.t0 + a.timeout
        while bw.clock() < tend:
            time.sleep(0.25)
            if srv.proc.poll() is not None and not srv.alive():
                rec["status"] = "server_exited"
                break
            if renderer is None and grab.any_frame():
                renderer = bw.renderer_libs([p for p, _ in bw.group_pids(srv.pgid)])
            if grab.all_done():
                rec["status"] = "ok"
                break
        else:
            rec["status"] = "timeout"
        pid = srv.find_server_pid()
        if pid:
            if renderer is None or not renderer.get("libs"):
                renderer = bw.renderer_libs([p for p, _ in bw.group_pids(srv.pgid)])
            ms = bw.mem_status(pid)
            rec["server_rss_peak_mb"] = round((ms.get("VmHWM") or 0) / 1024, 1)
        rec["renderer"] = renderer
    finally:
        grab.close()
        rec["stop"] = srv.stop() if srv.pgid else {}
        rec["log"] = srv.log_summary()
    rec["t_start_wall"] = srv.t0_wall
    results = []
    with grab.lock:
        states = {t: dict(s) for t, s in grab.st.items()}
    for v, t in zip(views, topics):
        s = states[t]
        r = {"name": v.name, "topic": t, "frames_seen": s["n"], "frame_diffs": s["diffs"], "stable": s["stable"],
             "errors": s["errors"], "first_frame_s": round(s["first"] - srv.t0, 2) if s["first"] else None,
             "saved_frame_s": round(s["wall"] - srv.t0, 2) if s["wall"] else None,
             "saved_frame_sim_time_s": s["stamp"], "file": None}
        if s["last"] is not None:
            png = out / f"{v.name}.png"
            r["bytes"] = save_png(png, s["last"])
            r["file"] = str(png)
            r["stats"] = image_stats(s["last"])
        results.append(r)
    rec["results"] = results
    return rec


def build_views(a: argparse.Namespace) -> list[View]:
    defaults = {"fov_deg": a.fov, "width": a.width, "height": a.height, "near": a.near, "far": a.far}
    views = [parse_view_arg(t, defaults, a.agl) for t in a.view]
    if a.views_file:
        for d in json.loads(Path(a.views_file).read_text()):
            views.append(view_from_dict(d, defaults, a.agl))
    if not views:
        raise ValueError("no views: give --view or --views-file")
    seen = set()
    for v in views:
        if not NAME_RE.match(v.name):
            raise ValueError(f"view name {v.name!r}: use letters, digits and _ only")
        if v.name in seen:
            raise ValueError(f"duplicate view name {v.name!r}")
        seen.add(v.name)
        if not (0.5 <= v.fov_deg <= 179.0):
            raise ValueError(f"view {v.name}: fov {v.fov_deg} deg out of range")
        if v.width < 16 or v.height < 16 or v.width * v.height > 4096 * 4096:
            raise ValueError(f"view {v.name}: bad resolution {v.width}x{v.height}")
        if not (0 < v.near < v.far):
            raise ValueError(f"view {v.name}: need 0 < near < far")
    return views


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world", help="world SDF (left unchanged; a copy is edited)")
    ap.add_argument("--out", required=True, help="output directory (PNGs, render_run.json, run_kkk/)")
    ap.add_argument("--view", action="append", default=[], help="NAME:X,Y,Z,YAW,PITCH[,FOV[,WIDTH,HEIGHT]] (deg)")
    ap.add_argument("--views-file", help="JSON list of view objects (see the module docstring)")
    ap.add_argument("--agl", action="store_true", help="z of --view (and default of views-file entries) is "
                                                       "height above the ground")
    ap.add_argument("--config", help="generator config: ground from its build (final 1 cm tiles + ring grid)")
    ap.add_argument("--grid", help="ground height grid prefix (lunar_world_gen.grid.Grid)")
    ap.add_argument("--ground-from-sdf", action="store_true", help="ground from the world's static GLB visuals")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fov", type=float, default=70.0, help="default horizontal FOV [deg]")
    ap.add_argument("--near", type=float, default=0.05)
    ap.add_argument("--far", type=float, default=30000.0)
    ap.add_argument("--anti-aliasing", type=int, default=4, help="MSAA samples (SDF image/anti_aliasing)")
    ap.add_argument("--rate", type=float, default=2.0, help="camera update rate [Hz, sim time]")
    ap.add_argument("--skip", type=int, default=3, help="frames skipped first per camera")
    ap.add_argument("--stable-tol", type=float, default=0.25, help="max mean |frame difference| [grey levels]")
    ap.add_argument("--max-frames", type=int, default=12, help="frames examined after the skipped ones")
    ap.add_argument("--batch", type=int, default=8, help="cameras per server run")
    ap.add_argument("--timeout", type=float, default=300.0, help="per batch: load + capture wall limit [s]")
    ap.add_argument("--light-intensity", action="append", default=[], metavar="NAME=VALUE",
                    help="override a light's <intensity> in the copy (e.g. sun=2.0)")
    ap.add_argument("--ambient", type=float, help="override the scene ambient (grey) in the copy")
    ap.add_argument("--allow-software", action="store_true", help="do not fail when the renderer is not D3D12")
    ap.add_argument("--partition", default=DEFAULT_PARTITION)
    ap.add_argument("--domain", type=int, default=DEFAULT_DOMAIN)
    ap.add_argument("--ros-home", help="private ROS_HOME (default DIR/run_kkk/ros_home)")
    ap.add_argument("--sheet", action="store_true", help="also write DIR/contact_sheet.png")
    ap.add_argument("--dry-run", action="store_true", help="write SDF copies and resolved views only")
    return ap


def main(argv=None) -> int:
    a = build_argparser().parse_args(argv)
    a.light_intensity_map = {}
    for t in a.light_intensity:
        k, sep, val = t.partition("=")
        if not sep:
            raise SystemExit(f"--light-intensity {t!r}: expected NAME=VALUE")
        a.light_intensity_map[k.strip()] = float(val)
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    world = str(Path(a.world).resolve())
    if not Path(world).is_file():
        raise SystemExit(f"world {world} does not exist")
    os.environ["GZ_PARTITION"] = a.partition       # the subscriber must be in the server's partition
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    views = build_views(a)
    ginfo: dict = {}
    hfn = None
    if any(v.agl or v.look_at_agl for v in views):
        sdf_ground = world if a.ground_from_sdf else None
        cfg = a.config if (a.config and not a.grid and not sdf_ground) else None
        hfn = bw.ground_height_fn(a.grid, sdf_ground, cfg, ginfo)
    for v in views:
        resolve_view(v, hfn)
    run = {"kind": "render_views", "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "world": world,
           "partition": a.partition, "domain": a.domain, "ground": ginfo, "clock": "monotonic",
           "settings": {k: getattr(a, k) for k in ("rate", "skip", "stable_tol", "max_frames", "batch", "timeout",
                                                   "anti_aliasing", "ambient")},
           "light_intensity": a.light_intensity_map, "views": [v.record() for v in views],
           "machine_before": None if a.dry_run else bw.machine_state(), "batches": []}

    def on_signal(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    ok = True
    try:
        for k0 in range(0, len(views), max(1, a.batch)):
            rec = run_batch(a, world, views[k0:k0 + a.batch], k0, out)
            run["batches"].append(rec)
            if a.dry_run:
                print(f"[render_views] dry run: wrote {rec['sdf_copy']}", flush=True)
                continue
            res = rec.get("results", [])
            rend = rec.get("renderer") or {}
            gpu_ok = bool(rend.get("d3d12")) and not bool(rend.get("software"))
            batch_ok = rec["status"] == "ok" and all(r["file"] and r["stable"] for r in res) and \
                (gpu_ok or a.allow_software)
            ok &= batch_ok
            print(f"[render_views] batch {k0}: {rec['status']} saved={sum(1 for r in res if r['file'])}/{len(res)} "
                  f"stable={sum(1 for r in res if r['stable'])} d3d12={rend.get('d3d12')} "
                  f"software={rend.get('software')} first_frame={min((r['first_frame_s'] for r in res if r['first_frame_s']), default=None)} s "
                  f"err={rec['log']['n_err']} left_in_group={rec['stop'].get('remaining_in_group')}", flush=True)
    except KeyboardInterrupt as e:
        run["status"] = f"interrupted ({e})"
        ok = False
    run["status"] = run.get("status", "dry_run" if a.dry_run else ("ok" if ok else "incomplete"))
    if not a.dry_run:
        run["machine_after"] = bw.machine_state()
        if a.sheet:
            items = [(r["name"], Path(r["file"])) for b in run["batches"] for r in b.get("results", []) if r.get("file")]
            run["contact_sheet"] = contact_sheet(items, out / "contact_sheet.png")
    (out / "render_run.json").write_text(json.dumps(run, indent=1, default=str))
    print(f"[render_views] wrote {out}/render_run.json status={run['status']}", flush=True)
    return 0 if (ok or a.dry_run) else 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except SystemExit as e:              # argparse / argument errors: show the message, keep the code
        if isinstance(e.code, int):
            code = e.code
        else:
            if e.code is not None:
                print(e.code, file=sys.stderr)
            code = 0 if e.code is None else 2
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)                   # gz-transport threads can keep a normal exit waiting
