"""Apollo 16 lunar world + TOGO rover, with a robust start-up sequence (DESIGN section 12).

Run by file path (no package needed)::

    ros2 launch /path/to/src/lunar_world_gen/launch/apollo16_sim.launch.py \
        [world:=/abs/world.sdf] [variant:=|inspect|stock|inspect_stock] [site:=flat] \
        [robot_x:=X robot_y:=Y robot_z:=Z [robot_yaw:=RAD]] [robot_collision:=simplified|original] \
        [rviz:=false] [ready_file:=/abs/marker] ...

It reproduces ``togo_gz/launch/sim_gz.launch.py`` (+ ``start_world.launch.py`` and the
``togo_deploy`` ``control.launch.py`` it includes): the same gz server command as
``ros_gz_sim gz_sim.launch.py`` (``ruby gz sim -r -s -v 4 <world> --force-version 8`` with the
same plugin / resource paths), clock bridge, robot_state_publisher with the same xacro arguments
(``togo_gz.urdf.xacro ns:= tf_prefix:= is_sim:=true use_fake_hardware:=false``), ``ros_gz_sim
create -name togo -topic robot_description``, the ``sim_bridge`` with ``togo_gz/config/bridge.yaml``
and the same QoS overrides, and the two controller spawners (``joint_state_broadcaster``,
``platform_velocity_controller``), plus optional RViz (``togo_deploy robot_sensor_checkout.launch.py``,
lidar_topic ``/husky/sensors/seyond/points``) and the optional rgbd point fixer.

What differs:

* **Sequencing.** The stock chain starts the spawners at a fixed 10 s, while the robot's Sensors
  system is still building the render scene and the simulation is stalled; their switch_controller
  call then times out. Here:

  1. t = 0: gz server with an ABSOLUTE world path, clock bridge, robot_state_publisher, sim_bridge
     and ``create`` (which itself waits for the world service and the description).
  2. ``wait_for_sensors`` (this file run as a script with the system python and the gz python
     bindings) waits until the robot's LiDAR topic is advertised and the simulation time advances
     again (the render scene is loaded); it prints a progress line every 10 s.
  3. only then the joint_state_broadcaster spawner, and after it exits the
     platform_velocity_controller spawner, both with generous timeouts.
  4. after the velocity-controller spawner exits: ``wait_ready`` checks with
     ``list_controllers`` that both controllers are active and that ``/clock`` advances, and the
     launch prints ``[apollo16_sim] READY at +T s`` (or a WARNING with the next step), writes the
     ready marker if one was requested (below); RViz and the point fixer start then, if requested.

* **Robot collisions** (``robot_collision:=simplified``, default). The robot description is
  generated from the user's xacro exactly as before; its 14 CAD collision meshes (368,456
  triangles: 4 motors with 46,604 each, the Seyond Robin W body with 173,872, beams, chassis,
  spacers, bumpers, enclosure, arch) are then replaced in the XML by the bounding boxes precomputed
  by ``tools/robot_collision_simplify.py`` (``config/robot/a300_collision_simplified.yaml``): per
  mesh the fewest axis-aligned slab boxes that reach at most 15 mm further towards the ground than
  the mesh (88 boxes in all), each placed at ``<origin> * box pose`` in its own ``<collision>``.
  Classic Bullet runs every mesh as GImpact, and those meshes made the physics step cost several ms
  on slopes (gz_validation.md 5.1). Visuals and inertials are unchanged; a mesh whose file no longer
  matches the spec's sha256 is kept; ``robot_collision:=original`` keeps all meshes.

* **Shutdown** (``gz_guard:=true``, default). gz-sim 8 segfaults when it is stopped with
  SIGINT/SIGTERM, and WSL then writes a 1.5-3.6 GB crash dump to %TEMP%\\wsl-crashes. The server is
  therefore started by a small guard (this file with ``--gz-guard``): on SIGINT/SIGTERM/SIGHUP (the
  launch's shutdown, Ctrl+C) it stops gz with SIGKILL; gz also gets SIGKILL from the kernel if the
  guard dies. The ROS nodes still stop normally.

* **Ready marker** (``ready_file:=PATH``, or the environment variable ``APOLLO16_READY_FILE``;
  default none). The launch deletes ``PATH`` and ``PATH.not_ready`` when it starts. When
  ``wait_ready`` succeeds it writes ``PATH`` (after the READY banner); when it fails it writes
  ``PATH.not_ready`` instead. Both are written atomically (a temporary file in the same folder,
  then a rename) and hold ``key=value`` lines (``status``, ``ready_after_s``, ``launch_pid``,
  ``world``, ...). ``scripts/run_apollo16_sim.sh`` waits for it before it opens the Gazebo GUI.

World selection: ``world`` (default ``~/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf``)
and ``variant`` (``inspect``, ``stock`` or ``inspect_stock`` select the sibling
``<stem>_<variant>.sdf``). If the chosen world's physics engine is an absolute plugin path that is
missing or cannot be loaded (checked with dlopen), ``<stem>_stock.sdf`` (``<stem>_inspect_stock.sdf``
for the inspect lighting, when it exists) is used instead, with a warning.

Robot pose: ``site:=<name>`` reads ``x, y, z, yaw`` from ``world_info.yaml`` next to the world
(or ``world_info:=``); explicit ``robot_x/robot_y/robot_z`` (and optional ``robot_yaw``, rad)
override the site.
"""
from __future__ import annotations

import hashlib
import math
import os
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_WORLD = "~/togo_assets/lunar/worlds/apollo16_core/apollo16_core.sdf"
DEFAULT_LIDAR_TOPIC_GZ = "/sensors/lidar3d_0/scan/points"
LIDAR_TOPIC_ROS = "/husky/sensors/seyond/points"
IMU_TOPIC_ROS = "/husky/sensors/imu_0/data_raw"
CMD_VEL_TOPIC = "/platform_velocity_controller/cmd_vel"
CONTROLLERS = ("joint_state_broadcaster", "platform_velocity_controller")
VARIANTS = ("", "inspect", "stock", "inspect_stock")
_VARIANT_SUFFIXES = ("_inspect_stock", "_inspect", "_stock")   # longest first
PKG_DIR = Path(__file__).resolve().parent.parent                  # src/lunar_world_gen
DEFAULT_COLLISION_SPEC = PKG_DIR / "config" / "robot" / "a300_collision_simplified.yaml"
ROBOT_COLLISION_MODES = ("simplified", "original")
BUILD_PLUGIN_HINT = "bash <repo>/src/lunar_world_gen/tools/gz_physics_bullet_lunar/build_lunar_bullet_plugin.sh"
BUILD_WORLD_HINT = ("cd <repo>/src/lunar_world_gen && ~/.venvs/lunar_world/bin/python -m lunar_world_gen.build "
                    "--preset full")
STOCK_CONSEQUENCES = ("stock classic Bullet, 4 ms step: in-place turns reach only ~45-66 % of the commanded "
                      "rate and mesh friction is fixed at 0.5")
READY_FILE_ENV = "APOLLO16_READY_FILE"
NOT_READY_SUFFIX = ".not_ready"


# ===================================================================================== world
def _engine_filename(sdf_file: Path) -> str | None:
    try:
        root = ET.parse(str(sdf_file)).getroot()
    except ET.ParseError as exc:
        raise RuntimeError(f"cannot parse world file {sdf_file}: {exc}") from exc
    for plugin in root.iter("plugin"):
        if plugin.get("name") == "gz::sim::systems::Physics":
            fn = plugin.find("engine/filename")
            return fn.text.strip() if fn is not None and fn.text else None
    return None


def _world_name(sdf_file: Path) -> str | None:
    try:
        w = ET.parse(str(sdf_file)).getroot().find("world")
        return None if w is None else w.get("name")
    except ET.ParseError:
        return None


def _split_variant(stem: str) -> tuple[str, str]:
    """('apollo16_core', 'inspect_stock') for 'apollo16_core_inspect_stock'."""
    for suffix in _VARIANT_SUFFIXES:
        if stem.endswith(suffix):
            return stem[: -len(suffix)], suffix[1:]
    return stem, ""


def plugin_loadable(path: str, timeout: float = 30.0) -> tuple[bool | None, str]:
    """Can the dynamic loader load `path` with all its dependencies and symbols (RTLD_NOW)?

    Runs in a child python so that nothing is loaded into this process. Returns (True, ''),
    (False, loader message) or (None, why) when the check itself could not run.
    """
    code = "import ctypes, os, sys; ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW | os.RTLD_LOCAL)"
    try:
        res = subprocess.run([sys.executable or "python3", "-c", code, path], capture_output=True, text=True,
                             timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:
        return None, f"check not run: {exc}"
    if res.returncode == 0:
        return True, ""
    lines = [ln.strip() for ln in (res.stderr or "").splitlines() if ln.strip()]
    msg = lines[-1] if lines else f"exit code {res.returncode}"
    return False, msg.replace("OSError: ", "")


def resolve_world(world: str, variant: str, check_plugin: bool = False) -> tuple[Path, list[str]]:
    """Absolute world file to load (+ notes). Applies the variant and the stock fallback.

    The fallback is taken when the world's physics engine is an absolute plugin path that does
    not exist or, with ``check_plugin``, cannot be loaded (missing library, undefined symbol,
    broken file). An inspect-lighting world falls back to ``<stem>_inspect_stock.sdf`` when it
    exists, else to ``<stem>_stock.sdf``.
    """
    notes: list[str] = []
    path = Path(os.path.expanduser(world or DEFAULT_WORLD))
    if not path.is_absolute():
        path = path.resolve()
    variant = (variant or "").strip()
    if variant:
        if variant not in VARIANTS:
            raise RuntimeError(f"unknown variant '{variant}' (use '', inspect, stock or inspect_stock)")
        base, _ = _split_variant(path.stem)
        path = path.with_name(f"{base}_{variant}{path.suffix}")
    if not path.is_file():
        raise RuntimeError(f"world file not found: {path}. Build the world: {BUILD_WORLD_HINT}")
    engine = _engine_filename(path)
    problem = None
    if engine and os.path.isabs(engine):
        if not os.path.isfile(engine):
            problem = "is missing"
        elif check_plugin:
            ok, why = plugin_loadable(engine)
            if ok is False:
                problem = f"cannot be loaded ({why})"
            elif ok is None:
                notes.append(f"could not check that the physics plugin {engine} loads ({why})")
    if problem:
        base, current = _split_variant(path.stem)
        inspect = current in ("inspect", "inspect_stock")
        names = ([f"{base}_inspect_stock", f"{base}_stock"] if inspect else [f"{base}_stock"])
        candidates = [path.with_name(f"{n}{path.suffix}") for n in names]
        stock = next((c for c in candidates if c.is_file() and c != path), None)
        if stock is None:
            raise RuntimeError(f"physics plugin {engine} (used by {path.name}) {problem} and there is no stock "
                               f"fallback {candidates[-1].name}. Build the plugin: {BUILD_PLUGIN_HINT}")
        notes.append(f"physics plugin {engine} {problem} -> falling back to {stock.name} ({STOCK_CONSEQUENCES}). "
                     f"Build the plugin once: {BUILD_PLUGIN_HINT}")
        if inspect and not stock.stem.endswith("_inspect_stock"):
            notes.append(f"inspect lighting is not available with the stock plugin: no {names[0]}{path.suffix} "
                         "(rebuild the world to get it); using the realistic lighting")
        path = stock
    return path, notes


def resolve_pose(world_file: Path, world_info: str, site: str, rx: str, ry: str, rz: str,
                 ryaw: str) -> tuple[tuple[float, float, float, float], str]:
    """(x, y, z, yaw) from explicit arguments or from the world_info.yaml site."""
    explicit = [v.strip() for v in (rx, ry, rz)]
    if any(explicit):
        if not all(explicit):
            raise RuntimeError("give all of robot_x, robot_y and robot_z (or none, to use site:=)")
        try:
            yaw = float(ryaw) if ryaw.strip() else 0.0
            return (float(explicit[0]), float(explicit[1]), float(explicit[2]), yaw), "explicit pose"
        except ValueError as exc:
            raise RuntimeError(f"robot pose must be numbers (x y z [yaw_rad]): {exc}") from exc
    import yaml  # PyYAML ships with ROS 2

    info_path = Path(os.path.expanduser(world_info)) if world_info else world_file.parent / "world_info.yaml"
    if not info_path.is_file():
        raise RuntimeError(f"no world_info.yaml at {info_path}: pass an explicit pose (wrapper: --pose X Y Z "
                           "[YAW]; launch: robot_x/y/z:=) or the file (wrapper: --world-info PATH; launch: "
                           "world_info:=)")
    info = yaml.safe_load(info_path.read_text()) or {}
    sites = info.get("sites") or {}
    if site not in sites:
        why = (info.get("missing_sites") or {}).get(site)
        raise RuntimeError(f"site '{site}' not in {info_path} (available: {sorted(sites)})"
                           + (f"; generator: {why}" if why else ""))
    s = sites[site]
    pose = (float(s["x"]), float(s["y"]), float(s["z"]), float(s.get("yaw", 0.0)))
    if ryaw.strip():
        try:
            pose = pose[:3] + (float(ryaw),)
        except ValueError as exc:
            raise RuntimeError(f"robot yaw must be a number (rad): {exc}") from exc
    return pose, f"site '{site}' from {info_path}"


def gui_config_for_pose(text: str, world_file: Path, world_info: str, pose, ref_site: str = "flat"
                        ) -> tuple[str, str]:
    """Move a Gazebo GUI config's <camera_pose> from the `ref_site` spawn to the spawn `pose`.

    The packaged config (config/gui/apollo16.gui.config) has a chase view of the rover at the
    default 'flat' site. For another spawn (x, y, z[, yaw]) the camera keeps the same pose relative
    to the rover: its offset is turned about z by the yaw difference and moved with the spawn, its
    yaw turned by the same angle (roll and pitch unchanged), its height moved by the spawn-z
    difference. Returns (config text, note); the text is unchanged when there is nothing to move.
    """
    import re

    m = re.search(r"<camera_pose>\s*([^<]*?)\s*</camera_pose>", text)
    if m is None:
        return text, "camera as in the file (no <camera_pose>)"
    try:
        vals = [float(v) for v in m.group(1).split()]
    except ValueError:
        vals = []
    if len(vals) != 6:
        return text, "camera as in the file (<camera_pose> is not 'x y z roll pitch yaw')"
    try:
        (rx, ry, rz, ryaw), _ = resolve_pose(world_file, world_info, ref_site, "", "", "", "")
    except Exception:  # noqa: BLE001
        return text, f"camera as in the file (no '{ref_site}' site to move it from)"
    x, y, z = (float(v) for v in pose[:3])
    dyaw = (float(pose[3]) - ryaw) if len(pose) > 3 else 0.0
    dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
    if max(abs(x - rx), abs(y - ry), abs(z - rz), abs(dyaw)) < 1e-6:
        return text, f"camera as in the file ('{ref_site}' view)"
    ox, oy = vals[0] - rx, vals[1] - ry
    c, s = math.cos(dyaw), math.sin(dyaw)
    new = [x + c * ox - s * oy, y + s * ox + c * oy, vals[2] + (z - rz), vals[3], vals[4],
           math.atan2(math.sin(vals[5] + dyaw), math.cos(vals[5] + dyaw))]
    out = text[:m.start(1)] + " ".join(f"{v:.6g}" for v in new) + text[m.end(1):]
    return out, (f"camera: the '{ref_site}' view moved to this spawn "
                 f"(by {x - rx:+.2f}, {y - ry:+.2f}, {z - rz:+.2f} m, turned {math.degrees(dyaw):+.1f} deg)")


def world_gravity(world_file: Path, world_info: str = "") -> float | None:
    """gravity_m_s2 from world_info.yaml (None if unknown)."""
    try:
        import yaml

        p = Path(os.path.expanduser(world_info)) if world_info else world_file.parent / "world_info.yaml"
        g = (yaml.safe_load(p.read_text()) or {}).get("gravity_m_s2")
        return float(g) if g is not None else None
    except Exception:  # noqa: BLE001
        return None


# ===================================================================================== ready marker
def ready_marker_paths(ready_file: str | os.PathLike | None) -> tuple[Path, Path] | None:
    """(ready marker, not-ready marker) for a ``ready_file`` value; None when no marker is wanted."""
    text = str(ready_file or "").strip()
    if not text:
        return None
    path = Path(os.path.expanduser(text))
    if not path.is_absolute():
        path = path.resolve()
    return path, path.with_name(path.name + NOT_READY_SUFFIX)


def clear_ready_markers(ready_file: str | os.PathLike | None) -> list[Path]:
    """Delete both markers of an earlier run (at launch start); returns the files that were deleted."""
    removed = []
    for p in ready_marker_paths(ready_file) or ():
        try:
            p.unlink()
            removed.append(p)
        except FileNotFoundError:
            pass
    return removed


def write_marker_atomic(path: Path, text: str) -> None:
    """Write `text` to `path` so that a reader sees either no file or the complete file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


def write_ready_marker(ready_file: str | os.PathLike | None, ready: bool, fields: dict | None = None
                       ) -> Path | None:
    """Record the wait_ready result: ``ready_file`` on success, ``ready_file.not_ready`` on failure.

    The content is ``key=value`` lines starting with ``status=READY`` or ``status=NOT_READY``
    (values on one line). The other marker is deleted. Returns the written file (None: no marker).
    """
    paths = ready_marker_paths(ready_file)
    if paths is None:
        return None
    target, other = paths if ready else paths[::-1]
    items = {"status": "READY" if ready else "NOT_READY", **(fields or {})}
    text = "".join(f"{k}={str(v).replace(chr(13), ' ').replace(chr(10), ' ')}\n" for k, v in items.items())
    write_marker_atomic(target, text)
    try:
        other.unlink()
    except FileNotFoundError:
        pass
    return target


def on_wait_ready_exit(returncode: int, ready_file: str | os.PathLike | None, fields: dict | None = None) -> str:
    """The launch's ready-marker step after ``wait_ready`` exits; returns a log line ('' = no marker)."""
    ok = returncode == 0
    extra = {} if ok else {"wait_ready_exit_code": returncode,
                           "reason": "wait_ready timed out: controllers not active or /clock not advancing"
                           if returncode == 2 else f"wait_ready failed (exit code {returncode})"}
    try:
        path = write_ready_marker(ready_file, ok, {**(fields or {}), **extra})
    except OSError as exc:
        return f"WARNING: cannot write the ready marker {ready_file}: {exc}"
    return "" if path is None else f"{'ready' if ok else 'not-ready'} marker written: {path}"


# ===================================================================================== robot collisions
def _rpy_matrix(r: float, p: float, y: float) -> list[list[float]]:
    """URDF fixed-axis roll/pitch/yaw: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _matrix_rpy(m: list[list[float]]) -> tuple[float, float, float]:
    sp = max(-1.0, min(1.0, -m[2][0]))
    p = math.asin(sp)
    if abs(sp) < 1.0 - 1e-12:
        return math.atan2(m[2][1], m[2][2]), p, math.atan2(m[1][0], m[0][0])
    # gimbal lock: only r - y (or r + y) is defined; put it all into roll
    return math.atan2(-m[1][2], m[1][1]), p, 0.0


def _clean(vals, eps: float = 1e-12) -> list[float]:
    """Round-off noise and -0.0 to 0.0 (for readable XML)."""
    return [0.0 if abs(v) < eps else float(v) for v in vals]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _floats(text: str | None, n: int = 3, default: float = 0.0) -> list[float]:
    vals = [float(v) for v in (text or "").split()]
    if not vals:
        return [default] * n
    if len(vals) == 1 and n == 3 and default == 1.0:  # a single scale value
        return vals * 3
    if len(vals) != n:
        raise ValueError(f"expected {n} numbers, got {text!r}")
    return vals


def load_collision_spec(path: str | Path) -> dict:
    import yaml

    spec = yaml.safe_load(Path(path).read_text()) or {}
    if spec.get("schema_version") != 1 or not isinstance(spec.get("meshes"), list):
        raise RuntimeError(f"{path}: not a robot collision spec (schema_version 1 with 'meshes')")
    return spec


def resolve_package_uri(uri: str) -> Path | None:
    """package://pkg/rel or file:// path -> existing file (ament index), else None."""
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        try:
            from ament_index_python.packages import get_package_share_directory

            p = Path(get_package_share_directory(pkg)) / rel
        except Exception:  # noqa: BLE001
            return None
        return p if p.is_file() else None
    p = Path(uri[len("file://"):] if uri.startswith("file://") else uri)
    return p if p.is_file() else None


def _file_matches(entry: dict, resolve) -> str | None:
    """None if the mesh file the description uses is the one the spec was computed from, else why not."""
    if resolve is None:
        return None
    path = resolve(entry["uri"])
    if path is None:
        return "mesh file not found"
    if entry.get("bytes") is not None and path.stat().st_size != int(entry["bytes"]):
        return f"{path} changed size ({path.stat().st_size} B, spec {entry['bytes']} B)"
    if entry.get("sha256"):
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        if h.hexdigest() != entry["sha256"]:
            return f"{path} changed content (sha256)"
    return None


def simplify_collisions(urdf_xml: str, spec: dict, resolve=resolve_package_uri) -> tuple[str, dict]:
    """Replace every <collision> mesh listed in `spec` by its bounding primitive.

    The new <origin> is the collision's origin composed with the primitive's pose in the mesh
    frame, so the primitive covers the mesh where the mesh was. Meshes whose file no longer
    matches the spec's sha256 / size are kept (reported in 'kept_changed'). Everything else in
    the description (visuals, inertials, plugins, comments) is left as it is.
    """
    from xml.dom import minidom

    doc = minidom.parseString(urdf_xml)
    entries = {}
    for e in spec.get("meshes", []):
        entries[(e["uri"], tuple(round(float(s), 9) for s in e.get("scale", (1, 1, 1))))] = e
    checked: dict[str, str | None] = {}
    report = {"replaced": [], "kept_changed": [], "triangles_removed": 0, "primitives": {}}

    def child(node, tag):
        return next((c for c in node.childNodes if c.nodeType == c.ELEMENT_NODE and c.tagName == tag), None)

    for col in list(doc.getElementsByTagName("collision")):
        geom = child(col, "geometry")
        mesh = child(geom, "mesh") if geom is not None else None
        if mesh is None:
            continue
        uri = mesh.getAttribute("filename")
        scale = tuple(round(s, 9) for s in _floats(mesh.getAttribute("scale") or None, 3, 1.0))
        e = entries.get((uri, scale))
        if e is None:
            continue
        link = col.parentNode.getAttribute("name") if col.parentNode is not None else ""
        if uri not in checked:
            checked[uri] = _file_matches(e, resolve)
        if checked[uri]:
            report["kept_changed"].append({"link": link, "uri": uri, "why": checked[uri]})
            continue
        prim = e["primitive"]
        parts = (prim.get("parts") or []) if prim["type"] == "compound" else [prim]
        if not parts:
            raise RuntimeError(f"empty compound primitive for {uri}")
        origin = child(col, "origin")
        oxyz = _floats(origin.getAttribute("xyz") or None) if origin is not None else [0.0, 0.0, 0.0]
        orpy = _floats(origin.getAttribute("rpy") or None) if origin is not None else [0.0, 0.0, 0.0]
        R = _rpy_matrix(*orpy)
        indent = col.previousSibling.data if (col.previousSibling is not None and
                                              col.previousSibling.nodeType == col.TEXT_NODE) else "\n"
        parent = col.parentNode
        parent.insertBefore(doc.createComment(f" lunar_world_gen robot_collision:=simplified: {uri} "
                                              f"({e.get('triangles', '?')} triangles) -> {len(parts)} "
                                              f"{'primitive' if len(parts) == 1 else 'primitives'} "), col)
        for idx, part in enumerate(parts):
            new = doc.createElement("collision")
            for k, v in col.attributes.items():
                new.setAttribute(k, f"{v}_{idx}" if (k == "name" and len(parts) > 1) else v)
            c = [float(v) for v in part["xyz"]]
            xyz = [oxyz[i] + sum(R[i][k] * c[k] for k in range(3)) for i in range(3)]
            rpy = _matrix_rpy(_matmul(R, _rpy_matrix(*[float(v) for v in part.get("rpy", (0, 0, 0))])))
            org = doc.createElement("origin")
            org.setAttribute("xyz", " ".join(f"{v + 0.0:.9g}" for v in _clean(xyz)))
            org.setAttribute("rpy", " ".join(f"{v + 0.0:.9g}" for v in _clean(rpy)))
            if part["type"] == "box":
                shape = doc.createElement("box")
                shape.setAttribute("size", " ".join(f"{float(v):.9g}" for v in part["size"]))
            elif part["type"] == "cylinder":
                shape = doc.createElement("cylinder")
                shape.setAttribute("radius", f"{float(part['radius']):.9g}")
                shape.setAttribute("length", f"{float(part['length']):.9g}")
            else:
                raise RuntimeError(f"unknown primitive type {part['type']!r} for {uri}")
            g = doc.createElement("geometry")
            g.appendChild(shape)
            new.appendChild(org)
            new.appendChild(g)
            parent.insertBefore(doc.createTextNode(indent), col)
            parent.insertBefore(new, col)
            report["primitives"][part["type"]] = report["primitives"].get(part["type"], 0) + 1
        parent.removeChild(col)
        report["replaced"].append({"link": link, "uri": uri, "type": prim["type"], "parts": len(parts)})
        report["triangles_removed"] += int(e.get("triangles", 0))
    return doc.toxml(), report


def robot_description_xml(urdf_xml: str, mode: str, spec_path: str) -> tuple[str, list[str]]:
    """The description for gz/RSP (`mode` simplified|original) + log lines. Never raises for a bad
    spec: the original description is used with a WARNING instead."""
    mode = (mode or "simplified").strip().lower()
    if mode not in ROBOT_COLLISION_MODES:
        raise RuntimeError(f"robot_collision must be simplified or original, not '{mode}'")
    if mode == "original":
        return urdf_xml, ["robot collision: original CAD meshes (robot_collision:=original)"]
    try:
        spec = load_collision_spec(spec_path)
        xml, rep = simplify_collisions(urdf_xml, spec)
    except Exception as exc:  # noqa: BLE001
        return urdf_xml, [f"WARNING: robot_collision:=simplified failed ({exc}); using the original CAD collision "
                          "meshes (slow physics on slopes). Re-create the spec: ~/.venvs/lunar_world/bin/python "
                          "tools/robot_collision_simplify.py"]
    prims = ", ".join(f"{n} {t}{'es' if t == 'box' and n > 1 else ('s' if n > 1 else '')}"
                      for t, n in sorted(rep["primitives"].items()))
    lines = [f"robot collision: simplified - {len(rep['replaced'])} CAD collision meshes "
             f"({rep['triangles_removed']:,} triangles) -> {prims or 'nothing'} (spec {spec_path})"]
    for k in rep["kept_changed"]:
        lines.append(f"WARNING: kept the original collision mesh of {k['link']}: {k['why']}; re-run "
                     "tools/robot_collision_simplify.py")
    return xml, lines


# ===================================================================================== gz server
def _upstream_gz_sim_env() -> tuple[dict, str]:
    """Environment and gz executable exactly as ros_gz_sim's gz_sim.launch.py computes them."""
    import importlib.util

    from ament_index_python.packages import get_package_share_directory

    src = Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
    spec = importlib.util.spec_from_file_location("_ros_gz_sim_gz_sim_launch", src)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    model_paths, plugin_paths = mod.GazeboRosPaths.get_paths()
    env = {
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join([os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                                                      os.environ.get("LD_LIBRARY_PATH", ""), plugin_paths]),
        "GZ_SIM_RESOURCE_PATH": os.pathsep.join([os.environ.get("GZ_SIM_RESOURCE_PATH", ""), model_paths]),
    }
    return env, mod.get_executable_path("gz")


# ===================================================================================== launch
def _truthy(v: str) -> bool:
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    from launch.actions import (ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                                OpaqueFunction, RegisterEventHandler, Shutdown)
    from launch.conditions import IfCondition
    from launch.event_handlers import OnProcessExit
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                      PathJoinSubstitution)
    from launch_ros.actions import Node, PushRosNamespace
    from launch_ros.parameter_descriptions import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    t_start = time.monotonic()
    world_file, notes = resolve_world(arg("world"), arg("variant"), check_plugin=_truthy(arg("check_plugin")))
    (x, y, z, yaw), pose_src = resolve_pose(world_file, arg("world_info"), arg("site"), arg("robot_x"),
                                            arg("robot_y"), arg("robot_z"), arg("robot_yaw"))
    engine = _engine_filename(world_file) or "(default)"
    gravity = world_gravity(world_file, arg("world_info"))
    ns = LaunchConfiguration("ns")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    tag = "[apollo16_sim]"

    pkg_gz = FindPackageShare("togo_gz")
    pkg_deploy = FindPackageShare("togo_deploy")

    # 1b. robot description: the user's xacro exactly as togo_deploy control.launch.py (is_sim:=true),
    #     then (default) the CAD collision meshes replaced by bounding primitives
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([pkg_gz, "urdf", "togo_gz.urdf.xacro"]), " ",
        "ns:=", ns, " ",
        "tf_prefix:=", tf_prefix, " ",
        "is_sim:=", "true", " ",
        "use_fake_hardware:=", "false", " ",
        LaunchConfiguration("extra_xacro_args"),
    ]).perform(context)
    description, collision_lines = robot_description_xml(robot_description_content, arg("robot_collision"),
                                                         arg("robot_collision_spec") or str(DEFAULT_COLLISION_SPEC))

    info = [
        LogInfo(msg=f"{tag} world: {world_file} (world name {_world_name(world_file)})"),
        LogInfo(msg=f"{tag} physics engine: {engine}"),
        LogInfo(msg=f"{tag} robot pose: x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.4f} rad ({pose_src})"),
    ] + [LogInfo(msg=f"{tag} WARNING: {n}") for n in notes] + [LogInfo(msg=f"{tag} {ln}") for ln in collision_lines]

    # ready marker (scripts/run_apollo16_sim.sh opens the GUI when it appears): a marker of an earlier run
    # must never announce this one, so both files go now
    ready_file = arg("ready_file").strip() or os.environ.get(READY_FILE_ENV, "").strip()
    marker_paths = ready_marker_paths(ready_file)
    if marker_paths is not None:
        clear_ready_markers(ready_file)
        info.append(LogInfo(msg=f"{tag} ready marker: {marker_paths[0]} is written at READY "
                                f"({marker_paths[1].name} if the READY check fails)"))

    # 1a. gz server: the ros_gz_sim gz_sim.launch.py command and environment, started by the guard
    gz_args = ["-r", "-s", "-v", str(int(arg("gz_verbosity")))]
    if _truthy(arg("headless_rendering")):
        gz_args.append("--headless-rendering")
    gz_args.append(str(world_file))
    gz_server = None
    if _truthy(arg("gz_guard")):
        try:
            env, gz_exe = _upstream_gz_sim_env()
            gz_server = ExecuteProcess(
                cmd=[sys.executable, os.path.abspath(__file__), "--gz-guard", "--", "ruby", gz_exe, "sim",
                     *gz_args, "--force-version", "8"],
                name="gazebo", output="screen", additional_env=env, on_exit=Shutdown(),
                sigterm_timeout="10", sigkill_timeout="5")
            info.append(LogInfo(msg=f"{tag} gz server started by the guard: stops with SIGKILL on shutdown "
                                    "(no segfault, no WSL crash dump)"))
        except Exception as exc:  # noqa: BLE001
            info.append(LogInfo(msg=f"{tag} WARNING: gz guard unavailable ({exc!r}); using ros_gz_sim "
                                    "gz_sim.launch.py directly (gz may segfault on Ctrl+C)"))
    if gz_server is None:
        gz_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch",
                                                                "gz_sim.launch.py"])),
            launch_arguments={"gz_args": " ".join(shlex.quote(a) for a in gz_args),
                              "on_exit_shutdown": "True"}.items())
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")

    robot_description = {"robot_description": ParameterValue(value=description, value_type=str)}
    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 output="both", parameters=[robot_description, {"use_sim_time": use_sim_time}])

    # 1c. spawn (create waits for the world service and for robot_description by itself) + bridge
    create = Node(package="ros_gz_sim", executable="create", output="screen",
                  arguments=["-name", "togo", "-topic", "robot_description",
                             "-x", f"{x:.4f}", "-y", f"{y:.4f}", "-z", f"{z:.4f}", "-Y", f"{yaw:.6f}"])
    sim_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="sim_bridge", output="screen",
        parameters=[{
            "config_file": PathJoinSubstitution([pkg_gz, "config", "bridge.yaml"]),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
            f"qos_overrides.{LIDAR_TOPIC_ROS}.publisher.reliability": "best_effort",
        }])

    # 2. wait until the robot's sensors exist (render scene loaded) and the sim runs again
    waiter = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), "--wait-sensors",
             "--topic", arg("lidar_topic_gz"), "--timeout", arg("sensor_wait_timeout")],
        name="wait_for_sensors", output="screen")

    # 3. controller spawners, one after the other, with generous timeouts
    def spawner(controller: str) -> Node:
        return Node(package="controller_manager", executable="spawner", name=controller, output="screen",
                    arguments=["--controller-manager", "controller_manager",
                               "--controller-manager-timeout", arg("spawner_cm_timeout"),
                               "--service-call-timeout", arg("spawner_service_timeout"),
                               "--switch-timeout", arg("spawner_switch_timeout"),
                               controller])

    jsb_spawner = spawner(CONTROLLERS[0])
    pvc_spawner = spawner(CONTROLLERS[1])

    # 4. readiness check (controllers active + /clock advancing), then optional aux nodes
    ns_val = arg("ns").strip("/")
    cm_path = f"/{ns_val}/controller_manager" if ns_val else "/controller_manager"
    ready = ExecuteProcess(
        cmd=[sys.executable, os.path.abspath(__file__), "--wait-ready", "--controller-manager", cm_path,
             "--controllers", ",".join(CONTROLLERS), "--timeout", arg("ready_timeout")],
        name="wait_ready", output="screen")
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_deploy, "launch",
                                                            "robot_sensor_checkout.launch.py"])),
        launch_arguments={"lidar_topic": LIDAR_TOPIC_ROS}.items(),
        condition=IfCondition(LaunchConfiguration("rviz")))
    rgbd_fixer = Node(package="togo_gz", executable="gz_rgbd_point_fixer", name="gz_rgbd_point_fixer",
                      parameters=[PathJoinSubstitution([pkg_gz, "config", "rgbd_point_fix.yaml"])],
                      condition=IfCondition(LaunchConfiguration("rgbd_point_fixer")))

    env_hint = f"source {PKG_DIR / 'scripts' / 'env.sh'}"
    exit_codes: dict[str, int] = {}

    def since() -> str:
        return f"+{time.monotonic() - t_start:.1f} s"

    def in_ns(*actions):
        # actions emitted later by event handlers run outside the group below, so they get
        # their own namespace push
        return GroupAction(actions=[PushRosNamespace(ns), *actions])

    def spawner_note(controller: str, rc: int) -> list:
        if rc == 0:
            return []
        return [LogInfo(msg=f"{tag} WARNING: {controller} spawner exited with code {rc}; the controller may still "
                            f"become active - check with: ros2 control list_controllers (after {env_hint})")]

    def after_wait(event, _context):
        rc = event.returncode
        msg = ("sensors ready" if rc == 0 else
               f"WARNING: wait_for_sensors exited with code {rc} (timeout or error); starting the spawners anyway")
        return [LogInfo(msg=f"{tag} {msg} at {since()} -> spawning {CONTROLLERS[0]}"), in_ns(jsb_spawner)]

    def after_jsb(event, _context):
        exit_codes[CONTROLLERS[0]] = event.returncode
        return [LogInfo(msg=f"{tag} {CONTROLLERS[0]} spawner exited with code {event.returncode} at "
                            f"{since()} -> spawning {CONTROLLERS[1]}"),
                *spawner_note(CONTROLLERS[0], event.returncode), in_ns(pvc_spawner)]

    def after_pvc(event, _context):
        exit_codes[CONTROLLERS[1]] = event.returncode
        return [LogInfo(msg=f"{tag} {CONTROLLERS[1]} spawner exited with code {event.returncode} at {since()}"),
                *spawner_note(CONTROLLERS[1], event.returncode), ready]

    def mark_ready(_context, returncode: int, after_s: float):
        line = on_wait_ready_exit(returncode, ready_file, {
            "ready_after_s": f"{after_s:.1f}", "launch_pid": os.getpid(),
            "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "world": world_file,
            "pose": f"{x:.4f} {y:.4f} {z:.4f} {yaw:.6f}", "pose_source": pose_src,
            "gz_partition": os.environ.get("GZ_PARTITION", ""), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "")})
        return [LogInfo(msg=f"{tag} {line}")] if line.startswith("WARNING") else []

    def after_ready(event, _context):
        out = []
        if event.returncode == 0:
            bar = "=" * 78
            g = (f"; world gravity {gravity:g} m/s^2 - DLIO odom/gravity must be {gravity:g}"
                 if gravity is not None else "")
            out += [LogInfo(msg=f"{tag} {bar}"),
                    LogInfo(msg=f"{tag} READY at {since()}: {' and '.join(CONTROLLERS)} active, /clock running"),
                    LogInfo(msg=f"{tag}   drive: {CMD_VEL_TOPIC} (geometry_msgs/TwistStamped)"),
                    LogInfo(msg=f"{tag}   LiDAR: {LIDAR_TOPIC_ROS}   IMU: {IMU_TOPIC_ROS}   clock: /clock"),
                    LogInfo(msg=f"{tag}   other terminals: {env_hint}"),
                    LogInfo(msg=f"{tag}   SLAM can start now{g}"),
                    LogInfo(msg=f"{tag} {bar}")]
        else:
            out.append(LogInfo(msg=f"{tag} WARNING: NOT READY at {since()} (wait_ready exit code {event.returncode}; "
                                   "see its lines above). Check: ros2 control list_controllers and ros2 topic hz "
                                   f"/clock (after {env_hint}); spawn a missing controller with: ros2 run "
                                   "controller_manager spawner <name> --controller-manager-timeout 60"))
        if marker_paths is not None:   # after the banner, so that it is printed when the marker appears
            out.append(OpaqueFunction(function=mark_ready, args=[event.returncode, time.monotonic() - t_start]))
        out.append(in_ns(rviz, rgbd_fixer))
        return out

    handlers = [
        RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=after_wait)),
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=after_jsb)),
        RegisterEventHandler(OnProcessExit(target_action=pvc_spawner, on_exit=after_pvc)),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)),
    ]
    group = GroupAction(actions=[PushRosNamespace(ns), gz_server, clock_bridge, robot_state_publisher,
                                 create, sim_bridge, waiter])
    return info + handlers + [group]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    args = [
        DeclareLaunchArgument("world", default_value=DEFAULT_WORLD,
                              description="Absolute path of the world SDF (~ is expanded)"),
        DeclareLaunchArgument("variant", default_value="",
                              description="'' (as given), 'inspect', 'stock' or 'inspect_stock': sibling "
                                          "<stem>_<variant>.sdf"),
        DeclareLaunchArgument("world_info", default_value="",
                              description="world_info.yaml (default: next to the world file)"),
        DeclareLaunchArgument("site", default_value="flat", description="world_info.yaml site to spawn at"),
        DeclareLaunchArgument("robot_x", default_value="", description="explicit spawn x (overrides site)"),
        DeclareLaunchArgument("robot_y", default_value="", description="explicit spawn y"),
        DeclareLaunchArgument("robot_z", default_value="", description="explicit spawn z (base_link)"),
        DeclareLaunchArgument("robot_yaw", default_value="", description="spawn yaw [rad] (overrides the site's)"),
        DeclareLaunchArgument("robot_collision", default_value="simplified",
                              description="simplified: the CAD collision meshes replaced by bounding boxes "
                                          "(config/robot/a300_collision_simplified.yaml); original: as in the "
                                          "robot description (slow physics on slopes)"),
        DeclareLaunchArgument("robot_collision_spec", default_value="",
                              description="primitive spec YAML (default config/robot/a300_collision_simplified.yaml)"),
        DeclareLaunchArgument("rviz", default_value="false", description="start RViz (sensor checkout)"),
        DeclareLaunchArgument("rgbd_point_fixer", default_value="false",
                              description="start togo_gz gz_rgbd_point_fixer (cameras are disabled in the robot)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("ns", default_value="", description="namespace for the robot"),
        DeclareLaunchArgument("extra_xacro_args", default_value=""),
        DeclareLaunchArgument("gz_verbosity", default_value="4"),
        DeclareLaunchArgument("headless_rendering", default_value="false",
                              description="add --headless-rendering to the gz server (EGL, no X display)"),
        DeclareLaunchArgument("gz_guard", default_value="true",
                              description="start the gz server under the guard that stops it with SIGKILL "
                                          "(gz-sim 8 segfaults on SIGINT/SIGTERM; WSL then writes a GB crash dump)"),
        DeclareLaunchArgument("check_plugin", default_value="true",
                              description="dlopen-check the patched physics plugin; fall back to the stock "
                                          "variant if it cannot be loaded"),
        DeclareLaunchArgument("lidar_topic_gz", default_value=DEFAULT_LIDAR_TOPIC_GZ,
                              description="gz topic whose advertisement gates the controller spawners"),
        DeclareLaunchArgument("sensor_wait_timeout", default_value="600",
                              description="wall s to wait for the LiDAR topic before spawning anyway"),
        DeclareLaunchArgument("spawner_cm_timeout", default_value="300"),
        DeclareLaunchArgument("spawner_service_timeout", default_value="60"),
        DeclareLaunchArgument("spawner_switch_timeout", default_value="30"),
        DeclareLaunchArgument("ready_timeout", default_value="60",
                              description="wall s for the READY check (controllers active, /clock advancing)"),
        DeclareLaunchArgument("ready_file", default_value="",
                              description=f"marker file written atomically at READY (<file>{NOT_READY_SUFFIX} if "
                                          f"the READY check fails); both are deleted at start. Default: "
                                          f"${READY_FILE_ENV}, else none"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])


# ===================================================================================== waiter
def _wait_main(argv: list[str]) -> int:
    """Wait for a gz topic to be advertised, then for the sim time to advance.

    Exit 0 when ready, 2 on timeout, 3 on error. Run with the system python (gz bindings)
    and the same GZ_PARTITION as the server. Prints a progress line every --progress s.
    """
    import argparse

    ap = argparse.ArgumentParser(prog="apollo16_sim.launch.py --wait-sensors")
    ap.add_argument("--wait-sensors", action="store_true")
    ap.add_argument("--topic", default=DEFAULT_LIDAR_TOPIC_GZ)
    ap.add_argument("--timeout", type=float, default=600.0, help="wall s to wait for the topic")
    ap.add_argument("--stats-topic", default="/stats")
    ap.add_argument("--sim-advance", type=float, default=0.5,
                    help="sim s that must elapse after the topic appears (sim running again)")
    ap.add_argument("--advance-timeout", type=float, default=120.0)
    ap.add_argument("--progress", type=float, default=10.0, help="wall s between progress lines")
    a = ap.parse_args(argv)
    tag = "[wait_for_sensors]"
    try:
        from gz.transport13 import Node, SubscribeOptions
        from gz.msgs10.world_stats_pb2 import WorldStatistics
    except ImportError as exc:
        print(f"{tag} gz python bindings unavailable ({exc}); sleeping 30 s instead", flush=True)
        time.sleep(30.0)
        return 3
    node = Node()
    t0 = time.monotonic()
    print(f"{tag} waiting for gz topic {a.topic} (timeout {a.timeout:.0f} s, "
          f"GZ_PARTITION={os.environ.get('GZ_PARTITION', '')})", flush=True)
    next_note = t0 + a.progress
    while a.topic not in node.topic_list():
        now = time.monotonic()
        if now - t0 > a.timeout:
            print(f"{tag} TIMEOUT: {a.topic} not advertised after {a.timeout:.0f} s", flush=True)
            return 2
        if now >= next_note:
            print(f"{tag} still waiting for {a.topic} (gz server loading the world), {now - t0:.0f} s", flush=True)
            next_note += a.progress
        time.sleep(0.25)
    t1 = time.monotonic()
    print(f"{tag} {a.topic} advertised after {t1 - t0:.1f} s; waiting for the simulation to run again "
          f"(it pauses while the robot's sensors build the render scene, ~25-30 s on this world)", flush=True)
    state: dict[str, float] = {}

    def on_stats(msg):
        t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        state["seen"] = t
        if not msg.paused:
            state.setdefault("first", t)
            state["last"] = t

    opts = SubscribeOptions()
    opts.msgs_per_sec = 20
    node.subscribe(WorldStatistics, a.stats_topic, on_stats, opts)
    next_note = t1 + a.progress
    while True:
        if "first" in state and state["last"] - state["first"] >= a.sim_advance:
            break
        now = time.monotonic()
        if now - t1 > a.advance_timeout:
            print(f"{tag} TIMEOUT: sim time did not advance {a.sim_advance} s within {a.advance_timeout:.0f} s",
                  flush=True)
            return 2
        if now >= next_note:
            st = f"sim t = {state['seen']:.2f} s" if "seen" in state else "no /stats yet"
            print(f"{tag} still loading the render scene ({st}), {now - t0:.0f} s", flush=True)
            next_note += a.progress
        time.sleep(0.05)
    print(f"{tag} sim running (sim t = {state['last']:.2f} s); ready after {time.monotonic() - t0:.1f} s",
          flush=True)
    return 0


# ===================================================================================== ready check
def _ready_main(argv: list[str]) -> int:
    """Exit 0 once all --controllers are active (list_controllers) and /clock advances.

    Exit 2 on timeout, 3 on error. Run with the system python (rclpy) in the sim's ROS environment.
    """
    import argparse

    ap = argparse.ArgumentParser(prog="apollo16_sim.launch.py --wait-ready")
    ap.add_argument("--wait-ready", action="store_true")
    ap.add_argument("--controller-manager", default="/controller_manager")
    ap.add_argument("--controllers", default=",".join(CONTROLLERS))
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--clock-advance", type=float, default=0.2, help="sim s /clock must advance")
    a = ap.parse_args(argv)
    tag = "[wait_ready]"
    wanted = [c for c in a.controllers.split(",") if c]
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("apollo16_ready_check")
    t0 = time.monotonic()
    deadline = t0 + a.timeout
    clock: dict[str, float] = {}

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        clock.setdefault("first", t)
        clock["last"] = t

    node.create_subscription(Clock, "/clock", on_clock, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    cli = node.create_client(ListControllers, f"{a.controller_manager}/list_controllers")
    states: dict[str, str] = {}
    ok_ctl = False
    while time.monotonic() < deadline and not ok_ctl:
        if cli.wait_for_service(timeout_sec=1.0):
            fut = cli.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            if fut.done() and fut.result() is not None:
                states = {c.name: c.state for c in fut.result().controller}
                ok_ctl = all(states.get(w) == "active" for w in wanted)
        if not ok_ctl:
            rclpy.spin_once(node, timeout_sec=1.0)
    print(f"{tag} controllers: " + (", ".join(f"{w}={states.get(w, 'missing')}" for w in wanted)), flush=True)
    while time.monotonic() < deadline and not ("first" in clock and clock["last"] - clock["first"] >= a.clock_advance):
        rclpy.spin_once(node, timeout_sec=0.1)
    ok_clock = "first" in clock and clock["last"] - clock["first"] >= a.clock_advance
    print(f"{tag} /clock: " + (f"advancing (sim t = {clock['last']:.2f} s)" if ok_clock else
                               ("not advancing" if clock else "no message")) +
          f"; checked in {time.monotonic() - t0:.1f} s", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_ctl and ok_clock else 2


# ===================================================================================== gz guard
def _gz_guard_main(argv: list[str]) -> int:
    """Run the gz server command (after '--') and stop it with SIGKILL instead of letting a signal
    shut it down.

    gz-sim 8 segfaults while shutting down after SIGINT/SIGTERM; WSL streams the core of that
    process to %TEMP%\\wsl-crashes (1.5-3.6 GB, 15-23 s). The server holds nothing worth a clean
    exit, so: SIGINT/SIGTERM/SIGHUP to the guard -> SIGKILL to gz; gz gets SIGKILL from the kernel
    if the guard dies (PR_SET_PDEATHSIG), and the guard kills gz if ros2 launch disappears.
    gz stays in the launch's process group, so tools that look for it there still find it.
    Exit code: 0 after a requested stop, else gz's own (128 + signal if it died of one).
    """
    import ctypes
    import signal

    tag = "[gz_guard]"
    cmd = argv[argv.index("--") + 1:] if "--" in argv else []
    if not cmd:
        print(f"{tag} usage: --gz-guard -- <gz command...>", flush=True)
        return 2
    guard_pid, parent = os.getpid(), os.getppid()
    try:
        libc = ctypes.CDLL(None, use_errno=True)
    except OSError:
        libc = None
    state: dict = {"child": None, "why": None, "t": None}

    def preexec():  # runs in the child between fork and exec
        if libc is not None:
            libc.prctl(1, int(signal.SIGKILL), 0, 0, 0)  # PR_SET_PDEATHSIG
        if os.getppid() != guard_pid:
            os._exit(1)

    def stop(why: str) -> None:
        if state["why"] is None:
            state["why"], state["t"] = why, time.monotonic()
        ch = state["child"]
        if ch is not None and ch.poll() is None:
            try:
                os.kill(ch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def on_signal(signum, _frame):
        stop(signal.Signals(signum).name)

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, on_signal)
    try:
        state["child"] = subprocess.Popen(cmd, preexec_fn=preexec, stdin=subprocess.DEVNULL)
    except OSError as exc:
        print(f"{tag} cannot start {cmd[0]}: {exc}", flush=True)
        return 127
    if state["why"] is not None:  # a signal arrived while starting
        stop(state["why"])
    print(f"{tag} gz server pid {state['child'].pid}; a shutdown stops it with SIGKILL", flush=True)
    while True:
        try:
            rc = state["child"].wait(timeout=0.5)
            break
        except subprocess.TimeoutExpired:
            if os.getppid() != parent:
                stop("ros2 launch exited")
    if state["why"] is not None:
        print(f"{tag} {state['why']}: gz server stopped with SIGKILL after {time.monotonic() - state['t']:.2f} s",
              flush=True)
        return 0
    return rc if rc >= 0 else 128 - rc


if __name__ == "__main__":
    modes = {"--wait-sensors": _wait_main, "--wait-ready": _ready_main, "--gz-guard": _gz_guard_main}
    mode = next((m for m in sys.argv[1:2] if m in modes), None) or \
        next((m for m in ("--wait-sensors", "--wait-ready") if m in sys.argv[1:]), None)
    if mode is None:
        print(__doc__)
        sys.exit(0)
    try:
        code = modes[mode](sys.argv[1:])
    except Exception as exc:  # noqa: BLE001
        print(f"[{mode.lstrip('-').replace('-', '_')}] error: {exc!r}", flush=True)
        code = 3
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)  # gz-transport / rclpy threads can abort a normal interpreter teardown
