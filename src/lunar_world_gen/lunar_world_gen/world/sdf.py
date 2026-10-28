"""World SDF writer (docs/DESIGN.md section 8).

``write_world(cfg, layout, visuals, collisions, variant)`` writes one self-contained world
file next to the meshes and textures it references (``layout.world_dir``):

=====================  ================================  =====================  =================
variant                file                              physics engine         lighting
=====================  ================================  =====================  =================
``""``                 ``apollo16_core.sdf``             ``physics.engine``     realistic
                                                         (patched by default)
``"inspect"``          ``apollo16_core_inspect``         ``physics.engine``     inspection
``"stock"``            ``apollo16_core_stock``           stock classic Bullet,  realistic
                                                         ``step_s.stock``
``"inspect_stock"``    ``apollo16_core_inspect_stock``   stock classic Bullet,  inspection
                                                         ``step_s.stock``
=====================  ================================  =====================  =================

``inspect_stock`` is the launch fallback for ``variant:=inspect`` when the patched plugin is
missing, so the fallback keeps the inspection lighting (usability review U11).

What goes into every variant:

* systems: Physics (with ``<engine><filename>``), UserCommands and SceneBroadcaster only.
  The robot brings Sensors, Imu, NavSat, PosePublisher and gz_ros2_control itself; a world
  copy of those would load a second instance (``FORBIDDEN_SYSTEMS``).
* ``<physics type="ignored">``: ``max_step_size = physics.step_s[engine]``, real-time factor 1,
  ``real_time_update_rate = 1 / step``. The patched engine is referenced by the absolute path
  ``cfg.paths.physics_plugin``; the stock one by its library name.
* gravity ``0 0 -physics.gravity_m_s2``, magnetic field 0.
* ``<spherical_coordinates>``: ``CUSTOM_SURFACE`` sphere (both axes ``spherical_coordinates.axis_m``),
  ENU, site lat/lon, elevation = ``site.href_m``, heading 0.
* scene and lights from :mod:`.lighting`.
* one static model ``apollo16_terrain``: link ``terrain`` holds every visual and the
  collisions of the most common friction pair; collisions with another (mu, mu2) go to
  extra static links ``terrain_mu_<mu>_<mu2>``. Classic Bullet stores friction per rigid
  body (= per link, the last collision wins), so one link per friction pair keeps each
  collision's ``<ode><mu>`` exact. (DESIGN says "a single link"; this only differs when
  collisions carry different friction values, and it is required for correctness then.)
* named PBR materials (``MATERIAL_NAMES``) with texture paths relative to the SDF.

Every mesh URI (and, by default, every texture) is checked before anything is written:
one unresolvable mesh makes Gazebo abort the whole world load.

Material notes (verified in Gazebo, see docs/impl/world.md and docs/impl/mesh.md): ogre2
multiplies the albedo map by the material ``<diffuse>`` colour, and decodes albedo PNGs as sRGB.
The albedo maps already hold the absolute base colour, so a textured material gets
``<diffuse>1 1 1 1`` and the base colour is used as ``<diffuse>`` only without an albedo map
(DESIGN said "<diffuse> from the base colour": that renders ~8x too dark). Likewise the scalar
``<roughness>`` is 1 when a roughness map is present (Ogre-Next multiplies the two; the map holds
the configured value), and ``<specular>`` is 0 0 0 1 (no specular lobe).
"""
from __future__ import annotations

import os
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

from .. import __version__
from ..config import Config, config_hash
from ..layout import Layout
from .lighting import LightingSetup, LightSpec, lighting_setup

SDF_VERSION = "1.11"
TERRAIN_MODEL = "apollo16_terrain"
TERRAIN_LINK = "terrain"
STOCK_ENGINE = "gz-physics-bullet-plugin"
MATERIAL_NAMES = ("regolith_core", "regolith_ring", "regolith_far", "regolith_horizon", "rocks")
#: variant -> (lighting mode, forced engine or None = cfg.physics.engine)
VARIANTS: dict[str, tuple[str, str | None]] = {
    "": ("realistic", None),
    "inspect": ("inspection", None),
    "stock": ("realistic", "stock"),
    "inspect_stock": ("inspection", "stock"),
}
REQUIRED_SYSTEMS = {
    "gz::sim::systems::Physics": "gz-sim-physics-system",
    "gz::sim::systems::UserCommands": "gz-sim-user-commands-system",
    "gz::sim::systems::SceneBroadcaster": "gz-sim-scene-broadcaster-system",
}
FORBIDDEN_SYSTEMS = (
    "gz::sim::systems::Sensors",
    "gz::sim::systems::Imu",
    "gz::sim::systems::NavSat",
    "gz::sim::systems::PosePublisher",
)
#: ``<diffuse>`` of a material that has an albedo map (the map holds the absolute colour)
TEXTURED_DIFFUSE = (1.0, 1.0, 1.0, 1.0)
#: ``<roughness>`` written when a roughness map is present: Ogre-Next PBS multiplies the scalar
#: into the map, and the maps hold the absolute roughness (regolith 0.95, baked rock values).
TEXTURED_ROUGHNESS = 1.0
#: material specular colour. 0 removes the PBR specular lobe: lunar regolith and rocks have no
#: forward-specular sheen (their photometry is dominated by backscatter, which no ogre2 BRDF
#: models), and every look test of the mesh owner (docs/impl/mesh.md) used 0 0 0 1.
DEFAULT_SPECULAR = (0.0, 0.0, 0.0, 1.0)
#: default texture files (relative to the SDF). The regolith maps come from
#: materials/regolith.py (make_regolith_textures, prefix "regolith"), the rock atlas from the
#: rock library (copied into textures/).
REGOLITH_TEXTURES = {
    "albedo_map": "textures/regolith_albedo.png",
    "normal_map": "textures/regolith_normal.png",
    "roughness_map": "textures/regolith_roughness.png",
}
ROCK_TEXTURES = {
    "albedo_map": "textures/rock_atlas_basecolor.png",
    "normal_map": "textures/rock_atlas_normal.png",
    # grey map written by the rock library next to the glTF ORM atlas: ogre2 reads a roughness
    # map's first channel, and R of the ORM map is occlusion (= 1)
    "roughness_map": "textures/rock_atlas_roughness.png",
}


# ---------------------------------------------------------------------------------- specs
@dataclass
class VisualSpec:
    """One ``<visual>`` of the terrain link. ``uri`` is relative to the SDF (or absolute)."""

    name: str
    uri: str
    material: str
    cast_shadows: bool = True
    visibility_flags: int | None = None


@dataclass
class CollisionSpec:
    """One ``<collision>`` of the terrain model; ``mu2`` defaults to ``mu``."""

    name: str
    uri: str
    mu: float
    mu2: float | None = None

    @property
    def friction(self) -> tuple[float, float]:
        return float(self.mu), float(self.mu if self.mu2 is None else self.mu2)


@dataclass
class MaterialDef:
    """SDF PBR (metal workflow) material. Map paths are relative to the SDF or absolute."""

    name: str
    roughness: float
    metalness: float = 0.0
    base_color: tuple[float, float, float] = (0.12, 0.107, 0.088)
    albedo_map: str | None = None
    normal_map: str | None = None
    roughness_map: str | None = None
    metalness_map: str | None = None
    diffuse: tuple[float, float, float, float] | None = None  # None: derived (see module doc)
    specular: tuple[float, float, float, float] = DEFAULT_SPECULAR

    def maps(self) -> dict[str, str]:
        out = {}
        for key in ("albedo_map", "normal_map", "roughness_map", "metalness_map"):
            v = getattr(self, key)
            if v:
                out[key] = v
        return out

    def effective_diffuse(self) -> tuple[float, float, float, float]:
        if self.diffuse is not None:
            return tuple(float(v) for v in self.diffuse)  # type: ignore[return-value]
        if self.albedo_map:
            return TEXTURED_DIFFUSE
        r, g, b = self.base_color
        return (float(r), float(g), float(b), 1.0)

    def effective_roughness(self) -> float:
        """Scalar ``<roughness>``: 1 with a roughness map (Ogre multiplies scalar x map and the
        map holds the absolute value), else ``roughness``."""
        return TEXTURED_ROUGHNESS if self.roughness_map else float(self.roughness)


@dataclass
class PhysicsSettings:
    engine: str            # "patched" | "stock"
    engine_filename: str   # absolute .so path or library name
    step_s: float
    update_rate_hz: float
    gravity_m_s2: float


@dataclass
class WorldSummary:
    """What :func:`write_world` wrote (also useful for world_info.yaml)."""

    path: Path
    variant: str
    physics: PhysicsSettings
    lighting_mode: str
    sun_event: str
    n_visuals: int
    n_collisions: int
    links: dict[str, int] = field(default_factory=dict)


# ---------------------------------------------------------------------------------- helpers
def default_materials(cfg: Config) -> dict[str, MaterialDef]:
    """The five named materials with the default texture file names."""
    reg = cfg.materials.regolith
    base = tuple(float(v) for v in reg.base_color_linear)
    mats: dict[str, MaterialDef] = {}
    for zone in ("core", "ring", "far", "horizon"):
        mats[f"regolith_{zone}"] = MaterialDef(
            name=f"regolith_{zone}", roughness=float(reg.roughness), metalness=0.0,
            base_color=base, **REGOLITH_TEXTURES)  # type: ignore[arg-type]
    rocks = cfg.materials.rocks
    mats["rocks"] = MaterialDef(name="rocks", roughness=float(rocks.roughness), metalness=0.0,
                                base_color=base, **ROCK_TEXTURES)  # type: ignore[arg-type]
    return mats


def sdf_path_for(layout: Layout, variant: str) -> Path:
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r} (expected one of {sorted(VARIANTS)})")
    return layout.sdf_path(variant)


def physics_settings(cfg: Config, variant: str = "") -> PhysicsSettings:
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r} (expected one of {sorted(VARIANTS)})")
    engine = VARIANTS[variant][1] or str(cfg.physics.engine)
    if engine not in ("patched", "stock"):
        raise ValueError(f"physics.engine must be 'patched' or 'stock', got {engine!r}")
    step = float(cfg.physics.step_s[engine])
    if not step > 0:
        raise ValueError(f"physics.step_s.{engine} must be > 0")
    if engine == "patched":
        plugin = Path(str(cfg.paths.physics_plugin)).expanduser()
        if not plugin.is_absolute():
            raise ValueError(f"paths.physics_plugin must be absolute, got {plugin}")
        filename = str(plugin)
    else:
        filename = STOCK_ENGINE
    return PhysicsSettings(engine=engine, engine_filename=filename, step_s=step,
                           update_rate_hz=1.0 / step, gravity_m_s2=float(cfg.physics.gravity_m_s2))


def resolve_uri(uri: str, base_dir: Path) -> Path | None:
    """Local file a mesh/texture URI points to (None if it cannot be resolved locally).

    Supports relative paths (to ``base_dir``, the SDF folder), absolute paths, ``file://``
    and ``model://`` (searched in ``GZ_SIM_RESOURCE_PATH``).
    """
    if uri.startswith("file://"):
        return Path(uri[len("file://"):])
    if uri.startswith("model://"):
        rel = uri[len("model://"):]
        for root in filter(None, os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(os.pathsep)):
            p = Path(root) / rel
            if p.is_file():
                return p
        return None
    if "://" in uri:
        return None
    p = Path(uri)
    return p if p.is_absolute() else (base_dir / p)


def find_missing(uris: Iterable[str], base_dir: Path) -> list[str]:
    """URIs that do not resolve to an existing file (order preserved, duplicates removed)."""
    missing = []
    seen = set()
    for uri in uris:
        if uri in seen:
            continue
        seen.add(uri)
        p = resolve_uri(uri, base_dir)
        if p is None or not p.is_file():
            missing.append(uri)
    return missing


def _fmt(v: float) -> str:
    """Compact float formatting for SDF text (10 significant digits, integers without '.0')."""
    f = float(v)
    if f != f or f in (float("inf"), float("-inf")):
        raise ValueError(f"non-finite value {v!r} in SDF")
    if f.is_integer() and abs(f) < 1e15:
        return str(int(f))
    return f"{f:.10g}"


def _vec(values: Sequence[float]) -> str:
    return " ".join(_fmt(v) for v in values)


def _sub(parent: ET.Element, tag: str, text: str | None = None, **attrib: str) -> ET.Element:
    e = ET.SubElement(parent, tag, attrib)
    if text is not None:
        e.text = text
    return e


def _mu_token(v: float) -> str:
    return f"{v:.3f}".replace(".", "p").replace("-", "m")


def _check_names(kind: str, names: Sequence[str]) -> None:
    dup = [n for n, c in Counter(names).items() if c > 1]
    if dup:
        raise ValueError(f"duplicate {kind} names: {dup[:10]}")
    bad = [n for n in names if not n or any(ch in n for ch in " <>&\"'")]
    if bad:
        raise ValueError(f"invalid {kind} names: {bad[:10]}")


# ---------------------------------------------------------------------------------- XML
def _light_element(parent: ET.Element, light: LightSpec) -> None:
    if light.elevation_deg is not None:
        parent.append(ET.Comment(f" {light.name}: elevation {light.elevation_deg:.2f} deg, azimuth "
                                 f"{light.azimuth_deg:.2f} deg (from north through east) "))
    e = _sub(parent, "light", name=light.name, type=light.kind)
    _sub(e, "pose", _vec(light.pose))
    _sub(e, "cast_shadows", "true" if light.cast_shadows else "false")
    _sub(e, "intensity", _fmt(light.intensity))
    _sub(e, "diffuse", _vec(tuple(light.diffuse) + (1.0,)))
    _sub(e, "specular", _vec(tuple(light.specular) + (1.0,)))
    att = _sub(e, "attenuation")
    _sub(att, "range", "100000")
    _sub(att, "constant", "1")
    _sub(att, "linear", "0")
    _sub(att, "quadratic", "0")
    _sub(e, "direction", _vec(light.direction))


def _material_element(parent: ET.Element, mat: MaterialDef) -> None:
    m = _sub(parent, "material")
    _sub(m, "diffuse", _vec(mat.effective_diffuse()))
    _sub(m, "specular", _vec(mat.specular))
    pbr = _sub(m, "pbr")
    metal = _sub(pbr, "metal")
    for key, value in mat.maps().items():
        _sub(metal, key, value)
    _sub(metal, "metalness", _fmt(mat.metalness))
    _sub(metal, "roughness", _fmt(mat.effective_roughness()))


def build_world_element(cfg: Config, layout: Layout, visuals: Sequence[VisualSpec],
                        collisions: Sequence[CollisionSpec], variant: str = "",
                        materials: dict[str, MaterialDef] | None = None,
                        ) -> tuple[ET.Element, WorldSummary]:
    """Build the ``<sdf>`` element (no file system checks; see :func:`write_world`)."""
    if variant not in VARIANTS:
        raise ValueError(f"unknown variant {variant!r} (expected one of {sorted(VARIANTS)})")
    materials = materials if materials is not None else default_materials(cfg)
    mode = VARIANTS[variant][0]
    light: LightingSetup = lighting_setup(cfg, mode)
    phys = physics_settings(cfg, variant)
    _check_names("visual", [v.name for v in visuals])
    _check_names("collision", [c.name for c in collisions])
    unknown = sorted({v.material for v in visuals} - set(materials))
    if unknown:
        raise ValueError(f"visuals use undefined materials {unknown} (defined: {sorted(materials)})")
    site = cfg.site
    if abs(float(site.get("yaw_deg", 0.0))) > 1e-9:
        raise ValueError("site.yaw_deg != 0 is not supported: the SDF heading would have to be "
                         "verified against tools/dtm_extract.py's yaw convention first")

    sdf = ET.Element("sdf", version=SDF_VERSION)
    world = _sub(sdf, "world", name=layout.world_name)
    world.append(ET.Comment(
        f" generated by lunar_world_gen {__version__} (world/sdf.py); variant '{variant or 'default'}'; "
        f"engine {phys.engine}; lighting {mode} ({light.sun_event}); config hash {config_hash(cfg)} "))

    ph = _sub(world, "physics", name=f"{phys.engine}_{int(round(phys.step_s * 1000))}ms", type="ignored")
    _sub(ph, "max_step_size", _fmt(phys.step_s))
    _sub(ph, "real_time_factor", "1")
    _sub(ph, "real_time_update_rate", _fmt(phys.update_rate_hz))

    p = _sub(world, "plugin", name="gz::sim::systems::Physics", filename=REQUIRED_SYSTEMS["gz::sim::systems::Physics"])
    eng = _sub(p, "engine")
    _sub(eng, "filename", phys.engine_filename)
    for name in ("gz::sim::systems::UserCommands", "gz::sim::systems::SceneBroadcaster"):
        _sub(world, "plugin", name=name, filename=REQUIRED_SYSTEMS[name])
    world.append(ET.Comment(" Sensors, Imu, NavSat, PosePublisher and gz_ros2_control come with the robot "))

    _sub(world, "gravity", _vec((0.0, 0.0, -phys.gravity_m_s2)))
    _sub(world, "magnetic_field", "0 0 0")

    sc = _sub(world, "spherical_coordinates")
    _sub(sc, "surface_model", str(cfg.spherical_coordinates.get("surface_model", "CUSTOM_SURFACE")))
    axis = float(cfg.spherical_coordinates.get("axis_m", site.sphere_radius_m))
    if sc.find("surface_model").text == "CUSTOM_SURFACE":  # type: ignore[union-attr]
        _sub(sc, "surface_axis_equatorial", _fmt(axis))
        _sub(sc, "surface_axis_polar", _fmt(axis))
    _sub(sc, "world_frame_orientation", "ENU")
    _sub(sc, "latitude_deg", _fmt(float(site.lat_deg)))
    _sub(sc, "longitude_deg", _fmt(float(site.lon_deg)))
    _sub(sc, "elevation", _fmt(float(site.href_m)))
    _sub(sc, "heading_deg", "0")

    scene = _sub(world, "scene")
    _sub(scene, "ambient", _vec(light.ambient))
    _sub(scene, "background", _vec(light.background))
    _sub(scene, "shadows", "true" if light.shadows else "false")
    _sub(scene, "grid", "false")
    _sub(scene, "origin_visual", "false")
    for ls in light.lights:
        _light_element(world, ls)

    model = _sub(world, "model", name=TERRAIN_MODEL)
    _sub(model, "static", "true")
    _sub(model, "pose", "0 0 0 0 0 0")
    main_link = _sub(model, "link", name=TERRAIN_LINK)
    for v in visuals:
        ve = _sub(main_link, "visual", name=v.name)
        _sub(ve, "cast_shadows", "true" if v.cast_shadows else "false")
        if v.visibility_flags is not None:
            if not 0 <= int(v.visibility_flags) < 2 ** 32:
                raise ValueError(f"visual {v.name}: visibility_flags out of range")
            _sub(ve, "visibility_flags", str(int(v.visibility_flags)))
        geo = _sub(ve, "geometry")
        mesh = _sub(geo, "mesh")
        _sub(mesh, "uri", v.uri)
        _material_element(ve, materials[v.material])

    links = {TERRAIN_LINK: 0}
    if collisions:
        groups: dict[tuple[float, float], list[CollisionSpec]] = {}
        for c in collisions:
            mu, mu2 = c.friction
            if not (mu >= 0 and mu2 >= 0):
                raise ValueError(f"collision {c.name}: friction must be >= 0, got {(mu, mu2)}")
            groups.setdefault((round(mu, 6), round(mu2, 6)), []).append(c)
        main_key = max(groups, key=lambda k: (len(groups[k]), -list(groups).index(k)))
        for key, members in groups.items():
            if key == main_key:
                link, lname = main_link, TERRAIN_LINK
            else:
                lname = f"{TERRAIN_LINK}_mu_{_mu_token(key[0])}_{_mu_token(key[1])}"
                link = _sub(model, "link", name=lname)
            links[lname] = len(members)
            for c in members:
                ce = _sub(link, "collision", name=c.name)
                geo = _sub(ce, "geometry")
                mesh = _sub(geo, "mesh")
                _sub(mesh, "uri", c.uri)
                surf = _sub(ce, "surface")
                fr = _sub(surf, "friction")
                ode = _sub(fr, "ode")
                _sub(ode, "mu", _fmt(key[0]))
                _sub(ode, "mu2", _fmt(key[1]))
    summary = WorldSummary(path=Path(), variant=variant, physics=phys, lighting_mode=mode,
                           sun_event=light.sun_event, n_visuals=len(visuals),
                           n_collisions=len(collisions), links=links)
    return sdf, summary


def to_xml_string(sdf: ET.Element) -> str:
    ET.indent(sdf, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"


def write_world(cfg: Config, layout: Layout, visuals: Sequence[VisualSpec],
                collisions: Sequence[CollisionSpec], variant: str = "", *,
                materials: dict[str, MaterialDef] | None = None,
                out_path: str | Path | None = None, require_textures: bool = True,
                validate: bool = False) -> Path:
    """Write one world variant and return its path.

    ``out_path`` defaults to ``layout.sdf_path(variant)``; relative URIs are checked against
    its folder. Raises ``FileNotFoundError`` (listing every missing file) before writing
    anything if a mesh URI - or, with ``require_textures``, a texture - does not resolve.
    ``validate=True`` also runs ``gz sdf -k`` on the result (needs Gazebo in PATH) and raises
    ``RuntimeError`` if it reports errors.
    """
    materials = materials if materials is not None else default_materials(cfg)
    path = Path(out_path) if out_path is not None else sdf_path_for(layout, variant)
    base = path.parent
    mesh_uris = [v.uri for v in visuals] + [c.uri for c in collisions]
    missing = find_missing(mesh_uris, base)
    used = {v.material for v in visuals}
    if require_textures:
        tex = [u for name in sorted(used) if name in materials for u in materials[name].maps().values()]
        missing += [u for u in find_missing(tex, base) if u not in missing]
    if missing:
        raise FileNotFoundError(
            f"{len(missing)} file(s) referenced by world '{path.name}' do not exist (relative to {base}): "
            + ", ".join(missing[:20]) + (" ..." if len(missing) > 20 else ""))
    sdf, summary = build_world_element(cfg, layout, visuals, collisions, variant, materials)
    text = to_xml_string(sdf)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.tmp{os.getpid()}_{uuid.uuid4().hex[:8]}")   # unique (CR-06)
    try:
        tmp.write_text(text)
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)
    if validate:
        ok, out = validate_sdf(path)
        if not ok:
            raise RuntimeError(f"gz sdf -k {path} failed:\n{out}")
    return path


def write_all_variants(cfg: Config, layout: Layout, visuals: Sequence[VisualSpec],
                       collisions: Sequence[CollisionSpec], **kwargs) -> dict[str, Path]:
    """Write every variant ('', 'inspect', 'stock', 'inspect_stock'); returns {variant: path}."""
    return {v: write_world(cfg, layout, visuals, collisions, v, **kwargs) for v in VARIANTS}


def summarize_world(cfg: Config, layout: Layout, visuals: Sequence[VisualSpec],
                    collisions: Sequence[CollisionSpec], variant: str = "",
                    materials: dict[str, MaterialDef] | None = None) -> WorldSummary:
    """Summary of what :func:`write_world` would write (no file access)."""
    _, summary = build_world_element(cfg, layout, visuals, collisions, variant, materials)
    summary.path = sdf_path_for(layout, variant)
    return summary


def validate_sdf(path: str | Path, gz: str | None = None, timeout_s: float = 120.0) -> tuple[bool, str]:
    """Run ``gz sdf -k <path>``; returns (ok, combined output).

    ok is True when the command exits 0 and prints no "Error". Needs the Gazebo CLI (``gz``)
    on PATH (system install / ROS Jazzy vendor packages).
    """
    exe = gz or shutil.which("gz")
    if not exe:
        raise FileNotFoundError("the Gazebo CLI 'gz' is not on PATH; source /opt/ros/jazzy/setup.bash")
    res = subprocess.run([exe, "sdf", "-k", str(path)], capture_output=True, text=True, timeout=timeout_s)
    out = (res.stdout or "") + (res.stderr or "")
    ok = res.returncode == 0 and "Error" not in out
    return ok, out.strip()


def read_engine_filename(sdf_file: str | Path) -> str | None:
    """The Physics system's ``<engine><filename>`` of a world file (None if absent)."""
    root = ET.parse(str(sdf_file)).getroot()
    for plugin in root.iter("plugin"):
        if plugin.get("name") == "gz::sim::systems::Physics":
            fn = plugin.find("engine/filename")
            return fn.text.strip() if fn is not None and fn.text else None
    return None
