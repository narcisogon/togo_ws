"""Integration build of the Apollo 16 world (docs/DESIGN.md sections 3-13).

CLI (WSL, project venv, from ``src/lunar_world_gen``)::

    python -m lunar_world_gen.build [--config PATH] [--preset full|fast] [--stages a,b,...]
                                    [--force] [--force-downstream] [--adopt a,b,...] [--set key=value ...]
                                    [--status] [--lock-timeout S]

Stages, in order. Each one writes ``build_dir/<stage>/manifest.json`` (key, run id, inputs, config-
section hash, outputs, timings, peak RSS, results) and is skipped when its key is unchanged and its
outputs still exist. The key is a hash of the stage version, the keys AND run ids of its upstream
stages (a re-run upstream stage invalidates everything downstream of it), the stage's config
sections (including the output paths it writes to), the source of the modules it calls and of the
foundation modules (``grid``, ``layout``, ``config``, this file), the versions of numpy / scipy /
numba / pydelatin / Pillow / PyYAML, and the input files it reads (rock library: every file's size
and mtime plus ``library.json``; site report; physics plugin presence; ``gz`` CLI presence).

=========  ===========================================================================================
terrain    ``terrain.compose.build_all`` (DTM base, fBm, craters, H0 core tiles, ring/far/horizon
           grids). The terrain module keeps its own caches and ``terrain/manifest.json``; the build
           adds ``terrain/build_record.json`` with a key over the terrain sources
           (``terrain/*.py``, ``tools/dtm_extract.py``), the foundation and the library versions, so
           editing terrain code rebuilds the terrain (``--adopt terrain`` accepts the cache instead).
rocks      ``rocks.population.sample_rocks`` for the core (ground: the H0 core assembled from the
           cached tiles including their 0.5 m margins, ``height_at`` beyond that) and the ring
           (ground: the ring H0 grid; ``avoid=`` the core catalog, so the ring and its core-edge band
           never interpenetrate core rocks that cross the edge); crater sources from the terrain 'big'
           set. Writes the PLACED catalog ``rocks/rocks_catalog.npz`` + ``.csv``.
heights    final core tiles ``H = apply_fillets(H0 tile)`` (1 cm, with the 0.5 m margin), cached per
           tile in ``heights/tile_ii_jj``. Ring, far and horizon keep H0.
ground     the RENDERED visual ground: 100 core tiles at ``core_visual_max_error_m`` (per-tile grid
           normals; optional seam-band refinement ``mesh.core_visual_seam_*``) and the ring / far /
           horizon strips, stitched and proven with ``check_stitched``; ``ground/visual/*.npz``.
contact    every rock without a regolith fillet (``rocks.fillets.fillet_mask``) is re-seated on the rendered ground TIN
           (``rocks.recontact``, the placement contact rule: lowest visual-LOD vertex touches the
           surface, then burial), so no rock hovers above or vanishes below what Gazebo draws
           (CR-05, FID-08, FID-10). Writes the FINAL catalog ``contact/rocks_catalog.npz/.csv``
           and copies both into ``world_dir``.
collision  core collision tiles from the final H subsampled to 2 cm (margin 0.5 m >= the 15 cm the
           low-pass needs) and the final catalog through ``rocks.collision_field.core_collision_grid``,
           cropped to the tile; the assembled 2 cm core grid (spawn search, validation); the ring grid
           (``ring_collision_grid``, rock tops max-pooled over +- ``mesh.collision.ring_rock_dilation``
           node spacings so the 25 cm grid covers every ring rock's footprint, FID-08) and the far grid
           (H0 + ground offset).
meshes     collision pieces (100 core tiles at ``mesh.collision.core_max_error_m``, refined to
           ``mesh.collision.core_max_edge_m`` by conforming longest-edge bisection; ring and far
           strips) stitched and checked; merged rock meshes per core tile and ring strip (float32,
           one tile at a time); triangle counts and the DESIGN 13a budgets (hard limits fail the
           build, ``*_warn_tris`` only warn).
export     GLBs (``mesh.gltf.write_glb``) into ``world_dir/meshes`` with the DESIGN 6 names, the
           regolith textures and the rock atlas maps (``rock_atlas_{basecolor,normal,roughness}.png``)
           into ``world_dir/textures`` (all written to temporary names and renamed). Stale GLBs of
           earlier builds are removed.
world      test sites (``world.spawn.find_sites`` on the 2 cm core collision grid), ``world_info.yaml``
           and the four SDF variants (``world.sdf.write_all_variants``: default, inspect, stock,
           inspect_stock), each validated with ``gz sdf -k``.
validate   ``validate.static_checks.run_static_checks`` (H0 tiles for the roughness check) and
           ``validate.report.build_report`` into ``world_dir/reports``. Gazebo results (bench / LiDAR /
           robot JSONs, LiDAR captures, camera images) older than this build's SDF describe an earlier
           world and are moved to ``reports/stale_before_<time>/`` first (``archive_stale_results``).
=========  ===========================================================================================

``--stages`` runs a subset; every upstream stage that is not in the subset must be up to date
(else the build stops and says which stage to add). ``--force`` re-runs the selected stages even if
they are cached; ``--force-downstream`` also adds and re-runs every stage downstream of them.
``--adopt`` marks the cached outputs of the named stages as current without re-running them (e.g.
after a comment-only code edit). A build takes an exclusive lock on ``build_dir/.build.lock`` and
``world_dir/.build.lock`` (a second build on the same dirs fails fast, or waits ``--lock-timeout``
seconds). A non-default ``--preset`` builds into ``<build_dir>_<preset>`` / ``<world_dir>_<preset>``
unless ``paths.build_dir`` / ``paths.world_dir`` are given with ``--set``. Exit code: 0 = built (and
validated, if that stage ran and every check ran and passed), 1 = a stage failed, 2 = the static
validation reported failures or skipped checks.

Peak memory per stage is the process high-water mark (``VmHWM``), reset before each stage through
``/proc/self/clear_refs``; children (dtm_extract) are reported by the terrain module.
"""
from __future__ import annotations

import argparse
import contextlib
import functools
import hashlib
import json
import math
import os
import platform
import resource
import shutil
import sys
import time
import traceback
import uuid
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np

from . import __version__
from .config import Config, config_hash, load_config
from .grid import Grid
from .layout import RING_SIDES, Layout, Rect, TileId

STAGES = ("terrain", "rocks", "heights", "ground", "contact", "collision", "meshes", "export", "world", "validate")
#: bump a stage's version when its logic in this file changes (its cache is then rebuilt)
STAGE_VERSION = {"terrain": "2", "rocks": "3", "heights": "3", "ground": "1", "contact": "2", "collision": "3",
                 "meshes": "2", "export": "3", "world": "2", "validate": "4"}
UPSTREAM = {
    "terrain": (),
    "rocks": ("terrain",),
    "heights": ("terrain", "rocks"),
    "ground": ("terrain", "heights"),
    "contact": ("rocks", "heights", "ground"),
    "collision": ("terrain", "heights", "contact"),
    "meshes": ("terrain", "ground", "contact", "collision"),
    "export": ("ground", "meshes"),
    "world": ("heights", "ground", "contact", "collision", "meshes", "export"),
    "validate": ("terrain", "rocks", "heights", "ground", "contact", "collision", "meshes", "export", "world"),
}
#: config sections (dotted keys) each stage depends on ("*" = the whole config, paths included).
#: Stages that write into world_dir carry ``paths.world_dir`` (CR-02).
CONFIG_KEYS: dict[str, tuple[str, ...] | str] = {
    "rocks": ("world.seed", "rocks", "zones.core", "zones.ring", "terrain.craters.d_over_D_fresh", "site.key"),
    "heights": ("world.seed", "terrain.fillets", "zones.core"),
    "ground": ("mesh.core_visual_max_error_m", "mesh.ring_visual_max_error_m", "mesh.far_visual_max_error_m",
               "mesh.horizon_visual_max_error_m", "mesh.uv_tile_m", "mesh.core_visual_seam_band_m",
               "mesh.core_visual_seam_max_edge_m", "zones", "budgets.core_tile_visual_max_tris"),
    "contact": ("rocks.contact_on_rendered_ground", "rocks.collision_min_D_m", "terrain.fillets", "zones",
                "paths.world_dir", "world.name"),
    "collision": ("mesh.collision.spacing_m", "mesh.collision.lowpass_wavelength_m",
                  "mesh.collision.ground_offset_m", "mesh.collision.rock_offset_m",
                  "mesh.collision.ring_rock_dilation", "rocks.D_min_m.ring", "zones"),
    "meshes": ("mesh.collision", "zones", "budgets"),
    "export": ("materials", "mesh.uv_tile_m", "terrain.roughness.hurst", "world.seed", "paths.world_dir",
               "world.name"),
    "world": ("world", "site", "physics", "lighting", "spherical_coordinates", "spawn", "zones", "materials",
              "paths.physics_plugin", "paths.world_dir", "budgets"),
    "validate": "*",
}
#: modules (relative to the package) whose source is part of a stage's key (plus FOUNDATION)
CODE = {
    "rocks": ("rocks/population.py", "rocks/placement.py", "rocks/catalog.py", "rocks/library.py"),
    "heights": ("rocks/fillets.py", "rocks/placement.py"),
    "ground": ("mesh/tin.py", "mesh/stitch.py", "mesh/normals.py", "mesh/uv.py", "mesh/types.py", "mesh/refine.py"),
    "contact": ("rocks/recontact.py", "rocks/placement.py", "rocks/catalog.py", "rocks/library.py", "mesh/locate.py",
                "rocks/fillets.py"),
    "collision": ("rocks/collision_field.py", "rocks/placement.py"),
    "meshes": ("mesh/tin.py", "mesh/stitch.py", "mesh/types.py", "mesh/refine.py", "rocks/merge.py",
               "rocks/catalog.py"),
    "export": ("mesh/gltf.py", "mesh/types.py", "materials/regolith.py"),
    "world": ("world/sdf.py", "world/spawn.py", "world/world_info.py", "world/lighting.py"),
    "validate": ("validate/static_checks.py", "validate/stats.py", "validate/report.py", "mesh/locate.py"),
}
FOUNDATION = ("__init__.py", "grid.py", "layout.py", "config.py", "build.py")
#: stages whose key includes the rock library digest
LIBRARY_STAGES = ("rocks", "heights", "contact", "collision", "meshes", "export")
#: DESIGN 13a defaults (overridden by the config section ``budgets``). ``*_max_*`` are hard limits
#: (BuildError), ``*_warn_*`` and ``core_collision_target_tris`` only warn.
DEFAULT_BUDGETS = {
    "core_tile_visual_max_tris": 200_000,
    "core_visual_max_tris": 10_000_000,
    "core_rocks_max_tris": 7_500_000,
    "core_rocks_warn_tris": 6_600_000,
    "visual_total_max_tris": 20_000_000,
    "visual_total_warn_tris": 16_000_000,
    "core_collision_target_tris": [250_000, 500_000],
    "core_collision_max_tris": 1_000_000,
    "collision_total_max_tris": 2_000_000,
    "collision_total_warn_tris": 1_000_000,
}
#: the per-tile visual cap is applied with this headroom so that pydelatin's overshoot (+1) and the
#: stitching inserts (up to ~15 per tile measured) never push a capped tile over the hard limit (CR-04)
TILE_CAP_HEADROOM = 0.03
ROCK_GLB_BASE_COLOR = (0.25, 0.24, 0.22, 1.0)       # fallback only: the SDF material (atlas) overrides it
COLLISION_GLB_COLOR = (0.5, 0.5, 0.5, 1.0)
NORMAL_MARGIN_M = {"ring": 1.0, "far": 4.0, "horizon": 40.0}   # grid window around a strip for its normals
SHADOW_ZONES = ("core", "ring")                     # far / horizon visuals do not cast shadows (see export)
PKG_DIR = Path(__file__).resolve().parent
PKG_ROOT = PKG_DIR.parent
RUN_ID_KEY = "run_id"


class BuildError(RuntimeError):
    """A stage cannot run or produced an invalid result."""


# =============================================================================================== helpers
def _json_default(o: Any) -> Any:
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o) if np.isfinite(o) else None
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (Path, Rect, TileId)):
        return str(o)
    if isinstance(o, set):
        return sorted(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")


def _plain(o: Any) -> Any:
    return json.loads(json.dumps(o, default=_json_default))


def _sha(obj: Any) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=_json_default).encode()).hexdigest()[:16]


def _file_sha(path: str | Path) -> str | None:
    p = Path(path)
    if not p.is_file():
        return None
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()[:16]


def _code_hash(rel_paths: Iterable[str]) -> str:
    return _sha({r: _file_sha(PKG_DIR / r) for r in rel_paths})


def terrain_code_hash() -> str:
    """Source of every terrain module and of tools/dtm_extract.py (CR-11)."""
    files = {str(p.relative_to(PKG_DIR)): _file_sha(p) for p in sorted((PKG_DIR / "terrain").glob("*.py"))}
    files["tools/dtm_extract.py"] = _file_sha(PKG_ROOT / "tools" / "dtm_extract.py")
    return _sha(files)


@functools.lru_cache(maxsize=1)
def env_versions() -> dict:
    """Python and third-party library versions that can change numerical outputs (CR-11)."""
    from importlib import metadata

    out = {"python": platform.python_version()}
    for dist in ("numpy", "scipy", "numba", "pydelatin", "Pillow", "PyYAML"):
        try:
            out[dist] = metadata.version(dist)
        except metadata.PackageNotFoundError:
            out[dist] = None
    return out


def library_digest(path: str | Path) -> str:
    """Digest of a rock library folder: ``library.json`` content plus every other file's relative
    path, size and mtime (the LOD / hull npz files and the atlas maps; CR-11)."""
    root = Path(path).expanduser()
    j = root / "library.json"
    if not j.is_file():
        raise FileNotFoundError(f"rock library not found: {j}")
    h = hashlib.sha256(j.read_bytes())
    for f in sorted(p for p in root.rglob("*") if p.is_file() and p.name != "library.json"):
        st = f.stat()
        h.update(f"{f.relative_to(root)}:{st.st_size}:{st.st_mtime_ns}\n".encode())
    return h.hexdigest()[:16]


def _section(cfg: Config, dotted: str) -> Any:
    node: Any = cfg
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = dict.__getitem__(node, part)
    return _plain(node)


def _opt_float(v: Any) -> float | None:
    """Optional positive float config value (None, 0, false or absent = off)."""
    if v in (None, False, "", "none", "None", "off"):
        return None
    f = float(v)
    return f if f > 0 else None


def _read_json(path: str | Path) -> dict | None:
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def _tmp_path(path: Path, suffix: str = "") -> Path:
    """A unique temporary sibling of ``path`` (pid + random; no dots, so suffix handling is safe)."""
    return path.with_name(f"{path.stem}_tmp{os.getpid()}_{uuid.uuid4().hex[:8]}{suffix}")


def _write_json(path: str | Path, obj: Any) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = _tmp_path(path, ".json")
    try:
        tmp.write_text(json.dumps(obj, indent=1, default=_json_default))
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    return path


def _copy_atomic(src: Path, dst: Path) -> Path:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = _tmp_path(dst, dst.suffix)
    try:
        shutil.copyfile(src, tmp)
        os.replace(tmp, dst)
    finally:
        tmp.unlink(missing_ok=True)
    return dst


def _reset_peak_rss() -> bool:
    """Reset the process high-water mark (VmHWM) so the next reading is the peak of one stage."""
    try:
        with open("/proc/self/clear_refs", "w") as f:
            f.write("5")
        return True
    except OSError:
        return False


def _peak_rss_mb() -> float:
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmHWM:"):
                return round(int(line.split()[1]) / 1024.0, 1)
    except OSError:
        pass
    return round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 1)


def _du(path: str | Path) -> int:
    p = Path(path)
    if p.is_file():
        return p.stat().st_size
    if not p.is_dir():
        return 0
    return sum(f.stat().st_size for f in p.rglob("*") if f.is_file())


def _save_grid(g: Grid, prefix: Path) -> None:
    """Grid.save through uniquely named temporary files (a crash or a concurrent writer never leaves
    a half-written cache entry)."""
    prefix = Path(prefix)
    tmp = prefix.with_name(f"{prefix.name}_tmp{os.getpid()}_{uuid.uuid4().hex[:8]}")
    try:
        g.save(tmp)
        os.replace(tmp.with_suffix(".npy"), prefix.with_suffix(".npy"))
        os.replace(tmp.with_suffix(".json"), prefix.with_suffix(".json"))
    finally:
        for s in (".npy", ".json"):
            tmp.with_suffix(s).unlink(missing_ok=True)


def _grid_meta(prefix: Path) -> dict | None:
    d = _read_json(Path(prefix).with_suffix(".json"))
    return None if d is None or not Path(prefix).with_suffix(".npy").is_file() else d.get("meta", {})


def _save_mesh(path: Path, mesh, float32: bool = False) -> int:
    arrays = {"vertices": np.asarray(mesh.vertices, dtype=np.float32 if float32 else np.float64),
              "triangles": np.asarray(mesh.triangles, dtype=np.uint32)}
    if mesh.normals is not None:
        arrays["normals"] = np.asarray(mesh.normals, dtype=np.float32)
    if mesh.uvs is not None:
        arrays["uvs"] = np.asarray(mesh.uvs, dtype=np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = _tmp_path(path, ".npz")
    try:
        np.savez(tmp, **arrays)
        os.replace(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    return path.stat().st_size


def _load_mesh(path: Path, name: str):
    from .mesh.types import Mesh

    with np.load(path) as d:
        return Mesh(d["vertices"], d["triangles"].astype(np.int64),
                    d["normals"] if "normals" in d else None, d["uvs"] if "uvs" in d else None, name=name)


def _load_vt(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with np.load(path) as d:
        return np.asarray(d["vertices"], dtype=np.float64), np.asarray(d["triangles"], dtype=np.int64)


def _save_catalog_atomic(cat: np.ndarray, prefix: Path, meta: dict) -> list[Path]:
    """rocks.catalog.save_catalog into a private temporary folder, then rename into place."""
    from .rocks import catalog as rcat

    prefix = Path(prefix)
    tmpdir = prefix.parent / f".tmp_catalog_{os.getpid()}_{uuid.uuid4().hex[:8]}"
    try:
        written = rcat.save_catalog(cat, tmpdir / prefix.name, meta=meta)
        out = []
        for p in written:
            dst = prefix.parent / p.name
            os.replace(p, dst)
            out.append(dst)
        return out
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def _stats(values: Iterable[float]) -> dict:
    v = np.asarray(list(values), dtype=np.float64)
    if v.size == 0:
        return {"n": 0}
    return {"n": int(v.size), "min": float(v.min()), "mean": float(v.mean()), "max": float(v.max()),
            "sum": float(v.sum())}


def _pct(v: np.ndarray, qs=(50, 90, 99)) -> dict:
    v = np.asarray(v, dtype=np.float64)
    if v.size == 0:
        return {"n": 0}
    out = {"n": int(v.size), "max": float(v.max())}
    out.update({f"p{q}": float(np.percentile(v, q)) for q in qs})
    return out


# =============================================================================================== lock
class BuildLock:
    """Exclusive ``fcntl.flock`` on lock files (build dir and world dir) for the duration of a build.

    A second build on the same dirs fails fast with the holder's pid (or waits ``timeout_s``). The
    lock is released by the kernel if the process dies. Without ``fcntl`` (Windows) it is a no-op.
    """

    def __init__(self, paths: Iterable[Path], timeout_s: float = 0.0):
        self.paths = [Path(p) for p in paths]
        self.timeout_s = float(timeout_s)
        self._fds: list[int] = []

    def __enter__(self) -> "BuildLock":
        try:
            import fcntl
        except ImportError:  # pragma: no cover - not on Linux
            return self
        try:
            for p in self.paths:
                p.parent.mkdir(parents=True, exist_ok=True)
                fd = os.open(p, os.O_RDWR | os.O_CREAT, 0o644)
                t0 = time.monotonic()
                while True:
                    try:
                        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except OSError:
                        if time.monotonic() - t0 >= self.timeout_s:
                            holder = os.pread(fd, 300, 0).decode("utf-8", "replace").strip()
                            os.close(fd)
                            raise BuildError(f"another build holds {p} ({holder or 'unknown holder'}); "
                                             "wait for it or use --lock-timeout") from None
                        time.sleep(0.5)
                os.ftruncate(fd, 0)
                os.pwrite(fd, f"pid {os.getpid()} host {platform.node()} since "
                              f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n".encode(), 0)
                self._fds.append(fd)
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *exc) -> None:
        try:
            import fcntl
        except ImportError:  # pragma: no cover
            return
        for fd in self._fds:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            finally:
                os.close(fd)
        self._fds = []


# =============================================================================================== grounds
class CoreGround:
    """Rock placement ground for the core (H0 without fillets).

    Bilinear on the 1 cm H0 grid assembled from the cached tiles including their margins (core
    +- 0.5 m); exact ``terrain.compose.height_at`` beyond it (contact points of rocks that overhang
    the margin). Normals by central differences over one node.
    """

    def __init__(self, cfg: Config, grid: Grid):
        self.cfg = cfg
        self.grid = grid

    def height(self, x, y) -> np.ndarray:
        from .terrain import compose as tc

        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = np.broadcast_shapes(x.shape, y.shape)
        xf = np.broadcast_to(x, shape).ravel()
        yf = np.broadcast_to(y, shape).ravel()
        out = np.empty(xf.shape)
        inside = self.grid.contains(xf, yf)
        if inside.any():
            out[inside] = self.grid.sample(xf[inside], yf[inside], order=1)
        if (~inside).any():
            out[~inside] = tc.height_at(self.cfg, xf[~inside], yf[~inside], "core").astype(np.float64)
        return out.reshape(shape)

    def normal(self, x, y) -> np.ndarray:
        h = self.grid.spacing
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        gx = (self.height(x + h, y) - self.height(x - h, y)) / (2 * h)
        gy = (self.height(x, y + h) - self.height(x, y - h)) / (2 * h)
        n = np.stack([-gx, -gy, np.ones(np.shape(gx))], axis=-1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)


class TileHeights:
    """Visual height function H(x, y) of the built world: bilinear on the final core tiles (with
    fillets), ``height_at`` (H0) outside the core."""

    def __init__(self, cfg: Config, prefix_of: Callable[[TileId], Path]):
        self.cfg = cfg
        self.lay = Layout(cfg)
        self.prefix_of = prefix_of
        self._grids: dict[tuple[int, int], Grid] = {}

    def _grid(self, i: int, j: int) -> Grid:
        if (i, j) not in self._grids:
            self._grids[(i, j)] = Grid.load(self.prefix_of(TileId(i, j)), mmap=True)
        return self._grids[(i, j)]

    def __call__(self, x, y) -> np.ndarray:
        from .terrain import compose as tc

        x = np.atleast_1d(np.asarray(x, dtype=np.float64)).ravel()
        y = np.atleast_1d(np.asarray(y, dtype=np.float64)).ravel()
        core = self.lay.zone_rect("core")
        T = self.lay.tile_size
        n = self.lay.n_tiles
        out = np.empty(x.shape)
        inside = core.contains(x, y)
        ti = np.clip(np.floor((x - core.x0) / T), 0, n - 1).astype(np.int64)
        tj = np.clip(np.floor((y - core.y0) / T), 0, n - 1).astype(np.int64)
        for i, j in set(zip(ti[inside].tolist(), tj[inside].tolist())):
            sel = inside & (ti == i) & (tj == j)
            out[sel] = self._grid(i, j).sample(x[sel], y[sel], order=1)
        if (~inside).any():
            out[~inside] = tc.height_at(self.cfg, x[~inside], y[~inside], "core").astype(np.float64)
        return out


def assemble_core_with_margin(cfg: Config) -> Grid:
    """The 1 cm H0 core including the tiles' margins (core + 2 x margin, e.g. 10101^2 nodes), from the
    cached terrain tiles (overlapping margins are bit-identical, see docs/impl/terrain.md)."""
    from .terrain import compose as tc

    lay = Layout(cfg)
    core = lay.zone_rect("core")
    sp = float(cfg.zones.core.spacing_m)
    per = int(round(lay.tile_size / sp))
    m = int(round(float(cfg.zones.core.margin_m) / sp))
    n = lay.n_tiles * per + 2 * m + 1
    z = np.empty((n, n), dtype=np.float32)
    for t in lay.tiles():
        g = tc.load_core_tile(cfg, t, mmap=True, build=False)
        if g.nx != per + 2 * m + 1 or g.ny != per + 2 * m + 1:
            raise BuildError(f"{t.name}: unexpected H0 tile shape {g.z.shape}")
        z[t.j * per:t.j * per + g.ny, t.i * per:t.i * per + g.nx] = g.z
    return Grid(z, core.x0 - m * sp, core.y0 - m * sp, sp, {"assembled_from": "terrain core_tiles with margins"})


def downstream(stages: Iterable[str]) -> set[str]:
    """Every stage that (transitively) depends on one of ``stages``."""
    out: set[str] = set()
    todo = set(stages)
    while todo:
        s = todo.pop()
        for d, ups in UPSTREAM.items():
            if s in ups and d not in out:
                out.add(d)
                todo.add(d)
    return out


# =============================================================================================== builder
class Builder:
    """Runs the stages; see the module docstring."""

    def __init__(self, cfg: Config, *, force: bool = False, explicit_stages: Iterable[str] | None = None,
                 log: Callable[[str], None] | None = None, force_downstream: bool = False,
                 adopt: Iterable[str] | None = None, lock_timeout_s: float = 0.0):
        self.cfg = cfg
        self.lay = Layout(cfg)
        self.force = bool(force)
        self.force_downstream = bool(force_downstream)
        self.explicit = set(explicit_stages) if explicit_stages else None
        self.adopt = set(adopt or ())
        self.lock_timeout_s = float(lock_timeout_s)
        self._t0 = time.time()
        self._log = log
        self._keys: dict[str, str] = {}
        self._lib = None
        self._lib_digest: str | None = None
        self._forced_extra: set[str] = set()
        self.records: dict[str, dict] = {}
        self.started = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    # ------------------------------------------------------------------------------- plumbing
    def log(self, msg: str) -> None:
        line = f"[build +{time.time() - self._t0:7.1f}s] {msg}"
        if self._log:
            self._log(line)
        else:
            print(line, flush=True)

    @property
    def library(self):
        if self._lib is None:
            from .rocks.library import RockLibrary

            self._lib = RockLibrary(self.cfg.paths.rock_library)
        return self._lib

    @property
    def library_digest(self) -> str:
        if self._lib_digest is None:
            self._lib_digest = library_digest(self.cfg.paths.rock_library)
        return self._lib_digest

    def budgets(self) -> dict:
        b = dict(DEFAULT_BUDGETS)
        b.update(_plain(self.cfg.get("budgets") or {}))
        return b

    def stage_dir(self, stage: str) -> Path:
        return self.lay.stage_dir(stage)

    def manifest_path(self, stage: str) -> Path:
        if stage == "terrain":
            return self.lay.build_dir / "terrain" / "manifest.json"       # written by terrain.compose
        return self.lay.build_dir / stage / "manifest.json"

    def terrain_record_path(self) -> Path:
        return self.lay.build_dir / "terrain" / "build_record.json"

    def heights_prefix(self, t: TileId) -> Path:
        return self.lay.build_dir / "heights" / t.name

    def collision_prefix(self, t: TileId) -> Path:
        return self.lay.build_dir / "collision" / f"{t.name}_collision"

    def placed_catalog_path(self) -> Path:
        return self.lay.build_dir / "rocks" / "rocks_catalog.npz"

    def catalog_path(self) -> Path:
        """The FINAL catalog (re-seated on the rendered ground; what the world holds)."""
        return self.lay.build_dir / "contact" / "rocks_catalog.npz"

    # ------------------------------------------------------------------------------- keys
    def run_id(self, stage: str) -> str | None:
        """Identity of the last successful execution of a stage (None if never built)."""
        if stage == "terrain":
            rec = _read_json(self.terrain_record_path()) or {}
            return rec.get(RUN_ID_KEY)
        man = _read_json(self.manifest_path(stage)) or {}
        return man.get(RUN_ID_KEY) if man.get("status") == "ok" else None

    def inputs(self, stage: str) -> dict:
        cfg = self.cfg
        if stage == "terrain":
            from .terrain import compose as tc

            m = tc.get_model(cfg)
            return {"terrain_key": m.key, "terrain_version": tc.TERRAIN_VERSION, "build_stage_version":
                    STAGE_VERSION["terrain"], "terrain_code": terrain_code_hash(), "foundation": _code_hash(FOUNDATION),
                    "env": env_versions()}
        keys = CONFIG_KEYS[stage]
        conf = config_hash(cfg) if keys == "*" else {k: _section(cfg, k) for k in keys}
        d: dict[str, Any] = {"stage": stage, "stage_version": STAGE_VERSION[stage], "generator": __version__,
                             "upstream": {u: [self.key(u), self.run_id(u)] for u in UPSTREAM[stage]},
                             "config": conf, "code": _code_hash(tuple(CODE[stage]) + FOUNDATION),
                             "env": env_versions()}
        if stage in LIBRARY_STAGES:
            d["library_digest"] = self.library_digest
        if stage == "rocks":
            d["site_yaml"] = _file_sha(cfg.paths.site_yaml)
        if stage == "world":
            d["physics_plugin_present"] = Path(str(cfg.paths.physics_plugin)).expanduser().is_file()
            d["sdf_validator"] = shutil.which("gz") is not None        # CR-08c: validated once gz exists
        return d

    def key(self, stage: str) -> str:
        if stage not in self._keys:
            self._keys[stage] = _sha(self.inputs(stage))
        return self._keys[stage]

    def up_to_date(self, stage: str) -> tuple[bool, str]:
        try:
            if stage == "terrain":
                return self._terrain_up_to_date()
            key = self.key(stage)
        except FileNotFoundError as exc:
            return False, f"input missing: {exc}"
        man = _read_json(self.manifest_path(stage))
        if not man:
            return False, "never built"
        if man.get("status") != "ok":
            return False, f"last run ended with status '{man.get('status')}'"
        if man.get("key") != key:
            return False, "inputs changed"
        roots = (self.lay.build_dir.resolve(), self.lay.world_dir.resolve())
        bad = foreign = 0
        for o in man.get("outputs", []):
            p = Path(o["path"])
            if not any(p.resolve().is_relative_to(r) for r in roots):
                foreign += 1
            elif not p.is_file() or (o.get("bytes") is not None and p.stat().st_size != o["bytes"]):
                bad += 1
        if foreign:
            return False, f"{foreign} recorded output(s) outside the current build/world dirs"
        if bad:
            return False, f"{bad} output file(s) missing or changed"
        return True, "up to date"

    def _terrain_cache_ok(self) -> tuple[bool, str]:
        from .terrain import base as tbase
        from .terrain import compose as tc
        from .terrain import craters as tcr

        cfg = self.cfg
        key = tc.get_model(cfg).key
        man = _read_json(self.manifest_path("terrain"))
        if not man or man.get("terrain_key") != key:
            return False, "terrain manifest missing or for another terrain key"
        stale = [t.name for t in self.lay.tiles() if not tc._cached(tc.tile_prefix(cfg, t), key)]
        if stale:
            return False, f"{len(stale)} H0 core tile(s) missing or stale"
        zones = [z for z in ("ring", "far", "horizon") if not tc._cached(tc.zone_height_prefix(cfg, z), key)]
        if zones:
            return False, f"zone grid(s) {zones} missing or stale"
        missing = [w for w in tcr.WHICH if not tcr.craters_path(cfg, w).is_file()]
        missing += [z for z in tbase.BASE_ZONES if not tbase.base_prefix(cfg, z).with_suffix(".npy").is_file()]
        if missing:
            return False, f"terrain components {missing} missing"
        return True, "cached (terrain key unchanged)"

    def _terrain_up_to_date(self) -> tuple[bool, str]:
        ok, why = self._terrain_cache_ok()
        if not ok:
            return ok, why
        rec = _read_json(self.terrain_record_path())
        if not rec:
            return False, "no build record (terrain built by an older build.py or outside it); re-run it or --adopt terrain"
        if rec.get("key") != self.key("terrain"):
            return False, "terrain code, foundation or library versions changed"
        return True, why

    def _forced(self, stage: str) -> bool:
        if stage in self._forced_extra:
            return True
        if not self.force:
            return False
        if stage == "terrain":
            return self.explicit is not None and "terrain" in self.explicit
        return True

    # ------------------------------------------------------------------------------- driver
    def status(self) -> dict[str, tuple[bool, str]]:
        out = {}
        for s in STAGES:
            try:
                out[s] = self.up_to_date(s)
            except Exception as exc:  # noqa: BLE001 - --status must report, not crash (CR-08b)
                out[s] = (False, f"cannot evaluate: {type(exc).__name__}: {exc}")
        return out

    def _adopt(self, stage: str) -> None:
        """Mark a stage's cached outputs as current (see --adopt)."""
        if stage == "terrain":
            ok, why = self._terrain_cache_ok()
            if not ok:
                raise BuildError(f"cannot adopt terrain: {why}")
            _write_json(self.terrain_record_path(), {"key": self.key("terrain"), RUN_ID_KEY: uuid.uuid4().hex,
                                                     "inputs": self.inputs("terrain"), "adopted": True,
                                                     "time": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
        else:
            man = _read_json(self.manifest_path(stage))
            if not man or man.get("status") != "ok":
                raise BuildError(f"cannot adopt {stage}: no successful earlier run")
            missing = [o["path"] for o in man.get("outputs", []) if not Path(o["path"]).is_file()]
            if missing:
                raise BuildError(f"cannot adopt {stage}: {len(missing)} output(s) missing")
            man["adopted"] = {"from_key": man.get("key"), "time": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
            man["key"] = self.key(stage)
            _write_json(self.manifest_path(stage), man)
        self._keys.clear()
        self.log(f"{stage}: cached outputs adopted as current")

    def run(self, stages: Iterable[str]) -> dict:
        stages = list(stages)
        unknown = set(stages) - set(STAGES)
        if unknown:
            raise BuildError(f"unknown stage(s) {sorted(unknown)}; stages are {', '.join(STAGES)}")
        bad_adopt = self.adopt - set(STAGES)
        if bad_adopt:
            raise BuildError(f"unknown stage(s) to adopt {sorted(bad_adopt)}")
        sel = set(stages)
        if self.force_downstream:
            self._forced_extra = (sel | downstream(sel)) - ({"terrain"} if "terrain" not in sel else set())
            sel |= downstream(sel)
        order = [s for s in STAGES if s in sel]
        with BuildLock([self.lay.build_dir / ".build.lock", self.lay.world_dir / ".build.lock"],
                       timeout_s=self.lock_timeout_s):
            for s in [x for x in STAGES if x in self.adopt]:
                self._adopt(s)
            needed: set[str] = set()
            for s in order:
                stack = list(UPSTREAM[s])
                while stack:
                    u = stack.pop()
                    if u not in needed:
                        needed.add(u)
                        stack.extend(UPSTREAM[u])
            for u in [x for x in STAGES if x in needed and x not in order]:
                ok, why = self.up_to_date(u)
                if not ok:
                    raise BuildError(f"upstream stage '{u}' is not up to date ({why}); add it to --stages")
            self.log(f"config {self.cfg.get('_source', {}).get('config_file')} preset "
                     f"{self.cfg.world.get('preset')} (hash {config_hash(self.cfg)}); stages {','.join(order)}; "
                     f"build_dir {self.lay.build_dir}; world_dir {self.lay.world_dir}")
            for s in order:
                ok, why = self.up_to_date(s)
                if ok and not self._forced(s):
                    self.log(f"{s}: skipped ({why})")
                    man = _read_json(self.manifest_path(s)) or {}
                    self.records[s] = {"status": "skipped", "reason": why, "key": self.key(s),
                                       "elapsed_s": 0.0, "built_elapsed_s": man.get("elapsed_s"),
                                       "built_peak_rss_mb": man.get("peak_rss_mb")}
                    continue
                self.log(f"{s}: running ({'forced' if ok else why})")
                self._execute(s)
                self._keys.clear()                 # downstream keys include this stage's new run id
            self._write_build_manifest()
        return self.records

    def _execute(self, stage: str) -> dict:
        fn = getattr(self, f"stage_{stage}")
        _reset_peak_rss()
        t0 = time.time()
        started = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        run_id = uuid.uuid4().hex
        base = {"stage": stage, "key": self.key(stage), RUN_ID_KEY: run_id, "inputs": self.inputs(stage),
                "started": started, "generator": f"lunar_world_gen {__version__} (build.py)",
                "config": self.cfg.get("_source"), "config_hash": config_hash(self.cfg)}
        base["config_section_hash"] = _sha(base["inputs"].get("config") if stage != "terrain" else base["inputs"])
        try:
            res = fn()
        except Exception as exc:
            man = dict(base, status="error", error=f"{type(exc).__name__}: {exc}",
                       traceback=traceback.format_exc()[-4000:], elapsed_s=round(time.time() - t0, 2),
                       peak_rss_mb=_peak_rss_mb(), finished=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
            if stage != "terrain":
                _write_json(self.manifest_path(stage), man)
            self.records[stage] = {"status": "error", "error": man["error"], "elapsed_s": man["elapsed_s"]}
            self._write_build_manifest()
            raise
        elapsed = round(time.time() - t0, 2)
        peak = _peak_rss_mb()
        outputs = [{"path": str(p), "bytes": Path(p).stat().st_size} for p in res.get("outputs", [])]
        man = dict(base, status="ok", elapsed_s=elapsed, peak_rss_mb=peak, timings_s=res.get("timings", {}),
                   outputs=outputs, n_outputs=len(outputs), output_bytes=sum(o["bytes"] for o in outputs),
                   results=res.get("results", {}), finished=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
        if stage == "terrain":
            _write_json(self.terrain_record_path(), {"key": base["key"], RUN_ID_KEY: run_id, "inputs": base["inputs"],
                                                     "time": man["finished"]})
        else:
            _write_json(self.manifest_path(stage), man)
        self.records[stage] = {"status": "built", "key": man["key"], "elapsed_s": elapsed, "peak_rss_mb": peak,
                               "n_outputs": len(outputs), "output_bytes": man["output_bytes"]}
        self.log(f"{stage}: done in {elapsed:.1f} s, peak RSS {peak / 1024:.2f} GB")
        self._write_build_manifest()
        return man

    def _write_build_manifest(self) -> None:
        path = self.lay.build_dir / "build_manifest.json"
        old = _read_json(path) or {}
        stages = dict(old.get("stages", {}))
        stages.update(self.records)
        _write_json(path, {"generator": f"lunar_world_gen {__version__}", "config": self.cfg.get("_source"),
                           "config_hash": config_hash(self.cfg), "preset": self.cfg.world.get("preset"),
                           "build_dir": str(self.lay.build_dir), "world_dir": str(self.lay.world_dir),
                           "started": self.started, "updated": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                           "stages": stages})

    # =========================================================================== stages
    def stage_terrain(self) -> dict:
        from .terrain import compose as tc

        rebuild = self._forced("terrain") or (_read_json(self.manifest_path("terrain")) is not None
                                              and self._terrain_cache_ok()[0])
        # a valid terrain cache whose build record is missing / stale (code or library versions changed)
        # is rebuilt from scratch: the terrain module's own key does not cover its source code
        man = tc.build_all(self.cfg, rebuild=rebuild, log=self.log)
        tc.clear_caches()
        return {"results": {"terrain_key": man.get("terrain_key"), "rebuilt_all": bool(rebuild),
                            "stage_elapsed_s": {k: v.get("elapsed_s") for k, v in man.get("stages", {}).items()},
                            "peak_rss": man.get("peak_rss"), "disk_bytes": man.get("disk_bytes"),
                            "checks": man.get("checks"), "manifest": str(self.manifest_path("terrain"))}}

    # ---------------------------------------------------------------------------- rocks
    def stage_rocks(self) -> dict:
        from .rocks import catalog as rcat
        from .rocks import population as pop
        from .rocks.placement import GroundGrid
        from .terrain import compose as tc

        cfg, lib = self.cfg, self.library
        timings: dict[str, float] = {}
        t = time.time()
        craters = tc.get_model(cfg).craters("big")
        core_grid = assemble_core_with_margin(cfg)
        timings["assemble_core_H0"] = round(time.time() - t, 2)
        t = time.time()
        field = pop.density_field(cfg)
        core, meta_core = pop.sample_rocks(cfg, "core", CoreGround(cfg, core_grid), craters=craters, library=lib,
                                           field=field, return_meta=True)
        timings["sample_core"] = round(time.time() - t, 2)
        del core_grid
        t = time.time()
        ring_H = tc.load_zone_height(cfg, "ring", mmap=True, build=False)
        # avoid=core: the ring rocks and the core-edge band are kept clear of the core rocks that cross
        # the core edge (FID-09 overlap repair across zones; fix_rocks.md 9.1)
        ring, meta_ring = pop.sample_rocks(cfg, "ring", GroundGrid(ring_H), craters=craters, library=lib,
                                           field=field, return_meta=True, avoid=core)
        timings["sample_ring"] = round(time.time() - t, 2)
        cat = rcat.concatenate([core, ring])
        problems = rcat.validate_catalog(cat, lib, cfg)
        if problems:
            raise BuildError("invalid rock catalog: " + "; ".join(problems))
        stats = rcat.catalog_stats(cat, lib, cfg)
        rows_core = rcat.cumulative_counts(core, rcat.SFD_CHECK_D_M, meta_core["expected_counts"])
        rows_ring = rcat.cumulative_counts(ring, pop.RING_CHECK_D_M, meta_ring["expected_counts"])
        t = time.time()
        meta = {"generator": "lunar_world_gen.build (rocks stage: placed on H0)", "config_hash": config_hash(cfg),
                "stage_key": self.key("rocks"), "core": meta_core, "ring": meta_ring,
                "library": lib.summary(), "stats": stats}
        paths = _save_catalog_atomic(cat, self.stage_dir("rocks") / "rocks_catalog", meta)
        timings["save"] = round(time.time() - t, 2)
        tc.clear_caches()                               # CR-12: terrain process caches (~0.2 GB) not needed now
        results = {"n_total": int(len(cat)), "n_core": int(len(core)), "n_ring": int(len(ring)),
                   "n_core_collide": int(core["collide"].sum()), "core_counts": rows_core,
                   "ring_counts": rows_ring, "stats": stats, "largest_core_rock_m": float(core["D"].max()),
                   "core_intensity_integral_m2": meta_core.get("intensity_integral_m2"),
                   "core_intensity_scale": meta_core.get("intensity_scale"),
                   "separation": {"core": meta_core.get("separation"), "ring": meta_ring.get("separation")},
                   "overlap": {"core": meta_core.get("overlap"), "ring": meta_ring.get("overlap"),
                               "edge_band": (meta_ring.get("edge_band") or {}).get("overlap"),
                               "ring_avoids_core": True},
                   "n_ring_main": meta_ring.get("n_main"),
                   "n_ring_edge_band": (meta_ring.get("edge_band") or {}).get("n"),
                   "n_perched": {"core": meta_core.get("n_perched"), "ring": meta_ring.get("n_perched")},
                   "n_fresh_crater_annuli": len(meta_core.get("fresh_craters", {}).get("x", [])),
                   "catalog": [str(p) for p in paths]}
        self.log(f"rocks: core {len(core)} (collide {int(core['collide'].sum())}), ring {len(ring)}; "
                 f"core triangles {stats['zones']['core'].get('triangles')}")
        return {"results": results, "outputs": list(paths), "timings": timings}

    # ---------------------------------------------------------------------------- heights
    def stage_heights(self) -> dict:
        from .rocks.catalog import load_catalog
        from .rocks.fillets import apply_fillets, fillet_mask
        from .terrain import compose as tc

        cfg, lay, lib = self.cfg, self.lay, self.library
        key = self.key("heights")
        cat = load_catalog(self.placed_catalog_path())
        # exactly the rows apply_fillets fillets (burial > terrain.fillets.min_burial: perched rocks bedded
        # only by the compliance get none), so 'filleted_rocks' counts real fillets (fix_rocks.md 9.2)
        sub = cat[fillet_mask(cfg, cat)]
        self.stage_dir("heights")
        outputs, built, fmax, zmin, zmax = [], 0, 0.0, np.inf, -np.inf
        n_nodes_filleted = 0
        t0 = time.time()
        for t in lay.tiles():
            prefix = self.heights_prefix(t)
            meta = _grid_meta(prefix)
            if self._forced("heights") or not meta or meta.get("stage_key") != key:
                H0 = tc.load_core_tile(cfg, t, mmap=False, build=False)
                H = apply_fillets(cfg, H0, sub, lib)
                f = np.asarray(H.z, dtype=np.float64) - np.asarray(H0.z, dtype=np.float64)
                H.meta.update({"kind": "core_H_final", "description": "H = H0 + fillets (1 cm, tile + margin)",
                               "stage_key": key, "terrain_key": H0.meta.get("terrain_key"),
                               "fillet_max_m": float(f.max()), "fillet_nodes": int(np.count_nonzero(f > 0)),
                               "z_min": float(H.z.min()), "z_max": float(H.z.max())})
                _save_grid(H, prefix)
                built += 1
                meta = H.meta
                del H0, H, f
            fmax = max(fmax, float(meta.get("fillet_max_m", 0.0)))
            n_nodes_filleted += int(meta.get("fillet_nodes", 0))
            zmin, zmax = min(zmin, float(meta["z_min"])), max(zmax, float(meta["z_max"]))
            outputs += [prefix.with_suffix(".npy"), prefix.with_suffix(".json")]
        results = {"n_tiles": len(lay.tiles()), "built": built, "filleted_rocks": int(len(sub)),
                   "fillet_max_m": fmax, "fillet_nodes_incl_margins": n_nodes_filleted,
                   "z_range_with_margins_m": [zmin, zmax]}
        self.log(f"heights: {built} tile(s) built, {len(sub)} filleted rocks, max fillet {fmax * 1000:.1f} mm")
        return {"results": results, "outputs": outputs, "timings": {"tiles": round(time.time() - t0, 2)}}

    # ---------------------------------------------------------------------------- ground (visual)
    def stage_ground(self) -> dict:
        from .mesh.stitch import check_stitched, make_piece, stitch_pieces, strip_priority, tile_priority
        from .terrain import compose as tc

        cfg, lay = self.cfg, self.lay
        mcfg = cfg.mesh
        uv = mcfg.uv_tile_m
        B = self.budgets()
        n = lay.n_tiles
        gdir = self.stage_dir("ground") / "visual"
        if gdir.exists():
            for f in gdir.glob("*.npz"):
                f.unlink()
        gdir.mkdir(parents=True, exist_ok=True)
        timings: dict[str, float] = {}
        cap = int(B["core_tile_visual_max_tris"])
        cap_eff = max(int(cap * (1.0 - TILE_CAP_HEADROOM)), 2)
        err_core = float(mcfg.core_visual_max_error_m)
        seam_edge = _opt_float(mcfg.get("core_visual_seam_max_edge_m"))
        seam_band = _opt_float(mcfg.get("core_visual_seam_band_m"))
        if seam_edge and not seam_band:
            seam_band = seam_edge
        t = time.time()
        vis, capped, refine_reps = [], [], []
        for tt in lay.tiles():
            H = Grid.load(self.heights_prefix(tt), mmap=True)
            kw = dict(name=f"{tt.name}_ground", uv_tile_m=float(uv.core), normal_margin_m=float(cfg.zones.core.margin_m),
                      max_edge_m=seam_edge, max_edge_band_m=seam_band if seam_edge else None)
            p = make_piece(H, lay.tile_rect(tt), err_core, tile_priority(tt, n), **kw)
            if p.mesh.n_triangles > cap_eff:
                n0 = p.mesh.n_triangles
                rep = p.mesh.meta.get("refine") or {}
                extra = int(rep.get("triangles_after", 0)) - int(rep.get("triangles_before", 0))
                p = make_piece(H, lay.tile_rect(tt), err_core, tile_priority(tt, n),
                               max_triangles=max(cap_eff - extra, 2), **kw)
                capped.append({"tile": tt.name, "uncapped_triangles": n0, "capped_triangles": p.mesh.n_triangles,
                               "cap": cap, "cap_applied": cap_eff, "tin_error_m": p.mesh.meta.get("tin_error")})
                self.log(f"ground: {tt.name} capped {n0} -> {p.mesh.n_triangles} triangles "
                         f"(error {p.mesh.meta.get('tin_error', 0) * 1000:.2f} mm)")
            if p.mesh.meta.get("refine"):
                refine_reps.append(p.mesh.meta["refine"])
            vis.append(p)
        timings["core_visual_tin_normals"] = round(time.time() - t, 2)
        t = time.time()
        for zone in ("ring", "far", "horizon"):
            G = tc.load_zone_height(cfg, zone, mmap=True, build=False)
            err = float(mcfg[f"{zone}_visual_max_error_m"])
            for side, r in lay.strip_rects(zone).items():
                vis.append(make_piece(G, r, err, strip_priority(zone, side), name=f"{zone}_{side}_ground",
                                      uv_tile_m=float(uv[zone]), normal_margin_m=NORMAL_MARGIN_M[zone]))
        timings["strips_visual"] = round(time.time() - t, 2)
        pre_stitch = {p.name: p.mesh.n_triangles for p in vis}
        t = time.time()
        rep_v = stitch_pieces(vis)
        timings["stitch_visual"] = round(time.time() - t, 2)
        t = time.time()
        chk_v = check_stitched(vis)
        timings["check_visual"] = round(time.time() - t, 2)
        if not chk_v["ok"]:
            raise BuildError(f"visual stitching check failed: {json.dumps(_plain(chk_v))[:2000]}")
        pieces_out, outputs = [], []
        for p in vis:
            rel = f"ground/visual/{p.name}.npz"
            outputs.append(self.lay.build_dir / rel)
            _save_mesh(self.lay.build_dir / rel, p.mesh)
            zone = "core" if p.name.startswith("tile_") else p.name.split("_")[0]
            pieces_out.append(self._piece_record(p, "ground", zone, rel, {"pre_stitch_triangles": pre_stitch[p.name]}))
        del vis
        seam = None
        if refine_reps:
            seam = {"max_edge_m": seam_edge, "band_m": seam_band, "tiles": len(refine_reps),
                    "triangles_added": int(sum(r["triangles_after"] - r["triangles_before"] for r in refine_reps)),
                    "edges_over_limit_after": int(sum(r["edges_over_limit_after"] for r in refine_reps)),
                    "flips": int(sum(r["flips"] for r in refine_reps))}
        core_pre = [pre_stitch[p["name"]] for p in pieces_out if p["zone"] == "core"]
        results = {"pieces": pieces_out, "capped_tiles": capped, "cap": cap, "cap_applied": cap_eff,
                   "core_tile_pre_stitch_max": int(max(core_pre)) if core_pre else 0,
                   "stitch_visual": rep_v, "check_visual": chk_v, "seam_refinement": seam,
                   "errors_m": {"core_visual": err_core, "ring_visual": float(mcfg.ring_visual_max_error_m),
                                "far_visual": float(mcfg.far_visual_max_error_m),
                                "horizon_visual": float(mcfg.horizon_visual_max_error_m)}}
        self.log(f"ground: visual pieces stitched ({rep_v['n_inserted']} vertices inserted), check ok; core "
                 f"{sum(p['n_triangles'] for p in pieces_out if p['zone'] == 'core'):,} triangles")
        return {"results": results, "outputs": outputs, "timings": timings}

    @staticmethod
    def _piece_record(p, kind: str, zone: str, rel: str, extra: dict | None = None) -> dict:
        m = p.mesh
        rec = {"name": p.name, "kind": kind, "zone": zone, "file": rel,
               "rect": [p.rect.x0, p.rect.x1, p.rect.y0, p.rect.y1], "n_triangles": m.n_triangles,
               "n_vertices": m.n_vertices, "max_error_m": m.meta.get("max_error"),
               "tin_error_m": m.meta.get("tin_error")}
        rec.update(extra or {})
        return rec

    def rendered_ground(self):
        """Height function of the rendered (stitched) visual ground of this build: the core tiles,
        ring and far strips of the ground stage (``mesh.locate.PiecewiseTinHeight``)."""
        from .mesh.locate import PiecewiseTinHeight
        from .terrain import compose as tc

        man = _read_json(self.manifest_path("ground"))
        if not man or man.get("status") != "ok":
            raise BuildError("the ground stage has not been built")
        pieces = []
        for p in man["results"]["pieces"]:
            if p["zone"] == "horizon":
                continue
            path = self.lay.build_dir / p["file"]
            pieces.append((tuple(p["rect"]), functools.partial(_load_vt, path)))
        cfg = self.cfg
        return PiecewiseTinHeight(pieces, fallback=lambda x, y: tc.height_at(cfg, x, y, "core").astype(np.float64))

    # ---------------------------------------------------------------------------- contact
    def stage_contact(self) -> dict:
        from .rocks import catalog as rcat
        from .rocks.fillets import fillet_settings
        from .rocks.recontact import contact_clearance, fillet_rows, recontact

        cfg, lib = self.cfg, self.library
        timings: dict[str, float] = {}
        cat = rcat.load_catalog(self.placed_catalog_path())
        enabled = bool(cfg.rocks.get("contact_on_rendered_ground", True))
        ground = self.rendered_ground()
        fil = fillet_rows(cfg, cat)
        rows = ~fil
        t = time.time()
        lo0, hi0 = contact_clearance(ground, lib, cat)
        timings["clearance_before"] = round(time.time() - t, 2)
        t = time.time()
        new = recontact(cfg, cat, lib, ground, rows=rows) if enabled else cat.copy()
        timings["recontact"] = round(time.time() - t, 2)
        t = time.time()
        lo1, hi1 = contact_clearance(ground, lib, new)
        timings["clearance_after"] = round(time.time() - t, 2)
        problems = rcat.validate_catalog(new, lib, cfg)
        if problems:
            raise BuildError("invalid re-seated rock catalog: " + "; ".join(problems))
        dz = new["z"] - cat["z"]

        def summary(mask) -> dict:
            out = {"n": int(mask.sum())}
            for tag, lo, hi in (("before", lo0, hi0), ("after", lo1, hi1)):
                lo_m, hi_m = lo[mask], hi[mask]
                out[tag] = {"hover_gt_1mm": int((lo_m > 0.001).sum()), "hover_gt_5mm": int((lo_m > 0.005).sum()),
                            "hover_gt_20mm": int((lo_m > 0.02).sum()),
                            "hover_max_mm": float(lo_m.max() * 1e3) if lo_m.size else 0.0,
                            "submerged": int((hi_m < 0).sum())}
            if mask.any():
                out["dz_mm"] = {"p1": float(np.percentile(dz[mask], 1) * 1e3), "p50": float(np.median(dz[mask]) * 1e3),
                                "p99": float(np.percentile(dz[mask], 99) * 1e3),
                                "max_abs": float(np.abs(dz[mask]).max() * 1e3)}
            return out

        core, ring = cat["zone"] == 0, cat["zone"] == 1
        # perched rocks carry a compliance burial (rocks.perched_burial, <= the fillets' min_burial) since
        # the rock fix round, so burial == 0 no longer marks them; 'light burial' = burial <= min_burial
        # holds every compliance-only perched rock (and buried rocks with a small Beta draw)
        light = cat["burial"] <= float(fillet_settings(cfg)["min_burial"])
        results = {"enabled": enabled, "n_rows": int(len(cat)), "n_reseated": int(rows.sum()) if enabled else 0,
                   "n_filleted_kept": int(fil.sum()), "pieces_loaded": ground.n_loads,
                   "core_reseated": summary(core & rows), "core_light_burial": summary(core & rows & light),
                   "core_filleted": summary(fil), "ring": summary(ring),
                   "note": "clearance = min over the rock's contact (visual LOD) vertices of z - rendered ground z; "
                           "hover = clearance > threshold; submerged = every vertex below the rendered ground; "
                           "core_light_burial = re-seated core rocks with burial <= terrain.fillets.min_burial"}
        meta = {"generator": "lunar_world_gen.build (contact stage: re-seated on the rendered ground)",
                "config_hash": config_hash(cfg), "stage_key": self.key("contact"),
                "placed_catalog": str(self.placed_catalog_path()), "contact": {k: v for k, v in results.items()
                                                                               if k != "note"}}
        t = time.time()
        paths = _save_catalog_atomic(new, self.stage_dir("contact") / "rocks_catalog", meta)
        copies = [_copy_atomic(p, self.lay.world_dir / p.name) for p in paths]
        timings["save"] = round(time.time() - t, 2)
        results["catalog"] = [str(p) for p in paths]
        results["world_copies"] = [str(p) for p in copies]
        a, b = results["core_reseated"], results["ring"]
        self.log(f"contact: {results['n_reseated']} rocks re-seated on the rendered ground; core hovering > 1 mm "
                 f"{a['before']['hover_gt_1mm']} -> {a['after']['hover_gt_1mm']}, submerged "
                 f"{a['before']['submerged']} -> {a['after']['submerged']}; ring hover max "
                 f"{b['before']['hover_max_mm']:.1f} -> {b['after']['hover_max_mm']:.1f} mm")
        return {"results": results, "outputs": list(paths) + copies, "timings": timings}

    # ---------------------------------------------------------------------------- collision
    def stage_collision(self) -> dict:
        from .rocks.catalog import load_catalog
        from .rocks.collision_field import core_collision_grid, ring_collision_grid
        from .terrain import compose as tc

        cfg, lay, lib = self.cfg, self.lay, self.library
        key = self.key("collision")
        col = cfg.mesh.collision
        sp_core = float(cfg.zones.core.spacing_m)
        sp = float(col.spacing_m)
        step = int(round(sp / sp_core))
        if step < 1 or abs(step * sp_core - sp) > 1e-9:
            raise BuildError(f"mesh.collision.spacing_m {sp} must be a multiple of the core spacing {sp_core}")
        if float(cfg.zones.core.margin_m) < 0.15:
            raise BuildError("zones.core.margin_m must be >= 0.15 m for the collision low-pass")
        cat = load_catalog(self.catalog_path())
        ring_dmin = float(cfg.rocks.D_min_m.ring)
        sub = cat[cat["collide"] | ((cat["zone"] == 1) & (cat["D"] >= ring_dmin))]
        cdir = self.stage_dir("collision")
        core = lay.zone_rect("core")
        per = int(round(lay.tile_size / sp))
        n = lay.n_tiles * per + 1
        zc = np.empty((n, n), dtype=np.float32)
        outputs, built, raised, rocks_used = [], 0, 0, 0
        t0 = time.time()
        for t in lay.tiles():
            prefix = self.collision_prefix(t)
            meta = _grid_meta(prefix)
            if self._forced("collision") or not meta or meta.get("stage_key") != key:
                H = Grid.load(self.heights_prefix(t), mmap=True)
                lx, ly = H.lattice_origin()
                if lx % step or ly % step:
                    raise BuildError(f"{t.name}: tile grid origin is not on the {sp} m collision lattice")
                G = Grid(np.ascontiguousarray(H.z[::step, ::step]), H.x0, H.y0, H.spacing * step, {})
                C = core_collision_grid(cfg, G, sub, lib)
                r = lay.tile_rect(t)
                Ct = C.crop(r.x0, r.x1, r.y0, r.y1)
                if Ct.nx != per + 1 or Ct.ny != per + 1:
                    raise BuildError(f"{t.name}: collision crop has shape {Ct.z.shape}")
                Ct.meta.update(C.meta)
                Ct.meta.update({"kind": "core_collision_tile", "stage_key": key, "tile": [t.i, t.j],
                                "name": t.name, "source": "heights (H with fillets) subsampled to "
                                                          f"{sp} m with a {cfg.zones.core.margin_m} m margin"})
                _save_grid(Ct, prefix)
                built += 1
                meta = Ct.meta
                z = Ct.z
                del H, G, C
            else:
                z = Grid.load(prefix, mmap=True).z
            zc[t.j * per:(t.j + 1) * per + 1, t.i * per:(t.i + 1) * per + 1] = z
            raised += int(meta.get("n_nodes_raised", 0))
            rocks_used += int(meta.get("n_rocks", 0))
            outputs += [prefix.with_suffix(".npy"), prefix.with_suffix(".json")]
        timings = {"core_tiles": round(time.time() - t0, 2)}
        cgrid = Grid(zc, core.x0, core.y0, sp, {"kind": "core_collision", "stage_key": key,
                                                 "assembled_from": "collision/tile_ii_jj_collision",
                                                 "ground_offset_m": float(col.ground_offset_m),
                                                 "rock_offset_m": float(col.rock_offset_m),
                                                 "lowpass_wavelength_m": float(col.lowpass_wavelength_m)})
        _save_grid(cgrid, cdir / "core_collision")
        outputs += [(cdir / "core_collision").with_suffix(s) for s in (".npy", ".json")]
        t = time.time()
        ring_H = tc.load_zone_height(cfg, "ring", mmap=True, build=False)
        ringC = ring_collision_grid(cfg, ring_H, cat, lib)
        ringC.meta["stage_key"] = key
        _save_grid(ringC, cdir / "ring_collision")
        far_H = tc.load_zone_height(cfg, "far", mmap=True, build=False)
        farC = Grid((np.asarray(far_H.z, dtype=np.float64) + float(col.ground_offset_m)).astype(np.float32),
                    far_H.x0, far_H.y0, far_H.spacing, {"kind": "far_collision", "stage_key": key,
                                                        "ground_offset_m": float(col.ground_offset_m)})
        _save_grid(farC, cdir / "far_collision")
        timings["ring_far"] = round(time.time() - t, 2)
        for name in ("ring_collision", "far_collision"):
            outputs += [(cdir / name).with_suffix(s) for s in (".npy", ".json")]
        results = {"tiles_built": built, "core_spacing_m": sp, "core_nodes_raised": raised,
                   "core_rock_rows_rasterised_sum_over_tiles": rocks_used, "n_collide_rows": int(len(sub)),
                   "ring_rocks": ringC.meta.get("n_rocks"), "ring_nodes_raised": ringC.meta.get("n_nodes_raised"),
                   "ring_rock_dilation_nodes": ringC.meta.get("rock_dilation_nodes"),
                   "ring_rock_dilation_half_width_m": ringC.meta.get("rock_dilation_half_width_m"),
                   "core_z_range_m": [float(zc.min()), float(zc.max())], "catalog": str(self.catalog_path())}
        self.log(f"collision: {built} tile(s) built, {raised} core nodes raised by rocks")
        return {"results": results, "outputs": outputs, "timings": timings}

    # ---------------------------------------------------------------------------- meshes
    def stage_meshes(self) -> dict:
        from .mesh.refine import edge_lengths, triangle_box_heights
        from .mesh.stitch import check_stitched, make_piece, stitch_pieces, strip_priority, tile_priority
        from .rocks.catalog import load_catalog, split_by_tile
        from .rocks.merge import rocks_mesh

        cfg, lay, lib = self.cfg, self.lay, self.library
        mcfg = cfg.mesh
        B = self.budgets()
        n = lay.n_tiles
        mdir = self.stage_dir("meshes")
        for sub in ("visual", "collision", "rocks"):         # 'visual' moved to the ground stage
            d = mdir / sub
            if d.exists():
                for f in d.glob("*.npz"):
                    f.unlink()
        shutil.rmtree(mdir / "visual", ignore_errors=True)
        for sub in ("collision", "rocks"):
            (mdir / sub).mkdir(parents=True, exist_ok=True)
        timings: dict[str, float] = {}
        pieces_out: list[dict] = []
        outputs: list[Path] = []

        # ---- collision pieces
        t = time.time()
        col = []
        err_ccore = float(mcfg.collision.core_max_error_m)
        edge_core = _opt_float(mcfg.collision.get("core_max_edge_m"))
        edge_ring = _opt_float(mcfg.collision.get("ring_max_edge_m"))
        for tt in lay.tiles():
            C = Grid.load(self.collision_prefix(tt), mmap=True)
            col.append(make_piece(C, lay.tile_rect(tt), err_ccore, tile_priority(tt, n), name=f"{tt.name}_collision",
                                  normals=False, max_edge_m=edge_core))
        timings["core_collision_tin"] = round(time.time() - t, 2)
        t = time.time()
        cdir = self.lay.build_dir / "collision"
        for zone in ("ring", "far"):
            C = Grid.load(cdir / f"{zone}_collision", mmap=True)
            err = float(mcfg.collision[f"{zone}_max_error_m"])
            for side, r in lay.strip_rects(zone).items():
                col.append(make_piece(C, r, err, strip_priority(zone, side), name=f"{zone}_{side}_collision",
                                      normals=False, max_edge_m=edge_ring if zone == "ring" else None))
        timings["strips_collision"] = round(time.time() - t, 2)
        refine = [p.mesh.meta["refine"] for p in col if p.mesh.meta.get("refine")]
        t = time.time()
        rep_c = stitch_pieces(col, copy_normals=False)
        chk_c = check_stitched(col, compare_normals=False)
        timings["stitch_check_collision"] = round(time.time() - t, 2)
        if not chk_c["ok"]:
            raise BuildError(f"collision stitching check failed: {json.dumps(_plain(chk_c))[:2000]}")
        shape: dict[str, dict] = {}
        for zone in ("core", "ring", "far"):
            ps = [p for p in col if (p.name.startswith("tile_") if zone == "core" else p.name.startswith(zone + "_"))]
            if ps:
                shape[zone] = {"box_height_m": _pct(np.concatenate([triangle_box_heights(p.mesh) for p in ps])),
                               "edge_length_m": _pct(np.concatenate([edge_lengths(p.mesh) for p in ps]))}
        for p in col:
            rel = f"meshes/collision/{p.name}.npz"
            outputs.append(self.lay.build_dir / rel)
            _save_mesh(self.lay.build_dir / rel, p.mesh)
            zone = "core" if p.name.startswith("tile_") else p.name.split("_")[0]
            pieces_out.append(self._piece_record(p, "collision", zone, rel))
        del col
        self.log("meshes: collision stitched, check ok" + (
            f"; refined to max edge {edge_core} m (core) / {edge_ring} m (ring): "
            f"+{sum(r['triangles_after'] - r['triangles_before'] for r in refine):,} triangles, "
            f"{sum(r['edges_over_limit_after'] for r in refine)} edge(s) still over the limit" if refine else ""))

        # ---- rocks: one tile at a time, float32 positions, no lattice (CR-12)
        t = time.time()
        cat = load_catalog(self.catalog_path())
        for (zc, ti, tj), idx in sorted(split_by_tile(cat).items()):
            name = f"tile_{ti:02d}_{tj:02d}_rocks" if zc == 0 else f"ring_{RING_SIDES[ti]}_rocks"
            m = rocks_mesh(cat[idx], lib, name=name, dtype=np.float32)
            m.lattice = None
            if m.n_triangles == 0:
                continue
            rel = f"meshes/rocks/{m.name}.npz"
            outputs.append(self.lay.build_dir / rel)
            _save_mesh(self.lay.build_dir / rel, m, float32=True)
            zone = "core" if zc == 0 else "ring"
            r = lay.tile_rect(TileId(ti, tj)) if zone == "core" else lay.strip_rects("ring")[RING_SIDES[ti]]
            pieces_out.append({"name": m.name, "kind": "rocks", "zone": zone, "file": rel,
                               "rect": [r.x0, r.x1, r.y0, r.y1], "n_triangles": m.n_triangles,
                               "n_vertices": m.n_vertices, "n_rocks": int(m.meta.get("n_rocks", 0)),
                               "triangles_by_lod": m.meta.get("triangles_by_lod")})
            del m
        timings["rocks_merge_save"] = round(time.time() - t, 2)

        # ---- counts and budgets (visual ground from the ground stage)
        gman = _read_json(self.manifest_path("ground"))["results"]
        counts = self._triangle_counts(gman["pieces"] + pieces_out)
        budget = self._check_budgets(counts, B, gman.get("capped_tiles", []))
        results = {"pieces": pieces_out, "counts": counts, "budgets": budget,
                   "stitch_collision": rep_c, "check_collision": chk_c, "collision_shape": shape,
                   "collision_refinement": None if not refine else {
                       "core_max_edge_m": edge_core, "ring_max_edge_m": edge_ring, "pieces": len(refine),
                       "triangles_added": int(sum(r["triangles_after"] - r["triangles_before"] for r in refine)),
                       "split_edges": int(sum(r["split_edges"] for r in refine)),
                       "flips": int(sum(r["flips"] for r in refine)),
                       "edges_over_limit_after": int(sum(r["edges_over_limit_after"] for r in refine)),
                       "max_edge_after_m": float(max(r["max_edge_after_m"] for r in refine)),
                       "time_s": round(sum(r["time_s"] for r in refine), 2)},
                   "errors_m": dict(gman.get("errors_m", {}), core_collision=err_ccore,
                                    ring_collision=float(mcfg.collision.ring_max_error_m),
                                    far_collision=float(mcfg.collision.far_max_error_m))}
        self.log("meshes: triangles " + ", ".join(f"{k} {v:,}" for k, v in counts.items() if isinstance(v, int)))
        violations = [k for k, v in budget.items() if v.get("ok") is False and v.get("hard", True)]
        for k, v in budget.items():
            if v.get("ok") is False:
                self.log(f"meshes: budget {'VIOLATION' if v.get('hard', True) else 'warning'} {k}: {v}")
        if violations:
            raise BuildError(f"budget exceeded: {violations} ({ {k: budget[k] for k in violations} })")
        return {"results": results, "outputs": outputs, "timings": timings}

    @staticmethod
    def _triangle_counts(pieces: list[dict]) -> dict:
        c: dict[str, Any] = {}
        for p in pieces:
            k = f"{p['zone']}_{p['kind']}"
            c[k] = c.get(k, 0) + int(p["n_triangles"])
        for k in ("core_ground", "core_rocks", "core_collision", "ring_ground", "ring_rocks", "ring_collision",
                  "far_ground", "far_collision", "horizon_ground"):
            c.setdefault(k, 0)
        c["visual_total"] = sum(v for k, v in c.items() if k.endswith(("_ground", "_rocks")))
        c["collision_total"] = sum(v for k, v in c.items() if k.endswith("_collision"))
        c["outer_visual"] = c["ring_ground"] + c["ring_rocks"] + c["far_ground"] + c["horizon_ground"]
        per = {}
        for kind in ("ground", "rocks", "collision"):
            vals = [p["n_triangles"] for p in pieces if p["zone"] == "core" and p["kind"] == kind]
            per[kind] = _stats(vals)
        c["core_per_tile"] = per
        c["core_tile_ground_plus_rocks_max"] = 0
        by_tile: dict[str, int] = {}
        for p in pieces:
            if p["zone"] == "core" and p["kind"] in ("ground", "rocks"):
                tname = p["name"].rsplit("_", 1)[0]
                by_tile[tname] = by_tile.get(tname, 0) + int(p["n_triangles"])
        if by_tile:
            c["core_tile_ground_plus_rocks_max"] = max(by_tile.values())
        return c

    @staticmethod
    def _check_budgets(c: dict, B: dict, capped: list) -> dict:
        """DESIGN 13a budgets: ``*_max_*`` keys are hard limits, ``*_warn_*`` keys and the core
        collision target range only warn (``hard: False``)."""
        B = dict(DEFAULT_BUDGETS, **{k: v for k, v in B.items() if v is not None})
        lo, hi = (int(v) for v in B["core_collision_target_tris"])
        per_tile = ((c.get("core_per_tile") or {}).get("ground") or {}).get("max", 0)
        hard = {
            "core_tile_visual_max_tris": ("core_per_tile_ground_max", int(per_tile)),
            "core_visual_max_tris": ("core_ground", int(c.get("core_ground", 0))),
            "core_rocks_max_tris": ("core_rocks", int(c.get("core_rocks", 0))),
            "visual_total_max_tris": ("visual_total", int(c.get("visual_total", 0))),
            "core_collision_max_tris": ("core_collision", int(c.get("core_collision", 0))),
            "collision_total_max_tris": ("collision_total", int(c.get("collision_total", c.get("core_collision", 0)))),
        }
        out: dict[str, dict] = {}
        for k, (what, value) in hard.items():
            out[k] = {"limit": int(B[k]), "value": value, "of": what, "ok": value <= int(B[k]), "hard": True}
            warn = k.replace("_max_", "_warn_")
            if warn in B and B[warn] is not None:
                out[warn] = {"limit": int(B[warn]), "value": value, "of": what, "ok": value <= int(B[warn]),
                             "hard": False}
        out["core_tile_visual_max_tris"]["capped_tiles"] = len(capped)
        cc = int(c.get("core_collision", 0))
        out["core_collision_target_tris"] = {"range": [lo, hi], "value": cc, "ok": lo <= cc <= hi, "hard": False}
        return out

    # ---------------------------------------------------------------------------- export
    def stage_export(self) -> dict:
        from .materials.regolith import make_regolith_textures
        from .mesh.gltf import write_glb
        from .mesh.types import MaterialSpec

        cfg, lay, lib = self.cfg, self.lay, self.library
        pieces = (_read_json(self.manifest_path("ground"))["results"]["pieces"]
                  + _read_json(self.manifest_path("meshes"))["results"]["pieces"])
        meshes_dir = lay.meshes_dir
        reg = cfg.materials.regolith
        base = tuple(float(v) for v in reg.base_color_linear)
        mats = {"ground": MaterialSpec("regolith", (*base, 1.0), 0.0, float(reg.roughness)),
                "rocks": MaterialSpec("rocks", ROCK_GLB_BASE_COLOR, 0.0, float(cfg.materials.rocks.roughness)),
                "collision": MaterialSpec("collision", COLLISION_GLB_COLOR, 0.0, 1.0)}
        timings: dict[str, float] = {}
        files, outputs = [], []
        t = time.time()
        for p in pieces:
            mesh = _load_mesh(self.lay.build_dir / p["file"], p["name"])
            fname = f"{p['name']}.glb"
            # extras carry only content facts (no config hash, which depends on paths too), so a
            # rebuild into another folder gives byte-identical GLBs
            info = write_glb(meshes_dir / fname, mesh, mats[p["kind"]],
                             extras={"piece": p["name"], "kind": p["kind"], "zone": p["zone"],
                                     "generator": f"lunar_world_gen {__version__}"})
            files.append({"file": fname, "name": p["name"], "kind": p["kind"], "zone": p["zone"],
                          "bytes": info["bytes"], "n_triangles": info["n_triangles"],
                          "n_vertices": info["n_vertices"]})
            outputs.append(meshes_dir / fname)
            del mesh
        timings["glb"] = round(time.time() - t, 2)
        keep = {f["file"] for f in files}
        stale = [f for f in meshes_dir.iterdir() if f.is_file() and f.name not in keep
                 and (f.suffix == ".glb" or ".glb.tmp" in f.name or "_tmp" in f.name)]
        for f in stale:
            f.unlink()
        t = time.time()
        tmpdir = lay.textures_dir / f".tmp_textures_{os.getpid()}_{uuid.uuid4().hex[:8]}"
        try:
            made = make_regolith_textures(cfg, tmpdir)
            tex = {}
            for k, src in made.items():
                dst = lay.textures_dir / Path(src).name
                os.replace(src, dst)
                tex[k] = str(dst)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)
        timings["regolith_textures"] = round(time.time() - t, 2)
        outputs += [Path(tex[k]) for k in ("albedo", "normal", "roughness", "meta")]
        atlas = lib.atlas_paths
        copied = {}
        for key in ("basecolor", "normal", "roughness"):
            src = atlas.get(key)
            if src is None or not Path(src).is_file():
                raise BuildError(f"rock atlas map '{key}' not found in {lib.path} (atlas paths {atlas})")
            dst = _copy_atomic(Path(src), lay.textures_dir / f"rock_atlas_{key}.png")
            copied[key] = {"from": str(src), "to": str(dst), "bytes": dst.stat().st_size}
            outputs.append(dst)
        results = {"files": files, "n_glb": len(files), "glb_bytes": sum(f["bytes"] for f in files),
                   "stale_removed": [f.name for f in stale], "textures": tex, "rock_atlas": copied,
                   "bytes_by_kind": {k: sum(f["bytes"] for f in files if f["kind"] == k)
                                     for k in ("ground", "rocks", "collision")}}
        self.log(f"export: {len(files)} GLBs ({results['glb_bytes'] / 1e6:.0f} MB), textures written, "
                 f"{len(stale)} stale file(s) removed")
        return {"results": results, "outputs": outputs, "timings": timings}

    # ---------------------------------------------------------------------------- world
    def _specs(self, files: list[dict]):
        from .world.sdf import CollisionSpec, VisualSpec

        cfg = self.cfg
        # one friction value: rock collision proxies are baked into the ground collision meshes, so
        # physics.ground_mu applies to them too (CR-09; physics.rock_mu is not used)
        mu = float(cfg.physics.ground_mu)
        zone_rank = {"core": 0, "ring": 1, "far": 2, "horizon": 3}
        kind_rank = {"ground": 0, "rocks": 1, "collision": 2}
        order = sorted(files, key=lambda f: (kind_rank[f["kind"]] == 2, zone_rank[f["zone"]], kind_rank[f["kind"]],
                                             f["name"]))
        visuals, collisions = [], []
        for f in order:
            uri = f"meshes/{f['file']}"
            if f["kind"] == "collision":
                collisions.append(CollisionSpec(f["name"], uri, mu))
            else:
                material = "rocks" if f["kind"] == "rocks" else f"regolith_{f['zone']}"
                visuals.append(VisualSpec(f["name"], uri, material, cast_shadows=f["zone"] in SHADOW_ZONES))
        return visuals, collisions

    def stage_world(self) -> dict:
        from .rocks.catalog import load_catalog
        from .world.sdf import validate_sdf, write_all_variants
        from .world.spawn import find_sites
        from .world.world_info import write_world_info

        cfg, lay = self.cfg, self.lay
        timings: dict[str, float] = {}
        exp = _read_json(self.manifest_path("export"))["results"]
        meshes_man = _read_json(self.manifest_path("meshes"))["results"]
        ground_man = _read_json(self.manifest_path("ground"))["results"]
        contact_man = (_read_json(self.manifest_path("contact")) or {}).get("results", {})
        visuals, collisions = self._specs(exp["files"])
        t = time.time()
        paths = write_all_variants(cfg, lay, visuals, collisions)
        timings["sdf_write"] = round(time.time() - t, 2)
        t = time.time()
        sdf_check: dict[str, Any] = {}
        if shutil.which("gz") is None:
            sdf_check["skipped"] = "the Gazebo CLI 'gz' is not on PATH"
            self.log("world: WARNING gz CLI not on PATH, SDF validation (gz sdf -k) skipped; the world stage "
                     "re-runs once gz is available")
        for variant, p in paths.items():
            if "skipped" in sdf_check:
                break
            ok, out = validate_sdf(p)
            sdf_check[p.name] = {"ok": ok, "output": out[-500:]}
            if not ok:
                raise BuildError(f"gz sdf -k {p} failed:\n{out}")
        timings["gz_sdf_check"] = round(time.time() - t, 2)
        t = time.time()
        C = Grid.load(self.lay.build_dir / "collision" / "core_collision", mmap=False)
        world_cat = lay.world_dir / "rocks_catalog.npz"
        cat = load_catalog(world_cat)
        heights = TileHeights(cfg, self.heights_prefix)
        res = find_sites(cfg, C, cat, height_fn=heights)
        timings["sites"] = round(time.time() - t, 2)
        del C
        counts = dict(meshes_man["counts"])
        rock_counts = self._rock_counts(cat)
        build_info = {"build": {"generator": f"lunar_world_gen.build {__version__}", "stage_keys":
                                {s: self.key(s) for s in STAGES if s != "validate"},
                                "budgets": meshes_man.get("budgets"), "errors_m": meshes_man.get("errors_m"),
                                "n_visuals": len(visuals), "n_collisions": len(collisions),
                                "glb_bytes": exp.get("glb_bytes"), "sdf_validated": "skipped" not in sdf_check,
                                "collision": {k: float(v) for k, v in cfg.mesh.collision.items()
                                              if isinstance(v, (int, float))},
                                "collision_refinement": meshes_man.get("collision_refinement"),
                                "collision_shape": meshes_man.get("collision_shape"),
                                "visual_seam_refinement": ground_man.get("seam_refinement"),
                                "rock_contact": {k: contact_man.get(k) for k in
                                                 ("enabled", "n_reseated", "n_filleted_kept")}}}
        info_path = write_world_info(cfg, lay, res, triangle_counts=counts, rock_counts=rock_counts,
                                     extra=build_info)
        sites = {k: s.to_dict() for k, s in res.sites.items()}
        for name, s in res.sites.items():
            self.log(f"world: site {name}: ({s.x:.2f}, {s.y:.2f}, z {s.z:.3f}, yaw {math.degrees(s.yaw):.1f} deg)"
                     + (f" relaxed: {s.relaxed}" if s.relaxed else ""))
        for name, why in res.missing.items():
            self.log(f"world: site {name} MISSING: {why}")
        results = {"sdf": {k or "default": str(p) for k, p in paths.items()}, "gz_sdf_check": sdf_check,
                   "world_info": str(info_path), "sites": sites, "missing_sites": res.missing,
                   "site_search": res.params, "n_visuals": len(visuals), "n_collisions": len(collisions),
                   "rock_counts": rock_counts}
        outputs = list(paths.values()) + [info_path]
        return {"results": results, "outputs": outputs, "timings": timings}

    def _rock_counts(self, cat: np.ndarray) -> dict:
        core = cat[cat["zone"] == 0]
        ring = cat[cat["zone"] == 1]
        return {"core": int(len(core)), "ring": int(len(ring)), "core_collide": int(core["collide"].sum()),
                "core_ge_D": {f"{d:g}": int((core["D"] >= d).sum()) for d in (0.02, 0.05, 0.1, 0.2, 0.5, 1.0)},
                "ring_ge_D": {f"{d:g}": int((ring["D"] >= d).sum()) for d in (0.3, 0.5, 1.0, 2.0)},
                "core_D_max_m": float(core["D"].max()) if len(core) else None}

    # ---------------------------------------------------------------------------- validate
    def stage_validate(self) -> dict:
        from .terrain import base as tbase
        from .terrain import compose as tc
        from .terrain import craters as tcr
        from .validate import report as vreport
        from .validate import static_checks as sc

        cfg, lay = self.cfg, self.lay
        # explicit inputs: the H0 tiles (no rocks, no fillets) for the roughness acceptance, the core
        # DTM base B (site-aware rule), the terrain crater sets, the assembled 2 cm collision grid
        inputs = {"core_grid": None,
                  "core_tile_grids": {t.name: str(tc.tile_prefix(cfg, t)) for t in lay.tiles()},
                  "core_base_grid": str(tbase.base_prefix(cfg, "core")),
                  "crater_catalogs": [str(tcr.craters_path(cfg, w)) for w in tcr.WHICH],
                  "collision_grid": str(lay.build_dir / "collision" / "core_collision"),
                  "rock_catalog": str(lay.world_dir / "rocks_catalog.npz")}
        t = time.time()
        rep = sc.run_static_checks(cfg, inputs=inputs)
        t_static = round(time.time() - t, 2)
        t = time.time()
        sdf = lay.sdf_path("")
        archived = archive_stale_results(lay.reports_dir, sdf.stat().st_mtime) if sdf.is_file() else {}
        if archived:
            self.log(f"validate: {len(archived['moved'])} Gazebo result(s) older than the world files moved to "
                     f"{archived['to']} (they describe an earlier build)")
        md = vreport.build_report(lay.reports_dir)
        t_report = round(time.time() - t, 2)
        checks = {k: {"status": v.get("status"), "seconds": v.get("seconds")} for k, v in rep["checks"].items()}
        results = {"summary": rep["summary"], "checks": checks, "static_checks": rep.get("written_to"),
                   "report": str(md), "archived_stale_results": archived}
        self.log("validate: " + ", ".join(f"{k} {v['status']}" for k, v in checks.items()))
        return {"results": results, "outputs": [Path(rep["written_to"]), md],
                "timings": {"static_checks": t_static, "report": t_report}}


#: result kinds of the Gazebo tools (bench_world.py, validate.lidar_checks, run_robot_tests.py) that the
#: validation report reads from world_dir/reports
GZ_RESULT_KINDS = ("bench_world", "lidar_checks", "robot_tests")


def archive_stale_results(reports: Path, world_mtime: float) -> dict:
    """Move Gazebo results made before the current world files into ``reports/stale_before_<time>/``.

    The report collects every result JSON in ``reports``; after a rebuild the bench / LiDAR / robot
    results (and their LiDAR captures and camera images) describe the previous world (other meshes,
    sites and triangle counts), so mixing them into the new report gave a wrong verdict (final
    integration: the rebuilt 17 M-triangle world was reported with the phase-3 world's 19.6 s load time
    and its robot failures). Results are kept, only moved; anything newer than ``world_mtime`` (the
    default SDF written by the world stage) stays. Returns {"to": folder, "moved": [relative paths]}
    or {} when nothing is stale."""
    from .validate.report import IMAGE_EXT

    reports = Path(reports)
    if not reports.is_dir():
        return {}
    stale: list[Path] = []
    for f in sorted(reports.glob("*.json")):
        d = _read_json(f)
        if isinstance(d, dict) and d.get("kind") in GZ_RESULT_KINDS and f.stat().st_mtime < world_mtime:
            stale.append(f)
    lidar = reports / "lidar"
    if lidar.is_dir():
        newest = max((p.stat().st_mtime for p in lidar.rglob("*")), default=lidar.stat().st_mtime)
        if newest < world_mtime:
            stale.append(lidar)
    for d in (reports, reports / "images", reports / "camera"):
        if d.is_dir():
            stale += [p for p in sorted(d.iterdir()) if p.is_file() and p.suffix.lower() in IMAGE_EXT
                      and not p.name.startswith("fig_") and p.stat().st_mtime < world_mtime]
    if not stale:
        return {}
    dest = reports / f"stale_before_{time.strftime('%Y%m%d_%H%M%S', time.localtime(world_mtime))}"
    moved = []
    for p in stale:
        rel = p.relative_to(reports)
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(p), str(target))
        moved.append(rel.as_posix())
    return {"to": str(dest), "moved": moved}


# =============================================================================================== CLI
def disk_usage(cfg: Config) -> dict:
    lay = Layout(cfg)
    out: dict[str, Any] = {}
    for root in (lay.build_dir, lay.world_dir):
        d: dict[str, int] = {}
        if root.is_dir():
            for sub in sorted(root.iterdir()):
                d[sub.name] = _du(sub)
        out[str(root)] = {"total": sum(d.values()), "entries": d}
    return out


def apply_preset_dirs(cfg: Config, config_path: str | Path | None, overrides: Iterable[str]) -> Config:
    """Give a non-default preset its own build/world dirs (CR-02): ``<dir>_<preset>`` unless the
    command line sets ``paths.build_dir`` / ``paths.world_dir`` itself. Modifies ``cfg`` in place."""
    import yaml

    from .config import DEFAULT_CONFIG

    raw = yaml.safe_load(Path(config_path or DEFAULT_CONFIG).read_text()) or {}
    default_preset = (raw.get("world") or {}).get("preset")
    preset = cfg.world.get("preset")
    if not preset or preset == default_preset:
        return cfg
    given = {o.partition("=")[0].strip() for o in overrides or []}
    paths = dict.__getitem__(cfg, "paths")
    changed = {}
    for key in ("build_dir", "world_dir"):
        if f"paths.{key}" in given:
            continue
        paths[key] = f"{str(paths[key]).rstrip('/')}_{preset}"
        changed[key] = paths[key]
    if changed:
        src = dict.get(cfg, "_source")
        if isinstance(src, dict):
            src["preset_dirs"] = changed
    return cfg


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m lunar_world_gen.build",
                                 description="Build the Apollo 16 lunar world (all stages, cached).")
    ap.add_argument("--config", default=None, help="config YAML (default: config/apollo16_core.yaml)")
    ap.add_argument("--preset", default=None, help="config preset: full | fast (non-default presets build into "
                                                   "<dir>_<preset> unless paths.* are set)")
    ap.add_argument("--stages", default=None, help=f"comma list of stages (default: all): {','.join(STAGES)}")
    ap.add_argument("--force", action="store_true",
                    help="re-run the selected stages even if cached (terrain only when named in --stages)")
    ap.add_argument("--force-downstream", action="store_true",
                    help="also add and re-run every stage downstream of the selected ones")
    ap.add_argument("--adopt", default=None,
                    help="comma list of stages whose cached outputs are accepted as current without re-running")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="config override")
    ap.add_argument("--status", action="store_true", help="only report which stages are up to date")
    ap.add_argument("--lock-timeout", type=float, default=0.0,
                    help="seconds to wait for another build on the same dirs (default: fail at once)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config, preset=args.preset, overrides=args.set)
    apply_preset_dirs(cfg, args.config, args.set)
    stages = [s.strip() for s in args.stages.split(",") if s.strip()] if args.stages else list(STAGES)
    adopt = [s.strip() for s in args.adopt.split(",") if s.strip()] if args.adopt else []
    b = Builder(cfg, force=args.force, explicit_stages=stages if args.stages else None,
                force_downstream=args.force_downstream, adopt=adopt, lock_timeout_s=args.lock_timeout)
    if args.status:
        lay = Layout(cfg)
        print(f"build_dir {lay.build_dir}\nworld_dir {lay.world_dir}")
        for s, (ok, why) in b.status().items():
            print(f"{s:10s} {'up to date' if ok else 'STALE':10s} {why}")
        return 0
    try:
        records = b.run(stages)
    except BuildError as exc:
        b.log(f"FAILED: {exc}")
        return 1
    except Exception as exc:  # noqa: BLE001 - report the failing stage, then exit non-zero
        b.log(f"FAILED: {type(exc).__name__}: {exc}")
        traceback.print_exc()
        return 1
    du = disk_usage(cfg)
    b.log("summary: " + json.dumps({s: {k: r.get(k) for k in ("status", "elapsed_s", "peak_rss_mb")}
                                     for s, r in records.items()}))
    b.log("disk: " + json.dumps({k: v["total"] for k, v in du.items()}))
    val = _read_json(b.manifest_path("validate")) if "validate" in records else None
    if val and val.get("status") == "ok":
        summ = val["results"]["summary"]
        b.log(f"validation: pass={summ.get('pass')} all_ran={summ.get('all_ran')} failed={summ.get('failed')} "
              f"skipped={summ.get('skipped')} errors={summ.get('errors')}")
        if not summ.get("pass") or not summ.get("all_ran"):
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
