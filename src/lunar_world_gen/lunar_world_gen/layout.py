"""Where every artifact of a build lives, and how tiles are named.

Two roots (both from ``cfg.paths``):

* ``build_dir`` - intermediate products, one folder per stage (height grids, catalogs,
  per-stage manifests). Safe to delete; stages re-run when inputs change.
* ``world_dir`` - the finished, self-contained world that Gazebo loads: SDF files,
  meshes, textures, world_info.yaml, catalogs and reports. Mesh and texture URIs inside
  the SDF are relative to the SDF file, so the folder can be moved as a whole.

Core tiles are indexed (i, j) with i west->east and j south->north; tile (i, j) covers
x in [core_x0 + i*T, core_x0 + (i+1)*T] and y in [core_y0 + j*T, core_y0 + (j+1)*T],
where T is the tile size and (core_x0, core_y0) the south-west corner of the core.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .config import Config

ZONES = ("core", "ring", "far", "horizon")
RING_SIDES = ("south", "north", "west", "east")  # outer zones are four strips around the inner square


@dataclass(frozen=True)
class TileId:
    i: int
    j: int

    @property
    def name(self) -> str:
        return f"tile_{self.i:02d}_{self.j:02d}"


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle in world metres (node-inclusive)."""

    x0: float
    x1: float
    y0: float
    y1: float

    def contains(self, x, y, eps: float = 1e-9):
        return (x >= self.x0 - eps) & (x <= self.x1 + eps) & (y >= self.y0 - eps) & (y <= self.y1 + eps)

    @property
    def width(self) -> float:
        return self.x1 - self.x0

    @property
    def height(self) -> float:
        return self.y1 - self.y0


class Layout:
    """Paths and geometry derived from a config."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.world_name: str = cfg.world.name
        self.build_dir = Path(cfg.paths.build_dir)
        self.world_dir = Path(cfg.paths.world_dir)

    # ------------------------------------------------------------------ directories
    def stage_dir(self, stage: str) -> Path:
        d = self.build_dir / stage
        d.mkdir(parents=True, exist_ok=True)
        return d

    @property
    def meshes_dir(self) -> Path:
        d = self.world_dir / "meshes"
        d.mkdir(parents=True, exist_ok=True)
        return d

    @property
    def textures_dir(self) -> Path:
        d = self.world_dir / "textures"
        d.mkdir(parents=True, exist_ok=True)
        return d

    @property
    def reports_dir(self) -> Path:
        d = self.world_dir / "reports"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def sdf_path(self, variant: str = "") -> Path:
        suffix = f"_{variant}" if variant else ""
        return self.world_dir / f"{self.world_name}{suffix}.sdf"

    @property
    def world_info_path(self) -> Path:
        return self.world_dir / "world_info.yaml"

    # ------------------------------------------------------------------ zone geometry
    def zone_rect(self, zone: str) -> Rect:
        size = self.cfg.zones[zone].size_m
        half = size / 2.0
        return Rect(-half, half, -half, half)

    @property
    def tile_size(self) -> float:
        return float(self.cfg.zones.core.tile_size_m)

    @property
    def n_tiles(self) -> int:
        n = self.zone_rect("core").width / self.tile_size
        if abs(n - round(n)) > 1e-9:
            raise ValueError("core size must be a multiple of the tile size")
        return int(round(n))

    def tiles(self) -> list[TileId]:
        return [TileId(i, j) for j in range(self.n_tiles) for i in range(self.n_tiles)]

    def tile_rect(self, t: TileId) -> Rect:
        core = self.zone_rect("core")
        T = self.tile_size
        return Rect(core.x0 + t.i * T, core.x0 + (t.i + 1) * T, core.y0 + t.j * T, core.y0 + (t.j + 1) * T)

    def strip_rects(self, zone: str) -> dict[str, Rect]:
        """The four strips of an outer zone around the next inner zone's square.

        south/north strips span the full outer width; west/east strips fill the remaining
        band between them. Adjacent strips share their boundary lines exactly.
        """
        inner = {"ring": "core", "far": "ring", "horizon": "far"}[zone]
        o = self.zone_rect(zone)
        i = self.zone_rect(inner)
        return {
            "south": Rect(o.x0, o.x1, o.y0, i.y0),
            "north": Rect(o.x0, o.x1, i.y1, o.y1),
            "west": Rect(o.x0, i.x0, i.y0, i.y1),
            "east": Rect(i.x1, o.x1, i.y0, i.y1),
        }

    # ------------------------------------------------------------------ file names
    def tile_mesh_name(self, t: TileId, kind: str) -> str:
        """kind: 'ground' | 'rocks' (visual) or 'collision'."""
        return f"{t.name}_{kind}.glb"

    def strip_mesh_name(self, zone: str, side: str, kind: str) -> str:
        return f"{zone}_{side}_{kind}.glb"
