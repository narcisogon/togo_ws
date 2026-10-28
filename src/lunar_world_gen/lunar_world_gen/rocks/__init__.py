"""rocks package of the lunar world generator (docs/DESIGN.md section 5 and the rock part of 7).

Modules:
    library          RockLibrary (reads the Blender-made prototype library), write_synthetic_library
    population       sample_rocks (SFD, density field, crater annuli, separation) -> catalog
    placement        class / prototype / orientation / burial / contact height; ground wrappers
    catalog          CATALOG_DTYPE, save/load (npz + csv), validation, N(>=D) with Poisson intervals
    fillets          apply_fillets (regolith fillets added to the core height field)
    collision_field  core_collision_grid / ring_collision_grid (rasterised hull tops)
    merge            rocks_mesh (one Mesh per tile from catalog rows)
"""
from .catalog import (CATALOG_DTYPE, check_catalog, cumulative_counts, load_catalog, load_catalog_meta,
                      save_catalog, split_by_tile, validate_catalog)
from .collision_field import core_collision_grid, ring_collision_grid
from .fillets import apply_fillets
from .library import RockLibrary, write_synthetic_library
from .merge import rocks_mesh, rocks_mesh_by_tile
from .placement import GroundFunction, GroundGrid, flat_ground
from .population import BrokenPowerLaw, density_field, sample_rocks

__all__ = [
    "CATALOG_DTYPE", "check_catalog", "cumulative_counts", "load_catalog", "load_catalog_meta", "save_catalog",
    "split_by_tile", "validate_catalog", "core_collision_grid", "ring_collision_grid", "apply_fillets",
    "RockLibrary", "write_synthetic_library", "rocks_mesh", "rocks_mesh_by_tile", "GroundFunction",
    "GroundGrid", "flat_ground", "BrokenPowerLaw", "density_field", "sample_rocks",
]
