"""materials package of the lunar world generator (see docs/DESIGN.md section 9).

* ``regolith.make_regolith_textures(cfg, out_dir)`` - tileable albedo / normal / roughness PNGs.
"""
from .regolith import make_regolith_textures, synthesize_regolith  # noqa: F401
