"""Rock catalog: one row per rock (docs/DESIGN.md section 5).

The catalog is a numpy structured array with the DESIGN fields::

    id, zone (0 core, 1 ring), tile_i, tile_j, proto, cls, lod, x, y, z,
    qw, qx, qy, qz, scale, D, exposed_h, burial, collide

* ``proto`` indexes ``RockLibrary.prototypes``; ``cls`` indexes ``cfg.rocks.classes`` in config
  order (0 pebble, 1 small, 2 medium, 3 large with the default config); ``lod`` is the LOD the rock
  is merged with.
* ``tile_i, tile_j``: core tile of the rock centre. Ring rocks carry ``tile_i`` = index of their
  ring strip in ``layout.RING_SIDES`` (0 south, 1 north, 2 west, 3 east) and ``tile_j = -1``.
* ``(x, y, z)`` is the rock origin (prototype volume centroid) in the world frame, ``(qw..qz)`` the
  rotation and ``scale`` the uniform scale: ``p_world = (x, y, z) + scale * R(q) @ p_proto``.
* ``D`` = longest axis [m] (= scale * a_proto), ``exposed_h`` = top above the ground at the centre,
  ``burial`` = buried fraction of the rotated rock height below its first contact (perched rocks
  carry their small compliance / bedding fraction, 0.02-0.18, since FID-01), ``collide`` = core rock
  with ``D >= rocks.collision_min_D_m`` (raised into the core collision field; ring rocks >= 30 cm
  are raised into the ring collision field by size, DESIGN 7).
* Ring rows (zone 1) are the ring population (D >= ``rocks.D_min_m.ring``, ids from 10,000,000) and
  the core-edge band (FID-04: D < ``rocks.D_min_m.ring`` within ``rocks.edge_band.width_m`` of the
  core edge, ids from 12,000,000, ``collide`` false). The format (fields, dtypes, files) is unchanged.

Files: ``rocks_catalog.npz`` (one array per field + JSON metadata ``meta_json``) and
``rocks_catalog.csv`` (same columns, header line).
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

CATALOG_DTYPE = np.dtype([
    ("id", "<i8"), ("zone", "<i1"), ("tile_i", "<i2"), ("tile_j", "<i2"),
    ("proto", "<i4"), ("cls", "<i1"), ("lod", "<i1"),
    ("x", "<f8"), ("y", "<f8"), ("z", "<f8"),
    ("qw", "<f8"), ("qx", "<f8"), ("qy", "<f8"), ("qz", "<f8"),
    ("scale", "<f8"), ("D", "<f8"), ("exposed_h", "<f8"), ("burial", "<f8"), ("collide", "?"),
])
FIELDS = CATALOG_DTYPE.names
ZONE_CODES = {"core": 0, "ring": 1}
SFD_CHECK_D_M = (0.02, 0.05, 0.10, 0.20)


def empty_catalog(n: int = 0) -> np.ndarray:
    return np.zeros(n, dtype=CATALOG_DTYPE)


def concatenate(catalogs: list[np.ndarray], renumber: bool = False) -> np.ndarray:
    """Concatenate catalogs (e.g. core + ring); optionally renumber ids 0..n-1."""
    cat = np.concatenate([np.asarray(c, dtype=CATALOG_DTYPE) for c in catalogs]) if catalogs else empty_catalog()
    if renumber:
        cat["id"] = np.arange(len(cat))
    return cat


# ---------------------------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------------------------
def _npz_path(path: str | Path) -> Path:
    p = Path(path)
    return p if p.suffix == ".npz" else p.with_suffix(".npz")


def save_catalog(catalog: np.ndarray, path: str | Path, meta: dict | None = None, csv: bool = True) -> list[Path]:
    """Write PATH.npz (+ PATH.csv). Returns the written paths.

    The npz holds one array per field (``d["x"]``, ``d["D"]``, ...) plus ``meta_json`` (a JSON string),
    so it can be read without this module (validate/static_checks.py reads it column-wise).
    """
    catalog = np.asarray(catalog, dtype=CATALOG_DTYPE)
    npz = _npz_path(path)
    npz.parent.mkdir(parents=True, exist_ok=True)
    meta_json = json.dumps(meta or {}, default=_json_default)
    np.savez_compressed(npz, **{name: np.ascontiguousarray(catalog[name]) for name in FIELDS},
                        meta_json=np.array(meta_json))
    out = [npz]
    if csv:
        out.append(write_csv(catalog, npz.with_suffix(".csv")))
    return out


def write_csv(catalog: np.ndarray, path: str | Path) -> Path:
    path = Path(path)
    fmt = []
    for name in FIELDS:
        kind = CATALOG_DTYPE[name].kind
        if kind in "iu":
            fmt.append("%d")
        elif kind == "b":
            fmt.append("%d")
        elif name in ("x", "y", "z"):
            fmt.append("%.6f")
        else:
            fmt.append("%.7g")
    cols = [catalog[name].astype(np.int64) if CATALOG_DTYPE[name].kind == "b" else catalog[name] for name in FIELDS]
    table = np.empty(len(catalog), dtype=[(n, c.dtype) for n, c in zip(FIELDS, cols)])
    for n, c in zip(FIELDS, cols):
        table[n] = c
    np.savetxt(path, table, fmt=fmt, delimiter=",", header=",".join(FIELDS), comments="")
    return path


def load_catalog(path: str | Path) -> np.ndarray:
    """Load a catalog from .npz (preferred) or .csv."""
    p = Path(path)
    if p.suffix == ".csv":
        return read_csv(p)
    with np.load(_npz_path(p), allow_pickle=False) as d:
        if "catalog" in d.files:                        # single structured array (older layout)
            return np.asarray(d["catalog"], dtype=CATALOG_DTYPE)
        n = len(d["x"])
        cat = empty_catalog(n)
        for name in FIELDS:
            cat[name] = d[name]
        return cat


def load_catalog_meta(path: str | Path) -> dict:
    with np.load(_npz_path(path), allow_pickle=False) as d:
        return json.loads(str(d["meta_json"])) if "meta_json" in d else {}


def read_csv(path: str | Path) -> np.ndarray:
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
    raw = np.atleast_1d(raw)
    cat = empty_catalog(len(raw))
    for name in FIELDS:
        cat[name] = raw[name].astype(bool) if CATALOG_DTYPE[name].kind == "b" else raw[name]
    return cat


# ---------------------------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------------------------
def validate_catalog(catalog: np.ndarray, library=None, cfg=None) -> list[str]:
    """Return a list of problems (empty if the catalog is consistent)."""
    problems: list[str] = []
    cat = np.asarray(catalog)
    if cat.dtype != CATALOG_DTYPE:
        missing = [f for f in FIELDS if f not in (cat.dtype.names or ())]
        if missing:
            return [f"missing fields: {missing}"]
    if len(cat) == 0:
        return problems
    if len(np.unique(cat["id"])) != len(cat):
        problems.append("duplicate ids")
    for f in ("x", "y", "z", "qw", "qx", "qy", "qz", "scale", "D", "exposed_h", "burial"):
        if not np.all(np.isfinite(cat[f])):
            problems.append(f"non-finite values in {f}")
    qn = np.sqrt(cat["qw"] ** 2 + cat["qx"] ** 2 + cat["qy"] ** 2 + cat["qz"] ** 2)
    if np.any(np.abs(qn - 1) > 1e-6):
        problems.append(f"{int(np.sum(np.abs(qn - 1) > 1e-6))} non-unit quaternions")
    if np.any(cat["scale"] <= 0) or np.any(cat["D"] <= 0):
        problems.append("non-positive scale or D")
    if np.any((cat["burial"] < 0) | (cat["burial"] > 1)):
        problems.append("burial outside [0, 1]")
    if not set(np.unique(cat["zone"]).tolist()) <= {0, 1}:
        problems.append("zone must be 0 (core) or 1 (ring)")
    if np.any(cat["lod"] < 0):
        problems.append("negative lod")
    if library is not None:
        n = len(library)
        if np.any(cat["proto"] < 0) or np.any(cat["proto"] >= n):
            problems.append("proto index out of range for the library")
        else:
            a = library.a_array()[cat["proto"]]
            rel = np.abs(cat["scale"] * a - cat["D"]) / cat["D"]
            if np.any(rel > 1e-5):
                problems.append(f"{int(np.sum(rel > 1e-5))} rows with D != scale * a_proto")
            nl = np.array([p.n_lods for p in library.prototypes])[cat["proto"]]
            if np.any(cat["lod"] >= nl):
                problems.append("lod beyond the prototype's LOD count")
    if cfg is not None:
        from ..layout import Layout

        lay = Layout(cfg)
        core = cat[cat["zone"] == 0]
        if len(core):
            r = lay.zone_rect("core")
            if not np.all(r.contains(core["x"], core["y"])):
                problems.append("core rock centre outside the core")
            T = lay.tile_size
            ti = np.clip(np.floor((core["x"] - r.x0) / T), 0, lay.n_tiles - 1).astype(int)
            tj = np.clip(np.floor((core["y"] - r.y0) / T), 0, lay.n_tiles - 1).astype(int)
            if np.any(ti != core["tile_i"]) or np.any(tj != core["tile_j"]):
                problems.append("core tile assignment does not match the rock centre")
        coll = float(cfg["rocks"]["collision_min_D_m"])
        if np.any(cat["collide"] != ((cat["zone"] == 0) & (cat["D"] >= coll))):
            problems.append("collide flag != (core rock and D >= collision_min_D_m)")
    return problems


def check_catalog(catalog: np.ndarray, library=None, cfg=None) -> None:
    problems = validate_catalog(catalog, library, cfg)
    if problems:
        raise ValueError("invalid rock catalog: " + "; ".join(problems))


# ---------------------------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------------------------
def poisson_interval(n: int | np.ndarray, conf: float = 0.95) -> tuple[np.ndarray, np.ndarray]:
    """Exact (Garwood) two-sided confidence interval for a Poisson mean given a count n."""
    from scipy.stats import chi2

    n = np.asarray(n, dtype=np.float64)
    alpha = 1.0 - conf
    lo = np.where(n > 0, 0.5 * chi2.ppf(alpha / 2, 2 * n), 0.0)
    hi = 0.5 * chi2.ppf(1 - alpha / 2, 2 * (n + 1))
    return lo, hi


def cumulative_counts(catalog: np.ndarray, D_list=SFD_CHECK_D_M, expected: dict | None = None,
                      conf: float = 0.95) -> list[dict]:
    """N(>=D) for each D with the exact Poisson interval; with ``expected`` ({D: mean}) also the
    z-score (n - mu) / sqrt(mu) and whether n lies within mu +- 3 sqrt(mu)."""
    D = np.asarray(catalog["D"])
    rows = []
    for d in D_list:
        n = int(np.sum(D >= d - 1e-12))
        lo, hi = poisson_interval(n, conf)
        row = {"D_m": float(d), "N": n, f"ci{int(round(conf * 100))}": [float(lo), float(hi)]}
        if expected is not None:
            mu = _lookup(expected, d)
            if mu is not None:
                row["expected"] = float(mu)
                row["z"] = float((n - mu) / np.sqrt(mu)) if mu > 0 else (0.0 if n == 0 else float("inf"))
                row["within_3sigma"] = bool(abs(n - mu) <= 3.0 * np.sqrt(max(mu, 1e-12)))
        rows.append(row)
    return rows


def _lookup(table: dict, d: float):
    for k, v in table.items():
        if abs(float(k) - d) < 1e-9:
            return v
    return None


def catalog_stats(catalog: np.ndarray, library=None, cfg=None) -> dict:
    """Counts per zone / class / LOD and the triangle totals (needs the library for triangles)."""
    cat = np.asarray(catalog)
    out: dict = {"n": int(len(cat)), "zones": {}}
    names = None
    if cfg is not None:
        from .placement import class_table

        names = [c["name"] for c in class_table(cfg)]
    tri_table = None
    if library is not None:
        nl = max(p.n_lods for p in library.prototypes)
        tri_table = np.zeros((len(library), nl), dtype=np.int64)
        for p in library.prototypes:
            tri_table[p.index, :p.n_lods] = p.lod_tris
    for zcode, zname in ((0, "core"), (1, "ring")):
        sub = cat[cat["zone"] == zcode]
        if not len(sub):
            continue
        z: dict = {"n": int(len(sub)), "n_collide": int(sub["collide"].sum()),
                   "D_m": [float(sub["D"].min()), float(sub["D"].max())], "classes": {}}
        tri_total = 0
        for c in np.unique(sub["cls"]):
            s = sub[sub["cls"] == c]
            key = names[int(c)] if names else str(int(c))
            entry = {"n": int(len(s)), "lods": {int(l): int(np.sum(s["lod"] == l)) for l in np.unique(s["lod"])}}
            if tri_table is not None:
                t = int(tri_table[s["proto"], s["lod"]].sum())
                entry["triangles"] = t
                tri_total += t
            z["classes"][key] = entry
        if library is not None:
            z["triangles"] = tri_total
        out["zones"][zname] = z
    return out


def split_by_tile(catalog: np.ndarray) -> dict[tuple[int, int, int], np.ndarray]:
    """Row indices grouped by (zone, tile_i, tile_j) (ring: (1, strip_index, -1))."""
    cat = np.asarray(catalog)
    key = (cat["zone"].astype(np.int64) * 1_000_000 + (cat["tile_i"].astype(np.int64) + 1) * 1000
           + (cat["tile_j"].astype(np.int64) + 1))
    order = np.argsort(key, kind="stable")
    uk, start = np.unique(key[order], return_index=True)
    out = {}
    bounds = list(start) + [len(order)]
    for k, s0, s1 in zip(uk, bounds[:-1], bounds[1:]):
        zone = int(k // 1_000_000)
        ti = int((k // 1000) % 1000) - 1
        tj = int(k % 1000) - 1
        out[(zone, ti, tj)] = order[s0:s1]
    return out


def _json_default(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")
