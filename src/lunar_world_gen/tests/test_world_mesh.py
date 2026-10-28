"""Tests for the mesh owner: mesh/{tin,stitch,normals,uv,gltf}.py and materials/regolith.py.

Run (WSL, project venv), from src/lunar_world_gen:
    ~/.venvs/lunar_world/bin/python -m pytest tests/test_world_mesh.py -q
"""
from __future__ import annotations

import struct

import numpy as np
import pytest

from lunar_world_gen.config import load_config
from lunar_world_gen.grid import Grid, from_lattice
from lunar_world_gen.layout import Rect
from lunar_world_gen.materials.regolith import (height_to_normal_map, make_regolith_textures, periodic_noise,
                                                seam_ratio, srgb_to_linear, synthesize_regolith)
from lunar_world_gen.mesh.gltf import planar_tangents, read_glb, read_glb_json, verify_glb, write_glb
from lunar_world_gen.mesh.normals import GridNormalField, assign_normals, triangle_normals, vertex_normals
from lunar_world_gen.mesh.stitch import (Piece, check_stitched, find_contacts, make_piece, stitch_pieces,
                                         strip_priority, tile_priority, union_topology)
from lunar_world_gen.mesh.tin import grid_node_index, tin_from_grid, tin_max_error, tin_surface_on_grid
from lunar_world_gen.mesh.types import MaterialSpec, Mesh
from lunar_world_gen.mesh.uv import assign_uvs, planar_uv
from lunar_world_gen.layout import TileId


# ---------------------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------------------
def fbm_grid(nx: int, ny: int, spacing: float, x0: float, y0: float, seed: int = 0, rms_1node: float = 0.0017,
             lam_min_nodes: float = 2.0) -> Grid:
    """Non-periodic fBm-like test grid (H = 0.8) with a given RMS height step per node."""
    rng = np.random.default_rng(seed)
    n = int(2 ** np.ceil(np.log2(max(nx, ny) * 1.5)))
    fx = np.fft.fftfreq(n)
    f = np.hypot(fx[None, :], fx[:, None])
    f[0, 0] = 1.0
    amp = f ** (-1.8)
    amp[0, 0] = 0.0
    amp[f > 1.0 / lam_min_nodes] = 0.0
    h = np.real(np.fft.ifft2((rng.standard_normal((n, n)) + 1j * rng.standard_normal((n, n))) * amp))
    h = h[:ny, :nx]
    h *= rms_1node / np.sqrt(np.mean(np.diff(h, axis=1) ** 2))
    return Grid(h.astype(np.float32), x0, y0, spacing)


def H(x, y):
    """Smooth analytic test height field with some cm-scale detail."""
    return (0.2 * np.sin(0.9 * x + 0.3) * np.cos(0.7 * y) + 0.03 * np.sin(7 * x) * np.sin(5 * y)
            + 0.004 * np.sin(37 * x + 11 * y) + 0.002 * np.cos(53 * y - 17 * x))


def analytic_grid(x0, x1, y0, y1, s, noise=0.0, seed=0) -> Grid:
    nx = int(round((x1 - x0) / s)) + 1
    ny = int(round((y1 - y0) / s)) + 1
    X, Y = np.meshgrid(x0 + np.arange(nx) * s, y0 + np.arange(ny) * s)
    z = H(X, Y)
    if noise:
        z = z + np.random.default_rng(seed).normal(0.0, noise, z.shape)
    return Grid(z, x0, y0, s)


def piece_from_grid(zone_grid: Grid, rect: Rect, max_error: float, priority: int, name: str,
                    uv_tile_m: float) -> Piece:
    g = zone_grid.crop(rect.x0, rect.x1, rect.y0, rect.y1)
    m = tin_from_grid(g, max_error, name)
    field = GridNormalField(zone_grid)
    assign_normals(m, field)
    assign_uvs(m, uv_tile_m)
    return Piece(m, rect, priority, height_fn=lambda x, y, g=g: g.sample(x, y), normal_fn=field,
                 uv_tile_m=uv_tile_m, name=name)


def synthetic_layout(core_noise: float = 0.0, outer_noise: float = 1e-5) -> list[Piece]:
    """2 x 2 core tiles (1 m, 1 cm) + 4 ring strips (25 cm) + 4 outer strips (2 m)."""
    pieces = []
    core = analytic_grid(-1, 1, -1, 1, 0.01, noise=core_noise, seed=3)
    for j in range(2):
        for i in range(2):
            r = Rect(-1.0 + i, float(i), -1.0 + j, float(j))
            pieces.append(piece_from_grid(core, r, 0.001, tile_priority(TileId(i, j), 2),
                                          f"tile_{i:02d}_{j:02d}", 2.0))
    ring = analytic_grid(-3, 3, -3, 3, 0.25, noise=outer_noise, seed=1)
    strips = {"south": Rect(-3, 3, -3, -1), "north": Rect(-3, 3, 1, 3), "west": Rect(-3, -1, -1, 1),
              "east": Rect(1, 3, -1, 1)}
    for side, r in strips.items():
        pieces.append(piece_from_grid(ring, r, 0.005, strip_priority("ring", side), f"ring_{side}", 2.0))
    far = analytic_grid(-11, 11, -11, 11, 2.0, noise=outer_noise, seed=2)
    strips = {"south": Rect(-11, 11, -11, -3), "north": Rect(-11, 11, 3, 11), "west": Rect(-11, -3, -3, 3),
              "east": Rect(3, 11, -3, 3)}
    for side, r in strips.items():
        pieces.append(piece_from_grid(far, r, 0.05, strip_priority("far", side), f"far_{side}", 20.0))
    return pieces


# ---------------------------------------------------------------------------------------------
# TIN
# ---------------------------------------------------------------------------------------------
def test_pydelatin_convention_probe():
    """pydelatin mirrors rows: output (x, y) = (col, H-1-row); tin_from_grid undoes it exactly."""
    from pydelatin import Delatin

    z = np.zeros((3, 5), np.float32)
    z[0, 1] = 1.0  # spike at row 0 (south), column 1
    d = Delatin(z, max_error=0.01)
    v = d.vertices
    spike = v[np.argmax(v[:, 2])]
    assert tuple(spike) == (1.0, 2.0, 1.0)  # mirrored row axis
    g = Grid(z.astype(np.float64), 10.0, -5.0, 0.25)
    m = tin_from_grid(g, 0.01, "probe")
    k = int(np.argmax(m.vertices[:, 2]))
    assert m.vertices[k, 0] == pytest.approx(10.25) and m.vertices[k, 1] == pytest.approx(-5.0)
    assert m.vertices[k, 2] == 1.0
    assert np.all(m.face_normals()[:, 2] > 0)


@pytest.mark.parametrize("max_error", [0.002, 0.005, 0.01])
def test_tin_error_bound_vertices_on_nodes(max_error):
    g = fbm_grid(241, 173, 0.01, -50.0, -48.27, seed=7)  # non-square, offset origin
    m = tin_from_grid(g, max_error, "t")
    m.validate(require_up=True)
    # error bound over every node (independent numba rasteriser)
    err = tin_max_error(m, g)
    assert err["uncovered_nodes"] == 0
    assert err["max_abs_m"] <= max_error + 2e-6
    # vertices on nodes with the node heights (float64, exact), lattice consistent
    i, j, on = grid_node_index(g, m.lattice)
    assert on.all()
    assert np.array_equal(m.vertices[:, 2], np.asarray(g.z, np.float64)[j, i])
    assert np.array_equal(m.vertices[:, :2], from_lattice(m.lattice))
    assert m.vertices.dtype == np.float64
    # corners
    corners = {(0, 0), (g.nx - 1, 0), (0, g.ny - 1), (g.nx - 1, g.ny - 1)}
    assert corners.issubset(set(zip(i.tolist(), j.tolist())))
    # CCW from +z and the triangles tile the rectangle exactly
    v = m.vertices
    t = m.triangles
    a2 = (v[t[:, 1], 0] - v[t[:, 0], 0]) * (v[t[:, 2], 1] - v[t[:, 0], 1]) - \
         (v[t[:, 1], 1] - v[t[:, 0], 1]) * (v[t[:, 2], 0] - v[t[:, 0], 0])
    assert np.all(a2 > 0)
    assert 0.5 * a2.sum() == pytest.approx((g.x1 - g.x0) * (g.y1 - g.y0), rel=1e-9)
    assert m.meta["tin_error"] <= max_error


def test_tin_flat_grid_is_two_triangles_and_float64_heights():
    z = np.full((101, 201), 1234.5678901234, dtype=np.float64)  # far-from-zero height, float64
    g = Grid(z, 0.0, 0.0, 0.25)
    m = tin_from_grid(g, 0.01)
    assert m.n_vertices == 4 and m.n_triangles == 2
    assert np.all(m.vertices[:, 2] == 1234.5678901234)


def test_tin_rejects_off_lattice_grid():
    g = Grid(np.zeros((5, 5)), 0.005, 0.0, 0.01)
    with pytest.raises(ValueError):
        tin_from_grid(g, 0.01)


def test_tin_surface_matches_exact_interpolation():
    g = fbm_grid(64, 48, 0.01, 0.0, 0.0, seed=2)
    m = tin_from_grid(g, 0.003)
    surf = tin_surface_on_grid(m, g)
    i, j, _ = grid_node_index(g, m.lattice)
    assert np.allclose(surf[j, i], m.vertices[:, 2], atol=1e-12)  # exact at vertices


# ---------------------------------------------------------------------------------------------
# normals and UVs
# ---------------------------------------------------------------------------------------------
def test_normals_plane_exact():
    a, b = 0.3, -0.12
    X, Y = np.meshgrid(np.arange(81) * 0.01, np.arange(61) * 0.01)
    g = Grid(a * X + b * Y + 0.001 * np.sin(40 * X) * 0, 0.0, 0.0, 0.01)
    m = tin_from_grid(g, 0.001)
    expect = np.array([-a, -b, 1.0]) / np.sqrt(a * a + b * b + 1)
    for adaptive in (True, False):
        n = vertex_normals(m, g, adaptive=adaptive)
        assert n.dtype == np.float32
        assert np.allclose(n, expect[None, :], atol=1e-6)


def test_normals_unit_up_and_keep_fine_relief():
    g = fbm_grid(201, 201, 0.01, 0.0, 0.0, seed=5)
    m = tin_from_grid(g, 0.005)
    n_grid = vertex_normals(m, g, adaptive=False, sigma_nodes=1.0)
    n_ad = vertex_normals(m, g)
    n_tri = triangle_normals(m)
    for n in (n_grid, n_ad, n_tri):
        assert np.allclose(np.linalg.norm(n.astype(np.float64), axis=1), 1.0, atol=1e-6)
        assert np.all(n[:, 2] > 0)
    # the grid normals carry more slope than the decimated-triangle normals (fine relief kept)
    tilt = lambda n: np.degrees(np.arccos(np.clip(n[:, 2], -1, 1))).mean()  # noqa: E731
    assert tilt(n_grid) > tilt(n_tri)
    assert tilt(n_grid) >= tilt(n_ad) - 1e-6  # adaptive smoothing only removes slope
    field = GridNormalField(g, (1.0, 2.0))
    assert np.allclose(field(m.vertices[:, 0], m.vertices[:, 1]), field.normals(m.vertices[:, 0], m.vertices[:, 1], 1.0))


def test_planar_uv():
    uv = planar_uv(np.array([0.0, 2.0, -3.0]), np.array([0.0, 1.0, 4.0]), 2.0)
    assert uv.dtype == np.float32
    assert np.allclose(uv, [[0, 0], [1, -0.5], [-1.5, -2.0]])
    m = Mesh(np.array([[1.0, 2.0, 0.0], [3.0, 2.0, 0.0], [1.0, 4.0, 0.0]]), np.array([[0, 1, 2]]))
    assign_uvs(m, 20.0)
    assert np.allclose(m.uvs[1], [0.15, -0.1])


# ---------------------------------------------------------------------------------------------
# stitching
# ---------------------------------------------------------------------------------------------
def test_stitch_synthetic_layout_zero_mismatch_watertight():
    pieces = synthetic_layout(core_noise=2e-4)
    before = check_stitched(pieces)
    assert before["t_junctions"] > 0 and not before["ok"]  # the raw TINs do not match
    rep = stitch_pieces(pieces)
    assert rep["n_inserted"] > 0
    assert rep["max_abs_dz_overwritten_m"] < 1e-3
    res = check_stitched(pieces)
    assert res["mismatches"] == 0
    assert res["t_junctions"] == 0
    assert res["normal_mismatches"] == 0
    topo = res["topology"]
    assert topo["cracks"] == 0 and topo["nonmanifold_edges"] == 0 and topo["orientation_conflicts"] == 0
    assert topo["euler"] == 1  # the union is one disc: watertight sheet, no internal boundary
    assert res["ok"]
    for p in pieces:
        p.mesh.validate(require_up=True)
        if p.mesh.uvs is not None:  # planar UVs stay exact for inserted vertices
            assert np.allclose(p.mesh.uvs, planar_uv(p.mesh.vertices[:, 0], p.mesh.vertices[:, 1],
                                                     p.uv_tile_m), atol=1e-6)
    # canonical heights: on the core boundary the ring strips carry the core tiles' heights
    core = analytic_grid(-1, 1, -1, 1, 0.01, noise=2e-4, seed=3)
    ring_s = next(p for p in pieces if p.name == "ring_south")
    on = (ring_s.mesh.lattice[:, 1] == -100) & (np.abs(ring_s.mesh.lattice[:, 0]) <= 100)
    assert on.sum() > 10
    i, j, ok = grid_node_index(core, ring_s.mesh.lattice[on])
    assert ok.all()
    assert np.array_equal(ring_s.mesh.vertices[on, 2], np.asarray(core.z)[j, i])
    # union as one trimesh: boundary edges only on the outer square
    import trimesh

    allv, allt, off = [], [], 0
    keymap: dict[tuple[int, int], int] = {}
    for p in pieces:
        ids = np.empty(p.mesh.n_vertices, dtype=np.int64)
        for k, (lx, ly) in enumerate(p.mesh.lattice.tolist()):
            if (lx, ly) not in keymap:
                keymap[(lx, ly)] = len(keymap)
                allv.append(p.mesh.vertices[k])
            ids[k] = keymap[(lx, ly)]
        allt.append(ids[p.mesh.triangles])
    tm = trimesh.Trimesh(np.array(allv), np.vstack(allt), process=False)
    edges = trimesh.grouping.group_rows(tm.edges_sorted, require_count=1)
    outer = tm.edges_sorted[edges]
    xy = tm.vertices[outer][:, :, :2]
    on_outer = np.all(np.isclose(np.abs(xy), 11.0).any(axis=2), axis=1)
    assert on_outer.all()
    assert tm.is_winding_consistent


def test_stitch_corners_three_and_four_pieces_canonical_heights():
    """4 tiles meeting in one point over a strip: every shared point takes the lowest priority's value."""
    pieces = []
    zone = analytic_grid(0, 2, 0, 2, 0.01)
    for j in range(2):
        for i in range(2):
            r = Rect(float(i), float(i + 1), float(j), float(j + 1))
            g = zone.crop(r.x0, r.x1, r.y0, r.y1)
            g = Grid(np.asarray(g.z) + 1e-3 * (j * 2 + i), g.x0, g.y0, g.spacing)  # per-piece offset
            m = tin_from_grid(g, 0.002, f"t{i}{j}")
            pieces.append(Piece(m, r, j * 2 + i, name=f"t{i}{j}"))
    outer = analytic_grid(-1, 3, -1, 0, 0.25)
    g = Grid(np.asarray(outer.z) + 0.01, outer.x0, outer.y0, outer.spacing)
    pieces.append(Piece(tin_from_grid(g, 0.003, "strip"), Rect(-1, 3, -1, 0), 100, name="strip"))
    kinds = sorted(c.kind for c in find_contacts(pieces))
    assert kinds.count("point") == 2  # (t00, t11) and (t10, t01) diagonals
    stitch_pieces(pieces)
    res = check_stitched(pieces)
    assert res["ok"], res
    # the 4-piece corner (1, 1) has t00's height everywhere
    z_ref = pieces[0].mesh.vertices[(pieces[0].mesh.lattice == [100, 100]).all(axis=1), 2]
    for p in pieces[:4]:
        k = (p.mesh.lattice == [100, 100]).all(axis=1)
        assert k.sum() == 1 and p.mesh.vertices[k, 2] == z_ref
    # 3-piece point (1, 0): t00 (prio 0), t10 (prio 1) and the strip (prio 100)
    z0 = pieces[0].mesh.vertices[(pieces[0].mesh.lattice == [100, 0]).all(axis=1), 2]
    for p in (pieces[1], pieces[4]):
        k = (p.mesh.lattice == [100, 0]).all(axis=1)
        assert k.sum() == 1 and p.mesh.vertices[k, 2] == z0
    assert union_topology(pieces)["euler"] == 1


def test_stitch_is_idempotent_and_deterministic():
    a = synthetic_layout()
    b = synthetic_layout()
    stitch_pieces(a)
    stitch_pieces(b)
    for pa, pb in zip(a, b):
        assert np.array_equal(pa.mesh.vertices, pb.mesh.vertices)
        assert np.array_equal(pa.mesh.triangles, pb.mesh.triangles)
    rep2 = stitch_pieces(a)
    assert rep2["n_inserted"] == 0 and rep2["max_abs_dz_overwritten_m"] == 0.0
    assert rep2["max_normal_change_deg"] == 0.0


def test_stitch_without_height_fn_keeps_canonical_surface():
    """Canonical piece without height_fn: inserted vertices interpolate its edge (surface unchanged)."""
    zone = analytic_grid(0, 2, 0, 1, 0.01)
    left = tin_from_grid(zone.crop(0, 1, 0, 1), 0.004, "L")
    right_g = analytic_grid(1, 2, 0, 1, 0.05)
    right = tin_from_grid(right_g, 0.0005, "R")  # dense right piece -> points inserted into L
    gL = zone.crop(0, 1, 0, 1)
    before = tin_surface_on_grid(left, gL)
    pieces = [Piece(left, Rect(0, 1, 0, 1), 0), Piece(right, Rect(1, 2, 0, 1), 1)]
    rep = stitch_pieces(pieces)
    assert rep["inserted"].get("L", 0) > 0
    after = tin_surface_on_grid(pieces[0].mesh, gL)
    assert np.nanmax(np.abs(after - before)) < 1e-12
    assert check_stitched(pieces)["ok"]


def test_make_piece_layout_and_alignment_check():
    """make_piece on zone grids (core tiles with a normal margin, ring strips) stitches cleanly."""
    core = analytic_grid(-1.5, 1.5, -1.5, 1.5, 0.01)  # core + 0.5 m margin
    ring = analytic_grid(-3, 3, -3, 3, 0.25, noise=1e-5, seed=4)
    pieces = []
    for j in range(2):
        for i in range(2):
            r = Rect(-1.0 + i, float(i), -1.0 + j, float(j))
            pieces.append(make_piece(core, r, 0.001, tile_priority(TileId(i, j), 2), name=f"tile_{i:02d}_{j:02d}",
                                     uv_tile_m=2.0, normal_margin_m=0.5))
    strips = {"south": Rect(-3, 3, -3, -1), "north": Rect(-3, 3, 1, 3), "west": Rect(-3, -1, -1, 1),
              "east": Rect(1, 3, -1, 1)}
    for side, r in strips.items():
        pieces.append(make_piece(ring, r, 0.005, strip_priority("ring", side), name=f"ring_{side}", uv_tile_m=2.0,
                                 normal_margin_m=1.0))
    # with a margin, the boundary normals of neighbouring tiles agree before any copying
    stitch_rep = stitch_pieces(pieces)
    res = check_stitched(pieces)
    assert res["ok"], res
    assert stitch_rep["n_inserted"] > 0
    with pytest.raises(ValueError):
        make_piece(ring, Rect(-1.1, 1.0, -3.0, -1.0), 0.01, 0, name="misaligned")


def test_overlapping_pieces_rejected():
    g = analytic_grid(0, 1, 0, 1, 0.1)
    m1 = tin_from_grid(g, 0.01)
    m2 = tin_from_grid(analytic_grid(0.5, 1.5, 0, 1, 0.1), 0.01)
    with pytest.raises(ValueError):
        stitch_pieces([Piece(m1, Rect(0, 1, 0, 1), 0), Piece(m2, Rect(0.5, 1.5, 0, 1), 1)])


# ---------------------------------------------------------------------------------------------
# GLB
# ---------------------------------------------------------------------------------------------
def test_glb_roundtrip_and_rules(tmp_path):
    g = fbm_grid(151, 101, 0.01, 12.34, -7.5, seed=1)
    m = tin_from_grid(g, 0.002, "tile_test")
    assign_normals(m, g)
    assign_uvs(m, 2.0)
    mat = MaterialSpec("regolith", (0.12, 0.107, 0.088, 1.0), 0.0, 0.95)
    info = write_glb(tmp_path / "t.glb", m, mat)
    assert info["n_triangles"] == m.n_triangles
    raw = (tmp_path / "t.glb").read_bytes()
    magic, version, total = struct.unpack_from("<III", raw, 0)
    assert magic == 0x46546C67 and version == 2 and total == len(raw) and total % 4 == 0
    doc = read_glb_json(tmp_path / "t.glb")
    prim = doc["meshes"][0]["primitives"][0]
    assert len(doc["meshes"]) == 1 and len(doc["meshes"][0]["primitives"]) == 1
    assert set(prim["attributes"]) == {"POSITION", "NORMAL", "TEXCOORD_0"}
    assert doc["accessors"][prim["indices"]]["componentType"] == 5125  # uint32
    assert all(doc["accessors"][k]["componentType"] == 5126 for k in prim["attributes"].values())
    pbr = doc["materials"][0]["pbrMetallicRoughness"]
    assert pbr["metallicFactor"] == 0.0 and pbr["roughnessFactor"] == pytest.approx(0.95)
    assert pbr["baseColorFactor"][:3] == pytest.approx([0.12, 0.107, 0.088])
    assert "images" not in doc and "textures" not in doc
    assert not any(k in doc["nodes"][0] for k in ("matrix", "translation", "rotation", "scale"))
    back = read_glb(tmp_path / "t.glb")
    assert np.array_equal(back.vertices, m.vertices.astype(np.float32).astype(np.float64))
    assert np.array_equal(back.triangles, m.triangles)
    assert np.allclose(back.normals, m.normals, atol=1e-7)
    assert np.array_equal(back.uvs, m.uvs)
    res = verify_glb(tmp_path / "t.glb", m, require_up=True)
    assert res["ok"], res["errors"]
    assert res["trimesh"]["faces"] == m.n_triangles
    assert res["up_normal_fraction"] == 1.0


def test_glb_tangents_and_empty(tmp_path):
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], float)
    m = Mesh(v, np.array([[0, 1, 2]]), normals=np.tile([0, 0, 1.0], (3, 1)), uvs=planar_uv(v[:, 0], v[:, 1], 2.0))
    tg = planar_tangents(m.normals)
    assert np.allclose(tg, [[1, 0, 0, 1]] * 3)
    write_glb(tmp_path / "tg.glb", m, MaterialSpec("x"), tangents=tg)
    doc = read_glb_json(tmp_path / "tg.glb")
    assert "TANGENT" in doc["meshes"][0]["primitives"][0]["attributes"]
    assert verify_glb(tmp_path / "tg.glb", m)["ok"]
    with pytest.raises(ValueError):
        write_glb(tmp_path / "empty.glb", Mesh(np.zeros((0, 3)), np.zeros((0, 3), int)), MaterialSpec("x"))


# ---------------------------------------------------------------------------------------------
# regolith textures
# ---------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_regolith_tiles_seamlessly_and_is_calibrated(cfg):
    res = synthesize_regolith(cfg, px=256)
    alb, nrm, rgh = res["albedo"], res["normal"], res["roughness"]
    assert alb.shape == (256, 256, 3) and alb.dtype == np.uint8
    # wrap-around continuity: the seam looks like any other neighbour pair
    for img in (alb, nrm, rgh, res["height"]):
        for ax in (0, 1):
            assert seam_ratio(img, ax) < 1.3
    # sanity of the metric: a smooth periodic field passes, a non-periodic crop of it does not
    smooth = periodic_noise(256, 0.01, np.random.default_rng(0), lambda f: f ** -2.0 * (f < 3.0))
    assert seam_ratio(smooth, 1) < 1.3 and seam_ratio(smooth[:, :170], 1) > 3.0
    # calibrated mean colour (linear) and low-contrast variation
    lin = srgb_to_linear(alb.astype(np.float64) / 255.0).reshape(-1, 3).mean(axis=0)
    assert np.allclose(lin, cfg.materials.regolith.base_color_linear, rtol=0.02)
    p01, p99 = res["stats"]["albedo_rel_p01_p99"]
    assert -0.13 < p01 < -0.05 and 0.05 < p99 < 0.13
    # normal map: target RMS slope, unit-ish vectors, +z dominant
    assert res["stats"]["normal_rms_slope_dir_deg"] == pytest.approx(12.0, abs=0.3)
    n = nrm.astype(np.float64) / 127.5 - 1.0
    assert np.all(n[..., 2] > 0.2)
    assert np.allclose(np.linalg.norm(n, axis=-1), 1.0, atol=0.02)
    # roughness ~0.95
    assert rgh.mean() / 255.0 == pytest.approx(0.95, abs=0.01)


def test_regolith_deterministic_and_flip(cfg, tmp_path):
    a = synthesize_regolith(cfg, px=128)
    b = synthesize_regolith(cfg, px=128)
    assert np.array_equal(a["albedo"], b["albedo"]) and np.array_equal(a["normal"], b["normal"])
    f = synthesize_regolith(cfg, px=128, flip_green=True)
    assert np.array_equal(f["normal"][..., 0], a["normal"][..., 0])
    assert np.abs(f["normal"][..., 1].astype(int) - (255 - a["normal"][..., 1].astype(int))).max() <= 1
    paths = make_regolith_textures(cfg, tmp_path, px=64, write_flipped=True)
    from PIL import Image

    for key in ("albedo", "normal", "roughness", "normal_flipped"):
        im = Image.open(paths[key])
        assert im.size == (64, 64) and im.mode == "RGB"


def test_normal_map_orientation_convention():
    """A ramp rising to the east tilts normals west (R < 128); rising to the north tilts them south (G < 128)."""
    n, pix = 64, 0.001
    c = np.arange(n)
    # periodic bumps; check the sign at the steepest point
    h_east = 0.002 * np.sin(2 * np.pi * c / n)[None, :].repeat(n, 0)  # rises with x near c = 0
    nm = height_to_normal_map(h_east, pix)
    assert nm[10, 0, 0] < 128 and abs(int(nm[10, 0, 1]) - 128) <= 1
    h_north = 0.002 * np.sin(2 * np.pi * (-c) / n)[:, None].repeat(n, 1)  # rows go south: rises to north at r=0
    nm = height_to_normal_map(h_north, pix)
    assert nm[0, 10, 1] < 128 and abs(int(nm[0, 10, 0]) - 128) <= 1
    nm_f = height_to_normal_map(h_north, pix, flip_green=True)
    assert nm_f[0, 10, 1] > 128
