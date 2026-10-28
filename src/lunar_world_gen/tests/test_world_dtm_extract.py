"""Tests for tools/dtm_extract.py on the real Apollo 16 DTM (skipped if the DTM is not present).

Run (WSL, project venv):
    ~/.venvs/lunar_world/bin/python -m pytest -q src/lunar_world_gen/tests/test_world_dtm_extract.py
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import dtm_extract as dx  # noqa: E402

pytestmark = pytest.mark.skipif(not dx.DEFAULT_DTM.exists(), reason="NAC_DTM_APOLLO16.TIF not available")


def run(tmp_path, name, *args):
    out = tmp_path / name
    dx.main([*map(str, args), "--out", str(out)])
    z = np.load(str(out) + ".npy")
    meta = json.loads(Path(str(out) + ".json").read_text())
    return z, meta


def test_projection_and_lm_height():
    assert dx.selftest(dx.EqcDTM(dx.DEFAULT_DTM))


def test_projection_constants():
    d = dx.EqcDTM(dx.DEFAULT_DTM)
    assert d.R == 1737400.0 and d.lat_ts == -9.0 and d.lon0 == 180.0
    x, y = d.ll2xy(-8.9734, 15.5011)  # LROC LM
    c, r = d.xy2px(x, y)
    assert abs(float(c) - 5402.355) < 1e-3 and abs(float(r) - 5449.776) < 1e-3


def test_core_extract(tmp_path):
    z, m = run(tmp_path, "core", "--site", "core", "--spacing", "1.0", "--lowpass-lambda", "10")
    assert z.shape == (101, 101) and np.isfinite(z).all()
    assert m["grid"]["row0"] == "south"
    w = np.ones(101); w[0] = w[-1] = 0.5
    area_mean = float((z * w[:, None] * w[None, :]).sum() / w.sum() ** 2)
    assert abs(area_mean) < 0.01          # href = area mean of the band-limited DTM
    assert -6 < z.min() < -3 and 4 < z.max() < 8
    assert abs(m["centre"]["pixel_col"] - 5552.4948) < 1e-3 and abs(m["centre"]["pixel_row"] - 5450.5041) < 1e-3


def test_row0_and_spacing_consistency(tmp_path):
    a, _ = run(tmp_path, "s", "--site", "core", "--spacing", "0.5", "--lowpass-lambda", "10")
    b, _ = run(tmp_path, "n", "--site", "core", "--spacing", "0.5", "--lowpass-lambda", "10", "--row0", "north")
    c, _ = run(tmp_path, "c", "--site", "core", "--spacing", "1.0", "--lowpass-lambda", "10")
    assert np.array_equal(a, b[::-1])
    assert np.abs(a[::2, ::2] - c).max() < 1e-5


def test_window_size_independence(tmp_path):
    a, _ = run(tmp_path, "a", "--site", "core", "--spacing", "1.0", "--lowpass-lambda", "10")
    b, _ = run(tmp_path, "b", "--site", "core", "--size", "400", "--spacing", "1.0", "--lowpass-lambda", "10")
    o = (b.shape[0] - a.shape[0]) // 2
    assert np.abs(a - b[o:o + a.shape[0], o:o + a.shape[1]]).max() < 1e-3


def test_curvature(tmp_path):
    a, _ = run(tmp_path, "cv", "--site", "core", "--size", "2000", "--spacing", "50")
    b, _ = run(tmp_path, "nc", "--site", "core", "--size", "2000", "--spacing", "50", "--no-curvature")
    d = b - a  # drop at the corner (d = 1414 m) ~ d^2 / 2R = 0.576 m
    assert abs(float(d[0, 0]) - 1414.2 ** 2 / (2 * 1737400.0)) < 0.02
    assert abs(float(d[20, 20])) < 1e-3


def test_chunk_size_does_not_change_the_output(tmp_path):
    """CR-07: the vertical-sampling iteration is decided once per extraction, not per row chunk.

    The production far extraction (2.6 km at 2 m) is close to the iteration threshold (estimated shift
    0.115 m vs 0.1 m); with 100-row chunks the old per-chunk rule mixed iterated and plain rows (74 mm
    steps). Now every chunking gives the same bits and the same decision record."""
    far = ("--site", "core", "--size", "2600", "--spacing", "2", "--lowpass-lambda", "10", "--deseam", "default")
    a, ma = run(tmp_path, "far_default", *far)
    b, mb = run(tmp_path, "far_100rows", *far, "--chunk-nodes", "130100")
    assert ma["frame"]["vertical_iteration"] is True
    assert ma["frame"]["vertical_iteration_decision"] == mb["frame"]["vertical_iteration_decision"]
    assert ma["frame"]["max_radial_vs_vertical_shift_m"] == mb["frame"]["max_radial_vs_vertical_shift_m"]
    assert np.array_equal(a, b)
    # explicit modes: 'never' / 'always' override the decision; chunking is still irrelevant
    small = ("--site", "core", "--size", "400", "--spacing", "1.0")
    n, mn = run(tmp_path, "never", *small, "--vertical-iteration", "never")
    s1, ms1 = run(tmp_path, "always", *small, "--vertical-iteration", "always")
    s2, _ = run(tmp_path, "always_chunked", *small, "--vertical-iteration", "always", "--chunk-nodes", "4010")
    assert mn["frame"]["vertical_iteration"] is False and ms1["frame"]["vertical_iteration"] is True
    assert np.array_equal(s1, s2)
    assert 0 < np.abs(s1.astype(np.float64) - n).max() < 1e-3      # sub-mm effect within 200 m


def test_deseam_far_field(tmp_path):
    def seam_step(z):
        s = (z[:, 2:-1] - z[:, 1:-2]) - 0.5 * ((z[:, 1:-2] - z[:, :-3]) + (z[:, 3:] - z[:, 2:-1]))
        i0 = int((958 + 1100) / 2)
        band = s[:, i0 - 40:i0 + 40]
        return np.median([np.abs(np.median(band[r:r + 50], axis=0)).max() for r in range(0, z.shape[0] - 50, 50)])
    raw, _ = run(tmp_path, "raw", "--site", "core", "--size", "2200", "--spacing", "2")
    fix, m = run(tmp_path, "fix", "--site", "core", "--size", "2200", "--spacing", "2", "--deseam", "default")
    assert m["processing"]["deseam"]
    assert seam_step(raw.astype(np.float64)) > 0.3
    assert seam_step(fix.astype(np.float64)) < 0.1
    assert np.abs(fix - raw)[500:601, 500:601].max() < 1e-6   # core untouched
