#!/usr/bin/env python3
"""Instrument OpenVINS's MSCKF update with per-update stage counts (applied by setup_openvins.sh).

Adds one DEBUG-level line per update to ov_msckf/src/update/UpdaterMSCKF.cpp:

    [MSCKF-STAGES] in=N tri_fail=N refine_fail=N chi2_fail=N used=N chi2_ratio_mean=F res_rms_px=F

(features offered, lost to triangulation, to the Gauss-Newton refinement, to the chi-square gate, used; the mean
chi2 / threshold ratio of the gated ones; the RMS reprojection residual of the used ones). It is what showed, on
2026-10-31, that three features in four failed triangulation on the lunar bags. Silent at the INFO verbosity of
the replays; `verbosity: "DEBUG"` in estimator_config.yaml shows it. Idempotent; a copy already carrying the line
at INFO level is moved to DEBUG.

usage: openvins_patch_msckf_stages.py PATH/TO/UpdaterMSCKF.cpp
"""
import sys
from pathlib import Path

p = Path(sys.argv[1])
s = p.read_text()
if "[MSCKF-STAGES]" in s:
    if 'PRINT_INFO("[MSCKF-STAGES]' in s:
        p.write_text(s.replace('PRINT_INFO("[MSCKF-STAGES]', 'PRINT_DEBUG("[MSCKF-STAGES]'))
        print("MSCKF-STAGES line moved to DEBUG")
    else:
        print("already patched")
    sys.exit(0)


def replace_once(text, old, new):
    assert text.count(old) == 1, f"anchor not found exactly once: {old[:70]!r}"
    return text.replace(old, new)


s = replace_once(s,
                 "  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements\n  auto it1 = feature_vec.begin();",
                 "  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements\n"
                 "  int dbg_n_in = (int)feature_vec.size(), dbg_n_tri_fail = 0, dbg_n_ref_fail = 0, dbg_n_chi2 = 0;\n"
                 "  double dbg_chi2_sum = 0, dbg_res_sum = 0;\n"
                 "  int dbg_res_n = 0;\n"
                 "  auto it1 = feature_vec.begin();")
s = replace_once(s,
                 "    if (!success_tri || !success_refine) {\n      (*it1)->to_delete = true;",
                 "    if (!success_tri || !success_refine) {\n      if (!success_tri) dbg_n_tri_fail++; else dbg_n_ref_fail++;\n      (*it1)->to_delete = true;")
s = replace_once(s,
                 "    if (chi2 > _options.chi2_multipler * chi2_check) {\n      (*it2)->to_delete = true;\n      it2 = feature_vec.erase(it2);",
                 "    dbg_chi2_sum += chi2 / chi2_check;\n"
                 "    if (chi2 > _options.chi2_multipler * chi2_check) {\n      dbg_n_chi2++;\n      (*it2)->to_delete = true;\n      it2 = feature_vec.erase(it2);")
s = replace_once(s,
                 "    // We are good!!! Append to our large H vector\n",
                 "    // We are good!!! Append to our large H vector\n    dbg_res_sum += res.norm() / std::sqrt((double)res.rows()); dbg_res_n++;\n")
s = replace_once(s,
                 '  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\\n"',
                 '  PRINT_DEBUG("[MSCKF-STAGES] in=%d tri_fail=%d refine_fail=%d chi2_fail=%d used=%d chi2_ratio_mean=%.2f res_rms_px=%.2f\\n", '
                 'dbg_n_in, dbg_n_tri_fail, dbg_n_ref_fail, dbg_n_chi2, dbg_res_n, '
                 '(dbg_n_in - dbg_n_tri_fail - dbg_n_ref_fail) > 0 ? dbg_chi2_sum / (dbg_n_in - dbg_n_tri_fail - dbg_n_ref_fail) : 0.0, '
                 'dbg_res_n > 0 ? dbg_res_sum / dbg_res_n : 0.0);\n'
                 '  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\\n"')
p.write_text(s)
print("patched", p)
