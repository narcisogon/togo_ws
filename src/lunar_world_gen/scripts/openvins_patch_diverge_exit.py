#!/usr/bin/env python3
"""Let OpenVINS's ROS 2 node exit when its estimate has diverged, so a launch with respawn restarts it (applied by
setup_openvins.sh).

A stock OpenVINS that has lost its features keeps integrating the IMU for ever (2026-11-01: 1 km off after a turn
on the spot at the slope waypoint). This adds one node parameter to ov_msckf's ROS2Visualizer:

    diverge_velocity_mps   when the estimated speed stays above this for diverge_hold_s seconds (default 1 s)
                           the node logs it and exits with code 3 (0 = off, the stock behaviour)
    diverge_hold_s

seyond_dlio_slam.launch.py starts the node with respawn, so it comes back and initialises again (statically at a
standstill, dynamically while driving when init_dyn_use is on). The SLAM side treats the new frame as a restart
(DLIO odom/iekf/vio/frameJumpM, the backend's vio_edge_frame_jump_m). Idempotent.

usage: openvins_patch_diverge_exit.py PATH/TO/ov_msckf/src/ros
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])
h, c = root / "ROS2Visualizer.h", root / "ROS2Visualizer.cpp"
hs, cs = h.read_text(), c.read_text()
if "diverge_velocity_mps" in cs:
    print("already patched")
    sys.exit(0)


def replace_once(text, old, new):
    assert text.count(old) == 1, f"anchor not found exactly once: {old[:80]!r}"
    return text.replace(old, new)


hs = replace_once(hs,
                  "  // Our publishers\n",
                  "  // Exit on divergence (diverge_velocity_mps / diverge_hold_s node parameters, 0 = off): the launch respawns the node\n"
                  "  double diverge_velocity_mps = 0.0;\n"
                  "  double diverge_hold_s = 1.0;\n"
                  "  double diverge_since = -1.0;\n"
                  "\n"
                  "  // Our publishers\n")
h.write_text(hs)

cs = replace_once(cs,
                  "  // Load if we should save the total state to file\n",
                  "  // Exit on divergence\n"
                  "  if (node->has_parameter(\"diverge_velocity_mps\")) {\n"
                  "    node->get_parameter<double>(\"diverge_velocity_mps\", diverge_velocity_mps);\n"
                  "  }\n"
                  "  if (node->has_parameter(\"diverge_hold_s\")) {\n"
                  "    node->get_parameter<double>(\"diverge_hold_s\", diverge_hold_s);\n"
                  "  }\n"
                  "  if (diverge_velocity_mps > 0) {\n"
                  "    PRINT_INFO(\"exit on divergence: speed above %.1f m/s for %.1f s\\n\", diverge_velocity_mps, diverge_hold_s);\n"
                  "  }\n"
                  "\n"
                  "  // Load if we should save the total state to file\n")
# the check: at the end of visualize(), after the state is published, when initialised
cs = replace_once(cs,
                  "  // Save total state\n",
                  "  // Exit on divergence: a speed no rover has, held for diverge_hold_s\n"
                  "  if (diverge_velocity_mps > 0 && _app->initialized()) {\n"
                  "    const double speed = _app->get_state()->_imu->vel().norm();\n"
                  "    const double now = _app->get_state()->_timestamp;\n"
                  "    if (speed > diverge_velocity_mps) {\n"
                  "      if (diverge_since < 0) {\n"
                  "        diverge_since = now;\n"
                  "      } else if (now - diverge_since > diverge_hold_s) {\n"
                  "        PRINT_ERROR(RED \"estimate diverged (%.1f m/s for %.1f s at %.3f): exiting for a respawn\\n\" RESET, speed, now - diverge_since, now);\n"
                  "        std::_Exit(3);   // immediate: a normal exit from inside a callback tears the node down under its threads\n"
                  "      }\n"
                  "    } else {\n"
                  "      diverge_since = -1.0;\n"
                  "    }\n"
                  "  }\n"
                  "\n"
                  "  // Save total state\n")
c.write_text(cs)
print("patched", h, c)
