#!/usr/bin/env python3
"""Give OpenVINS's ROS 2 front end a per-frame dark-region mask (applied by setup_openvins.sh).

The rover's own shadow moves with the rover, so features on its edge are static in the image while the world
moves; OpenVINS tracks them happily, triangulates them to bogus near points in turns on the spot and the filter
diverges (2026-11-01, the pitched-camera lap). Masks in OpenVINS are static images, so this patch adds two node
parameters to ov_msckf's ROS2Visualizer:

    dark_mask_threshold   pixels at or below this value are masked (0 = off, the stock behaviour)
    dark_mask_dilate_px   the mask is grown by this many pixels so the shadow's edge is masked too (default 12)

The mask is ORed with the static one when use_mask is on. Pass them as `-p dark_mask_threshold:=8` or through
the launch arguments vio_dark_mask_threshold / vio_dark_mask_dilate_px of seyond_dlio_slam.launch.py. Idempotent.

usage: openvins_patch_dark_mask.py PATH/TO/ov_msckf/src/ros
"""
import sys
from pathlib import Path

root = Path(sys.argv[1])
h, c = root / "ROS2Visualizer.h", root / "ROS2Visualizer.cpp"
hs, cs = h.read_text(), c.read_text()
if "dark_mask_threshold" in cs:
    print("already patched")
    sys.exit(0)


def replace_once(text, old, new):
    assert text.count(old) == 1, f"anchor not found exactly once: {old[:80]!r}"
    return text.replace(old, new)


# header: the two settings and the helper
hs = replace_once(hs,
                  "  // Our publishers\n",
                  "  // Per-frame dark-region mask (dark_mask_threshold / dark_mask_dilate_px node parameters, 0 = off)\n"
                  "  int dark_mask_threshold = 0;\n"
                  "  int dark_mask_dilate_px = 12;\n"
                  "  cv::Mat dark_mask(const cv::Mat &image, const cv::Mat &base) const;\n"
                  "\n"
                  "  // Our publishers\n")
h.write_text(hs)

# source: read the parameters next to save_total_state
cs = replace_once(cs,
                  "  // Load if we should save the total state to file\n",
                  "  // Per-frame dark-region mask\n"
                  "  if (node->has_parameter(\"dark_mask_threshold\")) {\n"
                  "    node->get_parameter<int>(\"dark_mask_threshold\", dark_mask_threshold);\n"
                  "  }\n"
                  "  if (node->has_parameter(\"dark_mask_dilate_px\")) {\n"
                  "    node->get_parameter<int>(\"dark_mask_dilate_px\", dark_mask_dilate_px);\n"
                  "  }\n"
                  "  if (dark_mask_threshold > 0) {\n"
                  "    PRINT_INFO(\"dark-region mask on: pixels <= %d masked, grown by %d px\\n\", dark_mask_threshold, dark_mask_dilate_px);\n"
                  "  }\n"
                  "\n"
                  "  // Load if we should save the total state to file\n")
# the helper, before publish_state
cs = replace_once(cs,
                  "void ROS2Visualizer::publish_state() {\n",
                  "cv::Mat ROS2Visualizer::dark_mask(const cv::Mat &image, const cv::Mat &base) const {\n"
                  "  cv::Mat mask;\n"
                  "  cv::threshold(image, mask, dark_mask_threshold, 255, cv::THRESH_BINARY_INV);\n"
                  "  if (dark_mask_dilate_px > 0) {\n"
                  "    const int k = 2 * dark_mask_dilate_px + 1;\n"
                  "    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k)));\n"
                  "  }\n"
                  "  if (!base.empty() && base.size() == mask.size()) {\n"
                  "    cv::bitwise_or(mask, base, mask);\n"
                  "  }\n"
                  "  return mask;\n"
                  "}\n"
                  "\n"
                  "void ROS2Visualizer::publish_state() {\n")
# monocular callback
cs = replace_once(cs,
                  "  if (_app->get_params().use_mask) {\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id0));\n"
                  "  } else {\n"
                  "    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));\n"
                  "  }\n",
                  "  if (dark_mask_threshold > 0) {\n"
                  "    message.masks.push_back(dark_mask(cv_ptr->image, _app->get_params().use_mask ? _app->get_params().masks.at(cam_id0) : cv::Mat()));\n"
                  "  } else if (_app->get_params().use_mask) {\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id0));\n"
                  "  } else {\n"
                  "    message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));\n"
                  "  }\n")
# stereo callback
cs = replace_once(cs,
                  "  if (_app->get_params().use_mask) {\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id0));\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id1));\n"
                  "  } else {\n",
                  "  if (dark_mask_threshold > 0) {\n"
                  "    const bool st = _app->get_params().use_mask;\n"
                  "    message.masks.push_back(dark_mask(cv_ptr0->image, st ? _app->get_params().masks.at(cam_id0) : cv::Mat()));\n"
                  "    message.masks.push_back(dark_mask(cv_ptr1->image, st ? _app->get_params().masks.at(cam_id1) : cv::Mat()));\n"
                  "  } else if (_app->get_params().use_mask) {\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id0));\n"
                  "    message.masks.push_back(_app->get_params().masks.at(cam_id1));\n"
                  "  } else {\n")
c.write_text(cs)
print("patched", h, c)
