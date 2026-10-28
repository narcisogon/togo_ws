"""Validation of a built world (docs/DESIGN.md section 13).

Modules
  stats          deviogram / RMS slope / PSD / size-frequency statistics (numpy, block-wise)
  static_checks  run_static_checks(cfg) -> reports/static_checks.json (roughness, craters, rocks,
                 visual-vs-collision dz, GLB files, stitching, SDF URIs, world_info)
  lidar_checks   analysis of static gpu_lidar captures (tools/lidar_capture.py): edge, residuals,
                 leaks (missing returns), rock detection vs size and range
  report         reports/validation_report.md (+ figures) from all JSON results

Gazebo-side tools (system python3): tools/bench_world.py, tools/lidar_capture.py,
tools/run_robot_tests.py.
"""
