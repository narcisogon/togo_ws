---
name: togo-slam
description: Develop, diagnose, and validate this Togo rover's Seyond LiDAR and IMU SLAM, graph mapping, hazard maps, and Nav2 integration in native WSL Gazebo and Docker. Use for this repository's simulation, replay, real-sensor SLAM, TF, clock, point-cloud timing, drift, and navigation issues.
---

# Togo SLAM

Use the working simulation as the baseline, and isolate whether a failure comes
from transport, sensors, estimation, mapping, navigation, or physical motion.
Confirm the requested operating mode before choosing launchers or parameters.

## Locate the active workspace

Find the repository containing `docker-compose.yml`, `src/togo/`, and
`lidarslam_ros2/`; the desktop folder may contain another folder with the same
name. All repository paths below are relative to that root. Read applicable
`AGENTS.md` files, Git status, and `docs/LOCAL_CHANGES.md`. Inspect current source
and active launch arguments; the ledger and comments can lag implementation.

Read [references/operations.md](references/operations.md) for launching,
rebuilding, sensor wiring, or switching between simulation, replay, and hardware.
Read [references/diagnostics.md](references/diagnostics.md) for failed discovery,
TF jumps, time errors, drift, empty maps, slow navigation, or wheel slippage.

## Preserve the integration contract

- Native WSL runs Gazebo and the primary RViz. Docker `dev` runs DLIO, graph
  SLAM, hazard mapping, and Nav2. ROS is Jazzy; the Gazebo invocation uses
  version 8. Check actual installed versions when diagnosing compatibility.
- Native and container processes currently use domain 29, CycloneDDS,
  loopback discovery, and host networking. Loopback discovery requires the
  processes to share the relevant network namespace; inspect the running
  container rather than assuming the Compose file has been applied.
- Simulation consumers, including RViz goal tools, must use `/clock` and
  `use_sim_time=true`. Live hardware uses wall time. Replay uses bag time with
  a single clock source. Do not mix live simulation and bag playback blindly.
- Graph SLAM owns `map -> odom`; DLIO owns `odom -> base_link`; robot state
  publisher owns robot joints and sensor transforms below `base_link`.
  `src/togo/togo_gz/config/gazebo_controllers.yaml` keeps
  `platform_velocity_controller.ros__parameters.enable_odom_tf: false`.
  Do not restore the redundant `dlio_odometry_to_tf` publisher or add identity
  `map -> odom` alongside a live graph backend.
- Match Gazebo gravity to DLIO's `odom/gravity`. The checked-in lunar world
  currently uses magnitude 9.82 for the traction test, not lunar gravity.
  Change both to 1.62 only when the task calls for lunar gravity.
- The validated lunar world selects classic Bullet (`gz-physics-bullet-plugin`)
  and separate detailed visual/lidar and decimated collision meshes. Preserve
  the no-slip wheel wrappers unless testing an explicit alternative. A physics
  engine switch is not a generic SLAM fix.

## Make and evaluate changes

Follow the active pipeline through launcher, YAML, remappings, and node code.
Runtime overrides can supersede YAML; inspect effective parameters before
tuning. Use the existing DLIO and graph diagnostic tools where applicable.
Avoid changing estimator gates, extrinsics, gravity, friction, and costmap
thresholds together: choose the smallest intervention supported by evidence.

For diagnosis requests, report findings without implementing unrequested fixes.
For implementation requests, rebuild the relevant workspace and validate the
changed behavior when the runtime is available. Simulation tests should cover
sensor reception, clock consistency, the two dynamic TF edges, and the affected
mapping or goal behavior. Report exactly which checks ran; static inspection
does not establish that Gazebo or the rover moved correctly.

Treat direct velocity commands as motion, including over Docker; establish
that the target is simulation or that hardware motion was requested. Disable
competing command sources for a motion test and stop the command afterward.
Do not start stacks, play bags, reset maps, or kill processes just to inspect
configuration. Existing launchers include process-group cleanup; inspect that
behavior before invoking them from a shared diagnostic shell.

Append non-dependency changes to `docs/LOCAL_CHANGES.md` with date, files,
reason, validation, and permanent/temporary status. Keep build products, bags,
maps, generated terrain, and local `.env` out of source commits according to
the repository ignore rules. Publish only when the current task authorizes it.
