# DLIO Frontend Architecture

This package provides the local LiDAR-inertial odometry frontend for the current
Seyond rover pipeline. It combines IMU propagation with scan-to-local-map
NanoGICP registration, publishes smooth odometry and motion-corrected clouds in
the `odom` frame, and maintains the keyframe submap needed for the next scan.

DLIO is intentionally local. It does not detect global loop closures, optimize
the global trajectory, publish `map -> odom`, construct the graph-corrected map,
or choose Nav2 paths. Those responsibilities belong to `graph_based_slam` and
the downstream navigation stack.

## End-to-end frontend flow

```text
Seyond PointCloud2                   IMU
        |                             |
        |                             v
        |                    calibration and frame transform
        |                             |
        |                    timestamped circular buffer
        |                             |
        v                             v
Point-field and timing normalization
        |
        +---- native point firing time, or
        +---- configured synthetic column timing
        |
        v
Crop / dust filtering / voxelization / optional normal-space sampling
        |
        v
Continuous IMU trajectory across the scan
        |
        v
Deskewed scan at one common time
        |
        v
IMU pose prediction -> NanoGICP against the local keyframe submap
        |
        v
Quality, timing, innovation, degeneracy, and recovery policy
        |
        v
Accepted or safely bridged local pose
        |
        +---- state and bias correction
        +---- odometry, TF, path, diagnostics, deskewed cloud
        +---- keyframe decision and asynchronous local-submap refresh
        |
        v
Graph SLAM, local hazards, Nav2, and visualization consumers
```

## Runtime nodes

The package builds two ROS 2 executables:

| Node | Responsibility |
| --- | --- |
| `dlio_odom_node` | LiDAR/IMU synchronization, deskewing, registration, state estimation, keyframes, and local odometry output |
| `dlio_map_node` | Optional accumulation and saving of DLIO keyframe clouds in the `odom` frame |

The map node is not the graph-SLAM backend. Its `/dlio/map` output is an
uncorrected local visualization map. The persistent graph-corrected map is built
and saved by `graph_based_slam`.

## Frame ownership

The current stack deliberately splits local continuity from global correction:

```text
map -- graph_based_slam --> odom -- DLIO / odometry_to_tf --> base_link
```

- DLIO estimates smooth local motion and publishes poses in `odom`.
- DLIO publishes deskewed point coordinates in `odom`, not in the LiDAR or body frame.
- The graph backend consumes those local poses and clouds, then publishes the
  global `map -> odom` correction after loop closures or other graph constraints.
- Nav2 receives continuous local feedback from `/dlio/odometry` while its global
  geometry is corrected through the map/odom transform chain.

DLIO currently publishes `odom -> base_link` internally from `publishPose`. The
integrated launch also starts `odometry_to_tf` from `/dlio/odometry` for the same
parent and child frames. Unless one publisher is disabled elsewhere, this gives
the transform two authorities. Their values should normally agree because both
come from DLIO odometry, but duplicate authority can still create timing jitter
and TF warnings. The stack should ultimately select one publisher explicitly;
this documentation pass does not change that runtime behavior.

## LiDAR point timing

Deskewing requires a firing time for every point. DLIO supports timing layouts
used by Ouster, Velodyne, Hesai, and Livox point types. `getScanFromROS` detects
the available fields and normalizes their meaning for the current scan.

The simulated Seyond cloud does not always provide the native timing layout
expected by those drivers. The frontend therefore supports deterministic
synthetic timing:

- `pointcloud/syntheticTiming/enabled` enables the fallback.
- `pointcloud/syntheticTiming/scanPeriodSec` specifies the complete scan period.
- `pointcloud/syntheticTiming/reverseColumns` reverses column firing order when
  the simulator's organization requires it.
- `pointcloud/deskew/maxUniqueTimestamps` bounds integration buckets without
  removing per-point timing from the scan.

Synthetic timing is an estimate of the acquisition pattern. A wrong scan period
or reversed column order creates systematic deskew error even when LiDAR and IMU
message headers appear synchronized.

## IMU initialization and propagation

The IMU callback transforms measurements using configured intrinsics and
`base_link -> imu` extrinsics, then stores them in a timestamped circular buffer.
Between LiDAR scans, each new measurement propagates position, orientation, and
velocity so consumers receive continuous local motion.

At startup, optional calibration estimates accelerometer and gyro bias while the
robot is stationary. Gravity alignment initializes orientation from the observed
gravity direction. Motion during this interval contaminates the initial bias and
gravity estimate, so the rover should remain still until initialization finishes.

`imuMeasFromTimeRange` verifies that the buffer covers the scan interval. Timing
protection can wait briefly for scan-end coverage, diagnose stale input, reject
unsafe corrections, or drop scans according to the active configuration.

## Point-cloud preprocessing

The registration source passes through several configurable stages:

1. Remove points within the robot crop box.
2. Optionally reject low-intensity or near-range dust returns.
3. Optionally apply statistical outlier removal.
4. Voxelize the cloud at the frontend registration resolution.
5. Estimate local covariances and normals for NanoGICP.
6. Optionally apply normal-space sampling to balance ground and feature geometry.

Normal-space sampling is intentionally registration-only. The published
deskewed cloud, accepted keyframe cloud, and target submap remain dense. If
sampling leaves fewer than `min_total_points`, registration falls back to the
unsampled post-voxel cloud.

This distinction matters for hazard mapping: changing registration sampling
should affect scan-matching influence without silently deleting points from the
hazard input cloud.

## Deskewing

DLIO constructs a continuous pose trajectory across the scan by integrating IMU
measurements at the normalized point timestamps. Each point is transformed from
its acquisition pose to one common scan pose. The result removes much of the
shear and double-wall behavior caused by rover motion during acquisition.

The integrated launch can override `pointcloud/deskew` at startup. The effective
runtime parameter therefore matters more than the base YAML value:

```bash
ros2 param get /dlio_odom_node pointcloud/deskew
```

Disabling deskew is useful for controlled comparison, but normally sacrifices
registration quality during translation or rotation.

## Scan-to-submap registration

NanoGICP registers the processed current scan against a local target assembled
from selected DLIO keyframes. The IMU-propagated pose supplies the initial guess,
so GICP acts as a LiDAR correction to local inertial motion rather than an
independent global localization system.

The registration configuration controls correspondence count and distance,
iteration limits, convergence thresholds, damping, and covariance
regularization. The result also exposes quality evidence such as overlap,
fitness, inlier count, Hessian eigenvalues, condition number, and solve time.

## Registration protection and recovery

The workspace adds protection around the base scan matcher for difficult rover
conditions such as flat terrain, sparse features, fast yaw, stale IMU data, and
ambiguous repetitive geometry. The major mechanisms are:

- Correction magnitude gates for implausible translation or rotation jumps.
- Spin protection with optional IMU-prior fallback.
- LiDAR/IMU timing protection and stale-scan policy.
- Overlap, fitness, inlier, and Hessian-quality classification.
- Partial corrections for usable but degenerate registrations.
- Innovation gating relative to the IMU-predicted pose.
- Hessian-eigenvector projection that damps weak geometric directions.
- Recovery searches after a configured rejection streak.
- Velocity damping or hold behavior during extended IMU-only bridging.
- Keyframe trust gates that prevent weak matches from poisoning the target map.

Many of these mechanisms are individually configurable and some are disabled in
the base simulation preset. The diagnostics expose their effective behavior;
their existence in code does not mean every protection is currently active.

## Optional ground constraint

The frontend can fit a dominant near-ground plane and apply a confidence-gated
soft height correction. The fit checks inlier count, inlier ratio, plane tilt,
height continuity, and per-scan correction limits. This mechanism addresses
locally accumulated vertical drift; it is not a global terrain model.

The ground constraint is separate from the graph backend's DEM:

- The DLIO ground constraint modifies the local frontend state when enabled.
- The graph DEM rasterizes globally corrected map snapshots downstream.
- Hazard logic should consume corrected terrain evidence rather than treating
  the frontend plane fit as a complete traversability decision.

## Keyframes and the local submap

A new keyframe may be accepted after sufficient translation or rotation. The
rotation threshold is important for a rover that can turn substantially without
moving far: rotation-only keyframes preserve newly observed viewing directions.

The local registration target combines several keyframe selections:

- Nearest keyframes by distance.
- Convex-hull keyframes that preserve spatial coverage.
- Concave-hull keyframes that add useful nearby geometry.

Submap construction can run asynchronously, but target replacement is
synchronized with the main LiDAR transaction. A keyframe trust policy can allow
odometry publication while refusing to insert a weak registration into the
future scan-matching map.

## ROS outputs and consumers

The integrated launch remaps generic DLIO names into the `/dlio` namespace:

| Topic | Frame and meaning | Main consumers |
| --- | --- | --- |
| `/dlio/odometry` | `odom` pose of `base_link` | Graph SLAM, Nav2 controller and behavior servers, odometry-to-TF bridge |
| `/dlio/pose` | PoseStamped form of local odometry | Visualization and inspection |
| `/dlio/path_raw` | Optional built-in accumulated path | Debugging; the integrated stack normally uses the bounded simple path |
| `/dlio/path_simple` | Bounded path generated from odometry | RViz and operator visualization |
| `/dlio/deskewed` | Deskewed PointXYZI cloud transformed into `odom` | Graph SLAM, local hazard grid, Nav2 obstacle processing, RViz |
| `/dlio/keyframes` | Keyframe pose array | Frontend inspection |
| `/dlio/keyframe_cloud` | Accepted keyframe clouds in `odom` | `dlio_map_node` and debugging |
| `/dlio/map` | Accumulated local keyframe map in `odom` | Optional frontend visualization |
| `/dlio/frontend_diagnostics` | Numeric health and quality vector | Runtime monitoring and diagnosis |

Because `/dlio/deskewed` is in `odom`, a downstream range test must measure from
the robot's current pose or transform into a robot-relative target frame first.
Computing `hypot(point.x, point.y)` directly would measure distance from the
`odom` origin and become wrong as the rover drives away.

## Handoff to graph-based SLAM

The combined launch remaps:

```text
/dlio/odometry  -> graph_based_slam/odom_input
/dlio/deskewed  -> graph_based_slam/cloud_input
```

The backend sets `odom_input_cloud_in_odom_frame: true`, creates graph submaps
from the local odometry stream, and later corrects those submap poses with loop,
IMU, or GNSS constraints. Loop correction must not be fed back into DLIO's local
registration target; doing so would couple a smooth local estimator to
discontinuous global graph updates.

The ownership rule is:

- DLIO answers: "How did the rover move locally and continuously?"
- Graph SLAM answers: "Where should that local trajectory sit in the global map?"

## Hazard mapping and Nav2 boundary

The local hazard grid consumes `/dlio/deskewed` for responsive nearby obstacle
and terrain evidence. Nav2 also uses `/dlio/odometry` for controller feedback and
may consume the deskewed cloud through configured obstacle layers. These paths
remain local and low latency even before a loop closure exists.

Graph-corrected map products provide the global context. A change to DLIO
deskewing, extrinsics, scan registration, or keyframe quality can change both
local hazard evidence and the geometry later optimized by the backend. It does
not directly change hazard thresholds, costmap plugins, planners, or controllers.

## Threading and shutdown

LiDAR and IMU subscriptions use separate mutually-exclusive callback groups and
a multi-threaded executor. This allows high-rate IMU propagation while the LiDAR
callback performs a longer registration transaction.

The LiDAR callback launches or coordinates auxiliary work for publication,
metrics, debug output, keyframes, and submap construction. A completion guard
always resets pipeline-stage bookkeeping and wakes paused submap work, including
early returns. Shutdown marks the node as stopping, wakes IMU and submap waits,
joins owned threads, and only then releases shared state.

## Diagnostics

`/dlio/frontend_diagnostics` reports initialization, pipeline, filtering,
registration, correction, timing, and protection values. The node also keeps a
bounded structured event history and can append it to a configured file.

Useful live checks are:

```bash
ros2 topic hz /dlio/odometry
ros2 topic hz /dlio/deskewed
ros2 topic echo /dlio/frontend_diagnostics
ros2 param get /dlio_odom_node pointcloud/deskew
ros2 param get /dlio_odom_node frames/odom
```

When investigating a failure, establish this order first:

1. LiDAR and IMU callbacks are both advancing.
2. Timestamps share the selected simulation, replay, or wall-clock domain.
3. IMU data covers the LiDAR scan interval.
4. Per-point timing and scan period match the sensor organization.
5. Extrinsic transforms match the physical or simulated sensor mount.
6. GICP quality and correction gates explain whether a scan was accepted,
   partially applied, bridged from IMU, recovered, or rejected.

## Parameter sources and precedence

The current integrated pipeline loads multiple parameter sources. Later launch
overrides win over earlier YAML values. The main sources are:

1. Package defaults under `cfg/`.
2. The combined simulation configuration at
   `../lidarslam/param/seyond_dlio_graph.yaml`.
3. Optional replay or live-real overrides.
4. Launch arguments such as the effective deskew selection.

Changing YAML does not require recompilation, but startup-only parameters require
the node to be restarted. Always query the live node when confirming an experiment.

## Source layout

| File | Responsibility |
| --- | --- |
| `include/dlio/dlio.h` | Shared point type and sensor timing layouts |
| `include/dlio/odom.h` | Frontend node contract and owned state |
| `src/dlio/odom.cc` | Complete LiDAR/IMU, registration, protection, state, keyframe, and submap pipeline |
| `src/dlio/odom_node.cc` | Multi-threaded ROS executable entry point |
| `include/dlio/map.h` and `src/dlio/map.cc` | Optional odom-frame keyframe map accumulator |
| `src/dlio/map_node.cc` | Map executable entry point |
| `include/nano_gicp/` and `src/nano_gicp/` | Scan-registration library used by the frontend |
| `cfg/` | Standalone DLIO defaults and sensor presets |

The NanoGICP library is documented through its types and algorithm-level source.
The frontend documentation focuses on how DLIO configures, calls, validates, and
uses that registration result.

## Active integrated configuration

The current rover pipeline is assembled by:

- [`../lidarslam/launch/seyond_dlio_slam.launch.py`](../lidarslam/launch/seyond_dlio_slam.launch.py)
- [`../lidarslam/param/seyond_dlio_graph.yaml`](../lidarslam/param/seyond_dlio_graph.yaml)
- [`../graph_based_slam/README.md`](../graph_based_slam/README.md)
- [`../togo_navigation/launch/rover_nav2.launch.py`](../togo_navigation/launch/rover_nav2.launch.py)

After changing C++ frontend code, rebuild the affected package in the ROS
environment:

```bash
colcon build --packages-select direct_lidar_inertial_odometry --symlink-install
```

Parameter-only changes need a node restart, not a C++ rebuild.
