# Graph-Based SLAM Backend Architecture

This package is the mapping backend between a LiDAR odometry frontend and the
corrected map products used by visualization, terrain processing, map saving,
and downstream navigation systems. It creates submaps when direct odometry and
cloud input mode is enabled, finds loop closures, constructs a pose graph,
optimizes that graph with GTSAM, and rebuilds map outputs from corrected submap
poses.

The backend does not replace the LiDAR odometry frontend, run a Nav2 planner, or
decide whether terrain is hazardous. It produces corrected geometry, transforms,
timed map metadata, and DEM layers that those downstream systems can consume.

## End-to-end data flow

```text
LiDAR packets + IMU
        |
        v
DLIO / selected odometry frontend
        |
        |  odometry + deskewed cloud
        v
Submap ingestion and storage
        |
        +---------------------> loop candidate generation
        |                              |
        |                              v
        |                     registration and validation
        |                              |
        v                              v
Adjacent / IMU / GNSS factors + accepted loop factors
        |
        v
Backend-neutral PoseGraphProblem
        |
        +---- gtsam_isam2: persistent incremental update
        |
        +---- gtsam: complete batch solve
        |
        v
Optimized submap poses
        |
        +---- corrected point-cloud products
        +---- map -> odom correction
        +---- modified path and submap array
        +---- DEM snapshot worker
        +---- saved map and pose-graph artifacts
        |
        v
Downstream visualization, hazard processing, and Nav2 integration
```

## Main component responsibilities

`GraphBasedSlamComponent` owns the ROS interfaces and backend lifecycle. Its
work is split across several implementation files so each part has a focused
responsibility:

| File | Responsibility |
| --- | --- |
| `graph_based_slam_component.cpp` | Parameters, ROS interfaces, backend construction, and worker lifetime |
| `graph_based_slam_inputs.cpp` | Direct odometry/cloud ingestion, submap creation, IMU, and GNSS input handling |
| `graph_based_slam_loop_closure.cpp` | Candidate generation, descriptor lookup, registration, validation, and loop-edge creation |
| `graph_based_slam_optimization.cpp` | Constraint assembly, optimizer dispatch, corrected map reconstruction, TF updates, and diagnostics |
| `graph_based_slam_workers.cpp` | Search, periodic publication, map-save scheduling, and synchronized graph snapshots |
| `graph_based_slam_map.cpp` | PCD cache lifecycle and persistent grid-divided map output |
| `graph_based_slam_dem.cpp` | Background DEM raster generation, atomic output, preview, and status reporting |
| `pose_graph_optimizer*.cpp` | Backend-neutral dispatch, batch GTSAM, and persistent GTSAM iSAM2 |

The public optimizer contract is declared in
`include/graph_based_slam/pose_graph_optimizer.hpp`. The similarly named header
under `src/` is private and only connects the dispatcher to its implementations.

Supporting runtime algorithms are kept in focused headers and one verifier
implementation:

| File | Responsibility |
| --- | --- |
| `adjacent_edge_auto_scale.hpp` | NIS-based adaptation of adjacent-edge information weights |
| `gnss_weighting.hpp` | GNSS covariance, RTK-like quality weighting, and timestamp fallback |
| `scan_context.hpp` | Polar Scan Context descriptors, yaw alignment, and database lookup |
| `submap_bev_descriptor.hpp` | Occupancy/density/height BEV descriptors and yaw search |
| `bev_mutual_visibility.hpp` | Partial-FOV-aware comparison over mutually observed BEV cells |
| `solid_descriptor.hpp` | SOLiD-style descriptor construction, similarity, and yaw estimation |
| `triangle_descriptor.hpp` | BEV, surface, and 3D-edge keypoints plus triangle construction |
| `triangle_descriptor_database.hpp` | Triangle hashing, voting, consensus verification, and pose hypotheses |
| `three_d_bbs_loop_verifier.*` | Bounded 3D branch-and-bound pose verification |
| `dynamic_object_filter.hpp` | Temporal voxel consistency filtering during corrected-map assembly |
| `loop_edge_robustifier.hpp` | Robust-loss selection and loop-edge residual downweighting |

These descriptor and filter files are header-only by design. They are active
backend implementations, not merely declarations, so their functions carry the
same file and function summaries as the component source files.

## Input modes

The component supports two ways to receive submaps:

1. `map_array` mode accepts a `lidarslam_msgs/msg/MapArray` produced elsewhere.
2. Direct input mode subscribes to `odom_input` and `cloud_input`, then creates a
   new submap after the configured travel threshold.

The Seyond/DLIO pipeline uses direct input mode. Its launch file remaps DLIO
odometry and the deskewed cloud into these generic backend topic names. Keeping
the backend names generic allows a different frontend to be connected without
changing pose-graph code.

## Pose graph construction

Every optimization request is represented as a complete `PoseGraphProblem`.
It contains initial poses and the current set of constraints. It is a snapshot,
not an incremental delta.

The graph can contain:

- Adjacent constraints derived from relative frontend submap poses. Multiple
  recent neighbors can be connected using `num_adjacent_pose_cnstraints`.
- Loop constraints produced by candidate search, geometric registration, and
  acceptance gates. These normally use a robust loss so one questionable loop
  cannot dominate the full trajectory.
- IMU rotation constraints when backend IMU preintegration is enabled.
- GNSS absolute-position constraints when usable fixes pass timing, covariance,
  and residual checks.
- A tight prior on the configured fixed pose, normally pose zero, to remove the
  graph's global gauge freedom.

Translation and rotation precision are stored as separate 3x3 information
matrices. This prevents GTSAM's rotation-first tangent ordering from being
mistaken for translation-first application data. Information is precision, the
inverse of covariance: a larger value means the optimizer trusts that dimension
more strongly.

## Optimizer modes

The `optimizer_backend` parameter selects one of two GTSAM modes:

| Value | State between calls | Intended use |
| --- | --- | --- |
| `gtsam_isam2` | Preserved | Normal live mapping |
| `gtsam` | Discarded after each solve | Batch comparison, debugging, and reference solves |

`gtsam_isam2` is the default. It preserves the Bayes tree, factors, and current
estimate. Ordinary new poses and adjacent factors are appended with one update.
New loop, IMU, or GNSS factors may receive additional empty iSAM2 updates so a
larger nonlinear correction can relinearize and propagate.

Although the caller always supplies a complete snapshot, iSAM2 compares stable
factor identities and values against its retained state. It performs a safe full
rebuild when any of these conditions occurs:

- The optimizer has not been initialized.
- The pose count decreases.
- The fixed-pose index changes.
- The fixed pose itself changes.
- A historical relative factor is removed or modified.
- A historical absolute-position factor is removed or modified.

The `optimizer_state_rebuilt` and `optimizer_rebuild_reason` diagnostic fields
make this behavior observable.

To use batch mode, set the active parameter and restart the graph node:

```yaml
optimizer_backend: gtsam
```

Runtime `ros2 param set` is not a supported backend switch because the component
constructs and owns the optimizer session during its running lifecycle.

## Loop-closure pipeline

Loop closure is separate from graph optimization. Candidate sources can include
distance search, Scan Context, bird's-eye-view descriptors, SOLiD descriptors,
triangle descriptors, and optional 3D branch-and-bound verification. Enabled
sources propose candidate submap pairs; the configured registration method then
estimates a relative transform and acceptance gates reject implausible results.

An accepted loop becomes a robust relative-pose factor. Only at that point does
the optimizer use it. Changing from batch GTSAM to iSAM2 does not change loop
detection or registration; it changes how the accepted factor is incorporated
into the existing graph solution.

## Corrected map assembly

After optimization, every submap cloud is transformed by its corrected pose.
The backend caches assembled contributions and can append only new submaps while
earlier corrected poses remain within configured translation and rotation
tolerances. A loop correction or meaningful historical pose movement forces the
affected map geometry to be rebuilt.

Periodic publication has a cheap heartbeat path. If the graph and cached map are
unchanged, the backend republishes cached messages with a fresh timestamp without
running optimization, loading PCDs, transforming clouds, or voxelizing again.

Primary ROS outputs include:

| Output | Meaning |
| --- | --- |
| `modified_map` | Corrected ordinary PointXYZI cloud for compatible consumers |
| `modified_map_timed` | Corrected cloud with per-point source time and submap index metadata |
| `modified_map_array` | Corrected submap poses and clouds |
| `modified_path` | Corrected backend trajectory |
| `submap_created` | Event describing each newly appended submap |
| `map -> odom` TF | Correction that aligns continuous frontend odometry with the optimized map frame |
| `lunar_dem/preview` | Downsampled terrain-height preview cloud |
| `lunar_dem/status` | DEM worker state and completion information |
| `loop_diagnostics` | Loop candidate and acceptance diagnostics |
| `backend_timing_diagnostics` | Search, optimization, map assembly, save, and worker timing data |

## DEM, hazard maps, and Nav2 boundary

The DEM worker receives corrected map snapshots through a latest-snapshot-wins
queue. It rasterizes measured terrain and surface height, interpolates only
small supported holes, writes uncertainty and observation layers, and optionally
publishes a preview. It runs independently so raster generation does not block
pose-graph updates or map publication.

The graph backend does not label cells as safe or hazardous. Downstream hazard
processing can use:

- Corrected XYZ geometry from `modified_map`.
- Source age and submap identity from `modified_map_timed`.
- Terrain, surface, variance, observation-count, and validity information from
  the saved DEM layers.

Nav2 remains downstream of this package. The relevant backend contracts are the
stable `map` frame, the `map -> odom` correction, and whichever map or hazard
representation the navigation configuration consumes. The timed cloud is kept
separate from the ordinary PointXYZI cloud so existing displays and navigation
adapters are not forced to understand custom fields.

An optimizer change can move corrected geometry and therefore indirectly change
the terrain or hazards computed from it. It does not change DEM thresholds,
hazard policy, costmap plugins, controller behavior, or planner behavior.

## Threading and synchronization

The backend uses dedicated workers rather than placing expensive operations in
ROS subscription callbacks:

- The search worker processes graph updates and loop-search work.
- The publish worker handles periodic output heartbeats and map-save requests.
- The DEM worker processes the newest corrected snapshot when DEM output is enabled.

Graph input and loop-edge snapshots are protected separately from corrected-map
publication state. `doPoseAdjustment` serializes optimizer access and corrected
map reconstruction, which is required because the iSAM2 session is deliberately
stateful. Shutdown signals workers, joins them, and only then releases cache and
optimizer resources.

## PCD cache lifecycle

When `use_pcd_cache` is enabled, submap clouds are stored beneath the configured
cache root instead of being retained indefinitely in process memory. The active
Seyond configuration uses `/dev/shm`, which is RAM-backed and therefore fast but
finite.

Each graph process creates and locks a private `run_*` child directory. A clean
shutdown removes that run directory. On startup, the node removes only abandoned
run directories whose lock can be acquired; it never clears another live node's
cache. The Docker Compose configuration increases shared memory beyond Docker's
small default because a full mapping run can store many compressed submaps.

The PCD cache is temporary working state. Persistent map outputs belong under
`map_save_dir`.

## Persistent outputs

An explicit `map_save` request writes the current corrected products beneath
`map_save_dir`, including:

- `pose_graph.g2o`: vertices and factors serialized by GTSAM in the interoperable
  g2o text format. The extension does not mean the g2o optimizer is installed or used.
- `map.pcd`: one compressed corrected point-cloud map.
- `pointcloud_map/`: grid-divided PCD files and metadata for map tooling.
- `map_projector_info.yaml`: projector metadata for point-cloud map consumers.
- `lunar_dem/`: DEM rasters and metadata when the DEM worker is enabled.

The graph path is currently derived as `map_save_dir/pose_graph.g2o`.
`save_pose_graph_path` entries in older configurations are legacy settings and
are not read by the current graph component.

## Active Seyond/DLIO configuration

The active combined configuration is
[`../lidarslam/param/seyond_dlio_graph.yaml`](../lidarslam/param/seyond_dlio_graph.yaml).
The active launch integration is
[`../lidarslam/launch/seyond_dlio_slam.launch.py`](../lidarslam/launch/seyond_dlio_slam.launch.py).

Useful checks while the system is running:

```bash
ros2 param get /graph_based_slam optimizer_backend
ros2 topic echo /backend_timing_diagnostics --once --field data
ros2 topic echo /loop_diagnostics --once --field data
```

The normal live result should report `gtsam_isam2`. A pose adjustment with only
new adjacent data should normally show added variables and factors without a
state rebuild. A newly accepted loop should show a new factor and more than one
iSAM2 update iteration.

## Optimizer validation

`test/test_pose_graph_optimizer.cpp` verifies the backend contract, including:

- Backend-name parsing and canonical names.
- Agreement between batch GTSAM and iSAM2.
- Correct translation and rotation information ordering.
- Pose-graph serialization from both modes.
- Append-only iSAM2 updates.
- Incremental loop-factor application.
- Safe rebuilds after historical factor changes.

Run the focused test inside the built ROS environment with:

```bash
colcon test --packages-select graph_based_slam \
  --ctest-args -R test_pose_graph_optimizer --output-on-failure
colcon test-result --verbose
```
