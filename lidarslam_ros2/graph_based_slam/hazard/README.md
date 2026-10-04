# Backend hazard mapping

The hazard implementation lives in this directory and compiles directly into
`graph_based_slam_component`. It is not a separate ROS node or library target.
The backend owns one worker thread with a bounded fresh-cloud batch and complete-graph
snapshots. Nav2 retains planning, path following, recoveries, and velocity
smoothing. Its global and local costmaps import the same complete `/map`
through StaticLayer, without obstacle, voxel, or inflation layers.

## Operator workflow

Rebuild the changed packages in the SLAM workspace before running the existing
two-terminal workflow. From the workspace, with the ROS/dependency overlay
sourced:

```bash
colcon build --symlink-install --packages-select graph_based_slam togo_navigation lidarslam --packages-ignore gtsam
source install/setup.bash
```

Then run `scripts/togo/run_live_seyond_dlio_slam.sh` and
`scripts/togo/run_nav2_only.sh` as before. Nav2's script explicitly loads the
source Nav2 configuration. Hazard parameters are all in the
`graph_based_slam.ros__parameters` section of
`lidarslam/param/seyond_dlio_graph.yaml`.

## Terrain processing

1. Match each deskewed odom-frame cloud to its acquisition pose using buffered
   odometry (interpolation, with bounded endpoint skew).
2. Filter height along gravity relative to that acquisition body origin,
   and limit horizontal range. Body pitch does not rotate the height band
   or become the terrain's slope reference. DLIO's odom Z must be gravity aligned.
3. Collect up to `hazard/batch_clouds` fresh scans, or process a partial batch
   after `hazard/update_period_sec` wall seconds. Separate returns above trusted
   retained ground, or a supported surrounding ground plane, into temporary
   obstacle evidence. Exclude those returns from terrain fusion. Combine the
   remaining samples into a low height percentile per observed cell (an interpolated median for
   fewer than five points). Estimate height uncertainty from acquisition range,
   point spread, and bounded point support. Consistent measurements fuse with
   cached heights according to that uncertainty; unobserved terrain is retained.
   Large disagreements remain separate candidates until enough reliable batches
   confirm the new height, rather than averaging different surfaces into a ramp.
4. Refit planes only in changed cells and their slope-support neighborhoods,
   using cached terrain along batch edges and uncertainty-weighted robust fits.
   Check spatial coverage in both axes and plane residuals, then measure normals
   against corrected gravity. Valid slopes at or above the limit become hazards.
   Uncertain, poorly distributed, or inconsistent support stays unknown when
   there was no previous hazard. A previous hazard remains lethal until repeated
   reliable, clearly safe observations cover its neighborhood.
5. Combine unsafe terrain and occupied temporary-obstacle columns into one raw
   hazard grid. Expand hazards for physical rover clearance, then add the hard
   and soft margins below. Raw missing/inconclusive terrain remains unknown;
   `hazard/unknown_cost` controls its published planning cost. The implementation
   does not infer holes from missing returns.

Stationary scans and rotation update terrain without waiting for a new
distance-based submap. DLIO already publishes `/dlio/deskewed` while stationary
in this workflow (`map/waitUntilMove: false`); there is no frontend timer change.
The input queue retains at most the configured batch size, drops older queued
scans on overload, and rejects repeated/out-of-order acquisition stamps.

The persistent grid uses dense neighbor lookups for plane fitting. Inflation
updates cover changed classifications plus the outer halo, with enough surrounding
hazard sources to preserve maximum-cost overlaps and remove obsolete halos.
Grid expansion recalculates inflation over the complete grid.

Terrain observations and temporary obstacle voxels retain graph-anchor coordinates
and acquisition ordering. Graph corrections invalidate readiness and rebuild the map from corrected
historical sources and retained height estimates, uncertainty, candidates, and
hazard-clearing history, plus temporary obstacle evidence; replayed clouds do not
count as new confirmations or occupancy votes.
Removed graph terrain does not linger at its old position. Historical clouds are loaded only for
initialization, corrections, or height/range/resolution/filter changes. Ordinary
graph appends and fresh-cloud batches do not refit historical terrain. PCD files
remain backend-owned; the worker joins before their cleanup.

Terrain classification still uses slope. Plane support scale and lower terrain
percentile can smooth small rocks and sharp steps; above-ground obstacle
separation does not provide a step-height or drop-off detector. Those cases need
recorded-cloud validation before adding a discontinuity rule. Confirmed safe
observations can remove old slope hazards and their inflation. Missing returns
alone do not erase terrain or establish free space.

## Temporary obstacles and bounded confidence

Temporary blockage is stored separately from ground in sparse 3D voxels. Each
voxel uses the grid resolution in X/Y and `obstacle_voxel_height_m` in Z. Returns
between `obstacle_min_height_m` and `obstacle_max_height_m` above a trusted ground
estimate contribute occupied evidence. Returns above the ground cutoff are
excluded from terrain fusion, so repeated observations of a person do not
replace the retained ground with the person's surface. Ground prediction requires
adequate surrounding coverage and a consistent plane. Without a ground reference,
unsupported surfaces above the acquisition body origin cannot establish safe
terrain; they stay unknown. This conservative bootstrap can leave elevated terrain
unknown until ground support becomes available.

Occupied and observed-free evidence add positive or negative log-odds, with a
bounded maximum. Each fresh batch contributes at most one update per voxel,
regardless of point count. The default occupied update is 0.85, the free update
is 0.25, marking occurs at 0.70, clearing occurs at 0.30, and evidence saturates
at 0.90. These values are tuning proxies, not calibrated probabilities. From a
saturated voxel, three qualified free batches produce approximately 75%, 50%,
then the 30% clearing bound, removing it. An obstacle present for hours has the same bounded clearing
effort as one that just reached saturation. A voxel remains lethal until its
clearing threshold is reached; decreasing confidence does not lower its cost
from 100 while it is still occupied. Fully cleared voxels are discarded.

Clearing requires positive visibility evidence through the actual occupied 3D
voxels. Rays start at the LiDAR origin from the body-to-LiDAR TF, then traverse
voxels only before the return endpoint, with an endpoint margin. A ray passing
above, below, or beside an occupied voxel does not clear it. The default support
is three occupied returns to mark, or three clearing ray crossings to earn a
free update. Any occupied return in the batch vetoes clearing for its voxel,
even when too sparse to establish a new obstacle. A ground return can clear
its own ground-height voxel, but cannot clear unobserved space above it. A person
moving away therefore clears only the previously occupied volumes actually seen
through by later rays. Occluded or unobserved volumes remain occupied.

The deskewed frontend cloud does not retain each return's acquisition origin.
This version allows a common ray origin only when odometry fully brackets
100 ms on either side of the cloud header and every pose in that interval stays
within 1 cm and 0.5 degrees of the acquisition pose. Missing TF or appreciable
motion disables ray clearing while terrain updates and obstacle marking continue.
Up to 16 deferred clouds wait for future odometry; once validated they contribute
clearing only, without replaying ground updates or marking old returns again.
Clouds that remain unsuitable, become stale, or are dropped provide no clearing
evidence. The source LiDAR frame is `hazard/lidar_frame`; changing it requires a
restart. Per-return origins would be needed to relax this conservative motion gate.

Ray tracing is evenly sampled and bounded by `hazard/max_rays` per worker job.
Skipping rays preserves occupancy rather than inventing free space. Reaching the
obstacle voxel allocation budget fails the job and makes readiness false. Graph
corrections reproject retained voxels through full SE(3) while preserving their
evidence, and removed graph sources remove their associated state. Historical
clouds and duplicate acquisition stamps do not accumulate occupancy evidence.
There is no time-only decay.

The final raw grid is lethal when terrain is unsafe or any occupied voxel lies
in that X/Y column. Clearing blockage exposes the underlying terrain state; it
does not make unknown ground safe or clear an unsafe slope. The backend publishes
one combined map and applies inflation once. At the default `unknown_cost: -1`,
the existing `allow_unknown: true` policy permits routes through unknown terrain. An obstacle over
entirely unmapped ground can initially remain unknown until a supported ground
reference is available; this version does not guarantee avoidance in that case.

## Main settings

Code defaults are listed below; the active operator settings are in the YAML.

| Backend parameter | Code default | Meaning |
| --- | --- | --- |
| `hazard/input_min_height_m` | -3.0 | Lowest cloud input relative to acquisition body origin, along gravity |
| `hazard/input_max_height_m` | 2.0 | Highest cloud input along gravity |
| `hazard/max_range_m` | 12.0 | Horizontal input range |
| `hazard/resolution_m` | 0.05 | Authoritative grid cell size |
| `hazard/unknown_cost` | -1 | Planning cost of unknown cells: -1 keeps the unknown sentinel; 0 free, 1–99 soft, 100 blocked. Raw evidence stays unknown. |
| `hazard/slope_radius_m` | 0.4 | Neighborhood radius for terrain-plane support |
| `hazard/max_slope_deg` | 25.0 | Lethal terrain slope threshold |
| `hazard/lethal_inflation_m` | 0.1 | Hard margin beyond physical rover clearance |
| `hazard/soft_inflation_m` | 0.3 | Total outer margin beyond physical rover clearance |
| `hazard/footprint` | rectangle ±0.55 × ±0.38 | Physical body polygon, conservatively enclosed by a circle |
| `hazard/batch_clouds` | 3 | Fresh scans per batch and maximum queued scan count (1–16) |
| `hazard/update_period_sec` | 0.2 | Maximum wall-time wait for a partial batch; not a sleep after each job |
| `hazard/max_input_age_sec` | 1.0 | Maximum completed-map source age in ROS time |
| `hazard/max_cells` | 4,000,000 | Grid allocation limit; exceeding it makes health false |
| `hazard/height_noise_m` | 0.01 | Base height uncertainty; increases with range and spread, decreases with bounded point support |
| `hazard/min_plane_coverage` | 0.20 | Minimum minor-axis spatial standard deviation divided by slope radius; rejects clustered/line support |
| `hazard/max_plane_error_m` | 0.05 | Plane RMS/residual limit; at least 75% of supporting cells must be within this error |
| `hazard/clear_confirmations` | 3 | Reliable batches to confirm a conflicting height and, separately, to clear a hazard |
| `hazard/clear_slope_margin_deg` | 3.0 | Clearing threshold is maximum slope minus this margin, including the slope uncertainty bound |
| `hazard/obstacle_min_height_m` | 0.15 | Lowest obstacle height above supported ground, along corrected gravity |
| `hazard/obstacle_max_height_m` | 1.8 | Highest obstacle height above ground to mark |
| `hazard/lidar_frame` | `lidar3d_0_laser` | LiDAR TF frame for ray origins; requires restart to change |
| `hazard/obstacle_voxel_height_m` | 0.10 | Vertical size of temporary obstacle voxels |
| `hazard/obstacle_min_points` | 3 | Minimum occupied returns or clearing crossings per voxel and batch |
| `hazard/obstacle_hit_probability` | 0.85 | Strength of one qualified occupied update in log-odds |
| `hazard/obstacle_miss_probability` | 0.25 | Strength of one qualified observed-free update in log-odds |
| `hazard/obstacle_mark_probability` | 0.70 | Evidence threshold for marking a voxel lethal |
| `hazard/obstacle_clear_probability` | 0.30 | Evidence threshold for clearing a previously lethal voxel |
| `hazard/obstacle_max_probability` | 0.90 | Evidence cap; prevents unlimited clearing debt |
| `hazard/max_rays` | 3,000 | Maximum clearing rays traced per worker job |
| `hazard/max_obstacle_voxels` | 200,000 | Sparse obstacle allocation limit; exceeding it makes health false |

Advanced support settings remain next to these parameters:
`min_points_per_cell`, `min_plane_neighbors`, and `terrain_percentile`.
Defaults are a starting point for this simulation, not validated rover limits.
Runtime hazard changes are validated atomically and rebuild the map;
`hazard/enabled` and `hazard/lidar_frame` require a restart.

Clearing requires a reliable observation of the hazard cell, enough fresh support
cells spread across the neighborhood, and at least 60% fresh support among its
cached neighbors. A batch earns at most one confirmation; duplicate timestamps,
graph replay, missing returns, and isolated cells earn none. Unsafe qualified
fits reset clearing confirmations. A terrain hazard clears only when the slope
plus its uncertainty bound is at most `max_slope_deg - clear_slope_margin_deg`.
A large height change may first need three batches to confirm the new surface,
followed by clearing confirmations. This intentionally
makes clearing slower than classifying an already established steep surface.

Uncertainty is a conservative tuning proxy, not calibrated LiDAR/pose covariance
or a physical probability of safety. Consecutive acquisition batches can share
the same viewpoint. Validate these defaults on recorded near/far terrain before
loosening clearing or interpreting the confidence as a measured probability.
There is no time-only decay that erases old hazards. Height/range/resolution/
percentile/minimum-point/height-noise and obstacle height/voxel-size changes rebuild
terrain from source clouds; geometry corrections and other setting changes
preserve retained history.

## Inflation and Nav2 geometry

The backend grid describes clearance for the **rover center**. It first adds
the circumscribed physical footprint radius, then the configured margins.
The half cell diagonal is included conservatively.

After that physical clearance, distances through 0.1 m have cost 100.
Between 0.1 and 0.3 m, known cells decrease linearly through 99 to 0.
Changing either radius moves the entire gradient; no intermediate distances
are hardcoded. Overlapping halos use the greatest cost.
The outer radius is total extent, so these defaults give a 0.2 m soft band.

Nav2 uses point collision geometry because physical clearance is already
included in this grid. Adding its footprint or InflationLayer again would
double count that clearance. Both Nav2 costmaps use the map frame and adopt
the complete backend grid geometry. StaticLayer rescales occupancy costs to
Nav2's internal cost range while preserving the gradient and lethal threshold.
RPP's exponential-inflation cost speed regulation is disabled; collision,
curvature, approach, and acceleration controls remain.

The existing simulation policy `allow_unknown: true` is retained in the planner
and goal relay. With `hazard/unknown_cost: -1`, unknown is represented explicitly
and this policy allows routes through it. A numeric unknown cost instead assigns
those cells the chosen planning price: 0 is free, 1–99 is traversable with cost,
and 100 is blocked, including for goal acceptance. This replaces the unknown
sentinel only in `/map`; `/hazard/raw`, ground evidence, and readiness still use
the actual classification. Unknown costs never seed obstacle inflation. Hard
exclusions always win, and numeric unknown costs combine with soft halos using
the maximum. Changing this parameter rebuilds costs without replaying evidence
or refiltering historical clouds. It applies only within existing map bounds.
For example, set `hazard/unknown_cost: 50` in the backend YAML, or run
`ros2 param set /graph_based_slam hazard/unknown_cost 50` for a temporary change.
The planner/relay `allow_unknown` policy applies to sentinel -1 cells, not to
numeric planning costs. Blocking unknown can disconnect sparse-cloud routes.

## Diagnostics and command freshness

| Topic | Contents |
| --- | --- |
| `/map` | Final authoritative planning costs (-1 unknown sentinel or configured unknown cost, 0 free, 1–99 soft, 100 lethal) |
| `/hazard/raw` | Combined terrain/temporary-obstacle classifications before footprint clearance/inflation |
| `/hazard/slope` | Plane slope in degrees in PointCloud2 intensity |
| `/hazard/obstacles` | Temporary obstacle voxel centers; PointCloud2 intensity is bounded confidence on a 0–100 scale |
| `/hazard/normalized_cloud` | Height/range filtered current cloud in corrected map coordinates |
| `/hazard/diagnostics` | Revisions, batch/dropped scan counts, terrain changes/refits, occupancy changes, traced/skipped rays, deferred clearing, map size, source age, and total worker milliseconds |
| `/hazard/ready` | Fresh volatile heartbeat; false on stale inputs, failed jobs, or corrections awaiting a grid |
| `/hazard/status` | Readiness reason: ready, waiting for first map, published map stale, configuration/graph correction pending, worker error, or stopping |

The existing goal relay forwards the requested waypoint unchanged when valid.
If the backend or Nav2 action server is temporarily unavailable, it keeps the
latest waypoint for up to `goal_wait_timeout_sec` (5 seconds in the Nav2 YAML).
A newer request replaces a pending one. Once readiness recovers, the goal is
validated against the current map; lethal and out-of-bounds goals are rejected.
Allowing unknown goals applies only within that map's bounds. Expired requests
are discarded and never run after a later recovery.
Nav2 commands go through `/nav2/cmd_vel`, the velocity smoother,
`/nav2/cmd_vel_smoothed`, and the relay to
`/platform_velocity_controller/cmd_vel`. The relay publishes zero commands
when health heartbeats or commands expire, using a wall-time watchdog.

Readiness describes the last completed map and is refreshed every 200 ms
independently of the mapping job. A brief odometry association wait or an appended
submap can use that map until its source exceeds `hazard/max_input_age_sec`.
Changed graph geometry or hazard settings still invalidate readiness immediately.
Use `ros2 topic echo /hazard/status` to distinguish a stale map from pending
corrections or configuration changes; the relay separately reports a missing
heartbeat or unavailable Nav2 action server.

Enable the optional **Temporary Obstacle Confidence** display in either
`lidarslam/rviz/mapping.rviz` or `togo_navigation/rviz/rover_nav_debug.rviz` to
inspect the confidence gradient. It uses fixed intensity bounds of 0–100, so
colors remain comparable across updates. A visible confidence change does not
mean navigation cost changes gradually: confirmed occupancy stays lethal until
cleared. The displays are disabled by default.

Debug clouds/raw grids are generated when subscribed. The worker publishes
full map snapshots; there is no separate map-save pulse or map converter.
`held_hazards` counts evaluated hazards retained in that job, not every stored
hazard. `cleared_hazards` counts confirmed removals; `rejected_heights` counts
height disagreements gated before fusion (including candidates still awaiting
confirmation). `marked_obstacles` and `cleared_obstacles` count voxel occupancy
transitions in the job. `obstacle_voxels` includes retained evidence awaiting
clearance. `traced_rays` and `skipped_rays` show the tracing budget and motion
gate; `ray_origins_enabled` and `ray_origins_disabled` describe the fresh batch.
`deferred_ray_clouds`, `clearing_clouds`, and `dropped_ray_clouds` describe queued,
processed, and discarded clearing-only clouds.

Nav2 uses `PoseProgressChecker`: either 0.25 m of translation or 0.20 radians
of rotation counts as progress within ten ROS seconds. This allows initial
turns toward goals behind the rover. RPP's `FollowPath.max_angular_accel` is
4.0 rad/s²: at 20 Hz, it permits a 0.20 rad/s change from the measured yaw rate.
The previous 1.5 setting produced roughly 0.08 rad/s commands and stalled in
the Gazebo turn test. The velocity smoother retains its 0.8 rad/s speed cap
and 0.8 rad/s² acceleration setting. These driving settings are in the Nav2
YAML, separate from terrain classification and inflation.

## Validation

`test/test_hazard_mapping.cpp` covers pitch independence, steep terrain,
full graph rotation, acquisition height filtering, unknown support, adaptive
inflation, physical clearance, maximum overlap, and invalid settings.
It compares partial inflation against a complete distance transform, checks
hazard removal and old-observation rejection, and tests batching with differing
acquisition poses. Confidence regressions cover weak distant contradictions,
missing/isolated/clustered support, duplicate stamps, slope hysteresis, height
mode confirmation, and range-dependent uncertainty. `test_hazard_blockage.cpp`
covers saturation and bounded clearing, preservation of ground beneath obstacles,
height-specific visibility, occupied-return vetoes, missing and duplicate data,
and the composite map/inflation behavior. `test_hazard_worker.cpp`
exercises correction/reprojection, preservation of uncleared hazards and their
history, retention of fresh observations over older graph sources, and removed sources
in isolated domain 199.
`test_hazard_worker_blockage.cpp` checks obstacle graph reprojection, preservation
of evidence, and removal when its graph source disappears.

`test/test_hazard_nav2_integration.py` is an explicit smoke test for a sourced
graph/Nav2 overlay. It uses synthetic clouds and odometry in isolated
`ROS_DOMAIN_ID=199`; it starts the real backend and Nav2 but no hardware.
It checks terrain classification under body pitch, runtime parameter rejection,
both costmap imports, waypoint planning, forwarded controller commands,
stationary batch/timeout behavior, isolated-return hazard retention, confirmed
removal of old hazards and halos, and stopping
on stale cloud input. Run it with `--log-dir <directory>`.
