# Evidence-first diagnostics

Use bounded sampling, normally 5-10 seconds for topic streams. Check the source
and consumer sides separately. Low `ros2 topic hz` in wall time can reflect a
low Gazebo real-time factor; compare header intervals before diagnosing dropped
scans. Do not leave diagnostic streams or path publishers running indefinitely.

## Discovery and clock

From WSL host, inspect the actual network mode:

```bash
docker inspect --format '{{.HostConfig.NetworkMode}}' "$(docker compose ps -q dev)"
```

In each relevant ROS environment, inspect domain, RMW, discovery range,
CycloneDDS config, and node/topic discovery. Source the appropriate install
before using the ROS CLI. If the CLI graph appears stale after a domain/RMW
change, investigate its daemon before editing transport configuration.

Useful read-only checks, after selecting the correct ROS environment:

```bash
ros2 topic info /clock -v
timeout 8s ros2 topic hz /husky/sensors/seyond/points
timeout 8s ros2 topic hz /husky/sensors/imu_0/data_raw
ros2 node list
ros2 param get /dlio_odom_node use_sim_time
ros2 param get /graph_based_slam use_sim_time
ros2 param get /bt_navigator use_sim_time
ros2 param get /rviz2 use_sim_time
```

Node names/namespaces and multiple RViz instances can differ; use discovered
names. Epoch timestamps around billions of seconds mixed with simulation
timestamps around tens of seconds indicate a clock mismatch. Fix the clock at
the producing node, including RViz goal tools, before increasing TF tolerance.
The native checkout RViz launch currently lacks a `use_sim_time` parameter,
despite the WSL launcher advertising a simulation-time override.

## TF jumps and drift

```bash
timeout 8s ros2 run tf2_ros tf2_echo map odom
timeout 8s ros2 run tf2_ros tf2_echo odom base_link
timeout 8s ros2 run tf2_ros tf2_echo base_link lidar3d_0_laser
ros2 topic info /tf -v
ros2 topic info /dlio/odometry -v
```

Several `/tf` publishers are normal. Publisher count alone does not prove
duplicate ownership of a particular edge; inspect messages, node connections,
and enabled launch processes. Compare rover motion in Gazebo with odometry
and each TF edge. Vertical jumps can result from duplicate odometry TF,
registration, IMU/gravity mismatch, or real collision jitter. Determine which
edge or physical state changes before choosing a fix.

For estimation drift, inspect LiDAR/IMU stamp alignment, point-time fields,
sensor frame/extrinsics, calibration while stationary, gravity, scan period,
deskew, and registration diagnostics. Then examine keyframe acceptance and
loop-closure results. Use `scripts/togo/dlio_diag_once.py` or relevant existing
bag timing tools under `lidarslam_ros2/`. Do not weaken registration rejection
gates merely to make odometry continue publishing.

## Empty or flickering maps

Trace data from `/dlio/deskewed` to graph inputs and `/modified_map_timed`, then
the global hazard pipeline and `/map`. Trace local hazards separately from
`/dlio/deskewed` to `/local_hazard_map` and debug points. Inspect frame, stamps,
cloud fields, occupancy values, and filtering thresholds at the failing link.

The Nav2 launch selects either the hazard-patch pipeline or the older global
grid conversion; both can publish `/map`. Preserve a single active map source.
`/map` existing or a debug helper publishing zero points does not establish that
the occupancy content is useful. Inspect the grid and local/global costmaps.
Check whether debug-map mode is hiding a broken real mapping pipeline.

## Slow navigation or failed rotation

Separate commanded motion from measured motion:

1. Verify Nav2 lifecycle state, goal frame/stamp, plan, command type, and actual
   command publishers. Follow controller, smoother, collision monitor, and
   launch remaps to the rover topic; look for competing outputs or stops.
2. If commands are small or suppressed, inspect costmaps, hazard inflation,
   controller limits, progress checker, and collision-monitor decisions.
3. If appropriate simulation commands arrive but the rover barely moves,
   inspect active ros2_control controllers, wheel joints, collisions, friction,
   real-time factor, and the selected physics plugin. Discover the actual
   joint-state topic; do not assume it is unnamespaced `/joint_states`.
4. For an authorized motion test, compare a flat world with the lunar world
   using the same rover/controller and command. Stop competing Nav2/teleop
   sources for the test. Measure forward displacement and yaw; spinning wheel
   visuals alone do not establish traction. End with zero velocity.

Classic Bullet was the successful local baseline after DART mesh-construction
errors and lack of turning under Bullet Featherstone. Treat that as repo-specific
evidence, not a claim about all Gazebo versions or robots. Check collision mesh
shape, spawn location, and collision-to-visual height before retuning friction.

## Graphics and missing resources

`glxinfo -B` on WSL showing D3D12 establishes acceleration for that environment;
`nvidia-smi` in Docker establishes GPU visibility and does not establish OpenGL
acceleration. Keep graphics checks separate from DDS or SLAM health.

For missing worlds/meshes, resolve `ros2 pkg prefix togo_gz` and inspect the
installed `share/togo_gz/worlds` and `media` files plus resource paths. Generated
terrain assets are ignored by Git and may need regeneration and installation
in a fresh checkout. Do not treat a Fuel download error as proof of networking
failure when a requested local world file is absent.

## Report meaningful validation

For a runtime fix, record mode, world, sensor topics, parameter overrides,
clock/TF evidence, and the observed outcome. For tuning, compare the same bag
or trajectory and report drift, map consistency, dropped/rejected data,
latency, or goal completion as relevant. Avoid claiming SLAM accuracy solely
from how aligned the RViz picture looks. If the runtime is unavailable, report
static validation and the remaining operator checks explicitly.
