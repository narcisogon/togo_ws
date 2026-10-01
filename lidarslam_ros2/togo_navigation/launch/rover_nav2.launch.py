"""Minimal Nav2 bringup for the SLAM-backed TOGO/Husky rover.

This launch assumes SLAM is already publishing:
  map -> odom       from graph_based_slam
  odom -> base_link from RKO-LIO

It starts only the navigation servers we need now. The stock Nav2
navigation_launch.py also starts route, collision, and docking servers in
Jazzy, which require extra configuration we do not want for the first rover
milestone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def nav2_node(package, executable, name, params_file, use_sim_time):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    debug_map = LaunchConfiguration('debug_map')
    use_slam_map = LaunchConfiguration('use_slam_map')
    use_patch_hazard_map = LaunchConfiguration('use_patch_hazard_map')
    request_initial_map_save = LaunchConfiguration('request_initial_map_save')

    default_params = PathJoinSubstitution([
        FindPackageShare('togo_navigation'),
        'config',
        'nav2_slam_params.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('togo_navigation'),
        'rviz',
        'rover_nav_debug.rviz',
    ])

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nav2 = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/platform_velocity_controller/cmd_vel'),
            SetRemap(src='cmd_vel', dst='/platform_velocity_controller/cmd_vel'),
            SetRemap(src='/cmd_vel_smoothed', dst='/platform_velocity_controller/cmd_vel'),
            SetRemap(src='cmd_vel_smoothed', dst='/platform_velocity_controller/cmd_vel'),
            Node(
                package='togo_navigation',
                executable='debug_map_publisher.py',
                name='debug_map_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(debug_map),
            ),
            Node(
                package='togo_navigation',
                executable='slam_to_occupancy_grid',
                name='slam_to_occupancy_grid',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_cloud_topic': '/modified_map',
                        'output_map_topic': '/map',
                        'target_frame': 'map',
                        'resolution': 0.20,
                        'width_m': 80.0,
                        'height_m': 80.0,
                        'origin_x': -40.0,
                        'origin_y': -40.0,
                        'min_obstacle_height': 0.25,
                        'max_obstacle_height': 1.20,
                        'min_points_per_cell': 1,
                        'initialize_as_free': True,
                        'obstacle_dilation_cells': 0,
                        'enable_gradient_costs': False,
                        'gradient_radius_m': 0.0,
                        'gradient_min_cost': 8,
                        'gradient_power': 1.3,
                        'enable_terrain_hazards': False,
                        'terrain_min_height': -0.45,
                        'terrain_max_height': 0.10,
                        'terrain_slope_hazard_deg': 50.0,
                        'terrain_step_hazard_m': 0.45,
                        'terrain_min_points_per_cell': 5,
                        'terrain_neighbor_radius_cells': 2,
                        'terrain_hazard_dilation_cells': 0,
                        'max_input_range_m': 120.0,
                        'clear_robot_radius_m': 1.2,
                        'enable_robot_trail_clearing': True,
                        'robot_trail_clear_radius_m': 0.9,
                        'robot_trail_max_poses': 300,
                        'robot_trail_min_distance_m': 0.15,
                        'robot_frame': 'base_link',
                        'center_map_on_robot': False,
                        'publish_empty_map_until_first_cloud': True,
                    },
                ],
                # Superseded by default by the per-submap hazard-patch pipeline
                # below (use_patch_hazard_map defaults true). Both publish /map,
                # so only one may run at a time -- flip use_patch_hazard_map:=false
                # to fall back to this whole-cloud path instead.
                condition=IfCondition(
                    PythonExpression([
                        "'", use_slam_map, "' == 'true' and '",
                        use_patch_hazard_map, "' == 'false'",
                    ])
                ),
            ),
            Node(
                package='hazard_mapping',
                executable='hazard_patch_node',
                name='hazard_patch_node',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        # Was 0.6 -- with lethal_cost_threshold back to 100
                        # (nav2_slam_params.yaml) the soft gradient can no
                        # longer accidentally become a second lethal wall, so
                        # it can be widened into a real "prefer the
                        # centerline between hazards" halo instead of a
                        # narrow buffer. hard_lethal_radius_m stays the same
                        # 0.3 -- only its ratio of the total radius shrinks.
                        'gradient_radius_m': 1.2,
                        # Inner ring that is unconditionally lethal (no
                        # falloff) -- makes "don't get this close" a hard
                        # constraint the planner can't route through, not
                        # just a strong-but-negotiable cost. Soft gradient
                        # covers the remaining 1.2 - 0.3 = 0.9m out to
                        # gradient_radius_m. Keep in sync with
                        # local_hazard_grid's hard_lethal_radius_m below.
                        'hard_lethal_radius_m': 0.3,
                        # Ceiling for the soft band -- must stay below
                        # lethal_cost_threshold (100) so it's genuinely a
                        # preference, not a de facto second lethal wall.
                        'gradient_min_cost': 5,
                        'soft_max_cost': 60,
                        'terrain_max_height': 0.10,      
          	        'terrain_min_height': -0.90,
                        'terrain_step_hazard_m': 0.30,
                        'terrain_slope_hazard_deg': 45.0,
                        'min_points_per_obstacle_cell': 3,
                    },
                ],
                condition=IfCondition(use_patch_hazard_map),
            ),
            Node(
                package='hazard_mapping',
                executable='global_costmap_composer',
                name='global_costmap_composer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_patch_hazard_map),
            ),
            Node(
                package='togo_navigation',
                executable='goal_safety_relay',
                name='goal_safety_relay',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'goal_topic': '/goal_pose_requested',
                        'costmap_topic': '/global_costmap/costmap',
                        'action_name': 'navigate_to_pose',
                        # Keep in sync with global_costmap's static_layer
                        # lethal_cost_threshold in nav2_slam_params.yaml.
                        'lethal_cost_threshold': 100,
                        # Relocated goals must land at least this far below
                        # lethal_cost_threshold, not just barely under it --
                        # otherwise the next replan's tiny recomputation
                        # difference can tip a barely-safe cell back into
                        # "occupied" and strand the robot ("Start occupied").
                        # Was 20 -- with lethal_cost_threshold now 100 and the
                        # soft gradient capped at soft_max_cost=60, a margin
                        # of 45 lands relocated goals in the 55-60 range: at
                        # the outer, cheapest edge of the soft band rather
                        # than deep inside it.
                        'safety_margin_cost': 45,
                        'search_radius_m': 3.0,
                    },
                ],
            ),
            Node(
                package='togo_navigation',
                executable='local_hazard_grid',
                name='local_hazard_grid',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_cloud_topic': '/dlio/deskewed',
                        'output_map_topic': '/local_hazard_map',
                        'target_frame': 'odom',
                        'robot_frame': 'base_link',
                        'resolution': 0.10,
                        'width_m': 10.0,
                        'height_m': 10.0,
                        'history_duration_sec': 3.0,
                        'max_history_points': 250000,
                        'publish_rate_hz': 5.0,
                        # A cell only confirms as hazard once it's been
                        # observed across at least this much time -- damps
                        # single-scan noise blips that were painting a
                        # hazard cell for one publish and vanishing the
                        # next, which the controller was reacting to as a
                        # real, sudden obstacle ("collision ahead" bursts
                        # that evaporate on their own a second later).
                        'hazard_confirm_sec': 0.3,
                        'min_obstacle_height': 0.25,
                        'max_obstacle_height': 1.20,
                        'terrain_min_height': -0.75,
                        'terrain_max_height': 1.20,
                        'terrain_slope_hazard_deg': 30.0,
                        'terrain_step_hazard_m': 0.25,
                        'terrain_min_points_per_cell': 5,
                        # Was 2 (~0.2m neighbor baseline at this resolution) --
                        # too short a baseline means a few cm of real terrain
                        # noise crosses the slope threshold, and the exact
                        # per-cell point count flickers across
                        # terrain_min_points_per_cell between the 5Hz publishes
                        # of a 2s rolling window, both producing hazard cells
                        # that pop in/out near the robot and trigger repeated
                        # stop/replan. Wider baseline damps single-cell noise
                        # without weakening detection of a real sustained slope.
                        'terrain_neighbor_radius_cells': 4,
                        'min_points_per_obstacle_cell': 1,
                        'gradient_radius_m': 0.4,
                        # Same hard-lethal-core rationale as hazard_patch_node
                        # above. Soft gradient covers the remaining
                        # 0.4 - 0.2 = 0.2m out to gradient_radius_m. Not
                        # widened like hazard_patch_node's -- this grid only
                        # reaches the local costmap through
                        # occupancy_grid_to_points' occupied_threshold=50
                        # binarization below, so the local obstacle layer's
                        # own inflation already provides the local soft
                        # buffer; widening here wouldn't change what the
                        # planner sees.
                        'hard_lethal_radius_m': 0.2,
                        'gradient_min_cost': 8,
                        # Ceiling for the soft band -- see hazard_patch_node's
                        # matching param. Keeps soft cells (8-60) mostly below
                        # occupied_threshold=50 so they're correctly NOT
                        # injected as obstacle points, while the hard-lethal
                        # core (100) always is.
                        'soft_max_cost': 60,
                        # Was 1.0 -- only barely larger than footprint extent
                        # (0.55m) + inflation_radius (0.35m) = 0.90m combined,
                        # so a flickering hazard cell just outside the old
                        # clear radius could still inflate onto the robot's
                        # own footprint. 1.2 gives real margin.
                        'clear_robot_radius_m': 1.2,
                        'max_input_range_m': 6.0,
                    },
                ],
            ),
            Node(
                package='togo_navigation',
                executable='occupancy_grid_to_points',
                name='map_debug_points',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': '/map',
                        'output_topic': '/map_debug_points',
                        'occupied_threshold': 50,
                        'include_unknown': False,
                        'point_z': 0.10,
                    },
                ],
            ),
            Node(
                package='togo_navigation',
                executable='occupancy_grid_to_points',
                name='local_hazard_debug_points',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': '/local_hazard_map',
                        'output_topic': '/local_hazard_debug_points',
                        'occupied_threshold': 50,
                        'include_unknown': False,
                        'point_z': 0.20,
                    },
                ],
            ),
            Node(
                package='togo_navigation',
                executable='occupancy_grid_to_points',
                name='global_costmap_debug_points',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'input_topic': '/global_costmap/costmap',
                        'output_topic': '/global_costmap/debug_points',
                        'occupied_threshold': 50,
                        'include_unknown': False,
                        'point_z': 0.16,
                    },
                ],
            ),
            ExecuteProcess(
                cmd=[
                    'bash', '-lc',
                    'source /opt/ros/jazzy/setup.bash; '
                    'source /ws/install/setup.bash; '
                    'until ros2 service type /map_save >/dev/null 2>&1; do sleep 1; done; '
                    'sleep 2; '
                    'ros2 service call /map_save std_srvs/srv/Empty'
                ],
                output='screen',
                condition=IfCondition(request_initial_map_save),
            ),
            nav2_node('nav2_controller', 'controller_server', 'controller_server', params_file, use_sim_time),
            nav2_node('nav2_smoother', 'smoother_server', 'smoother_server', params_file, use_sim_time),
            nav2_node('nav2_planner', 'planner_server', 'planner_server', params_file, use_sim_time),
            nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', params_file, use_sim_time),
            nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params_file, use_sim_time),
            nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params_file, use_sim_time),
            nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params_file, use_sim_time),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'node_names': lifecycle_nodes,
                    },
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='nav2_rviz',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(rviz),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'debug_map',
            default_value='false',
            description='Publish a simple /map OccupancyGrid until the SLAM-to-OGM mapper exists.',
        ),
        DeclareLaunchArgument(
            'use_slam_map',
            default_value='true',
            description='Convert /modified_map PointCloud2 into /map OccupancyGrid for Nav2.',
        ),
        DeclareLaunchArgument(
            'use_patch_hazard_map',
            default_value='true',
            description=(
                'Use the per-submap hazard-patch pipeline (hazard_patch_node + '
                'global_costmap_composer) for /map instead of slam_to_occupancy_grid. '
                'Set to false to fall back to the whole-cloud slam_to_occupancy_grid path '
                '(requires use_slam_map:=true).'
            ),
        ),
        DeclareLaunchArgument(
            'request_initial_map_save',
            default_value='true',
            description='Call /map_save once after startup so /modified_map feeds the SLAM occupancy mapper.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 params configured for SLAM-provided map->odom and RKO odom->base_link.',
        ),
        nav2,
    ])
