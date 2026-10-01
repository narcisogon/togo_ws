"""Flat live-real Husky/Seyond DLIO + graph SLAM.

Single-file launch — no nested IncludeLaunchDescription, no timestamp
corrector.  /iv_points feeds DLIO directly.  use_sim_time is always false.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    repo_root  = LaunchConfiguration('repo_root')
    param_file = LaunchConfiguration('param_file')

    # DLIO ships its own base configs inside the package source tree.
    # Our single param_file is the final override layer on top of both.
    dlio_base_params    = PathJoinSubstitution(
        [repo_root, 'direct_lidar_inertial_odometry', 'cfg', 'dlio.yaml'])
    dlio_runtime_params = PathJoinSubstitution(
        [repo_root, 'direct_lidar_inertial_odometry', 'cfg', 'params.yaml'])

    # ------------------------------------------------------------------
    # DLIO frontend
    # ------------------------------------------------------------------
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            param_file,                         # final word on every param
            {'use_sim_time': False},
            # Launch-arg overrides — these survive even if param_file sets them
            {'pointcloud/deskew': ParameterValue(
                LaunchConfiguration('deskew'), value_type=bool)},
            {'odom/groundConstraint/enabled': ParameterValue(
                LaunchConfiguration('ground_constraint'), value_type=bool)},
            {'odom/groundConstraint/baselinkHeight': ParameterValue(
                LaunchConfiguration('ground_baselink_height'), value_type=float)},
            # Synthetic timing is never used for live hardware
            {'pointcloud/syntheticTiming/enabled': False},
            # Frame IDs
            {'frames/odom':     LaunchConfiguration('odom_frame')},
            {'frames/baselink': LaunchConfiguration('base_frame')},
            {'frames/lidar':    LaunchConfiguration('lidar_frame')},
            {'frames/imu':      LaunchConfiguration('imu_frame')},
            {'odom/odom_frame': LaunchConfiguration('odom_frame')},
        ],
        remappings=[
            ('pointcloud',           LaunchConfiguration('lidar_topic')),
            ('imu',                  LaunchConfiguration('imu_topic')),
            ('odom',                 '/dlio/odometry'),
            ('pose',                 '/dlio/pose'),
            ('path',                 '/dlio/path_raw'),
            ('kf_pose',              '/dlio/keyframes'),
            ('kf_cloud',             '/dlio/keyframe_cloud'),
            ('deskewed',             '/dlio/deskewed'),
            ('frontend_diagnostics', '/dlio/frontend_diagnostics'),
        ],
    )

    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            param_file,
            {'use_sim_time': False},
        ],
        remappings=[
            ('keyframes', '/dlio/keyframe_cloud'),
            ('map',       '/dlio/map'),
        ],
    )

    # Publishes the DLIO odometry as a TF odom -> base_link transform
    dlio_odom_tf = Node(
        package='togo_navigation',
        executable='odometry_to_tf',
        name='dlio_odometry_to_tf',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'odom_topic':   '/dlio/odometry'},
            {'parent_frame': LaunchConfiguration('odom_frame')},
            {'child_frame':  LaunchConfiguration('base_frame')},
        ],
    )

    # ------------------------------------------------------------------
    # Graph SLAM backend
    # ------------------------------------------------------------------
    graph_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            param_file,
            {'use_sim_time': False},
            {'odom_frame_id':                  LaunchConfiguration('odom_frame')},
            {'odom_input_cloud_in_odom_frame': True},
            {'pcd_cache_dir':                  LaunchConfiguration('graph_pcd_cache_dir')},
            {'map_save_dir':                   LaunchConfiguration('graph_map_save_dir')},
            {'dem/output_dir':                 LaunchConfiguration('graph_dem_output_dir')},
        ],
        remappings=[
            ('odom_input',  '/dlio/odometry'),
            ('cloud_input', '/dlio/deskewed'),
        ],
    )

    # ------------------------------------------------------------------
    # Helper processes
    # ------------------------------------------------------------------
    frontend_path = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([repo_root, 'scripts', 'togo', 'odom_to_path.py']),
            '--ros-args',
            '-r', '__node:=dlio_path_publisher',
            '-p', 'odom_topic:=/dlio/odometry',
            '-p', 'path_topic:=/dlio/path_simple',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=2000',
            '-p', 'publish_every_n:=1',
            '-p', ['min_distance_m:=',     LaunchConfiguration('frontend_path_min_distance')],
            '-p', ['publish_period_sec:=',  LaunchConfiguration('frontend_path_publish_period')],
        ],
        output='screen',
    )

    # Periodic automatic map save — off by default on live runs
    map_save_pulse = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source "${SLAM_WS}/install/setup.bash"; '
            'period=${MAP_SAVE_PERIOD:-60}; '
            'until ros2 service list | grep -qx /map_save; do sleep 1; done; '
            'while true; do '
            '  ros2 service call /map_save std_srvs/srv/Empty; '
            '  sleep "$period"; '
            'done',
        ],
        additional_env={
            'MAP_SAVE_PERIOD': LaunchConfiguration('map_save_period'),
            'SLAM_WS':         LaunchConfiguration('slam_ws'),
        },
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_map_save_pulse')),
    )

    # ------------------------------------------------------------------
    # TF publishers
    # ------------------------------------------------------------------

    # Fallback identity map -> odom.  Keep false when graph SLAM is running
    # because graph_based_slam publishes the live corrected transform.
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_map_to_odom')),
    )

    # Sensor TF — the full rover bringup already publishes these.
    # Enable only for a sensor-only bench run.
    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='live_base_to_lidar_tf',
        arguments=[
            '--x', '0.400', '--y', '0.0', '--z', '0.313',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('lidar_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_sensor_static_tf')),
    )
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='live_base_to_imu_tf',
        arguments=[
            '--x', '0.059', '--y', '0.0', '--z', '0.161275',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('imu_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_sensor_static_tf')),
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ------------------------------------------------------------------
    # Argument declarations
    # ------------------------------------------------------------------
    return LaunchDescription([

        # Paths
        DeclareLaunchArgument(
            'repo_root',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2',
            description='Absolute lidarslam_ros2 source root.'),
        DeclareLaunchArgument(
            'slam_ws',
            default_value='/home/er4-user/slam_ws',
            description='Workspace root; used only by the map-save pulse sourcing.'),
        DeclareLaunchArgument(
            'param_file',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2'
                          '/lidarslam/param/seyond_dlio_live_real_flat.yaml',
            description='Single YAML containing all DLIO and graph SLAM parameters.'),

        # Topics
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/iv_points',
            description='Seyond point cloud topic, fed directly to DLIO.'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/husky/sensors/imu_0/data'),

        # Frame IDs
        DeclareLaunchArgument('odom_frame',  default_value='odom'),
        DeclareLaunchArgument('base_frame',  default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar3d_0_laser'),
        DeclareLaunchArgument('imu_frame',   default_value='imu_0_link'),

        # DLIO runtime tuning
        DeclareLaunchArgument(
            'deskew',
            default_value='false',
            description='DLIO per-point deskew. Validate timing/extrinsics before enabling.'),
        DeclareLaunchArgument(
            'ground_constraint',
            default_value='false',
            description='Gated ground-plane z constraint; enable only on confirmed flat terrain.'),
        DeclareLaunchArgument(
            'ground_baselink_height',
            default_value='0.30',
            description='Expected base_link height above ground in metres.'),

        # TF control
        DeclareLaunchArgument(
            'publish_sensor_static_tf',
            default_value='false',
            description='Publish base->lidar and base->imu TF. '
                        'Leave false when the rover bringup already does this.'),
        DeclareLaunchArgument(
            'publish_static_map_to_odom',
            default_value='false',
            description='Fallback identity map->odom TF. '
                        'Keep false when graph_based_slam publishes the live correction.'),

        # Graph SLAM I/O dirs
        DeclareLaunchArgument(
            'graph_pcd_cache_dir',
            default_value='/dev/shm/graph_slam_pcd_cache_live_real',
            description='RAM-backed submap cache. Use a unique path per run.'),
        DeclareLaunchArgument(
            'graph_map_save_dir',
            default_value=PathJoinSubstitution(
                [LaunchConfiguration('repo_root'), 'output', 'husky_seyond_live_real']),
            description='Final map output directory.'),
        DeclareLaunchArgument(
            'graph_dem_output_dir',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('repo_root'),
                'output', 'husky_seyond_live_real', 'lunar_dem']),
            description='DEM output directory.'),

        # Map-save pulse
        DeclareLaunchArgument('map_save_period',       default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),

        # Frontend path helper
        DeclareLaunchArgument('frontend_path_min_distance',   default_value='0.05'),
        DeclareLaunchArgument('frontend_path_publish_period', default_value='0.5'),

        # RViz
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2'
                          '/lidarslam/rviz/mapping.rviz'),

        # Nodes / processes
        dlio_odom_node,
        dlio_map_node,
        dlio_odom_tf,
        graph_node,
        frontend_path,
        map_save_pulse,
        map_to_odom_tf,
        base_to_lidar,
        base_to_imu,
        rviz_node,
    ])
