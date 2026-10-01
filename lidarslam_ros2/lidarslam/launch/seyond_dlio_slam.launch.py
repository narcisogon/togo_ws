"""Integrated live Seyond DLIO frontend + graph SLAM backend launch.

Sensor wiring (topics and frames) is fully parameterised so this launch file
works unchanged across different simulators — override the sensor args or the
matching env-vars in the wrapper bash script.

  Husky sim  -> run_dlio_slam_husky.sh
  a300 sim   -> run_dlio_slam.sh
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Param files ──────────────────────────────────────────────────────────────
    slam_param_file  = LaunchConfiguration('slam_param_file')
    dlio_param_file  = LaunchConfiguration('dlio_param_file')

    # ── Sensor wiring ────────────────────────────────────────────────────────────
    lidar_topic       = LaunchConfiguration('lidar_topic')
    lidar_topic_timed = LaunchConfiguration('lidar_topic_timed')
    imu_topic         = LaunchConfiguration('imu_topic')
    lidar_frame       = LaunchConfiguration('lidar_frame')
    imu_frame         = LaunchConfiguration('imu_frame')
    base_frame        = LaunchConfiguration('base_frame')
    odom_frame        = LaunchConfiguration('odom_frame')

    # ── Behaviour flags ──────────────────────────────────────────────────────────
    use_sim_time                     = LaunchConfiguration('use_sim_time')
    rviz                             = LaunchConfiguration('rviz')
    rviz_config                      = LaunchConfiguration('rviz_config')
    map_save_period                  = LaunchConfiguration('map_save_period')
    enable_map_save_pulse            = LaunchConfiguration('enable_map_save_pulse')


    publish_static_map_to_odom       = LaunchConfiguration('publish_static_map_to_odom')
    timed_cloud_scan_period          = LaunchConfiguration('timed_cloud_scan_period')
    timed_cloud_reverse_columns      = LaunchConfiguration('timed_cloud_reverse_columns')
    dlio_deskew                      = LaunchConfiguration('dlio_deskew')
    # ── New from other sim ───────────────────────────────────────────────────────
    correct_cloud_slant              = LaunchConfiguration('correct_cloud_slant')
    cloud_slant_z_correction_per_meter = LaunchConfiguration('cloud_slant_z_correction_per_meter')
    timed_cloud_input_topic          = LaunchConfiguration('timed_cloud_input_topic')

    # ── DLIO base configs (shipped with the package) ─────────────────────────────
    dlio_base_params    = '/home/er4-user/slam_ws/src/lidarslam_ros2/direct_lidar_inertial_odometry/cfg/dlio.yaml'
    dlio_runtime_params = '/home/er4-user/slam_ws/src/lidarslam_ros2/direct_lidar_inertial_odometry/cfg/params.yaml'

    # ────────────────────────────────────────────────────────────────────────────
    # Nodes
    # ────────────────────────────────────────────────────────────────────────────

    # Optional z-slant corrector — only active when correct_cloud_slant=true.
    # Corrects the Gazebo gpu_lidar range-dependent z slant before timing.
    # Input topic is always the raw lidar_topic for this sim.
    cloud_slant_corrector = ExecuteProcess(
        cmd=[
            'python3', '/home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/correct_seyond_cloud_slant.py',
            '--ros-args',
            '-r', '__node:=seyond_cloud_slant_corrector',
            '-p', ['input_topic:=',  lidar_topic],
            '-p', ['output_topic:=', lidar_topic, '_corrected'],
            '-p', ['z_correction_per_meter:=', cloud_slant_z_correction_per_meter],
        ],
        output='screen',
        condition=IfCondition(correct_cloud_slant),
    )



    # DLIO odometry frontend.
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        name='dlio_odom_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            dlio_param_file,            # robot-specific overrides (last YAML wins)
            {'use_sim_time':      use_sim_time},
            # Frame overrides — these take precedence over the YAML values so
            # the bash script is the single source of truth for frame names.
            {'frames/lidar':      lidar_frame},
            {'frames/imu':        imu_frame},
            {'frames/baselink':   base_frame},
            {'frames/odom':       odom_frame},
            {'odom/odom_frame':   odom_frame},
            {'pointcloud/deskew': ParameterValue(dlio_deskew, value_type=bool)},
            {'seyond/scanPeriodSec': timed_cloud_scan_period},
            {'pointcloud/deskew': ParameterValue(dlio_deskew, value_type=bool)},
            {'pointcloud/syntheticTiming/enabled': True},
            {'pointcloud/syntheticTiming/scanPeriodSec': ParameterValue(
                timed_cloud_scan_period, value_type=float)},
            {'pointcloud/syntheticTiming/reverseColumns': ParameterValue(
                timed_cloud_reverse_columns, value_type=bool)},

        ],
        remappings=[
            # Inputs
            ('pointcloud',           lidar_topic),
            ('imu',                  imu_topic),
            # Outputs — fixed names the rest of the pipeline depends on.
            # path is remapped to a dead topic — RAM fix.
            # /dlio/path_simple is published by odom_to_path.py with a cap.
            ('odom',                 '/dlio/odometry'),
            ('pose',                 '/dlio/pose'),
            ('path',                 '/dlio/path_raw_disabled'),
            ('kf_pose',              '/dlio/keyframes'),
            ('kf_cloud',             '/dlio/keyframe_cloud'),
            ('deskewed',             '/dlio/deskewed'),
            ('frontend_diagnostics', '/dlio/frontend_diagnostics'),
        ],
    )

    # DLIO local map accumulator.
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        name='dlio_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            dlio_base_params,
            dlio_runtime_params,
            dlio_param_file,
            {'use_sim_time':    use_sim_time},
            {'odom/odom_frame': odom_frame},
        ],
        remappings=[
            ('keyframes', '/dlio/keyframe_cloud'),
            ('map',       '/dlio/map'),
        ],
    )

    # Graph SLAM backend.
    graph_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {'use_sim_time':                   use_sim_time},
            {'odom_frame_id':                  odom_frame},
            {'global_frame_id':                'map'},
            {'odom_input_cloud_in_odom_frame':  True},
        ],
        remappings=[
            ('odom_input',  '/dlio/odometry'),
            ('cloud_input', '/dlio/deskewed'),
        ],
    )

    # Lightweight frontend odometry path for RViz / APE comparison.
    frontend_path = ExecuteProcess(
        cmd=[
            'python3', '/home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/pose_to_path.py',
            '--ros-args',
            '-r', '__node:=dlio_path_publisher',
            '-p', 'pose_topic:=/dlio/pose',
            '-p', 'path_topic:=/dlio/path_simple',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'publish_every_n:=1',


        ],
        output='screen',
    )

    # Ground-truth path aligned to the frontend's initial pose.
    # model_names and other tuning live in dlio_param_file under
    # reference_path_publisher.ros__parameters so each robot YAML
    # sets the right Gazebo model name without touching this file.
    reference_path = ExecuteProcess(
        cmd=[
            'python3', '/home/er4-user/slam_ws/src/lidarslam_ros2/scripts/togo/gazebo_pose_to_aligned_path.py',
            '--ros-args',
            '-r', '__node:=reference_path_publisher',
            '--params-file', dlio_param_file,
        ],
        output='screen',
    )

    # Fallback identity map->odom TF (keep false when graph_based_slam is live).
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', odom_frame,
        ],
        condition=IfCondition(publish_static_map_to_odom),
    )

    # Periodically calls /map_save to flush the backend map to disk.
    map_save_pulse = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source /home/er4-user/slam_ws/install/setup.bash; '
            'period=${MAP_SAVE_PERIOD:-60}; '
            'until ros2 service list 2>/dev/null | grep -qx /map_save; do sleep 1; done; '
            'while true; do ros2 service call /map_save std_srvs/srv/Empty; sleep "$period"; done'
        ],
        output='screen',
        additional_env={'MAP_SAVE_PERIOD': map_save_period},
        condition=IfCondition(enable_map_save_pulse),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    # ────────────────────────────────────────────────────────────────────────────
    return LaunchDescription([

        # ── Param files ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml',
            description='Backend graph SLAM parameters.',
        ),
        DeclareLaunchArgument(
            'dlio_param_file',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml',
            description='Robot-specific DLIO frontend override parameters.',
        ),

        # ── Sensor wiring ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/husky/sensors/seyond/points',
            description='Raw PointCloud2 topic from the simulator.',
        ),
        DeclareLaunchArgument(
            'lidar_topic_timed',
            default_value='/husky/sensors/seyond/points_timed',
            description='Timed PointCloud2 output from the adapter (auto-derived in bash script).',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/husky/sensors/imu_0/data_raw',
            description='IMU topic from the simulator.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='lidar3d_0_laser',
            description='LiDAR sensor frame ID.',
        ),
        DeclareLaunchArgument(
            'imu_frame',
            default_value='imu_0_link',
            description='IMU sensor frame ID.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame.',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry world frame.',
        ),

        # ── Behaviour flags ───────────────────────────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),
           
        DeclareLaunchArgument(
            'timed_cloud_scan_period',
            default_value='0.0666666667',
            description='Synthetic per-point timing span for the 15 Hz Gazebo scan.',
        ),
        DeclareLaunchArgument(
            'timed_cloud_reverse_columns',
            default_value='false',
            description='Reverse synthetic per-column time if scan ordering is opposite.',
        ),
        DeclareLaunchArgument(
            'dlio_deskew',
            default_value='false',
            description='Enable DLIO per-point deskew. Off by default for husky sim stability.',
        ),
        DeclareLaunchArgument(
            'correct_cloud_slant',
            default_value='false',
            description='Correct the Gazebo gpu_lidar range-dependent z slant before DLIO timing.',
        ),
        DeclareLaunchArgument(
            'timed_cloud_input_topic',
            default_value='/husky/sensors/seyond/points',
            description='Input cloud for the timing adapter. Set to points_corrected when correct_cloud_slant=true.',
        ),
        DeclareLaunchArgument(
            'cloud_slant_z_correction_per_meter',
            default_value='0.0268',
            description='Positive z correction per meter of horizontal range for the Gazebo gpu_lidar slant.',
        ),
        DeclareLaunchArgument(
            'publish_static_map_to_odom',
            default_value='false',
            description='Fallback identity map->odom TF. Keep false when graph_based_slam is live.',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/home/er4-user/slam_ws/src/lidarslam_ros2/lidarslam/rviz/mapping.rviz',
            description='RViz config file.',
        ),

        # ── Node order ────────────────────────────────────────────────────────────
        cloud_slant_corrector,
        dlio_odom_node,
        dlio_map_node,
        graph_node,
        frontend_path,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])
