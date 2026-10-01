"""Integrated live Seyond RKO-LIO frontend + graph SLAM backend launch.

All frontend and backend tuning parameters live in one YAML by default:
  lidarslam/param/seyond_live_slam.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_param_file = LaunchConfiguration('slam_param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_save_period = LaunchConfiguration('map_save_period')
    enable_map_save_pulse = LaunchConfiguration('enable_map_save_pulse')
    enable_frontend_stability_filter = LaunchConfiguration('enable_frontend_stability_filter')
    enable_imu_prediction = LaunchConfiguration('enable_imu_prediction')
    frontend_path_min_distance = LaunchConfiguration('frontend_path_min_distance')
    backend_odom_topic = LaunchConfiguration('backend_odom_topic')
    publish_static_map_to_odom = LaunchConfiguration('publish_static_map_to_odom')

    rko_node = Node(
        package='rko_lio',
        executable='online_node',
        name='rko_lio_online_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    xyzi_adapter = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/add_intensity_to_cloud.py',
            '--ros-args',
            '-p', 'input_topic:=/rko_lio/frame',
            '-p', 'output_topic:=/rko_lio/frame_xyzi',
            '-p', 'intensity:=1.0',
        ],
        output='screen',
    )

    graph_node = Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        name='graph_based_slam',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odom_input', backend_odom_topic),
            ('cloud_input', '/rko_lio/frame_xyzi'),
        ],
    )

    frontend_stability_filter = Node(
        package='rko_lio',
        executable='frontend_stability_filter_node',
        name='frontend_stability_filter',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_topic': '/rko_lio/odometry',
                'diagnostics_topic': '/rko_lio/registration_diagnostics',
                'stable_odom_topic': '/rko_lio/odometry_stable',
                'stable_path_topic': '/rko_lio/path_stable',
                'status_topic': '/rko_lio/stability_status',
                'fixed_frame': 'odom',
                'max_linear_velocity': 1.5,
                'max_yaw_rate_deg': 70.0,
                'min_overlap_ratio': 0.12,
                'max_mean_error_m': 1.5,
                'max_hessian_condition': 100000.0,
                'prediction_decay': 0.25,
                'max_prediction_frames': 2,
                'stationary_linear_velocity': 0.05,
                'stationary_yaw_rate_deg': 3.0,
            },
        ],
        condition=IfCondition(enable_frontend_stability_filter),
    )

    imu_prediction = Node(
        package='rko_lio',
        executable='imu_prediction_node',
        name='imu_prediction',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_topic': '/rko_lio/odometry',
                'imu_topic': '/a300_0000/sensors/seyond_robin_w/imu',
                'predicted_odom_topic': '/rko_lio/odometry_imu_predict',
                'predicted_path_topic': '/rko_lio/path_imu_predict',
                'fixed_frame': 'odom',
                'child_frame': 'base_link_imu_predict',
                'max_prediction_horizon_sec': 0.12,
                'max_imu_dt_sec': 0.05,
                'max_publish_rate_hz': 80.0,
                'max_path_poses': 12000,
                'path_min_distance_m': 0.03,
                'prefer_odom_twist': False,
                'twist_in_child_frame': True,
                'use_acceleration': False,
                'gravity_mps2': 9.8107,
                'velocity_smoothing_alpha': 0.9,
                'stationary_velocity_threshold': 0.02,
                'gyro_deadband_rad_s': 0.002,
                'max_linear_velocity': 2.0,
                'max_angular_velocity': 3.0,
                'max_position_extrapolation_m': 0.08,
            },
        ],
        condition=IfCondition(enable_imu_prediction),
    )

    frontend_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/odom_to_path.py',
            '--ros-args',
            '-r', '__node:=frontend_path_publisher',
            '-p', 'odom_topic:=/rko_lio/odometry',
            '-p', 'path_topic:=/rko_lio/path',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'publish_every_n:=1',
            '-p', ['min_distance_m:=', frontend_path_min_distance],
        ],
        output='screen',
    )

    reference_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/gazebo_pose_to_aligned_path.py',
            '--ros-args',
            '-r', '__node:=reference_path_publisher',
            '-p', 'pose_topic:=/gazebo/dynamic_pose',
            '-p', 'frontend_odom_topic:=/rko_lio/odometry',
            '-p', 'path_topic:=/reference/path',
            '-p', 'model_names:=a300-0000,a300_0000,a300,robot,base_link,chassis_link',
            '-p', 'fixed_frame:=odom',
            '-p', 'max_poses:=12000',
            '-p', 'min_distance_m:=0.02',
            '-p', 'extra_yaw_offset_deg:=0.0',
        ],
        output='screen',
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        condition=IfCondition(publish_static_map_to_odom),
    )

    map_save_pulse = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source /ws/install/setup.bash; '
            'period=${MAP_SAVE_PERIOD:-10}; '
            'until ros2 service list | grep -qx /map_save; do sleep 1; done; '
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml',
            description='Single YAML containing both RKO-LIO frontend and graph SLAM backend parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument(
            'enable_map_save_pulse',
            default_value='false',
            description='Periodically call /map_save. Disable during Nav2 tests to avoid extra ros2 CLI DDS participants.',
        ),
        DeclareLaunchArgument(
            'enable_frontend_stability_filter',
            default_value='false',
            description='Publish /rko_lio/odometry_stable with motion-prior gating and confidence-aware smoothing.',
        ),
        DeclareLaunchArgument(
            'enable_imu_prediction',
            default_value='false',
            description='Publish high-rate short-horizon IMU-predicted odometry/path without changing the main TF tree.',
        ),
        DeclareLaunchArgument(
            'frontend_path_min_distance',
            default_value='0.005',
            description='Minimum XY movement before appending a pose to /rko_lio/path.',
        ),
        DeclareLaunchArgument(
            'backend_odom_topic',
            default_value='/rko_lio/odometry',
            description='Odometry topic consumed by graph_based_slam.',
        ),
        DeclareLaunchArgument(
            'publish_static_map_to_odom',
            default_value='false',
            description='Fallback identity map->odom TF. Keep false when graph_based_slam publishes dynamic correction.',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        rko_node,
        xyzi_adapter,
        frontend_stability_filter,
        imu_prediction,
        graph_node,
        frontend_path,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])
