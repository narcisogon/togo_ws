"""Live real-Husky Seyond + IMU input through DLIO and graph SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    timestamp_corrector = Node(
        package='lidarslam',
        executable='seyond_live_timestamp_corrector',
        name='seyond_live_timestamp_corrector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'input_topic': LaunchConfiguration('raw_lidar_topic'),
            'output_topic': LaunchConfiguration('corrected_lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'point_time_field': LaunchConfiguration('point_time_field'),
            'offset_sec': ParameterValue(
                LaunchConfiguration('lidar_time_offset_sec'), value_type=float),
            'adjust_point_times': True,
            'require_imu_sync': ParameterValue(
                LaunchConfiguration('enforce_timing_sync'), value_type=bool),
            'diagnostics_period_sec': ParameterValue(
                LaunchConfiguration('timing_diagnostics_period_sec'), value_type=float),
            'sync_warning_sec': ParameterValue(
                LaunchConfiguration('timing_sync_warning_sec'), value_type=float),
            'point_header_sanity_sec': ParameterValue(
                LaunchConfiguration('point_header_sanity_sec'), value_type=float),
        }],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LaunchConfiguration('source_launch')),
        launch_arguments={
            'repo_root': LaunchConfiguration('repo_root'),
            'slam_param_file': LaunchConfiguration('base_param_file'),
            'slam_override_param_file': LaunchConfiguration('live_param_file'),
            'dlio_param_file': LaunchConfiguration('base_param_file'),
            'dlio_override_param_file': LaunchConfiguration('live_param_file'),
            'use_sim_time': 'false',
            'timed_cloud_input_topic': LaunchConfiguration('corrected_lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'synthetic_timing': 'false',
            'timed_cloud_scan_period': LaunchConfiguration('scan_period'),
            'timed_cloud_reverse_columns': 'false',
            'dlio_deskew': LaunchConfiguration('deskew'),
            'dlio_ground_constraint': LaunchConfiguration('ground_constraint'),
            'dlio_ground_baselink_height': LaunchConfiguration('ground_baselink_height'),
            'correct_cloud_slant': 'false',
            'enable_reference_path': 'false',
            'publish_static_map_to_odom': 'false',
            'graph_pcd_cache_dir': LaunchConfiguration('graph_pcd_cache_dir'),
            'graph_map_save_dir': LaunchConfiguration('graph_map_save_dir'),
            'graph_dem_output_dir': LaunchConfiguration('graph_dem_output_dir'),
            'map_save_period': LaunchConfiguration('map_save_period'),
            'enable_map_save_pulse': LaunchConfiguration('enable_map_save_pulse'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    # The full rover normally supplies these transforms. These conditional
    # publishers make a sensor-only bringup possible without creating a TF
    # conflict in the normal case.
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'source_launch',
            default_value='/ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py'),
        DeclareLaunchArgument('repo_root', default_value='/ws/src/lidarslam_ros2'),
        DeclareLaunchArgument(
            'base_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml'),
        DeclareLaunchArgument(
            'live_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_live_real.yaml'),
        DeclareLaunchArgument('raw_lidar_topic', default_value='/iv_points'),
        DeclareLaunchArgument(
            'corrected_lidar_topic', default_value='/iv_points/time_corrected'),
        DeclareLaunchArgument('imu_topic', default_value='/husky/sensors/imu_0/data'),
        DeclareLaunchArgument('point_time_field', default_value='timestamp'),
        DeclareLaunchArgument(
            'lidar_time_offset_sec',
            default_value='-6.575',
            description='Added to the live LiDAR header and every FLOAT64 point timestamp.'),
        DeclareLaunchArgument('timing_diagnostics_period_sec', default_value='5.0'),
        DeclareLaunchArgument('timing_sync_warning_sec', default_value='0.15'),
        DeclareLaunchArgument(
            'point_header_sanity_sec',
            default_value='0.5',
            description='Reject native point times that are implausibly far from the cloud header.'),
        DeclareLaunchArgument(
            'enforce_timing_sync',
            default_value='true',
            description='Block LiDAR output if corrected stamps are not close to live IMU stamps.'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar3d_0_laser'),
        DeclareLaunchArgument('imu_frame', default_value='imu_0_link'),
        DeclareLaunchArgument('scan_period', default_value='0.1'),
        DeclareLaunchArgument('deskew', default_value='false'),
        DeclareLaunchArgument('ground_constraint', default_value='false'),
        DeclareLaunchArgument('ground_baselink_height', default_value='0.30'),
        DeclareLaunchArgument(
            'publish_sensor_static_tf',
            default_value='false',
            description='Enable only if the rover bringup does not already publish sensor TF.'),
        DeclareLaunchArgument(
            'graph_pcd_cache_dir',
            default_value='/dev/shm/graph_slam_pcd_cache_live_real'),
        DeclareLaunchArgument(
            'graph_map_save_dir',
            default_value=PathJoinSubstitution(
                [LaunchConfiguration('repo_root'), 'output', 'husky_seyond_live_real'])),
        DeclareLaunchArgument(
            'graph_dem_output_dir',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('repo_root'), 'output',
                'husky_seyond_live_real', 'lunar_dem'])),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/ws/src/lidarslam_ros2/lidarslam/rviz/mapping.rviz'),
        timestamp_corrector,
        slam,
        base_to_lidar,
        base_to_imu,
    ])
