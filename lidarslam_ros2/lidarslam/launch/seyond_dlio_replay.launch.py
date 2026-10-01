"""Replay real Husky/Seyond sensor data through DLIO and graph SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    source_launch = LaunchConfiguration('source_launch')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(source_launch),
        launch_arguments={
            'slam_param_file': LaunchConfiguration('slam_param_file'),
            'dlio_param_file': LaunchConfiguration('dlio_param_file'),
            'dlio_override_param_file': LaunchConfiguration('dlio_override_param_file'),
            'repo_root': LaunchConfiguration('repo_root'),
            'use_sim_time': 'true',
            'rviz': LaunchConfiguration('rviz'),
            'map_save_period': LaunchConfiguration('map_save_period'),
            'enable_map_save_pulse': LaunchConfiguration('enable_map_save_pulse'),
            'timed_cloud_input_topic': LaunchConfiguration('lidar_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'timed_cloud_scan_period': LaunchConfiguration('scan_period'),
            'timed_cloud_reverse_columns': LaunchConfiguration('reverse_columns'),
            'synthetic_timing': LaunchConfiguration('synthetic_timing'),
            'dlio_deskew': LaunchConfiguration('deskew'),
            'correct_cloud_slant': 'false',
            'enable_reference_path': 'false',
        }.items(),
    )

    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='replay_base_to_lidar_tf',
        arguments=[
            '--x', '0.400', '--y', '0.0', '--z', '0.313',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('lidar_frame'),
        ],
    )
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='replay_base_to_imu_tf',
        arguments=[
            '--x', '0.059', '--y', '0.0', '--z', '0.161275',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('imu_frame'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('source_launch', default_value='/ws/src/lidarslam_ros2/lidarslam/launch/seyond_dlio_slam.launch.py'),
        DeclareLaunchArgument('repo_root', default_value='/ws/src/lidarslam_ros2'),
        DeclareLaunchArgument('slam_param_file', default_value='/slam_ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_replay.yaml'),
        DeclareLaunchArgument('dlio_param_file', default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_graph.yaml'),
        DeclareLaunchArgument('dlio_override_param_file', default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_dlio_replay.yaml'),
        DeclareLaunchArgument('lidar_topic', default_value='/iv_points'),
        DeclareLaunchArgument('imu_topic', default_value='/husky/sensors/imu_0/data'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar3d_0_laser'),
        DeclareLaunchArgument('imu_frame', default_value='imu_0_link'),
        DeclareLaunchArgument('scan_period', default_value='0.1'),
        DeclareLaunchArgument('reverse_columns', default_value='false'),
        DeclareLaunchArgument('synthetic_timing', default_value='false'),
        DeclareLaunchArgument('deskew', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        slam,
        base_to_lidar,
        base_to_imu,
    ])
