"""Integrated live Seyond Faster-LIO frontend + graph SLAM backend launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_param_file = LaunchConfiguration('slam_param_file')
    faster_lio_param_file = LaunchConfiguration('faster_lio_param_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    map_save_period = LaunchConfiguration('map_save_period')
    enable_map_save_pulse = LaunchConfiguration('enable_map_save_pulse')
    frontend_path_min_distance = LaunchConfiguration('frontend_path_min_distance')
    publish_static_map_to_odom = LaunchConfiguration('publish_static_map_to_odom')
    rviz_config = LaunchConfiguration('rviz_config')
    timed_cloud_scan_period = LaunchConfiguration('timed_cloud_scan_period')
    timed_cloud_reverse_columns = LaunchConfiguration('timed_cloud_reverse_columns')

    timed_cloud_adapter = Node(
        package='togo_navigation',
        executable='seyond_cloud_time_adapter',
        name='seyond_cloud_time_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_cloud_topic': '/a300_0000/sensors/seyond_robin_w/scan/points'},
            {'output_cloud_topic': '/a300_0000/sensors/seyond_robin_w/scan/points_timed'},
            {'scan_period_sec': timed_cloud_scan_period},
            {'reverse_column_time': timed_cloud_reverse_columns},
        ],
    )

    faster_lio_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='laserMapping',
        output='screen',
        emulate_tty=True,
        parameters=[
            faster_lio_param_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('Odometry', '/faster_lio/odometry'),
            ('cloud_registered', '/faster_lio/cloud_registered'),
            ('cloud_registered_body', '/faster_lio/cloud_registered_body'),
            ('cloud_registered_effect_world', '/faster_lio/cloud_registered_effect_world'),
            ('path', '/faster_lio/path'),
            ('frontend_diagnostics', '/faster_lio/frontend_diagnostics'),
        ],
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
            {'odom_frame_id': 'camera_init'},
        ],
        remappings=[
            ('odom_input', '/faster_lio/odometry'),
            ('cloud_input', '/faster_lio/cloud_registered'),
        ],
    )

    frontend_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/odom_to_path.py',
            '--ros-args',
            '-r', '__node:=faster_lio_path_publisher',
            '-p', 'odom_topic:=/faster_lio/odometry',
            '-p', 'path_topic:=/faster_lio/path_simple',
            '-p', 'fixed_frame:=camera_init',
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
            '-p', 'frontend_odom_topic:=/faster_lio/odometry',
            '-p', 'path_topic:=/reference/path',
            '-p', 'model_names:=a300-0000,a300_0000,a300,robot,base_link,chassis_link',
            '-p', 'fixed_frame:=camera_init',
            '-p', 'max_poses:=12000',
            '-p', 'min_distance_m:=0.02',
            '-p', 'extra_yaw_offset_deg:=0.0',
        ],
        output='screen',
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'camera_init',
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
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_param_file',
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_live_slam.yaml',
            description='Backend graph SLAM parameters.',
        ),
        DeclareLaunchArgument(
            'faster_lio_param_file',
            default_value='/ws/src/lidarslam_ros2/faster_lio/faster-lio/config/seyond_robin_w.yaml',
            description='Seyond Faster-LIO frontend parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument('enable_map_save_pulse', default_value='false'),
        DeclareLaunchArgument('frontend_path_min_distance', default_value='0.005'),
        DeclareLaunchArgument(
            'timed_cloud_scan_period',
            default_value='0.0666666667',
            description='Synthetic per-point timing span for the 15 Hz organized Gazebo scan.',
        ),
        DeclareLaunchArgument(
            'timed_cloud_reverse_columns',
            default_value='false',
            description='Reverse synthetic per-column time if the simulated scan ordering is opposite.',
        ),
        DeclareLaunchArgument(
            'publish_static_map_to_odom',
            default_value='false',
            description='Fallback identity map->camera_init TF. Keep false when graph_based_slam publishes dynamic correction.',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/ws/src/lidarslam_ros2/lidarslam/rviz/mapping.rviz',
            description='RViz config with /modified_map, /map, paths, and TF displays.',
        ),
        timed_cloud_adapter,
        faster_lio_node,
        graph_node,
        frontend_path,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])
