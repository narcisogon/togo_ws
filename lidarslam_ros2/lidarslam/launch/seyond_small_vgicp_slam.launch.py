"""Integrated live Seyond scanmatcher frontend + graph SLAM backend launch."""

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
    raw_cloud_topic = LaunchConfiguration('raw_cloud_topic')
    xyzi_cloud_topic = LaunchConfiguration('xyzi_cloud_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    registration_method = LaunchConfiguration('registration_method')

    xyzi_adapter = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/add_intensity_to_cloud.py',
            '--ros-args',
            '-p', ['input_topic:=', raw_cloud_topic],
            '-p', ['output_topic:=', xyzi_cloud_topic],
            '-p', 'intensity:=1.0',
        ],
        output='screen',
    )

    scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        name='scan_matcher',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_param_file,
            {
                'use_sim_time': use_sim_time,
                'global_frame_id': 'map',
                'robot_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'registration_method': registration_method,
            },
        ],
        remappings=[
            ('/input_cloud', xyzi_cloud_topic),
            ('/imu', imu_topic),
            ('current_pose', '/small_vgicp/current_pose'),
            ('path', '/small_vgicp/path'),
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
        ],
    )

    pose_to_odom = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/pose_to_odom.py',
            '--ros-args',
            '-r', '__node:=small_vgicp_pose_to_odom',
            '-p', 'pose_topic:=/small_vgicp/current_pose',
            '-p', 'odom_topic:=/small_vgicp/odometry',
            '-p', 'child_frame_id:=base_link',
        ],
        output='screen',
    )

    reference_path = ExecuteProcess(
        cmd=[
            'python3', '/ws/src/lidarslam_ros2/scripts/togo/gazebo_pose_to_aligned_path.py',
            '--ros-args',
            '-r', '__node:=small_vgicp_reference_path_publisher',
            '-p', 'pose_topic:=/gazebo/dynamic_pose',
            '-p', 'frontend_odom_topic:=/small_vgicp/odometry',
            '-p', 'path_topic:=/reference/path',
            '-p', 'model_names:=a300-0000,a300_0000,a300,robot,base_link,chassis_link',
            '-p', 'fixed_frame:=map',
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
    )

    map_save_pulse = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'source /opt/ros/jazzy/setup.bash; '
            'source /ws/install/setup.bash; '
            'period=${MAP_SAVE_PERIOD:-60}; '
            'until ros2 service list | grep -qx /map_save; do sleep 1; done; '
            'while true; do ros2 service call /map_save std_srvs/srv/Empty; sleep "$period"; done'
        ],
        output='screen',
        additional_env={'MAP_SAVE_PERIOD': map_save_period},
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
            default_value='/ws/src/lidarslam_ros2/lidarslam/param/seyond_small_vgicp_slam.yaml',
            description='YAML containing small_vgicp frontend and graph SLAM backend parameters.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_save_period', default_value='60'),
        DeclareLaunchArgument(
            'registration_method',
            default_value='NDT',
            description='Frontend registration method. NDT is available by default; SMALL_VGICP requires small_gicp.',
        ),
        DeclareLaunchArgument(
            'raw_cloud_topic',
            default_value='/a300_0000/sensors/seyond_robin_w/scan/points',
        ),
        DeclareLaunchArgument(
            'xyzi_cloud_topic',
            default_value='/seyond_robin_w/points_xyzi',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/a300_0000/sensors/seyond_robin_w/imu',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        xyzi_adapter,
        graph_node,
        scanmatcher_node,
        pose_to_odom,
        reference_path,
        map_to_odom_tf,
        map_save_pulse,
        rviz_node,
    ])
