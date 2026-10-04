"""Nav2 planning/driving consuming the graph backend's authoritative /map.

Graph SLAM owns map -> odom; DLIO owns odom -> base_link. Terrain processing
and both inflation bands run inside graph_based_slam.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def nav2_node(package, executable, name, params_file, use_sim_time, remappings=None):
    return Node(
        package=package, executable=executable, name=name, output='screen',
        parameters=[params_file, {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=remappings or [],
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'velocity_smoother',
    ]
    default_params = PathJoinSubstitution([
        FindPackageShare('togo_navigation'), 'config', 'nav2_slam_params.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('togo_navigation'), 'rviz', 'rover_nav_debug.rviz',
    ])
    command_input = [('cmd_vel', '/nav2/cmd_vel')]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        nav2_node('togo_navigation', 'goal_safety_relay', 'goal_safety_relay',
                  params_file, use_sim_time),
        nav2_node('nav2_controller', 'controller_server', 'controller_server',
                  params_file, use_sim_time, command_input),
        nav2_node('nav2_smoother', 'smoother_server', 'smoother_server',
                  params_file, use_sim_time),
        nav2_node('nav2_planner', 'planner_server', 'planner_server',
                  params_file, use_sim_time),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                  params_file, use_sim_time, command_input),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
                  params_file, use_sim_time),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower',
                  params_file, use_sim_time),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  params_file, use_sim_time,
                  [('cmd_vel', '/nav2/cmd_vel'), ('cmd_vel_smoothed', '/nav2/cmd_vel_smoothed')]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'node_names': lifecycle_nodes,
            }],
        ),
        Node(
            package='rviz2', executable='rviz2', name='nav2_rviz', output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
