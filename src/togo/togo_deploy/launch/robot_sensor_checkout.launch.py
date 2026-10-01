from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    lidar_topic = LaunchConfiguration("lidar_topic")

    # RViz config
    rviz_config_file = PathJoinSubstitution([FindPackageShare("togo_deploy"), "rviz", "robot_sensor_checkout.rviz"])

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        remappings=[("/iv_points", lidar_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/iv_points",
            description="Point cloud topic displayed by the Seyond RViz panel.",
        ),
        rviz_node,
    ])
