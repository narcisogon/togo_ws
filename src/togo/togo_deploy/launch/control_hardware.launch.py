from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for the hardware robot",
        )
    )

    # initialize arguments
    ns = LaunchConfiguration("ns")

    # launch control for hardware
    launch_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("togo_deploy"), "launch", "control.launch.py"])
        ),
        launch_arguments={
            "use_fake_hardware": "false",
            "ns": ns,
        }.items(),
    )

    # launch teleop
    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("togo_deploy"), "launch", "teleop.launch.py"])
        ),
        launch_arguments={
            "ns": ns,
        }.items(),
    )

    ns_action = GroupAction(actions=[PushRosNamespace(ns)] + [launch_control, launch_teleop])

    return LaunchDescription(declared_arguments + [ns_action])
