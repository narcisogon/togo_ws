import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    # initialize arguments
    world_pkg = LaunchConfiguration("world_pkg").perform(context)
    world = LaunchConfiguration("world").perform(context)
    # need to perform context to make them strings

    # world file
    world_file = os.path.join(get_package_share_directory(world_pkg), "worlds", world)
    # this needs to be a string object; PathJoinSubstituion doesn't quite work the same way

    # start Gazebo with world
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                "-r -s -v 4 " + world_file
            ],  # -r to unpause the sim (required to load controls), -v verbose, 0-4 verbosity level with 4 as debug
            "on_exit_shutdown": "True",
        }.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return [gz_sim_launch, clock_bridge]


def generate_launch_description():
    # declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_pkg",
            default_value="togo_gz",
            description="Name of the package that has the world file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="togo_parking_lot_world.sdf",
            description="Name of the world file; must exist in worlds/ directory of world_pkg",
        )
    )

    # get nodes and create launch description
    nodes = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(declared_arguments)
    ld.add_action(nodes)
    return ld
