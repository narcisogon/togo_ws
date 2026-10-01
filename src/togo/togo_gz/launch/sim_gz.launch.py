from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,           # ← ADD THIS
    RegisterEventHandler,  # ← ADD THIS
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart  # ← ADD THIS
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Declare launch arguments ───────────────────────────────────────────
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, joint names in the controllers' \
        configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_x",
            default_value="0.0",
            description="Initial X-position of the robot when spawned into Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_y",
            default_value="0.0",
            description="Initial Y-position of the robot when spawned into Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_z",
            # FIX #2: Was 15.2 — robot was in freefall when controllers fired.
            # Set to just above terrain surface so hardware interfaces
            # are stable when the controller spawner runs.
            default_value="0.5",
            description="Initial Z-position of the robot when spawned into Gazebo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Flag to start RViz for robot and sensor checkout."
        )
    )
    # FIX #1: Expose controller timeout as a tunable argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawn_delay",
            default_value="10.0",
            description="Seconds to wait after world+robot load before "
                        "activating controllers. Increase if terrain mesh "
                        "is large and Gazebo loads slowly.",
        )
    )

    # ── Initialize arguments ───────────────────────────────────────────────
    tf_prefix             = LaunchConfiguration("tf_prefix")
    ns                    = LaunchConfiguration("ns")
    x                     = LaunchConfiguration("robot_x")
    y                     = LaunchConfiguration("robot_y")
    z                     = LaunchConfiguration("robot_z")
    rviz                  = LaunchConfiguration("rviz")
    controller_spawn_delay = LaunchConfiguration("controller_spawn_delay")

    # ── Packages ───────────────────────────────────────────────────────────
    pkg_deploy = FindPackageShare("togo_deploy")
    pkg_gazebo = FindPackageShare("togo_gz")

    # ── Config files ───────────────────────────────────────────────────────
    gz_bridge_config      = PathJoinSubstitution(
        [pkg_gazebo, "config", "bridge.yaml"])
    rgbd_point_fix_config = PathJoinSubstitution(
        [pkg_gazebo, "config", "rgbd_point_fix.yaml"])

    # ── Stage 1: Start world (fires immediately) ───────────────────────────
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "start_world.launch.py"]))
    )

    # ── Stage 2: Spawn robot + bridge (small delay after world) ───────────
    # FIX #1: Give Gazebo time to load the terrain mesh before spawning robot
    gz_sim_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity",      "togo",
            "-name",        "togo",
            "-topic",       "robot_description",
            "-x",           x,
            "-y",           y,
            "-z",           z,
            "-controller_manager", "controller_manager",
        ],
        output="screen",
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sim_bridge",
        parameters=[
            {
                "config_file": gz_bridge_config,
                "qos_overrides./tf_static.publisher.durability":
                    "transient_local",
                "qos_overrides./husky/sensors/seyond/points.publisher.reliability":
                    "best_effort",
            }
        ],
        output="screen",
    )

    # Delay robot spawn until world mesh has loaded
    # Increase world_load_delay if Gazebo is still slow to load your terrain
    world_load_delay = 6.0    # seconds — tune up if terrain loads slowly
    stage2 = TimerAction(
        period=world_load_delay,
        actions=[
            gz_sim_node,
            gz_bridge_node,
        ]
    )

    # ── Stage 3: Controllers (delayed until robot+hardware ready) ─────────
    # FIX #1 + #3: control_launch fires AFTER robot is spawned and settled.
    # controller_spawn_delay default = 10s (world_load_delay + 4s settle time)
    # If you still get timeouts increase controller_spawn_delay at launch:
    #   ros2 launch togo_gz togo_sim.launch.py controller_spawn_delay:=20.0
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_deploy, "launch", "control.launch.py"])),
        launch_arguments={
            "robot_description_package": "togo_gz",
            "robot_description_file":    "togo_gz.urdf.xacro",
            "is_sim":                    "true",
            "tf_prefix":                 tf_prefix,
            "ns":                        ns,
        }.items(),
    )

    stage3 = TimerAction(
        period=controller_spawn_delay,
        actions=[control_launch]
    )

    # ── Stage 4: Aux nodes (after controllers up) ──────────────────────────
    gz_rgbd_point_fixer = Node(
        package="togo_gz",
        executable="gz_rgbd_point_fixer",
        name="gz_rgbd_point_fixer",
        parameters=[rgbd_point_fix_config],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_deploy, "launch", "robot_sensor_checkout.launch.py"])
        ),
        launch_arguments={
            "lidar_topic": "/husky/sensors/seyond/points",
        }.items(),
        condition=IfCondition(rviz),
    )

    aux_delay = 14.0    # after controllers have had time to activate
    stage4 = TimerAction(
        period=aux_delay,
        actions=[
            gz_rgbd_point_fixer,
            rviz_launch,
        ]
    )

    # ── Assemble with namespace ────────────────────────────────────────────
    launches_nodes = [
        world_launch,   # t=0s   — Gazebo world + terrain mesh loads
        stage2,         # t=6s   — robot spawns, bridge starts
        stage3,         # t=10s  — controllers activate
        stage4,         # t=14s  — RGBD fixer + RViz
    ]

    ns_action = GroupAction(
        actions=[PushRosNamespace(ns)] + launches_nodes
    )

    return LaunchDescription(declared_arguments + [ns_action])
