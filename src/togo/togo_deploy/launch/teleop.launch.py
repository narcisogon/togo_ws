from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If the robot is running in simulation, use the published clock",
        )
    )

    # launch configurations
    ns = LaunchConfiguration("ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # include packages
    pkg_togo_deploy = FindPackageShare("togo_deploy")

    # config files
    config_teleop_joy = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "teleop_joy.yaml"])
    config_interactive_markers = PathJoinSubstitution(
        [pkg_togo_deploy, "config", "husky", "teleop_interactive_markers.yaml"]
    )
    config_twist_mux = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "twist_mux.yaml"])

    # Linux joystick
    node_joy = Node(
        package="joy_linux",
        executable="joy_linux_node",
        namespace=ns,
        output="screen",
        name="joy_node",
        parameters=[
            config_teleop_joy,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("joy", "joy_teleop/joy"),
            ("joy/set_feedback", "joy_teleop/joy/set_feedback"),
        ],
        respawn=True,
    )

    # teleop joystick
    node_teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        namespace=ns,
        output="screen",
        name="teleop_twist_joy_node",
        parameters=[
            config_teleop_joy,
            {"use_sim_time": use_sim_time},
            {"publish_stamped_twist": True},
        ],
        remappings=[
            ("joy", "joy_teleop/joy"),
            ("cmd_vel", "joy_teleop/cmd_vel"),
        ],
    )

    # interactive marker server
    node_interactive_marker_twist_server = Node(
        package="interactive_marker_twist_server",
        executable="marker_server",
        namespace=ns,
        name="twist_server_node",
        remappings=[
            ("cmd_vel", "twist_marker_server/cmd_vel"),
            ("twist_server/feedback", "twist_marker_server/feedback"),
            ("twist_server/update", "twist_marker_server/update"),
        ],
        parameters=[
            config_interactive_markers,
            {"use_sim_time": use_sim_time},
            {"use_stamped_msgs": True},
        ],
        output="screen",
    )

    # twist mux node; prioritizes topics
    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        namespace=ns,
        output="screen",
        remappings=[
            (
                "cmd_vel_out",
                "platform_velocity_controller/cmd_vel",
            ),
            ("/diagnostics", "diagnostics"),
            ("/platform/emergency_stop", "/husky/platform/emergency_stop"),
        ],
        parameters=[
            config_twist_mux,
            {"use_sim_time": use_sim_time},
            {"use_stamped": True},
        ],
    )

    nodes = [
        node_joy,
        node_teleop_twist_joy,
        node_interactive_marker_twist_server,
        node_twist_mux,
    ]

    return LaunchDescription(declared_arguments + nodes)
