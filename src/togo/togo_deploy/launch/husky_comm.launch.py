from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # initialize arguments
    ns = LaunchConfiguration("ns")
    default_ns = "husky"
    vcan1_ns = "husky/platform/bms"
    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix = tf_prefix  # dummy use to stop precommit complaining about unused variables
    # tf_prefix will get passed into nodes implicitly

    # include packages
    pkg_togo_deploy = FindPackageShare("togo_deploy")
    pkg_clearpath_diagnostics = FindPackageShare("clearpath_diagnostics")
    pkg_clearpath_ros2_socketcan_interface = FindPackageShare("clearpath_ros2_socketcan_interface")
    pkg_canopen_inventus_bringup = FindPackageShare("canopen_inventus_bringup")

    # config files
    motor_driver_config = PathJoinSubstitution([pkg_togo_deploy, "config", "motor_driver.yaml"])
    # path to the robot.yaml file
    setup_path = PathJoinSubstitution([pkg_togo_deploy, "config", "husky"])
    # diagnostics configs
    diagnostic_updater_params = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "diagnostic_updater.yaml"])
    diagnostic_aggregator_params = PathJoinSubstitution(
        [pkg_togo_deploy, "config", "husky", "diagnostic_aggregator.yaml"]
    )
    # foxglove configs
    foxglove_bridge_params = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "foxglove_bridge.yaml"])

    # launch files
    # diagnostics
    launch_file_diagnostics = PathJoinSubstitution([pkg_clearpath_diagnostics, "launch", "diagnostics.launch.py"])
    # foxglove
    launch_file_foxglove = PathJoinSubstitution([pkg_clearpath_diagnostics, "launch", "foxglove_bridge.launch.py"])
    # vcan0
    launch_file_receiver = PathJoinSubstitution(
        [pkg_clearpath_ros2_socketcan_interface, "launch", "receiver.launch.py"]
    )
    launch_file_sender = PathJoinSubstitution([pkg_clearpath_ros2_socketcan_interface, "launch", "sender.launch.py"])
    # vcan1
    launch_file_inventus = PathJoinSubstitution([pkg_canopen_inventus_bringup, "launch", "inventus.launch.py"])

    # include launch files
    launch_diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_diagnostics),
        launch_arguments={
            "namespace": default_ns,
            "updater_parameters": diagnostic_updater_params,
            "aggregator_parameters": diagnostic_aggregator_params,
        }.items(),
    )
    launch_foxglove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_foxglove),
        launch_arguments={
            "namespace": default_ns,
            "parameters": foxglove_bridge_params,
        }.items(),
    )
    launch_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_receiver),
        launch_arguments={
            "namespace": default_ns,
            "interface": "vcan0",
            "from_can_bus_topic": "vcan0/rx",
            "enable_can_fd": "false",
            "interval_sec": "0.01",
            "use_bus_time": "false",
            "filters": "0:0",
            "auto_configure": "true",
            "auto_activate": "true",
            "timeout": "5.0",
            "transition_attempts": "3",
        }.items(),
    )
    launch_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_sender),
        launch_arguments={
            "namespace": default_ns,
            "interface": "vcan0",
            "to_can_bus_topic": "vcan0/tx",
            "enable_can_fd": "false",
            "interval_sec": "0.01",
            "auto_configure": "true",
            "auto_activate": "true",
            "timeout": "5.0",
            "transition_attempts": "3",
        }.items(),
    )
    launch_inventus = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_inventus),
        launch_arguments={
            "namespace": vcan1_ns,
            "interface": "vcan1",
            "battery_count": "2",
            "master_id": "49",
            "battery_0_id": "49",
            "battery_1_id": "50",
            "battery_2_id": "51",
            "battery_3_id": "52",
            "battery_4_id": "53",
            "battery_5_id": "54",
        }.items(),
    )

    # nodes
    node_wireless_watcher = Node(
        name="wireless_watcher",
        executable="wireless_watcher",
        package="wireless_watcher",
        namespace=default_ns,
        output="screen",
        remappings=[
            ("/diagnostics", "diagnostics"),
        ],
        parameters=[
            {
                "hz": 1.0,
                "dev": "",
                "connected_topic": "platform/wifi_connected",
                "connection_topic": "platform/wifi_status",
            },
        ],
    )

    node_battery_state_control = Node(
        name="battery_state_control",
        executable="battery_state_control",
        package="clearpath_hardware_interfaces",
        namespace=default_ns,
        output="screen",
        arguments=[
            "-s",
            setup_path,
        ],
    )

    node_micro_ros_agent = Node(
        name="micro_ros_agent",
        executable="micro_ros_agent",
        package="micro_ros_agent",
        namespace=default_ns,
        output="screen",
        arguments=[
            "udp4",
            "--port",
            "11411",
        ],
    )

    node_lighting_node = Node(
        name="lighting_node",
        executable="lighting_node",
        package="clearpath_hardware_interfaces",
        namespace=default_ns,
        output="screen",
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/husky/platform/motors/system_protection", "/platform/motors/system_protection"),
        ],
        parameters=[
            {
                "platform": "a300",
            },
        ],
    )

    node_lynx_control = Node(
        name="lynx_control",
        executable="lynx_motor_driver",
        package="lynx_motor_driver",
        namespace=default_ns,
        output="screen",
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/husky/platform/motors/cmd", "/platform/motors/cmd"),
            ("/husky/platform/motors/feedback", "/platform/motors/feedback"),
            ("/husky/platform/motors/status", "/platform/motors/status"),
            ("/husky/platform/motors/system_protection", "/platform/motors/system_protection"),
        ],
        parameters=[
            ParameterFile(motor_driver_config, allow_substs=True),
        ],
    )

    node_a300_fan_control = Node(
        name="a300_fan_control",
        executable="fan_control_node",
        package="clearpath_hardware_interfaces",
        namespace=default_ns,
        output="screen",
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("/husky/platform/motors/status", "/platform/motors/status"),
        ],
    )

    node_a300_sw_low_soc_cutoff = Node(
        name="a300_sw_low_soc_cutoff",
        executable="sw_low_soc_cutoff_node",
        package="clearpath_hardware_interfaces",
        namespace=default_ns,
        output="screen",
    )

    node_pinout_control_node = Node(
        name="pinout_control_node",
        executable="pinout_control_node",
        package="clearpath_hardware_interfaces",
        namespace=default_ns,
        output="screen",
        parameters=[
            {
                "platform": "a300",
            },
        ],
    )

    # processes
    # note this will be incorrectly namespaced if a namespace is pushed for this file
    # this should be converted to a node so it picks up the namespace
    # IMPORTANT: The ROS_DOMAIN_ID must be 0 when calling the service to match the MCU's
    # initial ROS_DOMAIN_ID. Temporarily override it here just for that purpose.
    process_configure_mcu = ExecuteProcess(
        shell=True,
        cmd=[
            [
                "export ROS_DOMAIN_ID=0;",
            ],
            [
                FindExecutable(name="ros2"),
                " service call /platform/mcu/configure",
                " clearpath_platform_msgs/srv/ConfigureMcu",
                ' "{domain_id: 76,',
                f" robot_namespace: '{default_ns}'}}\"",
            ],
        ],
    )

    launches = [launch_diagnostics, launch_foxglove, launch_receiver, launch_sender, launch_inventus]
    nodes = [
        node_wireless_watcher,
        node_battery_state_control,
        # NOTE for now, the micro-ROS agent gets started in its own docker container
        # due to dependency versioning issues when installing in the Togo docker image.
        # Until this is fixed, there is no need to launch the micro-ROS agent here,
        # with the remainder of the Husky hardware comm nodes.
        # node_micro_ros_agent,
        node_lighting_node,
        node_lynx_control,
        node_a300_fan_control,
        node_a300_sw_low_soc_cutoff,
        node_pinout_control_node,
    ]
    processes = [process_configure_mcu]

    ns_action = GroupAction(actions=[PushRosNamespace(ns)] + launches + nodes + processes)

    return LaunchDescription(declared_arguments + [ns_action])
