from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # DECLARE LAUNCH ARGUMENTS
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_seyond",
            default_value="true",
            description="Flag to start the Seyond LIDAR",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_front_oakd",
            default_value="true",
            description="Flag to start the front OAK-D Camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rear_oakd",
            default_value="true",
            description="Flag to start the rear OAK-D Camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_fixposition",
            default_value="true",
            description="Flag to start the FixPosition INS",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_phidgets",
            default_value="true",
            description="Flag to start the Phidgets IMU. "
            "Will also launch IMU filter and localization, since they depend on IMU data.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Flag to start RViz for robot and sensor checkout."
        )
    )

    # Initialize Arguments
    launch_seyond = LaunchConfiguration("launch_seyond")
    launch_front_oakd = LaunchConfiguration("launch_front_oakd")
    launch_rear_oakd = LaunchConfiguration("launch_rear_oakd")
    launch_fixposition = LaunchConfiguration("launch_fixposition")
    launch_phidgets = LaunchConfiguration("launch_phidgets")
    rviz = LaunchConfiguration("rviz")
    default_ns = "husky"
    sensor_ns = "husky/sensors"

    # INCLUDE PACKAGES
    pkg_togo_deploy = FindPackageShare("togo_deploy")

    # SENSOR CONFIGS
    config_seyond = PathJoinSubstitution([pkg_togo_deploy, "config", "sensors", "seyond_config.yaml"])
    config_front_oakd = PathJoinSubstitution([pkg_togo_deploy, "config", "sensors", "front_oakd_config.yaml"])
    config_rear_oakd = PathJoinSubstitution([pkg_togo_deploy, "config", "sensors", "rear_oakd_config.yaml"])
    config_ins = PathJoinSubstitution([pkg_togo_deploy, "config", "sensors", "ins_config.yaml"])
    config_phidgets = PathJoinSubstitution([pkg_togo_deploy, "config", "sensors", "phidgets_imu_config.yaml"])
    # SENSOR DEPENDENT CONFIGS
    config_localization = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "localization.yaml"])
    config_imu_filter = PathJoinSubstitution([pkg_togo_deploy, "config", "husky", "imu_filter.yaml"])

    # SENSOR NODES

    # Seyond LIDAR
    seyond_node = Node(
        package="seyond",
        executable="seyond_node",
        namespace=sensor_ns,
        parameters=[
            {"config_path": config_seyond},
        ],
        condition=IfCondition(launch_seyond),
    )

    # OAK-D Front Camera
    front_depthai_oakd_node = ComposableNode(
        package="depthai_ros_driver",
        name="front_oakd",
        namespace=sensor_ns,
        plugin="depthai_ros_driver::Camera",
        parameters=[config_front_oakd],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=IfCondition(launch_front_oakd),
    )

    front_depthai_pcl_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzNode",
        name="front_point_cloud_xyz_node",
        namespace=sensor_ns,
        remappings=[
            ("image_rect", "/husky/sensors/front_oakd/stereo/image_raw"),
            ("camera_info", "/husky/sensors/front_oakd/stereo/camera_info"),
            ("points", "/husky/sensors/front_oakd/points"),
        ],
        condition=IfCondition(launch_front_oakd),
    )

    front_image_processing_container = ComposableNodeContainer(
        name="front_image_processing_container",
        package="rclcpp_components",
        namespace=sensor_ns,
        executable="component_container",
        composable_node_descriptions=[
            front_depthai_oakd_node,
            front_depthai_pcl_node,
        ],
        output="screen",
        condition=IfCondition(launch_front_oakd),
    )

    # OAK-D Rear Camera
    rear_depthai_oakd_node = ComposableNode(
        package="depthai_ros_driver",
        name="rear_oakd",
        namespace=sensor_ns,
        plugin="depthai_ros_driver::Camera",
        parameters=[config_rear_oakd],
        extra_arguments=[{"use_intra_process_comms": True}],
        condition=IfCondition(launch_rear_oakd),
    )

    rear_depthai_pcl_node = ComposableNode(
        package="depth_image_proc",
        plugin="depth_image_proc::PointCloudXyzNode",
        name="rear_point_cloud_xyz_node",
        namespace=sensor_ns,
        remappings=[
            ("image_rect", "/rear_oakd/stereo/image_raw"),
            ("camera_info", "/rear_oakd/stereo/camera_info"),
            ("points", "/rear_oakd/points"),
        ],
        condition=IfCondition(launch_rear_oakd),
    )

    rear_image_processing_container = ComposableNodeContainer(
        name="rear_image_processing_container",
        package="rclcpp_components",
        namespace=sensor_ns,
        executable="component_container",
        composable_node_descriptions=[
            rear_depthai_oakd_node,
            rear_depthai_pcl_node,
        ],
        output="screen",
        condition=IfCondition(launch_rear_oakd),
    )

    # FixPosition INS
    fixposition_node = Node(
        package="fixposition_driver_ros2",
        executable="fixposition_driver_ros2_exec",
        name="fixposition_driver",
        namespace=sensor_ns,
        output="screen",
        parameters=[config_ins],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        condition=IfCondition(launch_fixposition),
    )

    # Phidgets IMU
    phidgets_node = ComposableNode(
        package="phidgets_spatial",
        plugin="phidgets::SpatialRosI",
        name="phidgets_spatial",
        namespace=sensor_ns,
        parameters=[config_phidgets],
        remappings=[
            ("imu/data_raw", "/husky/sensors/imu_0/data_raw"),
            ("imu/is_calibrated", "/husky/sensors/imu_0/is_calibrated"),
            ("imu/mag", "/husky/sensors/imu_0/mag"),
        ],
        condition=IfCondition(launch_phidgets),
    )

    # IMU Filter
    imu_filter_node = ComposableNode(
        package="imu_filter_madgwick",
        plugin="ImuFilterMadgwickRos",
        name="imu_filter_madgwick",
        namespace=default_ns,
        parameters=[config_imu_filter],
        remappings=[
            ("imu/data", "sensors/imu_0/data"),
            ("imu/data_raw", "sensors/imu_0/data_raw"),
            ("imu/mag", "sensors/imu_0/mag"),
        ],
        condition=IfCondition(launch_phidgets),
    )

    imu_filter_container = ComposableNodeContainer(
        name="imu_filter_container",
        namespace=sensor_ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            phidgets_node,
            imu_filter_node,
        ],
        output="screen",
        condition=IfCondition(launch_phidgets),
    )

    # Localization
    node_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        namespace=default_ns,
        output="screen",
        parameters=[config_localization],
        remappings=[
            ("odometry/filtered", "platform/odom/filtered"),
            ("/diagnostics", "diagnostics"),
        ],
        condition=IfCondition(launch_phidgets),
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_togo_deploy, "launch", "robot_sensor_checkout.launch.py"])
        ),
        condition=IfCondition(rviz),
    )

    # LAUNCH DESCRIPTION
    sensor_launches = [
        seyond_node,
        front_image_processing_container,
        rear_image_processing_container,
        fixposition_node,
        imu_filter_container,
    ]
    sensor_dependent_nodes = [node_localization]

    return LaunchDescription(declared_arguments + sensor_launches + sensor_dependent_nodes + [rviz_launch])
