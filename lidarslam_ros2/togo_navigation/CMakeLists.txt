cmake_minimum_required(VERSION 3.8)
project(togo_navigation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(slam_to_occupancy_grid src/slam_to_occupancy_grid.cpp)
target_compile_features(slam_to_occupancy_grid PUBLIC cxx_std_17)
ament_target_dependencies(slam_to_occupancy_grid
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(occupancy_grid_to_points src/occupancy_grid_to_points.cpp)
target_compile_features(occupancy_grid_to_points PUBLIC cxx_std_17)
ament_target_dependencies(occupancy_grid_to_points
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(local_hazard_grid src/local_hazard_grid.cpp)
target_compile_features(local_hazard_grid PUBLIC cxx_std_17)
ament_target_dependencies(local_hazard_grid
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(seyond_cloud_time_adapter src/seyond_cloud_time_adapter.cpp)
target_compile_features(seyond_cloud_time_adapter PUBLIC cxx_std_17)
ament_target_dependencies(seyond_cloud_time_adapter
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(odometry_to_tf src/odometry_to_tf.cpp)
target_compile_features(odometry_to_tf PUBLIC cxx_std_17)
ament_target_dependencies(odometry_to_tf
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(goal_safety_relay src/goal_safety_relay.cpp)
target_compile_features(goal_safety_relay PUBLIC cxx_std_17)
ament_target_dependencies(goal_safety_relay
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/debug_map_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  slam_to_occupancy_grid
  occupancy_grid_to_points
  local_hazard_grid
  seyond_cloud_time_adapter
  odometry_to_tf
  goal_safety_relay
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
