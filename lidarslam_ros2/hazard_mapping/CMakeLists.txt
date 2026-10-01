cmake_minimum_required(VERSION 3.8)
project(hazard_mapping)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hazard_mapping_msgs REQUIRED)
find_package(lidarslam_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hazard_patch_node src/hazard_patch_node.cpp)
target_compile_features(hazard_patch_node PUBLIC cxx_std_17)
ament_target_dependencies(hazard_patch_node
  geometry_msgs
  hazard_mapping_msgs
  lidarslam_msgs
  rclcpp
  sensor_msgs
)

add_executable(global_costmap_composer src/global_costmap_composer.cpp)
target_compile_features(global_costmap_composer PUBLIC cxx_std_17)
ament_target_dependencies(global_costmap_composer
  geometry_msgs
  hazard_mapping_msgs
  lidarslam_msgs
  map_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  hazard_patch_node
  global_costmap_composer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
