cmake_minimum_required(VERSION 3.8)
project(togo_gz)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(gz_rgbd_point_fixer src/gz_rgbd_point_fixer.cpp)
ament_target_dependencies(gz_rgbd_point_fixer rclcpp tf2_sensor_msgs tf2_geometry_msgs)

# install
install(
  DIRECTORY config launch urdf worlds models media   
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS gz_rgbd_point_fixer DESTINATION lib/${PROJECT_NAME})

ament_package()
