cmake_minimum_required(VERSION 3.8)
project(ma1_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stonefish_ros2 REQUIRED)

install(DIRECTORY
  launch
  scenarios
  data
  metadata
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
