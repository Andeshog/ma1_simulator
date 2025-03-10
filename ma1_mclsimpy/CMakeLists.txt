cmake_minimum_required(VERSION 3.8)
project(ma1_mclsimpy)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_with_waves.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
