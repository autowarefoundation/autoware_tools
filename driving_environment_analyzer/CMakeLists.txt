cmake_minimum_required(VERSION 3.14)
project(driving_environment_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "driving_environment_analyzer::DrivingEnvironmentAnalyzer"
  EXECUTABLE driving_environment_analyzer
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
