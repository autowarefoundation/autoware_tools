cmake_minimum_required(VERSION 3.14)
project(driving_environment_analyzer)

find_package(autoware_cmake REQUIRED)
add_compile_options(-Wno-deprecated-declarations)
autoware_package()
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/driving_environment_analyzer_node.hpp
  include/${PROJECT_NAME}/driving_environment_analyzer_rviz_plugin.hpp
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "driving_environment_analyzer::DrivingEnvironmentAnalyzerNode"
  EXECUTABLE driving_environment_analyzer_node
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  plugins
)
