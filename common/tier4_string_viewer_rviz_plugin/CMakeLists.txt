cmake_minimum_required(VERSION 3.14)
project(tier4_string_viewer_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/string_viewer_panel.hpp
  src/string_viewer_panel.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
    icons
    plugins
)
