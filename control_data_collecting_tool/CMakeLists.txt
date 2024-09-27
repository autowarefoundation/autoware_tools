cmake_minimum_required(VERSION 3.14)
project(control_data_collecting_tool)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
  scripts/data_collecting_pure_pursuit_trajectory_follower.py
  scripts/data_collecting_trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/data_collecting_area_selection.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES})

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
