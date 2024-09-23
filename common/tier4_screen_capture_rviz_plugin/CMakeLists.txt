cmake_minimum_required(VERSION 3.14)
project(tier4_screen_capture_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Capture.srv"
  DEPENDENCIES
  std_msgs
  ADD_LINTER_TESTS
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/screen_capture_panel.hpp
  src/screen_capture_panel.cpp
)

rosidl_get_typesupport_target(
  cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

target_link_libraries(${PROJECT_NAME}_lib
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
