cmake_minimum_required(VERSION 3.8)
project(pose_replay_panel)

# RCLCPP/RCLCPP.HPP explicitly setting to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

find_package(geometry_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

find_package(pose_replay_interfaces REQUIRED)

#####
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/pose_replay_panel/pose_replay_panel.hpp
)

add_library(pose_replay_panel SHARED src/pose_replay_panel.cpp ${MOC_FILES})
target_include_directories(pose_replay_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_replay_panel
  pluginlib
  rviz_common
  Qt5
  pose_replay_interfaces
  geometry_msgs
  autoware_adapi_v1_msgs
)
install(TARGETS pose_replay_panel
        EXPORT export_pose_replay_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_pose_replay_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
#####

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
