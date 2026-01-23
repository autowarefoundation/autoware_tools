cmake_minimum_required(VERSION 3.8)
project(autoware_route_history)

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

#####
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/route_history/rviz_panel.hpp
)

add_library(autoware_route_history SHARED src/rviz_panel.cpp ${MOC_FILES})
target_include_directories(autoware_route_history PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(autoware_route_history
  pluginlib
  rviz_common
  Qt5
  geometry_msgs
  autoware_adapi_v1_msgs
)

install(TARGETS autoware_route_history
        EXPORT export_autoware_route_history
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
ament_export_targets(export_autoware_route_history)
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
