cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_saver_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)


find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pointcloud_saver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=deprecated-copy -Wno-error=pedantic)
# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
