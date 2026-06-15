cmake_minimum_required(VERSION 3.14)
project(autoware_position_error_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_WIDGETS_LIB Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Wno-deprecated-declarations)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/position_error_evaluator_tool.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_WIDGETS_LIB}
)

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=deprecated-copy -Wno-error=pedantic)
# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
