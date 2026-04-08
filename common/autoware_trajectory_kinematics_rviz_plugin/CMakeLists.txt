cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_kinematics_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Qt5 REQUIRED Core Widgets Charts)
set(QT_LIBRARIES Qt5::Widgets Qt5::Charts)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/trajectory_message_adapter.cpp
  src/trajectory_series_manager.cpp
  src/trajectory_kinematics_chart_view.cpp
  src/trajectory_kinematics_plot_widget.cpp
  src/trajectory_kinematics_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_kinematics_types
    test/test_kinematics_types.cpp
  )
  target_include_directories(test_${PROJECT_NAME}_kinematics_types PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(test_${PROJECT_NAME}_kinematics_types
    ${QT_LIBRARIES}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  icons
  plugins
)
