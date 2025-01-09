cmake_minimum_required(VERSION 3.14)
project(control_data_collecting_tool)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Configure Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)

# Find necessary ROS 2 packages
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Build the library as a plugin
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/data_collecting_area_selection.cpp
  src/data_collecting_goal_pose.cpp
)

# Link necessary libraries
ament_target_dependencies(${PROJECT_NAME}
  Qt5
  rviz_common
  pluginlib
  rclcpp
)

# Export the plugin XML file
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

# Install scripts
install(PROGRAMS
  scripts/data_collecting_pure_pursuit_trajectory_follower_acceleration_cmd.py
  scripts/data_collecting_pure_pursuit_trajectory_follower_actuation_cmd.py
  scripts/data_collecting_trajectory_publisher.py
  scripts/data_collecting_plotter.py
  scripts/data_collecting_rosbag_record.py
  scripts/data_collecting_data_counter.py
  scripts/data_collecting_base_node.py
  scripts/calibration/data_collecting_acceleration_cmd.py
  scripts/calibration/data_collecting_actuation_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other configurations and files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Build configuration for the package
ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
