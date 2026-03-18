cmake_minimum_required(VERSION 3.5)
project(autoware_vector_map_marker_bag_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
  scripts/insert_vector_map_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE)
