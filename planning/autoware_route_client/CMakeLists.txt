cmake_minimum_required(VERSION 3.5)
project(autoware_route_client)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(std_msgs REQUIRED)

ament_auto_package()

install(PROGRAMS
  scripts/route_client.py
  DESTINATION lib/${PROJECT_NAME}
)
