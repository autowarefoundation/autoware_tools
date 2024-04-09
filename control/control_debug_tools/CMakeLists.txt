cmake_minimum_required(VERSION 3.5)
project(control_debug_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()


install(PROGRAMS
  scripts/consistency_checker.py
  DESTINATION lib/${PROJECT_NAME}
)
