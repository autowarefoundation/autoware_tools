cmake_minimum_required(VERSION 3.5)
project(autoware_dependency_checker)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  scripts/dependency_checker.sh
  DESTINATION lib/${PROJECT_NAME})
