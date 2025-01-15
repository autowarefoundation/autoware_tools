cmake_minimum_required(VERSION 3.5)
project(autoware_tp_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

install(PROGRAMS
  scripts/tp_collector.py
  scripts/tp_checker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
