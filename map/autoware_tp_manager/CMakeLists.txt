cmake_minimum_required(VERSION 3.5)
project(autoware_tp_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

install(PROGRAMS
  scripts/tp_collector.py
  scripts/tp_checker.py
  scripts/tp_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_auto_package(INSTALL_TO_SHARE launch config)
ament_auto_package()
