cmake_minimum_required(VERSION 3.5)
project(vehicle_calibration_scripts)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_package(
  INSTALL_TO_SHARE
    scripts
)
