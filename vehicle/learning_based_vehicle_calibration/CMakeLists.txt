cmake_minimum_required(VERSION 3.8)
project(learning_based_vehicle_calibration)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(builtin_interfaces REQUIRED)

install(
  PROGRAMS
  scripts/data_collection.py
  scripts/data_monitor.py
  scripts/neural_network_brake.py
  scripts/neural_network_throttle.py
  scripts/throttle_data_visualization.py
  scripts/rosbag_collection.sh


  DESTINATION lib/${PROJECT_NAME}

)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LongitudinalProgress.msg"
  "msg/LongitudinalProcesses.msg"
  DEPENDENCIES std_msgs

  )



# ament_export_dependencies(rosidl_default_runtime)


# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
