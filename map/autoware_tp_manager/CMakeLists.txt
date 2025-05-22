cmake_minimum_required(VERSION 3.5)
project(autoware_tp_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

find_package(Python3 COMPONENTS Development REQUIRED)

# Using pybind11_add_module caused lots of trouble in linking to pointcloud divider
# So I just used ament_auto_add_library
ament_auto_add_library(pcd_divider_wrapper MODULE src/python_bindings.cpp)
# This remove the prefix 'lib' of pcd_divider_wrapper
# Without this, the module is named libpcd_divider_wrapper.so
# It is not able to be import by the Python code
set_target_properties(pcd_divider_wrapper PROPERTIES PREFIX "")
# This is to fix "Python.h not found"
target_include_directories(pcd_divider_wrapper PRIVATE ${Python3_INCLUDE_DIRS})
# Linking to pointcloud divider
ament_target_dependencies(pcd_divider_wrapper autoware_pointcloud_divider)

# Copy the pcd_divider_wrapper.so to the install/${PROJECT_NAME}/lib/
install(TARGETS pcd_divider_wrapper
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tp_collector.py
  scripts/tp_checker.py
  scripts/tp_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_auto_package(INSTALL_TO_SHARE launch config)
ament_auto_package()
