cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_traffic_rule_filter_visualizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(autoware_boundary_departure_checker REQUIRED)
find_package(autoware_trajectory_traffic_rule_filter REQUIRED)

add_executable(amber_light_visualizer src/visualizer.cpp)
target_include_directories(amber_light_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(amber_light_visualizer
  rclcpp
  autoware_boundary_departure_checker
  autoware_trajectory_traffic_rule_filter
)

target_link_libraries(amber_light_visualizer
  ${OpenCV_LIBRARIES}
)

install(TARGETS amber_light_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
