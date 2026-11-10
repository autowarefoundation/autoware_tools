cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_map_divider)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

find_package(yaml-cpp REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lanelet2_map_divider.cpp
  src/lanelet2_map_divider_node.cpp
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lanelet2_map_divider::Lanelet2MapDividerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lanelet2_map_divider test/test_lanelet2_map_divider.cpp)
  target_link_libraries(test_lanelet2_map_divider ${PROJECT_NAME} yaml-cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
