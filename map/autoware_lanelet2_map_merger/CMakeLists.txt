cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_map_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lanelet2_map_merger.cpp
  src/lanelet2_map_merger_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lanelet2_map_merger::Lanelet2MapMergerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lanelet2_map_merger test/test_lanelet2_map_merger.cpp)
  target_link_libraries(test_lanelet2_map_merger ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
