cmake_minimum_required(VERSION 3.14)
project(autoware_planning_data_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/base_evaluator.cpp
  src/open_loop_evaluator.cpp
  src/or_event_extractor.cpp
  src/or_scene_evaluator.cpp
  src/metrics/deviation_metrics.cpp
  src/metrics/trajectory_metrics.cpp
  src/utils/bag_utils.cpp
  src/utils/json_utils.cpp
  src/autoware_planning_data_analyzer_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::planning_data_analyzer::AutowarePlanningDataAnalyzerNode"
  EXECUTABLE autoware_planning_data_analyzer_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_metrics test/metrics/test_deviation_metrics.cpp)
  target_link_libraries(test_metrics ${PROJECT_NAME})

  ament_add_gtest(test_offline_evaluation test/test_bag_handler.cpp test/test_replay_evaluation.cpp)
  target_link_libraries(test_offline_evaluation ${PROJECT_NAME})
  ament_target_dependencies(test_offline_evaluation autoware_test_utils)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  scripts
)
