cmake_minimum_required(VERSION 3.14)
project(autoware_planning_data_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/base_evaluator.cpp
  src/open_loop_evaluator.cpp
  src/metrics/deviation_metrics.cpp
  src/metrics/epdms/aggregation/epdms_aggregation.cpp
  src/metrics/epdms/subscores/ego_progress.cpp
  src/metrics/epdms/subscores/driving_direction_compliance.cpp
  src/metrics/epdms/subscores/history_comfort.cpp
  src/metrics/epdms/subscores/lane_keeping.cpp
  src/metrics/geometry/metric_utils.cpp
  src/metrics/epdms/subscores/drivable_area_compliance.cpp
  src/metrics/epdms/subscores/extended_comfort.cpp
  src/metrics/epdms/subscores/ttc_within_bound.cpp
  src/metrics/epdms/subscores/no_at_fault_collision.cpp
  src/metrics/epdms/subscores/traffic_light_compliance.cpp
  src/metrics/trajectory_metrics.cpp
  src/utils/bag_utils.cpp
  src/utils/json_utils.cpp
  src/utils/path_utils.cpp
  src/autoware_planning_data_analyzer_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::planning_data_analyzer::AutowarePlanningDataAnalyzerNode"
  EXECUTABLE autoware_planning_data_analyzer_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_metrics
    test/metrics/test_deviation_metrics.cpp
    test/metrics/test_driving_direction_compliance.cpp
    test/metrics/test_epdms_aggregation.cpp
    test/metrics/test_lane_keeping.cpp
    test/metrics/test_ego_progress.cpp
    test/metrics/test_drivable_area_compliance.cpp
    test/metrics/test_extended_comfort.cpp
    test/metrics/test_ttc_within_bound.cpp
    test/metrics/test_no_at_fault_collision.cpp
    test/metrics/test_traffic_light_compliance.cpp
  )

  target_include_directories(test_metrics
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  
  target_link_libraries(test_metrics ${PROJECT_NAME})

  ament_add_gtest(test_offline_evaluation
    test/test_bag_handler.cpp
    test/test_replay_evaluation.cpp
    test/test_open_loop_gt_source_mode.cpp
  )

  target_include_directories(test_offline_evaluation
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_link_libraries(test_offline_evaluation ${PROJECT_NAME})
  ament_target_dependencies(test_offline_evaluation autoware_test_utils)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
