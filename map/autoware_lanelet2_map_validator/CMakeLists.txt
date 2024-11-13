cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_map_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_find_build_dependencies()
find_package(nlohmann_json REQUIRED)

file(GLOB_RECURSE autoware_lanelet2_map_validator_lib_src
  src/common/*.cpp
  src/validators/*.cpp
)

ament_auto_add_library(autoware_lanelet2_map_validator_lib SHARED
  ${autoware_lanelet2_map_validator_lib_src}
)

target_include_directories(
  autoware_lanelet2_map_validator_lib PUBLIC
  src/include
)

ament_target_dependencies(autoware_lanelet2_map_validator_lib nlohmann_json)

ament_auto_add_executable(autoware_lanelet2_map_validator
  src/main.cpp
)

target_link_libraries(autoware_lanelet2_map_validator
  autoware_lanelet2_map_validator_lib
)

if(BUILD_TESTING)
  find_package(ament_index_cpp REQUIRED)

  # test for json processing
  ament_add_ros_isolated_gtest(
    json_processing_test
    test/src/test_json_processing.cpp
  )

  target_link_libraries(
    json_processing_test
    autoware_lanelet2_map_validator_lib
    ament_index_cpp::ament_index_cpp
  )

  # test for general lanelet2 map validators
  function(add_validation_test VALIDATION_NAME)
    ament_add_ros_isolated_gtest(
      ${VALIDATION_NAME}_test
      test/src/test_${VALIDATION_NAME}.cpp
    )
    target_link_libraries(
      ${VALIDATION_NAME}_test
      autoware_lanelet2_map_validator_lib
    )
  endfunction()

  add_validation_test(missing_regulatory_elements_for_stop_lines)
  add_validation_test(missing_regulatory_elements_for_crosswalks)
  add_validation_test(missing_regulatory_elements_for_traffic_lights)
  add_validation_test(regulatory_elements_details_for_traffic_lights)
  add_validation_test(regulatory_elements_details_for_crosswalks)
  add_validation_test(regulatory_element_details)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  test/data
)
