cmake_minimum_required(VERSION 3.14)
project(autoware_lanelet2_map_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_find_build_dependencies()
find_package(nlohmann_json REQUIRED)

include_directories(
  src
)

file(GLOB_RECURSE autoware_lanelet2_map_validator_lib_src
  src/common/*.cpp
  src/validators/*.cpp
)
ament_auto_add_library(autoware_lanelet2_map_validator_lib SHARED
  ${autoware_lanelet2_map_validator_lib_src}
)

ament_auto_add_executable(autoware_lanelet2_map_validator src/main.cpp)
add_dependencies(autoware_lanelet2_map_validator autoware_lanelet2_map_validator_lib)
target_link_libraries(autoware_lanelet2_map_validator
  autoware_lanelet2_map_validator_lib
  nlohmann_json
)

if(BUILD_TESTING)
  function(add_validation_test VALIDATION_NAME)
    ament_add_ros_isolated_gtest(
      ${VALIDATION_NAME}_test
      test/src/test_${VALIDATION_NAME}.cpp
    )
    target_link_libraries(
      ${VALIDATION_NAME}_test
      autoware_lanelet2_map_validator_lib
      nlohmann_json
    )
  endfunction()

  add_validation_test(missing_regulatory_elements)
  add_validation_test(regulatory_element_details)
endif()

ament_auto_package()
