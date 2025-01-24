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

install(PROGRAMS
  template/create_new_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_src "test/src/test_*.cpp")
  ament_auto_add_library(autoware_lanelet2_map_validator_test_lib ${test_src})

  target_include_directories(autoware_lanelet2_map_validator_test_lib PUBLIC
    test/src/include
    src/include)

  # test for general lanelet2 map validators
  function(add_validation_test TEST_FILE)
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    string(REPLACE "test_" "" VALIDATION_NAME ${TEST_NAME})
    ament_add_ros_isolated_gtest(
      ${VALIDATION_NAME}_test
      ${TEST_FILE}
    )
    target_link_libraries(
      ${VALIDATION_NAME}_test
      autoware_lanelet2_map_validator_test_lib
      autoware_lanelet2_map_validator_lib
    )
  endfunction()

  foreach(TEST_FILE ${test_src})
    add_validation_test(${TEST_FILE})
  endforeach()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  test/data
)
