cmake_minimum_required(VERSION 3.14)
project(autoware_msg_bag_converter)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest)
  ament_add_pytest_test(autoware_msg_bag_converter
    test
  )
endif()

ament_python_install_package(${PROJECT_NAME})
ament_auto_package()
