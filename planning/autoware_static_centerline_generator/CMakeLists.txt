cmake_minimum_required(VERSION 3.14)
project(autoware_static_centerline_generator)

find_package(autoware_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glog REQUIRED)

rosidl_generate_interfaces(
  autoware_static_centerline_generator
  "srv/LoadMap.srv"
  "srv/PlanRoute.srv"
  "srv/PlanPath.srv"
  "msg/PointsWithLaneId.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

autoware_package()

ament_auto_add_executable(main
  src/main.cpp
  src/static_centerline_generator_node.cpp
  src/centerline_source/optimization_trajectory_based_centerline.cpp
  src/centerline_source/bag_ego_trajectory_based_centerline.cpp
  src/utils.cpp
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(main
    autoware_static_centerline_generator "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target autoware_static_centerline_generator "rosidl_typesupport_cpp")
    target_link_libraries(main "${cpp_typesupport_target}" glog::glog)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    rviz
)

target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_TESTING)
  # 1. default settings
  # goal_method=None
  # default settings (centerline_source=optimization_trajectory_base, mode=AUTO)
  add_launch_test(
    test/test_static_centerline_generator_launch.test.py
    TIMEOUT "30"
  )
  # 2. mode=AUTO
  # 2.1. goal_method=path_generator, centerline_source=optimization_trajectory_base
  #      various cases with start/end pose
  add_launch_test(
    test/test_static_centerline_generator_path_generator_case1_launch.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_static_centerline_generator_path_generator_case2_launch.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_static_centerline_generator_path_generator_case3_launch.test.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_static_centerline_generator_path_generator_case4_launch.test.py
    TIMEOUT "30"
  )
  # 2.2. goal_method=behavior_path_planner, centerline_source=optimization_trajectory_base
  add_launch_test(
    test/test_static_centerline_generator_behavior_path_planner_launch.test.py
    TIMEOUT "30"
  )
  # 3. mode=GUI
  # centerline_source=bag_ego_trajectory_base
  add_launch_test(
    test/test_static_centerline_generator_gui_launch.test.py
    TIMEOUT "30"
  )
  install(DIRECTORY
    test/data/
    DESTINATION share/${PROJECT_NAME}/test/data/
  )
endif()

install(PROGRAMS
  scripts/app.py
  scripts/centerline_updater_helper.py
  scripts/show_lanelet2_map_diff.py
  DESTINATION lib/${PROJECT_NAME}
)
