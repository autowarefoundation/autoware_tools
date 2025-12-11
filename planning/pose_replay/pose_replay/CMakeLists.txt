cmake_minimum_required(VERSION 3.8)
project(pose_replay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pose_replay_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# test
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

# add_executable(topic_listener src/topic_listener.cpp)
add_library(pose_replay_node_component SHARED src/pose_replay_node.cpp)

ament_target_dependencies(pose_replay_node_component 
	rclcpp 
	geometry_msgs 
	autoware_adapi_v1_msgs 
	autoware_planning_msgs 
	pose_replay_interfaces
	rclcpp_components
)

target_link_libraries(pose_replay_node_component yaml-cpp uuid)

install(TARGETS pose_replay_node_component
	EXPORT export_pose_replay_node_component
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

rclcpp_components_register_node(
	pose_replay_node_component
	PLUGIN "PoseReplayNode"
	EXECUTABLE pose_replay_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
