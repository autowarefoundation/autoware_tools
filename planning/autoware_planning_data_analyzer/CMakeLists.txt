cmake_minimum_required(VERSION 3.14)
project(autoware_planning_data_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w)
endif()

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/utils.cpp
  src/bag_handler.cpp
  src/evaluation.cpp
  src/data_structs.cpp
)

target_link_libraries(${PROJECT_NAME} matplotlib_cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::behavior_analyzer::BehaviorAnalyzerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_executable(minimal
  src/main.cpp
  src/node.cpp
  src/utils.cpp
  src/bag_handler.cpp
  src/evaluation.cpp
  src/data_structs.cpp
  )

target_link_libraries(minimal matplotlib_cpp)
# target_link_libraries(minimal PRIVATE matplotlib_cpp)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  rviz
)
