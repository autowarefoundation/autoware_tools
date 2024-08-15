cmake_minimum_required(VERSION 3.5)
project(autoware_pointcloud_merger)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED src/pointcloud_merger_node.cpp src/pcd_merger.cpp)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} autoware_pointcloud_divider)

rclcpp_components_register_node(${PROJECT_NAME} 
  PLUGIN "autoware::pointcloud_merger::PointCloudMerger"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}_node
        EXPORT ${PROJECT_NAME}_node
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# install(PROGRAMS scripts/pointcloud_merger.sh scripts/merger_core.sh DESTINATION bin)

ament_auto_package(INSTALL_TO_SHARE launch config)
