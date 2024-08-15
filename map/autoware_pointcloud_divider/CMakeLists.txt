cmake_minimum_required(VERSION 3.14)
project(autoware_pointcloud_divider)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(include)

# Add divider library
ament_auto_add_library(${PROJECT_NAME} SHARED src/pointcloud_divider_node.cpp src/voxel_grid_filter.cpp src/pcd_divider.cpp)
target_link_libraries(${PROJECT_NAME} yaml-cpp ${PCL_LIBRARIES})
rclcpp_components_register_node(${PROJECT_NAME} 
  PLUGIN "autoware::pointcloud_divider::PointCloudDivider"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# install(PROGRAMS scripts/pointcloud_divider.sh scripts/divider_core.sh DESTINATION bin)

ament_auto_package(INSTALL_TO_SHARE launch config)
