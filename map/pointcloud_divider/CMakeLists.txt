cmake_minimum_required(VERSION 3.5)
project(pointcloud_divider)

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(ament_cmake_auto REQUIRED COMPONENTS)
ament_auto_find_build_dependencies()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

# Add divider library
add_library(pointcloud_divider_lib SHARED src/pointcloud_divider_core.cpp src/voxel_grid_filter.cpp)
target_link_libraries(pointcloud_divider_lib yaml-cpp ${PCL_LIBRARIES})

ament_export_libraries(pointcloud_divider_lib)

install(TARGETS pointcloud_divider_lib
        EXPORT pointcloud_divider_lib
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)

add_executable(pointcloud_divider src/pointcloud_divider.cpp)

# Linking libraries
target_link_libraries(pointcloud_divider ${PROJECT_NAME}_lib)

install(DIRECTORY include/
        DESTINATION include
        )

install(TARGETS pointcloud_divider
        EXPORT pointcloud_divider
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

install(PROGRAMS scripts/pointcloud_divider.sh scripts/divider_core.sh DESTINATION bin)

ament_auto_package()