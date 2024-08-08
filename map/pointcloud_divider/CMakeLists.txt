cmake_minimum_required(VERSION 3.5)
project(pointcloud_divider)

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

# Add source files
set(SOURCES
    src/pointcloud_divider_core.cpp
    src/pointcloud_divider.cpp
    src/voxel_grid_filter.cpp
)

add_executable(pointcloud_divider ${SOURCES})

# Linking libraries
target_link_libraries(pointcloud_divider yaml-cpp ${PCL_LIBRARIES})

install(TARGETS pointcloud_divider
        EXPORT pointcloud_divider
        RUNTIME DESTINATION bin)

install(PROGRAMS scripts/pointcloud_divider.sh scripts/divider_core.sh DESTINATION bin)
