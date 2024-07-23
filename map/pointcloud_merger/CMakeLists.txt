cmake_minimum_required(VERSION 3.5)
project(pointcloud_merger)

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

# Find packages
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
# find_package(pointcloud_divider REQUIRED)

include_directories(include)

# Add source files
set(SOURCES
    src/pointcloud_merger_core.cpp
    src/pointcloud_merger.cpp
)

add_executable(pointcloud_merger ${SOURCES})

# Linking libraries
target_link_libraries(pointcloud_merger yaml-cpp ${PCL_LIBRARIES})

install(TARGETS pointcloud_merger
        EXPORT pointcloud_merger
        RUNTIME DESTINATION bin)

# install(PROGRAMS scripts/pointcloud_divider.sh scripts/divider_core.sh DESTINATION bin)
