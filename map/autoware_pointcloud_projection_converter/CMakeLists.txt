cmake_minimum_required(VERSION 3.14)

project(autoware_pointcloud_projection_converter)

# Find required packages
find_package(autoware_cmake REQUIRED)
autoware_package()

if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(PCL REQUIRED)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(converter_lib src/converter_from_llh.cpp src/converter_to_llh.cpp)
target_link_libraries(converter_lib ${GeographicLib_LIBRARIES} ${PCL_LIBRARIES})

ament_auto_add_executable(pointcloud_projection_converter src/pcd_conversion.cpp)
target_link_libraries(pointcloud_projection_converter converter_lib yaml-cpp)

ament_auto_package(INSTALL_TO_SHARE
    config
)
