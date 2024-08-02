cmake_minimum_required(VERSION 3.5)
project(autoware_pointcloud_merger)

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

add_library(${PROJECT_NAME}_lib SHARED src/pointcloud_merger_core.cpp)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_lib autoware_pointcloud_divider)

ament_export_libraries(${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib
        EXPORT ${PROJECT_NAME}_lib
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include)

ament_auto_add_executable(${PROJECT_NAME}_node src/pointcloud_merger.cpp)

# Linking libraries
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_node
        EXPORT ${PROJECT_NAME}_node
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

install(PROGRAMS scripts/pointcloud_merger.sh scripts/merger_core.sh DESTINATION bin)

ament_auto_package()
