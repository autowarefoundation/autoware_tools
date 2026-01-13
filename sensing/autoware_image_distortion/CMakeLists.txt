cmake_minimum_required(VERSION 3.8)
project(autoware_image_distortion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_distortion src/image_distortion.cpp)


target_link_libraries(image_distortion
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(image_distortion
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
)

install(TARGETS image_distortion
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()