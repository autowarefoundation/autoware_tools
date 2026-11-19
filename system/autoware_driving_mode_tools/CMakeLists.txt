cmake_minimum_required(VERSION 3.14)
project(autoware_driving_mode_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(PROGRAMS script/debug_panel DESTINATION lib/${PROJECT_NAME})
ament_auto_package(INSTALL_TO_SHARE launch)
