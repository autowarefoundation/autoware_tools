cmake_minimum_required(VERSION 3.5)
project(autoware_localization_evaluation_scripts)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
    scripts/analyze_rosbags_parallel.sh
    scripts/compare_trajectories.py
    scripts/extract_pose_from_rosbag.py
    scripts/plot_diagnostics.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
