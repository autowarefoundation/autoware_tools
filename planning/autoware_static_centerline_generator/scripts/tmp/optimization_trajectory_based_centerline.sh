#!/bin/bash

if [ "$1" = "0" ]; then
    ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="optimization_trajectory_base" mode:="AUTO" lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/cargo_transport_map.osm" vehicle_model:=lexus goal_method:="path_generator" start_lanelet_id:=633 start_pose:="[986.2,1003.5,100.0,0.0,0.0,-0.011,0.999]" end_lanelet_id:=279 end_pose:="[1112.4,1229.5,100.0,0.0,0.0,0.701,0.713]"
elif [ "$1" = "1" ]; then
    ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="optimization_trajectory_base" mode:="AUTO" lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/cargo_transport_map.osm" vehicle_model:=lexus goal_method:="path_generator" start_lanelet_id:=279 start_pose:="[1112.4,1229.5,100.0,0.0,0.0,0.701,0.713]" end_lanelet_id:=633 end_pose:="[986.2,1003.5,100.0,0.0,0.0,-0.011,0.999]"
elif [ "$1" = "2" ]; then
    ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="optimization_trajectory_base" mode:="AUTO" lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/cargo_transport_map.osm" vehicle_model:=lexus goal_method:="path_generator" start_lanelet_id:=661 start_pose:="[963.9,1035.8,100.0,0.0,0.0,-0.11,0.999]" end_lanelet_id:=1480 end_pose:="[1259.2,1349.3,100.0,0.0,0.0,-0.714,0.699]"
elif [ "$1" = "3" ]; then
    ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="optimization_trajectory_base" mode:="AUTO" lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/cargo_transport_map.osm" vehicle_model:=lexus goal_method:="path_generator" start_lanelet_id:=1480 start_pose:="[1259.2,1349.3,100.0,0.0,0.0,-0.714,0.699]" end_lanelet_id:=661 end_pose:="[963.9,1035.8,100.0,0.0,0.0,0.0,1.0]"
else
    echo "Please designate the argument from '0, 1, 2 or 3'."
fi
