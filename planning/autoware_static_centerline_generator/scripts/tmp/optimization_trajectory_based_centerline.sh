#!/bin/bash

ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="optimization_trajectory_base" run_background:=false lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/lanelet2_map.osm" start_lanelet_id:=215 end_lanelet_id:=216 vehicle_model:=lexus
