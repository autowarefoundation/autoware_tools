#!/bin/bash

ros2 launch autoware_static_centerline_generator static_centerline_generator.launch.xml centerline_source:="bag_ego_trajectory_base" mode:="GUI" lanelet2_input_file_path:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/cargo_transport_map.osm" bag_filename:="$(ros2 pkg prefix autoware_static_centerline_generator --share)/test/data/bag_ego_trajectory.db3" vehicle_model:=lexus
