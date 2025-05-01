#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing


def generate_test_description_impl(
    mode=None,
    map_path=None,
    centerline_source=None,
    bag_file=None,
    start_lanelet_id=None,
    start_pose=None,
    end_lanelet_id=None,
    end_pose=None,
    goal_method=None,
):
    test_static_centerline_generator_launch_file = os.path.join(
        get_package_share_directory("autoware_static_centerline_generator"),
        "launch",
        "static_centerline_generator.launch.xml",
    )

    static_centerline_generator = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_static_centerline_generator_launch_file),
    )

    launch_description = []
    if mode:
        launch_description.append(DeclareLaunchArgument("mode", default_value=mode))
    if map_path:
        launch_description.append(
            DeclareLaunchArgument("lanelet2_input_file_path", default_value=map_path)
        )
    if centerline_source:
        launch_description.append(
            DeclareLaunchArgument("centerline_source", default_value=centerline_source)
        )
    if bag_file:
        launch_description.append(DeclareLaunchArgument("bag_filename", default_value=bag_file))
    if start_lanelet_id:
        launch_description.append(
            DeclareLaunchArgument("start_lanelet_id", default_value=start_lanelet_id)
        )
    if start_pose:
        launch_description.append(DeclareLaunchArgument("start_pose", default_value=start_pose))
    if end_lanelet_id:
        launch_description.append(
            DeclareLaunchArgument("end_lanelet_id", default_value=end_lanelet_id)
        )
    if end_pose:
        launch_description.append(DeclareLaunchArgument("end_pose", default_value=end_pose))
    if goal_method:
        launch_description.append(DeclareLaunchArgument("goal_method", default_value=goal_method))

    return LaunchDescription(
        launch_description
        + [
            DeclareLaunchArgument(
                "lanelet2_output_file_path", default_value="/tmp/lanelet2_map.osm"
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "map_loader_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_map_loader"),
                    "config/lanelet2_map_loader.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "behavior_path_planner_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_behavior_path_planner"),
                    "config/behavior_path_planner.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "behavior_velocity_planner_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_behavior_velocity_planner"),
                    "config/behavior_velocity_planner.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "path_generator_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_path_generator"),
                    "config/path_generator.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "path_smoother_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_path_smoother"),
                    "config/elastic_band_smoother.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "path_optimizer_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_path_optimizer"),
                    "config/path_optimizer.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "mission_planner_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_mission_planner_universe"),
                    "config/mission_planner.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "common_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_static_centerline_generator"),
                    "config/common.param.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "nearest_search_param",
                default_value=os.path.join(
                    get_package_share_directory("autoware_static_centerline_generator"),
                    "config/nearest_search.param.yaml",
                ),
            ),
            static_centerline_generator,
            # Start test after 3s - gives time for the autoware_static_centerline_generator to finish initialization
            TimerAction(period=3.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )
