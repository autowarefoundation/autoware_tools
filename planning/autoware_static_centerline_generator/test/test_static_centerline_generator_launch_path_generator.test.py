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
import unittest

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import pytest


@pytest.mark.launch_test
def generate_test_description():
    test_static_centerline_generator_launch_file = os.path.join(
        get_package_share_directory("autoware_static_centerline_generator"),
        "launch",
        "static_centerline_generator.launch.xml",
    )

    static_centerline_generator = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(test_static_centerline_generator_launch_file),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lanelet2_input_file_path",
                default_value=os.path.join(
                    get_package_share_directory("autoware_static_centerline_generator"),
                    "test/data/lanelet2_map.osm",
                ),
            ),
            DeclareLaunchArgument(
                "lanelet2_output_file_path", default_value="/tmp/lanelet2_map.osm"
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("start_lanelet_id", default_value="215"),
            DeclareLaunchArgument("start_pose", default_value="[89775.5, 43115.8, 6.0, 0.0, 0.0, 0.2876, 0.9575]"),
            DeclareLaunchArgument("end_lanelet_id", default_value="216"),
            DeclareLaunchArgument("end_pose", default_value="[89853.8, 43164.8, 6.2, 0.0, 0.0, 0.2876, 0.9575]"),
            DeclareLaunchArgument("goal_method", default_value="path_generator"),
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
            # Start test after 10s - gives time for the autoware_static_centerline_generator to finish initialization
            launch.actions.TimerAction(period=10.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
