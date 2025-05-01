#!/usr/bin/env python3

# Copyright 2025 TIER IV, Inc.
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
import sys
import unittest

from ament_index_python import get_package_share_directory
import launch_testing
import pytest
import rclpy

sys.path.append(os.path.dirname(__file__))
from test_static_centerline_generator_launch_base import generate_test_description_impl  # noqa
from test_utils import TestBase  # noqa
from test_utils import is_centerline_connected_to_goal  # noqa
from test_utils import validate_map_centerline_lane_ids  # noqa

GOAL_POSE = "[89853.8, 43164.8, 6.2, 0.0, 0.0, 0.2876, 0.9575]"


@pytest.mark.launch_test
def generate_test_description():
    mode = "AUTO"
    map_path = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/lanelet2_map.osm"
    )
    centerline_source = "optimization_trajectory_base"
    bag_file = None
    start_lanelet_id = "215"
    start_pose = "[89775.5, 43115.8, 6.0, 0.0, 0.0, 0.2876, 0.9575]"
    end_lanelet_id = "216"
    end_pose = GOAL_POSE
    goal_method = "behavior_path_planner"
    return generate_test_description_impl(
        mode,
        map_path,
        centerline_source,
        bag_file,
        start_lanelet_id,
        start_pose,
        end_lanelet_id,
        end_pose,
        goal_method,
    )


class TestAutoOperation(TestBase):
    def callback(self, msg):
        self.centerline = msg

    def test(self):
        # subscribe the centerline
        rclpy.spin_once(self.test_node, timeout_sec=10.0)

        # check if the subscription is successful
        self.assertIsNotNone(self.centerline)

        # check if the centerline's back is close to the goal
        dist_to_goal = is_centerline_connected_to_goal(self.centerline, GOAL_POSE)
        self.assertLessEqual(dist_to_goal, 0.1)

        # check if the centerline is in the lanelet2_map.osm
        map_centerline_lane_ids = validate_map_centerline_lane_ids()
        expected_map_centerline_lane_ids = ["113", "115", "22"]
        self.assertEqual(sorted(map_centerline_lane_ids), sorted(expected_map_centerline_lane_ids))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
