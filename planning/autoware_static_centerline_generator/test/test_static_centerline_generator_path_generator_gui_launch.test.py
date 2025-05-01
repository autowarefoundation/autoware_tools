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
import time
import unittest

from ament_index_python import get_package_share_directory
import launch_testing
import pytest
import rclpy
from std_msgs.msg import Empty
from std_msgs.msg import Int32

sys.path.append(os.path.dirname(__file__))
from test_static_centerline_generator_launch_base import generate_test_description_impl  # noqa
from test_utils import TestBase  # noqa
from test_utils import is_centerline_connected_to_goal  # noqa
from test_utils import validate_map_centerline_lane_ids  # noqa


@pytest.mark.launch_test
def generate_test_description():
    mode = "GUI"
    map_path = "/home/takayuki/autoware_map/sample_map/lanelet2_map.osm"  # get_package_share_directory("autoware_static_centerline_generator") + "/test/data/lanelet2_map.osm"
    centerline_source = "bag_ego_trajectory_base"
    bag_file = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/bag_ego_trajectory_turn-right.db3"
    )
    return generate_test_description_impl(mode, map_path, centerline_source, bag_file)


class TestAutoOperation(TestBase):
    def callback(self, msg):
        self.centerline = msg

    def test(self):
        # traj start idx
        self.pub_traj_start_idx = self.test_node.create_publisher(
            Int32, "/static_centerline_generator/traj_start_index", 1
        )
        traj_start_idx = Int32()
        traj_start_idx.data = 2
        self.pub_traj_start_idx.publish(traj_start_idx)

        # traj end idx
        self.pub_traj_end_idx = self.test_node.create_publisher(
            Int32, "/static_centerline_generator/traj_end_index", 1
        )
        traj_end_idx = Int32()
        traj_end_idx.data = 5
        self.pub_traj_end_idx.publish(traj_end_idx)

        # validate
        time.sleep(2)
        self.pub_validate = self.test_node.create_publisher(
            Empty, "/static_centerline_generator/validate", 1
        )
        self.pub_validate.publish(Empty())

        # save map
        time.sleep(5)
        self.pub_save_map = self.test_node.create_publisher(
            Empty, "/static_centerline_generator/save_map", 1
        )
        self.pub_save_map.publish(Empty())

        # subscribe the centerline
        rclpy.spin_once(self.test_node, timeout_sec=10.0)

        # check if the subscription is successful
        self.assertIsNotNone(self.centerline)

        """
        # check if the centerline's back is close to the goal
        dist_to_goal = is_centerline_connected_to_goal(self.centerline, GOAL_POSE)
        self.assertLessEqual(dist_to_goal, 0.1)

        # check if the centerline is in the lanelet2_map.osm
        map_centerline_lane_ids = validate_map_centerline_lane_ids()
        expected_map_centerline_lane_ids = ["113", "115", "22"]
        self.assertEqual(sorted(map_centerline_lane_ids), sorted(expected_map_centerline_lane_ids))
        """


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
