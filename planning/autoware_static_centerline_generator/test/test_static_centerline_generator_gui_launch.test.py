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

from ament_index_python import get_package_share_directory
import pytest
import rclpy
from std_msgs.msg import Empty
from std_msgs.msg import Int32

sys.path.append(os.path.dirname(__file__))
from utils.test_static_centerline_generator_launch_base import (  # noqa
    generate_test_description_impl,
)
from utils.test_utils import TestBase  # noqa


def create_Int32(idx):
    msg = Int32()
    msg.data = idx
    return msg


@pytest.mark.launch_test
def generate_test_description():
    mode = "GUI"
    map_path = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/cargo_transport_map.osm"
    )
    centerline_source = "bag_ego_trajectory_base"
    bag_file = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/bag_ego_trajectory.db3"
    )
    return generate_test_description_impl(mode, map_path, centerline_source, bag_file)


class TestGuiOperation(TestBase):
    def test(self):
        # traj start idx
        self.pub_traj_start_idx = self.traj_sub_node.create_publisher(
            Int32, "/static_centerline_generator/traj_start_index", 1
        )
        self.pub_traj_start_idx.publish(create_Int32(50))

        # traj end idx
        self.pub_traj_end_idx = self.traj_sub_node.create_publisher(
            Int32, "/static_centerline_generator/traj_end_index", 1
        )
        self.pub_traj_end_idx.publish(create_Int32(150))

        rclpy.spin_once(self.traj_sub_node, timeout_sec=10.0)

        # validate
        self.pub_validate = self.traj_sub_node.create_publisher(
            Empty, "/static_centerline_generator/validate", 1
        )
        self.pub_validate.publish(Empty())

        # save map
        time.sleep(3)  # wait for validation
        self.pub_save_map = self.traj_sub_node.create_publisher(
            Empty, "/static_centerline_generator/save_map", 1
        )
        self.pub_save_map.publish(Empty())

        # subscribe the centerline
        rclpy.spin_once(self.map_saved_sub_node, timeout_sec=10.0)

        # check if the subscription is successful
        self.assertIsNotNone(self.centerline)
        # check if the centerline is in the lanelet2_map.osm
        self.validate_map_centerline_lane_ids(["661"])


# NOTE: The following test will fail due to centerline_updater_helper
"""
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
"""
