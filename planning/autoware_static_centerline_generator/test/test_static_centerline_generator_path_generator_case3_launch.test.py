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

sys.path.append(os.path.dirname(__file__))
from utils.test_static_centerline_generator_launch_base import (  # noqa
    generate_test_description_impl,
)
from utils.test_utils import TestBase  # noqa

GOAL_POSE = "[1259.2,1349.3,100.0,0.0,0.0,-0.714,0.699]"


@pytest.mark.launch_test
def generate_test_description():
    mode = "AUTO"
    map_path = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/cargo_transport_map.osm"
    )
    centerline_source = "optimization_trajectory_base"
    bag_file = None
    start_lanelet_id = "661"
    start_pose = "[963.9,1035.8,100.0,0.0,0.0,-0.11,0.999]"
    end_lanelet_id = "1480"
    end_pose = GOAL_POSE
    goal_method = "path_generator"
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
    def test(self):
        # check if the subscription is successful
        self.assertIsNotNone(self.centerline)
        # check if the centerline's back is close to the goal
        self.validate_goal_pose(GOAL_POSE)
        # check if the centerline is in the lanelet2_map.osm
        self.validate_map_centerline_lane_ids(
            [
                "73",
                "86",
                "201",
                "432",
                "661",
                "670",
                "675",
                "686",
                "780",
                "890",
                "895",
                "900",
                "905",
                "910",
                "915",
                "922",
                "927",
                "1085",
                "1102",
                "1118",
                "1127",
                "1134",
                "1211",
                "1216",
                "1221",
                "1235",
                "1284",
                "1327",
                "1386",
                "1412",
                "1423",
                "1434",
                "1443",
                "1448",
                "1459",
                "1464",
                "1469",
                "1480",
            ]
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
