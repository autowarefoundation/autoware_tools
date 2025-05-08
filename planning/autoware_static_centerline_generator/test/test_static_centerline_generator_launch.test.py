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


@pytest.mark.launch_test
def generate_test_description():
    map_path = (
        get_package_share_directory("autoware_static_centerline_generator")
        + "/test/data/cargo_transport_map.osm"
    )
    start_lanelet_id = "633"
    end_lanelet_id = "279"
    return generate_test_description_impl(
        map_path=map_path, start_lanelet_id=start_lanelet_id, end_lanelet_id=end_lanelet_id
    )


class TestAutoOperation(TestBase):
    def test(self):
        # check if the subscription is successful
        self.assertIsNotNone(self.centerline)
        # check if the centerline is in the lanelet2_map.osm
        self.validate_map_centerline_lane_ids(
            [
                "25",
                "30",
                "40",
                "53",
                "68",
                "73",
                "86",
                "201",
                "211",
                "216",
                "236",
                "249",
                "254",
                "270",
                "279",
                "432",
                "633",
            ]
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
