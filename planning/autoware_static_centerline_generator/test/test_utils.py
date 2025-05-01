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


import math
import unittest
import xml.etree.ElementTree as ET

from autoware_planning_msgs.msg import Trajectory
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile


def is_centerline_connected_to_goal(centerline, goal_pose):
    goal_pose_list = goal_pose.split("[")[1].split("]")[0].split(", ")
    centerline_end_point = centerline.points[-1].pose.position

    return math.hypot(
        centerline_end_point.x - float(goal_pose_list[0]),
        centerline_end_point.y - float(goal_pose_list[1]),
    )


def validate_map_centerline_lane_ids():
    tree = ET.parse("/tmp/autoware_static_centerline_generator/lanelet2_map.osm")
    root = tree.getroot()

    map_centerline_ids = []
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            if member.attrib.get("role") == "centerline":
                map_centerline_ids.append(relation.attrib["id"])

    return map_centerline_ids


class TestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")
        self.centerline = None

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.test_node.create_subscription(
            Trajectory, "/static_centerline_generator/output/centerline", self.callback, qos_profile
        )

    def tearDown(self):
        self.test_node.destroy_node()
