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
from std_msgs.msg import Empty


def is_centerline_connected_to_goal(centerline, goal_pose):
    goal_pose_list = goal_pose.split("[")[1].split("]")[0].split(",")
    centerline_end_point = centerline.points[-1].pose.position

    return math.hypot(
        centerline_end_point.x - float(goal_pose_list[0]),
        centerline_end_point.y - float(goal_pose_list[1]),
    )


def get_map_centerline_lane_ids():
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

    def callbackTrajectory(self, msg):
        print("callbackTrajectory")
        self.centerline = msg

    def callbackMapSaved(self, msg):
        print("callbackMapSaved")
        pass

    def setUp(self):
        print("setUp")
        self.traj_sub_node = rclpy.create_node("traj_sub_node")
        self.map_saved_sub_node = rclpy.create_node("map_saved_sub_node")
        self.centerline = None

        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # subscribe the centerline
        self.traj_sub_node.create_subscription(
            Trajectory,
            "/static_centerline_generator/output/centerline",
            self.callbackTrajectory,
            qos_profile,
        )
        print("self.traj_sub_node1")
        rclpy.spin_once(self.traj_sub_node, timeout_sec=10.0)
        print("self.traj_sub_node2")

        # subscribe the map_saved flag
        self.map_saved_sub_node.create_subscription(
            Empty,
            "/static_centerline_generator/output/map_saved",
            self.callbackMapSaved,
            qos_profile,
        )
        print("self.map_saved_sub_node1")
        rclpy.spin_once(self.map_saved_sub_node, timeout_sec=10.0)
        print("self.map_saved_sub_node2")

    def tearDown(self):
        self.traj_sub_node.destroy_node()
        self.map_saved_sub_node.destroy_node()

    def validate_goal_pose(self, goal_pose):
        dist_to_goal = is_centerline_connected_to_goal(self.centerline, goal_pose)
        self.assertLessEqual(dist_to_goal, 0.2)

    def validate_map_centerline_lane_ids(self, expected_map_centerline_lane_ids):
        map_centerline_lane_ids = get_map_centerline_lane_ids()
        self.assertEqual(sorted(map_centerline_lane_ids), sorted(expected_map_centerline_lane_ids))
