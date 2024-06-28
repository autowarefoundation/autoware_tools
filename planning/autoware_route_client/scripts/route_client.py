#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2024 TIER IV, Inc. All rights reserved.
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

import argparse

from autoware_adapi_v1_msgs.msg import RouteOption
from autoware_adapi_v1_msgs.msg import RoutePrimitive
from autoware_adapi_v1_msgs.msg import RouteSegment
from autoware_adapi_v1_msgs.srv import SetRoute
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import yaml


class route_client(Node):
    def __init__(self):
        super().__init__("route_client")
        self.cli = self.create_client(SetRoute, "/api/routing/change_route")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available...")

        self.req = SetRoute.Request()

    def send_request(self, filepath):
        self.req.header = Header()
        self.req.header.frame_id = "map"
        self.req.header.stamp = self.get_clock().now().to_msg()
        self.req.option = RouteOption()
        self.req.option.allow_goal_modification = True

        with open(filepath) as file:
            data = yaml.safe_load(file)
            self.req.goal = self.set_goal(data["goal"])
            self.req.segments = self.set_segments(data["segments"])
            self.future = self.cli.call_async(self.req)

    def set_goal(self, goal):
        out = Pose()
        out.position.x = goal["position"]["x"]
        out.position.y = goal["position"]["y"]
        out.position.z = goal["position"]["z"]
        out.orientation.x = goal["orientation"]["x"]
        out.orientation.y = goal["orientation"]["y"]
        out.orientation.z = goal["orientation"]["z"]
        out.orientation.w = goal["orientation"]["w"]

        return out

    def set_segments(self, segments):
        out = []
        for segment in segments:
            out_segment = RouteSegment()
            out_segment.preferred.id = segment["preferred"]["id"]
            out_segment.preferred.type = segment["preferred"]["type"]
            for alt in segment["alternatives"]:
                out_alt = RoutePrimitive()
                out_alt.id = alt["id"]
                out_alt.type = alt["type"]
                out_segment.alternatives.append(out_alt)

            out.append(out_segment)
        return out


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Send set route request.")
    parser.add_argument("filepath", type=str, help="path to yaml file containing route information")
    args = parser.parse_args()

    client = route_client()
    client.send_request(args.filepath)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                client.get_logger().info("Service requested.")
            except Exception as e:
                client.get_logger().info("Error: %r" % (e,))
            break

    client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
