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
import sys

import rcl_interfaces
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node


def get_rosout_format():
    try:
        rosout_format = os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"]
        if rosout_format != "":
            return rosout_format
    except KeyError:
        pass

    # use default rosout format
    return "[{severity} {time}] [{name}]: {message} ({function_name}():{line_number})"


def get_logger_level_name(level):
    if level == int.from_bytes(rcl_interfaces.msg.Log.DEBUG, byteorder="big"):
        return "DEBUG"
    if level == int.from_bytes(rcl_interfaces.msg.Log.INFO, byteorder="big"):
        return "INFO"
    if level == int.from_bytes(rcl_interfaces.msg.Log.WARN, byteorder="big"):
        return "WARN"
    if level == int.from_bytes(rcl_interfaces.msg.Log.ERROR, byteorder="big"):
        return "ERROR"
    return "FATAL"


class RosoutLogReconstructor(Node):
    def __init__(self):
        super().__init__("rosout_log_reconstructor")

        self.rosout_format = get_rosout_format()

        transient_local = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        transient_local_profile = rclpy.qos.QoSProfile(depth=1, durability=transient_local)
        self.sub_rosout = self.create_subscription(
            Log, "/rosout", self.on_rosout, transient_local_profile
        )

    def on_rosout(self, msg):
        severity = get_logger_level_name(msg.level)

        text = self.rosout_format.format(
            time=msg.stamp,
            name=msg.name,
            severity=severity,
            function_name=msg.function,
            message=msg.msg,
            file_name=msg.file,
            line_number=msg.line,
        )
        if severity == "WARN":
            sys.stderr.write("\x1b[33m" + text + "\x1b[0m" + "\n")
        elif severity == "ERROR":
            sys.stderr.write("\x1b[1;31m" + text + "\x1b[0m" + "\n")
        else:
            print(text)


if __name__ == "__main__":
    rclpy.init()
    node = RosoutLogReconstructor()
    rclpy.spin(node)
