#!/usr/bin/env python3

# Copyright 2020 Proxima Technology Inc, Tier IV Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


# Load yaml file
def get_course_name(yaml_file_path):
    with open(yaml_file_path, "r") as file:
        data = yaml.safe_load(file)

    # get 'COURSE_NAME'
    course_name = data.get("/**", {}).get("ros__parameters", {}).get("COURSE_NAME", None)
    return course_name


def generate_launch_description():
    # Define the argument for map_path
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        description="Path to the map directory (optional; defaults to None if not provided).",
        default_value="",  # Default to None
    )
    # Use the map_path argument in parameters
    map_path = LaunchConfiguration("map_path")

    # Get the path to the common param file
    package_share_directory = get_package_share_directory("control_data_collecting_tool")
    common_param_file_path = os.path.join(package_share_directory, "config", "common_param.yaml")

    # Get the path to the course-specific param file
    course_name = get_course_name(common_param_file_path)
    course_specific_param_file_path = os.path.join(
        package_share_directory, "config/course_param", course_name + "_param.yaml"
    )

    return launch.LaunchDescription(
        [
            map_path_arg,
            Node(
                package="control_data_collecting_tool",
                executable="data_collecting_pure_pursuit_trajectory_follower.py",
                name="data_collecting_pure_pursuit_trajectory_follower",
                parameters=[common_param_file_path],
            ),
            Node(
                package="control_data_collecting_tool",
                executable="data_collecting_trajectory_publisher.py",
                name="data_collecting_trajectory_publisher",
                parameters=[
                    common_param_file_path,
                    course_specific_param_file_path,
                    {"map_path": map_path},
                ],
            ),
            Node(
                package="control_data_collecting_tool",
                executable="data_collecting_plotter.py",
                name="data_collecting_plotter",
                parameters=[common_param_file_path],
            ),
            Node(
                package="control_data_collecting_tool",
                executable="data_collecting_rosbag_record.py",
                name="data_collecting_rosbag_record",
            ),
            Node(
                package="control_data_collecting_tool",
                executable="data_collecting_data_counter.py",
                name="data_collecting_data_counter",
                parameters=[common_param_file_path],
            ),
        ]
    )


if __name__ == "__main__":
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
