# Copyright 2022 Tier IV, Inc. All rights reserved.
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

from ament_index_python import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        arg = DeclareLaunchArgument(name, default_value=default_value, description=description)
        launch_arguments.append(arg)

    add_launch_arg("input_pcd_dir", "", "Path to the folder containing the input PCD files")
    add_launch_arg("output_pcd_dir", "", "Path to the folder containing the segmented PCD files")
    add_launch_arg("prefix", "", "Prefix for the name of the output PCD files")
    add_launch_arg(
        "config_file",
        [FindPackageShare("autoware_pointcloud_divider"), "/config/pointcloud_divider.param.yaml"],
        "Path to the parameter file of the package autoware_pointcloud_divider",
    )

    # Parameter of pointcloud divider
    config_file = os.path.join(
        get_package_share_path("autoware_pointcloud_divider"),
        "config",
        "pointcloud_divider.param.yaml",
    )
    with open(config_file, "r") as f:
        divider_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    divider_node = Node(
        package="autoware_pointcloud_divider",
        executable="autoware_pointcloud_divider_node",
        parameters=[
            divider_param,
            {
                "input_pcd_or_dir": LaunchConfiguration("input_pcd_or_dir"),
                "output_pcd_dir": LaunchConfiguration("output_pcd_dir"),
                "prefix": LaunchConfiguration("prefix"),
                "config_file": LaunchConfiguration("config_file"),
            },
        ],
        output="screen",
    )
    divider_shutdown = get_shutdown_handler(divider_node)

    return launch.LaunchDescription(
        launch_arguments
        + [
            divider_node,
            divider_shutdown,
        ]
    )


def get_shutdown_handler(node):
    return launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=node,
            on_exit=[
                launch.actions.LogInfo(msg="shutdown launch"),
                launch.actions.EmitEvent(event=launch.events.Shutdown()),
            ],
        )
    )
