# Copyright (c) 2024 TIER IV.inc
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

from pathlib import Path

from rosbag2_py import TopicMetadata

from autoware_msg_bag_converter.converter import change_topic_type


def test_change_topic_type() -> None:
    old_type = TopicMetadata(
        name="/vehicle/status/control_mode",
        type="autoware_auto_vehicle_msgs/msg/ControlModeReport",
        serialization_format="cdr",
    )
    new_type = change_topic_type(old_type)
    assert new_type.name == "/vehicle/status/control_mode"
    assert new_type.type == "autoware_vehicle_msgs/msg/ControlModeReport"
    assert new_type.serialization_format == "cdr"


def test_get_rosbag_path() -> None:
    # Test to confirm bag path acquisition in directory mode
    input_root = Path(__file__).resolve().parent.joinpath("resource")
    output_root = Path(__file__).resolve().parent.joinpath("converted")
    bag_paths = input_root.glob("**/*.db3")
    for db3_path in bag_paths:
        input_bag_dir = db3_path.parent
        rel_path = input_bag_dir.relative_to(input_root)
        output_bag_dir = output_root.joinpath(rel_path)
        print(output_bag_dir)  # noqa
