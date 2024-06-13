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

# refer the test code of rosbag2_py
# https://github.com/ros2/rosbag2/blob/rolling/rosbag2_py/test/test_sequential_writer.py
# https://github.com/ros2/rosbag2/blob/rolling/rosbag2_py/test/test_reindexer.py

from rosbag2_py import Reindexer
from rosbag2_py import TopicMetadata

from autoware_msg_bag_converter.bag import create_reader
from autoware_msg_bag_converter.bag import create_writer
from autoware_msg_bag_converter.bag import get_default_storage_options


def change_topic_type(old_type: TopicMetadata) -> TopicMetadata:
    # If old_type is not of type autoware_auto, the original message type remains
    return TopicMetadata(
        name=old_type.name,
        type=old_type.type.replace("autoware_auto_", "autoware_"),
        serialization_format="cdr",
    )


def convert_bag(input_bag_path: str, output_bag_path: str) -> None:
    # open reader
    reader = create_reader(input_bag_path)
    # open writer
    writer = create_writer(output_bag_path)

    # create topic
    type_map = {}
    for topic_type in reader.get_all_topics_and_types():
        type_map[topic_type.name] = topic_type.type
        new_topic_type = change_topic_type(
            topic_type,
        )
        writer.create_topic(new_topic_type)

    # copy data from input bag to output bag
    while reader.has_next():
        topic_name, msg, stamp = reader.read_next()
        writer.write(topic_name, msg, stamp)

    # reindex to update metadata.yaml
    del writer
    Reindexer().reindex(get_default_storage_options(output_bag_path))
