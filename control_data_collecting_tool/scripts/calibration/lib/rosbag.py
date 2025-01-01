#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import re
import subprocess

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

TOPIC_LIST = "/control/(.*)|/vehicle/(.*)|/imu/(.*)|/sensing/imu/(.*)"


def record_ros2_bag(bag_name, topic_list):
    command = ["ros2", "bag", "record", "-o", bag_name, "-e", topic_list]

    process = subprocess.Popen(
        command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

    return process


def validate(bag_name, topic_list):
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", bag_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True,
        )
        output = result.stdout

        missing_topics = []
        for topic_name in topic_list:
            pattern = rf"Topic: {re.escape(topic_name)}\s+\|.+?\| Count: (\d+)"
            match = re.search(pattern, output)

            if match:
                count = int(match.group(1))
                if count == 0:
                    missing_topics.append(topic_name)
            else:
                missing_topics.append(topic_name)

        if missing_topics:
            print("The following topics are missing:")
            for topic_name in missing_topics:
                print(f"- {topic_name}")
            return False

        return True
    except subprocess.CalledProcessError as e:
        print(f"Error running ros2 bag info: {e.stderr.strip()}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False


def get_topics(rosbag_path, topic_names):
    storage_options = rosbag2_py.StorageOptions(uri=rosbag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_filter = rosbag2_py.StorageFilter(topics=topic_names)
    reader.set_filter(topic_filter)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    topics = {}
    for topic_name in topic_names:
        topics[topic_name] = []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        topics[topic].append(msg)

    return topics
