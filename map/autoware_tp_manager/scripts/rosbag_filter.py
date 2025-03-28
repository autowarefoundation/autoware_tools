#!/usr/bin/env python3

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
from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
    SequentialWriter,
    TopicMetadata,
    # For debug
    Info
)
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import NavSatFix
import os
import shutil
import csv
import yaml
import tqdm

def remove_file_politely(output_path: str):
    print("are you sure to overwrite {0}? [y/n]".format(output_path))
    if input() != "y":
        print("exit")
        exit()
    shutil.rmtree(output_path)

def get_message_count(bag_path):
    info = Info()

    metadata = info.read_metadata(bag_path, "sqlite3")

    return metadata.message_count

def main(input_path: str, output_path: str, save_dir: str, gnss_topics: str, sensor_topics: str, roc_th: float):
    # Split gnss topics and sensor topics
    gnss_topic = set()
    gnss_topic_list = gnss_topics.split(',')
    for topic in gnss_topic_list:
        gnss_topic.add(topic)

    sensor_topic = set()
    sensor_topic_list = sensor_topics.split(',')
    for topic in sensor_topic_list:
        sensor_topic.add(topic)

    # setup rosbag reader
    reader = SequentialReader()
    bag_storage_options = StorageOptions(uri=input_path, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(bag_storage_options, bag_converter_options)

    # Reindex for testing
    message_count = get_message_count(input_path)

    # check file existence
    if os.path.exists(output_path):
        remove_file_politely(output_path)

    candidate_stamp = []
    if not(os.path.exists(save_dir + "/area_score.csv")):
        print("Error: an area_score.csv file does not exist at {0}. ".format(save_dir + "/area_score.csv"))
    else:
        # Parse the area_score.csv to obtain a list of start and end stamp of candidate lidar messages
        candidate_stamp = parse_area_score(save_dir + "/area_score.csv", roc_th)

    # setup rosbag writer
    writer = SequentialWriter()
    bag_storage_options = StorageOptions(uri=output_path, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(bag_storage_options, bag_converter_options)

    # prepare meta data for writer
    topic_meta_data_array = reader.get_all_topics_and_types()
    for topic_meta_data in topic_meta_data_array:
        writer.create_topic(
            TopicMetadata(
                name=topic_meta_data.name,
                type=topic_meta_data.type,
                serialization_format="cdr",
            )
        )

    progress_bar = tqdm.tqdm(total = message_count)

    # read and write
    nav_sat_fix_topic_count = 0
    while reader.has_next():
        progress_bar.update(1)
        (topic, data, stamp) = reader.read_next()
        s_stamp = stamp / 1000000000
        if topic in gnss_topic:
            nav_sat_fix = deserialize_message(data, NavSatFix)

            if nav_sat_fix.status.status == -1:
                continue

            writer.write(topic, serialize_message(nav_sat_fix), stamp)

            nav_sat_fix_topic_count += 1
        elif topic in sensor_topic and candidate_stamp:
            # If the message's timestamp falls in one candidate range, write the message to the output
            # Otherwise, skip the message, since it is not corresponding to an outdated map area
            for (start_stamp, end_stamp) in candidate_stamp:
                if (s_stamp >= start_stamp and s_stamp <= end_stamp):
                    writer.write(topic, data, stamp)
                    break
        else:
            writer.write(topic, data, stamp)

    progress_bar.close()
    print("Finish filtering rosbag")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", help="path to input rosbag")
    parser.add_argument("output_path", help="path to filtered rosbag")
    parser.add_argument("--itopics", help="Path to the file including name of topics to be included in the output bag ", default="", required=True)
    parser.add_argument("--etopics", help="Path to the file including name of topics to be excluded from the output bag ", default="", required=False)
    args = parser.parse_args()

    print("Input rosbag at " + args.input_path)
    print("Output rosbag at " + args.output_path)
    print("Path to the output directory of map_assessment_tools " + args.save_dir)
    print("GNSS topic to be filtered " + args.gnss_topics)
    if (args.sensor_topics == ""):
        print("No Sensor topic was specified. All messages from all sensor topics are kept the same.")
    else:
        print("Sensor topic to be filtered " + args.sensor_topics)
    print("The minimum RoC value to mark an area as decay " + args.roc_threshold)

    main(args.input_path, args.output_path, args.save_dir, args.gnss_topics, args.sensor_topics, float(args.roc_threshold))