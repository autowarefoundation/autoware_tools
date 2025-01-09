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

import os
import argparse
from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
    SequentialWriter,
    BagMetadata,
    TopicMetadata,
    Info
)

from rclpy.serialization import deserialize_message, serialize_message
from subprocess import call, run, Popen
import os
import time
import csv
import yaml
import shutil

##### Get the list of PCD segments #####
def get_pcd_segments(pcd_map_dir: str):
    if not os.path.exists(pcd_map_dir):
        print("Error: %s does not exist!"%(pcd_map_dir))
        exit()

    pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd/")

    if not os.path.exists(pcd_path):
        print("Error: %s does not exist!"%(pcd_path))
        exit()

    # Get list of PCD segments in the folder pointcloud_map.pcd 
    pcd_segment_list = []

    for filename in os.listdir(pcd_path):
        if filename.endswith(".pcd") or filename.endswith(".PCD"):
            segment_path = os.path.join(pcd_path, filename)
            pcd_segment_list.append(segment_path)

    




##### Setup the working directory
def setup(work_dir: str):
    if os.path.exists(work_dir):
        if os.path.isdir(work_dir):
            print("%s already exists. Do you want to overwrite its content? [y/n]"%(work_dir))
        else:
            print("%s is not a directory. Do you want to delete it and create a new directory? [y/n]"%(work_dir))
        if input() != "y":
            return
        shutil.rmtree(work_dir)    
    os.mkdir(work_dir)

##### Search for the topic list from the metadata
def topic_search(topic_list, metadata):
    """
    This function iterates on each topic in the topic list,
    and search for each topic from the metadata string.
    :param topic_list: list of topic string
    :param metadata: dict of {topic_name : number of messages}
    """
    
    output_list = []

    for topic in topic_list:
        value = metadata.get(topic, "unknown")
        if value != "unknown":
            output_list.append(topic)
    
    return output_list

###### Step 0: Check if some topics exist in the rosbag
# Print the metadata of the rosbag and look for the topics in the metadata
def topic_setup(topic_dict: dict, in_bag_dir: str):
    # Setup lidar topic
    lidar_topics_candidate = [
        # "/sensing/lidar/concatenated/pointcloud",
                                "/sensing/lidar/left_upper/pandar_packets",
                                # "/sensing/lidar/right_upper/pandar_packets",
                                # "/sensing/lidar/rear_upper/pandar_packets",
                                # "/sensing/lidar/right_lower/pandar_packets",
                                # "/sensing/lidar/left_lower/pandar_packets",
                                # "/sensing/lidar/front_lower/pandar_packets",
                                # "/sensing/lidar/front_upper/pandar_packets",
                                "/sensing/lidar/left_upper/pointcloud_raw_ex"
                                ]
    # Setup GNSS topic
    gnss_topic_candidate = ["/sensing/gnss/septentrio/nav_sat_fix",
                                "/sensing/gnss/ublox/nav_sat_fix",
                                "/sensing/gnss/nav_sat_fix"]
    # Setup imu topic
    imu_topic_candidate = ["/sensing/imu/imu_data",
                            "/sensing/imu/tamagawa/imu_raw"]
    # Setup tf_static topic
    tf_static_topic_candidate = ["/tf_static"]
    # Setup velocity_status topic
    velocity_status_topic_candidate = ["/vehicle/status/velocity_status"]

    # Setup a dictionary for rosbag metadata
    bag_path = ""
    for (dirpath, dirnames, filenames) in os.walk(in_bag_dir):
        for fname in filenames:
            if fname.endswith(".db3"):
                bag_path = dirpath + "/" + fname
                break

    info = Info()
    metadata = info.read_metadata(bag_path, "sqlite3")
    bag_topic_dict = {}
    
    for topic_info in metadata.topics_with_message_count:
        bag_topic_dict[topic_info.topic_metadata.name] = topic_info.topic_metadata.type

    topic_dict["lidar"] = topic_search(lidar_topics_candidate, bag_topic_dict)
    topic_dict["gnss"] = topic_search(gnss_topic_candidate, bag_topic_dict)
    topic_dict["imu"] = topic_search(imu_topic_candidate, bag_topic_dict)
    topic_dict["tf_static"] = topic_search(tf_static_topic_candidate, bag_topic_dict)
    topic_dict["velocity_status"] = topic_search(velocity_status_topic_candidate, bag_topic_dict)
    
    print(topic_dict)

# Filter a rosbag to keep specified topcis only
def rosbag_filter(in_bag: str, out_bag: str, topic_mark: dict):
    reader = SequentialReader()
    bag_storage_options = StorageOptions(uri = in_bag, storage_id = "sqlite3")
    bag_converter_options = ConverterOptions(input_serialization_format = "cdr", output_serialization_format = "cdr")
    reader.open(bag_storage_options, bag_converter_options)

    writer = SequentialWriter()
    bag_storage_options = StorageOptions(uri = out_bag, storage_id = "sqlite3")
    bag_converter_options = ConverterOptions(input_serialization_format = "cdr", output_serialization_format = "cdr")
    writer.open(bag_storage_options, bag_converter_options)

    all_topic_metadata = reader.get_all_topics_and_types()

    for topic_metadata in all_topic_metadata:
        if topic_metadata.name in topic_mark:
            writer.create_topic(
                TopicMetadata(
                    name = topic_metadata.name, 
                    type = topic_metadata.type, 
                    serialization_format = "cdr"
                )
            )

    while reader.has_next():
        (topic, data, stamp) = reader.read_next()

        if topic in topic_mark:
            writer.write(topic, data, stamp)

# Step 1: Pre-processing the input rosbags
# Remove unused topics and merge small rosbags to a big one
def preprocessing(topic_dict: dict, in_bag_dir: str, work_dir: str):
    """
    Filtering rosbag to extract necessary topics only, also
    generate a metadata.yaml for the filtered rosbag.
    :param topic_dict: dict, a dictionary of required topics from the input rosbag
    :param in_bag_dir: str, path to the input rosbag
    :param work_dir: str, path to the working directory, which contains all temporal and output data
    """
    # If the merged rosbag already exist, quit
    merged_bag = os.path.normpath(work_dir + "/merged.db3")
    if os.path.exists(merged_bag):
        print("Merged rosbag already exists at %s. Do you want to re-create? [y/n]"%(merged_bag))
        if input() != "y":
            return

    # Scan the input bag directory and get all files having the .db3 extension
    bag_list = []
    filtered_bag_list = []
    for (dirpath, dirnames, filenames) in os.walk(in_bag_dir):
        for fname in filenames:
            if fname.endswith(".db3"):
                bag_list.append(dirpath + fname)
    
    bag_list.sort()

    topic_set = set()

    for (topic_tag, topic_names) in topic_dict.items():
        for name in topic_names:
            print("Adding topic %s"%(name))
            topic_set.add(name)

    # Filter every input rosbag
    for in_bag in bag_list:    
        filtered_in_bag = work_dir + "/" + os.path.splitext(os.path.basename(in_bag))[0] + "_filtered.db3"
        rosbag_filter(in_bag, filtered_in_bag, topic_set)
        filtered_bag_list.append(filtered_in_bag)
    
    # Merge all filtered rosbag to a single one
    merge_cmd = "ros2 bag merge -o " + merged_bag
    for filtered_in_bag in filtered_bag_list:
        merge_cmd += " " + filtered_in_bag
    
    call(merge_cmd, shell = True)

    # Delete all filtered rosbags, only keep the merged rosbag
    for filtered_in_bag in filtered_bag_list:
        call("rm -fr " + filtered_in_bag, shell = True)

# Step 2: compare the rosbag with the PCD map and detect changed areas
def map_change_checking(topic_dict: dict, work_dir: str, map_path: str, m4e_mount_path: str):
    """
    Compare the filtered rosbag with the PCD map, and detect map areas that changed.
    Filter the filtered rosbag to extract scans that cover the changed areas only,
    and remove invalid GNSS messages from the filtered rosbag. Convert the remaining
    of the filtered rosbag to the ROS1 format.
    """
    # Generate rosbag for evaluation
    eval_bag = os.path.normpath(work_dir + "/eval_bag")
    vehicle_model = "j6_gen1"
    sensor_model = "aip_x2"

    # Launch pilot-auto.x2's logging simulator
    # Popen(["ros2", "launch", "autoware_launch", "logging_simulator.launch.xml", \
    #                 "map_path:=" + map_path, "vehicle_model:=j6_gen1", "sensor_model:=aip_x2"])
    #                 # ,\
    #                 # "perception:=false", "planning:=false", "control:=false"])
    # Wait for the logging simulator finishes setting up
    # time.sleep(360)
    # record_proc = Popen(["ros2", "bag", "record", "-o", eval_bag, "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", \
    #                 "/localization/util/downsample/pointcloud"])
    # ros2 bag record -o eval_bag /localization/pose_twist_fusion_filter/biased_pose_with_covariance /localization/util/downsample/pointcloud
    
    # Play the rosbag and wait for it to finish
    # call("ros2 bag play " + work_dir + "/merged.db3 -r 2.0 -s sqlite3 --clock 100", shell = True)

    # Quit
    # os.system("/usr/bin/pgrep ros | /usr/bin/awk '{ print \"kill -2 $(pgrep -P \", $1, \") > /dev/null 2>&1\" }' | sh")
    # os.system("/usr/bin/pgrep ros | /usr/bin/awk '{ print \"kill -2 \", $1, \" > /dev/null 2>&1\" }' | sh")
    # os.system("/usr/bin/pgrep ros | /usr/bin/awk '{ print \"kill -9 $(pgrep -P \", $1, \") > /dev/null 2>&1\" }' | sh")
    # os.system("/usr/bin/pgrep ros | /usr/bin/awk '{ print \"kill -9 \", $1, \" > /dev/null 2>&1\" }' | sh")
    
    # # Check if the eval rosbag exists
    # if not(os.path.exists(eval_bag)):
    #     print("Evaluation rosbag does not exist at %s. Exit..."%(eval_bag))
    #     exit()
    # else:
    #     print("Evaluation rosbag exists at %s."%(eval_bag))

    # # Run map_assessment_tools
    # mast_output = os.path.normpath(work_dir + "/mast_result/")

    # mast_cmd = "ros2 launch map_assessment_tools map_evaluation.launch.py rosbag_file_name:=" + \
    #             eval_bag + " map_path:=" + map_path + " save_dir:=" + mast_output + \
    #             " sensor_topic_name:=/localization/util/downsample/pointcloud roc:=true"

    # if not(os.path.exists(mast_output)):
    #     os.mkdir(mast_output)
    #     call(mast_cmd, shell = True)
    # else:
    #     print("map_assessment_tools output already exists. Do you want to re-run? [y/n]")
    #     if input() == "y":
    #         call(mast_cmd, shell = True)

    # Filter the merged bag to keep the scans that cover the changed area
    # Also remove invalid GNSS messages
    final_bag = os.path.normpath(work_dir + "/final_bag.db3")
    filter_cmd = "ros2 run map_assessment_tools rosbag_interval_filter.py " + \
                    work_dir + "/merged.db3 " + final_bag + \
                    " --gnss_topic " + ",".join(topic_dict["gnss"]) + \
                    " --sensor_topic " + ",".join(topic_dict["lidar"])
                    # " --save_dir " + mast_output +          \
                    # " --gnss_topic " + topic_dict["gnss"] + \
                    # " --sensor_topic " + topic_dict["lidar"]

    if os.path.exists(final_bag):
        print("The filtered final ROS2 rosbag already exists. Do you want to re-filter? [y/n]")
        if input() == "y":
            shutil.rmtree(final_bag)
            call(filter_cmd, shell = True)
    else:        
        call(filter_cmd, shell = True)

    # Convert the final rosbag from ROS2 format to ROS1 format
    print("Convert to ROS1 format...")
    call("/home/anh/Work/rosbags/venv/bin/rosbags-convert-2to1 " + final_bag, shell = True)
    # Move the ROS1 bag to map4-cli output path
    m4e_path = os.path.normpath(m4e_mount_path + "/" + os.path.basename(work_dir) + "/")
    if os.path.exists(m4e_path):
        print("map4-cli mount path already exists at %s. Do you want to re-create? [y/n]"%(m4e_path))
        if input() == "y":
            shutil.rmtree(m4e_path)
            os.mkdir(m4e_path)
            call("mv " + final_bag + ".bag " + m4e_path + "/input_rosbag.bag", shell = True)
            print("A ROS1 rosbag was copied to %s."%(m4e_path + "/input_rosbag.bag"))
    else:
        os.mkdir(m4e_path)            
        call("mv " + final_bag + ".bag " + m4e_path + "/input_rosbag.bag", shell = True)
        print("A ROS1 rosbag was copied to %s."%(m4e_path + "/input_rosbag.bag"))
    
    # Merge pcd files and put them to m4e_path/map
    m4e_map_path = os.path.normpath(m4e_path + "/map")
    if os.path.exists(m4e_map_path):
        print("The map folder for PCD update already exists at %s. Do you want to re-create? [y/n]"%(m4e_map_path))
        if input() == "y":
            shutil.rmtree(m4e_map_path)
            os.mkdir(m4e_map_path)
    else:
        os.mkdir(m4e_map_path)

    merge_cmd = "ros2 launch autoware_pointcloud_merger pointcloud_merger.launch.xml " + \
                    "input_pcd_dir:=" + map_path + "/pointcloud_map.pcd/" + \
                    " output_pcd:=" + m4e_map_path + "/merged.pcd"
    
    call(merge_cmd, shell = True)
    print("PCD Update preparation is done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the folder containing PCD files of the old map")
    parser.add_argument("bag_path", help="The path to the input rosbag")
    parser.add_argument("output_path", help="The path to the folder containing the updated PCD files")
    parser.add_argument("--type", help="Type of launcher pilot-auto.x2/pilot-auto.xx1", default="pilot-auto.x2", required=False)

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.map_path))
    print("Input rosbag at %s" % (args.bag_path))
    print("Output PCD map at %s" % (args.output_path))

    # Step 0: Setup required topics
    # To evaluate and update map, the filtered rosbag must contain
    # IMU, GNSS, Lidar, TF Static, and velocity status
    topic_dict = {
        "lidar": "unknown",
        "gnss": "unknown",
        "imu": "unknown",
        "tf_static": "unknown",
        "velocity_status": "unknown",
    }
    # Cleanup the working directory
    work_dir = os.path.normpath(args.output_path)
    setup(work_dir)
    # Setup topics for filtering and merge
    topic_setup(topic_dict, args.bag_path)
    # Step 1: Pre-process rosbag
    preprocessing(topic_dict, args.bag_path, work_dir)
    # Step 2: Check map changes
    m4e_mount_path = os.path.normpath("/media/anh/AnhNguyen/map4_engine_output/")
    map_change_checking(topic_dict, work_dir, args.map_path, m4e_mount_path)
    # Step 3: Update PCD map
    # Step 4: Post validation
