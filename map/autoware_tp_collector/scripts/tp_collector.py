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
import os
import csv
import yaml
from scipy import spatial as sp
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tier4_debug_msgs.msg import Float32Stamped
from builtin_interfaces.msg import Time
import pandas as pd
import tqdm


def compute_total_message_count(rosbag_path):
    if rosbag_path.endswith(".db3"):
        yaml_path = os.path.join(os.path.dirname(rosbag_path), "metadata.yaml")
    else:
        yaml_path = os.path.join(rosbag_path, "metadata.yaml")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    total_message_count = 0
    for topic_data in data["rosbag2_bagfile_information"]["topics_with_message_count"]:
        total_message_count += topic_data["message_count"]

    return total_message_count

##### Convert names of pcd segments to 2d coordinates #####
def name_to_coordinate(seg_name: str):
    # Remove the extension
    name_only = os.path.splitext(seg_name)[0]
    str_coors = name_only.split("_")

    # Return the coordinates as a 1x2 ndarray
    return np.array(str_coors, dtype = float).reshape(1, 2)

##### Read the YAML file to get the list of PCD segments and scores #####
def get_pcd_segments_and_scores(pcd_map_dir: str) -> dict:
    if not os.path.exists(pcd_map_dir):
        print("Error: %s does not exist!"%(pcd_map_dir))
        exit()

    pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd/")

    if not os.path.exists(pcd_path):
        print("Error: %s does not exist!"%(pcd_path))
        exit()

    yaml_path = os.path.join(pcd_map_dir, "pointcloud_map_metadata.yaml")

    # Create a dataframe to record the avg tp of 2D segments
    segment_df = pd.DataFrame(columns = ["x", "y", "tp"])

    with open(yaml_path, "r") as f:
        for key, value in yaml.safe_load(f).items():
            if key != "x_resolution" and key != "y_resolution":
                segment_df.loc[len(segment_df)] = [float(value[0]), float(value[1]), 0]

    # A 2D array that contains the 2D coordinates of segments
    # We'll use this to build a KDtree
    seg_tree_nodes = np.zeros((len(segment_df), 2), dtype = float)

    for index, row in segment_df.iterrows():
        seg_tree_nodes[index,:] = [row["x"], row["y"]]

    # Create a 2D kdtree on the segment list
    kdtree = sp.KDTree(seg_tree_nodes)

    # Load the score map
    score_path = os.path.join(pcd_map_dir, "scores.csv")

    if os.path.exists(score_path):
        with open(score_path, "r") as f:
            reader = csv.reader(f)
            for index, row in enumerate(reader):
                segment_df.loc[index, "tp"] = float(row[1])
    else:
        # If the score file does not exist, initialize scores to all 0
        segment_df.loc[:,"tp"] = 0

    return {"segment_df": segment_df, "kdtree": kdtree}
    
##### Stamp search #####
def stamp_search(stamp: int, tp_df: pd.DataFrame) -> int:
    left = 0
    right = len(tp_df)
    res = -1

    # Find the closest stamp that goes before the stamp key
    while left < right:
        mid = (left + right) // 2
        cur_stamp = tp_df["stamp"][mid]

        if cur_stamp <= stamp:
            left = mid + 1
            res = mid
        elif cur_stamp > stamp:
            right = mid - 1

    return res

##### Read the input rosbag to obtain ndt pose and TP values #####
def collect_rosbag_tp(bag_path: str) -> pd.DataFrame:
    reader = SequentialReader()
    bag_storage_options = StorageOptions(uri = bag_path, storage_id = "sqlite3")
    bag_converter_options = ConverterOptions(
        input_serialization_format = "cdr", output_serialization_format = "cdr"
    )
    reader.open(bag_storage_options, bag_converter_options)

    total_message_count = compute_total_message_count(bag_path)
    progress_bar = tqdm.tqdm(total=total_message_count)


    pose_df = pd.DataFrame(columns = ["stamp", "pose_msg"])
    tp_df = pd.DataFrame(columns = ["stamp", "tp"])

    while reader.has_next():
        progress_bar.update(1)
        (topic, data, stamp) = reader.read_next()

        if topic == "/localization/pose_twist_fusion_filter/biased_pose_with_covariance":
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            stamp = pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec
            pose_df.loc[len(pose_df)] = [stamp, pose_msg.pose.pose]
        elif topic == "/localization/pose_estimator/transform_probability":
            tp_msg = deserialize_message(data, Float32Stamped)
            stamp = tp_msg.stamp.sec * 1e9 + tp_msg.stamp.nanosec

            tp_df.loc[len(tp_df)] = [stamp, tp_msg.data]
    
    # Now from the two list above build a table of estimated pose and corresponding TPs
    ndt_res_df = pd.DataFrame(columns = ["x", "y", "tp"])

    progress_bar = tqdm.tqdm(total=len(pose_df))

    for row in pose_df.itertuples():
        progress_bar.update(1)
        stamp = row.stamp
        pose = row.pose_msg
        # Find the tp whose stamp is the closest to the pose
        tid = stamp_search(stamp, tp_df)
        
        if tid >= 0:
            ndt_res_df.loc[len(ndt_res_df)] = [pose.position.x, pose.position.y, tp_df["tp"][tid]]
        
    return ndt_res_df

##### Update map's TP #####
def update_avg_tp(ndt_res_df: pd.DataFrame, segment_dict: dict):
    print("UPDATE AVG TP, len of ndt_res_df = {0}".format(len(ndt_res_df)))
    # Iterate on poses and the corresponding TPs
    progress_bar = tqdm.tqdm(total=len(ndt_res_df))

    for tp_row in ndt_res_df.itertuples():
        # Find indices of map segments that cover the current pose
        nn_idx = segment_dict["kdtree"].query_ball_point([tp_row.x, tp_row.y], 20.0)

        for idx in nn_idx:
            # If the TP from rosbag is greater than the current TP of the segment,
            # replace the segment TP with the TP from the rosbag
            # TODO: record the average TP of segments
            if tp_row.tp > segment_dict["segment_df"].loc[idx, "tp"]:
                segment_dict["segment_df"].loc[idx, "tp"] = tp_row.tp

##### Save the segment TPs #####
def save_tps(pcd_map_dir: str, segment_dict: dict):
    print("SAVE AVG TP")

    score_path = os.path.join(pcd_map_dir, "score.csv")

    print(segment_dict)

    with open(score_path, "w") as f:
        for row in segment_dict["segment_df"].itertuples():
            f.write("{0}_{1},{2}\n".format(int(row.x), int(row.y), row.tp))

def processing(pcd_map_dir: str, rosbag_path: str):
    # Get the segment lists and scores
    segment_dict = get_pcd_segments_and_scores(pcd_map_dir)
    print("PAUSE at 179 segment dict is \n{0}".format(segment_dict))

    # Read the rosbag and get the ndt poses and corresponding tps
    ndt_res_df = collect_rosbag_tp(rosbag_path)
    # Update the TPs of segments
    update_avg_tp(ndt_res_df, segment_dict)
    # Save the new TPs
    save_tps(pcd_map_dir, segment_dict)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the PCD folder")
    parser.add_argument("bag_path", help="The path to the input rosbag")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.map_path))
    print("Input rosbag at %s" % (args.bag_path))

    # Run
    processing(args.map_path, args.bag_path)
