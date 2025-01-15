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

##### Convert names of pcd segments to 2d numpy array #####
def name_to_coordinate(seg_name: str):
    # Remove the extension
    name_only = os.path.splitext(seg_name)[0]
    str_coors = name_only.split("_")

    # Return the coordinates as a 1x2 ndarray
    return np.array(str_coors, dtype = float).reshape(1, 2)

def get_pcd_segments_and_scores(pcd_map_dir: str) -> pd.DataFrame:
    # Check if the required folders/files exist
    if not os.path.exists(pcd_map_dir):
        print("Error: {0} does not exist!".format(pcd_map_dir))
        exit()
    
    pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd")

    if not os.path.exists(pcd_path):
        print("Error: {0} does not exist!".format(pcd_path))
        exit()
    
    metadata_path = os.path.join(pcd_map_dir, "pointcloud_map_metadata.yaml")

    if not os.path.exists(metadata_path):
        print("Error: {0} does not exist!".format(metadata_path))
        exit()

    score_path = os.path.join(pcd_map_dir, "score.csv")

    if not os.path.exists(score_path):
        print("Error: {0} does not exist!".format(score_path))
        exit()

    # Read the metadata file and get the list of segments
    segment_df = pd.DataFrame(columns = ["x", "y", "tp"])

    with open(metadata_path, "r") as f:
        for key, value in yaml.safe_load(f).items():
            if key != "x_resolution" and key != "y_resolution":
                seg_idx = name_to_coordinate(key)
                segment_df.loc[len(segment_df)] = [seg_idx[0], seg_idx[1], 0]

    # Create a 2D kdtree on the segments
    seg_tree_nodes = np.zeros((len(segment_df), 2), dtype = float)

    for index, row in segment_df.iterrows():
        seg_tree_nodes[index, :] = [row["x"], row["y"]]

    kdtree = sp.KDTree(seg_tree_nodes)

    # Read the score file
    with open(score_path, "r") as f:
        reader = csv.reader(f)

        for index, row in enumerate(reader):
            segment_df.loc[index, "tp"] = float(row[1])
    
    return {"segment_df" : segment_df, "kdtree" : kdtree}

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

    pose_df = pd.DataFrame(columns = ["stamp", "pose_msg"])
    tp_df = pd.DataFrame(columns = ["stamp", "tp"])

    while reader.has_next():
        (topic, data, stamp) = reader.read_next()

        if topic == "/localization/pose_twist_fusion_filter/biased_pose_with_covariance":
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            stamp = pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec

            pose_df.loc[len(pose_df)] = [stamp, pose_msg.pose.pose]
        elif topic == "/localization/pose_estimator/transform_probability":
            tp_msg = deserialize_message(data, Float32Stamped)
            stamp = tp_msg.header.stamp.sec * 1e9 + tp_msg.header.stamp.nanosec

            tp_df.loc[len(tp_df)] = [stamp, tp_msg.data]
    
    # Now from the two list above build a table of estimated pose and corresponding TPs
    ndt_res_df = pd.DataFrame(columns = ["x", "y", "tp", "avg_tp"])

    for row in pose_df.itertuples():
        stamp = row["stamp"]
        pose = row["pose_msg"]
        # Find the tp whose stamp is the closest to the pose
        tid = stamp_search(stamp, tp_df)
        
        if tid >= 0:
            ndt_res_df[len(ndt_res_df)] = [pose.position.x, pose.position.y, tp_df["data"][tid], 0.0]
        
    return ndt_res_df

def estimate_tps(segment_dict: dict, ndt_res_df: pd.DataFrame):
    for row in ndt_res_df.itertuples():
        nn_idx = segment_dict["kdtree"].query_ball_point([row["x"], row["y"]], 20.0)
        sum_tp = 0.0

        for idx in nn_idx:
            sum_tp += segment_dict["segment_df"].loc[idx, "tp"]
        
        row["avg_tp"] = sum_tp / len(nn_idx)

##### Save the tp and average tps to CSV file #####
def save_results(ndt_res_df: pd.DataFrame, result_path: str):
    ndt_res_df.to_csv(result_path)

##### Show the results on 2D map #####
def show(result_path: str):
    print("Testing")

def processing(pcd_map_dir: str, rosbag_path: str):
    # Get the segment lists and scores
    segment_dict = get_pcd_segments_and_scores(pcd_map_dir)
    # Read the rosbag and get the ndt poses and corresponding tps
    ndt_res_df = collect_rosbag_tp(rosbag_path)
    # Update the TPs of segments
    estimate_tps(segment_dict, ndt_res_df)
    # Save the new TPs
    result_path = os.path.join(pcd_map_dir, "result.csv")
    save_results(ndt_res_df, result_path)
    show(result_path)


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
