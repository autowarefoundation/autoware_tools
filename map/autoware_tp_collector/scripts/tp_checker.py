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
import open3d as o3d
import matplotlib.pyplot as plt
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

class TPChecker:
    def __init__(self):
        self.segment_dict = {}  # Segment indices, TP, kdtree
        self.base_path = ""     # Path to the base directory of the PCD segments
        self.ndt_res_df = pd.DataFrame()    # A set of tp read from a rosbag
        self.pcd_map_2d = pd.DataFrame(columns = ["x", "y", "color"])
                                # A 2D map of pcd, color indicates the map changes
        self.point_num = 0

    # Read the input map directory and setup the segment dictionary
    def set_pcd_map(self, pcd_map_dir: str):
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
        segment_df = pd.DataFrame(columns = ["x", "y", "tp", "pcd"])

        with open(metadata_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    # Read PCD files and store them into a 2D map
                    seg_path = os.path.join(pcd_path, key)
                    pcd = o3d.io.read_point_cloud(seg_path)
                    # Downsample the segment to a greater resolution
                    # This segment is just for visualizing the result, no need to be detailed
                    ds_pcd = self.__downsample(np.asarray(pcd.points))
                    segment_df.loc[len(segment_df)] = [float(value[0]), float(value[1]), 0, ds_pcd]
                    self.point_num += len(ds_pcd)

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
        
        self.segment_dict["segment_df"] = segment_df
        self.segment_dict["kdtree"] = kdtree
        self.base_path = pcd_path

    ##### Stamp search #####
    def __stamp_search(self, stamp: int, tp_df: pd.DataFrame) -> int:
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
    def __collect_rosbag_tp(self, bag_path: str) -> pd.DataFrame:
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
        self.ndt_res_df = pd.DataFrame(columns = ["x", "y", "tp", "avg_tp"])

        progress_bar = tqdm.tqdm(total=len(pose_df))

        for row in pose_df.itertuples():
            progress_bar.update(1)

            stamp = row.stamp
            pose = row.pose_msg
            # Find the tp whose stamp is the closest to the pose
            tid = self.__stamp_search(stamp, tp_df)
            
            if tid >= 0:
                self.ndt_res_df[len(self.ndt_res_df)] = [pose.position.x, pose.position.y, tp_df["tp"][tid], 0.0]

    # Mark the segments which may has changed
    def __mark_changes(self):
        print("Checking map changes...\n")
        change_mark = np.zeros(len(self.segment_dict["segment_df"]), dtype = int)

        progress_bar = tqdm.tqdm(total = len(self.ndt_res_df))

        for row in self.ndt_res_df.itertuples():
            progress_bar.update(1)
            nn_idx = self.segment_dict["kdtree"].query_ball_point([row.x, row.y], 20.0)

            if len(nn_idx) == 0:
                continue

            sum_tp = 0.0

            for idx in nn_idx:
                sum_tp += self.segment_dict["segment_df"].loc[idx, "tp"]
            
            row.avg_tp = sum_tp / len(nn_idx)

            if abs(row.tp - row.avg_tp) / row.tp >= 0.2:
                for idx in nn_idx:
                    change_mark[idx] = 1

        # Export the 2D map
        print("Coloring 2D map...\n")
        progress_bar = tqdm.tqdm(total = len(change_mark))
        self.pcd_map_2d = pd.DataFrame({"x" : np.zeros(self.point_num), 
                                        "y" : np.zeros(self.point_num), 
                                        "color" : ["" for j in range(self.point_num)]})
        
        print("Length of change mark = {0}".format(len(change_mark)))
        i = 0

        for idx, value in enumerate(change_mark):
            progress_bar.update(1)
            if value == 0:
                color = "blue"
            else:
                color = "red"
            # Insert points with color to the 2D map
            for p in self.segment_dict["segment_df"].loc[idx, "pcd"]:
                self.pcd_map_2d.loc[i] = [p[0], p[1], color]
                i += 1

    # Downsample a point cloud
    def __downsample(self, cloud: np.array) -> np.array:
        return cloud        

    # Save the tp and average tps to CSV file #####
    def save_results(self, result_path: str):
        self.ndt_res_df.to_csv(result_path)

    # Plot the points with color from the 2D segment map
    def show(self, result_path: str):
        print("Plotting")
        plt.scatter(self.pcd_map_2d["x"], self.pcd_map_2d["y"], 
                    c = self.pcd_map_2d["color"])
        
        plt.legend()
        plt.grid()
        plt.savefig(result_path)

        try:
            plt.show()
        except KeyboardInterrupt:
            print("Quit by keyboard interrupt")

    def check(self, rosbag_path: str):
        # Read the rosbag and get the ndt poses and corresponding tps
        self.__collect_rosbag_tp(rosbag_path)
        # Update the TPs of segments
        self.__mark_changes()
        # Save the new TPs
        self.save_results()
        # Display the result
        self.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the PCD folder")
    parser.add_argument("bag_path", help="The path to the input rosbag")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.map_path))
    print("Input rosbag at %s" % (args.bag_path))

    # Run
    checker = TPChecker()

    checker.set_pcd_map(args.map_path)
    checker.check(args.bag_path)
