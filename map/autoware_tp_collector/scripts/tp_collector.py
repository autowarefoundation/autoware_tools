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
import csv
import os
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import open3d as o3d
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosbag2_py import BagMetadata
from rosbag2_py import ConverterOptions
from rosbag2_py import Info
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata
from scipy import spatial as sp
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg as std_msgs
from tier4_debug_msgs.msg import Float32Stamped
import tqdm
import yaml


class TPCollector(Node):
    def __init__(self):
        super().__init__("tp_collector")
        self.pcd_path = None
        self.yaml_path = None
        self.score_path = None
        self.segment_df = None
        self.kdtree = None

    def __compute_total_message_count(self, rosbag_path):
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

    ##### Read the YAML file to get the list of PCD segments and scores #####
    def __get_pcd_segments_and_scores(self, pcd_map_dir: str):
        if not os.path.exists(pcd_map_dir):
            print("Error: %s does not exist!" % (pcd_map_dir))
            exit()

        self.pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd/")

        if not os.path.exists(self.pcd_path):
            print("Error: %s does not exist!" % (self.pcd_path))
            exit()

        self.yaml_path = os.path.join(pcd_map_dir, "pointcloud_map_metadata.yaml")

        # Create a dataframe to record the avg tp of 2D segments
        self.segment_df = pd.DataFrame(columns=["x", "y", "tp", "key"])

        with open(self.yaml_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    self.segment_df.loc[len(self.segment_df)] = [
                        float(value[0]),
                        float(value[1]),
                        0,
                        key,
                    ]

        # A 2D array that contains the 2D coordinates of segments
        # We'll use this to build a KDtree
        seg_tree_nodes = np.zeros((len(self.segment_df), 2), dtype=float)

        for index, row in self.segment_df.iterrows():
            seg_tree_nodes[index, :] = [row["x"], row["y"]]

        # Create a 2D kdtree on the segment list
        self.kdtree = sp.KDTree(seg_tree_nodes)

        # Load the score map
        self.score_path = os.path.join(pcd_map_dir, "scores.csv")

        if os.path.exists(self.score_path):
            with open(self.score_path, "r") as f:
                reader = csv.reader(f)
                for index, row in enumerate(reader):
                    self.segment_df.loc[index, "tp"] = float(row[1])
        else:
            # If the score file does not exist, initialize scores to all 0
            self.segment_df.loc[:, "tp"] = 0

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
        bag_storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
        bag_converter_options = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        reader.open(bag_storage_options, bag_converter_options)

        print("Reading rosbag...")

        total_message_count = self.__compute_total_message_count(bag_path)
        progress_bar = tqdm.tqdm(total=total_message_count)

        pose_df = pd.DataFrame(columns=["stamp", "pose_msg"])
        tp_df = pd.DataFrame(columns=["stamp", "tp"])

        while reader.has_next():
            progress_bar.update(1)
            (topic, data, stamp) = reader.read_next()

            # if topic == "/localization/pose_twist_fusion_filter/biased_pose_with_covariance":
            if (
                topic
                == "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias"
            ):
                pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
                stamp = pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec
                pose_df.loc[len(pose_df)] = [stamp, pose_msg.pose.pose]
            elif topic == "/localization/pose_estimator/transform_probability":
                tp_msg = deserialize_message(data, Float32Stamped)
                stamp = tp_msg.stamp.sec * 1e9 + tp_msg.stamp.nanosec

                tp_df.loc[len(tp_df)] = [stamp, tp_msg.data]

        progress_bar.close()

        # Now from the two list above build a table of estimated pose and corresponding TPs
        ndt_res_df = pd.DataFrame(columns=["x", "y", "tp"])

        progress_bar = tqdm.tqdm(total=len(pose_df))

        for row in pose_df.itertuples():
            progress_bar.update(1)
            stamp = row.stamp
            pose = row.pose_msg
            # Find the tp whose stamp is the closest to the pose
            tid = self.__stamp_search(stamp, tp_df)

            if tid >= 0:
                ndt_res_df.loc[len(ndt_res_df)] = [
                    pose.position.x,
                    pose.position.y,
                    tp_df["tp"][tid],
                ]

        progress_bar.close()

        return ndt_res_df

    ##### Update map's TP #####
    def __update_avg_tp(self, ndt_res_df: pd.DataFrame):
        print("Updating segments' TP...")
        # Iterate on poses and the corresponding TPs
        progress_bar = tqdm.tqdm(total=len(ndt_res_df))

        for tp_row in ndt_res_df.itertuples():
            progress_bar.update(1)
            # Find indices of map segments that cover the current pose
            nn_idx = self.kdtree.query_ball_point([tp_row.x, tp_row.y], 20.0)

            for idx in nn_idx:
                # If the TP from rosbag is greater than the current TP of the segment,
                # replace the segment TP with the TP from the rosbag
                # TODO: record the average TP of segments
                if tp_row.tp > self.segment_df.loc[idx, "tp"]:
                    self.segment_df.loc[idx, "tp"] = tp_row.tp
        progress_bar.close()

    ##### Save the segment TPs #####
    def __save_tps(self):
        print("Saving TP to files")
        with open(self.score_path, "w") as f:
            f.write("segment,tp\n")
            for row in self.segment_df.itertuples():
                f.write("{0},{1}\n".format(row.key, row.tp))
        print("Done. Segments' TPs are saved at {0}.".format(self.score_path))

    def processing(self, pcd_map_dir: str, rosbag_path: str):
        # Get the segment lists and scores
        self.__get_pcd_segments_and_scores(pcd_map_dir)

        # Read the rosbag and get the ndt poses and corresponding tps
        ndt_res_df = self.__collect_rosbag_tp(rosbag_path)

        # Update the TPs of segments
        self.__update_avg_tp(ndt_res_df)

        # Save the new TPs
        self.__save_tps()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the PCD folder")
    parser.add_argument("bag_path", help="The path to the input rosbag")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.map_path))
    print("Input rosbag at %s" % (args.bag_path))

    # Run
    tp_collector = TPCollector()
    tp_collector.processing(args.map_path, args.bag_path)
