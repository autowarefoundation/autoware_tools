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
import shutil
import struct
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
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from scipy import spatial as sp
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg as std_msgs
from tier4_debug_msgs.msg import Float32Stamped
import tqdm
import yaml


class TPChecker(Node):

    def __init__(self):
        super().__init__("tp_checker")
        self.segment_df = None  # Segment indices, TP, kdtree
        self.kdtree = None  # A 2D kdtree to search for segments in the vicinity of poses
        self.ndt_res_df = None  # A set of tp read from a rosbag
        self.pcd_path = None  # Path to the directory containing PCD segments
        self.changed_dir = None  # A directory contains the segments that need to be examined
        self.result_csv = None  # Path to the result CSV file

    # Read the input map directory and setup the segment dictionary
    def __set_pcd_map(self, pcd_map_dir: str):
        # Check if the required folders/files exist
        if not os.path.exists(pcd_map_dir):
            print("Error: {0} does not exist!".format(pcd_map_dir))
            exit()

        self.pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd")

        if not os.path.exists(self.pcd_path):
            print("Error: {0} does not exist!".format(self.pcd_path))
            exit()

        metadata_path = os.path.join(pcd_map_dir, "pointcloud_map_metadata.yaml")

        if not os.path.exists(metadata_path):
            print("Error: {0} does not exist!".format(metadata_path))
            exit()

        score_path = os.path.join(pcd_map_dir, "scores.csv")

        if not os.path.exists(score_path):
            print("Error: {0} does not exist!".format(score_path))
            exit()

        # Read the metadata file and get the list of segments
        print("Read the PCD map at {0}".format(pcd_map_dir))
        self.segment_df = pd.DataFrame(columns=["x", "y", "tp", "seg_key"])

        with open(metadata_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    self.segment_df.loc[len(self.segment_df)] = [
                        float(value[0]),
                        float(value[1]),
                        0,
                        key,
                    ]

        # Create a 2D kdtree on the segments
        seg_tree_nodes = np.zeros((len(self.segment_df), 2), dtype=float)

        for index, row in self.segment_df.iterrows():
            seg_tree_nodes[index, :] = [row["x"], row["y"]]

        self.kdtree = sp.KDTree(seg_tree_nodes)

        # Read the score file
        with open(score_path, "r") as f:
            reader = csv.reader(f)

            for index, row in enumerate(reader):
                self.segment_df.loc[index, "tp"] = float(row[1])

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

        total_message_count = self.__compute_total_message_count(bag_path)

        print("Read the input rosbag at {0}".format(bag_path))
        progress_bar = tqdm.tqdm(total=total_message_count)

        pose_df = pd.DataFrame(columns=["stamp", "pose_msg"])
        tp_df = pd.DataFrame(columns=["stamp", "tp"])

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

        progress_bar.close()
        # Now from the two list above build a table of estimated pose and corresponding TPs
        self.ndt_res_df = pd.DataFrame(columns=["x", "y", "tp", "expected_tp"])

        print("Prepare the tp list...")

        progress_bar = tqdm.tqdm(total=len(pose_df))

        for row in pose_df.itertuples():
            progress_bar.update(1)

            stamp = row.stamp
            pose = row.pose_msg
            # Find the tp whose stamp is the closest to the pose
            tid = self.__stamp_search(stamp, tp_df)

            if tid >= 0:
                self.ndt_res_df.loc[len(self.ndt_res_df)] = [
                    pose.position.x,
                    pose.position.y,
                    tp_df["tp"][tid],
                    0.0,
                ]

        progress_bar.close()

    # Mark the segments which may has changed
    def __mark_changes(self):
        print("Checking map changes... tp len = {0}".format(len(self.ndt_res_df)))
        self.change_mark = np.zeros(len(self.segment_df), dtype=int)

        progress_bar = tqdm.tqdm(total=len(self.ndt_res_df))

        for i, row in self.ndt_res_df.iterrows():
            progress_bar.update(1)
            nn_idx = self.kdtree.query_ball_point([row.x, row.y], 20.0)

            if len(nn_idx) == 0:
                continue

            sum_tp = 0.0

            for idx in nn_idx:
                sum_tp += self.segment_df.loc[idx, "tp"]

            self.ndt_res_df.loc[i, "expected_tp"] = sum_tp / float(len(nn_idx))

            if abs(row.tp - row.expected_tp) / row.tp >= 0.2:
                for idx in nn_idx:
                    self.change_mark[idx] = 1

        progress_bar.close()

        # Save the changed segments
        progress_bar = tqdm.tqdm(total=len(self.change_mark))

        for idx, value in enumerate(self.change_mark):
            progress_bar.update(1)
            # Copy changed segments to the destinated folder
            seg_key = self.segment_df.loc[idx, "seg_key"]

            if value != 0:
                shutil.copyfile(
                    os.path.join(self.pcd_path, seg_key), os.path.join(self.changed_dir, seg_key)
                )
        progress_bar.close()

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

    # Save the tp and average tps to CSV file #####
    def __save_results(self: str):
        self.ndt_res_df.to_csv(self.result_csv)
        print("The checking results are saved at {0}".format(self.result_csv))

    def __show(self):
        ros_float_dtype = sensor_msgs.PointField.FLOAT32
        ros_uint32_dtype = sensor_msgs.PointField.UINT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=ros_float_dtype, count=1),
            sensor_msgs.PointField(name="y", offset=itemsize, datatype=ros_float_dtype, count=1),
            sensor_msgs.PointField(
                name="z", offset=itemsize * 2, datatype=ros_float_dtype, count=1
            ),
            sensor_msgs.PointField(
                name="rgba", offset=itemsize * 3, datatype=ros_uint32_dtype, count=1
            ),
        ]

        points = []
        pc2_width = 0

        progress_bar = tqdm.tqdm(total=len(self.segment_df))
        origin = None

        for i, tuple in self.segment_df.iterrows():
            progress_bar.update(1)
            # Load the current segment
            pcd = o3d.io.read_point_cloud(os.path.join(self.pcd_path, tuple.seg_key))
            np_pcd = np.asarray(pcd.points)
            rgba = self.__set_color_based_on_mark(self.change_mark[i])

            for p in np_pcd:
                if origin == None:
                    origin = [p[0], p[1], p[2]]
                pt = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2], rgba]
                points.append(pt)
                pc2_width += 1

        print("Publishing result...")
        header = std_msgs.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        pc2_msg = pc2.create_cloud(header, fields, points)
        pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "/autoware_tp_checker", 10)

        while True:
            pcd_publisher.publish(pc2_msg)
            time.sleep(5)

    def __set_color_based_on_mark(self, mark) -> int:
        # The may-be-changed segments are colored red
        # The may-not-be-changed segments are colored white
        if mark == 1:
            r = 255
            g = 0
            b = 0
        else:
            r = 255
            g = 255
            b = 255
        a = 255

        tmp_rgb = struct.pack("BBBB", b, g, r, a)
        rgba = struct.unpack("I", tmp_rgb)[0]

        return rgba

    def processing(self, pcd_path: str, rosbag_path: str, result_path: str):
        if not os.path.exists(result_path):
            os.makedirs(result_path)
        else:
            shutil.rmtree(result_path)
            os.makedirs(result_path)

        self.changed_dir = os.path.join(result_path, "changed_dir")
        os.makedirs(self.changed_dir)
        self.result_csv = os.path.join(result_path, "result.csv")

        self.__set_pcd_map(pcd_path)

        # Read the rosbag and get the ndt poses and corresponding tps
        self.__collect_rosbag_tp(rosbag_path)
        # Update the TPs of segments
        self.__mark_changes()
        self.__save_results()
        self.__show()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the PCD folder")
    parser.add_argument("bag_path", help="The path to the input rosbag")
    parser.add_argument("result_path", help="The path to the result folder")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at {0}".format(args.map_path))
    print("Input rosbag at {0}".format(args.bag_path))
    print("Results are saved at {0}".format(args.result_path))

    # Run
    checker = TPChecker()

    checker.processing(args.map_path, args.bag_path, args.result_path)
