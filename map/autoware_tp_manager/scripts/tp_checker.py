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
import struct
import time

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from scipy import spatial as sp
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg as std_msgs
import tp_utility as tpu
import tqdm
import yaml


def mark_changes(candidate_segments, rosbag_tp: float, segment_dict, segment_df):
    tp_sum = 0.0
    valid_segment_num = 0

    for key in candidate_segments:
        if key in segment_dict:
            tp_sum += segment_df[segment_dict[key], 1]
            valid_segment_num += 1
    if valid_segment_num > 0:
        expected_tp = tp_sum / float(valid_segment_num)
    else:
        expected_tp = 0

    if (expected_tp > 0 and abs(expected_tp - rosbag_tp) / expected_tp >= 0.2) or (
        expected_tp == 0 and rosbag_tp > 0
    ):
        for key in candidate_segments:
            if key in segment_dict:
                segment_df[segment_dict[key], 2] += 1


class TPChecker(Node):

    def __init__(self):
        super().__init__("tp_checker")
        self.segment_df = None  # Segment indices, TP
        self.segment_dict = {}  # key: segment name, value: index to the segment_df
        self.pcd_path = None  # Path to the directory containing PCD segments
        self.changed_dir = None  # A directory contains the segments that need to be examined
        self.result_csv = None  # Path to the result CSV file
        self.tp_path = None  # Path to the file that contains TPs of map segments

    def __initialize(self, score_dir: str):
        if not os.path.exists(score_dir):
            print("Error: {0} does not exist! Abort!".format(score_dir))
            exit()

        self.pcd_path = os.path.join(score_dir, "pointcloud_map.pcd")

        if not os.path.exists(self.pcd_path):
            print("Error: {0} does not exist! Abort!".format(self.pcd_path))
            exit()

        self.yaml_path = os.path.join(score_dir, "pointcloud_map_metadata.yaml")

        if not os.path.exists(self.yaml_path):
            print("Error: A map metadata file is not found at {0}! Abort!".format(self.yaml_path))
            exit()

        self.tp_path = os.path.join(score_dir, "scores.csv")

        if not os.path.exists(self.tp_path):
            print(
                "Error: A TP file, which contains the TPs of map segments, is not found at {0}! Abort!".format(
                    self.tp_path
                )
            )
            exit()

    # Read the input map directory and setup the segment dictionary
    def __get_pcd_segments_and_scores(self):
        # Read the metadata file and get the list of segments
        print("Loading the PCD maps...")
        self.segment_df = []

        with open(self.yaml_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    self.segment_df.append([key, 0, 0])
                    seg_key = str(value[0]) + "_" + str(value[1])
                    self.segment_dict[seg_key] = len(self.segment_df) - 1
                elif key == "x_resolution":
                    self.resolution = value

        self.segment_df = np.array(self.segment_df, dtype=object)

        # Load the TPs
        with open(self.tp_path, "r") as f:
            reader = csv.reader(f)

            # Skip the header
            next(reader)
            # Load the maps' TPs
            for index, row in enumerate(reader):
                self.segment_df[index, 1] = float(row[1])

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

        for i in range(self.segment_df.shape[0]):
            progress_bar.update(1)
            # Load the current segment
            pcd = o3d.io.read_point_cloud(os.path.join(self.pcd_path, self.segment_df[i, 0]))
            np_pcd = np.asarray(pcd.points)
            rgba = self.__set_color_based_on_mark(self.segment_df[i, 2])

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
        # The may-have-changed segments are colored red
        # The may-not-have-changed segments are colored white
        if mark >= 100:
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

    def processing(
        self, score_path: str, rosbag_path: str, pose_topic: str, tp_topic: str, scan_topic: str
    ):
        self.__initialize(score_path)
        self.__get_pcd_segments_and_scores()

        tpu.collect_rosbag_tp(
            rosbag_path,
            pose_topic,
            tp_topic,
            scan_topic,
            self.resolution,
            mark_changes,
            self.segment_dict,
            self.segment_df,
        )

        self.__show()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("score_path", help="The path to the folder containing the TP file")
    parser.add_argument("bag_path", help="The path to the input rosbag")
    parser.add_argument(
        "--pose_topic",
        help="Pose topic",
        default="/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--tp_topic",
        help="TP topic",
        default="/localization/pose_estimator/transform_probability",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--scan_topic",
        help="Point cloud topic",
        default="/localization/util/downsample/pointcloud",
        required=False,
        type=str,
    )

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at {0}".format(args.score_path))
    print("Input rosbag at {0}".format(args.bag_path))
    print("Topic of NDT poses {0}".format(args.pose_topic))
    print("Topic of Transformation Probability {0}".format(args.tp_topic))
    print("Topic of scan data {0}".format(args.scan_topic))

    # Run
    checker = TPChecker()

    checker.processing(
        args.score_path, args.bag_path, args.pose_topic, args.tp_topic, args.scan_topic
    )
