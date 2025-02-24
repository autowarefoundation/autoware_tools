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


class TPVisualizer(Node):
    def __init__(self):
        super().__init__("tp_visualizer")
        self.pcd_map_dir = None
        self.pcd_path = None
        self.yaml_path = None
        self.score_path = None
        # Color based on rounded TP
        self.color = {
            0: [255, 255, 255],  # White
            1: [255, 255, 51],  # Yellow
            2: [255, 128, 0],  # Orange
            3: [204, 0, 0],  # Dark red
            4: [51, 153, 255],  # Blue
            5: [51, 153, 255],
            6: [51, 153, 255],
            7: [51, 153, 255],
            8: [51, 153, 255],
        }

    ##### Read the YAML file to get the list of PCD segments and scores #####
    def __get_pcd_segments_and_scores(self, pcd_map_dir: str):
        if not os.path.exists(pcd_map_dir):
            print("Error: the input PCD folder does not exist at {0}! Abort!".format(pcd_map_dir))
            exit()

        self.pcd_map_dir = pcd_map_dir
        self.pcd_path = os.path.join(pcd_map_dir, "pointcloud_map.pcd/")

        if not os.path.exists(self.pcd_path):
            print("Error: no PCD file was found at {0}! Abort!".format(self.pcd_path))
            exit()

        self.score_path = os.path.join(pcd_map_dir, "scores.csv")

        if not os.path.exists(self.score_path):
            print("Error: a score file does not exist at {0}".format(self.score_path))
            exit()

        self.segment_df = pd.read_csv(self.score_path)

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

        for tuple in self.segment_df.itertuples():
            progress_bar.update(1)
            # Load the current segment
            seg_path = self.pcd_path + "/" + tuple.segment
            pcd = o3d.io.read_point_cloud(seg_path)
            np_pcd = np.asarray(pcd.points)
            rgba = self.__set_color_based_on_score(tuple.tp)

            for p in np_pcd:
                if origin == None:
                    origin = [p[0], p[1], p[2]]
                pt = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2], rgba]
                points.append(pt)
                pc2_width += 1

        progress_bar.close()

        print("Publishing result...")
        header = std_msgs.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        pc2_msg = pc2.create_cloud(header, fields, points)
        pcd_publisher = self.create_publisher(
            sensor_msgs.PointCloud2, "/autoware_tp_visualizer", 10
        )

        while True:
            pcd_publisher.publish(pc2_msg)
            time.sleep(1)

    def __set_color_based_on_score(self, score) -> int:
        r, g, b = self.color[int(score)]
        a = 255

        tmp_rgb = struct.pack("BBBB", b, g, r, a)
        rgba = struct.unpack("I", tmp_rgb)[0]

        return rgba

    def processing(self, pcd_map_dir: str):
        # Get the segment lists and scores
        self.__get_pcd_segments_and_scores(pcd_map_dir)
        # Publish to rviz
        self.__show()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the result folder")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.map_path))

    # Run
    tp_collector = TPVisualizer()
    tp_collector.processing(args.map_path)
