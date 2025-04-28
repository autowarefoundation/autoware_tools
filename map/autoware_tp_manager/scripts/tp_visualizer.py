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
import os
import struct
import time

from geometry_msgs.msg import Pose
import numpy as np
import open3d as o3d
import pandas as pd
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg as std_msgs
import tqdm
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml


class TPVisualizer(Node):
    def __init__(self):
        super().__init__("tp_visualizer")
        self.pcd_path = None
        self.yaml_path = None
        self.score_path = None
        self.trajectory_path = None
        # Color based on rounded TP
        self.color = {
            0: [255, 255, 255],  # White 0 <= tp < 1
            1: [255, 0, 0],  # Red 1 <= tp < 2
            2: [0, 255, 0],  # Green 2 <= tp < 3
            3: [0, 0, 255],  # Blue 3 <= tp < 4
            4: [255, 255, 0],  # Yellow 4 <= tp < 5
            5: [0, 255, 255],  # Cyan 5 <= tp < 6
            6: [255, 0, 255],  # Magenta 6 <= tp
            7: [255, 0, 255],
            8: [255, 0, 255],
        }

    # Read the YAML file to get the list of PCD segments and scores
    def __get_pcd_segments_and_scores(self, result_path: str):
        if not os.path.exists(result_path):
            print("Error: the result folder does not exist at {0}! Abort!".format(result_path))
            exit()

        map_path_file = os.path.join(result_path, "map_path.txt")

        if not os.path.exists(map_path_file):
            print("Error: the file containing the path to the PCD folder does not exist at {0}! Abort!".format(map_path_file))
            exit()

        with open(map_path_file, "r") as f:
            self.pcd_path = f.read()

        if not os.path.exists(self.pcd_path):
            print("Warning: no PCD file was found at {0}! Abort".format(self.pcd_path))
            exit()

        # Look for the PCD map files
        self.map_list = []
        for fname in os.listdir(self.pcd_path):
            full_name = os.path.join(self.pcd_path, fname)
            
            if os.path.isfile(full_name):
                name, ext = os.path.splitext(fname)

                if ext == ".PCD" or ext == ".pcd":
                    self.map_list.append(full_name)

        self.yaml_path = os.path.join(result_path, "pointcloud_map_metadata.yaml")

        if not os.path.exists(self.yaml_path):
            print("Error: no PCD metadata file was found at {0}".format(self.yaml_path))
            exit()
        else:
            with open(self.yaml_path, "r") as f:
                for key, value in yaml.safe_load(f).items():
                    if key == "x_resolution":
                        self.map_resolution = value
                        break

        self.score_path = os.path.join(result_path, "scores.csv")

        if not os.path.exists(self.score_path):
            print("Error: a score file does not exist at {0}".format(self.score_path))
            exit()

        self.segment_df = pd.read_csv(self.score_path).set_index("segment")["tp"].to_dict()
        self.trajectory_path = os.path.join(result_path, "trajectory.csv")
        self.trajectory = pd.read_csv(self.trajectory_path)

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

        print("Reading map data...")

        progress_bar = tqdm.tqdm(total=len(self.map_list))
        origin = None

        # for seg in self.segment_df.itertuples():
        for seg_path in self.map_list:
            progress_bar.update(1)
            # Load the current segment
            # seg_path = self.pcd_path + "/" + seg.segment
            pcd = o3d.io.read_point_cloud(seg_path)
            np_pcd = np.asarray(pcd.points)

            for p in np_pcd:
                if origin is None:
                    origin = [p[0], p[1], p[2]]

                # Find the TP segments that contain the point @p
                sx = int(p[0] / self.map_resolution) * int(self.map_resolution)
                sy = int(p[1] / self.map_resolution) * int(self.map_resolution)

                # Create a key to look for the segment
                key = str(sx) + "_" + str(sy)
                rgba = self.__set_color_based_on_score(self.segment_df[key])

                pt = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2], rgba]
                points.append(pt)
                pc2_width += 1

        progress_bar.close()

        print("Reading trajectory...")

        progress_bar = tqdm.tqdm(total=len(self.trajectory))

        # Publish poses
        self.pose_pub = self.create_publisher(MarkerArray, "/trajectory", 10)
        marker_array_msg = MarkerArray()

        # Delete the current markers on rviz2
        dummy_marker = Marker()
        dummy_marker.header.stamp = self.get_clock().now().to_msg()
        dummy_marker.header.frame_id = "map"
        dummy_marker.id = 0
        dummy_marker.type = Marker.SPHERE
        dummy_marker.action = Marker.DELETEALL

        marker_array_msg.markers.append(dummy_marker)

        self.pose_pub.publish(marker_array_msg)

        marker_array_msg = MarkerArray()

        for i, pose in self.trajectory.iterrows():
            progress_bar.update(1)

            marker = Marker()

            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.MODIFY
            marker.pose = Pose()
            # Shift toward origin
            marker.pose.position.x = pose[0] - origin[0]
            marker.pose.position.y = pose[1] - origin[1]
            marker.pose.position.z = pose[2] - origin[2]

            marker.scale.x = self.map_resolution / 2.0
            marker.scale.y = self.map_resolution / 2.0
            marker.scale.z = self.map_resolution / 2.0

            marker.color.a = 1.0

            marker.color.r = 169.0
            marker.color.g = 169.0
            marker.color.b = 169.0
            marker_array_msg.markers.append(marker)

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
            self.pose_pub.publish(marker_array_msg)
            time.sleep(1)

    def __set_color_based_on_score(self, score) -> int:
        r, g, b = self.color[int(score)]
        a = 255

        tmp_rgb = struct.pack("BBBB", b, g, r, a)
        rgba = struct.unpack("I", tmp_rgb)[0]

        return rgba

    def processing(self, result_path: str):
        # Get the segment lists and scores
        self.__get_pcd_segments_and_scores(result_path)
        # Publish to rviz
        self.__show()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("result_path", help="The path to the result folder")

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at %s" % (args.result_path))

    # Run
    tp_collector = TPVisualizer()
    tp_collector.processing(args.result_path)
