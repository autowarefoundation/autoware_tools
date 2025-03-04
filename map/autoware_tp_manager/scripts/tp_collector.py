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
from subprocess import call

import numpy as np
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import tp_utility as tpu
import tqdm
import yaml


class TPCollector(Node):
    def __init__(self):
        super().__init__("tp_collector")
        self.pcd_path = None
        self.yaml_path = None
        self.score_path = None
        self.segment_df = None
        self.segment_dict = {}  # Pairs of 2D coordinate and index
        self.resolution = None

    def __initialize(self, pcd_map_dir: str, output_path: str, resolution: float):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        else:
            print("The output directory {0} already existed. Re-create? [y/n]".format(output_path))
            if input() == "y":
                shutil.rmtree(output_path)
                os.makedirs(output_path)

        self.output_path = output_path
        self.resolution = resolution

        print("Type resolution = {0}".format(type(self.resolution)))

        if not os.path.exists(pcd_map_dir):
            print("Error: {0} does not exist!".format(pcd_map_dir))
            exit()

        self.pcd_path = pcd_map_dir

        if not os.path.exists(self.pcd_path):
            print("Error: {0} does not exist!".format(self.pcd_path))
            exit()

    # Read the YAML file to get the list of PCD segments and scores
    def __get_pcd_segments_and_scores(self):
        # Downsample the input PCDs to make it lighter so it can be published to rviz
        # Create a dataframe to record the avg tp of 2D segments
        self.segment_df = []

        # Downsample the input clouds if necessary
        self.yaml_path = os.path.join(self.output_path, "pointcloud_map_metadata.yaml")

        if not os.path.exists(self.yaml_path):
            ds_cmd = (
                "ros2 launch autoware_pointcloud_divider pointcloud_divider.launch.xml "
                + "input_pcd_or_dir:="
                + self.pcd_path
                + " output_pcd_dir:="
                + self.output_path
                + " prefix:=test leaf_size:=0.5"
                + " grid_size_x:="
                + str(self.resolution)
                + " grid_size_y:="
                + str(self.resolution)
            )
            call(ds_cmd, shell=True)
            self.pcd_path = os.path.join(self.output_path, "pointcloud_map.pcd")

        # Now scan the downsample directory and get the segment list
        with open(self.yaml_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    self.segment_df.append([key, 0, 0])
                    seg_key = str(value[0]) + "_" + str(value[1])
                    self.segment_dict[seg_key] = len(self.segment_df) - 1

        self.segment_df = np.array(self.segment_df, dtype=object)

        # Load the score map
        self.score_path = os.path.join(self.output_path, "scores.csv")

        if os.path.exists(self.score_path):
            with open(self.score_path, "r") as f:
                reader = csv.reader(f)

                # Skip the header
                next(reader)
                # Load the current maps' TPs
                for index, row in enumerate(reader):
                    self.segment_df[index, [1, 2]] = [float(row[1]), 1]

    def __update_avg_tp(self):
        progress_bar = tqdm.tqdm(total=self.scan_df.shape[0])

        for i in range(self.scan_df.shape[0]):
            progress_bar.update(1)
            stamp, tmp_scan = self.scan_df[i, :]
            scan = pc2.read_points(tmp_scan, skip_nans=True)

            # Find the closest pose and tp
            pid = tpu.stamp_search(stamp, self.pose_df, self.pose_df.shape[0])
            tid = tpu.stamp_search(stamp, self.tp_df, self.tp_df.shape[0])

            if pid < 0 or tid < 0:
                continue

            closest_pose = self.pose_df[pid, 1].T

            # Skip invalid poses
            if closest_pose[3, 0] == 0 and closest_pose[3, 1] == 0 and closest_pose[3, 2] == 0:
                continue

            closest_tp = self.tp_df[tid, 1]

            # Transform the scan and find the segments that cover the transformed points
            segment_set = set()

            for p in scan:
                tp = tpu.transform_p(p, closest_pose)

                # Hash the point to find the segment containing it
                sx = int(tp[0] / self.resolution) * int(self.resolution)
                sy = int(tp[1] / self.resolution) * int(self.resolution)
                seg_idx = str(sx) + "_" + str(sy)

                segment_set.add(seg_idx)

            for key in segment_set:
                if key in self.segment_dict:
                    i = self.segment_dict[key]
                    tp, counter = self.segment_df[i, [1, 2]]
                    self.segment_df[i, [1, 2]] = [
                        tp + 1.0 / (counter + 1) * (closest_tp - tp),
                        counter + 1,
                    ]

        progress_bar.close()

    # Save the segment TPs
    def __save_tps(self):
        print("Saving TP to files")
        with open(self.score_path, "w") as f:
            f.write("segment,tp\n")
            print("Number of segments = {0}".format(self.segment_df.shape[0]))
            for i in np.arange(0, self.segment_df.shape[0], dtype=int):
                f.write("{0},{1}\n".format(self.segment_df[i, 0], self.segment_df[i, 1]))
        print("Done. Segments' TPs are saved at {0}.".format(self.score_path))

    def processing(
        self,
        pcd_map_dir: str,
        rosbag_path: str,
        output_path: str,
        resolution: float,
        pose_topic: str,
        tp_topic: str,
        scan_topic: str,
    ):
        # Initialize paths to directories
        self.__initialize(pcd_map_dir, output_path, resolution)

        # Get the segment lists and scores
        self.__get_pcd_segments_and_scores()

        # Read the rosbag and get the ndt poses and corresponding tps
        output_dict = tpu.parse_rosbag(rosbag_path, pose_topic, tp_topic, scan_topic)
        self.pose_df = output_dict["mat_pose"]
        self.tp_df = output_dict["tp"]
        self.scan_df = output_dict["scan"]

        self.__update_avg_tp()

        # Save the new TPs
        self.__save_tps()


if __name__ == "__main__":
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument("map_path", help="The path to the PCD folder", type=str)
    parser.add_argument("bag_path", help="The path to the input rosbag", type=str)
    parser.add_argument("output", help="The path to the output directory", type=str)
    parser.add_argument(
        "--resolution", help="Map segment resolution", default=20, required=False, type=float
    )
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
    print("Input PCD map at {0}".format(args.map_path))
    print("Input rosbag at {0}".format(args.bag_path))
    print("Output results at {0}".format(args.output))
    print("Segmentation resolution {0}".format(args.resolution))
    print("Topic of NDT poses {0}".format(args.pose_topic))
    print("Topic of Transformation Probability {0}".format(args.tp_topic))
    print("Topic of scan data {0}".format(args.scan_topic))

    # Run
    tp_collector = TPCollector()
    tp_collector.processing(
        args.map_path,
        args.bag_path,
        args.output,
        args.resolution,
        args.pose_topic,
        args.tp_topic,
        args.scan_topic,
    )
