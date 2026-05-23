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

# Add the path to the compiled pybind11 module (.so)
import sys

import numpy as np
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
import tp_utility as tpu
import tqdm
import yaml

script_dir = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.abspath(script_dir))

print(f"Current path {os.path.abspath(script_dir)}")


class TPCollector(Node):
    def __init__(self):
        super().__init__("tp_collector")
        self.pcd_path = None
        self.yaml_path = None
        self.score_path = None
        self.map_path_file = None
        self.segment_df = None
        self.segment_dict = {}  # Pairs of 2D coordinate and index
        self.resolution = None
        self.output_pose = []
        self.trajectory_path = None

    def __initialize(self, pcd_map_dir: str, output_path: str, resolution: float):
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        else:
            print("The output directory {0} already existed. Re-create? [y/n]".format(output_path))
            if input() == "y":
                shutil.rmtree(output_path)
                os.makedirs(output_path)

        self.output_path = output_path
        self.map_path_file = os.path.join(output_path, "map_path.txt")
        self.resolution = resolution

        if not os.path.exists(pcd_map_dir):
            print("Error: {0} does not exist!".format(pcd_map_dir))
            exit()

        self.pcd_path = pcd_map_dir

        if not os.path.exists(self.pcd_path):
            print("Error: {0} does not exist!".format(self.pcd_path))
            exit()

        self.trajectory_path = os.path.join(self.output_path, "trajectory.csv")

    # Read the YAML file to get the list of PCD segments and scores
    def __get_pcd_segments_and_scores(self):
        # Downsample the input PCDs to make it lighter so it can be published to rviz
        # Create a dataframe to record the avg tp of 2D segments
        self.segment_df = []

        # Downsample the input clouds if necessary
        self.yaml_path = os.path.join(self.output_path, "pointcloud_map_metadata.yaml")

        # Call the PCD divider wrapper
        import pcd_divider_wrapper

        if not os.path.exists(self.yaml_path):
            pcd_divider_wrapper.init_ros()

            pcd_divider = pcd_divider_wrapper.PCDDivider("tp_collector_log")
            pcd_divider.setInput(self.pcd_path)
            pcd_divider.setOutputDir(self.output_path)
            pcd_divider.setLeafSize(0.5)
            pcd_divider.setPrefix("test")

            if self.resolution >= 5.0:
                pcd_divider.setGridSize(self.resolution, self.resolution)
            else:
                pcd_divider.setGridSize(50.0, 50.0)

            pcd_divider.divide_pcd()

            # Generate a metadata file
            tmp_path = os.path.join(self.output_path, "tmp_metadata")
            pcd_divider.setInput(self.pcd_path)
            pcd_divider.setOutputDir(tmp_path)
            pcd_divider.setGridSize(self.resolution, self.resolution)
            # The metadata file is at the tmp_path
            pcd_divider.meta_generator()

            pcd_divider_wrapper.shutdown_ros()

            # Move the metadata file back to the output_dir, overwrite the current metadata file
            shutil.move(
                os.path.join(tmp_path, "pointcloud_map_metadata.yaml"),
                os.path.join(self.output_path, "pointcloud_map_metadata.yaml"),
            )
            # Remove the tmp folder
            shutil.rmtree(tmp_path)

            self.pcd_path = os.path.join(self.output_path, "pointcloud_map.pcd")

        # Now scan the downsample directory and get the segment list
        with open(self.yaml_path, "r") as f:
            for key, value in yaml.safe_load(f).items():
                if key != "x_resolution" and key != "y_resolution":
                    seg_key = str(value[0]) + "_" + str(value[1])
                    self.segment_df.append([seg_key, 0, 0])
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

    def __find_candidate_segments(self, stamp, t_pose):
        if self.scan_df.shape[0] > 0:
            sid = tpu.stamp_search(stamp, self.scan_df, self.scan_df.shape[0])

            if sid < 0:
                return None

            closest_scan = pc2.read_points(self.scan_df[sid, 1], skip_nans=True)

            segment_set = set()

            for p in closest_scan:
                tp = tpu.transform_p(p, t_pose)

                # Hash the point to find the segment containing it
                sx = int(tp[0] / self.resolution) * int(self.resolution)
                sy = int(tp[1] / self.resolution) * int(self.resolution)
                seg_idx = str(sx) + "_" + str(sy)

                segment_set.add(seg_idx)

            return segment_set
        else:
            # If scans are not available, use range query
            lx = int((t_pose[3, 0] - 50.0) / self.resolution) * int(self.resolution)
            ux = int((t_pose[3, 0] + 50.0) / self.resolution) * int(self.resolution)
            ly = int((t_pose[3, 1] - 50.0) / self.resolution) * int(self.resolution)
            uy = int((t_pose[3, 1] + 50.0) / self.resolution) * int(self.resolution)

            for sx in range(start=lx, stop=ux, step=1):
                for sy in range(start=ly, stop=uy, step=1):
                    segment_set.add(str(sx) + "_" + str(sy))

            return segment_set

    def __update_avg_tp(self):
        progress_bar = tqdm.tqdm(total=self.pose_df.shape[0])

        for i in range(self.pose_df.shape[0]):
            progress_bar.update(1)
            stamp, pose = self.pose_df[i, :]
            pose = pose.T

            if pose is None:
                continue

            # Skip invalid poses
            if pose[3, 0] == 0 and pose[3, 1] == 0 and pose[3, 2] == 0:
                continue

            # Find the closest tp
            tid = tpu.stamp_search(stamp, self.tp_df, self.tp_df.shape[0])

            if tid < 0:
                continue

            closest_tp = self.tp_df[tid, 1]

            # Find the segments that cover the transformed points
            segment_set = self.__find_candidate_segments(stamp, pose)

            for key in segment_set:
                if key in self.segment_dict:
                    i = self.segment_dict[key]
                    tp, counter = self.segment_df[i, [1, 2]]
                    self.segment_df[i, [1, 2]] = [
                        tp + 1.0 / (counter + 1) * (closest_tp - tp),
                        counter + 1,
                    ]

            self.output_pose.append(pose)

        progress_bar.close()

    # Save the segment TPs
    def __save_tps(self):
        print("Saving TP to files")
        with open(self.score_path, "w") as f:
            f.write("segment,tp\n")
            print("Number of segments = {0}".format(self.segment_df.shape[0]))
            for i in range(self.segment_df.shape[0]):
                f.write("{0},{1}\n".format(self.segment_df[i, 0], self.segment_df[i, 1]))
        print("Done. Segments' TPs are saved at {0}.".format(self.score_path))

        print("Saving trajectory")
        with open(self.trajectory_path, "w") as f:
            f.write("x,y,z\n")
            print("Number of poses = {0}".format(len(self.output_pose)))
            for i in range(len(self.output_pose)):
                p = self.output_pose[i]
                f.write("{0},{1},{2}\n".format(p[3, 0], p[3, 1], p[3, 2]))
        print("Done. Poses are saved at {0}".format(self.trajectory_path))

        print("Saving a path to the map")
        with open(self.map_path_file, "w") as f:
            f.write(self.pcd_path)
        print("Done. The map path is saved at {0}".format(self.map_path_file))

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
        "--resolution", help="Map segment resolution", default=20.0, required=False, type=float
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
