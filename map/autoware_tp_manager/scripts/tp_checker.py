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
import sensor_msgs.msg as sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg as std_msgs
import tp_utility as tpu
import tqdm
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml


class TPChecker(Node):

    def __init__(self):
        super().__init__("tp_checker")
        self.segment_df = None  # Segment indices, TP
        self.segment_dict = {}  # key: segment name, value: index to the segment_df
        self.pcd_path = None  # Path to the directory containing PCD segments
        self.changed_dir = None  # A directory contains the segments that need to be examined
        self.result_csv = None  # Path to the result CSV file
        self.tp_path = None  # Path to the file that contains TPs of map segments
        self.query_range = 50.0

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

        self.output_path = os.path.join(score_dir, "checking_result.csv")

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
        # Publish map

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
                if origin is None:
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

        # Publish poses
        self.pose_pub = self.create_publisher(MarkerArray, "/pose_result", 10)
        marker_array_msg = MarkerArray()

        for i in range(self.pose_to_publish.shape[0]):
            pose = self.pose_to_publish[i, 1]

            if pose is None:
                continue
            marker = Marker()

            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            # Shift toward origin
            marker.pose.position.x -= origin[0]
            marker.pose.position.y -= origin[1]
            marker.pose.position.z -= origin[2]

            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 2.0

            marker.color.a = 1.0

            if self.mark[i] == 1:
                marker.color.r = 255.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 255.0

            marker_array_msg.markers.append(marker)

        while True:
            pcd_publisher.publish(pc2_msg)
            self.pose_pub.publish(marker_array_msg)

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

    def __find_candidate_segments(self, stamp, t_pose):
        if not (self.scan_df is None) and self.scan_df.shape[0] > 0:
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
            segment_set = set()
            # If scans are not available, use range query
            lx = int((t_pose[3, 0] - self.query_range) / self.resolution) * int(self.resolution)
            ux = int((t_pose[3, 0] + self.query_range) / self.resolution) * int(self.resolution)
            ly = int((t_pose[3, 1] - self.query_range) / self.resolution) * int(self.resolution)
            uy = int((t_pose[3, 1] + self.query_range) / self.resolution) * int(self.resolution)

            for sx in range(lx, ux + 1, 1):
                for sy in range(ly, uy + 1, 1):
                    segment_set.add(str(sx) + "_" + str(sy))

            return segment_set

    def __mark_changed_poses(self):
        progress_bar = tqdm.tqdm(total=self.pose_df.shape[0])

        for i in range(self.pose_df.shape[0]):
            progress_bar.update(1)
            stamp, pose = self.pose_df[i, :]
            # Transposed pose for multiplication, as the points are in a row-major table
            pose = pose.T
            # Skip invalid poses
            # print("Pose at {0} = {1}".format(i, pose))
            if pose is None:
                # print("None pose. Skip!")
                continue

            if pose[3, 0] == 0 and pose[3, 1] == 0 and pose[3, 2] == 0:
                # print("Invalid pose. Skip!")
                continue

            # Find the closest tp
            tid = tpu.stamp_search(stamp, self.tp_df, self.tp_df.shape[0])

            if tid < 0:
                print("No closest tp were found. Skip!")
                continue
            closest_tp = self.tp_df[tid, 1]

            # Find the candidate segments that cover the pose
            segment_set = self.__find_candidate_segments(stamp, pose)

            if segment_set is None:
                continue

            tp_sum = 0.0
            valid_segment_num = 0

            for key in segment_set:
                if key in self.segment_dict:
                    tp_sum += self.segment_df[self.segment_dict[key], 1]
                    valid_segment_num += 1
            if valid_segment_num > 0:
                expected_tp = tp_sum / float(valid_segment_num)
            else:
                expected_tp = 0

            # If the expected tp and the actual tp is quite different, mark the pose as changed
            if (expected_tp > 0 and abs(expected_tp - closest_tp) / expected_tp >= 0.2) or (
                expected_tp == 0 and closest_tp > 0
            ):
                self.mark[i] = 1

        progress_bar.close()

    def __mark_changed_segments(self):
        progress_bar = tqdm.tqdm(total=self.pose_df.shape[0])

        for i in range(self.scan_df.shape[0]):
            progress_bar.update(1)
            stamp, tmp_scan = self.scan_df[i, :]
            scan = pc2.read_points(tmp_scan, skip_nans=True)

            # Find the closest pose and tp
            pid = tpu.stamp_search(stamp, self.pose_df, self.pose_df.shape[0])
            tid = tpu.stamp_search(stamp, self.tp_df, self.tp_df.shape[0])

            if pid < 0 or tid < 0:
                continue

            closest_pose = self.pose_df[pid, 1]

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

            tp_sum = 0.0
            valid_segment_num = 0

            for key in segment_set:
                if key in self.segment_dict:
                    tp_sum += self.segment_df[self.segment_dict[key], 1]
                    valid_segment_num += 1
            if valid_segment_num > 0:
                expected_tp = tp_sum / float(valid_segment_num)
            else:
                expected_tp = 0

            self.dual_tp[i, :] = [closest_tp, expected_tp]

            if (expected_tp > 0 and abs(expected_tp - closest_tp) / expected_tp >= 0.2) or (
                expected_tp == 0 and closest_tp > 0
            ):
                # Mark changed segments
                for key in segment_set:
                    if key in self.segment_dict:
                        self.segment_df[self.segment_dict[key], 2] += 1

        progress_bar.close()

    def __save_results(self):
        print("Saving checking results to a file {0}".format(self.output_path))

        with open(self.output_path, "w") as f:
            f.write("Actual_TP,expected_TP\n")
            print("Number of poses = {0}".format(self.pose_df.shape[0]))

            for i in range(self.pose_df.shape[0]):
                f.write("{0},{1}\n".format(self.dual_tp[i, 0], self.dual_tp[i, 1]))

        print("Done. Saved TPs at {0}".format(self.output_path))

    def processing(
        self,
        score_path: str,
        rosbag_path: str,
        pose_topic: str,
        tp_topic: str,
        scan_topic: str,
        mode: str,
        query_range: float,
    ):
        if query_range > 0:
            self.query_range = query_range
        self.__initialize(score_path)
        self.__get_pcd_segments_and_scores()

        output_dict = tpu.parse_rosbag(rosbag_path, pose_topic, tp_topic, scan_topic)
        self.pose_df = output_dict["mat_pose"]
        self.pose_to_publish = output_dict["pose"]
        self.tp_df = output_dict["tp"]
        self.scan_df = output_dict["scan"]
        self.mark = np.zeros((self.pose_to_publish.shape[0], 1))
        # Actual TP and expected TP
        self.dual_tp = np.zeros((self.pose_df.shape[0], 2))

        if mode == "check_pose":
            self.__mark_changed_poses()
            self.__save_results()
        elif mode == "check_segment":
            self.__mark_changed_segments()
        else:
            print("Unrecognized mode! Exit!")
            exit()

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
    parser.add_argument(
        "--mode",
        help="Check mode",
        default="check_pose",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--radius",
        help="The range to query segments that cover a pose",
        default=50.0,
        required=False,
        type=float,
    )

    args = parser.parse_args()

    # Practice with string % a bit
    print("Input PCD map at: {0}".format(args.score_path))
    print("Input rosbag at: {0}".format(args.bag_path))
    print("Topic of NDT poses: {0}".format(args.pose_topic))
    print("Topic of Transformation Probability: {0}".format(args.tp_topic))
    print("Topic of scan data: {0}".format(args.scan_topic))
    print("Mode of checking: {0}".format(args.mode))
    print("Range to query map segments that cover a pose: {0}".format(args.radius))

    # Run
    checker = TPChecker()

    checker.processing(
        args.score_path,
        args.bag_path,
        args.pose_topic,
        args.tp_topic,
        args.scan_topic,
        args.mode,
        args.radius,
    )
