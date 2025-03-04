#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc. All rights reserved.
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

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import Info
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
import sensor_msgs.msg as sensor_msgs
from tier4_debug_msgs.msg import Float32Stamped
import tqdm

# Some utility functions that is shared by both the tp collector and tp checker
# Convert a pose to a 4x4 transformation matrix
def __pose_to_mat(pose: Pose) -> np.array:
    t = pose.position  # Translation
    q = pose.orientation  # Rotation

    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    return np.asarray(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qw * qy + qx * qz), t.x],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qz), t.y],
            [2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy), t.z],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


# Point-wise transform
def transform_p(p, trans_mat_t: np.ndarray) -> np.ndarray:
    tp = np.asarray([p[0], p[1], p[2], 1.0])
    return np.matmul(tp, trans_mat_t)


# Stamp search
def stamp_search(stamp: int, ref_df: np.ndarray, search_size: int) -> int:
    left = 0
    right = search_size - 1
    res = -1

    # Find the closest stamp that goes before the stamp key
    while left <= right:
        mid = (left + right) // 2
        cur_stamp = ref_df[mid, 0]

        if cur_stamp <= stamp:
            left = mid + 1
            res = mid
        elif cur_stamp > stamp:
            right = mid - 1

    return res


def __get_message_count(rosbag_path: str):
    info = Info()
    metadata = info.read_metadata(rosbag_path, "sqlite3")

    output = {
        item.topic_metadata.name: item.message_count for item in metadata.topics_with_message_count
    }
    output["total"] = metadata.message_count

    return output


# Parse the input rosbag and extract messages from the specified topics
def parse_rosbag(bag_path: str, pose_topic: str, tp_topic: str, scan_topic: str):
    reader = SequentialReader()
    bag_storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(bag_storage_options, bag_converter_options)

    print("Reading rosbag...")

    msg_count_by_topic = __get_message_count(bag_path)
    progress_bar = tqdm.tqdm(total=msg_count_by_topic["total"])

    if not(pose_topic in msg_count_by_topic) or (msg_count_by_topic[pose_topic] == 0):
        print(
            "Error: the input rosbag contains no message from the topic {0}. Exit!".format(
                pose_topic
            )
        )
        pose_df = None
        pose_to_publish = None
    else:
        pose_df = np.ndarray((msg_count_by_topic[pose_topic], 2), dtype=object)
        pose_to_publish = np.ndarray((msg_count_by_topic[pose_topic], 2), dtype=object)

    if not(tp_topic in msg_count_by_topic) or (msg_count_by_topic[tp_topic] == 0):
        print(
            "Error: the input rosbag contains no message from the topic {0}. Exit!".format(tp_topic)
        )
        tp_df = None
    else:
        tp_df = np.ndarray((msg_count_by_topic[tp_topic], 2), dtype=object)

    if not(scan_topic in msg_count_by_topic) or (msg_count_by_topic[scan_topic] == 0):
        print(
            "The input rosbag contains no message from the topic {0}.!".format(
                scan_topic
            )
        )
        scan_df = None
    else:
        scan_df = np.ndarray((msg_count_by_topic[scan_topic], 2), dtype=object)
    
    pose_df_idx = 0
    tp_df_idx = 0
    scan_df_idx = 0

    while reader.has_next():
        progress_bar.update(1)
        (topic, data, stamp) = reader.read_next()

        if topic == pose_topic:
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            stamp = pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec
            pose_df[pose_df_idx, :] = [stamp, __pose_to_mat(pose_msg.pose.pose)]
            pose_to_publish[pose_df_idx, :] = [stamp, pose_msg.pose.pose]
            pose_df_idx += 1

        elif topic == tp_topic:
            tp_msg = deserialize_message(data, Float32Stamped)
            stamp = tp_msg.stamp.sec * 1e9 + tp_msg.stamp.nanosec
            tp_df[tp_df_idx, :] = [stamp, tp_msg.data]
            tp_df_idx += 1

        elif topic == scan_topic:
            pc_msg = deserialize_message(data, sensor_msgs.PointCloud2)
            stamp = pc_msg.header.stamp.sec * 1e9 + pc_msg.header.stamp.nanosec
            scan_df[scan_df_idx, :] = [stamp, pc_msg]
            scan_df_idx += 1

    progress_bar.close()

    return {"mat_pose": pose_df, "pose": pose_to_publish, "tp": tp_df, "scan": scan_df}
