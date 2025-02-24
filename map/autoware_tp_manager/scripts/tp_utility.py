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
import sensor_msgs_py.point_cloud2 as pc2
from tier4_debug_msgs.msg import Float32Stamped
import tqdm


class FixQueue:
    def __init__(self):
        self.queue_limit = 100  # Max number of items in the queue
        self.data_queue = None  # Queue of data
        self.head = 0  # Index to append a new item (enqueue index)
        self.tail = 0  # Index to start remove/process items (dequeue index)
        self.current_size = 0  # Number of items in the queue

    def enqueue(self, item):
        if self.is_full():
            print("Cannot add new item. Queue is full!")
            return False

        if self.is_empty() or self.data_queue.shape[1] != len(item):
            self.data_queue = np.ndarray((self.queue_limit, len(item)), dtype=object)
        self.data_queue[self.tail, :] = item
        self.tail += 1

        if self.tail == self.queue_limit:
            self.tail = 0

        self.current_size += 1

        return True

    def drop(self, limitless=False, callback_func=None, *args):
        if limitless == True:
            end_id = self.tail
        else:
            end_id = self.head + int(self.queue_limit / 2)

        if self.head < end_id:
            drop_list = np.arange(self.head, end_id)
        else:
            drop_list = np.arange(self.head, end_id + self.queue_limit) % self.queue_limit

        for i in drop_list:
            # Process the item at index i
            callback_func(self.data_queue[i], *args)

        self.current_size -= len(drop_list)
        self.head = end_id

        if self.head == self.queue_limit:
            self.head = 0

    def is_full(self) -> bool:
        if self.current_size == self.queue_limit:
            return True
        return False

    def is_empty(self) -> bool:
        if self.current_size == 0:
            return True
        return False


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


##### Stamp search #####
def __stamp_search(stamp: int, ref_df: np.ndarray, search_size: int) -> int:
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


def __scan_callback(
    scan_tuple, pose_df, pose_df_len, tp_df, tp_df_len, resolution, callback_func=None, *args
):
    stamp, scan = scan_tuple
    tmp_scan = pc2.read_points(scan, skip_nans=True)

    # Find the closest pose
    pid = __stamp_search(stamp, pose_df, pose_df_len)

    if pid <= 0:
        return

    # Get the transposed transformation matrix
    closest_p_t = pose_df[pid, 1].T

    # Skip some 0 poses at the beginning of the rosbag
    if closest_p_t[3, 0] == 0 and closest_p_t[3, 1] == 0 and closest_p_t[3, 2] == 0:
        return

    # Find the closest tp
    tid = __stamp_search(stamp, tp_df, tp_df_len)

    if tid <= 0:
        return

    closest_tp = tp_df[tid, 1]

    # Transform the scan and find the segments that cover the transformed points
    # Record the segments that cover the scan
    segment_set = set()

    for p in tmp_scan:
        tp = transform_p(p, closest_p_t)

        # Hash the point to find the segment containing it
        sx = int(tp[0] / resolution) * int(resolution)
        sy = int(tp[1] / resolution) * int(resolution)
        seg_idx = str(sx) + "_" + str(sy)

        segment_set.add(seg_idx)

    # Update/examine the segments' avg TP
    callback_func(segment_set, closest_tp, *args)


##### Read the input rosbag and update map's TP #####
def collect_rosbag_tp(bag_path: str, pose_topic: str, tp_topic: str, scan_topic: str, *args):
    reader = SequentialReader()
    bag_storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(bag_storage_options, bag_converter_options)

    print("Reading rosbag...")

    msg_count_by_topic = __get_message_count(bag_path)
    progress_bar = tqdm.tqdm(total=msg_count_by_topic["total"])

    if msg_count_by_topic[pose_topic] == 0:
        print(
            "Error: the input rosbag contains no message from the topic {0}. Exit!".format(
                pose_topic
            )
        )
    if msg_count_by_topic[tp_topic] == 0:
        print(
            "Error: the input rosbag contains no message from the topic {0}. Exit!".format(tp_topic)
        )
    if msg_count_by_topic[scan_topic] == 0:
        print(
            "Error: the input rosbag contains no message from the topic {0}. Exit!".format(
                scan_topic
            )
        )

    pose_df = np.ndarray((msg_count_by_topic[pose_topic], 2), dtype=object)
    tp_df = np.ndarray((msg_count_by_topic[tp_topic], 2), dtype=object)
    pose_df_idx = 0
    tp_df_idx = 0
    scan_df = FixQueue()

    skip_count = 0

    while reader.has_next():
        progress_bar.update(1)
        (topic, data, stamp) = reader.read_next()

        if skip_count <= 2000:
            skip_count += 1
            continue

        if topic == pose_topic:
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            stamp = pose_msg.header.stamp.sec * 1e9 + pose_msg.header.stamp.nanosec
            pose_df[pose_df_idx, :] = [stamp, __pose_to_mat(pose_msg.pose.pose)]
            pose_df_idx += 1

        elif topic == tp_topic:
            tp_msg = deserialize_message(data, Float32Stamped)
            stamp = tp_msg.stamp.sec * 1e9 + tp_msg.stamp.nanosec
            tp_df[tp_df_idx, :] = [stamp, tp_msg.data]
            tp_df_idx += 1

        elif topic == scan_topic:
            pc_msg = deserialize_message(data, sensor_msgs.PointCloud2)
            stamp = pc_msg.header.stamp.sec * 1e9 + pc_msg.header.stamp.nanosec

            scan_df.enqueue([stamp, pc_msg])

            if scan_df.is_full():
                scan_df.drop(False, __scan_callback, pose_df, pose_df_idx, tp_df, tp_df_idx, *args)

    # Process the rest of the queue
    scan_df.drop(True, __scan_callback, pose_df, pose_df_idx, tp_df, tp_df_idx, *args)

    progress_bar.close()
