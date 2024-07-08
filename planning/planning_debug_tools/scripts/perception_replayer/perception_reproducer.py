#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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
from collections import deque
import pickle

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import StopWatch
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        self.rosbag_ego_odom_search_radius = (
            args.search_radius
        )  # (m) the range of the ego odom to search,
        self.ego_odom_search_radius = (
            self.rosbag_ego_odom_search_radius
        )  # it may be set by an individual parameter.

        self.speed_threshold = (
            args.speed_threshold
        )  # (m/s) the speed threshold to determine the ego is stopping.

        super().__init__(args, "perception_reproducer")

        self.reproduce_sequence_indices = deque()  # contains ego_odom_idx

        self.last_published_idx = 0
        self.last_published_idx_in_sequence = 0
        self.prev_ego_odom_msg = None
        self.prev_objects_msg = None
        self.prev_traffic_signals_msg = None

        self.stopwatch = StopWatch(self.args.verbose)  # for debug

        # refresh cool down for setting initial pose in psim.
        def refresh_stat():
            self.last_published_idx = 0
            self.last_published_idx_in_sequence = 0
        self.sub_init_pos = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", refresh_stat, 1
        )

        # to make some data to accelerate computation
        self.preprocess_data()

        # start main timer callback
        time_diffs = []
        prev_stamp = self.rosbag_ego_odom_data[0][0]
        for stamp, _ in self.rosbag_ego_odom_data[1:]:
            time_diff = (stamp - prev_stamp) / 1e9
            time_diffs.append(time_diff)
            prev_stamp = stamp

        average_ego_odom_interval = sum(time_diffs) / len(time_diffs)
        # slow down the publication speed.
        average_ego_odom_interval *= args.publishing_speed_factor
        self.timer = self.create_timer(average_ego_odom_interval, self.on_timer)

        # kill perception process to avoid a conflict of the perception topics
        self.timer_check_perception_process = self.create_timer(3.0, self.on_timer_kill_perception)

        print("Start timer callback")

    def preprocess_data(self):
        # closest search with numpy data is much faster than usual
        self.rosbag_ego_odom_data_numpy = np.array(
            [
                [data[1].pose.pose.position.x, data[1].pose.pose.position.y]
                for data in self.rosbag_ego_odom_data
            ]
        )

    def on_timer_kill_perception(self):
        self.kill_online_perception_node()

    def on_timer(self):
        if self.args.verbose:
            print("\n-- on_timer start --")
        self.stopwatch.tic("total on_timer")

        timestamp = self.get_clock().now()
        timestamp_msg = timestamp.to_msg()

        if self.args.detected_object:
            pointcloud_msg = create_empty_pointcloud(timestamp_msg)
            self.pointcloud_pub.publish(pointcloud_msg)

        if not self.ego_odom:
            print("No ego odom found.")
            return

        ego_pose = self.ego_odom.pose.pose
        ego_speed = np.sqrt(self.ego_odom.twist.twist.linear.x**2 +
                            self.ego_odom.twist.twist.linear.y**2)

        # Just get the closest odom's objects if the ego is moving (old reproducer).
        if ego_speed > self.speed_threshold or self.prev_ego_odom_msg is None:
            self.update_counter = 0
            ego_odom_indices = [self.find_nearest_ego_odom_index(ego_pose)]
        elif self.update_counter < 50 and len(self.reproduce_sequence_indices) != 0:
            self.update_counter += 1
            ego_odom_indices = []
        else:
            self.update_counter = 0
            # find the nearby ego odom arange the ego.
            self.stopwatch.tic("find_nearby_ego_odom_indies")
            nearby_ego_odom_indies = self.find_nearby_ego_odom_indies(
                [self.ego_odom.pose.pose], self.ego_odom_search_radius
            )
            nearby_ego_odom_indies = [
                self.rosbag_ego_odom_data[idx][1].pose.pose for idx in nearby_ego_odom_indies
            ]
            if len(nearby_ego_odom_indies) == 0:
                nearby_ego_odom_indies = [self.rosbag_ego_odom_data[self.find_nearest_ego_odom_index(ego_pose)][1].pose.pose
                                          ]

            # find a list of ego odom around the nearby_ego_odom_indies.
            ego_odom_indices = self.find_nearby_ego_odom_indies(
                nearby_ego_odom_indies, self.rosbag_ego_odom_search_radius
            )
            self.stopwatch.toc("find_nearby_ego_odom_indies")

            # speed filter for the ego_odom_indices
            def get_speed(idx):
                log_ego_odom = self.rosbag_ego_odom_data[idx][1]
                return np.sqrt(log_ego_odom.twist.twist.linear.x**2 +
                               log_ego_odom.twist.twist.linear.y**2)
            ego_odom_indices = [idx for idx in ego_odom_indices if get_speed(
                idx) < self.speed_threshold]

        # Filter reproduce_sequence with timestamp before self.last_published_idx
        self.stopwatch.tic("sourt reproduce_sequence")
        if len(ego_odom_indices) != 0:
            last_timestamp = self.rosbag_ego_odom_data[self.last_published_idx][0]
            ego_odom_indices = [
                idx for idx in ego_odom_indices if self.rosbag_ego_odom_data[idx][0] > last_timestamp
            ]
            ego_odom_indices = sorted(ego_odom_indices)
            self.reproduce_sequence_indices = deque(ego_odom_indices)
        self.stopwatch.toc("sourt reproduce_sequence")

        # Get data to publish
        repeat_trigger = len(self.reproduce_sequence_indices) == 0
        if not repeat_trigger:
            ego_odom_idx = self.reproduce_sequence_indices.popleft()
            # if self.update_counter % 5 != 0:
            #     self.last_published_idx = ego_odom_idx
            #     return
        else:
            ego_odom_idx = self.last_published_idx
            self.last_published_idx = ego_odom_idx

        # extract messages by the nearest ego odom timestamp
        pose_timestamp, ego_odom_msg = self.rosbag_ego_odom_data[ego_odom_idx]

        self.stopwatch.tic("find_topics_by_timestamp")
        objects_msg, traffic_signals_msg = self.find_topics_by_timestamp(pose_timestamp)
        self.stopwatch.toc("find_topics_by_timestamp")
        if self.args.verbose:
            print("reproduce_sequence_indices: ", list(self.reproduce_sequence_indices)[:20])

        # Transform and publish current data.
        self.stopwatch.tic("transform and publish")

        # ego odom
        self.recorded_ego_pub.publish(ego_odom_msg)

        # objects
        if objects_msg:
            objects_msg.header.stamp = timestamp_msg
            if self.args.detected_object:
                # copy the messages
                self.stopwatch.tic("message deepcopy")
                objects_msg_copied = pickle.loads(
                    pickle.dumps(objects_msg)
                )  # this is x5 faster than deepcopy
                self.stopwatch.toc("message deepcopy")

                log_ego_pose = ego_odom_msg.pose.pose
                translate_objects_coordinate(ego_pose, log_ego_pose, objects_msg_copied)
            else:
                objects_msg_copied = objects_msg
            self.objects_pub.publish(objects_msg_copied)

        # traffic signals
        if traffic_signals_msg:
            traffic_signals_msg.stamp = timestamp_msg
            self.traffic_signals_pub.publish(traffic_signals_msg)
            self.prev_traffic_signals_msg = traffic_signals_msg
        elif self.prev_traffic_signals_msg:
            self.prev_traffic_signals_msg.stamp = timestamp_msg

            self.traffic_signals_pub.publish(self.prev_traffic_signals_msg)
        self.stopwatch.toc("transform and publish")

        self.stopwatch.toc("total on_timer")

    def find_nearest_ego_odom_index(self, ego_pose):
        # nearest search with numpy format is much (~ x100) faster than regular for loop
        self_pose = np.array([ego_pose.position.x, ego_pose.position.y])
        dists_squared = np.sum((self.rosbag_ego_odom_data_numpy - self_pose) ** 2, axis=1)
        nearest_idx = np.argmin(dists_squared)
        return nearest_idx

    def find_nearby_ego_odom_indies(self, ego_poses: list, search_radius: float):
        ego_poses_np = np.array([[pose.position.x, pose.position.y] for pose in ego_poses])
        dists_squared = np.sum(
            (self.rosbag_ego_odom_data_numpy[:, None] - ego_poses_np) ** 2, axis=2
        )
        nearby_indices = np.where(np.any(dists_squared <= search_radius**2, axis=1))[0]

        return nearby_indices


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag", help="rosbag", default=None)
    parser.add_argument(
        "-d", "--detected-object", help="publish detected object", action="store_true"
    )
    parser.add_argument(
        "-t", "--tracked-object", help="publish tracked object", action="store_true"
    )
    parser.add_argument(
        "-f", "--rosbag-format", help="rosbag data format (default is db3)", default="db3"
    )
    parser.add_argument(
        "-v", "--verbose", help="output debug data", action="store_true", default=False
    )
    parser.add_argument(
        "-r",
        "--search-radius",
        help="the search radius for searching rosbag's ego odom messages around the nearest ego odom pose (default is 1.5 meters), if the search radius is set to 0, it will always publish the closest message, just as the old reproducer did.",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--publishing-speed-factor",
        type=float,
        default=1.2,
        help="A factor to slow down the publication speed.",
    )
    parser.add_argument(
        "-s",
        "--speed-threshold",
        type=float,
        default=2.,
        help="The speed threshold to determine whether to publish in time sequence."
    )

    args = parser.parse_args()

    rclpy.init()
    node = PerceptionReproducer(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
