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

import numpy as np
from perception_replayer_common import PerceptionReplayerCommon
import rclpy
from utils import StopWatch
from utils import create_empty_pointcloud
from utils import translate_objects_coordinate

dist_eps = 1e-2  # (meters)


class PerceptionReproducer(PerceptionReplayerCommon):
    def __init__(self, args):
        self.rosbag_ego_odom_search_radius = (
            args.search_radius
        )  # (m) the range of the ego odom to search,
        self.ego_odom_search_radius = (
            self.rosbag_ego_odom_search_radius
        )  # it may be set by an individual parameter.

        self.reproduce_cool_down = (
            args.reproduce_cool_down if args.search_radius != 0.0 else 0.0
        )  # (sec) the cool down time for republishing published data, please make sure that it's greater than the ego's stopping time.

        super().__init__(args, "perception_reproducer")

        self.reproduce_sequence_indices = deque()  # contains ego_odom_idx
        self.cool_down_indices = deque()  # contains ego_odom_idx
        self.ego_odom_id2last_published_timestamp = {}  # for checking last published timestamp.

        self.prev_ego_pos = None
        self.prev_ego_odom_msg = None
        self.perv_objects_msg = None
        self.prev_traffic_signals_msg = None

        self.stopwatch = StopWatch(self.args.verbose)  # for debug

        # to make some data to accelerate computation
        self.preprocess_data()

        # start main timer callback
        time_diffs = []
        prev_stamp = self.rosbag_ego_odom_data[0][0]
        for stamp, _ in self.rosbag_ego_odom_data[1:]:
            time_diff = (stamp - prev_stamp) / 1e9
            time_diffs.append(time_diff)
            prev_stamp = stamp

        average_ego_odom_interval = 0.1  # sum(time_diffs) / len(time_diffs)
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

        if not self.ego_pose:
            print("No ego pose found.")
            return

        # Update reproduce list if ego_pos is moved.
        if (
            self.ego_pose is None
            or self.prev_ego_pos is None
            or (
                (self.ego_pose.position.x - self.prev_ego_pos.position.x) ** 2
                + (self.ego_pose.position.y - self.prev_ego_pos.position.y) ** 2
            )
            > dist_eps**2
        ):
            # find the nearest ego odom by simulation observation
            self.stopwatch.tic("find_nearest_ego_odom_by_observation")
            nearby_ego_odom_indies = self.find_nearby_ego_odom_indies(
                [self.ego_pose], self.ego_odom_search_radius
            )
            nearby_ego_odom_indies = [
                self.rosbag_ego_odom_data[idx][1].pose.pose for idx in nearby_ego_odom_indies
            ]
            if not nearby_ego_odom_indies:
                nearest_ego_odom_ind = self.find_nearest_ego_odom_index(self.ego_pose)
                nearby_ego_odom_indies += [
                    self.rosbag_ego_odom_data[nearest_ego_odom_ind][1].pose.pose
                ]
            self.stopwatch.toc("find_nearest_ego_odom_by_observation")

            # find a list of ego odom around the nearest_ego_odom_pos.
            self.stopwatch.tic("find_nearby_ego_odom_indies")
            ego_odom_indices = self.find_nearby_ego_odom_indies(
                nearby_ego_odom_indies, self.rosbag_ego_odom_search_radius
            )
            self.stopwatch.toc("find_nearby_ego_odom_indies")

            # update reproduce_sequence with those data not in cool down list.
            while self.cool_down_indices:
                last_timestamp = self.ego_odom_id2last_published_timestamp[
                    self.cool_down_indices[0]
                ]
                if (timestamp - last_timestamp).nanoseconds / 1e9 > self.reproduce_cool_down:
                    self.cool_down_indices.popleft()
                else:
                    break

            self.stopwatch.tic("update reproduce_sequence")
            ego_odom_indices = [
                idx for idx in ego_odom_indices if idx not in self.cool_down_indices
            ]
            ego_odom_indices = sorted(ego_odom_indices)
            self.reproduce_sequence_indices = deque(ego_odom_indices)

            self.stopwatch.toc("update reproduce_sequence")

        self.prev_ego_pos = self.ego_pose

        # get data to publish
        repeat_trigger = len(self.reproduce_sequence_indices) == 0
        if not repeat_trigger:  # pop data from reproduce_sequence if sequence is not empty.
            ego_odom_idx = self.reproduce_sequence_indices.popleft()
            pose_timestamp, ego_odom_msg = self.rosbag_ego_odom_data[ego_odom_idx]
            # extract message by the nearest ego odom timestamp
            self.stopwatch.tic("find_topics_by_timestamp")
            objects_msg, traffic_signals_msg = self.find_topics_by_timestamp(pose_timestamp)
            self.stopwatch.toc("find_topics_by_timestamp")
            # if self.args.verbose:
            #     print("reproduce_sequence_indices: ", list(self.reproduce_sequence_indices)[:20])

        else:  # get perv data to publish if reproduce_sequence is empty.
            ego_odom_msg = self.prev_ego_odom_msg
            objects_msg = self.perv_objects_msg
            traffic_signals_msg = self.prev_traffic_signals_msg

        # transform and publish current data.
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
                translate_objects_coordinate(self.ego_pose, log_ego_pose, objects_msg_copied)
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

        if not repeat_trigger:
            # save data for repeat publication.
            self.prev_ego_odom_msg = ego_odom_msg
            self.perv_objects_msg = (
                objects_msg if objects_msg is not None else self.perv_objects_msg
            )
            self.prev_traffic_signals_msg = (
                traffic_signals_msg
                if traffic_signals_msg is not None
                else self.prev_traffic_signals_msg
            )

            # update cool down info.
            self.ego_odom_id2last_published_timestamp[ego_odom_idx] = timestamp
            self.cool_down_indices.append(ego_odom_idx)

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
        help="the search radius for searching rosbag's ego odom messages around the nearest ego odom pose (default is 2 meters), if the search radius is set to 0, it will always publish the closest message, just as the old reproducer did.",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "-c",
        "--reproduce-cool-down",
        help="The cool down time for republishing published messages (default is 15.0 seconds)",
        type=float,
        default=15.0,
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
