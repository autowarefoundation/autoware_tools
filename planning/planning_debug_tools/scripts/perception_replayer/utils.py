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

import math
import time
from typing import Tuple

from autoware_adapi_v1_msgs.srv import SetRoutePoints
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
from tier4_localization_msgs.srv import InitializeLocalization


def get_starting_time(uri: str):
    info = rosbag2_py.Info().read_metadata(uri, "sqlite3")
    return info.starting_time


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


def open_reader(path: str):
    storage_options, converter_options = get_rosbag_options(path)
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def calc_squared_distance(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def create_empty_pointcloud(timestamp):
    pointcloud_msg = PointCloud2()
    pointcloud_msg.header.stamp = timestamp
    pointcloud_msg.header.frame_id = "map"
    pointcloud_msg.height = 1
    pointcloud_msg.is_dense = True
    pointcloud_msg.point_step = 16
    field_name_vec = ["x", "y", "z"]
    offset_vec = [0, 4, 8]
    for field_name, offset in zip(field_name_vec, offset_vec):
        field = PointField()
        field.name = field_name
        field.offset = offset
        field.datatype = 7
        field.count = 1
        pointcloud_msg.fields.append(field)
    return pointcloud_msg


def get_yaw_from_quaternion(orientation):
    orientation_list = [
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ]
    return euler_from_quaternion(orientation_list)[2]


def get_quaternion_from_yaw(yaw):
    q = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion()
    orientation.x = q[0]
    orientation.y = q[1]
    orientation.z = q[2]
    orientation.w = q[3]
    return orientation


def translate_objects_coordinate(ego_pose, log_ego_pose, objects_msg):
    log_ego_yaw = get_yaw_from_quaternion(log_ego_pose.orientation)
    log_ego_pose_trans_mat = np.array(
        [
            [
                math.cos(log_ego_yaw),
                -math.sin(log_ego_yaw),
                log_ego_pose.position.x,
            ],
            [math.sin(log_ego_yaw), math.cos(log_ego_yaw), log_ego_pose.position.y],
            [0.0, 0.0, 1.0],
        ]
    )

    ego_yaw = get_yaw_from_quaternion(ego_pose.orientation)
    ego_pose_trans_mat = np.array(
        [
            [math.cos(ego_yaw), -math.sin(ego_yaw), ego_pose.position.x],
            [math.sin(ego_yaw), math.cos(ego_yaw), ego_pose.position.y],
            [0.0, 0.0, 1.0],
        ]
    )

    for o in objects_msg.objects:
        log_object_pose = o.kinematics.pose_with_covariance.pose
        log_object_yaw = get_yaw_from_quaternion(log_object_pose.orientation)
        log_object_pos_vec = np.array([log_object_pose.position.x, log_object_pose.position.y, 1.0])

        # translate object pose from ego pose in log to ego pose in simulation
        object_pos_vec = np.linalg.inv(ego_pose_trans_mat).dot(
            log_ego_pose_trans_mat.dot(log_object_pos_vec.T)
        )

        object_pose = o.kinematics.pose_with_covariance.pose
        object_pose.position.x = object_pos_vec[0]
        object_pose.position.y = object_pos_vec[1]
        object_pose.orientation = get_quaternion_from_yaw(log_object_yaw + log_ego_yaw - ego_yaw)


def create_reader(bag_dir: str) -> SequentialReader:
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def is_close_pose(p0, p1, min_dist, max_dist) -> bool:
    dist = np.linalg.norm([p0.x - p1.x, p0.y - p1.y, p0.z - p1.z])
    return dist < min_dist or dist > max_dist


def get_pose_from_bag(input_path: str, interval=(0.1, 10000.0)) -> Tuple[Pose, Pose]:
    reader = create_reader(input_path)
    type_map = {
        topic_type.name: topic_type.type for topic_type in reader.get_all_topics_and_types()
    }
    pose_list = []
    is_first_pose = True
    prev_trans = None

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf":
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            for transform in msg.transforms:
                if transform.child_frame_id != "base_link":
                    continue
                trans = transform.transform.translation
                rot = transform.transform.rotation
                if is_first_pose:
                    is_first_pose = False
                    prev_trans = trans
                elif is_close_pose(prev_trans, trans, interval[0], interval[1]):
                    continue
                pose_list.append((trans, rot))
                prev_trans = trans

    if not pose_list:
        raise ValueError("No valid poses found in the bag file.")

    initial_trans, initial_rot = pose_list[0]
    goal_trans, goal_rot = pose_list[-1]

    initial_pose = Pose()
    initial_pose.position = Point(x=initial_trans.x, y=initial_trans.y, z=initial_trans.z)
    initial_pose.orientation = initial_rot

    goal_pose = Pose()
    goal_pose.position = Point(x=goal_trans.x, y=goal_trans.y, z=goal_trans.z)
    goal_pose.orientation = goal_rot

    return initial_pose, goal_pose


def pub_route(input_path: str):
    try:
        first_pose, last_pose = get_pose_from_bag(input_path)
    except Exception as e:
        print(f"Error retrieving poses from bag: {e}")
        return

    localization_client = LocalizationInitializer()
    future_init = localization_client.send_initial_pose(first_pose)
    rclpy.spin_until_future_complete(localization_client, future_init)
    if future_init.result() is not None:
        print("Successfully initialized localization.")
    else:
        print("Failed to initialize localization.")

    # temporarily add a sleep because sometimes the route is not generated correctly without it.
    # Need to consider a proper solution.
    time.sleep(2)

    route_client = RoutePointsClient()
    future_route = route_client.send_request(last_pose)
    rclpy.spin_until_future_complete(route_client, future_route)
    if future_route.result() is not None:
        print("Successfully set route points.")
    else:
        print("Failed to set route points.")


class StopWatch:
    def __init__(self, verbose):
        # A dictionary to store the starting times
        self.start_times = {}
        self.verbose = verbose

    def tic(self, name):
        """Store the current time with the given name."""
        self.start_times[name] = time.perf_counter()

    def toc(self, name):
        """Print the elapsed time since the last call to tic() with the same name."""
        if name not in self.start_times:
            print(f"No start time found for {name}!")
            return

        elapsed_time = (
            time.perf_counter() - self.start_times[name]
        ) * 1000  # Convert to milliseconds
        if self.verbose:
            print(f"Time for {name}: {elapsed_time:.2f} ms")

        # Reset the starting time for the name
        del self.start_times[name]


class LocalizationInitializer(Node):
    def __init__(self):
        super().__init__("localization_initializer")
        self.callback_group = ReentrantCallbackGroup()
        self.client = self.create_client(
            InitializeLocalization,
            "/localization/initialize",
            callback_group=self.callback_group,
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for '/localization/initialize' service...")

    def create_initial_pose(self, pose: Pose) -> PoseWithCovarianceStamped:
        pose_with_cov_stamped = PoseWithCovarianceStamped()
        pose_with_cov_stamped.header.frame_id = "map"
        pose_with_cov_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = pose
        covariance = np.identity(6) * 0.01  # Create a 6x6 identity matrix scaled by 0.01
        pose_with_cov.covariance = covariance.flatten().tolist()
        pose_with_cov_stamped.pose = pose_with_cov
        return pose_with_cov_stamped

    def send_initial_pose(self, pose: Pose):
        request = InitializeLocalization.Request()
        request.pose_with_covariance = [self.create_initial_pose(pose)]
        request.method = 1
        self.get_logger().info("Sending initial pose request...")
        return self.client.call_async(request)


class RoutePointsClient(Node):
    def __init__(self):
        super().__init__("route_points_client")
        self.client = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for '/api/routing/set_route_points' service...")

    def send_request(self, goal_pose: Pose):
        request = SetRoutePoints.Request()
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "map"
        request.goal = goal_pose
        request.waypoints = []
        self.get_logger().info("Sending route points request...")
        return self.client.call_async(request)
