#!/usr/bin/env python3

# Copyright 2025 TIER IV, Inc.
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
# cspell:disable

"""Add ground truth trajectory topic to a bag."""

import argparse

from autoware_planning_msgs.msg import Trajectory
from autoware_planning_msgs.msg import TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata


def quat_slerp(q1, q2, t):
    """SLERP interpolation between two quaternions."""
    # Normalize
    q1 = np.array([q1.x, q1.y, q1.z, q1.w])
    q2 = np.array([q2.x, q2.y, q2.z, q2.w])

    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot = np.dot(q1, q2)

    # If negative, reverse one quaternion
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    # If very close, use linear interpolation
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    # SLERP
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    result = (np.sin((1 - t) * theta) * q1 + np.sin(t * theta) * q2) / np.sin(theta)
    return result


def interpolate_pose(odom1, odom2, t):
    """Interpolate pose between two odometry messages."""
    pose = Pose()

    # Linear interpolation for position
    pose.position.x = odom1.pose.pose.position.x + t * (
        odom2.pose.pose.position.x - odom1.pose.pose.position.x
    )
    pose.position.y = odom1.pose.pose.position.y + t * (
        odom2.pose.pose.position.y - odom1.pose.pose.position.y
    )
    pose.position.z = odom1.pose.pose.position.z + t * (
        odom2.pose.pose.position.z - odom1.pose.pose.position.z
    )

    # SLERP for orientation
    q_interp = quat_slerp(odom1.pose.pose.orientation, odom2.pose.pose.orientation, t)
    pose.orientation.x = q_interp[0]
    pose.orientation.y = q_interp[1]
    pose.orientation.z = q_interp[2]
    pose.orientation.w = q_interp[3]

    return pose


def generate_gt_trajectory(
    all_odometry, current_index, current_time_ns, horizon_sec, resolution_sec
):
    """Generate GT trajectory from future kinematic states."""
    trajectory = Trajectory()

    # Header
    trajectory.header.frame_id = "map"
    trajectory.header.stamp.sec = int(current_time_ns // 1e9)
    trajectory.header.stamp.nanosec = int(current_time_ns % 1e9)

    # Generate points by looking ahead
    horizon_ns = int(horizon_sec * 1e9)
    resolution_ns = int(resolution_sec * 1e9)

    for dt_ns in range(0, horizon_ns + 1, resolution_ns):
        target_time_ns = current_time_ns + dt_ns

        # Find bracketing odometry messages
        idx1 = None
        idx2 = None

        for i in range(current_index, len(all_odometry)):
            odom_time_ns = all_odometry[i]["timestamp"]

            if odom_time_ns <= target_time_ns:
                idx1 = i
            elif odom_time_ns > target_time_ns:
                idx2 = i
                break

        if idx1 is None:
            break  # No data

        # Create trajectory point
        point = TrajectoryPoint()

        if idx2 is None or idx1 == idx2:
            # Use exact point or last available
            point.pose = all_odometry[idx1]["odom"].pose.pose
        else:
            # Interpolate
            time1 = all_odometry[idx1]["timestamp"]
            time2 = all_odometry[idx2]["timestamp"]
            t = (target_time_ns - time1) / (time2 - time1)
            point.pose = interpolate_pose(all_odometry[idx1]["odom"], all_odometry[idx2]["odom"], t)

        # Time from start
        point.time_from_start = Duration()
        point.time_from_start.sec = int(dt_ns // 1e9)
        point.time_from_start.nanosec = int(dt_ns % 1e9)

        trajectory.points.append(point)

    return trajectory if len(trajectory.points) > 0 else None


def add_gt_trajectories(input_bag, output_bag, horizon_sec=8.0, resolution_sec=0.1):
    """Add GT trajectory topic to bag."""
    print("Adding GT trajectories to bag:")
    print(f"  Input: {input_bag}")
    print(f"  Output: {output_bag}")
    print(f"  Horizon: {horizon_sec}s")
    print(f"  Resolution: {resolution_sec}s")
    print("")

    # Step 1: Load all kinematic states
    # Use empty storage_id to let rosbag2 auto-detect format
    print("Step 1: Loading all kinematic states...")
    storage_options_read = StorageOptions(uri=str(input_bag), storage_id="")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader1 = SequentialReader()
    reader1.open(storage_options_read, converter_options)

    all_odometry = []
    while reader1.has_next():
        topic, data, timestamp = reader1.read_next()
        if topic == "/localization/kinematic_state":
            odom = deserialize_message(data, Odometry)
            all_odometry.append({"timestamp": timestamp, "odom": odom})

    del reader1
    print(f"  Loaded {len(all_odometry)} kinematic states")

    # Step 2: Process bag and add GT trajectories
    print("\nStep 2: Processing bag and generating GT trajectories...")

    reader2 = SequentialReader()
    reader2.open(storage_options_read, converter_options)

    storage_options_write = StorageOptions(uri=str(output_bag), storage_id="mcap")
    writer = SequentialWriter()
    writer.open(storage_options_write, converter_options)

    # Create all existing topics
    for topic_metadata in reader2.get_all_topics_and_types():
        writer.create_topic(topic_metadata)

    # Add GT trajectory topic
    gt_topic = TopicMetadata(
        name="/ground_truth/trajectory",
        type="autoware_planning_msgs/msg/Trajectory",
        serialization_format="cdr",
    )
    writer.create_topic(gt_topic)
    print("Created topic: /ground_truth/trajectory")

    # Process messages
    message_count = 0
    gt_count = 0
    odom_index = 0

    while reader2.has_next():
        topic, data, timestamp = reader2.read_next()

        # Write original message
        writer.write(topic, data, timestamp)
        message_count += 1

        # Generate GT trajectory for each kinematic_state
        if topic == "/localization/kinematic_state":
            gt_traj = generate_gt_trajectory(
                all_odometry, odom_index, timestamp, horizon_sec, resolution_sec
            )

            if gt_traj:
                gt_data = serialize_message(gt_traj)
                writer.write("/ground_truth/trajectory", gt_data, timestamp)
                gt_count += 1

            odom_index += 1

        if message_count % 10000 == 0:
            print(f"  Processed {message_count} messages, generated {gt_count} GT trajectories...")

    print("\nCopy complete:")
    print(f"  Total messages copied: {message_count}")
    print(f"  GT trajectories generated: {gt_count}")

    del writer
    del reader2

    # Reindex
    print("\nReindexing...")
    Reindexer().reindex(storage_options_write)
    print("Done!")


def main():
    parser = argparse.ArgumentParser(description="Add ground truth trajectory topic to a rosbag")
    parser.add_argument("--input", required=True, help="Input bag directory or mcap file")
    parser.add_argument("--output", required=True, help="Output bag directory")
    parser.add_argument(
        "--horizon", type=float, default=8.0, help="Look-ahead horizon in seconds (default: 8.0)"
    )
    parser.add_argument(
        "--resolution", type=float, default=0.1, help="Sample resolution in seconds (default: 0.1)"
    )

    args = parser.parse_args()

    rclpy.init()

    add_gt_trajectories(args.input, args.output, args.horizon, args.resolution)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
