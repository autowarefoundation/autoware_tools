import argparse
import pathlib

import geometry_msgs.msg
import matplotlib.pyplot as plt
import nav_msgs.msg
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import tf_transformations


def euler_from_quaternion(quaternion):
    return tf_transformations.euler_from_quaternion(
        [quaternion.qx, quaternion.qy, quaternion.qz, quaternion.qw]
    )


def extract_pose_data(msg, msg_type):
    if msg_type == geometry_msgs.msg.Pose:
        return msg
    elif msg_type == geometry_msgs.msg.PoseStamped:
        return msg.pose
    elif msg_type == geometry_msgs.msg.PoseWithCovarianceStamped:
        return msg.pose.pose
    elif msg_type == nav_msgs.msg.Odometry:
        return msg.pose.pose
    else:
        raise ValueError(f"Unsupported message type: {msg_type}")


def interpolate_pose(df_pose: pd.DataFrame, target_timestamp: pd.Series) -> pd.DataFrame:
    POSITIONS_KEY = ["x", "y", "z"]
    ORIENTATIONS_KEY = ["qw", "qx", "qy", "qz"]
    target_index = 0
    df_index = 0
    data_dict = {
        "x": [],
        "y": [],
        "z": [],
        "qx": [],
        "qy": [],
        "qz": [],
        "qw": [],
        "timestamp": [],
    }
    while df_index < len(df_pose) - 1 and target_index < len(target_timestamp):
        curr_time = df_pose.iloc[df_index]["timestamp"]
        next_time = df_pose.iloc[df_index + 1]["timestamp"]
        target_time = target_timestamp[target_index]

        # Find a df_index that includes target_time
        if not (curr_time <= target_time <= next_time):
            df_index += 1
            continue

        curr_weight = (next_time - target_time) / (next_time - curr_time)
        next_weight = 1.0 - curr_weight

        curr_position = df_pose.iloc[df_index][POSITIONS_KEY]
        next_position = df_pose.iloc[df_index + 1][POSITIONS_KEY]
        target_position = curr_position * curr_weight + next_position * next_weight

        curr_orientation = df_pose.iloc[df_index][ORIENTATIONS_KEY]
        next_orientation = df_pose.iloc[df_index + 1][ORIENTATIONS_KEY]
        curr_r = Rotation.from_quat(curr_orientation)
        next_r = Rotation.from_quat(next_orientation)
        slerp = Slerp([curr_time, next_time], Rotation.concatenate([curr_r, next_r]))
        target_orientation = slerp([target_time]).as_quat()[0]

        data_dict["timestamp"].append(target_timestamp[target_index])
        data_dict["x"].append(target_position[0])
        data_dict["y"].append(target_position[1])
        data_dict["z"].append(target_position[2])
        data_dict["qw"].append(target_orientation[0])
        data_dict["qx"].append(target_orientation[1])
        data_dict["qy"].append(target_orientation[2])
        data_dict["qz"].append(target_orientation[3])
        target_index += 1
    result_df = pd.DataFrame(data_dict)
    return result_df


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=pathlib.Path)
    parser.add_argument("--pose_topic1", type=str, default="/localization/pose_estimator/pose")
    parser.add_argument("--pose_topic2", type=str, default="/sensing/gnss/pose")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    pose_topic1 = args.pose_topic1
    pose_topic2 = args.pose_topic2

    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(uri=str(rosbag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    target_topics = [pose_topic1, pose_topic2]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    pose1_data = []
    pose2_data = []

    while reader.has_next():
        (topic_name, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_type)
        timestamp_header = int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9
        if topic_name == pose_topic1:
            pose = extract_pose_data(msg, msg_type)
            pose1_data.append(
                {
                    "timestamp": timestamp_header,
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "qw": pose.orientation.w,
                    "qx": pose.orientation.x,
                    "qy": pose.orientation.y,
                    "qz": pose.orientation.z,
                }
            )
        elif topic_name == pose_topic2:
            pose = extract_pose_data(msg, msg_type)
            pose2_data.append(
                {
                    "timestamp": timestamp_header,
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "qw": pose.orientation.w,
                    "qx": pose.orientation.x,
                    "qy": pose.orientation.y,
                    "qz": pose.orientation.z,
                }
            )
        else:
            assert False, f"Unknown topic: {topic_name}"

    # Processing data
    df_pose1 = pd.DataFrame(pose1_data)
    df_pose2 = pd.DataFrame(pose2_data)

    # Synchronize timestamps
    df_pose2 = interpolate_pose(df_pose2, df_pose1["timestamp"])

    assert len(df_pose1) == len(
        df_pose2
    ), f"Lengths of pose1({len(df_pose1)}) and pose2({len(df_pose1)}) are different"

    df_length = len(pose1_data)
    translation_error = []
    yaw_error = []
    distance_traveled = [0]

    for i in range(1, df_length):
        pose1_pos = df_pose1.iloc[i][["x", "y", "z"]]
        pose2_pos = df_pose2.iloc[i][["x", "y", "z"]]
        translation_error.append(np.linalg.norm(pose1_pos - pose2_pos))

        pose1_yaw = euler_from_quaternion(df_pose1.iloc[i])[2]
        pose2_yaw = euler_from_quaternion(df_pose2.iloc[i])[2]
        yaw_error.append(abs(pose1_yaw - pose2_yaw))

        prev_pose1_pos = df_pose1.iloc[i - 1][["x", "y", "z"]]
        distance_traveled.append(distance_traveled[-1] + np.linalg.norm(pose1_pos - prev_pose1_pos))

    num_subdivisions = 5
    max_distance = max(distance_traveled)
    distance_interval = max_distance / num_subdivisions

    # Categorize the distance traveled into subdivisions
    distance_categories = np.digitize(
        distance_traveled, np.arange(0, max_distance, distance_interval)
    )

    # Ensure that the arrays are of the same length
    length = min(len(distance_categories), len(translation_error), len(yaw_error))
    distance_categories = distance_categories[:length]
    translation_error = translation_error[:length]
    yaw_error = yaw_error[:length]

    # Now create the DataFrames
    df_translation_error = pd.DataFrame(
        {
            "DistanceCategory": distance_categories,
            "DistanceTraveled": distance_categories * distance_interval,
            "TranslationError": np.array(translation_error),
        }
    )

    print(df_translation_error)

    df_yaw_error = pd.DataFrame(
        {
            "DistanceCategory": distance_categories,
            "DistanceTraveled": distance_categories * distance_interval,
            "YawError": np.array(yaw_error) * 180 / np.pi,
        }
    )

    # Create a single figure for both subplots
    fig, axs = plt.subplots(1, 2, figsize=(15, 6))  # 1 row, 2 columns

    # Translation Error Box Plot
    df_translation_error.boxplot(
        column="TranslationError", by="DistanceCategory", grid=False, ax=axs[0]
    )
    axs[0].set_title("Translation Error vs Distance Traveled")
    axs[0].set_xlabel("Distance traveled [m]")
    axs[0].set_ylabel("Translation error [m]")
    axs[0].set_xticklabels([f"{(i + 1) * distance_interval:.2f}" for i in range(num_subdivisions)])

    # Yaw Error Box Plot
    df_yaw_error.boxplot(column="YawError", by="DistanceCategory", grid=False, ax=axs[1])
    axs[1].set_title("Yaw Error vs Distance Traveled")
    axs[1].set_xlabel("Distance traveled [m]")
    axs[1].set_ylabel("Yaw error [deg]")
    axs[1].set_xticklabels([f"{(i + 1) * distance_interval:.2f}" for i in range(num_subdivisions)])

    plt.suptitle("")

    # Adjust layout
    plt.tight_layout()

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent

    save_path = save_dir / "performance_box.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
