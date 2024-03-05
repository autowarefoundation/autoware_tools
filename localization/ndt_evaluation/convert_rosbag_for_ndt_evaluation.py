"""Convert rosbag for NDT evaluation.

(1) Check if the rosbag is suitable for ndt evaluation
(2) Filter topics
(3) Rename the topic name of the reference kinematic state to /localization/reference_kinematic_state
"""

import argparse
import pathlib

import pandas as pd
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseWithCovarianceStamped

REQUIRED_FPS_PATTERN_A = {
    "/localization/kinematic_state": 40,
    "/localization/util/downsample/pointcloud": 7.5,
    "/sensing/vehicle_velocity_converter/twist_with_covariance": 20,
    "/sensing/imu/imu_data": 20,
    "/tf_static": 0,
    "/sensing/gnss/pose_with_covariance": -1,  # optional topic
}

REQUIRED_FPS_PATTERN_B = {
    "/localization/kinematic_state": 40,
    "/localization/util/downsample/pointcloud": 7.5,
    "/localization/twist_estimator/twist_with_covariance": 20,
}


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=pathlib.Path)
    parser.add_argument(
        "--reference_topic_name",
        type=str,
        default="/localization/kinematic_state",
        choices=[
            "/localization/kinematic_state",
            "/awsim/ground_truth/localization/kinematic_state",
        ],
    )
    parser.add_argument(
        "--add_initialpose",
        action="store_true")
    return parser.parse_args()


def check_rosbag(duration: float, topic_name_to_msg_list: dict, required_fps: dict) -> bool:
    """Check if the rosbag is suitable for ndt evaluation."""
    print(f"{duration=:.1f} sec")

    df = pd.DataFrame(
        {
            "topic": list(topic_name_to_msg_list.keys()),
            "count": [len(msg_list) for msg_list in topic_name_to_msg_list.values()],
        }
    )
    df = df[df["topic"].isin(required_fps.keys())]
    df["fps"] = df["count"] / duration
    df["enough_fps"] = df["fps"] > df["topic"].map(required_fps)
    print(df)
    return df["enough_fps"].all()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    reference_topic_name = args.reference_topic_name

    # prepare option
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    # prepare reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(rosbag_path), storage_id="sqlite3")
    reader.open(storage_options, converter_options)

    # filter topics
    target_topics = list(REQUIRED_FPS_PATTERN_A.keys()) + list(REQUIRED_FPS_PATTERN_B.keys())
    target_topics = list(set(target_topics))
    if reference_topic_name not in target_topics:
        target_topics.append(reference_topic_name)
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    # get map of topic name and type
    type_map = {}
    for topic_type in reader.get_all_topics_and_types():
        if topic_type.name in target_topics:
            type_map[topic_type.name] = topic_type.type

    # read rosbag
    topic_name_to_msg_list = {topic: [] for topic in target_topics}
    tuple_list = []
    while reader.has_next():
        (topic_name, data, timestamp_rosbag) = reader.read_next()
        tuple_list.append((topic_name, data, timestamp_rosbag))

        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_type)
        topic_name_to_msg_list[topic_name].append(msg)
    duration = (tuple_list[-1][2] - tuple_list[0][2]) * 1e-9

    # check
    save_topics = list()
    if check_rosbag(duration, topic_name_to_msg_list, REQUIRED_FPS_PATTERN_A):
        save_topics = list(REQUIRED_FPS_PATTERN_A.keys())
        print("OK as pattern A")
    elif check_rosbag(duration, topic_name_to_msg_list, REQUIRED_FPS_PATTERN_B):
        save_topics = list(REQUIRED_FPS_PATTERN_B.keys())
        print("OK as pattern B")
    else:
        print("The rosbag is not suitable for ndt evaluation")
        exit()

    # get first msg from /localization/kinematic_state
    first_reference = topic_name_to_msg_list[reference_topic_name][0]

    # if there is not gnss topic or selected from args, add initialpose
    save_initialpose = (
        len(topic_name_to_msg_list["/sensing/gnss/pose_with_covariance"]) == 0 or args.add_initialpose)
    if save_initialpose:
        print("Add /initialpose")
        type_map["/initialpose"] = "geometry_msgs/msg/PoseWithCovarianceStamped"
        save_topics.append("/initialpose")
        stamp = first_reference.header.stamp
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.pose = first_reference.pose
        data = serialize_message(msg)
        timestamp = int(stamp.sec * 1e9 + stamp.nanosec)
        tuple_list.append(
            ("/initialpose", data, timestamp)
        )

    # write rosbag
    rosbag_dir = rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    filtered_rosbag_path = rosbag_dir / "input_bag"
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(filtered_rosbag_path), storage_id="sqlite3")
    writer.open(storage_options, converter_options)
    for topic_name, topic_type in type_map.items():
        if topic_name not in save_topics:
            continue
        if topic_name == reference_topic_name:
            topic_name = "/localization/reference_kinematic_state"
        topic_info = rosbag2_py.TopicMetadata(
            name=topic_name, type=topic_type, serialization_format=serialization_format
        )
        writer.create_topic(topic_info)
    for topic_name, data, timestamp_rosbag in tuple_list:
        if topic_name not in save_topics:
            continue
        if topic_name == reference_topic_name:
            topic_name = "/localization/reference_kinematic_state"
        writer.write(topic_name, data, timestamp_rosbag)

    print(f"rosbag is saved at {filtered_rosbag_path}")
