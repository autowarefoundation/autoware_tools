"""Fix timestamp of a rosbag which is recorded by AWSIM.

When a rosbag is recorded by AWSIM without '--use-sim-time' option,
the timestamp in the header of each message is simulated time, but the timestamp in the rosbag file is the wall time.
This script fixes the timestamp in the rosbag file to match the simulated time.
"""

import argparse
import pathlib
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=pathlib.Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path

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

    tuple_list = []

    first_timestamp = None

    while reader.has_next():
        (topic_name, data, timestamp_rosbag) = reader.read_next()

        msg_type = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_type)
        if hasattr(msg, "header"):
            timestamp_header = int(int(msg.header.stamp.sec) * 1e9 + int(msg.header.stamp.nanosec))
            if first_timestamp is None:
                first_timestamp = timestamp_header
        else:
            # /tf_static does not have header, so use the first timestamp - 1
            # (/tf_static should be at the beginning of the rosbag file)
            timestamp_header = 0 if first_timestamp is None else first_timestamp - 1
        tuple_list.append((topic_name, data, timestamp_header))

    # write rosbag
    rosbag_dir = rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    filtered_rosbag_path = rosbag_dir / "input_bag_sim_time"
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(filtered_rosbag_path), storage_id="sqlite3")
    writer.open(storage_options, converter_options)
    for topic_name, topic_type in type_map.items():
        topic_info = rosbag2_py.TopicMetadata(
            name=topic_name, type=topic_type, serialization_format=serialization_format
        )
        writer.create_topic(topic_info)
    for topic_name, data, timestamp_rosbag in tuple_list:
        writer.write(topic_name, data, timestamp_rosbag)

    print(f"rosbag is saved at {filtered_rosbag_path}")
