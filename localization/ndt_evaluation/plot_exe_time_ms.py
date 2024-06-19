import argparse
import pathlib

from interpolate_pose import interpolate_pose
import matplotlib.pyplot as plt
import pandas as pd
from rclpy.serialization import deserialize_message
import rosbag2_py
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

    target_topics = [
        "/localization/pose_estimator/exe_time_ms",
        "/localization/pose_estimator/pose",
    ]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    ndt_exe_time_ms = list()
    ndt_pose_list = list()

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic == "/localization/pose_estimator/exe_time_ms":
            timestamp_header = int(msg.stamp.sec) + int(msg.stamp.nanosec) * 1e-9
            ndt_exe_time_ms.append(
                {
                    "timestamp": timestamp_header,
                    "value": msg.data,
                }
            )
        elif topic == "/localization/pose_estimator/pose":
            timestamp_header = int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9
            pose = msg.pose
            ndt_pose_list.append(
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
            assert False, f"Unknown topic: {topic}"

    print(f"{len(ndt_exe_time_ms)=}")
    print(f"{len(ndt_pose_list)=}")

    df_ndt_exe_time_ms = pd.DataFrame(ndt_exe_time_ms)
    df_ndt_pose = pd.DataFrame(ndt_pose_list)

    df_ndt_pose = interpolate_pose(df_ndt_pose, df_ndt_exe_time_ms["timestamp"])
    print(f"{len(df_ndt_exe_time_ms)=}")
    print(f"{len(df_ndt_pose)=}")

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "exe_time_ms"
    save_dir.mkdir(exist_ok=True)

    # plot exe_time_ms
    plt.plot(df_ndt_exe_time_ms["timestamp"], df_ndt_exe_time_ms["value"])
    plt.xlabel("timestamp")
    plt.ylabel("exe_time [ms]")
    save_path = save_dir / "exe_time_ms.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot exe_time_ms on trajectory of ndt_pose
    plt.scatter(
        df_ndt_pose["x"], df_ndt_pose["y"], c=df_ndt_exe_time_ms["value"].values, cmap="viridis"
    )
    plt.colorbar(label="pose_instability_diff_pose[m]")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    save_path = save_dir / "exe_time_ms_on_ndt_pose.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
