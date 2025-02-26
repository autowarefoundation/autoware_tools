#!/usr/bin/env python3
"""A script to parse diagnostics messages from a rosbag file."""

import argparse
from pathlib import Path

from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt
import pandas as pd
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--save_dir", type=Path, default=None)
    return parser.parse_args()


def parse_stamp(stamp: Time) -> int:
    return stamp.sec * int(1e9) + stamp.nanosec


def diag_name_to_filename(diag_name: str) -> str:
    return diag_name.replace(":", "_").replace(" ", "_")


def parse_diagnostics_msgs(rosbag_path: str, target_list: list) -> dict:
    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(
        uri=str(rosbag_path),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=["/diagnostics"])
    reader.set_filter(storage_filter)

    data_dict: dict = {key: [] for key in target_list}

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = parse_stamp(msg.header.stamp)
        for status in msg.status:
            if status.name not in target_list:
                continue
            curr_data = {kv.key: kv.value for kv in status.values}
            curr_data["timestamp_rosbag"] = timestamp_rosbag
            curr_data["timestamp_header"] = timestamp_header
            curr_data["level"] = int.from_bytes(status.level, "big")
            curr_data["message"] = status.message
            data_dict[status.name].append(curr_data)
    return data_dict


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    save_dir = args.save_dir

    if save_dir is None:
        if rosbag_path.is_dir():  # if specified directory containing db3 files
            save_dir = rosbag_path.parent / "diagnostics_result"
        else:  # if specified db3 file directly
            save_dir = rosbag_path.parent.parent / "diagnostics_result"

    target_list = [
        "ndt_scan_matcher: scan_matching_status",
        "localization: ekf_localizer",
        "localization_error_monitor: ellipse_error_status",
        "localization: pose_instability_detector",
        "gyro_bias_validator: gyro_bias_validator",
    ]

    data_dict = parse_diagnostics_msgs(rosbag_path, target_list)

    save_dir.mkdir(exist_ok=True)
    print(f"{save_dir=}")

    ################################
    # save diagnostics data as tsv #
    ################################
    for key, data in data_dict.items():
        df = pd.DataFrame(data)
        for col in df.columns:
            if pd.api.types.is_numeric_dtype(df[col]):
                df[col] = df[col].astype(float)
                df[col] = df[col].apply(lambda x: int(x) if x.is_integer() else x)
        filename = diag_name_to_filename(key)
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    # Fix timestamp to relative time from the first message and convert to seconds
    # (for better visualization)
    for one_data_dict_key in data_dict:
        one_data_dict = data_dict[one_data_dict_key]
        if len(one_data_dict) == 0:
            continue
        first_h = int(one_data_dict[0]["timestamp_header"])
        first_r = int(one_data_dict[0]["timestamp_rosbag"])
        for row in one_data_dict:
            row["timestamp_header"] = int(row["timestamp_header"]) - first_h
            row["timestamp_rosbag"] = int(row["timestamp_rosbag"]) - first_r
            row["timestamp_header"] /= 1e9
            row["timestamp_rosbag"] /= 1e9

    ####################
    # ndt_scan_matcher #
    ####################
    diag_name = "ndt_scan_matcher: scan_matching_status"
    df = pd.DataFrame(data_dict[diag_name])
    key_list = [
        "execution_time",
        "iteration_num",
        "sensor_points_size",
        "sensor_points_delay_time_sec",
        "skipping_publish_num",
        "transform_probability",
        "transform_probability_diff",
        "nearest_voxel_transformation_likelihood",
        "nearest_voxel_transformation_likelihood_diff",
        "local_optimal_solution_oscillation_num",
        "distance_initial_to_result",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        df = df.dropna(subset=[key])
        plt.subplot(4, 3, i + 1)
        plt.plot(df["timestamp_header"], df[key], label=key)
        if key == "nearest_voxel_transformation_likelihood":
            plt.plot(
                df["timestamp_header"],
                [2.3 for _ in range(len(df))],
                label="threshold",
                linestyle="dashed",
            )
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.ylim(bottom=min(0, df[key].min()))
        plt.grid()

    plt.tight_layout()
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)

    #################
    # ekf_localizer #
    #################
    diag_name = "localization: ekf_localizer"
    df = pd.DataFrame(data_dict[diag_name])
    df = df[df["is_activated"] == "True"]
    key_list = [
        "pose_mahalanobis_distance",
        "twist_mahalanobis_distance",
        "cov_ellipse_long_axis_size",
        "cov_ellipse_lateral_direction_size",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        key_threshold = (
            key + "_threshold" if "mahalanobis" in key else key.replace("_size", "_warn_threshold")
        )
        df[key_threshold] = df[key_threshold].astype(float)
        plt.subplot(4, 1, (i + 1))
        plt.plot(df["timestamp_header"], df[key], label=key)
        plt.plot(df["timestamp_header"], df[key_threshold], label=key_threshold)
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.grid()

    plt.tight_layout()
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)

    #############################
    # pose_instability_detector #
    #############################
    diag_name = "localization: pose_instability_detector"
    df = pd.DataFrame(data_dict[diag_name])

    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    target_names = ["position", "angle"]
    for i, target_name in enumerate(target_names):
        plt.subplot(2, 1, i + 1)
        key_list = [
            f"diff_{target_name}_x",
            f"diff_{target_name}_y",
            f"diff_{target_name}_z",
        ]
        for key in key_list:
            key_value = key + ":value"
            key_threshold = key + ":threshold"
            if key_value not in df.columns or key_threshold not in df.columns:
                print(f"Skip {key}")
                continue
            df[key_value] = df[key_value].astype(float)
            df[key_threshold] = df[key_threshold].astype(float)
            plt.plot(df["timestamp_header"], df[key_value], label=key_value)
            plt.plot(
                df["timestamp_header"],
                df[key_threshold],
                linestyle="dashed",
            )
            plt.plot(
                df["timestamp_header"],
                -df[key_threshold],
                linestyle="dashed",
            )
        plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left", borderaxespad=0)
        plt.xlabel("time [s]")
        unit = "[m]" if target_name == "position" else "[rad]"
        plt.ylabel(f"diff_{target_name} {unit}")
        plt.grid()

    plt.tight_layout()
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)

    ##############################
    # localization_error_monitor #
    ##############################
    diag_name = "localization_error_monitor: ellipse_error_status"
    df = pd.DataFrame(data_dict[diag_name])
    key_list = [
        "localization_error_ellipse",
        "localization_error_ellipse_lateral_direction",
        "level",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for _, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        plt.plot(df["timestamp_header"], df[key], label=key)
    plt.xlabel("time [s]")
    plt.ylabel("error_ellipse [m]")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)

    #######################
    # gyro_bias_validator #
    #######################
    diag_name = "gyro_bias_validator: gyro_bias_validator"
    df = pd.DataFrame(data_dict[diag_name])
    key_list = [
        "estimated_gyro_bias_x",
        "estimated_gyro_bias_y",
        "estimated_gyro_bias_z",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for _, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        plt.plot(df["timestamp_header"], df[key], label=key)
    if "gyro_bias_threshold" in df.columns:
        plt.plot(
            df["timestamp_header"],
            df["gyro_bias_threshold"].astype(float),
            linestyle="dashed",
        )
        plt.plot(
            df["timestamp_header"],
            -df["gyro_bias_threshold"].astype(float),
            linestyle="dashed",
        )
    plt.xlabel("time [s]")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
