#!/usr/bin/env python3
"""A script to analyze rosbags in parallel."""

import argparse
import json
from multiprocessing import Pool
from pathlib import Path

import compare_trajectories
import extract_values_from_rosbag
import pandas as pd
import plot_diagnostics
from utils.calc_acceleration_diff import calc_acceleration_diff


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--parallel_num", type=int, default=1)
    parser.add_argument("--topic_subject", type=str, default="/localization/kinematic_state")
    parser.add_argument(
        "--topic_reference",
        type=str,
        default="/localization/pose_estimator/pose_with_covariance",
    )
    parser.add_argument("--save_dir_relative", type=str, default="")
    return parser.parse_args()


def process_directory(
    directory: Path, topic_subject: str, topic_reference: str, save_dir_relative: str
) -> None:
    target_rosbag = directory / "result_bag"
    save_dir = directory if save_dir_relative == "" else directory / save_dir_relative
    compare_result_dir = save_dir / "compare_trajectories"
    compare_result_dir.mkdir(parents=True, exist_ok=True)
    diagnostics_result_dir = save_dir / "diagnostics_result"

    # (1) Plot diagnostics
    plot_diagnostics.main(rosbag_path=target_rosbag, save_dir=diagnostics_result_dir)

    # (2) Extract values from rosbag
    save_name_subject = extract_values_from_rosbag.topic_name_to_save_name(topic_subject)
    save_name_reference = extract_values_from_rosbag.topic_name_to_save_name(topic_reference)
    extract_values_from_rosbag.main(
        rosbag_path=target_rosbag,
        target_topics=[
            topic_subject,
            topic_reference,
        ],
        save_dir=compare_result_dir,
    )

    # (3) Compare trajectories
    compare_trajectories.main(
        subject_tsv=compare_result_dir / f"{save_name_subject}.tsv",
        reference_tsv=compare_result_dir / f"{save_name_reference}.tsv",
    )

    # (4) Extract twist
    twist_result_dir = save_dir / "result_twist"
    twist_result_dir.mkdir(parents=True, exist_ok=True)
    extract_values_from_rosbag.main(
        rosbag_path=target_rosbag,
        target_topics=["/localization/kinematic_state"],
        save_dir=twist_result_dir,
    )

    # (5) Extract acceleration
    acceleration_result_dir = save_dir / "result_acceleration"
    acceleration_result_dir.mkdir(parents=True, exist_ok=True)
    extract_values_from_rosbag.main(
        rosbag_path=target_rosbag,
        target_topics=["/localization/acceleration"],
        save_dir=acceleration_result_dir,
    )

    # (6) Create summary
    final_success = True
    final_summary = ""

    # (6-1) relative pose
    relative_pose_tsv = (
        save_dir / "compare_trajectories/localization__kinematic_state_result/relative_pose.tsv"
    )
    df = pd.read_csv(relative_pose_tsv, sep="\t")

    # position
    mean_position_norm = df["position.norm"].mean()
    THRESHOLD_MEAN_POSITION_NORM = 0.5
    if mean_position_norm > THRESHOLD_MEAN_POSITION_NORM:
        final_success = False
        final_summary += f"{mean_position_norm=:.3f} [m] is too large."
    else:
        final_summary += f"{mean_position_norm=:.3f} [m]"

    # angle
    mean_angle_norm = df["angle.norm"].mean()
    THRESHOLD_MEAN_ANGLE_NORM = 0.5
    if mean_angle_norm > THRESHOLD_MEAN_ANGLE_NORM:
        final_success = False
        final_summary += f"|{mean_angle_norm=:.3f} [deg] is too large."
    else:
        final_summary += f"|{mean_angle_norm=:.3f} [deg]"

    # linear velocity
    if "linear_velocity.norm" in df.columns:
        mean_linear_velocity_norm = df["linear_velocity.norm"].mean()
        THRESHOLD_MEAN_LINEAR_VELOCITY_NORM = 0.05
        if mean_linear_velocity_norm > THRESHOLD_MEAN_LINEAR_VELOCITY_NORM:
            final_success = False
            final_summary += f"|{mean_linear_velocity_norm=:.3f} [m/s] is too large."
        else:
            final_summary += f"|{mean_linear_velocity_norm=:.3f} [m/s]"

    # angular velocity
    if "angular_velocity.norm" in df.columns:
        mean_angular_velocity_norm = df["angular_velocity.norm"].mean()
        THRESHOLD_MEAN_ANGULAR_VELOCITY_NORM = 0.05
        if mean_angular_velocity_norm > THRESHOLD_MEAN_ANGULAR_VELOCITY_NORM:
            final_success = False
            final_summary += f"|{mean_angular_velocity_norm=:.3f} [rad/s] is too large."
        else:
            final_summary += f"|{mean_angular_velocity_norm=:.3f} [rad/s]"

    # acceleration
    df_ref = pd.read_csv(compare_result_dir / f"{save_name_reference}.tsv", sep="\t")
    if "linear_velocity.x" in df_ref.columns:
        THRESHOLD_MEAN_ACCELERATION_NORM = 0.5
        acceleration_tsv = save_dir / "result_acceleration/localization__acceleration.tsv"
        df_acceleration = pd.read_csv(acceleration_tsv, sep="\t")
        df_result = calc_acceleration_diff(df_acceleration, df_ref)
        df_result.to_csv(
            save_dir / "result_acceleration/localization__acceleration_diff.tsv",
            sep="\t",
            index=False,
        )
        mean_acceleration_norm_diff = df_result["diff.norm"].mean()

        # Check threshold
        if mean_acceleration_norm_diff > THRESHOLD_MEAN_ACCELERATION_NORM:
            final_success = False
            final_summary += f"|{mean_acceleration_norm_diff=:.3f} [m/s²] is too large."
        else:
            final_summary += f"|{mean_acceleration_norm_diff=:.3f} [m/s²]"

    # (6-2) diagnostics
    target_tsv_list = [
        "localization__ekf_localizer",
        "localization__pose_instability_detector",
        "localization_error_monitor__ellipse_error_status",
        "ndt_scan_matcher__scan_matching_status",
    ]
    for target_tsv in target_tsv_list:
        diagnostics_tsv = diagnostics_result_dir / f"{target_tsv}.tsv"
        df = pd.read_csv(diagnostics_tsv, sep="\t")
        # filter out WARNs before activation
        df = df[df["message"] != "[WARN]process is not activated; [WARN]initial pose is not set"]
        df = df[df["message"] != "Node is not activated."]
        not_ok_num = len(df[df["level"] != 0])
        if len(df) == 0:
            not_ok_percentage = 0
        else:
            not_ok_num = len(df[df["level"] != 0])
            not_ok_percentage = not_ok_num / len(df) * 100
        if not_ok_percentage > 1:
            final_success = False
            final_summary += f"|{target_tsv} {not_ok_percentage:.3f} [%] is too large."
        else:
            final_summary += f"|{target_tsv} {not_ok_percentage:.3f} [%]"

    with open(save_dir / "summary.json", "w") as f:
        json.dump(
            {
                "Result": {
                    "Success": final_success,
                    "Summary": final_summary,
                }
            },
            f,
            indent=2,
        )


if __name__ == "__main__":
    args = parse_args()
    result_dir = args.result_dir
    parallel_num = args.parallel_num
    topic_subject = args.topic_subject
    topic_reference = args.topic_reference
    save_dir_relative = args.save_dir_relative

    target_rosbags = list(result_dir.glob("**/*.db3")) + list(result_dir.glob("**/*.mcap"))
    directories = [path.parent.parent for path in target_rosbags]
    directories = list(set(directories))
    directories = [d for d in directories if not d.is_symlink()]
    directories = sorted(directories)

    with Pool(args.parallel_num) as pool:
        pool.starmap(
            process_directory,
            [(d, topic_subject, topic_reference, save_dir_relative) for d in directories],
        )
