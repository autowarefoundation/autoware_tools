#!/usr/bin/env python3
"""A script to analyze rosbags in parallel."""

import argparse
from multiprocessing import Pool
from pathlib import Path

import compare_trajectories
import extract_pose_from_rosbag
import plot_diagnostics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--parallel_num", type=int, default=1)
    parser.add_argument("--topic_subject", type=str, default="/localization/kinematic_state")
    parser.add_argument(
        "--topic_reference", type=str, default="/localization/pose_estimator/pose_with_covariance"
    )
    return parser.parse_args()


def process_directory(directory: Path, topic_subject: str, topic_reference: str) -> None:
    target_rosbag = directory / "result_bag"
    compare_result_dir = directory / "compare_trajectories"
    compare_result_dir.mkdir(parents=True, exist_ok=True)

    plot_diagnostics.main(rosbag_path=target_rosbag)

    save_name_subject = extract_pose_from_rosbag.topic_name_to_save_name(topic_subject)
    save_name_reference = extract_pose_from_rosbag.topic_name_to_save_name(topic_reference)

    extract_pose_from_rosbag.main(
        rosbag_path=target_rosbag,
        target_topics=[
            topic_subject,
            topic_reference,
        ],
        save_dir=compare_result_dir,
    )

    compare_trajectories.main(
        subject_tsv=compare_result_dir / f"{save_name_subject}.tsv",
        reference_tsv=compare_result_dir / f"{save_name_reference}.tsv",
    )


if __name__ == "__main__":
    args = parse_args()
    result_dir = args.result_dir
    parallel_num = args.parallel_num
    topic_subject = args.topic_subject
    topic_reference = args.topic_reference

    directories = sorted([d for d in args.result_dir.iterdir() if d.is_dir() and not d.is_symlink()])

    with Pool(args.parallel_num) as pool:
        pool.starmap(
            process_directory,
            [(d, topic_subject, topic_reference) for d in directories],
        )
