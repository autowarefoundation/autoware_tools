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
    return parser.parse_args()


def process_directory(directory: Path):
    target_rosbag = directory / "result_bag"
    compare_result_dir = directory / "compare_trajectories"
    compare_result_dir.mkdir(parents=True, exist_ok=True)

    plot_diagnostics.main(rosbag_path=target_rosbag)

    extract_pose_from_rosbag.main(
        rosbag_path=target_rosbag,
        target_topics=[
            "/localization/kinematic_state",
            "/localization/reference_kinematic_state",
        ],
        save_dir=compare_result_dir,
    )

    compare_trajectories.main(
        subject_tsv=compare_result_dir / "localization__kinematic_state.tsv",
        reference_tsv=compare_result_dir / "localization__reference_kinematic_state.tsv",
    )


if __name__ == "__main__":
    args = parse_args()
    result_dir = args.result_dir
    parallel_num = args.parallel_num

    directories = sorted([d for d in args.result_dir.iterdir() if d.is_dir()])

    with Pool(args.parallel_num) as pool:
        pool.map(process_directory, directories)
