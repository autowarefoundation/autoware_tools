#!/usr/bin/env python3
"""A script to extract pose from rosbag and save as tsv."""

import argparse
from pathlib import Path

from utils.parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--save_dir", type=Path, default=None)
    parser.add_argument("--target_topics", type=str, required=True, nargs="+")
    return parser.parse_args()


def topic_name_to_save_name(topic_name: str) -> str:
    return "__".join(topic_name.split("/")[1:])


def main(rosbag_path: Path, target_topics: list, save_dir: Path = None) -> None:
    if save_dir is None:
        if rosbag_path.is_dir():  # if specified directory containing db3 files
            save_dir = rosbag_path.parent / "tsv"
        else:  # if specified db3 file directly
            save_dir = rosbag_path.parent.parent / "tsv"
    save_dir.mkdir(parents=True, exist_ok=True)

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    for target_topic in target_topics:
        save_name = topic_name_to_save_name(target_topic)
        df = df_dict[target_topic]
        df.to_csv(
            f"{save_dir}/{save_name}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    print(f"Saved pose tsv files to {save_dir}")


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topics = args.target_topics
    save_dir = args.save_dir
    main(rosbag_path, target_topics, save_dir)
