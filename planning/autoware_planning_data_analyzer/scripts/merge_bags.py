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
"""Merge multiple rosbag2 bags into a single output bag."""

import argparse
from pathlib import Path
import sys

from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions


def merge_bags(input_bags: list[str], output_bag: str, storage_id: str = "mcap") -> None:
    """
    Merge multiple input bags into a single output bag.

    Args:
        input_bags: List of input bag paths
        output_bag: Output bag path
        storage_id: Storage format (default: mcap)
    """
    print(f"Merging {len(input_bags)} bags into {output_bag}")
    print(f"Input bags: {input_bags}")

    # Set up writer
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=output_bag, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    writer.open(storage_options, converter_options)

    # Track topics we've already added to avoid duplicates
    added_topics = set()

    # Process each input bag
    for bag_path in input_bags:
        print(f"\nProcessing: {bag_path}")

        reader = SequentialReader()
        input_storage = StorageOptions(uri=bag_path, storage_id="")
        input_converter = ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )

        try:
            reader.open(input_storage, input_converter)
        except Exception as e:
            print(f"ERROR: Failed to open {bag_path}: {e}")
            continue

        # Get metadata
        metadata = reader.get_metadata()
        print(f"  Topics: {len(metadata.topics_with_message_count)}")
        print(f"  Messages: {metadata.message_count}")
        print(f"  Duration: {metadata.duration.nanoseconds / 1e9:.2f}s")

        # Add topics from this bag
        topic_types = reader.get_all_topics_and_types()
        for topic_metadata in topic_types:
            topic_name = topic_metadata.name
            if topic_name not in added_topics:
                writer.create_topic(topic_metadata)
                added_topics.add(topic_name)
                print(f"    Added topic: {topic_name}")

        # Copy all messages
        message_count = 0
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            writer.write(topic, data, timestamp)
            message_count += 1

            if message_count % 1000 == 0:
                print(f"    Copied {message_count} messages...", end="\r")

        print(f"    Copied {message_count} messages total")

    # Close writer
    print(f"\nMerge complete! Output: {output_bag}")
    print(f"Total topics: {len(added_topics)}")


def main():
    parser = argparse.ArgumentParser(
        description="Merge multiple rosbag2 bags into one",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Merge DLR result + evaluation metrics
  python3 merge_bags.py \\
    --input /tmp/result_bag/result_bag_0.mcap /tmp/eval_metrics.bag \\
    --output /tmp/final_combined.bag \\
    --storage mcap

  # Merge multiple evaluation runs
  python3 merge_bags.py \\
    --input run1.bag run2.bag run3.bag \\
    --output all_runs_combined.bag
        """,
    )

    parser.add_argument(
        "--input", "-i", nargs="+", required=True, help="Input bag paths (space-separated)"
    )
    parser.add_argument("--output", "-o", required=True, help="Output bag path")
    parser.add_argument(
        "--storage",
        "-s",
        default="mcap",
        choices=["mcap", "sqlite3"],
        help="Output storage format (default: mcap)",
    )

    args = parser.parse_args()

    # Validate inputs exist
    for bag_path in args.input:
        if not Path(bag_path).exists():
            print(f"ERROR: Input bag does not exist: {bag_path}")
            sys.exit(1)

    # Check output doesn't exist
    if Path(args.output).exists():
        print(f"ERROR: Output bag already exists: {args.output}")
        print("Please remove it first or choose a different output path")
        sys.exit(1)

    try:
        merge_bags(args.input, args.output, args.storage)
    except Exception as e:
        print(f"ERROR: Merge failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
