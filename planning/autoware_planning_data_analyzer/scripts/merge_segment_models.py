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
"""Merge multiple model result bags for a single OR segment."""

import argparse
from pathlib import Path
import shutil
import sys
import tempfile

from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions


def merge_model_bags(output_dir: str, segment_id: int, model_names: list, final_output: str):
    """Merge all model result bags for one segment."""
    print(f"Merging segment {segment_id} models: {', '.join(model_names)}")

    temp_bag = tempfile.mkdtemp(prefix="merge_segment_")
    shutil.rmtree(temp_bag)

    try:
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=temp_bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

        all_topics = {}
        all_messages = []

        # Read each model's result bag
        for model_name in model_names:
            result_bag = f"{output_dir}/or_event_{segment_id}_{model_name}_output/result_bag"

            if not Path(result_bag).exists():
                print(f"  WARNING: Result bag not found: {result_bag}")
                continue

            print(f"  Reading {model_name} results...")

            reader = SequentialReader()
            reader.open(
                StorageOptions(uri=result_bag, storage_id=""), ConverterOptions("cdr", "cdr")
            )

            # Collect topics
            for meta in reader.get_all_topics_and_types():
                if meta.name not in all_topics:
                    all_topics[meta.name] = meta

            # Collect messages (skip unprefixed trajectory)
            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                # Skip unprefixed trajectory - we only want prefixed versions
                if (
                    topic
                    == "/planning/trajectory_generator/diffusion_planner_node/output/trajectory"
                ):
                    continue

                all_messages.append((topic, data, timestamp))

            del reader

        if len(all_messages) == 0:
            print("ERROR: No messages to merge!")
            sys.exit(1)

        print(f"  Total messages collected: {len(all_messages)}")
        print(f"  Total topics: {len(all_topics)}")

        # Create all topics
        for meta in all_topics.values():
            # Skip unprefixed trajectory topic
            if (
                meta.name
                == "/planning/trajectory_generator/diffusion_planner_node/output/trajectory"
            ):
                continue
            writer.create_topic(meta)

        # Sort messages by timestamp
        all_messages.sort(key=lambda x: x[2])

        # Write all messages
        for topic, data, timestamp in all_messages:
            writer.write(topic, data, timestamp)

        del writer

        # Skip Python reindexing - preserves message schemas for Lichtblick
        # print("  Reindexing...")
        # Reindexer().reindex(StorageOptions(uri=temp_bag, storage_id="mcap"))

        # Move to final location
        if Path(final_output).exists():
            shutil.rmtree(final_output)
        shutil.move(temp_bag, final_output)

        print(f"  ✓ Merged {len(model_names)} models into {final_output}")

    except Exception as e:
        if Path(temp_bag).exists():
            shutil.rmtree(temp_bag)
        raise e


def main():
    parser = argparse.ArgumentParser(description="Merge model result bags for OR segment")
    parser.add_argument("--output-dir", required=True, help="Base output directory")
    parser.add_argument("--segment-id", type=int, required=True, help="Segment ID")
    parser.add_argument("--models", nargs="+", required=True, help="Model names (e.g., v2.0 v2.1)")
    parser.add_argument("--final-output", required=True, help="Final merged bag directory")

    args = parser.parse_args()

    merge_model_bags(args.output_dir, args.segment_id, args.models, args.final_output)


if __name__ == "__main__":
    main()
