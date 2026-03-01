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

"""Rename a topic in a rosbag file."""

import argparse
from pathlib import Path

from rosbag2_py import ConverterOptions
from rosbag2_py import Reindexer
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import TopicMetadata


def rename_topic_in_bag(input_bag, output_bag, old_topic, new_topic):
    """
    Copy a bag while renaming a specific topic.

    Args:
        input_bag: Path to input bag directory or mcap file
        output_bag: Path to output bag directory
        old_topic: Original topic name to rename
        new_topic: New topic name
    """
    print("Renaming topic in bag:")
    print(f"  Input: {input_bag}")
    print(f"  Output: {output_bag}")
    print(" {old_topic} → {new_topic}")
    print("")

    # Auto-detect storage type
    input_path = Path(input_bag)
    if input_path.is_file() and input_path.suffix == ".mcap":
        storage_id = "mcap"
        uri = str(input_path)
    elif input_path.is_dir():
        storage_id = "sqlite3"
        uri = str(input_path)
    else:
        raise ValueError(f"Invalid input bag path: {input_bag}")

    # Setup reader
    storage_options_in = StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options_in, converter_options)

    # Setup writer (always output as mcap for consistency)
    storage_options_out = StorageOptions(uri=str(output_bag), storage_id="mcap")
    writer = SequentialWriter()
    writer.open(storage_options_out, converter_options)

    # Create topics in output (with renaming)
    topic_renamed = False
    for topic_metadata in reader.get_all_topics_and_types():
        output_topic_name = topic_metadata.name

        if topic_metadata.name == old_topic:
            output_topic_name = new_topic
            topic_renamed = True
            print(f"✓ Renamed topic: {old_topic} → {new_topic}")

        new_metadata = TopicMetadata(
            name=output_topic_name,
            type=topic_metadata.type,
            serialization_format=topic_metadata.serialization_format,
        )
        writer.create_topic(new_metadata)

    if not topic_renamed:
        print(f"Warning: Topic {old_topic} not found in bag!")

    # Copy all messages (with topic renaming)
    message_count = 0
    renamed_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # Rename if needed
        if topic == old_topic:
            topic = new_topic
            renamed_count += 1

        writer.write(topic, data, timestamp)
        message_count += 1

        if message_count % 10000 == 0:
            print(f"Processed {message_count} messages...")

    print("\nCopy complete:")
    print(f"  Total messages: {message_count}")
    print(f"  Renamed messages: {renamed_count}")

    # Cleanup
    del writer
    del reader

    # Reindex for clean metadata
    print("Reindexing...")
    Reindexer().reindex(storage_options_out)
    print("Done!")


def main():
    parser = argparse.ArgumentParser(description="Rename a topic in a rosbag file")
    parser.add_argument("input_bag", help="Input bag directory or mcap file")
    parser.add_argument("output_bag", help="Output bag directory")
    parser.add_argument("old_topic", help="Topic name to rename")
    parser.add_argument("new_topic", help="New topic name")

    args = parser.parse_args()

    rename_topic_in_bag(args.input_bag, args.output_bag, args.old_topic, args.new_topic)


if __name__ == "__main__":
    main()
