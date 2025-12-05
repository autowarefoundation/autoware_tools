#!/usr/bin/env python3
"""
Add prefixed trajectory topic to MCAP bag (in-place modification).
Workaround for broken topic_tools/relay in DLR.
"""
import argparse
import sys
import tempfile
import shutil
from pathlib import Path
from rosbag2_py import (
    SequentialReader, SequentialWriter, StorageOptions,
    ConverterOptions, TopicMetadata, Reindexer
)

def add_prefixed_topic(bag_path: str, prefix: str, source_topic: str):
    """Add prefixed copy of source topic to bag."""

    # Replace dots with underscores for ROS compliance (v2.0 -> v2_0)
    prefix_safe = prefix.replace('.', '_')
    prefixed_topic = f"/{prefix_safe}{source_topic}"

    print(f"Adding prefixed topic to: {bag_path}")
    print(f"  Source: {source_topic}")
    print(f"  Prefixed: {prefixed_topic}")

    # Step 1: Read all messages
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id=""), ConverterOptions("cdr", "cdr"))

    all_topics = reader.get_all_topics_and_types()
    traj_type = None
    for topic_meta in all_topics:
        if topic_meta.name == source_topic:
            traj_type = topic_meta.type
            break

    if not traj_type:
        print(f"ERROR: Topic {source_topic} not found in bag")
        sys.exit(1)

    traj_messages = []
    all_messages = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        all_messages.append((topic, data, timestamp))
        if topic == source_topic:
            traj_messages.append((data, timestamp))

    del reader

    if len(traj_messages) == 0:
        print(f"WARNING: No messages found at {source_topic}")
        sys.exit(0)

    print(f"  Found {len(traj_messages)} trajectory messages")

    # Step 2: Write new bag to temp location
    temp_bag = tempfile.mkdtemp(prefix="bag_prefix_")
    shutil.rmtree(temp_bag)  # Remove the directory, we just need the unique name

    try:
        writer = SequentialWriter()
        writer.open(StorageOptions(uri=temp_bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

        # Create all original topics
        for topic_meta in all_topics:
            writer.create_topic(topic_meta)

        # Create prefixed topic
        writer.create_topic(TopicMetadata(name=prefixed_topic, type=traj_type, serialization_format="cdr"))

        # Write all original messages
        for topic, data, timestamp in all_messages:
            writer.write(topic, data, timestamp)

        # Write prefixed trajectory messages
        for data, timestamp in traj_messages:
            writer.write(prefixed_topic, data, timestamp)

        del writer

        # Skip Python reindexing - preserves message schemas for Lichtblick
        # Reindexer().reindex(StorageOptions(uri=temp_bag, storage_id="mcap"))

        # Replace original
        shutil.rmtree(bag_path)
        shutil.move(temp_bag, bag_path)

        print(f"  ✓ Added {len(traj_messages)} messages to {prefixed_topic}")

    except Exception as e:
        if Path(temp_bag).exists():
            shutil.rmtree(temp_bag)
        raise e

def main():
    parser = argparse.ArgumentParser(description="Add prefixed trajectory topic to bag")
    parser.add_argument("--bag", required=True, help="Path to result_bag directory")
    parser.add_argument("--prefix", required=True, help="Topic prefix (e.g., v2.0)")
    parser.add_argument("--source-topic", default="/planning/trajectory_generator/diffusion_planner_node/output/trajectory",
                        help="Source topic to copy")

    args = parser.parse_args()

    if not Path(args.bag).exists():
        print(f"ERROR: Bag not found: {args.bag}")
        sys.exit(1)

    add_prefixed_topic(args.bag, args.prefix, args.source_topic)

if __name__ == "__main__":
    main()
