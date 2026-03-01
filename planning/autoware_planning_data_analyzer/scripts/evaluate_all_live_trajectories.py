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
"""Evaluate all LIVE trajectory topics in a bag simultaneously."""

import argparse
import json
from pathlib import Path
import sys

import rclpy
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions


def detect_live_trajectory_topics(bag_path: str):
    """Detect all LIVE trajectory topics with prefixes."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id=""), ConverterOptions("cdr", "cdr"))

    live_topics = []
    for meta in reader.get_all_topics_and_types():
        # Look for trajectory topics with prefixes (e.g., /v2.0/planning/...)
        if "trajectory_generator" in meta.name and "/output/trajectory" in meta.name:
            # Extract prefix (first component after /)
            parts = meta.name.strip("/").split("/")
            if parts[0] not in ["planning", "control", "localization", "perception"]:
                # This is a prefixed topic
                prefix = parts[0]
                # Convert underscores back to dots for display (v2_0 -> v2.0)
                prefix_display = prefix.replace("_", ".")
                live_topics.append(
                    {
                        "topic": meta.name,
                        "prefix": prefix,
                        "prefix_display": prefix_display,
                        "type": meta.type,
                    }
                )

    del reader
    return live_topics


def evaluate_all_trajectories(
    bag_path: str,
    input_bag_path: str,
    map_path: str,
    output_bag_path: str,
    json_output_path: str = None,
    time_window: float = 0.5,
):
    """
    Evaluate all LIVE trajectories in a single pass.

    Args:
        bag_path: Path to bag with LIVE trajectories (result bag)
        input_bag_path: Path to original input bag (for OR event extraction)
        map_path: Path to lanelet2 map file
        output_bag_path: Output bag with metrics injected
        json_output_path: Optional JSON output file
    """
    print(f"Evaluating all LIVE trajectories in: {bag_path}")
    print(f"Input bag (for OR extraction): {input_bag_path}")

    # Step 1: Detect LIVE trajectory topics
    live_topics = detect_live_trajectory_topics(bag_path)

    if len(live_topics) == 0:
        print("ERROR: No LIVE trajectory topics found!")
        sys.exit(1)

    print(f"\nDetected {len(live_topics)} LIVE trajectory topics:")
    for lt in live_topics:
        print(f"  {lt['topic']} (prefix: {lt['prefix']})")

    # Step 2: Run OR scene evaluation for each LIVE trajectory
    all_results = {}

    for lt in live_topics:
        prefix = lt["prefix"]
        traj_topic = lt["topic"]

        print(f"\nEvaluating {prefix}...")

        # Clean up old metric bag if it exists
        import shutil

        metric_bag_dir = f"/tmp/eval_{prefix}_metrics.bag"
        if Path(metric_bag_dir).exists():
            shutil.rmtree(metric_bag_dir)

        # Run autoware_planning_data_analyzer_node for this trajectory
        import subprocess

        cmd = [
            "ros2",
            "run",
            "autoware_planning_data_analyzer",
            "autoware_planning_data_analyzer_node",
            "--ros-args",
            "-p",
            f"bag_path:={bag_path}",
            "-p",
            "evaluation.mode:=or_scene",
            "-p",
            f"trajectory_topic:={traj_topic}",
            "-p",
            f"or_scene_evaluation.input_bag_path:={input_bag_path}",
            "-p",
            f"or_scene_evaluation.map_path:={map_path}",
            "-p",
            f"or_scene_evaluation.time_window_sec:={time_window}",
            "-p",
            "or_scene_evaluation.enable_debug_visualization:=false",
            "-p",
            f"evaluation_output_bag_path:=/tmp/eval_{prefix}_metrics.bag",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"  WARNING: Evaluation failed for {prefix}")
            print(f"  {result.stderr[:500]}")
            continue

        # Find the JSON results (saved to home directory with timestamp)
        import glob

        json_pattern = str(Path.home() / "or_scene_evaluation_results_*.json")
        json_files = sorted(
            glob.glob(json_pattern), key=lambda x: Path(x).stat().st_mtime, reverse=True
        )

        if len(json_files) > 0:
            latest_json = json_files[0]
            with open(latest_json, "r") as f:
                all_results[prefix] = json.load(f)
            print(f"  ✓ Evaluation complete for {prefix}")
            print(f"    JSON: {latest_json}")
        else:
            print(f"  WARNING: JSON output not found for {prefix}")

    # Step 3: Merge all metric bags into output bag
    print(f"\nMerging metrics into output bag: {output_bag_path}")

    # Collect all topics and messages from original bag
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id=""), ConverterOptions("cdr", "cdr"))

    all_topics = {}
    for meta in reader.get_all_topics_and_types():
        all_topics[meta.name] = meta

    all_messages = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        all_messages.append((topic, data, timestamp))

    del reader

    # Collect metric topics and messages from each model
    for prefix in all_results.keys():
        metric_bag = f"/tmp/eval_{prefix}_metrics.bag"

        if not Path(metric_bag).exists():
            continue

        print(f"  Reading metrics for {prefix}...")

        metric_reader = SequentialReader()
        metric_reader.open(
            StorageOptions(uri=metric_bag, storage_id=""), ConverterOptions("cdr", "cdr")
        )

        # Collect metric topics (only those with messages)
        metric_topics = {}
        for meta in metric_reader.get_all_topics_and_types():
            metric_topics[meta.name] = meta

        # Collect metric messages
        topics_with_messages = set()
        while metric_reader.has_next():
            topic, data, timestamp = metric_reader.read_next()
            all_messages.append((topic, data, timestamp))
            topics_with_messages.add(topic)

        del metric_reader

        # Add metric topics that have messages
        for topic_name, meta in metric_topics.items():
            if topic_name in topics_with_messages:
                all_topics[topic_name] = meta

        print(f"  ✓ Added metrics for {prefix}")

    print(f"  Total messages: {len(all_messages)}")
    print(f"  Total topics: {len(all_topics)}")

    # Write to temp bag
    import shutil
    import tempfile

    temp_bag = tempfile.mkdtemp(prefix="eval_merge_")
    shutil.rmtree(temp_bag)  # Remove the directory, we just need the unique name

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=temp_bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

    # Create all topics
    for meta in all_topics.values():
        writer.create_topic(meta)

    # Sort messages by timestamp (CRITICAL!)
    all_messages.sort(key=lambda x: x[2])

    # Write all messages
    for topic, data, timestamp in all_messages:
        writer.write(topic, data, timestamp)

    del writer

    # Skip Python reindexing - use CLI instead to preserve message schemas
    # print("  Reindexing...")
    # Reindexer().reindex(StorageOptions(uri=temp_bag, storage_id="mcap"))

    # Move to final location
    if Path(output_bag_path).exists():
        shutil.rmtree(output_bag_path)
    shutil.move(temp_bag, output_bag_path)

    # Save combined JSON
    if json_output_path:
        with open(json_output_path, "w") as f:
            json.dump(all_results, f, indent=2)
        print(f"  ✓ Combined results saved to: {json_output_path}")

    print(f"\n✓ Evaluation complete for {len(all_results)} models")
    print(f"  Output bag: {output_bag_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate all LIVE trajectories in a bag simultaneously"
    )
    parser.add_argument("--bag", required=True, help="Bag with LIVE trajectories")
    parser.add_argument("--input-bag", required=True, help="Original input bag for OR extraction")
    parser.add_argument("--map-path", required=True, help="Path to lanelet2 map file")
    parser.add_argument("--output-bag", required=True, help="Output bag with metrics")
    parser.add_argument("--json-output", help="Combined JSON output file")
    parser.add_argument(
        "--time-window",
        type=float,
        default=5.0,
        help="Evaluation window on each side of OR (default: 5.0s)",
    )

    args = parser.parse_args()

    if not Path(args.bag).exists():
        print(f"ERROR: Bag not found: {args.bag}")
        sys.exit(1)

    if not Path(args.input_bag).exists():
        print(f"ERROR: Input bag not found: {args.input_bag}")
        sys.exit(1)

    if not Path(args.map_path).exists():
        print(f"ERROR: Map not found: {args.map_path}")
        sys.exit(1)

    rclpy.init()

    evaluate_all_trajectories(
        args.bag, args.input_bag, args.map_path, args.output_bag, args.json_output, args.time_window
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
