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
"""Auto-discover and evaluate all OR segment bags in a directory."""
import argparse
from pathlib import Path
import subprocess
import sys


def find_or_segment_bags(input_dir: str):
    """Find all or_event_* bags in input directory."""
    bags = []
    input_path = Path(input_dir)

    # Look for or_event_N directories (N = 0, 1, 2, ...)
    for item in sorted(input_path.iterdir()):
        if item.is_dir() and item.name.startswith("or_event_") and item.name[9:].isdigit():
            # Check if it has MCAP files
            mcap_files = list(item.glob("*.mcap"))
            if len(mcap_files) > 0:
                segment_id = int(item.name[9:])
                bags.append({"segment_id": segment_id, "path": str(item), "name": item.name})

    return sorted(bags, key=lambda x: x["segment_id"])


def evaluate_all_segments(input_dir: str, map_path: str, time_window: float = 5.0):
    """Evaluate all OR segment bags found in input directory."""
    print(f"Searching for OR segment bags in: {input_dir}")

    segment_bags = find_or_segment_bags(input_dir)

    if len(segment_bags) == 0:
        print("ERROR: No OR segment bags found!")
        print("  Expected: or_event_0/, or_event_1/, ...")
        sys.exit(1)

    print(f"Found {len(segment_bags)} OR segment bags:")
    for bag in segment_bags:
        print(f"  {bag['name']}")

    print("")

    # Evaluate each segment
    for bag in segment_bags:
        segment_id = bag["segment_id"]
        bag_path = bag["path"]

        output_bag = f"{input_dir}/or_event_{segment_id}_WITH_METRICS"
        json_output = f"{input_dir}/or_event_{segment_id}_results.json"

        print(f"=== Evaluating Segment {segment_id} ===")

        cmd = [
            "python3",
            str(Path(__file__).parent / "evaluate_all_live_trajectories.py"),
            "--bag",
            bag_path,
            "--input-bag",
            bag_path,  # Use segment bag for OR extraction (has route already)
            "--map-path",
            map_path,
            "--output-bag",
            output_bag,
            "--json-output",
            json_output,
            "--time-window",
            str(time_window),
        ]

        result = subprocess.run(cmd)

        if result.returncode != 0:
            print(f"ERROR: Evaluation failed for segment {segment_id}")
            sys.exit(1)

        print(f"✓ Segment {segment_id} complete")
        print(f"  Output: {output_bag}")
        print(f"  JSON: {json_output}")
        print("")

    print(f"✓ All {len(segment_bags)} segments evaluated successfully")


def main():
    parser = argparse.ArgumentParser(description="Auto-discover and evaluate all OR segment bags")
    parser.add_argument("--input-dir", required=True, help="Directory containing or_event_* bags")
    parser.add_argument("--map-path", required=True, help="Path to lanelet2 map file")
    parser.add_argument(
        "--time-window",
        type=float,
        default=5.0,
        help="Evaluation window on each side of OR in seconds (default: 5.0)",
    )

    args = parser.parse_args()

    if not Path(args.input_dir).exists():
        print(f"ERROR: Input directory not found: {args.input_dir}")
        sys.exit(1)

    if not Path(args.map_path).exists():
        print(f"ERROR: Map not found: {args.map_path}")
        sys.exit(1)

    evaluate_all_segments(args.input_dir, args.map_path, args.time_window)


if __name__ == "__main__":
    main()
