#!/usr/bin/env python3
"""
Detect OR (Override) events and extract route message from rosbag.

Outputs JSON with OR event timestamps and saves route message for injection into segments.
"""

import argparse
import json
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions

try:
    from autoware_vehicle_msgs.msg import ControlModeReport
except ImportError:
    print("ERROR: Required ROS message types not found.")
    print("Make sure to source Autoware workspace first:")
    print("  source ~/pilot-auto/install/setup.bash")
    sys.exit(1)


def detect_or_events_and_extract_route(bag_path: str, before_sec: float, after_sec: float):
    """
    Detect OR events and extract route message.

    Args:
        bag_path: Input bag directory or file
        before_sec: Seconds before OR to include in segment
        after_sec: Seconds after OR to include in segment

    Returns:
        dict with 'or_events' list and 'route_info' dict
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=str(bag_path), storage_id="")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    or_events = []
    route_msg_data = None
    route_timestamp = None
    prev_mode = None
    or_count = 0

    # Get bag start/end times
    metadata = reader.get_metadata()
    bag_start_ns = metadata.starting_time.nanoseconds
    bag_duration_ns = metadata.duration.nanoseconds
    bag_end_ns = bag_start_ns + bag_duration_ns

    print(f"Scanning bag: {bag_path}")
    print(f"  Duration: {bag_duration_ns / 1e9:.2f}s")
    print(f"  Start: {bag_start_ns}")
    print(f"  End: {bag_end_ns}")
    print("")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        # Detect OR events
        if topic == "/vehicle/status/control_mode":
            try:
                msg = deserialize_message(data, ControlModeReport)
                # AUTONOMOUS (1) → MANUAL (4) = Override
                if prev_mode == 1 and msg.mode == 4:
                    or_count += 1

                    # Calculate segment boundaries
                    seg_start_ns = max(timestamp - int(before_sec * 1e9), bag_start_ns)
                    seg_end_ns = min(timestamp + int(after_sec * 1e9), bag_end_ns)

                    or_events.append(
                        {
                            "event_id": or_count - 1,
                            "or_timestamp_ns": timestamp,
                            "or_timestamp_sec": timestamp / 1e9,
                            "segment_start_ns": seg_start_ns,
                            "segment_end_ns": seg_end_ns,
                            "segment_start_sec": seg_start_ns / 1e9,
                            "segment_end_sec": seg_end_ns / 1e9,
                            "segment_duration_sec": (seg_end_ns - seg_start_ns) / 1e9,
                        }
                    )

                    print(f"OR Event #{or_count} detected at t={timestamp / 1e9:.3f}s")
                    print(
                        f"  Segment: [{seg_start_ns / 1e9:.3f}s, {seg_end_ns / 1e9:.3f}s] "
                        f"(duration: {(seg_end_ns - seg_start_ns) / 1e9:.2f}s)"
                    )

                prev_mode = msg.mode
            except Exception as e:
                print(f"Warning: Failed to deserialize control_mode: {e}")

        # Extract route (first occurrence)
        if topic == "/planning/mission_planning/route" and route_msg_data is None:
            route_msg_data = data
            route_timestamp = timestamp
            print(f"✓ Route message found at t={timestamp / 1e9:.3f}s")

    if not or_events:
        print("ERROR: No OR events found in bag!")
        sys.exit(1)

    if route_msg_data is None:
        print("ERROR: No route message found in bag!")
        print("Route topic: /planning/mission_planning/route")
        sys.exit(1)

    print(f"\nTotal OR events detected: {len(or_events)}")

    return {
        "or_events": or_events,
        "route_info": {
            "original_timestamp_ns": route_timestamp,
            "original_timestamp_sec": route_timestamp / 1e9,
            "data_size_bytes": len(route_msg_data),
        },
        "route_msg_data": route_msg_data,  # Binary data for injection
        "bag_info": {
            "start_ns": bag_start_ns,
            "end_ns": bag_end_ns,
            "duration_sec": bag_duration_ns / 1e9,
        },
    }


def main():
    parser = argparse.ArgumentParser(
        description="Detect OR events and extract route message from rosbag"
    )
    parser.add_argument("--input", "-i", required=True, help="Input bag path")
    parser.add_argument("--output", "-o", required=True, help="Output JSON file for OR events")
    parser.add_argument(
        "--route-output",
        required=True,
        help="Output file to save route message binary data",
    )
    parser.add_argument(
        "--before",
        type=float,
        default=5.0,
        help="Seconds before OR to include (default: 5.0)",
    )
    parser.add_argument(
        "--after",
        type=float,
        default=10.0,
        help="Seconds after OR to include (default: 10.0)",
    )

    args = parser.parse_args()

    # Validate input
    if not Path(args.input).exists():
        print(f"ERROR: Input bag not found: {args.input}")
        sys.exit(1)

    # Detect OR events and extract route
    result = detect_or_events_and_extract_route(args.input, args.before, args.after)

    # Save OR events JSON (without binary route data)
    output_json = {
        "or_events": result["or_events"],
        "route_info": result["route_info"],
        "bag_info": result["bag_info"],
        "parameters": {"before_sec": args.before, "after_sec": args.after},
    }

    with open(args.output, "w") as f:
        json.dump(output_json, f, indent=2)

    print(f"\n✓ OR events saved to: {args.output}")

    # Save route message binary data
    with open(args.route_output, "wb") as f:
        f.write(result["route_msg_data"])

    print(f"✓ Route message saved to: {args.route_output}")
    print(f"  Size: {len(result['route_msg_data'])} bytes")


if __name__ == "__main__":
    main()
