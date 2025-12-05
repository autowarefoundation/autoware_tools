#!/usr/bin/env python3
"""
Cut rosbag segment for OR event time window and inject route message.

This ensures each segment is self-contained with route available for DLR.
"""

import argparse
import sys
from pathlib import Path

from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions


def cut_segment_with_route(
    input_bag: str,
    output_bag: str,
    start_ns: int,
    end_ns: int,
    route_msg_data: bytes,
    route_topic: str = "/planning/mission_planning/route",
):
    """
    Cut time segment and inject route message at segment start.

    Args:
        input_bag: Source bag path
        output_bag: Destination bag path
        start_ns: Segment start timestamp (nanoseconds)
        end_ns: Segment end timestamp (nanoseconds)
        route_msg_data: Serialized route message to inject
        route_topic: Route topic name
    """
    print(f"Cutting segment: [{start_ns / 1e9:.3f}s, {end_ns / 1e9:.3f}s]")
    print(f"  Input: {input_bag}")
    print(f"  Output: {output_bag}")

    # Open reader
    reader = SequentialReader()
    reader_storage = StorageOptions(uri=str(input_bag), storage_id="")
    reader_converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(reader_storage, reader_converter)

    # Open writer
    writer = SequentialWriter()
    writer_storage = StorageOptions(uri=str(output_bag), storage_id="mcap")
    writer_converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(writer_storage, writer_converter)

    # Create all topics from input bag
    for topic_metadata in reader.get_all_topics_and_types():
        writer.create_topic(topic_metadata)

    # CRITICAL: Inject route message at segment start
    writer.write(route_topic, route_msg_data, start_ns)
    print(f"  ✓ Injected route at t={start_ns / 1e9:.3f}s")

    # Copy messages in time window
    found_topics = {route_topic}  # Route already added
    message_count = 1  # Route counts
    skipped_route_count = 0

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if start_ns <= timestamp <= end_ns:
            # Skip route if it appears in the segment (we already injected it at start)
            if topic == route_topic:
                skipped_route_count += 1
                continue

            writer.write(topic, data, timestamp)
            message_count += 1
            found_topics.add(topic)

            if message_count % 1000 == 0:
                print(f"  Copied {message_count} messages...", end="\r")

    print(f"  Copied {message_count} messages total")
    if skipped_route_count > 0:
        print(f"  Skipped {skipped_route_count} duplicate route messages")

    # Check for critical topics
    critical_topics = [
        "/planning/mission_planning/route",
        "/localization/kinematic_state",
        "/perception/object_recognition/tracking/objects",
        "/vehicle/status/control_mode",
        "/vehicle/status/turn_indicators_status",
    ]

    missing = []
    for topic in critical_topics:
        if topic not in found_topics:
            missing.append(topic)

    if missing:
        print(f"  WARNING: Segment missing topics: {missing}")

    print(f"  ✓ Segment duration: {(end_ns - start_ns) / 1e9:.2f}s")
    print(f"  ✓ Topics found: {len(found_topics)}")

    return {
        "message_count": message_count,
        "duration_sec": (end_ns - start_ns) / 1e9,
        "topics_found": len(found_topics),
        "has_route": route_topic in found_topics,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Cut rosbag segment with route message injection"
    )
    parser.add_argument("--input", "-i", required=True, help="Input bag path")
    parser.add_argument("--output", "-o", required=True, help="Output bag path")
    parser.add_argument(
        "--start", type=int, required=True, help="Start timestamp (nanoseconds)"
    )
    parser.add_argument(
        "--end", type=int, required=True, help="End timestamp (nanoseconds)"
    )
    parser.add_argument(
        "--route-data", required=True, help="File containing serialized route message"
    )
    parser.add_argument(
        "--route-topic",
        default="/planning/mission_planning/route",
        help="Route topic name",
    )

    args = parser.parse_args()

    # Validate inputs
    if not Path(args.input).exists():
        print(f"ERROR: Input bag not found: {args.input}")
        sys.exit(1)

    if not Path(args.route_data).exists():
        print(f"ERROR: Route data file not found: {args.route_data}")
        sys.exit(1)

    # Remove output if it exists (allow overwriting)
    if Path(args.output).exists():
        import shutil
        shutil.rmtree(args.output)
        print(f"Removed existing output: {args.output}")

    # Load route message data
    with open(args.route_data, "rb") as f:
        route_msg_data = f.read()

    print(f"Loaded route message: {len(route_msg_data)} bytes")

    # Cut segment
    try:
        result = cut_segment_with_route(
            args.input, args.output, args.start, args.end, route_msg_data, args.route_topic
        )
        print("\n✓ Segment cut successfully")
        return 0
    except Exception as e:
        print(f"\nERROR: Failed to cut segment: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
