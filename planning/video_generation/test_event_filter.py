"""Behavioral checks for conditional MPPI slowdown."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from generate_bag_video import API_MODE_TYPE
from generate_bag_video import Bag
from generate_bag_video import NSEC
from generate_bag_video import Timeline
from generate_bag_video import config_from_yaml
from generate_bag_video import qualifying_intervals


class FakeReader:
    def __init__(self, records):
        self.records = iter(records)
        self.next_record = next(self.records, None)

    def has_next(self):
        return self.next_record is not None

    def read_next(self):
        record = self.next_record
        self.next_record = next(self.records, None)
        return record


class EventFilterTest(unittest.TestCase):
    def test_movement_must_overlap_raw_activation_and_enabled_control(self):
        intervals = [(10, 20), (30, 40), (50, 60)]
        states = [(12, True), (25, False), (35, True), (55, False)]
        motion = [(9, 1.0), (15, 0.1), (32, 1.0), (36, 0.2), (55, 1.0), (60, 1.0)]
        self.assertEqual(
            qualifying_intervals(intervals, states, motion, 0.1),
            [(10, 20), (30, 40)],
        )

    def test_visuals_keep_rejected_activations_and_margins_follow_filter(self):
        sec = lambda number: number * NSEC
        topics = {
            "/metric": "std_msgs/msg/Float32",
            "/enabled": "std_msgs/msg/Bool",
            "/mode": API_MODE_TYPE,
            "/motion": "autoware_vehicle_msgs/msg/VelocityReport",
        }
        bag = Bag(Path("example.mcap"), sec(0), sec(90), topics)
        records = {
            "/enabled": [
                ("/enabled", SimpleNamespace(data=value), sec(stamp))
                for stamp, value in [
                    (10, True),
                    (20, False),
                    (30, True),
                    (40, False),
                    (50, True),
                    (60, False),
                ]
            ],
            "/mode": [
                (
                    "/mode",
                    SimpleNamespace(mode=mode, is_autoware_control_enabled=enabled),
                    sec(stamp),
                )
                for stamp, mode, enabled in [
                    (25, 1, True),
                    (35, 1, True),
                    (45, 2, False),
                    (54, 2, True),
                ]
            ],
            "/motion": [
                (
                    "/motion",
                    SimpleNamespace(longitudinal_velocity=value, lateral_velocity=0.0),
                    sec(stamp),
                )
                for stamp, value in [(15, 1.0), (32, 1.0), (36, 1.0), (53, 1.0), (55, 0.2)]
            ],
        }

        def reader(_bag, selected):
            entries = sorted(
                (entry for topic in selected for entry in records[topic]),
                key=lambda entry: entry[2],
            )
            return FakeReader(entries)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plot.yaml"
            path.write_text(
                "metrics:\n  - topic: /metric\n"
                "event_detection:\n  topic: /enabled\n"
                "  pre_margin_s: 2\n  post_margin_s: 2\n"
                "  slowdown_filter:\n    operation_mode_topic: /mode\n"
                "    motion_topic: /motion\n"
            )
            with patch("generate_bag_video.reader_for", side_effect=reader), patch(
                "generate_bag_video.deserialize_message", side_effect=lambda message, _cls: message
            ), patch(
                "generate_bag_video.get_message",
                side_effect=lambda kind: SimpleNamespace(AUTONOMOUS=2),
            ):
                config = config_from_yaml(path, Timeline(sec(0), sec(90)), topics, [bag])
        self.assertEqual([(e.start, e.end) for e in config.events], [(sec(48), sec(62))])
        self.assertEqual(
            [(e.start, e.end) for e in config.visual_events],
            [(sec(8), sec(22)), (sec(28), sec(42)), (sec(48), sec(62))],
        )
        self.assertEqual(
            config.raw_activation_intervals,
            [(sec(10), sec(20)), (sec(30), sec(40)), (sec(50), sec(60))],
        )
        self.assertEqual(config.mode_topic, "/mode")

    def test_missing_control_stream_keeps_visuals_without_slowdown(self):
        topics = {"/metric": "std_msgs/msg/Float32", "/enabled": "std_msgs/msg/Bool"}
        bag = Bag(Path("example.mcap"), 0, 100, topics)
        records = [
            ("/enabled", SimpleNamespace(data=True), 10),
            ("/enabled", SimpleNamespace(data=False), 20),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plot.yaml"
            path.write_text(
                "metrics:\n  - topic: /metric\n"
                "event_detection:\n  topic: /enabled\n  slowdown_filter: {}\n"
            )
            with patch("generate_bag_video.reader_for", return_value=FakeReader(records)), patch(
                "generate_bag_video.deserialize_message", side_effect=lambda message, _cls: message
            ), patch("generate_bag_video.get_message", return_value=SimpleNamespace()):
                config = config_from_yaml(path, Timeline(0, 100), topics, [bag])
        self.assertEqual(config.events, [])
        self.assertEqual([(e.start, e.end) for e in config.visual_events], [(10, 20)])


if __name__ == "__main__":
    unittest.main()
