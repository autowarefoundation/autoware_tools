"""Checks for motion-based video starts and event clipping."""

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from generate_bag_video import Bag
from generate_bag_video import Timeline
from generate_bag_video import config_from_yaml
from generate_bag_video import motion_speed


class VideoStartTest(unittest.TestCase):
    def test_vehicle_report_speed_includes_reverse_and_lateral_motion(self):
        message = SimpleNamespace(longitudinal_velocity=-0.3, lateral_velocity=0.4)
        self.assertAlmostEqual(
            motion_speed(message, "autoware_vehicle_msgs/msg/VelocityReport"), 0.5
        )

    def test_explicit_events_are_clipped_to_video_start(self):
        bag = Bag(Path("example.mcap"), 0, 1_000, {"/metric": "std_msgs/msg/Float32"})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plot.yaml"
            path.write_text(
                "metrics:\n  - topic: /metric\nevents:\n"
                "  - {start_ns: 100, end_ns: 200}\n"
                "  - {start_ns: 400, end_ns: 600}\n"
                "  - {start_ns: 700, end_ns: 800}\n"
            )
            config = config_from_yaml(path, Timeline(500, 1_000), bag.topics, [bag])
        self.assertEqual(
            [(event.start, event.end) for event in config.events], [(500, 600), (700, 800)]
        )


if __name__ == "__main__":
    unittest.main()
