"""Checks the recording-time strip's state and frame alignment."""

from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from generate_bag_video import API_MODE_TYPE
from generate_bag_video import Bag
from generate_bag_video import NSEC
from generate_bag_video import Timeline
from generate_bag_video import operation_mode_samples
import numpy as np
from timeline_bar import JST
from timeline_bar import MPPI
from timeline_bar import _static_bar
from timeline_bar import mode_spans
from timeline_bar import render_timeline_bar
from timeline_bar import track_x


class FrameCollector:
    def __init__(self):
        self.frames = []
        self.released = False

    def write(self, frame):
        self.frames.append(frame.copy())

    def release(self):
        self.released = True


class TimelineBarTest(unittest.TestCase):
    def test_only_autonomous_with_control_enabled_is_blue(self):
        topic = "/api/operation_mode/state"
        bag = Bag(Path("a.mcap"), 0, 100, {topic: API_MODE_TYPE})
        records = iter(
            [
                (topic, SimpleNamespace(mode=2, is_autoware_control_enabled=True), 10),
                (topic, SimpleNamespace(mode=2, is_autoware_control_enabled=False), 20),
                (topic, SimpleNamespace(mode=1, is_autoware_control_enabled=True), 30),
            ]
        )

        class Reader:
            def __init__(self):
                self.next = next(records, None)

            def has_next(self):
                return self.next is not None

            def read_next(self):
                result = self.next
                self.next = next(records, None)
                return result

        with patch("generate_bag_video.reader_for", return_value=Reader()), patch(
            "generate_bag_video.get_message", return_value=SimpleNamespace(AUTONOMOUS=2)
        ), patch("generate_bag_video.deserialize_message", side_effect=lambda value, _kind: value):
            self.assertEqual(
                operation_mode_samples([bag], topic, bag.topics),
                [(10, True), (20, False), (30, False)],
            )

    def test_mode_is_unknown_before_samples_and_across_bag_gap(self):
        bags = [
            Bag(Path("a.mcap"), 0, 40 * NSEC, {}),
            Bag(Path("b.mcap"), 60 * NSEC, 100 * NSEC, {}),
        ]
        spans = mode_spans(
            bags,
            [(10 * NSEC, True), (30 * NSEC, False), (70 * NSEC, True)],
            Timeline(5 * NSEC, 95 * NSEC),
        )
        self.assertEqual(
            spans,
            [
                (5 * NSEC, 10 * NSEC, None),
                (10 * NSEC, 30 * NSEC, True),
                (30 * NSEC, 40 * NSEC, False),
                (60 * NSEC, 70 * NSEC, None),
                (70 * NSEC, 95 * NSEC, True),
            ],
        )

    def test_mode_carries_across_adjacent_parts_without_new_messages(self):
        bags = [
            Bag(Path("a.mcap"), 0, 40 * NSEC, {}),
            Bag(Path("b.mcap"), 40 * NSEC + NSEC // 1000, 80 * NSEC, {}),
        ]
        spans = mode_spans(bags, [(10 * NSEC, True)], Timeline(0, 80 * NSEC))
        self.assertEqual(spans[-1], (bags[1].start, bags[1].end, True))

    def test_trimmed_start_uses_state_from_earlier_contiguous_part(self):
        bags = [
            Bag(Path("a.mcap"), 0, 40 * NSEC, {}),
            Bag(Path("b.mcap"), 40 * NSEC, 80 * NSEC, {}),
        ]
        spans = mode_spans(bags, [(10 * NSEC, True)], Timeline(50 * NSEC, 80 * NSEC))
        self.assertEqual(spans, [(50 * NSEC, 80 * NSEC, True)])

    def test_raw_activation_tick_and_jst_clock_share_video_frames(self):
        start = 1_762_732_800 * NSEC  # 2025-11-10 00:00:00 UTC
        timeline = Timeline(start, start + NSEC // 10)
        bag = Bag(Path("a.mcap"), timeline.start, timeline.end, {})
        activation = (start + NSEC // 100, start + NSEC // 50)
        spans = mode_spans([bag], [(start, True)], timeline)
        base, left, right = _static_bar(timeline, spans, [activation], 2400)
        x = track_x(activation[0], timeline, left, right)
        self.assertTrue(np.array_equal(base[90, x], MPPI))
        self.assertEqual(datetime.fromtimestamp(start // NSEC, JST).strftime("%H:%M"), "09:00")
        writer = FrameCollector()
        render_timeline_bar(
            [bag],
            timeline,
            [(start, True)],
            [activation],
            Path("unused.mp4"),
            lambda _path, size: writer if size == (2400, 120) else None,
            2400,
        )
        self.assertEqual(len(writer.frames), timeline.count)
        self.assertTrue(writer.released)
        self.assertNotEqual(
            track_x(timeline.stamp(0), timeline, left, right),
            track_x(timeline.stamp(timeline.count - 1), timeline, left, right),
        )


if __name__ == "__main__":
    unittest.main()
