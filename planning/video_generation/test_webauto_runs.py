"""Checks WebAuto run boundaries, pagination, and topic selection."""

from datetime import datetime
from datetime import timezone
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from generate_bag_video import CameraSource
from generate_bag_video import filtered_download_topics
from webauto_runs import LogPart
from webauto_runs import MCAP_MAGIC
from webauto_runs import datetime_ns
from webauto_runs import parse_datetime
from webauto_runs import pull_part
from webauto_runs import search_parts
from webauto_runs import select_runs


def part(stem: str, number: int, start: int, end: int) -> LogPart:
    epoch = datetime.fromtimestamp
    return LogPart(
        f"{stem}_{number}",
        f"{stem}_{number}.mcap",
        stem,
        number,
        epoch(start, timezone.utc),
        epoch(end, timezone.utc),
    )


class WebAutoRunsTest(unittest.TestCase):
    def test_range_selects_only_closed_fully_contained_runs(self):
        parts = [
            part("car_earlier_p0900", 0, 90, 95),
            part("car_earlier_p0900", 1, 96, 105),
            part("car_good_p0900", 0, 110, 115),
            part("car_good_p0900", 1, 116, 120),
            part("car_late_p0900", 0, 140, 160),
            part("car_later_p0900", 0, 170, 180),
        ]
        runs = select_runs(
            parts,
            datetime.fromtimestamp(100, timezone.utc),
            datetime.fromtimestamp(150, timezone.utc),
        )
        self.assertEqual([run.stem for run in runs], ["car_good_p0900"])
        self.assertEqual([item.number for item in runs[0].parts], [0, 1])

    def test_missing_next_start_or_numbered_part_defers_run(self):
        start = datetime.fromtimestamp(0, timezone.utc)
        end = datetime.fromtimestamp(100, timezone.utc)
        parts = [
            part("car_a_p0900", 0, 10, 20),
            part("car_a_p0900", 2, 21, 30),
            part("car_b_p0900", 0, 40, 50),
        ]
        self.assertEqual(select_runs(parts, start, end), [])

    def test_other_stream_does_not_close_run(self):
        parts = [part("car_a_p0900", 0, 10, 20), part("car_b_p0800", 0, 30, 40)]
        self.assertEqual(
            select_runs(
                parts,
                datetime.fromtimestamp(0, timezone.utc),
                datetime.fromtimestamp(100, timezone.utc),
            ),
            [],
        )

    def test_search_paginates_and_keeps_source_timestamps(self):
        pages = [
            {
                "log_files": [
                    {
                        "id": "one",
                        "log_file_name": "car_a_p0900_0.mcap",
                        "start_timestamp": "2026-11-01T00:00:00Z",
                        "end_timestamp": "2026-11-01T00:00:10Z",
                    }
                ],
                "next_page_token": "more",
            },
            {
                "log_files": [
                    {
                        "id": "two",
                        "log_file_name": "car_b_p0900_0.mcap",
                        "start_timestamp": "2026-11-01T00:01:00Z",
                        "end_timestamp": "2026-11-01T00:01:10Z",
                    }
                ]
            },
        ]
        search_end = parse_datetime("2026-11-20T00:00:00Z", "to")
        with patch("webauto_runs.run_cli", side_effect=[json.dumps(x) for x in pages]) as cli:
            found = search_parts(
                "project",
                "vehicle",
                parse_datetime("2026-11-01T00:00:00Z", "from"),
                search_end,
            )
        self.assertEqual([item.id for item in found], ["one", "two"])
        self.assertIn("--page-token", cli.call_args_list[1].args[0])
        for call in cli.call_args_list:
            command = call.args[0]
            self.assertEqual(command[command.index("--datetime-to") + 1], search_end.isoformat())

    def test_pull_uses_full_source_bag_and_reuses_matching_cache(self):
        source = part("car_a_p0900", 0, 10, 20)
        commands = []

        def fake_cli(command):
            commands.append(command)
            target = Path(command[command.index("--target-dir") + 1])
            content = target / "filtered-id" / source.name
            content.parent.mkdir(parents=True)
            content.write_bytes(MCAP_MAGIC + b"example")
            return ""

        with tempfile.TemporaryDirectory() as directory:
            with patch("webauto_runs.run_cli", side_effect=fake_cli):
                first = pull_part("project", source, {"/tf", "/camera"}, Path(directory))
                second = pull_part("project", source, {"/tf", "/camera"}, Path(directory))
            self.assertEqual(first, second)
            self.assertTrue(first.is_file())
        self.assertEqual(len(commands), 1)
        self.assertNotIn("--slicing-start-timestamp", commands[0])
        self.assertNotIn("--slicing-end-timestamp", commands[0])
        self.assertEqual(commands[0][commands[0].index("--topics") + 1], "/camera,/tf")

    def test_datetime_ns_does_not_round_through_float(self):
        self.assertEqual(
            datetime_ns(parse_datetime("2026-11-05T10:33:11.123456+09:00", "from")),
            1793842391123456000,
        )

    def test_webauto_variable_fractional_seconds(self):
        self.assertEqual(
            parse_datetime("2026-11-16T06:54:26.99+00:00", "end_timestamp").microsecond,
            990000,
        )
        self.assertEqual(
            parse_datetime("2026-11-16T06:54:26.9Z", "end_timestamp").microsecond,
            900000,
        )
        self.assertEqual(
            parse_datetime("2026-11-16T06:54:26.123456789Z", "end_timestamp").microsecond,
            123456,
        )

    def test_topic_filter_includes_render_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "view.rviz").write_text(
                "Visualization Manager:\n  Displays:\n  - Topic: /rviz_topic\n"
            )
            (root / "plot.yaml").write_text(
                "clearance:\n  optimized_topic: /optimized\n"
                "  reference_topic: /reference\n"
                "event_detection:\n  topic: /enabled\n"
                "  slowdown_filter:\n    operation_mode_topic: /mode\n"
                "    motion_topic: /filter_velocity\n"
            )
            args = SimpleNamespace(
                rviz_config=root / "view.rviz",
                plot_config=root / "plot.yaml",
                vehicle_model="j6_gen2",
                trajectory_topic="/trajectory",
                mppi_trajectory_topic="/optimized",
                start_before_motion_s=1,
                motion_topic="/velocity",
                birdseye=True,
                birdseye_ego_topic="/odometry",
                birdseye_ml_topic="/ml",
            )
            topics = filtered_download_topics(
                args, [CameraSource("one", "Camera", "/image", "/camera_info")]
            )
            self.assertTrue(
                {
                    "/rviz_topic",
                    "/tf",
                    "/tf_static",
                    "/image",
                    "/camera_info",
                    "/trajectory",
                    "/optimized",
                    "/reference",
                    "/enabled",
                    "/velocity",
                    "/mode",
                    "/filter_velocity",
                    "/odometry",
                    "/ml",
                    "/diagnostics",
                    "/map/vector_map_marker",
                    "/perception/object_recognition/tracking/objects",
                }
                <= topics
            )


if __name__ == "__main__":
    unittest.main()
