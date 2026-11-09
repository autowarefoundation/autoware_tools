#!/usr/bin/env python3
"""Generate a synchronized RViz, camera, and Float32 plot video from ROS 2 MCAPs.

See plot_config.example.yaml for the plot and event format. Run from a sourced
ROS 2 Humble environment with MoviePy 2 installed.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from decimal import Decimal
import logging
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from urllib.parse import unquote
from urllib.parse import urlparse
from xml.etree import ElementTree

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml

LOG = logging.getLogger("bag_video")
FPS = 30
NSEC = 1_000_000_000
SCREEN = (1920, 1080)
PLOT = (640, 360)


@dataclass(frozen=True)
class Bag:
    path: Path
    start: int
    end: int
    topics: dict[str, str]


@dataclass(frozen=True)
class Event:
    start: int
    end: int
    speed: float


@dataclass(frozen=True)
class Config:
    metrics: list[dict]
    events: list[Event]
    default_speed: float
    plot_window_s: float | None


@dataclass(frozen=True)
class Timeline:
    start: int
    end: int

    @property
    def count(self) -> int:
        # Bag metadata's end is the last message timestamp. Include the first
        # 30 FPS sample at or after that message so it appears in the video.
        return max(1, ((self.end - self.start) * FPS + NSEC - 1) // NSEC + 1)

    def stamp(self, index: int) -> int:
        return self.start + index * NSEC // FPS

    def first_frame(self, stamp: int) -> int:
        return min(self.count, max(0, ((stamp - self.start) * FPS + NSEC - 1) // NSEC))


def validate_ros_environment() -> None:
    """Require the caller's CycloneDDS settings before creating any ROS node."""
    implementation = os.environ.get("RMW_IMPLEMENTATION")
    uri = os.environ.get("CYCLONEDDS_URI")
    if implementation != "rmw_cyclonedds_cpp" or not uri:
        raise RuntimeError(
            "Run from your configured zsh session: RMW_IMPLEMENTATION must be "
            "rmw_cyclonedds_cpp and CYCLONEDDS_URI must be exported"
        )
    parsed = urlparse(uri)
    if parsed.scheme != "file" or parsed.netloc not in ("", "localhost"):
        raise RuntimeError("CYCLONEDDS_URI must reference a local file:// XML config")
    config_path = Path(unquote(parsed.path))
    if not config_path.is_file():
        raise RuntimeError(f"CycloneDDS config does not exist: {config_path}")
    try:
        root = ElementTree.parse(config_path).getroot()
    except ElementTree.ParseError as error:
        raise RuntimeError(f"Invalid CycloneDDS XML: {config_path}") from error
    interfaces = root.findall(".//{*}General/{*}Interfaces/{*}NetworkInterface")
    if not interfaces or any(
        item.get("name") != "lo" or item.get("autodetermine", "false").lower() != "false"
        for item in interfaces
    ):
        raise RuntimeError(
            f"CycloneDDS config must select only the loopback interface (lo): {config_path}"
        )
    LOG.info("ROS communication: %s, %s (loopback only)", implementation, uri)


def read_bags(paths: list[str]) -> list[Bag]:
    result = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if not path.exists() or not (path.is_dir() or path.suffix == ".mcap"):
            raise ValueError(f"Expected an MCAP file or ROS 2 bag directory: {path}")
        meta = rosbag2_py.Info().read_metadata(str(path), "mcap")
        if not meta.message_count:
            raise ValueError(f"Bag has no messages: {path}")
        start = int(meta.starting_time.nanoseconds)
        end = start + int(meta.duration.nanoseconds)
        topics = {
            item.topic_metadata.name: item.topic_metadata.type
            for item in meta.topics_with_message_count
            if item.message_count
        }
        result.append(Bag(path, start, end, topics))
        LOG.info("Bag %s: %d..%d ns", path, start, end)
    for left, right in zip(result, result[1:]):
        if right.start < left.start:
            raise ValueError("--bag-files must be in timestamp order")
        if right.start < left.end:
            raise ValueError(f"Overlapping bags are unsupported: {left.path}, {right.path}")
        if right.start > left.end:
            LOG.info("Gap before %s: %.3f seconds", right.path, (right.start - left.end) / NSEC)
    return result


def rviz_topics(path: Path) -> set[str]:
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"Invalid RViz YAML: {path}")
    result: set[str] = set()

    def visit(node: object, key: str = "") -> None:
        if isinstance(node, dict):
            if key == "Topic" and isinstance(node.get("Value"), str):
                topic = node["Value"].strip()
                if topic.startswith("/"):
                    result.add(topic)
            for child_key, child in node.items():
                visit(child, str(child_key))
        elif isinstance(node, list):
            for child in node:
                visit(child, key)
        elif (
            isinstance(node, str)
            and (key == "Topic" or key.endswith(" Topic"))
            and node.strip().startswith("/")
        ):
            result.add(node.strip())

    visit(document)
    return result


def speed(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def margin_ns(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a nonnegative number of seconds")
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return round(number * NSEC)


def events_from_bool_topic(
    bags: list[Bag], topic: str, timeline: Timeline, pre_ns: int, post_ns: int, event_speed: float
) -> list[Event]:
    cls = get_message("std_msgs/msg/Bool")
    intervals: list[Event] = []
    active_start = None
    for bag in bags:
        if topic not in bag.topics:
            LOG.warning("Event topic absent from %s; retaining its previous state", bag.path)
            continue
        reader = reader_for(bag, [topic])
        while reader.has_next():
            _, payload, stamp = reader.read_next()
            enabled = bool(deserialize_message(payload, cls).data)
            if enabled and active_start is None:
                active_start = stamp
            elif not enabled and active_start is not None:
                start = max(timeline.start, active_start - pre_ns)
                end = min(timeline.end, stamp + post_ns)
                if start < end:
                    intervals.append(Event(start, end, event_speed))
                active_start = None
    if active_start is not None:
        start = max(timeline.start, active_start - pre_ns)
        if start < timeline.end:
            intervals.append(Event(start, timeline.end, event_speed))

    merged: list[Event] = []
    for event in intervals:
        if merged and event.start <= merged[-1].end:
            previous = merged[-1]
            merged[-1] = Event(previous.start, max(previous.end, event.end), event_speed)
        else:
            merged.append(event)
    LOG.info(
        "Detected %d enabled intervals on %s; %d event(s) after margins/merging",
        len(intervals),
        topic,
        len(merged),
    )
    for event in merged:
        LOG.info(
            "Event: %d..%d ns (%.3f..%.3f s from bag start)",
            event.start,
            event.end,
            (event.start - timeline.start) / NSEC,
            (event.end - timeline.start) / NSEC,
        )
    return merged


def config_from_yaml(
    path: Path, timeline: Timeline, types: dict[str, str], bags: list[Bag] | None = None
) -> Config:
    with path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict) or not isinstance(raw.get("metrics"), list) or not raw["metrics"]:
        raise ValueError("Plot YAML needs a nonempty 'metrics' list")
    metrics = []
    seen = set()
    for item in raw["metrics"]:
        if not isinstance(item, dict) or not isinstance(item.get("topic"), str):
            raise ValueError("Each metric needs a topic string")
        topic = item["topic"]
        if topic in seen:
            raise ValueError(f"Duplicate metric topic: {topic}")
        seen.add(topic)
        kind = types.get(topic)
        if kind is None:
            LOG.warning("Plot topic missing from all bags: %s", topic)
            continue
        if kind not in ("std_msgs/msg/Float32", "autoware_internal_debug_msgs/msg/Float64Stamped"):
            raise ValueError(f"{topic} is {kind}; supported metrics are Float32 and Float64Stamped")
        metrics.append(
            {"topic": topic, "label": str(item.get("label", topic)), "color": item.get("color")}
        )
    if not metrics:
        raise ValueError("No configured metric topics exist in the bags")
    events = []
    if not isinstance(raw.get("events", []), list):
        raise ValueError("'events' must be a list")
    for item in raw.get("events", []):
        if not isinstance(item, dict):
            raise ValueError("Each event must be a mapping")
        start, end = item.get("start_ns"), item.get("end_ns")
        if any(isinstance(value, bool) or not isinstance(value, int) for value in (start, end)):
            raise ValueError("Event start_ns and end_ns must be integer nanoseconds")
        if not timeline.start <= start < end <= timeline.end:
            raise ValueError(f"Event [{start}, {end}) is outside the bag timeline")
        events.append(Event(start, end, speed(item.get("speed", 0.5), "event speed")))
    events.sort(key=lambda event: event.start)
    for left, right in zip(events, events[1:]):
        if right.start < left.end:
            raise ValueError("Event intervals must not overlap")
    detection = raw.get("event_detection")
    if detection is not None:
        if events:
            raise ValueError("Use either 'events' or 'event_detection', not both")
        if not isinstance(detection, dict) or not isinstance(detection.get("topic"), str):
            raise ValueError("event_detection needs a Bool topic string")
        topic = detection["topic"]
        if types.get(topic) != "std_msgs/msg/Bool":
            raise ValueError(
                f"Event topic {topic} must be std_msgs/msg/Bool; got {types.get(topic)}"
            )
        if bags is None:
            raise ValueError("Bag inputs are required for event_detection")
        events = events_from_bool_topic(
            bags,
            topic,
            timeline,
            margin_ns(detection.get("pre_margin_s", 0), "pre_margin_s"),
            margin_ns(detection.get("post_margin_s", 0), "post_margin_s"),
            speed(detection.get("speed", 0.5), "event speed"),
        )
    plot_window_s = raw.get("plot_window_s")
    if plot_window_s is not None:
        plot_window_s = speed(plot_window_s, "plot_window_s")
    return Config(
        metrics, events, speed(raw.get("default_speed", 2.0), "default_speed"), plot_window_s
    )


def reader_for(bag: Bag, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def writer_for(path: Path, size: tuple[int, int]):
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, size)
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create MP4: {path}")
    return writer


def decode_camera(message, kind: str) -> np.ndarray:
    if kind == "sensor_msgs/msg/CompressedImage":
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
    elif kind == "sensor_msgs/msg/Image":
        from cv_bridge import CvBridge

        image = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
    else:
        raise ValueError(f"Unsupported camera type: {kind}")
    if image is None or not image.size:
        raise ValueError("Could not decode camera image")
    return np.ascontiguousarray(image)


def camera_video(bags: list[Bag], topic: str, kind: str, path: Path, timeline: Timeline) -> None:
    writer = None
    last = None
    blank = None
    next_frame = 0
    cls = get_message(kind)
    try:
        for bag in bags:
            if topic not in bag.topics:
                LOG.warning("Camera topic absent from %s; holding the previous frame", bag.path)
                continue
            reader = reader_for(bag, [topic])
            while reader.has_next():
                _, payload, stamp = reader.read_next()
                image = decode_camera(deserialize_message(payload, cls), kind)
                if writer is None:
                    height, width = image.shape[:2]
                    writer = writer_for(path, (width, height))
                    blank = np.zeros_like(image)
                elif image.shape != blank.shape:
                    image = cv2.resize(image, (blank.shape[1], blank.shape[0]))
                while next_frame < timeline.count and timeline.stamp(next_frame) < stamp:
                    writer.write(last if last is not None else blank)
                    next_frame += 1
                last = image
        if writer is None:
            raise ValueError(f"No decodable frames on {topic}")
        while next_frame < timeline.count:
            writer.write(last)
            next_frame += 1
    finally:
        if writer is not None:
            writer.release()
    LOG.info("Camera: %d frames", next_frame)


def metric_samples(
    bags: list[Bag], metrics: list[dict]
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    topics = {item["topic"] for item in metrics}
    rows = {topic: [] for topic in topics}
    classes = {}
    for item in metrics:
        topic = item["topic"]
        kind = next(bag.topics[topic] for bag in bags if topic in bag.topics)
        classes[topic] = get_message(kind)
    for bag in bags:
        active = sorted(topics & bag.topics.keys())
        if not active:
            continue
        reader = reader_for(bag, active)
        while reader.has_next():
            topic, payload, stamp = reader.read_next()
            value = float(deserialize_message(payload, classes[topic]).data)
            if math.isfinite(value):
                rows[topic].append((stamp, value))
    output = {}
    for topic, series in rows.items():
        series.sort()
        output[topic] = (
            np.array([x[0] for x in series], dtype=np.int64),
            np.array([x[1] for x in series], dtype=np.float64),
        )
        if not series:
            LOG.warning("No finite metric values for %s", topic)
    return output


def plot_video(bags: list[Bag], config: Config, path: Path, timeline: Timeline) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    data = metric_samples(bags, config.metrics)
    figure = Figure(figsize=(PLOT[0] / 100, PLOT[1] / 100), dpi=100, facecolor="#101820")
    canvas = FigureCanvasAgg(figure)
    curves = []
    for index, metric in enumerate(config.metrics):
        axis = figure.add_subplot(len(config.metrics), 1, index + 1)
        axis.set_facecolor("#152431")
        axis.tick_params(colors="white", labelsize=7)
        axis.set_ylabel(metric["label"], color="white", fontsize=8)
        axis.set_xlim(
            0, config.plot_window_s or max((timeline.end - timeline.start) / NSEC, 1 / FPS)
        )
        stamps, values = data[metric["topic"]]
        if len(values):
            low, high = float(values.min()), float(values.max())
            pad = max((high - low) * 0.08, abs(high) * 0.01, 0.1)
            axis.set_ylim(low - pad, high + pad)
        else:
            axis.set_ylim(-1, 1)
        (line,) = axis.plot([], [], color=metric["color"] or f"C{index % 10}", linewidth=1.5)
        cursor = axis.axvline(0, color="white", alpha=0.5, linewidth=0.8)
        curves.append((axis, line, cursor, stamps, (stamps - timeline.start) / NSEC, values))
    axis.set_xlabel("Seconds from bag start", color="white", fontsize=8)
    figure.subplots_adjust(left=0.16, right=0.98, top=0.97, bottom=0.13, hspace=0.32)
    writer = writer_for(path, PLOT)
    try:
        for index in range(timeline.count):
            now = timeline.stamp(index)
            elapsed = (now - timeline.start) / NSEC
            for axis, line, cursor, stamps, seconds, values in curves:
                count = int(np.searchsorted(stamps, now, side="right"))
                line.set_data(seconds[:count], values[:count])
                cursor.set_xdata([elapsed, elapsed])
                if config.plot_window_s is not None:
                    right = max(elapsed, config.plot_window_s)
                    axis.set_xlim(right - config.plot_window_s, right)
            canvas.draw()
            writer.write(cv2.cvtColor(np.asarray(canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR))
    finally:
        writer.release()
        figure.clear()
    LOG.info("Plot: %d frames", timeline.count)


class ClockTrace:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy
        from rosgraph_msgs.msg import Clock

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("bag_video_clock_trace")
        self.samples: list[tuple[int, int]] = []
        self.lock = threading.Lock()

        def on_clock(message: Clock):
            stamp = message.clock.sec * NSEC + message.clock.nanosec
            with self.lock:
                self.samples.append((time.time_ns(), stamp))

        self.node.create_subscription(
            Clock,
            "/clock",
            on_clock,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def close(self) -> list[tuple[int, int]]:
        self.rclpy.shutdown()
        self.thread.join(timeout=5)
        self.node.destroy_node()
        with self.lock:
            return list(self.samples)


def stop(process, interrupt=False):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT if interrupt else signal.SIGTERM)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def alive(process, name):
    if process.poll() is not None:
        raise RuntimeError(f"{name} exited early (status {process.returncode})")


def fit_rviz_window(pid: int, env: dict[str, str], config_path: Path) -> None:
    """Apply the saved window geometry and verify it fits the captured screen."""
    with config_path.open(encoding="utf-8") as stream:
        geometry = yaml.safe_load(stream).get("Window Geometry", {})
    try:
        x, y = int(geometry["X"]), int(geometry["Y"])
        width, height = int(geometry["Width"]), int(geometry["Height"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"RViz config has no valid Window Geometry: {config_path}") from error
    if (
        x < 0
        or y < 0
        or width <= 0
        or height <= 0
        or x + width > SCREEN[0]
        or y + height > SCREEN[1]
    ):
        raise ValueError(
            f"RViz window ({x}, {y}, {width}, {height}) does not fit "
            f"the {SCREEN[0]}x{SCREEN[1]} capture screen"
        )

    window_ids = []
    for _ in range(100):
        result = subprocess.run(
            ["xdotool", "search", "--onlyvisible", "--pid", str(pid)],
            env=env,
            text=True,
            capture_output=True,
        )
        window_ids = result.stdout.split()
        if window_ids:
            break
        time.sleep(0.1)
    if not window_ids:
        raise RuntimeError(f"Could not find the RViz window for PID {pid}")

    def window_geometry(window_id: str) -> dict[str, int]:
        result = subprocess.run(
            ["xdotool", "getwindowgeometry", "--shell", window_id],
            env=env,
            check=True,
            text=True,
            capture_output=True,
        )
        return {
            key: int(value)
            for line in result.stdout.splitlines()
            if "=" in line
            for key, value in [line.split("=", 1)]
            if key in {"X", "Y", "WIDTH", "HEIGHT"}
        }

    def window_area(window_id: str) -> int:
        current = window_geometry(window_id)
        return current["WIDTH"] * current["HEIGHT"]

    window_id = max(window_ids, key=window_area)
    subprocess.run(
        ["xdotool", "windowsize", window_id, str(width), str(height)],
        env=env,
        check=True,
    )
    subprocess.run(
        ["xdotool", "windowmove", window_id, str(x), str(y)],
        env=env,
        check=True,
    )
    for _ in range(50):
        actual = window_geometry(window_id)
        if actual == {"X": x, "Y": y, "WIDTH": width, "HEIGHT": height}:
            LOG.info("RViz window fits capture screen: %s", actual)
            return
        time.sleep(0.1)
    raise RuntimeError(f"RViz window geometry is {actual}, expected {(x, y, width, height)}")


def capture_rviz(bags: list[Bag], rviz_config: Path, topics: list[str], path: Path):
    display = next(
        (
            f":{n}"
            for n in range(100, 400)
            if not Path(f"/tmp/.X11-unix/X{n}").exists() and not Path(f"/tmp/.X{n}-lock").exists()
        ),
        None,
    )
    if display is None:
        raise RuntimeError("No free Xvfb display")
    env = os.environ.copy()
    env.update(DISPLAY=display, QT_QPA_PLATFORM="xcb", LIBGL_ALWAYS_SOFTWARE="1")
    # Importing pip OpenCV sets these to its bundled Qt plugins, which RViz
    # cannot load. Let RViz use the system Qt plugin search path instead.
    env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    env.pop("QT_QPA_FONTDIR", None)
    xvfb = rviz = ffmpeg = clock = None
    try:
        xvfb = subprocess.Popen(
            ["Xvfb", display, "-screen", "0", f"{SCREEN[0]}x{SCREEN[1]}x24", "-nolisten", "tcp"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        socket = Path(f"/tmp/.X11-unix/X{display[1:]}")
        for _ in range(100):
            if socket.exists():
                break
            alive(xvfb, "Xvfb")
            time.sleep(0.1)
        else:
            raise RuntimeError("Xvfb did not create its display socket")
        rviz_log = path.with_suffix(".rviz.log")
        rviz_log_stream = rviz_log.open("w", encoding="utf-8")
        rviz = subprocess.Popen(
            ["rviz2", "-d", str(rviz_config), "--ros-args", "-p", "use_sim_time:=true"],
            env=env,
            stdout=rviz_log_stream,
            stderr=subprocess.STDOUT,
        )
        rviz_log_stream.close()
        time.sleep(4)
        if rviz.poll() is not None:
            detail = rviz_log.read_text(encoding="utf-8", errors="replace")[-2000:]
            raise RuntimeError(f"rviz2 exited early (status {rviz.returncode}): {detail}")
        fit_rviz_window(rviz.pid, env, rviz_config)
        clock = ClockTrace()
        capture_start = time.time_ns()
        ffmpeg_log = path.with_suffix(".ffmpeg.log")
        ffmpeg_log_stream = ffmpeg_log.open("w", encoding="utf-8")
        ffmpeg = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "warning",
                "-y",
                "-f",
                "x11grab",
                "-framerate",
                str(FPS),
                "-video_size",
                f"{SCREEN[0]}x{SCREEN[1]}",
                "-use_wallclock_as_timestamps",
                "1",
                "-i",
                display,
                "-vsync",
                "0",
                "-copyts",
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-pix_fmt",
                "yuv420p",
                str(path),
            ],
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=ffmpeg_log_stream,
        )
        ffmpeg_log_stream.close()
        time.sleep(1)
        if ffmpeg.poll() is not None:
            detail = ffmpeg_log.read_text(encoding="utf-8", errors="replace")[-2000:]
            raise RuntimeError(
                f"ffmpeg capture exited early (status {ffmpeg.returncode}): {detail}"
            )
        for index, bag in enumerate(bags):
            # ROS 2 Humble's player accepts exactly one bag_path per invocation.
            command = [
                "ros2",
                "bag",
                "play",
                str(bag.path),
                "--clock",
                "100",
                "--disable-keyboard-controls",
                "--topics",
                *topics,
            ]
            LOG.info("Playing %s (%d/%d)", bag.path, index + 1, len(bags))
            subprocess.run(command, env=env, check=True)
            alive(rviz, "rviz2")
            alive(ffmpeg, "ffmpeg capture")
            if index + 1 < len(bags):
                gap = bags[index + 1].start - bag.end
                if gap > 0:
                    time.sleep(gap / NSEC)
        time.sleep(0.25)
    finally:
        stop(ffmpeg, interrupt=True)
        samples = clock.close() if clock else []
        stop(rviz)
        stop(xvfb)
    if not samples or not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError("RViz capture or /clock trace is empty")
    return samples, capture_start


def capture_pts(path: Path, fallback_start: int) -> list[int]:
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "frame=best_effort_timestamp_time",
            "-of",
            "csv=p=0",
            str(path),
        ],
        check=True,
        text=True,
        capture_output=True,
    )
    stamps = [
        int(Decimal(line.strip().split(",")[0]) * NSEC)
        for line in result.stdout.splitlines()
        if line.strip() and not line.startswith("N/A")
    ]
    if not stamps:
        raise RuntimeError("No frame timestamps in RViz capture")
    if stamps[0] < 1_000_000_000_000_000_000:
        LOG.warning("Capture PTS were normalized; using capture launch time for alignment")
        offset = fallback_start - stamps[0]
        stamps = [value + offset for value in stamps]
    return stamps


def align_rviz(
    capture: Path,
    output: Path,
    samples: list[tuple[int, int]],
    timeline: Timeline,
    capture_start: int,
) -> None:
    pts = capture_pts(capture, capture_start)
    samples.sort()
    walls = [item[0] for item in samples]
    ros = [item[1] for item in samples]
    mapped = []
    for stamp in pts:
        pos = bisect_right(walls, stamp)
        if pos == 0:
            mapped.append(ros[0])
        elif pos == len(walls):
            mapped.append(ros[-1])
        else:
            left, right = pos - 1, pos
            fraction = (stamp - walls[left]) / (walls[right] - walls[left])
            mapped.append(ros[left] + int((ros[right] - ros[left]) * fraction))
    if mapped[-1] < timeline.end - NSEC // FPS:
        LOG.warning("RViz capture ends %.3f s early", (timeline.end - mapped[-1]) / NSEC)
    source = cv2.VideoCapture(str(capture))
    if not source.isOpened():
        raise RuntimeError(f"Could not open capture: {capture}")
    writer = writer_for(output, SCREEN)
    current = -1
    image = None
    try:
        for index in range(timeline.count):
            wanted = max(0, bisect_right(mapped, timeline.stamp(index)) - 1)
            while current < wanted:
                ok, image = source.read()
                if not ok:
                    raise RuntimeError("Capture has fewer frames than ffprobe reported")
                current += 1
            writer.write(image)
    finally:
        source.release()
        writer.release()
    LOG.info("RViz: %d aligned frames", timeline.count)


def video_frames(path: Path) -> int:
    video = cv2.VideoCapture(str(path))
    if not video.isOpened():
        raise RuntimeError(f"Could not open video: {path}")
    try:
        return int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    finally:
        video.release()


def compose(
    rviz_path: Path,
    camera_path: Path,
    plot_path: Path,
    final_path: Path,
    config: Config,
    timeline: Timeline,
) -> None:
    try:
        from moviepy import CompositeVideoClip
        from moviepy import VideoFileClip
        from moviepy import concatenate_videoclips
        from moviepy import vfx
    except ImportError as error:
        raise RuntimeError("MoviePy 2 is required: pip install 'moviepy>=2,<3'") from error
    # ffmpeg may report MP4 duration rounded to centiseconds (e.g. 0.63 for
    # 19 frames at 30 FPS). The verified frame count is our duration authority.
    asset_duration = timeline.count / FPS
    rviz, camera, plot = (
        VideoFileClip(str(path)).with_duration(asset_duration)
        for path in (rviz_path, camera_path, plot_path)
    )
    segments = []
    try:
        width, height = rviz.size
        margin = 24
        overlay_width = min(480, width // 3)
        camera = camera.resized(width=overlay_width)
        plot = plot.resized(width=overlay_width)
        if camera.h + plot.h + 3 * margin > height:
            raise ValueError("RViz canvas too short for overlays")
        boundaries = []
        cursor = 0
        for event in config.events:
            first, last = timeline.first_frame(event.start), timeline.first_frame(event.end)
            if first < cursor:
                raise ValueError("Event intervals overlap on the 30 FPS grid")
            if first > cursor:
                boundaries.append((cursor, first, config.default_speed, False))
            if last > first:
                boundaries.append((first, last, event.speed, True))
            else:
                LOG.warning("Event shorter than one frame omitted: %d..%d", event.start, event.end)
            cursor = last
        if cursor < timeline.count:
            boundaries.append((cursor, timeline.count, config.default_speed, False))
        for first, last, multiplier, show_plot in boundaries:
            begin, end = first / FPS, last / FPS
            layers = [
                rviz.subclipped(begin, end),
                camera.subclipped(begin, end).with_position(
                    (width - overlay_width - margin, margin)
                ),
            ]
            if show_plot:
                layers.append(
                    plot.subclipped(begin, end).with_position(
                        (width - overlay_width - margin, height - plot.h - margin)
                    )
                )
            segment = CompositeVideoClip(layers, size=(width, height))
            segments.append(segment.with_effects([vfx.MultiplySpeed(multiplier)]))
        if not segments:
            raise RuntimeError("No video segments were produced")
        final = concatenate_videoclips(segments, method="chain")
        try:
            final.write_videofile(
                str(final_path), fps=FPS, codec="libx264", audio=False, preset="fast"
            )
        finally:
            final.close()
    finally:
        for segment in segments:
            segment.close()
        rviz.close()
        camera.close()
        plot.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-files", nargs="+", required=True)
    parser.add_argument("--rviz-config", type=Path, required=True)
    parser.add_argument("--camera-topic", required=True)
    parser.add_argument("--plot-config", type=Path, required=True)
    parser.add_argument("--intermediate-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    validate_ros_environment()
    for path in (args.rviz_config, args.plot_config):
        if not path.is_file():
            parser.error(f"File does not exist: {path}")
    bags = read_bags(args.bag_files)
    timeline = Timeline(bags[0].start, bags[-1].end)
    kinds = {}
    for bag in bags:
        for topic, kind in bag.topics.items():
            if topic in kinds and kinds[topic] != kind:
                raise ValueError(f"Topic type changes across bags: {topic}")
            kinds[topic] = kind
    kind = kinds.get(args.camera_topic)
    if kind not in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}:
        raise ValueError(
            f"Camera topic {args.camera_topic} has unsupported or missing type: {kind}"
        )
    requested = rviz_topics(args.rviz_config)
    for topic in sorted(requested - kinds.keys()):
        LOG.warning("RViz topic missing from bags: %s", topic)
    play_topics = sorted((requested | {args.camera_topic, "/tf", "/tf_static"}) & kinds.keys())
    LOG.info("RViz playback topics (%d): %s", len(play_topics), ", ".join(play_topics))
    config = config_from_yaml(args.plot_config, timeline, kinds, bags)
    args.intermediate_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    camera_path = args.intermediate_dir / "camera_1x.mp4"
    plot_path = args.intermediate_dir / "plot_1x.mp4"
    rviz_path = args.intermediate_dir / "rviz_1x.mp4"
    final_path = args.output_dir / "final_video.mp4"
    camera_video(bags, args.camera_topic, kind, camera_path, timeline)
    plot_video(bags, config, plot_path, timeline)
    with tempfile.TemporaryDirectory(prefix="bag_video_rviz_") as directory:
        capture = Path(directory) / "rviz_capture.mkv"
        trace, capture_start = capture_rviz(bags, args.rviz_config.resolve(), play_topics, capture)
        align_rviz(capture, rviz_path, trace, timeline, capture_start)
    for path in (camera_path, plot_path, rviz_path):
        count = video_frames(path)
        if count != timeline.count:
            raise RuntimeError(f"{path} has {count} frames, expected {timeline.count}")
    compose(rviz_path, camera_path, plot_path, final_path, config, timeline)
    LOG.info("Final video: %s", final_path)


if __name__ == "__main__":
    main()
