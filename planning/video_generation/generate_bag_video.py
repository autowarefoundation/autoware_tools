#!/usr/bin/env python3
"""Generate a synchronized RViz, camera, and metric plot video from ROS 2 MCAPs.

See plot_config.example.yaml for the plot and event format. Run from a sourced
ROS 2 Humble environment with MoviePy 2 installed.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from dataclasses import field
from decimal import Decimal
import logging
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from urllib.parse import unquote
from urllib.parse import urlparse
from xml.etree import ElementTree

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
from tf2_ros import TransformException
import yaml

LOG = logging.getLogger("bag_video")
FPS = 30
NSEC = 1_000_000_000
SCREEN = (1440, 1080)
SIDEBAR_WIDTH = 960
PLOT = (960, 450)
MAIN_RIBBON_COLOR = (235, 210, 40)  # BGR: cyan
MPPI_RIBBON_COLOR = (200, 45, 240)  # BGR: magenta
MPPI_MAX_AGE_NS = NSEC


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
    clearance: dict | None = None


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


@dataclass(frozen=True)
class TrajectoryRibbon:
    record_stamp: int
    frame_id: str
    vertices: np.ndarray  # (point count, left/right, xyz)


@dataclass
class CameraOverlay:
    tf: Buffer
    infos: list[tuple[int, object]]
    info_stamps: list[int]
    trajectories: dict[str, list[TrajectoryRibbon]]
    trajectory_stamps: dict[str, list[int]]
    main_topic: str
    mppi_topic: str | None
    events: list[Event]
    alpha: float
    warnings: set[str] = field(default_factory=set)


@dataclass(frozen=True)
class CameraSource:
    name: str
    label: str
    image_topic: str
    info_topic: str | None


def camera_sources(path: Path) -> list[CameraSource]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get("cameras"), list):
        raise ValueError("Camera config must contain a cameras list")
    entries = data["cameras"]
    if not entries:
        raise ValueError("Camera config must contain at least one camera")
    sources = []
    for entry in entries:
        if not isinstance(entry, dict) or any(
            not isinstance(entry.get(key), str) or not entry[key].strip()
            for key in ("name", "label", "image_topic", "info_topic")
        ):
            raise ValueError("Each camera needs nonempty name, label, image_topic, info_topic")
        if not entry["name"].replace("_", "").isalnum():
            raise ValueError("Camera names may contain only letters, digits, and underscores")
        sources.append(
            CameraSource(entry["name"], entry["label"], entry["image_topic"], entry["info_topic"])
        )
    for key in ("name", "image_topic", "info_topic"):
        values = [getattr(source, key) for source in sources]
        if len(values) != len(set(values)):
            raise ValueError(f"Camera {key} values must be unique")
    return sources


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


MOTION_TOPIC = "/vehicle/status/velocity_status"
MOTION_MESSAGE_TYPES = {
    "autoware_vehicle_msgs/msg/VelocityReport",
    "nav_msgs/msg/Odometry",
    "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/msg/TwistWithCovarianceStamped",
}


def motion_speed(message, kind: str) -> float:
    if kind == "autoware_vehicle_msgs/msg/VelocityReport":
        return math.hypot(message.longitudinal_velocity, message.lateral_velocity)
    if kind == "nav_msgs/msg/Odometry":
        linear = message.twist.twist.linear
    elif kind == "geometry_msgs/msg/TwistWithCovarianceStamped":
        linear = message.twist.twist.linear
    else:
        linear = message.twist.linear
    return math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)


def first_motion_stamp(
    bags: list[Bag], topic: str = MOTION_TOPIC, threshold_mps: float = 0.1
) -> int:
    kinds = {bag.topics[topic] for bag in bags if topic in bag.topics}
    if len(kinds) != 1 or next(iter(kinds), None) not in MOTION_MESSAGE_TYPES:
        raise ValueError(
            f"Motion topic {topic} must have one of {sorted(MOTION_MESSAGE_TYPES)} "
            f"across bags; found {sorted(kinds)}"
        )
    kind = next(iter(kinds))
    cls = get_message(kind)
    for bag in bags:
        if topic not in bag.topics:
            LOG.warning("Motion topic absent from %s", bag.path)
            continue
        reader = reader_for(bag, [topic])
        while reader.has_next():
            _, payload, stamp = reader.read_next()
            value = motion_speed(deserialize_message(payload, cls), kind)
            if math.isfinite(value) and value >= threshold_mps:
                LOG.info("First ego motion: %d ns (%.3f m/s) on %s", stamp, value, topic)
                return stamp
    raise ValueError(f"No ego motion at or above {threshold_mps:g} m/s on {topic}")


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
            "Event: %d..%d ns (%.3f..%.3f s from video start)",
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
    if not isinstance(raw, dict):
        raise ValueError("Plot YAML must be a mapping")
    clearance = raw.get("clearance")
    if clearance is not None:
        if not isinstance(clearance, dict):
            raise ValueError("'clearance' must be a mapping")
        clearance = {
            "optimized_topic": clearance.get("optimized_topic"),
            "reference_topic": clearance.get("reference_topic"),
            "live_ml_topic": clearance.get("live_ml_topic"),
            "ml_road_border_threshold_m": clearance.get("ml_road_border_threshold_m", 1.5),
            "objects_topic": clearance.get(
                "objects_topic", "/perception/object_recognition/tracking/objects"
            ),
            "diagnostics_topic": clearance.get("diagnostics_topic", "/diagnostics"),
            "road_border_markers_topic": clearance.get(
                "road_border_markers_topic", "/map/vector_map_marker"
            ),
        }
        expected = {
            "optimized_topic": "autoware_planning_msgs/msg/Trajectory",
            "reference_topic": "autoware_planning_msgs/msg/Trajectory",
            "objects_topic": "autoware_perception_msgs/msg/TrackedObjects",
            "diagnostics_topic": "diagnostic_msgs/msg/DiagnosticArray",
            "road_border_markers_topic": "visualization_msgs/msg/MarkerArray",
        }
        if clearance["live_ml_topic"] is not None:
            expected["live_ml_topic"] = "autoware_planning_msgs/msg/Trajectory"
        for key, kind in expected.items():
            topic = clearance[key]
            if not isinstance(topic, str) or types.get(topic) != kind:
                raise ValueError(
                    f"clearance.{key} requires a {kind} topic; got {topic}: {types.get(topic)}"
                )
        if len({clearance[key] for key in expected}) != len(expected):
            raise ValueError("Clearance topics must be distinct")
        clearance["ml_road_border_threshold_m"] = speed(
            clearance["ml_road_border_threshold_m"], "clearance.ml_road_border_threshold_m"
        )
    raw_metrics = raw.get("metrics", [])
    if not isinstance(raw_metrics, list) or (not raw_metrics and clearance is None):
        raise ValueError("Plot YAML needs 'clearance' or a nonempty 'metrics' list")
    if raw_metrics and clearance is not None:
        raise ValueError("Use either scalar 'metrics' or 'clearance' in one plot YAML")
    metrics = []
    seen = set()
    for item in raw_metrics:
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
    if raw_metrics and not metrics:
        raise ValueError("No configured metric topics exist in the bags")
    events = []
    raw_events = raw.get("events", [])
    if not isinstance(raw_events, list):
        raise ValueError("'events' must be a list")
    if raw_events and raw.get("event_detection") is not None:
        raise ValueError("Use either 'events' or 'event_detection', not both")
    bag_start = bags[0].start if bags else timeline.start
    for item in raw_events:
        if not isinstance(item, dict):
            raise ValueError("Each event must be a mapping")
        start, end = item.get("start_ns"), item.get("end_ns")
        if any(isinstance(value, bool) or not isinstance(value, int) for value in (start, end)):
            raise ValueError("Event start_ns and end_ns must be integer nanoseconds")
        if not bag_start <= start < end <= timeline.end:
            raise ValueError(f"Event [{start}, {end}) is outside the bag timeline")
        event_speed = speed(item.get("speed", 0.5), "event speed")
        if end > timeline.start:
            events.append(Event(max(start, timeline.start), end, event_speed))
    events.sort(key=lambda event: event.start)
    for left, right in zip(events, events[1:]):
        if right.start < left.end:
            raise ValueError("Event intervals must not overlap")
    detection = raw.get("event_detection")
    if detection is not None:
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
        metrics,
        events,
        speed(raw.get("default_speed", 2.0), "default_speed"),
        plot_window_s,
        clearance,
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


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError("Zero-length orientation quaternion")
    scale = 2.0 / norm
    return np.array(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def ribbon_vertices(message, kind: str, width_m: float) -> np.ndarray:
    items = message.poses if kind == "nav_msgs/msg/Path" else message.points
    poses = [item.pose for item in items]
    vertices = np.empty((len(poses), 2, 3), dtype=np.float64)
    for index, pose in enumerate(poses):
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        offset = width_m / 2 * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        center = np.array([pose.position.x, pose.position.y, pose.position.z])
        vertices[index, 0] = center + offset
        vertices[index, 1] = center - offset
    return vertices


def load_camera_overlays(
    bags: list[Bag],
    timeline: Timeline,
    kinds: dict[str, str],
    camera_info_topics: list[str],
    main_topic: str,
    mppi_topic: str | None,
    events: list[Event],
    width_m: float,
    mppi_width_m: float,
    alpha: float,
) -> dict[str, CameraOverlay]:
    for topic, expected in (
        *((topic, "sensor_msgs/msg/CameraInfo") for topic in camera_info_topics),
        ("/tf", "tf2_msgs/msg/TFMessage"),
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
    ):
        if kinds.get(topic) != expected:
            raise ValueError(
                f"Overlay requires {topic} with type {expected}; found {kinds.get(topic)}"
            )
    trajectory_topics = [main_topic] + ([mppi_topic] if mppi_topic else [])
    supported = {"nav_msgs/msg/Path", "autoware_planning_msgs/msg/Trajectory"}
    for topic in trajectory_topics:
        if kinds.get(topic) not in supported:
            raise ValueError(f"Unsupported or missing trajectory topic {topic}: {kinds.get(topic)}")
    classes = {topic: get_message(kinds[topic]) for topic in trajectory_topics}
    for topic in camera_info_topics:
        classes[topic] = get_message("sensor_msgs/msg/CameraInfo")
    classes["/tf"] = classes["/tf_static"] = get_message("tf2_msgs/msg/TFMessage")
    buffer = Buffer(cache_time=Duration(nanoseconds=timeline.end - timeline.start + 60 * NSEC))
    infos: dict[str, list[tuple[int, object]]] = {topic: [] for topic in camera_info_topics}
    invalid_infos: dict[str, int] = {topic: 0 for topic in camera_info_topics}
    trajectories = {topic: [] for topic in trajectory_topics}
    for bag in bags:
        active = sorted(set(classes) & bag.topics.keys())
        if not active:
            continue
        reader = reader_for(bag, active)
        while reader.has_next():
            topic, payload, stamp = reader.read_next()
            message = deserialize_message(payload, classes[topic])
            if topic == "/tf":
                for transform in message.transforms:
                    buffer.set_transform(transform, "bag_video")
            elif topic == "/tf_static":
                for transform in message.transforms:
                    buffer.set_transform_static(transform, "bag_video")
            elif topic in infos:
                if (
                    message.distortion_model not in {"plumb_bob", "rational_polynomial"}
                    or not message.k[0]
                ):
                    invalid_infos[topic] += 1
                else:
                    infos[topic].append((stamp, message))
            else:
                ribbon_width = mppi_width_m if topic == mppi_topic else width_m
                vertices = ribbon_vertices(message, kinds[topic], ribbon_width)
                if len(vertices) >= 2 and message.header.frame_id:
                    trajectories[topic].append(
                        TrajectoryRibbon(stamp, message.header.frame_id, vertices)
                    )
    for topic, series in infos.items():
        if not series:
            raise ValueError(f"No calibrated supported CameraInfo messages on {topic}")
        if invalid_infos[topic]:
            LOG.warning("Skipped %d invalid CameraInfo messages on %s", invalid_infos[topic], topic)
        series.sort(key=lambda item: item[0])
    for topic, series in trajectories.items():
        series.sort(key=lambda item: item.record_stamp)
        LOG.info("Trajectory overlay %s: %d usable messages", topic, len(series))
    if not trajectories[main_topic]:
        raise ValueError(f"No usable trajectory messages on {main_topic}")
    if mppi_topic and not trajectories[mppi_topic]:
        LOG.warning("No usable MPPI trajectory messages on %s", mppi_topic)
    stamps = {
        topic: [item.record_stamp for item in series] for topic, series in trajectories.items()
    }
    return {
        topic: CameraOverlay(
            buffer,
            series,
            [stamp for stamp, _ in series],
            trajectories,
            stamps,
            main_topic,
            mppi_topic,
            events,
            alpha,
        )
        for topic, series in infos.items()
    }


def load_camera_overlay(
    bags: list[Bag],
    timeline: Timeline,
    kinds: dict[str, str],
    camera_info_topic: str,
    main_topic: str,
    mppi_topic: str | None,
    events: list[Event],
    width_m: float,
    mppi_width_m: float,
    alpha: float,
) -> CameraOverlay:
    return load_camera_overlays(
        bags,
        timeline,
        kinds,
        [camera_info_topic],
        main_topic,
        mppi_topic,
        events,
        width_m,
        mppi_width_m,
        alpha,
    )[camera_info_topic]


def latest_ribbon(overlay: CameraOverlay, topic: str, stamp: int) -> TrajectoryRibbon | None:
    index = bisect_right(overlay.trajectory_stamps[topic], stamp) - 1
    return overlay.trajectories[topic][index] if index >= 0 else None


def draw_ribbon(
    image: np.ndarray,
    ribbon: TrajectoryRibbon,
    overlay: CameraOverlay,
    optical_frame: str,
    image_stamp: int,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    color: tuple[int, int, int],
) -> np.ndarray:
    try:
        transform = overlay.tf.lookup_transform(
            optical_frame, ribbon.frame_id, Time(nanoseconds=image_stamp)
        )
    except TransformException as error:
        warning = f"{ribbon.frame_id} -> {optical_frame}"
        if warning not in overlay.warnings:
            LOG.warning("No camera overlay TF for %s: %s", warning, error)
            overlay.warnings.add(warning)
        return image
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    matrix = quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    vertices = ribbon.vertices.reshape(-1, 3) @ matrix.T
    vertices += np.array([translation.x, translation.y, translation.z])
    vertices = vertices.reshape(-1, 2, 3)
    quads = np.stack((vertices[:-1, 0], vertices[1:, 0], vertices[1:, 1], vertices[:-1, 1]), axis=1)
    # A small near plane also avoids enormous pixel coordinates just above Z=0.
    quads = quads[np.isfinite(quads).all(axis=(1, 2)) & (quads[:, :, 2] > 0.05).all(axis=1)]
    if not len(quads):
        return image
    pixels, _ = cv2.projectPoints(
        quads.reshape(-1, 3), np.zeros(3), np.zeros(3), camera_matrix, distortion
    )
    pixels = pixels.reshape(-1, 4, 2)
    pixels = pixels[np.isfinite(pixels).all(axis=(1, 2))]
    if not len(pixels):
        return image
    limit = 10 * max(image.shape[:2])
    polygons = np.rint(np.clip(pixels, -limit, limit)).astype(np.int32)
    layer = image.copy()
    cv2.fillPoly(layer, list(polygons), color)
    return cv2.addWeighted(layer, overlay.alpha, image, 1 - overlay.alpha, 0)


def draw_camera_overlay(
    image: np.ndarray, image_message, record_stamp: int, overlay: CameraOverlay
) -> np.ndarray:
    info_index = max(0, bisect_right(overlay.info_stamps, record_stamp) - 1)
    info = overlay.infos[info_index][1]
    if (info.width, info.height) != (image.shape[1], image.shape[0]):
        raise ValueError("CameraInfo dimensions do not match the decoded image")
    if image_message.header.frame_id != info.header.frame_id:
        raise ValueError("Camera image and CameraInfo optical frames differ")
    image_stamp = image_message.header.stamp.sec * NSEC + image_message.header.stamp.nanosec
    if not image_stamp:
        image_stamp = record_stamp
    camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(info.d, dtype=np.float64)
    main = latest_ribbon(overlay, overlay.main_topic, record_stamp)
    if main:
        image = draw_ribbon(
            image,
            main,
            overlay,
            info.header.frame_id,
            image_stamp,
            camera_matrix,
            distortion,
            MAIN_RIBBON_COLOR,
        )
    in_event = any(event.start <= record_stamp < event.end for event in overlay.events)
    if overlay.mppi_topic and in_event:
        mppi = latest_ribbon(overlay, overlay.mppi_topic, record_stamp)
        if mppi and record_stamp - mppi.record_stamp <= MPPI_MAX_AGE_NS:
            image = draw_ribbon(
                image,
                mppi,
                overlay,
                info.header.frame_id,
                image_stamp,
                camera_matrix,
                distortion,
                MPPI_RIBBON_COLOR,
            )
    return image


def camera_video(
    bags: list[Bag],
    topic: str,
    kind: str,
    path: Path,
    timeline: Timeline,
    overlay: CameraOverlay | None = None,
    output_width: int | None = None,
    label: str | None = None,
) -> None:
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
                message = deserialize_message(payload, cls)
                image = decode_camera(message, kind)
                if overlay:
                    image = draw_camera_overlay(image, message, stamp, overlay)
                if output_width is not None:
                    image = cv2.resize(
                        image,
                        (output_width, round(image.shape[0] * output_width / image.shape[1])),
                        interpolation=cv2.INTER_AREA,
                    )
                if label:
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    scale = max(0.5, image.shape[1] / 900)
                    thickness = max(1, round(scale * 2))
                    (text_width, text_height), _ = cv2.getTextSize(label, font, scale, thickness)
                    cv2.rectangle(image, (8, 8), (text_width + 22, text_height + 24), (0, 0, 0), -1)
                    cv2.putText(
                        image,
                        label,
                        (15, text_height + 15),
                        font,
                        scale,
                        (255, 255, 255),
                        thickness,
                        cv2.LINE_AA,
                    )
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


def plot_video(
    bags: list[Bag],
    config: Config,
    path: Path,
    timeline: Timeline,
    kinds: dict[str, str] | None = None,
    vehicle_model: str | None = None,
) -> None:
    if config.clearance is not None:
        if kinds is None or not vehicle_model:
            raise ValueError("Clearance plots require bag topic types and --vehicle-model")
        from clearance_metrics import render_clearance_plot

        render_clearance_plot(bags, kinds, config, vehicle_model, path, timeline, FPS, PLOT)
        return
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
    axis.set_xlabel("Seconds from video start", color="white", fontsize=8)
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


def fullscreen_rviz_window(pid: int, env: dict[str, str]) -> None:
    """Send RViz's F11 shortcut and verify its window fills the virtual screen."""
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
        ["xdotool", "windowactivate", "--sync", window_id],
        env=env,
        check=True,
    )
    subprocess.run(
        ["xdotool", "key", "F11"],
        env=env,
        check=True,
    )
    for _ in range(50):
        actual = window_geometry(window_id)
        if actual == {"X": 0, "Y": 0, "WIDTH": SCREEN[0], "HEIGHT": SCREEN[1]}:
            LOG.info("RViz is full screen: %s", actual)
            return
        time.sleep(0.1)
    raise RuntimeError(f"RViz did not enter full screen mode; window geometry is {actual}")


def vehicle_description(model: str, env: dict[str, str]) -> str:
    package = f"{model}_description"
    xacro_path = Path(get_package_share_directory(package)) / "urdf/vehicle.xacro"
    if not xacro_path.is_file():
        raise FileNotFoundError(f"Vehicle Xacro not found: {xacro_path}")
    result = subprocess.run(
        ["xacro", str(xacro_path)], env=env, check=True, text=True, capture_output=True
    )
    root = ElementTree.fromstring(result.stdout)
    if root.tag != "robot" or not any(
        link.get("name") == "base_link" for link in root.findall("link")
    ):
        raise ValueError(f"Vehicle Xacro must produce a robot with a base_link: {xacro_path}")
    # The standalone j6_gen2 Xacro omits the robot name. Its normal workspace
    # wrapper supplies one, and robot_state_publisher requires it.
    if not root.get("name"):
        root.set("name", model)
    LOG.info("Loaded vehicle description from %s", xacro_path)
    return ElementTree.tostring(root, encoding="unicode")


def capture_rviz(
    bags: list[Bag],
    rviz_config: Path,
    topics: list[str],
    path: Path,
    vehicle_model: str | None = None,
):
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
    xvfb = window_manager = rviz = ffmpeg = clock = vehicle_publisher = None
    vehicle_params = None
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
        window_manager_log = path.with_suffix(".window_manager.log")
        with window_manager_log.open("w", encoding="utf-8") as stream:
            window_manager = subprocess.Popen(
                ["dbus-run-session", "--", "xfwm4", "--compositor=off"],
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
            )
        time.sleep(1)
        if window_manager.poll() is not None:
            detail = window_manager_log.read_text(encoding="utf-8", errors="replace")[-2000:]
            raise RuntimeError(f"xfwm4 exited early: {detail}")
        if vehicle_model:
            description = vehicle_description(vehicle_model, env)
            params_stream = tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                suffix=".yaml",
                prefix="vehicle_description_",
                dir=path.parent,
                delete=False,
            )
            vehicle_params = Path(params_stream.name)
            with params_stream:
                yaml.safe_dump(
                    {
                        "robot_state_publisher": {
                            "ros__parameters": {
                                "robot_description": description,
                                "use_sim_time": True,
                            }
                        }
                    },
                    params_stream,
                )
            executable = (
                Path(get_package_prefix("robot_state_publisher"))
                / "lib/robot_state_publisher/robot_state_publisher"
            )
            vehicle_log = path.with_suffix(".vehicle.log")
            with vehicle_log.open("w", encoding="utf-8") as stream:
                vehicle_publisher = subprocess.Popen(
                    [str(executable), "--ros-args", "--params-file", str(vehicle_params)],
                    env=env,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                )
            time.sleep(1)
            if vehicle_publisher.poll() is not None:
                detail = vehicle_log.read_text(encoding="utf-8", errors="replace")[-2000:]
                raise RuntimeError(
                    f"robot_state_publisher exited early "
                    f"(status {vehicle_publisher.returncode}): {detail}"
                )
        # RViz writes display state back to its config on exit. Record with a
        # copy so bag-specific marker state cannot alter the user's config.
        runtime_config = path.with_suffix(".rviz")
        shutil.copyfile(rviz_config, runtime_config)
        rviz_log = path.with_suffix(".rviz.log")
        rviz_log_stream = rviz_log.open("w", encoding="utf-8")
        rviz = subprocess.Popen(
            ["rviz2", "-d", str(runtime_config), "--ros-args", "-p", "use_sim_time:=true"],
            env=env,
            stdout=rviz_log_stream,
            stderr=subprocess.STDOUT,
        )
        rviz_log_stream.close()
        time.sleep(4)
        if rviz.poll() is not None:
            detail = rviz_log.read_text(encoding="utf-8", errors="replace")[-2000:]
            raise RuntimeError(f"rviz2 exited early (status {rviz.returncode}): {detail}")
        fullscreen_rviz_window(rviz.pid, env)
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
                "-draw_mouse",
                "0",
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
            if vehicle_publisher:
                alive(vehicle_publisher, "robot_state_publisher")
            if index + 1 < len(bags):
                gap = bags[index + 1].start - bag.end
                if gap > 0:
                    time.sleep(gap / NSEC)
        time.sleep(0.25)
    finally:
        stop(ffmpeg, interrupt=True)
        samples = clock.close() if clock else []
        stop(rviz)
        stop(vehicle_publisher)
        stop(window_manager)
        stop(xvfb)
        if vehicle_params:
            vehicle_params.unlink(missing_ok=True)
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
    camera_paths: Path | list[Path],
    plot_path: Path,
    final_path: Path,
    config: Config,
    timeline: Timeline,
) -> None:
    try:
        from moviepy import CompositeVideoClip
        from moviepy import ImageClip
        from moviepy import VideoFileClip
        from moviepy import concatenate_videoclips
        from moviepy import vfx
    except ImportError as error:
        raise RuntimeError("MoviePy 2 is required: pip install 'moviepy>=2,<3'") from error
    # ffmpeg may report MP4 duration rounded to centiseconds (e.g. 0.63 for
    # 19 frames at 30 FPS). The verified frame count is our duration authority.
    asset_duration = timeline.count / FPS
    paths = [camera_paths] if isinstance(camera_paths, Path) else camera_paths
    rviz = VideoFileClip(str(rviz_path)).with_duration(asset_duration)
    cameras = [VideoFileClip(str(path)).with_duration(asset_duration) for path in paths]
    plot = VideoFileClip(str(plot_path)).with_duration(asset_duration)
    segments = []
    try:
        if tuple(rviz.size) != SCREEN:
            raise ValueError(f"RViz asset must be {SCREEN[0]}x{SCREEN[1]}; got {rviz.size}")
        rviz_width, height = rviz.size
        width = rviz_width + SIDEBAR_WIDTH
        sidebar_x = rviz_width
        if len(cameras) > 1:
            gap = 8
            if tuple(plot.size) != PLOT:
                raise ValueError(f"Plot asset must be {PLOT[0]}x{PLOT[1]}; got {plot.size}")
            columns = 2 if len(cameras) <= 4 else 3
            rows = math.ceil(len(cameras) / columns)
            tile_width = (SIDEBAR_WIDTH - (columns - 1) * gap) // columns
            grid_top = plot.h + gap
            tile_height = (height - grid_top - (rows - 1) * gap) // rows
            if tile_width <= 0 or tile_height <= 0:
                raise ValueError("Too many cameras for the available sidebar space")
            camera_positions = []
            resized_cameras = []
            for index, camera in enumerate(cameras):
                scale = min(tile_width / camera.w, tile_height / camera.h)
                fitted = camera.resized(scale)
                row, col = divmod(index, columns)
                row_count = min(columns, len(cameras) - row * columns)
                row_left = (
                    sidebar_x
                    + (SIDEBAR_WIDTH - row_count * tile_width - (row_count - 1) * gap) // 2
                )
                camera_positions.append(
                    (
                        row_left + col * (tile_width + gap) + (tile_width - fitted.w) // 2,
                        grid_top + row * (tile_height + gap) + (tile_height - fitted.h) // 2,
                    )
                )
                resized_cameras.append(fitted)
            cameras = resized_cameras
            plot_position = (sidebar_x, 0)
        elif len(cameras) == 1:
            margin = 24
            overlay_width = min(480, SIDEBAR_WIDTH - 2 * margin)
            cameras = [cameras[0].resized(width=overlay_width)]
            plot = plot.resized(width=overlay_width)
            if cameras[0].h + plot.h + 3 * margin > height:
                raise ValueError("RViz canvas too short for overlays")
            sidebar_center = sidebar_x + (SIDEBAR_WIDTH - overlay_width) // 2
            camera_positions = [(sidebar_center, margin)]
            plot_position = (sidebar_center, height - plot.h - margin)
        else:
            raise ValueError("Compositor expects at least one camera video")
        boundaries = []
        cursor = 0
        for event in config.events:
            first, last = timeline.first_frame(event.start), timeline.first_frame(event.end)
            if first < cursor:
                raise ValueError("Event intervals overlap on the 30 FPS grid")
            if first > cursor:
                boundaries.append((cursor, first, config.default_speed))
            if last > first:
                boundaries.append((first, last, event.speed))
            else:
                LOG.warning("Event shorter than one frame omitted: %d..%d", event.start, event.end)
            cursor = last
        if cursor < timeline.count:
            boundaries.append((cursor, timeline.count, config.default_speed))
        for first, last, multiplier in boundaries:
            begin, end = first / FPS, last / FPS
            layers = [rviz.subclipped(begin, end)]
            layers.extend(
                camera.subclipped(begin, end).with_position(position)
                for camera, position in zip(cameras, camera_positions)
            )
            layers.append(plot.subclipped(begin, end).with_position(plot_position))
            label = f"Replay: {multiplier:g}x"
            font = cv2.FONT_HERSHEY_SIMPLEX
            (text_width, text_height), _ = cv2.getTextSize(label, font, 0.9, 2)
            badge = np.zeros((text_height + 28, text_width + 32, 4), dtype=np.uint8)
            badge[:] = (12, 20, 30, 205)
            cv2.putText(
                badge,
                label,
                (16, text_height + 14),
                font,
                0.9,
                (255, 255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            layers.append(ImageClip(badge).with_duration(end - begin).with_position((48, 64)))
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
        for camera in cameras:
            camera.close()
        plot.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-files", nargs="+", required=True)
    parser.add_argument("--rviz-config", type=Path, required=True)
    parser.add_argument(
        "--vehicle-model",
        help="Publish the installed <name>_description vehicle Xacro during RViz capture",
    )
    camera_group = parser.add_mutually_exclusive_group(required=True)
    camera_group.add_argument("--camera-topic", help="Single camera image topic")
    camera_group.add_argument("--camera-config", type=Path, help="Multi-camera grid YAML")
    parser.add_argument(
        "--birdseye", action="store_true", help="Put a bird's-eye view in the fourth grid cell"
    )
    parser.add_argument(
        "--birdseye-ego-topic",
        default="/localization/kinematic_state",
        help="Odometry topic for the bird's-eye ego footprint",
    )
    parser.add_argument(
        "--birdseye-ml-topic",
        default="/planning/trajectory_generator/neural_network_based_planner/ml_planner_node/output/trajectory",
        help="Continuous ML planner trajectory topic for the bird's-eye view",
    )
    parser.add_argument(
        "--birdseye-range-m",
        type=float,
        default=4.0,
        help="Fore/aft half-range around ego in the bird's-eye view (default: 4 m)",
    )
    parser.add_argument("--camera-info-topic", help="CameraInfo for the camera image overlay")
    parser.add_argument("--trajectory-topic", help="Trajectory ribbon shown throughout the video")
    parser.add_argument(
        "--mppi-trajectory-topic", help="Second trajectory ribbon shown during events"
    )
    parser.add_argument("--ribbon-width-m", type=float, default=2.0)
    parser.add_argument("--mppi-ribbon-width-m", type=float, default=1.0)
    parser.add_argument("--ribbon-alpha", type=float, default=0.4)
    parser.add_argument(
        "--start-before-motion-s",
        type=float,
        help="Begin this many seconds before the first ego speed reaching the threshold",
    )
    parser.add_argument(
        "--motion-topic",
        default=MOTION_TOPIC,
        help=f"Ego speed topic for start detection (default: {MOTION_TOPIC})",
    )
    parser.add_argument(
        "--motion-threshold-mps",
        type=float,
        default=0.1,
        help="Speed that counts as moving (default: 0.1 m/s)",
    )
    parser.add_argument("--plot-config", type=Path, required=True)
    parser.add_argument("--intermediate-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    if args.camera_config and args.camera_info_topic:
        parser.error("--camera-info-topic is supplied by --camera-config")
    if args.trajectory_topic and not (args.camera_info_topic or args.camera_config):
        parser.error("--trajectory-topic requires camera info")
    if args.mppi_trajectory_topic and not args.trajectory_topic:
        parser.error("--mppi-trajectory-topic requires --trajectory-topic")
    if args.camera_info_topic and not args.trajectory_topic:
        parser.error("--camera-info-topic requires --trajectory-topic")
    if not math.isfinite(args.ribbon_width_m) or args.ribbon_width_m <= 0:
        parser.error("--ribbon-width-m must be positive and finite")
    if not math.isfinite(args.mppi_ribbon_width_m) or args.mppi_ribbon_width_m <= 0:
        parser.error("--mppi-ribbon-width-m must be positive and finite")
    if not math.isfinite(args.ribbon_alpha) or not 0 < args.ribbon_alpha <= 1:
        parser.error("--ribbon-alpha must be within (0, 1]")
    if args.start_before_motion_s is not None:
        if not math.isfinite(args.start_before_motion_s) or args.start_before_motion_s < 0:
            parser.error("--start-before-motion-s must be finite and nonnegative")
        if not math.isfinite(args.motion_threshold_mps) or args.motion_threshold_mps <= 0:
            parser.error("--motion-threshold-mps must be finite and positive")
    if not math.isfinite(args.birdseye_range_m) or args.birdseye_range_m <= 0:
        parser.error("--birdseye-range-m must be finite and positive")
    validate_ros_environment()
    for path in (args.rviz_config, args.plot_config, args.camera_config):
        if path is None:
            continue
        if not path.is_file():
            parser.error(f"File does not exist: {path}")
    sources = (
        camera_sources(args.camera_config)
        if args.camera_config
        else [CameraSource("camera", "Camera", args.camera_topic, args.camera_info_topic)]
    )
    if args.birdseye and (not args.camera_config or len(sources) != 3):
        parser.error("--birdseye requires a camera config with exactly three cameras")
    bags = read_bags(args.bag_files)
    start = bags[0].start
    if args.start_before_motion_s is not None:
        first = first_motion_stamp(bags, args.motion_topic, args.motion_threshold_mps)
        start = max(start, first - round(args.start_before_motion_s * NSEC))
        LOG.info(
            "Video starts at %d ns, %.3f s before first ego motion (%.3f s into bag)",
            start,
            (first - start) / NSEC,
            (start - bags[0].start) / NSEC,
        )
    timeline = Timeline(start, bags[-1].end)
    kinds = {}
    for bag in bags:
        for topic, kind in bag.topics.items():
            if topic in kinds and kinds[topic] != kind:
                raise ValueError(f"Topic type changes across bags: {topic}")
            kinds[topic] = kind
    for source in sources:
        kind = kinds.get(source.image_topic)
        if kind not in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}:
            raise ValueError(
                f"Camera topic {source.image_topic} has unsupported or missing type: {kind}"
            )
    requested = rviz_topics(args.rviz_config)
    generated_topics = {"/robot_description"} if args.vehicle_model else set()
    for topic in sorted(requested - kinds.keys() - generated_topics):
        LOG.warning("RViz topic missing from bags: %s", topic)
    play_topics = sorted(
        (
            (requested | {source.image_topic for source in sources} | {"/tf", "/tf_static"})
            & kinds.keys()
        )
        - generated_topics
    )
    LOG.info("RViz playback topics (%d): %s", len(play_topics), ", ".join(play_topics))
    config = config_from_yaml(args.plot_config, timeline, kinds, bags)
    if config.clearance is not None and not args.vehicle_model:
        parser.error("Clearance plots require --vehicle-model for the ego footprint")
    if args.birdseye and (config.clearance is None or not args.vehicle_model):
        parser.error("--birdseye requires clearance plot config and --vehicle-model")
    overlays = (
        load_camera_overlays(
            bags,
            timeline,
            kinds,
            [source.info_topic for source in sources],
            args.trajectory_topic,
            args.mppi_trajectory_topic,
            config.events,
            args.ribbon_width_m,
            args.mppi_ribbon_width_m,
            args.ribbon_alpha,
        )
        if args.trajectory_topic
        else {}
    )
    args.intermediate_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    camera_paths = [
        args.intermediate_dir
        / ("camera_1x.mp4" if not args.camera_config else f"camera_{source.name}_1x.mp4")
        for source in sources
    ]
    plot_path = args.intermediate_dir / "plot_1x.mp4"
    rviz_path = args.intermediate_dir / "rviz_1x.mp4"
    final_path = args.output_dir / "final_video.mp4"
    for source, camera_path in zip(sources, camera_paths):
        camera_video(
            bags,
            source.image_topic,
            kinds[source.image_topic],
            camera_path,
            timeline,
            overlays.get(source.info_topic),
            output_width=800 if args.camera_config else None,
            label=source.label if args.camera_config else None,
        )
    if args.birdseye:
        from birdseye_view import render_video

        birdseye_path = args.intermediate_dir / "birdseye_1x.mp4"
        render_video(
            bags,
            timeline,
            kinds,
            config.clearance,
            args.birdseye_ego_topic,
            args.vehicle_model,
            birdseye_path,
            FPS,
            args.birdseye_ml_topic,
            args.birdseye_range_m,
        )
        camera_paths.append(birdseye_path)
    plot_video(bags, config, plot_path, timeline, kinds, args.vehicle_model)
    with tempfile.TemporaryDirectory(prefix="bag_video_rviz_") as directory:
        capture = Path(directory) / "rviz_capture.mkv"
        trace, capture_start = capture_rviz(
            bags, args.rviz_config.resolve(), play_topics, capture, args.vehicle_model
        )
        align_rviz(capture, rviz_path, trace, timeline, capture_start)
    for path in (*camera_paths, plot_path, rviz_path):
        count = video_frames(path)
        if count != timeline.count:
            raise RuntimeError(f"{path} has {count} frames, expected {timeline.count}")
    compose(rviz_path, camera_paths, plot_path, final_path, config, timeline)
    LOG.info("Final video: %s", final_path)


if __name__ == "__main__":
    main()
