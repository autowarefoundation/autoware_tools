"""Render an ego-centered, synchronized bird's-eye view from ROS bag data."""

from __future__ import annotations

from bisect import bisect_right
import logging
import math

from clearance_metrics import _at_pose
from clearance_metrics import _object_polygon
from clearance_metrics import _reader
from clearance_metrics import _yaw
from clearance_metrics import road_border_segments
from clearance_metrics import vehicle_footprint
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LOG = logging.getLogger("bag_video")
NSEC = 1_000_000_000
SIZE = (800, 516)
DEFAULT_RANGE_M = 4.0
DEFAULT_FOOTPRINT_ALPHA = 0.40
DEFAULT_FOOTPRINT_LINE_WIDTH = 2
EGO_MAX_AGE_NS = 500_000_000
OBJECT_MAX_AGE_NS = 500_000_000
TRAJECTORY_MAX_AGE_NS = NSEC
STATIONARY_SPEED_MPS = 0.5
VEHICLE_LABELS = {1, 2, 3, 4}  # CAR, TRUCK, BUS, TRAILER
COLORS = {"MPPI": (166, 213, 139), "ML planner": (230, 215, 75)}  # BGR


def project_points(
    points: np.ndarray, center: np.ndarray, yaw: float, range_m: float = DEFAULT_RANGE_M
) -> np.ndarray:
    """Map world XY into an ego-up view with a metric fore/aft half-range."""
    delta = np.asarray(points, dtype=np.float64) - center
    forward = math.cos(yaw) * delta[..., 0] + math.sin(yaw) * delta[..., 1]
    left = -math.sin(yaw) * delta[..., 0] + math.cos(yaw) * delta[..., 1]
    scale = (SIZE[1] / 2) / range_m
    return np.stack((SIZE[0] / 2 - scale * left, SIZE[1] / 2 - scale * forward), axis=-1)


def ego_geometry(local_footprint, pose):
    """Return the rectangular footprint's world center, yaw, and corners."""
    outline = np.asarray(_at_pose(local_footprint, pose), dtype=np.float64)
    return outline.mean(axis=0), _yaw(pose.orientation), outline


def latest(series, stamps, now: int, maximum_age: int | None = None):
    index = bisect_right(stamps, now) - 1
    if index < 0 or (maximum_age is not None and now - stamps[index] > maximum_age):
        return None
    return series[index][1]


def parked_polygons(message):
    """Use stationary vehicle classifications from the tracked-object snapshot."""
    polygons = []
    for obj in message.objects:
        if not obj.classification:
            continue
        classification = max(obj.classification, key=lambda item: item.probability)
        if classification.label not in VEHICLE_LABELS:
            continue
        velocity = obj.kinematics.twist_with_covariance.twist.linear
        if math.hypot(velocity.x, velocity.y) >= STATIONARY_SPEED_MPS:
            continue
        polygon = _object_polygon(obj)
        if polygon is not None and not polygon.is_empty and polygon.geom_type == "Polygon":
            polygons.append(np.asarray(polygon.exterior.coords, dtype=np.float64))
    return polygons


def extract_scene(bags, timeline, kinds, clearance, ego_topic, vehicle_model, ml_topic=None):
    topics = {
        "MPPI": clearance["optimized_topic"],
        "ML planner": ml_topic or clearance["reference_topic"],
    }
    objects_topic = clearance["objects_topic"]
    borders_topic = clearance["road_border_markers_topic"]
    required = set(topics.values()) | {ego_topic, objects_topic, borders_topic}
    missing = required - kinds.keys()
    if missing:
        raise ValueError(f"Bird's-eye view topics missing from bags: {', '.join(sorted(missing))}")
    if kinds[ego_topic] != "nav_msgs/msg/Odometry":
        raise ValueError(f"Bird's-eye ego topic must be nav_msgs/msg/Odometry: {ego_topic}")
    classes = {topic: get_message(kinds[topic]) for topic in required}
    footprint = vehicle_footprint(vehicle_model)
    ego, objects, maps = [], [], []
    trajectories = {name: [] for name in topics}
    name_by_topic = {topic: name for name, topic in topics.items()}
    for bag in bags:
        active = sorted(required & bag.topics.keys())
        if not active:
            continue
        reader = _reader(bag, active)
        while reader.has_next():
            topic, payload, stamp = reader.read_next()
            if topic != borders_topic and stamp < timeline.start - NSEC:
                continue
            message = deserialize_message(payload, classes[topic])
            if topic == ego_topic:
                center, yaw, outline = ego_geometry(footprint, message.pose.pose)
                ego.append((stamp, (message.header.frame_id, center, yaw, outline)))
            elif topic == objects_topic:
                objects.append((stamp, (message.header.frame_id, parked_polygons(message))))
            elif topic == borders_topic:
                if any(marker.ns == "road_borders" for marker in message.markers):
                    maps.append((stamp, road_border_segments(message)))
            else:
                name = name_by_topic[topic]
                if message.points:
                    path = np.asarray(
                        [
                            (point.pose.position.x, point.pose.position.y)
                            for point in message.points
                        ],
                        dtype=np.float64,
                    )
                    outlines = [
                        np.asarray(_at_pose(footprint, point.pose)) for point in message.points
                    ]
                    trajectories[name].append((stamp, (message.header.frame_id, path, outlines)))
    if not maps:
        raise ValueError(f"No road borders found on {borders_topic}")
    if not ego:
        raise ValueError(f"No ego poses found on {ego_topic}")
    for series in (ego, objects, maps, *trajectories.values()):
        series.sort(key=lambda item: item[0])
    return ego, objects, maps, trajectories


def render_frame(
    now,
    scene,
    range_m: float = DEFAULT_RANGE_M,
    footprint_alpha: float = DEFAULT_FOOTPRINT_ALPHA,
    footprint_line_width: int = DEFAULT_FOOTPRINT_LINE_WIDTH,
) -> np.ndarray:
    ego, objects, maps, trajectories = scene
    frame = np.full((SIZE[1], SIZE[0], 3), (28, 35, 42), dtype=np.uint8)
    ego_state = latest(ego, [stamp for stamp, _ in ego], now, EGO_MAX_AGE_NS)
    if ego_state is None:
        cv2.putText(
            frame,
            "Waiting for ego pose",
            (245, 265),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        return frame
    frame_id, center, yaw, ego_outline = ego_state
    scale = (SIZE[1] / 2) / range_m
    grid_step_m = 1 if range_m <= 4 else 5
    half_width_m = SIZE[0] / (2 * scale)
    for offset in range(
        -math.ceil(half_width_m / grid_step_m) * grid_step_m,
        math.ceil(half_width_m / grid_step_m) * grid_step_m + 1,
        grid_step_m,
    ):
        x = round(SIZE[0] / 2 + offset * scale)
        cv2.line(frame, (x, 0), (x, SIZE[1]), (47, 55, 63), 1)
    for offset in range(
        -math.ceil(range_m / grid_step_m) * grid_step_m,
        math.ceil(range_m / grid_step_m) * grid_step_m + 1,
        grid_step_m,
    ):
        y = round(SIZE[1] / 2 - offset * scale)
        cv2.line(frame, (0, y), (SIZE[0], y), (47, 55, 63), 1)
    border = latest(maps, [stamp for stamp, _ in maps], now)
    if border is not None and border[0] == frame_id:
        pixels = project_points(border[1], center, yaw, range_m)
        visible = np.any(
            (pixels[..., 0] >= -4)
            & (pixels[..., 0] < SIZE[0] + 4)
            & (pixels[..., 1] >= -4)
            & (pixels[..., 1] < SIZE[1] + 4),
            axis=1,
        )
        contours = [np.rint(line).astype(np.int32) for line in pixels[visible]]
        if contours:
            cv2.polylines(frame, contours, False, (145, 158, 170), 2, cv2.LINE_AA)
    current_objects = latest(objects, [stamp for stamp, _ in objects], now, OBJECT_MAX_AGE_NS)
    if current_objects is not None and current_objects[0] == frame_id:
        polygons = [
            np.rint(project_points(shape, center, yaw, range_m)).astype(np.int32)
            for shape in current_objects[1]
        ]
        if polygons:
            overlay = frame.copy()
            cv2.fillPoly(overlay, polygons, (40, 145, 242))
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
            cv2.polylines(frame, polygons, True, (60, 185, 255), 2, cv2.LINE_AA)
    ego_pixels = np.rint(project_points(ego_outline, center, yaw, range_m)).astype(np.int32)
    overlay = frame.copy()
    cv2.fillPoly(overlay, [ego_pixels], (90, 210, 105))
    cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
    cv2.polylines(frame, [ego_pixels], True, (135, 255, 145), 3, cv2.LINE_AA)
    for name in ("ML planner", "MPPI"):
        series = trajectories[name]
        current = latest(series, [stamp for stamp, _ in series], now, TRAJECTORY_MAX_AGE_NS)
        if current is None or current[0] != frame_id:
            continue
        color = COLORS[name]
        _, path, outlines = current
        path_pixels = np.rint(project_points(path, center, yaw, range_m)).astype(np.int32)
        footprint_pixels = [
            np.rint(project_points(shape, center, yaw, range_m)).astype(np.int32)
            for shape in outlines
        ]
        overlay = frame.copy()
        cv2.polylines(overlay, footprint_pixels, True, color, footprint_line_width, cv2.LINE_AA)
        cv2.addWeighted(overlay, footprint_alpha, frame, 1 - footprint_alpha, 0, frame)
        if len(path_pixels) > 1:
            cv2.polylines(
                frame, [path_pixels], False, color, 7 if name == "ML planner" else 4, cv2.LINE_AA
            )
    cv2.rectangle(frame, (0, SIZE[1] - 37), (SIZE[0], SIZE[1]), (16, 24, 30), -1)
    for x, label, color in (
        (15, "Border", (145, 158, 170)),
        (155, "Parked", (60, 185, 255)),
        (305, "Ego", (135, 255, 145)),
        (420, "MPPI", COLORS["MPPI"]),
        (555, "ML planner", COLORS["ML planner"]),
    ):
        cv2.line(frame, (x, SIZE[1] - 19), (x + 23, SIZE[1] - 19), color, 3)
        cv2.putText(
            frame,
            label,
            (x + 30, SIZE[1] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
    return frame


def render_video(
    bags,
    timeline,
    kinds,
    clearance,
    ego_topic,
    vehicle_model,
    path,
    fps,
    ml_topic=None,
    range_m: float = DEFAULT_RANGE_M,
):
    if not math.isfinite(range_m) or range_m <= 0:
        raise ValueError("Bird's-eye range must be positive and finite")
    scene = extract_scene(bags, timeline, kinds, clearance, ego_topic, vehicle_model, ml_topic)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, SIZE)
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create bird's-eye MP4: {path}")
    try:
        for index in range(timeline.count):
            writer.write(render_frame(timeline.stamp(index), scene, range_m))
    finally:
        writer.release()
    LOG.info("Bird's-eye view: %d frames", timeline.count)
