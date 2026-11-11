"""Offline footprint clearances for the MPPI and ML planner trajectories."""

from __future__ import annotations

from bisect import bisect_left
from bisect import bisect_right
import logging
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.strtree import STRtree
import yaml

LOG = logging.getLogger("bag_video")
NSEC = 1_000_000_000
SELECTOR_MATCH_NS = 20_000_000
OBJECT_MAX_AGE_NS = 500_000_000


def _reader(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def _yaw(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _at_pose(local_xy, pose):
    angle = _yaw(pose.orientation)
    cosine, sine = math.cos(angle), math.sin(angle)
    x, y = pose.position.x, pose.position.y
    return [(x + cosine * a - sine * b, y + sine * a + cosine * b) for a, b in local_xy]


def vehicle_footprint(model: str):
    path = (
        Path(get_package_share_directory(f"{model}_description")) / "config/vehicle_info.param.yaml"
    )
    with path.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["/**"]["ros__parameters"]
    front = params["wheel_base"] + params["front_overhang"]
    rear = -params["rear_overhang"]
    left = params["wheel_tread"] / 2 + params["left_overhang"]
    right = -params["wheel_tread"] / 2 - params["right_overhang"]
    if not (front > 0 > rear and left > 0 > right):
        raise ValueError(f"Invalid vehicle footprint dimensions: {path}")
    return [(front, left), (front, right), (rear, right), (rear, left)]


def road_border_segments(message):
    """Recover centerline endpoints from the map visualizer's triangle strips."""
    segments = []
    frame = None
    for marker in message.markers:
        if marker.ns != "road_borders" or marker.action != 0:
            continue
        if marker.type != 11 or len(marker.points) % 6:
            raise ValueError("road_borders marker must contain six vertices per line segment")
        if frame is not None and frame != marker.header.frame_id:
            raise ValueError("road_borders markers have mixed frames")
        frame = marker.header.frame_id
        points = np.array([(point.x, point.y) for point in marker.points], dtype=np.float64)
        groups = points.reshape(-1, 6, 2)
        if not (
            np.allclose(groups[:, 4], groups[:, 2], rtol=0, atol=1e-8)
            and np.allclose(groups[:, 5], groups[:, 1], rtol=0, atol=1e-8)
        ):
            raise ValueError("road_borders triangle vertices do not form paired line segments")
        starts = (groups[:, 0] + groups[:, 1]) / 2
        ends = (groups[:, 2] + groups[:, 3]) / 2
        segments.extend(
            (start, end) for start, end in zip(starts, ends) if np.linalg.norm(start - end) > 1e-6
        )
    if not segments or not frame:
        raise ValueError("No usable road_borders marker in map visualization")
    return frame, np.asarray(segments, dtype=np.float64)


def road_border_lines(message):
    """Return indexed road-border centerlines for clearance queries."""
    frame, segments = road_border_segments(message)
    lines = [LineString(segment) for segment in segments]
    return frame, STRtree(lines), len(lines)


def _selector_ids(message):
    for status in message.status:
        if status.name != "trajectory_modifier: mppi_cost_breakdown":
            continue
        values = {item.key: item.value for item in status.values}
        if values.get("selector/status") != "ready":
            continue
        prefix = "selector/objects/"
        selected = {
            key[len(prefix) : -len("/reason")]
            for key, value in values.items()
            if key.startswith(prefix)
            and key.endswith("/reason")
            and value == "parked_off_lane_center"
            and values.get(key[: -len("reason")] + "disposition") == "avoidance"
        }
        return selected
    return None


def _object_polygon(obj):
    pose = obj.kinematics.pose_with_covariance.pose
    shape = obj.shape
    if shape.type == 0:  # BOUNDING_BOX
        length, width = shape.dimensions.x, shape.dimensions.y
        if length <= 0 or width <= 0:
            return None
        half_l, half_w = length / 2, width / 2
        local = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
        return Polygon(_at_pose(local, pose))
    if shape.type == 1:  # CYLINDER
        if shape.dimensions.x <= 0:
            return None
        return Point(pose.position.x, pose.position.y).buffer(shape.dimensions.x / 2)
    if shape.type == 2 and len(shape.footprint.points) >= 3:  # POLYGON
        local = [(point.x, point.y) for point in shape.footprint.points]
        polygon = Polygon(_at_pose(local, pose))
        return polygon if polygon.is_valid else polygon.buffer(0)
    return None


def _closest(stamps, stamp, tolerance_ns):
    index = bisect_left(stamps, stamp)
    choices = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
    if not choices:
        return None
    chosen = min(choices, key=lambda i: abs(stamps[i] - stamp))
    return chosen if abs(stamps[chosen] - stamp) <= tolerance_ns else None


def _nearest_border(tree, polygon):
    nearest = tree.nearest(polygon)
    # Shapely 1.x returns a geometry; Shapely 2.x returns its tree index.
    return tree.geometries[nearest] if isinstance(nearest, (int, np.integer)) else nearest


def compute_clearances(bags, kinds, config: dict, vehicle_model: str):
    """Return road-border and parked-vehicle series for each trajectory."""
    topics = {
        "MPPI": config["optimized_topic"],
        "ML planner": config["reference_topic"],
    }
    objects_topic = config["objects_topic"]
    diagnostics_topic = config["diagnostics_topic"]
    borders_topic = config["road_border_markers_topic"]
    active_topics = list(topics.values()) + [objects_topic, diagnostics_topic, borders_topic]
    classes = {topic: get_message(kinds[topic]) for topic in active_topics}
    trajectories = {name: [] for name in topics}
    objects = []
    selectors = []
    maps = []
    for bag in bags:
        active = sorted(set(active_topics) & bag.topics.keys())
        reader = _reader(bag, active)
        while reader.has_next():
            topic, payload, stamp = reader.read_next()
            message = deserialize_message(payload, classes[topic])
            if topic in topics.values():
                name = next(name for name, value in topics.items() if value == topic)
                if message.points:
                    trajectories[name].append((stamp, message))
            elif topic == objects_topic:
                objects.append((stamp, message))
            elif topic == diagnostics_topic:
                selected = _selector_ids(message)
                if selected is not None:
                    selectors.append((stamp, selected))
            else:
                if any(marker.ns == "road_borders" for marker in message.markers):
                    maps.append((stamp, *road_border_lines(message)))
    if not maps:
        raise ValueError(f"No road_borders markers found on {borders_topic}")
    if not objects:
        raise ValueError(f"No tracked-object messages found on {objects_topic}")
    if not selectors:
        raise ValueError(f"No MPPI selector decisions found on {diagnostics_topic}")
    for name, series in trajectories.items():
        if not series:
            raise ValueError(f"No trajectory messages found for {name}: {topics[name]}")
    maps.sort(key=lambda item: item[0])
    objects.sort(key=lambda item: item[0])
    selectors.sort(key=lambda item: item[0])
    object_stamps = [stamp for stamp, _ in objects]
    selector_stamps = [stamp for stamp, _ in selectors]
    map_stamps = [item[0] for item in maps]
    local_footprint = vehicle_footprint(vehicle_model)
    LOG.info(
        "Clearance: %d road-border segments; %d selector diagnostics", maps[0][3], len(selectors)
    )
    output = {category: {} for category in ("road_border", "static_vehicle")}
    for name, series in trajectories.items():
        series.sort(key=lambda item: item[0])
        times = []
        border_values = []
        vehicle_values = []
        for stamp, trajectory in series:
            map_index = max(0, bisect_right(map_stamps, stamp) - 1)
            _, frame, border_tree, _ = maps[map_index]
            if trajectory.header.frame_id != frame:
                raise ValueError(
                    f"Trajectory/map frames differ: {trajectory.header.frame_id} != {frame}"
                )
            footprints = [
                Polygon(_at_pose(local_footprint, point.pose)) for point in trajectory.points
            ]
            border_values.append(
                min(
                    polygon.distance(_nearest_border(border_tree, polygon))
                    for polygon in footprints
                )
            )
            selector_index = _closest(selector_stamps, stamp, SELECTOR_MATCH_NS)
            object_index = bisect_right(object_stamps, stamp) - 1
            distance = math.nan
            if selector_index is not None and object_index >= 0:
                if stamp - object_stamps[object_index] <= OBJECT_MAX_AGE_NS:
                    selected_ids = selectors[selector_index][1]
                    snapshot = objects[object_index][1]
                    if snapshot.header.frame_id != frame:
                        raise ValueError("Tracked-object/map frames differ")
                    shapes = [
                        _object_polygon(obj)
                        for obj in snapshot.objects
                        if bytes(obj.object_id.uuid).hex() in selected_ids
                    ]
                    shapes = [shape for shape in shapes if shape is not None and not shape.is_empty]
                    if shapes:
                        distance = min(
                            polygon.distance(shape) for polygon in footprints for shape in shapes
                        )
            times.append(stamp)
            vehicle_values.append(distance)
        stamps = np.asarray(times, dtype=np.int64)
        output["road_border"][name] = (stamps, np.asarray(border_values, dtype=np.float64))
        output["static_vehicle"][name] = (stamps, np.asarray(vehicle_values, dtype=np.float64))
        LOG.info(
            "Clearance %s: %d trajectories, %d parked-vehicle values",
            name,
            len(times),
            np.isfinite(vehicle_values).sum(),
        )
    return output


def render_clearance_plot(bags, kinds, config, vehicle_model, path, timeline, fps, size):
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    data = compute_clearances(bags, kinds, config.clearance, vehicle_model)
    figure = Figure(figsize=(size[0] / 100, size[1] / 100), dpi=100, facecolor="#101820")
    canvas = FigureCanvasAgg(figure)
    axes = []
    colors = {"MPPI": "#e65acb", "ML planner": "#40d9e6"}
    for index, (category, title) in enumerate(
        (
            ("road_border", "Road border clearance (m)"),
            ("static_vehicle", "Parked vehicle clearance (m)"),
        )
    ):
        axis = figure.add_subplot(2, 1, index + 1)
        axis.set_facecolor("#152431")
        axis.tick_params(colors="white", labelsize=7)
        axis.set_title(title, color="white", fontsize=9, loc="left", pad=3)
        axis.axhline(0, color="#9caab3", linewidth=0.6)
        axis.set_xlim(
            0, config.plot_window_s or max((timeline.end - timeline.start) / NSEC, 1 / fps)
        )
        finite = np.concatenate(
            [values[np.isfinite(values)] for _, values in data[category].values()]
        )
        high = max(0.1, float(finite.max()) * 1.15) if len(finite) else 1.0
        axis.set_ylim(-0.03 * high, high)
        if not len(finite):
            axis.text(
                0.5,
                0.5,
                "No parked targets selected",
                transform=axis.transAxes,
                color="#9caab3",
                ha="center",
                va="center",
                fontsize=8,
            )
        traces = []
        for name in ("MPPI", "ML planner"):
            stamps, values = data[category][name]
            (line,) = axis.plot([], [], label=name, color=colors[name], linewidth=1.7)
            traces.append((line, stamps, (stamps - timeline.start) / NSEC, values))
        axis.legend(loc="upper left", fontsize=7, framealpha=0.2, labelcolor="white", ncol=2)
        cursor = axis.axvline(0, color="white", alpha=0.4, linewidth=0.8)
        axes.append((axis, traces, cursor))
    axes[-1][0].set_xlabel("Seconds from video start", color="white", fontsize=8)
    figure.subplots_adjust(left=0.11, right=0.98, top=0.94, bottom=0.13, hspace=0.48)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create MP4: {path}")
    try:
        for index in range(timeline.count):
            now = timeline.stamp(index)
            elapsed = (now - timeline.start) / NSEC
            for axis, traces, cursor in axes:
                for line, stamps, seconds, values in traces:
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
    LOG.info("Clearance plot: %d frames", timeline.count)
