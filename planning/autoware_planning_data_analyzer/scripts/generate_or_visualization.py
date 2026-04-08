#!/usr/bin/env python3

# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# cspell:disable

"""Generate debug visualization for OR scene trajectory comparison."""

import json
import os
import sys

import matplotlib

matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Import lanelet2 for map visualization
try:
    from lanelet2.io import Origin  # noqa: E402
    from lanelet2.io import load  # noqa: E402

    try:
        from autoware_lanelet2_extension_python.projection import MGRSProjector
    except ImportError:
        try:
            from lanelet2.projection import MGRSProjector
        except ImportError:
            from lanelet2.projection import UtmProjector as MGRSProjector
    LANELET2_AVAILABLE = True
except ImportError:
    LANELET2_AVAILABLE = False
    print("Warning: lanelet2 not available, map visualization disabled")


def load_lanelet_map(map_path):
    """Load lanelet2 map from OSM file."""
    if not LANELET2_AVAILABLE:
        return None
    try:
        projector = MGRSProjector(Origin(0.0, 0.0))
        return load(map_path, projector)
    except Exception as e:
        print(f"Warning: Failed to load map from {map_path}: {e}")
        return None


def get_lanelets_in_area(lanelet_map, x_min, x_max, y_min, y_max):
    """Filter lanelets within visualization bounds."""
    visible_lanelets = []
    for lanelet in lanelet_map.laneletLayer:
        for pt in lanelet.centerline:
            if x_min <= pt.x <= x_max and y_min <= pt.y <= y_max:
                visible_lanelets.append(lanelet)
                break
    return visible_lanelets


def plot_lanelet_boundaries(ax, lanelets):
    """Plot lanelet boundaries as background layer."""
    for lanelet in lanelets:
        left_x = [p.x for p in lanelet.leftBound]
        left_y = [p.y for p in lanelet.leftBound]
        right_x = [p.x for p in lanelet.rightBound]
        right_y = [p.y for p in lanelet.rightBound]

        ax.plot(
            left_x, left_y, color="lightgray", linewidth=1.0, alpha=0.6, linestyle="-", zorder=1
        )
        ax.plot(
            right_x, right_y, color="lightgray", linewidth=1.0, alpha=0.6, linestyle="-", zorder=1
        )

        center_x = [p.x for p in lanelet.centerline]
        center_y = [p.y for p in lanelet.centerline]
        ax.plot(
            center_x,
            center_y,
            color="lightgray",
            linewidth=0.5,
            alpha=0.4,
            linestyle="--",
            zorder=1,
        )


def quat_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def create_bbox_polygon(x, y, yaw, length, width):
    """Create bounding box polygon corners in global frame."""
    # Half dimensions
    half_l = length / 2.0
    half_w = width / 2.0

    # Corners in vehicle frame (front-left, front-right, rear-right, rear-left)
    corners_local = np.array(
        [
            [half_l, half_w],  # Front-left
            [half_l, -half_w],  # Front-right
            [-half_l, -half_w],  # Rear-right
            [-half_l, half_w],  # Rear-left
        ]
    )

    # Rotation matrix
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rot = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

    # Transform to global frame
    corners_global = corners_local @ rot.T
    corners_global[:, 0] += x
    corners_global[:, 1] += y

    return corners_global


def generate_or_visualization(data_json_path, output_image_path):
    """Generate bird's eye view plot of predicted vs GT trajectories."""
    with open(data_json_path, "r") as f:
        data = json.load(f)

    # Extract data
    pred_poses = data["predicted_trajectory"]
    gt_poses = data["ground_truth_trajectory"]
    metrics = data["metrics"]
    event_info = data["event_info"]
    objects = data.get("objects", [])

    # Create figure
    fig, ax = plt.subplots(figsize=(14, 12))

    # Calculate bounds first (needed for map filtering)
    pred_x = [p["x"] for p in pred_poses]
    pred_y = [p["y"] for p in pred_poses]
    gt_x = [p["x"] for p in gt_poses]
    gt_y = [p["y"] for p in gt_poses]

    all_traj_x = pred_x + gt_x
    all_traj_y = pred_y + gt_y

    if not all_traj_x or not all_traj_y:
        print("ERROR: No trajectory data to plot!")
        return

    x_min = min(all_traj_x)
    x_max = max(all_traj_x)
    y_min = min(all_traj_y)
    y_max = max(all_traj_y)

    x_range = x_max - x_min
    y_range = y_max - y_min
    x_epsilon = max(1.0, x_range * 0.1)
    y_epsilon = max(1.0, y_range * 0.1)

    # Load and plot map (background layer, zorder=1)
    map_path = data.get("map_path", None)
    if map_path and os.path.exists(map_path) and LANELET2_AVAILABLE:
        try:
            lanelet_map = load_lanelet_map(map_path)
            if lanelet_map:
                visible_lanelets = get_lanelets_in_area(
                    lanelet_map,
                    x_min - x_epsilon,
                    x_max + x_epsilon,
                    y_min - y_epsilon,
                    y_max + y_epsilon,
                )
                plot_lanelet_boundaries(ax, visible_lanelets)
                print(f"Plotted {len(visible_lanelets)} lanelets from map")
        except Exception as e:
            print(f"Warning: Failed to plot map: {e}")

    # Plot trajectories (already extracted above for bounds calculation)
    # Debug output (optional)
    # print(f"Predicted trajectory: {len(pred_x)} points")
    # print(f"GT trajectory: {len(gt_x)} points")
    # print(f"Objects: {len(objects)}")

    # Make trajectories visible with thinner lines
    ax.plot(gt_x, gt_y, "g-", linewidth=2.5, label="Ground Truth", alpha=0.9, zorder=5)
    ax.plot(
        pred_x, pred_y, "b--", linewidth=2, label="Predicted", alpha=0.9, zorder=5, dashes=(5, 2)
    )

    # Mark start and end points (same size as trajectory line markers)
    ax.plot(gt_x[0], gt_y[0], "go", markersize=8, label="GT Start", zorder=6)
    ax.plot(gt_x[-1], gt_y[-1], "gs", markersize=8, label="GT End", zorder=6)
    ax.plot(pred_x[0], pred_y[0], "bo", markersize=8, label="Pred Start", zorder=6)
    ax.plot(pred_x[-1], pred_y[-1], "bs", markersize=8, label="Pred End", zorder=6)

    # Mark OR event location (vehicle position at OR)
    or_x = event_info["vehicle_x_at_or"]
    or_y = event_info["vehicle_y_at_or"]
    ax.plot(or_x, or_y, "r*", markersize=15, label="OR Event", zorder=10, markeredgewidth=1.5)

    # Plot objects as bounding boxes
    class_colors = {
        1: "cyan",  # CAR
        2: "orange",  # TRUCK
        3: "purple",  # BUS
        6: "yellow",  # BICYCLE
        7: "pink",  # PEDESTRIAN
    }

    class_names = {
        1: "Car",
        2: "Truck",
        3: "Bus",
        6: "Bicycle",
        7: "Pedestrian",
    }

    # Track which object types are present for legend
    present_classes = set()

    for i, obj in enumerate(objects):
        # Get object pose
        obj_x = obj["x"]
        obj_y = obj["y"]

        # Calculate yaw from quaternion
        quat = obj["orientation"]
        yaw = quat_to_yaw(quat["x"], quat["y"], quat["z"], quat["w"])

        # Use footprint if available, otherwise create bbox from dimensions
        if "footprint" in obj and obj["footprint"]:
            # Use provided footprint polygon
            footprint_pts = np.array([[pt["x"], pt["y"]] for pt in obj["footprint"]])
            polygon = patches.Polygon(
                footprint_pts, fill=False, edgecolor="gray", linewidth=1.5, linestyle="-", alpha=0.7
            )
        else:
            # Create bounding box from dimensions
            length = obj["length"]
            width = obj["width"]
            corners = create_bbox_polygon(obj_x, obj_y, yaw, length, width)

            # Get color based on class
            class_label = obj.get("class_label", 0)
            color = class_colors.get(class_label, "gray")
            present_classes.add(class_label)

            polygon = patches.Polygon(
                corners, fill=False, edgecolor=color, linewidth=2.5, linestyle="-", alpha=0.8
            )

        ax.add_patch(polygon)

    # Add metrics text box
    coverage_pct = metrics.get("gt_coverage_ratio", 1.0) * 100
    num_objects = len(objects)

    # Calculate distance between starts
    if pred_x and gt_x:
        start_dist = np.sqrt((pred_x[0] - gt_x[0]) ** 2 + (pred_y[0] - gt_y[0]) ** 2)
    else:
        start_dist = 0.0

    metrics_text = f"""OR Scene Evaluation
Event #{event_info['event_id']}
Prediction: t={metrics['time_relative_to_or']:+.3f}s
Vehicle Speed: {event_info['vehicle_speed_at_or']:.2f} m/s

ADE: {metrics['ade']:.3f} m
FDE: {metrics['fde']:.3f} m
Lateral Dev: {metrics['mean_lateral_deviation']:+.3f} m
Start Dist: {start_dist:.3f} m
Min TTC: {metrics['min_ttc']:.1f} s

Coverage: {coverage_pct:.1f}%
Points: {metrics['num_points']}/{metrics.get('num_points_original', metrics['num_points'])}
Duration: {metrics['trajectory_duration']:.1f}/{metrics.get('trajectory_duration_original', metrics['trajectory_duration']):.1f} s
Objects: {num_objects}"""

    ax.text(
        0.02,
        0.98,
        metrics_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox={"boxstyle": "round", "facecolor": "wheat", "alpha": 0.8},
        family="monospace",
    )

    # Set bounds with epsilon (already calculated above for map filtering)
    ax.set_xlim(x_min - x_epsilon, x_max + x_epsilon)
    ax.set_ylim(y_min - y_epsilon, y_max + y_epsilon)

    # Add object type legend entries (only for classes present in scene)
    for class_id in sorted(present_classes):
        if class_id in class_colors and class_id in class_names:
            ax.plot(
                [],
                [],
                color=class_colors[class_id],
                linewidth=2.5,
                label=class_names[class_id],
                alpha=0.8,
            )

    # Formatting
    ax.set_xlabel("X (meters)", fontsize=12)
    ax.set_ylabel("Y (meters)", fontsize=12)
    ax.set_title(f'OR Scene Trajectory Comparison\n{event_info["bag_name"]}', fontsize=14)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)

    # Save
    plt.tight_layout()
    plt.savefig(output_image_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"Saved visualization to: {output_image_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: generate_or_visualization.py <data.json> <output.png>")
        sys.exit(1)

    generate_or_visualization(sys.argv[1], sys.argv[2])
