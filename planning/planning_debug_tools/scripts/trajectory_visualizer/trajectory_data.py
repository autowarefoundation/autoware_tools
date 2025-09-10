#!/usr/bin/env python3

# Copyright 2025 Tier IV, Inc.
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
from angles import shortest_angular_distance
from autoware_planning_msgs.msg import TrajectoryPoint
from nav_msgs.msg import Odometry
import numpy as np
from rclpy import time
from shapely.geometry import LineString
from shapely.geometry import Point
from tf_transformations import euler_from_quaternion


def _dist(p1: TrajectoryPoint, p2: TrajectoryPoint):
    dx = p1.pose.position.x - p2.pose.position.x
    dy = p1.pose.position.y - p2.pose.position.y
    return np.sqrt(dx**2 + dy**2)


def _triangle_area(p1: TrajectoryPoint, p2: TrajectoryPoint, p3: TrajectoryPoint):
    """Calculate the area of a triangle defined by three 2D points using shoelace formula."""
    return 0.5 * np.abs(
        p1.pose.position.x * (p2.pose.position.y - p3.pose.position.y)
        + p2.pose.position.x * (p3.pose.position.y - p1.pose.position.y)
        + p3.pose.position.x * (p1.pose.position.y - p2.pose.position.y)
    )


def zero_fn(_: Odometry):
    return 0.0


def _get_points(msg):
    # Trajectory: msg.points (TrajectoryPoint)
    # PathWithLaneId: msg.points (PathPointWithLaneId) → .point (PathPoint)
    # Path: msg.points (PathPoint)
    if hasattr(msg, "points") and len(msg.points) > 0:
        # TrajectoryPoint: pose, twist, accel
        if (
            hasattr(msg.points[0], "pose")
            and hasattr(msg.points[0], "twist")
            and hasattr(msg.points[0], "accel")
        ):
            return msg.points
        # PathPointWithLaneId: .point
        elif hasattr(msg.points[0], "point"):
            return [p.point for p in msg.points]
        # PathPoint: pose, longitudinal_velocity_mps
        elif hasattr(msg.points[0], "pose") and hasattr(msg.points[0], "longitudinal_velocity_mps"):
            return msg.points
    return []


def get_velocites(msg):
    points = _get_points(msg)
    if len(points) == 0:
        return []
    if hasattr(points[0], "longitudinal_velocity_mps"):
        return [getattr(p, "longitudinal_velocity_mps", 0.0) for p in points]
    elif hasattr(points[0], "twist"):
        return [getattr(p.twist, "linear", type("obj", (), {"x": 0})).x for p in points]
    return []


def get_velocity(odom: Odometry):
    return odom.twist.twist.linear.x


def get_arc_lengths(msg, zero_pose=None):
    points = _get_points(msg)
    arc_lengths = [0.0]

    # Early return for empty list or single element
    if len(points) <= 1:
        return arc_lengths

    if zero_pose is not None and len(points) > 1:
        ls = LineString([(p.pose.position.x, p.pose.position.y) for p in points])
        p = Point(zero_pose.pose.pose.position.x, zero_pose.pose.pose.position.y)
        arc_lengths = [-ls.project(p)]

    for i in range(0, len(points) - 1):
        arc_lengths.append(arc_lengths[-1] + _dist(points[i], points[i + 1]))
    return arc_lengths


def get_times(msg):
    points = _get_points(msg)
    if len(points) > 0 and not hasattr(points[0], "time_from_start"):
        return [0.0 for _ in points]
    times = []
    for p in points:
        d = time.Duration.from_msg(p.time_from_start)
        times.append(d.nanoseconds * 1e-9)
    return times


def calculate_curvature_2d(msg):
    """
    Calculate the curvature at each point of a 2D trajectory.

    The trajectory is given as a list of (x, y) tuples or an Nx2 NumPy array.
    Curvature is calculated using the formula:
        kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    where derivatives are computed numerically using numpy.gradient, assuming
    a unit spacing for the parameter t (e.g., time or arc length segment).
    """
    points = _get_points(msg)
    num_points = len(points)

    # Handle trivial cases
    if num_points == 0:
        return np.array([])
    if num_points == 1:
        return np.array([0.0])
    # For num_points == 2 (straight line), the formula will yield 0, so no special handling needed.

    x = [p.pose.position.x for p in points]
    y = [p.pose.position.y for p in points]

    # First derivatives (velocity components)
    # np.gradient assumes unit spacing if not specified otherwise
    x_prime = np.gradient(x)
    y_prime = np.gradient(y)

    # Second derivatives (acceleration components)
    x_double_prime = np.gradient(x_prime)
    y_double_prime = np.gradient(y_prime)

    # Numerator of the curvature formula: x'y'' - y'x''
    numerator = x_prime * y_double_prime - y_prime * x_double_prime

    # Denominator term: (x'^2 + y'^2)
    # This is the squared speed.
    speed_squared = x_prime**2 + y_prime**2

    # Denominator of the curvature formula: (x'^2 + y'^2)^(3/2)
    denominator = speed_squared ** (1.5)

    # Calculate curvature
    # Initialize curvature as zeros. This will be the value for points where
    # the denominator is zero (i.e., speed is zero, stationary points).
    curvature = np.zeros_like(denominator)

    # A small epsilon to avoid division by zero and handle near-zero speeds.
    # If speed is very low, the point is practically stationary, and curvature
    # can be considered zero for many practical trajectory applications.
    epsilon = 1e-9

    # Calculate curvature only where the denominator is not close to zero
    valid_indices = denominator > epsilon
    curvature[valid_indices] = np.abs(numerator[valid_indices]) / denominator[valid_indices]

    # For cases where denominator was zero (or near zero) due to zero speed:
    # If numerator was also zero (e.g. single point, or truly stationary), curvature is 0 (already set).
    # If numerator was non-zero and denominator was zero (a true cusp), geometrically curvature is infinite.
    # Here, we've opted to keep it 0 if speed is near zero, which is a common simplification.
    # If you need to represent infinite curvature, you could modify this part:
    # e.g., `curvature[~valid_indices & (np.abs(numerator[~valid_indices]) > epsilon)] = np.inf`

    return curvature


def calculate_menger_curvature(msg):
    """
    Calculate the Menger curvature at each point of a 2D trajectory using triplets of points.

    Menger curvature for points P1, P2, P3 is:
        kappa = (4 * Area(P1, P2, P3)) / (|P1P2| * |P2P3| * |P3P1|)
    The curvature is assigned to the middle point (P2).
    Curvature for the first and last points is set to 0.
    """
    points = _get_points(msg)
    num_points = len(points)

    if num_points == 0:
        return np.array([])
    if num_points < 3:
        return np.zeros(num_points)  # Curvature is 0 for first/last or if not enough points

    curvatures = np.zeros(num_points)
    epsilon = 1e-9  # To avoid division by zero for collinear/coincident points

    for i in range(1, num_points - 1):
        p1 = points[i - 1]
        p2 = points[i]
        p3 = points[i + 1]

        # Calculate side lengths of the triangle
        d12 = _dist(p1, p2)
        d23 = _dist(p2, p3)
        d13 = _dist(p1, p3)

        # If points are coincident or very close, side length can be zero.
        # If any side length is zero, or points are collinear, area will be zero or product of sides will be zero.
        if d12 < epsilon or d23 < epsilon or d13 < epsilon:
            curvatures[i] = 0.0  # Collinear or coincident points
            continue

        # Calculate area of the triangle
        area = _triangle_area(p1, p2, p3)

        # If area is very small, points are nearly collinear.
        if area < epsilon:  # Effectively collinear
            curvatures[i] = 0.0
            continue

        denominator_menger = d12 * d23 * d13
        if denominator_menger < epsilon:  # Should be caught by individual distance checks too
            curvatures[i] = 0.0
            continue

        curvatures[i] = (4 * area) / denominator_menger

    return curvatures


def get_accelerations(msg):
    points = _get_points(msg)
    if len(points) == 0:
        return []
    if hasattr(points[0], "acceleration_mps2"):
        return [getattr(p, "acceleration_mps2", 0.0) for p in points]
    elif hasattr(points[0], "accel"):
        return [getattr(p.accel, "linear", type("obj", (), {"x": 0})).x for p in points]
    return []


def _get_yaw_from_quaternion(quaternion_msg):
    quaternion = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw


def relative_angles(msg):
    points = _get_points(msg)
    angles = [0.0]
    for i in range(0, len(points) - 1):
        yaw0 = _get_yaw_from_quaternion(points[i].pose.orientation)
        yaw1 = _get_yaw_from_quaternion(points[i + 1].pose.orientation)
        angles.append(shortest_angular_distance(yaw0, yaw1))
    return angles


def x_values(msg):
    points = _get_points(msg)
    return [p.pose.position.x for p in points]


def x_value(odom: Odometry):
    return odom.pose.pose.position.x


def y_values(msg):
    points = _get_points(msg)
    return [p.pose.position.y for p in points]


def y_value(odom: Odometry):
    return odom.pose.pose.position.y


def yaws(msg):
    points = _get_points(msg)
    return [_get_yaw_from_quaternion(p.pose.orientation) for p in points]


def yaw(odom: Odometry):
    return _get_yaw_from_quaternion(odom.pose.pose.orientation)


def index_values(msg):
    points = _get_points(msg)
    return list(range(len(points)))


class DataFunction:
    def __init__(self, trajectory_fn, ego_fn):
        self.trajectory_fn = trajectory_fn
        self.ego_fn = ego_fn


"""
return a dictionnary where the key is the name of the function and the value is a pair of the function
"""


def get_data_functions() -> dict:
    return {
        "Index": DataFunction(index_values, zero_fn),
        "Arc Length [m]": DataFunction(get_arc_lengths, zero_fn),
        "Velocity [m/s]": DataFunction(get_velocites, get_velocity),
        "Times [s]": DataFunction(get_times, zero_fn),
        "Curvature (derivatives) [m⁻¹]": DataFunction(calculate_curvature_2d, zero_fn),
        "Curvature (Menger) [m⁻¹]": DataFunction(calculate_menger_curvature, zero_fn),
        "Acceleration [m/s²]": DataFunction(get_accelerations, zero_fn),  # TODO: get accel
        "Relative angles [rad]": DataFunction(relative_angles, zero_fn),
        "X values": DataFunction(x_values, x_value),
        "Y values": DataFunction(y_values, y_value),
        "Yaws [rad]": DataFunction(yaws, yaw),
    }
