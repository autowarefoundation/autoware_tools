#!/usr/bin/env python3

# Copyright 2024 Proxima Technology Inc, TIER IV
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

from collections import deque
import copy
import math
import random
import statistics

from courses.base_course import Base_Course
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor


def safe_acos(value):
    return math.acos(max(-1, min(1, value)))


def calc_tangent_contact_points(x, y, r, a, b, c):
    # Check if the line has a non-zero b to solve for y = (c - ax) / b
    if b != 0:
        # Define coefficients for the quadratic equation A*x^2 + B*x + C = 0
        A = 1 + (a / b) ** 2
        B = -2 * (x + (a * (c - b * y)) / b**2)
        C = x**2 + ((c - b * y) / b) ** 2 - r**2

        # Calculate the discriminant
        abs_discriminant = abs(B**2 - 4 * A * C)

        # Calculate the two x coordinates of the tangent points
        sqrt_discriminant = math.sqrt(abs_discriminant)
        x_contact1 = (-B + sqrt_discriminant) / (2 * A)
        x_contact2 = (-B - sqrt_discriminant) / (2 * A)

        # Corresponding y coordinates for each x
        y_contact1 = (c - a * x_contact1) / b
        y_contact2 = (c - a * x_contact2) / b

    else:
        # If b == 0, we have a vertical line (ax = c), so x is constant
        x_contact1 = x_contact2 = c / a
        # Solve for y coordinates using the circle equation
        y_contact1 = y + math.sqrt(r**2 - (x_contact1 - x) ** 2)
        y_contact2 = y - math.sqrt(r**2 - (x_contact1 - x) ** 2)

    return [(x_contact1, y_contact1), (x_contact2, y_contact2)]


def calc_tangents_and_points(x1, y1, r1, x2, y2, r2):
    dx, dy = x2 - x1, y2 - y1  # Calculate the distance components between circle centers
    d_sq = dx**2 + dy**2  # Square of the distance between centers
    d = math.sqrt(d_sq)  # Distance between centers

    # If centers coincide or one circle is nested within the other, no common tangents exist
    if d == 0 or d < abs(r1 - r2):
        return []

    tangents_and_points = []  # List to store tangents and their contact points

    # Calculate external tangents
    try:
        a = math.atan2(dy, dx)  # Angle between the centers
        b = safe_acos((r1 - r2) / d)  # Adjust angle based on radii difference for external tangents
        t1 = a + b
        t2 = a - b

        # Calculate tangents and contact points for each angle
        for t in [t1, t2]:
            cos_t = math.cos(t)
            sin_t = math.sin(t)

            a = cos_t  # x-coefficient of tangent line
            b = sin_t  # y-coefficient of tangent line
            c = a * x1 + b * y1 + r1  # Line constant term

            # Find contact points on each circle
            points1 = calc_tangent_contact_points(x1, y1, r1, a, b, c)
            points2 = calc_tangent_contact_points(x2, y2, r2, a, b, c)
            if points1 is not None and points2 is not None:
                tangents_and_points.append(
                    {
                        "tangent": (a, b, c),
                        "contact_point1": points1[0],
                        "contact_point2": points2[0],
                    }
                )
    except ValueError:
        pass  # Handle cases where no solution exists for external tangents

    # Calculate internal tangents
    try:
        c = safe_acos((r1 + r2) / d)  # Adjust angle based on radii sum for internal tangents
        t3 = a + c
        t4 = a - c

        # Calculate tangents and contact points for each angle
        for t in [t3, t4]:
            cos_t = math.cos(t)
            sin_t = math.sin(t)

            a = cos_t  # x-coefficient of tangent line
            b = sin_t  # y-coefficient of tangent line
            c = a * x1 + b * y1 + r1  # Line constant term

            # Find contact points on each circle
            points1 = calc_tangent_contact_points(x1, y1, r1, a, b, c)
            points2 = calc_tangent_contact_points(x2, y2, r2, a, b, c)

            if points1 is not None and points2 is not None:
                tangents_and_points.append(
                    {
                        "tangent": (a, b, c),
                        "contact_point1": points1[0],
                        "contact_point2": points2[0],
                    }
                )
    except ValueError:
        pass  # Handle cases where no solution exists for internal tangents

    return tangents_and_points  # Return list of tangents and contact points


def calc_excircle_tangent_to_two_circles(x1, y1, x2, y2, r, r_ext):
    # Distance between the centers of the two circles
    d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # Check for concentric circles, where an excircle is not possible
    if d == 0:
        raise ValueError("An excircle does not exist for concentric circles.")

    # Calculate the center of the excircle
    R_r = r_ext - r  # Difference between the excircle radius and the given circles' radius
    theta = np.arctan2(
        np.sqrt(R_r**2 - (d / 2) ** 2), d / 2
    )  # Angle relative to the line between centers

    # Coordinates of the excircle’s center
    x_center = (x2 - R_r * np.cos(theta) + x1 - R_r * np.cos(np.pi - theta)) / 2
    y_center = (y2 - R_r * np.sin(theta) + y1 - R_r * np.sin(np.pi - theta)) / 2

    # Calculate the contact point coordinates on each circle
    contact_1_x = x1 + r * np.cos(np.pi - theta)
    contact_1_y = y1 + r * np.sin(np.pi - theta)

    contact_2_x = x2 + r * np.cos(theta)
    contact_2_y = y2 + r * np.sin(theta)

    # Return angle of the tangent line, excircle center, and contact points on each circle
    return (
        np.pi - 2 * theta,  # Angle of the tangent line relative to the x-axis
        [x_center, y_center],  # Center of the excircle
        [contact_1_x, contact_1_y],  # Contact point on the first circle
        [contact_2_x, contact_2_y],  # Contact point on the second circle
    )


def calc_two_circles_one_line_trajectory(
    center_point1,
    r1,
    theta_1_start,
    theta_1_end,
    center_point2,
    r2,
    theta_2_start,
    theta_2_end,
    step,
    amplitude=0.001,
):
    # Generate a trajectory connecting two circles and a straight line segment.

    # Calculate the endpoint of the first circle's arc
    B = [center_point1[0] + r1 * np.cos(theta_1_end), center_point1[1] + r1 * np.sin(theta_1_end)]

    # Calculate the starting point of the second circle's arc
    C = [
        center_point2[0] + r2 * np.cos(theta_2_start),
        center_point2[1] + r2 * np.sin(theta_2_start),
    ]

    # Compute lengths of individual segments
    circle_AB = r1 * abs(theta_1_end - theta_1_start)  # Arc length of the first circle
    circle_CD = r2 * abs(theta_2_end - theta_2_start)  # Arc length of the second circle
    BC = np.linalg.norm(np.array(B) - np.array(C))  # Length of the straight-line segment

    # Calculate total trajectory distance
    total_distance = circle_AB + BC + circle_CD

    # Create a time array to parameterize the trajectory
    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")

    # Initialize arrays to store trajectory points and attributes
    x = np.zeros(len(t_array))
    y = np.zeros(len(t_array))
    yaw = np.zeros(len(t_array))
    curvature = np.zeros(len(t_array))
    achievement_rates = np.zeros(len(t_array))
    parts = np.zeros(len(t_array), dtype=object)
    i_end = len(t_array)

    # Generate trajectory points
    for i, t in enumerate(t_array):
        if t > total_distance:  # Stop if total distance is exceeded
            i_end = i
            break

        if t <= circle_AB:  # Trajectory on the first circle
            t1 = t
            alpha = t1 / circle_AB
            t1_rad = (1 - alpha) * theta_1_start + alpha * theta_1_end  # Interpolate angle
            x[i] = center_point1[0] + r1 * np.cos(t1_rad)
            y[i] = center_point1[1] + r1 * np.sin(t1_rad)
            achievement_rates[i] = t / total_distance
            parts[i] = "circle"  # Label as circle segment

        elif circle_AB < t <= circle_AB + BC:  # Trajectory on the straight line
            t2 = t - circle_AB
            alpha = t2 / BC
            x[i] = (1 - alpha) * B[0] + alpha * C[0]  # Linear interpolation for x
            y[i] = (1 - alpha) * B[1] + alpha * C[1]  # Linear interpolation for y
            achievement_rates[i] = t / total_distance
            parts[i] = "linear"  # Label as linear segment

        elif circle_AB + BC < t <= total_distance:  # Trajectory on the second circle
            t3 = t - (circle_AB + BC)
            alpha = t3 / circle_CD
            t3_rad = (1 - alpha) * theta_2_start + alpha * theta_2_end  # Interpolate angle
            x[i] = center_point2[0] + r2 * np.cos(t3_rad)
            y[i] = center_point2[1] + r2 * np.sin(t3_rad)
            achievement_rates[i] = t / total_distance
            parts[i] = "circle"  # Label as circle segment

    # Trim arrays to the valid trajectory length
    x = x[:i_end]
    y = y[:i_end]
    parts = parts[:i_end]
    achievement_rates = achievement_rates[:i_end] / achievement_rates[-1]  # Normalize progress

    # Add sinusoidal perturbations to the trajectory
    delta_x = amplitude * np.sin(4.0 * np.pi * achievement_rates) * np.sin(yaw)
    delta_y = amplitude * np.sin(4.0 * np.pi * achievement_rates) * np.cos(yaw)
    x += delta_x
    y += delta_y

    # Calculate derivatives for yaw and curvature
    dx = (x[1:] - x[:-1]) / step
    dy = (y[1:] - y[:-1]) / step
    ddx = (dx[1:] - dx[:-1]) / step
    ddy = (dy[1:] - dy[:-1]) / step

    yaw = np.arctan2(dy, dx)
    yaw = np.array(yaw.tolist() + [yaw[-1]])  # Append last value for consistent length

    curvature = (
        1e-9 + abs(ddx * dy[:-1] - ddy * dx[:-1]) / (dx[:-1] ** 2 + dy[:-1] ** 2 + 1e-9) ** 1.5
    )
    curvature = np.array(curvature.tolist() + [curvature[-2], curvature[-1]])  # Append values

    yaw = yaw[:i_end]
    curvature = curvature[:i_end]

    # Return trajectory details
    return np.array([x, y]).T, yaw, curvature, parts, achievement_rates, total_distance


def calc_two_circles_excircle_trajectory(
    center_point1,
    r1,
    theta_1_start,
    theta_1_end,
    center_point2,
    r2,
    theta_2_start,
    theta_2_end,
    excircle_center_point,
    ext_r,
    theta_excircle_start,
    theta_excircle_end,
    step,
    amplitude=0.001,
):
    # Generate a trajectory connecting two circular arcs and an excircle arc.

    # Calculate arc lengths of the first circle, excircle, and second circle
    circle_AB = r1 * abs(theta_1_end - theta_1_start)  # Arc length of the first circle
    circle_BC = ext_r * abs(theta_excircle_end - theta_excircle_start)  # Arc length of the excircle
    circle_CD = r2 * abs(theta_2_end - theta_2_start)  # Arc length of the second circle

    # Calculate the total distance of the trajectory
    total_distance = circle_AB + circle_BC + circle_CD

    # Create a time array for parameterizing the trajectory
    t_array = np.arange(start=0.0, stop=total_distance, step=step).astype("float")

    # Initialize arrays to store trajectory details
    x = np.zeros(len(t_array))  # x-coordinates
    y = np.zeros(len(t_array))  # y-coordinates
    yaw = np.zeros(len(t_array))  # yaw angles
    curvature = np.zeros(len(t_array))  # curvature values
    achievement_rates = np.zeros(len(t_array))  # progress rates
    parts = np.zeros(len(t_array), dtype=object)  # trajectory part labels

    i_end = len(t_array)  # Valid index range for trajectory points

    # Generate trajectory points
    for i, t in enumerate(t_array):
        if t > total_distance:  # Stop if total distance is exceeded
            i_end = i
            break

        if t <= circle_AB:  # On the first circle arc
            t1 = t
            alpha = t1 / circle_AB
            t1_rad = (1 - alpha) * theta_1_start + alpha * theta_1_end  # Interpolate angle
            x[i] = center_point1[0] + r1 * np.cos(t1_rad)
            y[i] = center_point1[1] + r1 * np.sin(t1_rad)
            achievement_rates[i] = t / total_distance
            parts[i] = "circle"  # Label as circle part

        elif circle_AB < t <= circle_AB + circle_BC:  # On the excircle arc
            kappa = 0.0 / ext_r  # Optional curvature scaling factor (not used here)
            t2 = t - circle_AB
            alpha = t2 / circle_BC
            t2_rad = (
                1 - alpha
            ) * theta_excircle_start + alpha * theta_excircle_end  # Interpolate angle
            x[i] = excircle_center_point[0] + (
                1.0 + kappa * 4 * alpha * (1 - alpha) * 4 * alpha * (1 - alpha)
            ) * ext_r * np.cos(t2_rad)
            y[i] = excircle_center_point[1] + (
                1.0 + kappa * 4 * alpha * (1 - alpha) * 4 * alpha * (1 - alpha)
            ) * ext_r * np.sin(t2_rad)
            achievement_rates[i] = t / total_distance
            parts[i] = "linear"  # Label as linear part (for simplicity)

        elif circle_AB + circle_BC < t <= total_distance:  # On the second circle arc
            t3 = t - (circle_AB + circle_BC)
            alpha = t3 / circle_CD
            t3_rad = (1 - alpha) * theta_2_start + alpha * theta_2_end  # Interpolate angle
            x[i] = center_point2[0] + r2 * np.cos(t3_rad)
            y[i] = center_point2[1] + r2 * np.sin(t3_rad)
            achievement_rates[i] = t / total_distance
            parts[i] = "circle"  # Label as circle part

    # Trim arrays to the valid trajectory range
    x = x[:i_end]
    y = y[:i_end]
    parts = parts[:i_end]
    achievement_rates = achievement_rates[:i_end]

    # Add sinusoidal perturbations to the trajectory
    delta_x = (
        amplitude * np.sin(4.0 * np.pi * achievement_rates)
        + 0.05 * amplitude * np.sin(16.0 * np.pi * achievement_rates)
    ) * np.sin(yaw)
    delta_y = (
        amplitude * np.sin(4.0 * np.pi * achievement_rates)
        + 0.05 * amplitude * np.sin(16.0 * np.pi * achievement_rates)
    ) * np.cos(yaw)
    x += delta_x
    y += delta_y

    # Compute derivatives for yaw and curvature
    dx = (x[1:] - x[:-1]) / step
    dy = (y[1:] - y[:-1]) / step
    ddx = (dx[1:] - dx[:-1]) / step
    ddy = (dy[1:] - dy[:-1]) / step

    yaw = np.arctan2(dy, dx)
    yaw = np.array(yaw.tolist() + [yaw[-1]])  # Append the last value for consistency

    curvature = (
        1e-9 + abs(ddx * dy[:-1] - ddy * dx[:-1]) / (dx[:-1] ** 2 + dy[:-1] ** 2 + 1e-9) ** 1.5
    )
    curvature = np.array(curvature.tolist() + [curvature[-2], curvature[-1]])  # Append values
    yaw = yaw[:i_end]
    curvature = curvature[:i_end]

    # Return trajectory details
    return np.array([x, y]).T, yaw, curvature, parts, achievement_rates, total_distance


class TrajectorySegment:
    def __init__(
        self,
        segment_type: str,
        trajectory,
        yaw,
        curvature,
        parts,
        achievement_rates,
        in_direction: str,
        out_direction: str,
        reversing: bool,
        distance: float,
    ):
        # Validation for segment_type
        if segment_type not in ["boundary", "non_boundary"]:
            raise ValueError("segment_type must be 'boundary' or 'non_boundary'")

        # Validation for in_direction and out_direction
        if in_direction not in ["clock_wise", "counter_clock_wise"]:
            raise ValueError("in_direction must be 'clock_wise' or 'counter_clock_wise'")
        if out_direction not in ["clock_wise", "counter_clock_wise"]:
            raise ValueError("out_direction must be 'clock_wise' or 'counter_clock_wise'")

        self.type = segment_type
        self.trajectory = trajectory
        self.yaw = yaw
        self.curvature = curvature
        self.parts = parts
        self.achievement_rates = achievement_rates
        self.in_direction = in_direction
        self.out_direction = out_direction
        self.reversing = reversing
        self.distance = distance

    def __repr__(self):
        return (
            f"TrajectorySegment(type={self.type}, trajectory={self.trajectory}, yaw={self.yaw}, "
            f"curvature={self.curvature}, parts={self.parts}, achievement_rates={self.achievement_rates}, "
            f"in_direction={self.in_direction}, out_direction={self.out_direction}, reversing={self.reversing}), distance={self.distance}"
        )


def calc_two_circles_common_tangent_trajectory(R, r, step, amplitude=0.001):
    # Case 1: Smaller circle is inside the larger circle (r < R)
    if r < R:
        # Define centers of the two circles
        center_1 = [-(R - r), 0.0]  # Center of the first circle
        center_2 = [(R - r), 0.0]  # Center of the second circle

        # Calculate tangents and contact points between the two circles
        tangents_and_points = calc_tangents_and_points(
            center_1[0], center_1[1], r, center_2[0], center_2[1], r
        )

        # Iterate through the possible tangents
        for tangent_and_point in tangents_and_points:
            contact_point1 = tangent_and_point["contact_point1"]
            contact_point2 = tangent_and_point["contact_point2"]

            # Condition 1: Specific tangent selection based on y-coordinates and radius
            if contact_point1[1] > 0.0 and contact_point2[1] < 0.0 and r < R / 2:
                # Compute angles for the start and end of the trajectory
                theta_1_end = np.arctan2(
                    contact_point1[1] - center_1[1], contact_point1[0] - center_1[0]
                )
                theta_2_start = np.arctan2(
                    contact_point2[1] - center_2[1], contact_point2[0] - center_2[0]
                )

                # Calculate the trajectory for this configuration
                (
                    trajectory,
                    yaw,
                    curvature,
                    parts,
                    achievement_rates,
                    total_distance,
                ) = calc_two_circles_one_line_trajectory(
                    center_1,
                    r,
                    np.pi,
                    theta_1_end,
                    center_2,
                    r,
                    theta_2_start,
                    0.0,
                    step=step,
                    amplitude=amplitude,
                )

                # Return a TrajectorySegment object with all calculated details
                return TrajectorySegment(
                    "non_boundary",
                    trajectory,
                    yaw,
                    curvature,
                    parts,
                    achievement_rates,
                    "clock_wise",
                    "counter_clock_wise",
                    True,
                    total_distance,
                )

            # Condition 2: Alternative tangent selection for a different configuration
            if contact_point1[1] > 0.0 and contact_point2[1] > 0.0 and R / 2 <= r < R:
                # Compute angles for the start and end of the trajectory
                theta_1_end = np.arctan2(
                    contact_point1[1] - center_1[1], contact_point1[0] - center_1[0]
                )
                theta_2_start = np.arctan2(
                    contact_point2[1] - center_2[1], contact_point2[0] - center_2[0]
                )

                # Calculate the trajectory for this configuration
                (
                    trajectory,
                    yaw,
                    curvature,
                    parts,
                    achievement_rates,
                    total_distance,
                ) = calc_two_circles_one_line_trajectory(
                    center_1,
                    r,
                    np.pi,
                    theta_1_end,
                    center_2,
                    r,
                    theta_2_start,
                    0.0,
                    step=step,
                    amplitude=amplitude,
                )

                # Return a TrajectorySegment object with all calculated details
                return TrajectorySegment(
                    "non_boundary",
                    trajectory,
                    yaw,
                    curvature,
                    parts,
                    achievement_rates,
                    "clock_wise",
                    "clock_wise",
                    False,
                    total_distance,
                )

    # Case 2: Smaller circle is outside or overlaps the larger circle (R <= r)
    elif R <= r:
        # Define centers for the two circles
        center_1 = [-(R - R * 0.75), 0.0]  # Adjusted center of the first circle
        center_2 = [(R - R * 0.75), 0.0]  # Adjusted center of the second circle

        # Calculate external tangent and points for the excircle
        (
            theta,
            (x_center, y_center),
            (contact_1_x, contact_1_y),
            (contact_2_x, contact_2_y),
        ) = calc_excircle_tangent_to_two_circles(
            center_1[0], center_1[1], center_2[0], center_2[1], R * 0.75, r
        )

        # Compute angles for the start and end of the trajectory
        theta_1_end = np.arctan2(contact_1_y - center_1[1], contact_1_x - center_1[0])
        theta_2_start = np.arctan2(contact_2_y - center_2[1], contact_2_x - center_2[0])

        # Calculate the trajectory for the excircle configuration
        (
            trajectory,
            yaw,
            curvature,
            parts,
            achievement_rates,
            total_distance,
        ) = calc_two_circles_excircle_trajectory(
            center_point1=center_1,
            r1=R * 0.75,
            theta_1_start=np.pi,
            theta_1_end=theta_1_end,
            center_point2=center_2,
            r2=R * 0.75,
            theta_2_start=theta_2_start,
            theta_2_end=0.0,
            excircle_center_point=[x_center, y_center],
            ext_r=r,
            theta_excircle_start=np.pi - (np.pi - theta) / 2,
            theta_excircle_end=(np.pi - theta) / 2,
            step=step,
            amplitude=amplitude,
        )

        # Return a TrajectorySegment object with all calculated details
        return TrajectorySegment(
            "non_boundary",
            trajectory,
            yaw,
            curvature,
            parts,
            achievement_rates,
            "clock_wise",
            "clock_wise",
            False,
            total_distance,
        )

    # Return None if no valid trajectory can be calculated
    return None


def calc_boundary_trajectory(R):
    # Define the angular change for the boundary trajectory
    delta_theta = 1.0  # Angle span in radians
    circle = R * delta_theta  # Arc length of the circular segment

    # Total distance of the trajectory
    total_distance = circle

    # Generate a time array for parameterization
    t_array = np.arange(start=0.0, stop=total_distance, step=0.10).astype("float")

    # Initialize trajectory arrays
    x = np.zeros(len(t_array))  # X-coordinates of the trajectory
    y = np.zeros(len(t_array))  # Y-coordinates of the trajectory
    yaw = np.zeros(len(t_array))  # Yaw (orientation) at each point
    curvature = np.zeros(len(t_array))  # Curvature at each point
    achievement_rates = np.zeros(len(t_array))  # Progress along the trajectory
    parts = np.zeros(len(t_array), dtype=object)  # Labels for trajectory parts

    i_end = len(t_array)  # Index where the trajectory ends

    # Define the start and end angles for the circular arc
    theta_start = np.pi  # Starting angle
    theta_end = np.pi - delta_theta  # Ending angle

    # Generate the trajectory
    for i, t in enumerate(t_array):
        if t > total_distance:  # Stop if the distance exceeds the total arc length
            i_end = i
            break

        if t <= circle:  # Arc trajectory generation
            t1 = t  # Distance traveled along the arc
            alpha = t1 / circle  # Normalized progress along the arc
            t1_rad = (1 - alpha) * theta_start + alpha * theta_end  # Interpolated angle

            # Calculate X, Y coordinates
            x[i] = R * np.cos(t1_rad)
            y[i] = R * np.sin(t1_rad)

            # Calculate tangent vectors for yaw computation
            tangent_x = (
                np.cos(np.pi / 2 + t1_rad)
                * (theta_end - theta_start)
                / abs(theta_end - theta_start)
            )
            tangent_y = (
                np.sin(np.pi / 2 + t1_rad)
                * (theta_end - theta_start)
                / abs(theta_end - theta_start)
            )
            yaw[i] = math.atan2(tangent_y, tangent_x)  # Calculate yaw from tangent

            # Set curvature (constant for circular arc)
            curvature[i] = 1 / R

            # Progress along the trajectory as a fraction of total distance
            achievement_rates[i] = t / total_distance

            # Label this segment as part of the "circle"
            parts[i] = "circle"

    # Trim arrays to the valid trajectory length
    x = x[:i_end]
    y = y[:i_end]
    yaw = yaw[:i_end]
    curvature = curvature[:i_end]
    parts = parts[:i_end]
    achievement_rates = achievement_rates[:i_end]

    # Return the trajectory segment object
    return TrajectorySegment(
        "boundary",  # Segment type
        np.array([x, y]).T,  # Trajectory points
        yaw,  # Yaw angles
        curvature,  # Curvatures
        parts,  # Segment labels
        achievement_rates,  # Achievement rates
        "clock_wise",  # Direction of rotation for this arc
        "clock_wise",  # Direction of rotation for a hypothetical next arc
        False,  # Flag for boundary inclusion
        total_distance,  # Total distance of the trajectory
    )


def reverse_trajectory_segment(trajectory_segment):
    # Define the rotation matrix to reverse the trajectory
    # This flips the trajectory along the X-axis
    Rot = np.array([[1.0, 0.0], [0.0, -1.0]])

    # Reverse direction initialization
    # Start by assuming the reversed directions are "clock_wise"
    in_direction_reversed = "clock_wise"
    out_direction_reversed = "clock_wise"

    # Reverse the "in_direction"
    if trajectory_segment.in_direction == "clock_wise":
        in_direction_reversed = "counter_clock_wise"
    elif trajectory_segment.in_direction == "counter_clock_wise":
        in_direction_reversed = "clock_wise"

    # Reverse the "out_direction"
    if trajectory_segment.out_direction == "clock_wise":
        out_direction_reversed = "counter_clock_wise"
    elif trajectory_segment.out_direction == "counter_clock_wise":
        out_direction_reversed = "clock_wise"

    # Create a deep copy of the input trajectory segment
    # This ensures the original trajectory_segment remains unmodified
    trajectory_segment_reversed = copy.deepcopy(trajectory_segment)

    # Apply the rotation to reverse the trajectory
    # Multiply the trajectory's coordinates by the rotation matrix
    trajectory_segment_reversed.trajectory = trajectory_segment.trajectory @ Rot

    # Reverse the yaw (orientation) for all points in the trajectory
    trajectory_segment_reversed.yaw = -trajectory_segment.yaw

    # Update the reversed directions
    trajectory_segment_reversed.in_direction = in_direction_reversed
    trajectory_segment_reversed.out_direction = out_direction_reversed

    # Return the reversed trajectory segment
    return trajectory_segment_reversed


def declare_reversal_loop_circle_params(node):
    node.declare_parameter(
        "trajectory_radius",
        35.0,
        ParameterDescriptor(
            description="Radius of the circle where trajectories are generated [m]"
        ),
    )

    node.declare_parameter(
        "enclosing_radius",
        40.0,
        ParameterDescriptor(
            description="Radius of the circle enclosing the generated trajectories [m]"
        ),
    )

    node.declare_parameter(
        "look_ahead_distance",
        15.0,
        ParameterDescriptor(
            description="The distance referenced ahead of the vehicle for collecting steering angle data"
        ),
    )


class Reversal_Loop_Circle(Base_Course):
    def __init__(self, step: float, param_dict):
        # Initialize the Reversal_Loop_Circle class and set up trajectories for straight, turning, and boundary paths.

        # Call the constructor of the parent class (Base_Course).
        super().__init__(step, param_dict)

        # Initialize general parameters.
        self.closed = False  # Indicates whether the trajectory is closed.
        self.wheel_base = param_dict["wheel_base"]  # Vehicle's wheel base.
        self.trajectory_R = param_dict[
            "trajectory_radius"
        ]  # Circle radius for trajectory generation.
        self.enclosing_R = param_dict["enclosing_radius"]
        self.vel_hist = deque([float(0.0)] * 30, maxlen=30)  # Velocity history for smoothing.
        self.previous_updated_time = 0.0  # Timestamp for tracking updates.

        # Initialize velocity and acceleration targets for trajectory segmentation.
        self.target_vel_on_segmentation = 6.0  # Target velocity.
        self.target_acc_on_segmentation = 0.0  # Target acceleration.
        self.vel_idx, self.acc_idx = 0, 0  # Indices for velocity and acceleration bins.

        # Set the initial vehicle phase and other state variables.
        self.vehicle_phase = "acceleration"  # Vehicle's current motion phase.
        self.const_velocity_start_time = 0.0  # Start time for constant velocity phase.
        self.updated_target_velocity = (
            False  # Indicates whether the target velocity has been updated.
        )

        self.look_ahead_distance = param_dict[
            "look_ahead_distance"
        ]  # Distance to look ahead for collecting steer angle.

        # Generate nearly straight trajectories inside the circle.

        self.steer_list = [
            0.001,
            0.01,
            0.02,
            0.03,
        ]  # List of steering angles for nearly straight paths.
        self.trajectory_nearly_straight_clock_wise = (
            {}
        )  # Clockwise trajectories for nearly straight paths.
        self.trajectory_nearly_straight_counter_clock_wise = (
            {}
        )  # Counterclockwise trajectories for nearly straight paths.

        for i in range(len(self.steer_list)):
            # Calculate the radius of curvature for a given steering angle.
            r = self.wheel_base / np.tan(self.steer_list[i])

            # Generate and store clockwise trajectories.
            trajectory = calc_two_circles_common_tangent_trajectory(
                self.trajectory_R, r, step=self.step
            )
            self.trajectory_nearly_straight_clock_wise[self.steer_list[i]] = trajectory

            # Generate and store counterclockwise trajectories by reversing the clockwise trajectory.
            self.trajectory_nearly_straight_counter_clock_wise[
                self.steer_list[i]
            ] = reverse_trajectory_segment(trajectory)

        # Generate trajectories for changing directions (turning).

        self.trajectory_for_turning = {}  # Store trajectories for direction changes.
        self.amplitude_list = []  # List of amplitude values for turning.
        self.turning_steer_list = []  # List of steering angles for turning.

        for i in range(200):
            amplitude = 0.1 * i  # Gradually increase the amplitude for trajectory variations.
            self.amplitude_list.append(str(amplitude))

            # Create a trajectory dictionary for transitions between directions.
            trajectory_for_turning__ = {}
            r = self.trajectory_R / 2 * 0.75  # Use a reduced radius for sharper turns.
            trajectory = calc_two_circles_common_tangent_trajectory(
                self.trajectory_R, r, step=self.step, amplitude=amplitude
            )

            # Generate clockwise-to-counterclockwise turning trajectories.
            trajectory_for_turning__["clock_wise_to_counter_clock_wise"] = trajectory

            # Generate counterclockwise-to-clockwise turning trajectories by reversing.
            trajectory_reversed = reverse_trajectory_segment(trajectory)
            trajectory_for_turning__["counter_clock_wise_to_clock_wise"] = trajectory_reversed

            # Store the trajectories for turning.
            self.trajectory_for_turning[str(amplitude)] = trajectory_for_turning__

            # Calculate and store the median turning steering angle.
            self.turning_steer_list.append(
                np.arctan2(
                    (
                        0.5 * statistics.median(trajectory_reversed.curvature)
                        + 0.5 * np.max(trajectory_reversed.curvature)
                    )
                    * self.wheel_base,
                    1.0,
                )
            )

        # Generate boundary trajectories.

        self.boundary_trajectory = {}  # Store boundary trajectories.

        # Generate clockwise boundary trajectory.
        trajectory = calc_boundary_trajectory(self.trajectory_R)
        self.boundary_trajectory["clock_wise"] = trajectory

        # Generate counterclockwise boundary trajectory by reversing.
        trajectory_reversed = reverse_trajectory_segment(trajectory)
        self.boundary_trajectory["counter_clock_wise"] = trajectory_reversed

        # Initialize the trajectory list with a boundary trajectory.

        self.trajectory_list = [
            self.boundary_trajectory["clock_wise"]
        ]  # Start with a clockwise boundary trajectory.
        self.end_point = self.trajectory_list[-1].trajectory[
            -1
        ]  # Endpoint of the initial trajectory.
        self.latest_direction = "clock_wise"  # Initial direction.
        self.trajectory_length_list = [
            len(self.trajectory_list[-1].trajectory)
        ]  # Track trajectory lengths.

        # Add initial straight and turning trajectories to the trajectory list.
        while len(self.trajectory_length_list) < 3:
            self.add_trajectory_nearly_straight()  # Add nearly straight trajectories.
            self.add_trajectory_nearly_straight()
            steer_of_trajectory = 1e-9  # Minimal steer value for turning initialization.
            self.add_trajectory_for_turning(steer_of_trajectory)

    def add_trajectory(self, trajectory):
        # Add a new trajectory to the trajectory list, ensuring proper alignment and continuity.

        # If the last trajectory and the new trajectory are both non-boundary types,
        # add a boundary trajectory to ensure proper alignment.
        if self.trajectory_list[-1].type == "non_boundary" and trajectory.type == "non_boundary":
            self.add_trajectory(self.boundary_trajectory[self.latest_direction])

        # If the incoming trajectory's direction is different from the latest trajectory's direction,
        # and the latest trajectory is a boundary type, add a turning trajectory to connect them.
        if (
            trajectory.in_direction is not self.latest_direction
            and self.trajectory_list[-1].type == "boundary"
        ):
            key_ = self.latest_direction + "_to_" + getattr(trajectory, "in_direction")
            self.add_trajectory(self.trajectory_for_turning[self.amplitude_list[0]][key_])

        # Ensure proper alignment if there are consecutive non-boundary trajectories.
        if self.trajectory_list[-1].type == "non_boundary" and trajectory.type == "non_boundary":
            self.add_trajectory(self.boundary_trajectory[self.latest_direction])

        # Compute rotation to align the new trajectory with the end of the last trajectory.
        cos = self.end_point[0] / self.trajectory_R  # Cosine of the angle for alignment
        sin = self.end_point[1] / self.trajectory_R  # Sine of the angle for alignment
        delta_theta = np.arctan2(sin, cos) - np.pi  # Rotation angle
        Rotation = (-np.eye(2)) @ np.array([[cos, -sin], [sin, cos]])  # Rotation matrix

        # Create a modified copy of the input trajectory for alignment.
        trajectory_modified = copy.deepcopy(trajectory)
        trajectory_modified.trajectory = (Rotation @ trajectory.trajectory.T).T
        trajectory_modified.yaw = trajectory.yaw + delta_theta

        # Update the latest direction and endpoint for the trajectory.
        self.latest_direction = trajectory.out_direction
        self.end_point = trajectory_modified.trajectory[-1]

        # Add the modified trajectory to the list and update the length list.
        self.trajectory_list.append(trajectory_modified)
        self.trajectory_length_list.append(len(trajectory_modified.trajectory))

    def add_trajectory_nearly_straight(self):
        # Add a nearly straight trajectory to the trajectory list, depending on the current direction.

        if self.latest_direction == "clock_wise":
            # Add a trajectory for a clockwise direction using a random steering angle.
            self.add_trajectory(
                self.trajectory_nearly_straight_clock_wise[random.choice(self.steer_list)]
            )
        elif self.latest_direction == "counter_clock_wise":
            # Add a trajectory for a counterclockwise direction using a random steering angle.
            self.add_trajectory(
                self.trajectory_nearly_straight_counter_clock_wise[random.choice(self.steer_list)]
            )

    def remove_trajectory(self):
        # Remove the oldest trajectory from the trajectory list to manage memory and data size.

        self.trajectory_list.pop(0)
        self.trajectory_length_list.pop(0)

    def add_trajectory_for_turning(self, steer):
        # Add a trajectory for transitioning between clockwise and counterclockwise directions.

        # Find the index of the closest available steering angle in the turning list.
        index = (np.abs(np.array(self.turning_steer_list) - steer)).argmin()

        if self.latest_direction == "clock_wise":
            # Add a trajectory for transitioning from clockwise to counterclockwise direction.
            self.add_trajectory(
                self.trajectory_for_turning[self.amplitude_list[index]][
                    self.latest_direction + "_to_" + "counter_clock_wise"
                ]
            )
        elif self.latest_direction == "counter_clock_wise":
            # Add a trajectory for transitioning from counterclockwise to clockwise direction.
            self.add_trajectory(
                self.trajectory_for_turning[self.amplitude_list[index]][
                    self.latest_direction + "_to_" + "clock_wise"
                ]
            )

    def get_trajectory_points(
        self,
        long_side_length: float,
        short_side_length: float,
        ego_point=np.array([0.0, 0.0]),
        goal_point=np.array([0.0, 0.0]),
    ):
        # Concatenate the trajectory points from the first four trajectory segments
        self.trajectory_points = np.concatenate(
            [self.trajectory_list[i].trajectory for i in range(4)],
            axis=0,
        )
        # Concatenate yaw angles from the first four trajectory segments
        self.yaw = np.concatenate(
            [self.trajectory_list[i].yaw for i in range(4)],
            axis=0,
        )
        # Concatenate parts (labels) from the first four trajectory segments
        self.parts = np.concatenate([self.trajectory_list[i].parts for i in range(4)], axis=0)
        # Concatenate curvatures from the first four trajectory segments
        self.curvature = np.concatenate(
            [self.trajectory_list[i].curvature for i in range(4)], axis=0
        )
        # Concatenate achievement rates from the first four trajectory segments
        self.achievement_rates = np.concatenate(
            [self.trajectory_list[i].achievement_rates for i in range(4)],
            axis=0,
        )

    def get_target_velocity(
        self,
        nearestIndex,
        current_time,
        current_vel,
        current_acc,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
        mask_vel_acc,
        mask_vel_steer
    ):
        # Calculate the target velocity for the vehicle based on its current state, trajectory phase, and constraints such as acceleration limits and trajectory curvature.

        # Initialize target acceleration and velocity if not already updated
        if not self.updated_target_velocity:
            # Choose velocity and acceleration bins based on collected data
            self.acc_idx, self.vel_idx = self.choose_target_velocity_acc(
                collected_data_counts_of_vel_acc,
                mask_vel_acc
            )
            self.target_acc_on_segmentation = self.params.a_bin_centers[self.acc_idx]
            self.target_vel_on_segmentation = self.params.v_bin_centers[self.vel_idx]
            self.acc_on_line = self.target_acc_on_segmentation

            # Set the vehicle's phase to "acceleration"
            self.vehicle_phase = "acceleration"
            self.updated_target_velocity = True

        acc_kp_of_pure_pursuit = self.params.acc_kp  # Proportional gain for acceleration control
        T = 10.0  # Period of the sine wave used to modulate velocity
        sine = np.sin(2 * np.pi * current_time / T)  # Sine wave for smooth velocity modulation

        # Handle acceleration phase
        if self.vehicle_phase == "acceleration":
            if current_vel < self.target_vel_on_segmentation - 1.0 * abs(
                self.target_acc_on_segmentation
            ):
                # Increase velocity with a maximum allowable acceleration
                target_vel = current_vel + self.params.a_max / acc_kp_of_pure_pursuit * (
                    1.25 + 0.50 * sine
                )
            else:
                # Increase velocity with a absolute target acceleration
                target_vel = current_vel + abs(
                    self.target_acc_on_segmentation
                ) / acc_kp_of_pure_pursuit * (1.25 + 0.50 * sine)

            # Transition to "constant speed" phase once the target velocity is reached
            if current_vel > self.target_vel_on_segmentation:
                self.vehicle_phase = "constant_speed"
                self.const_velocity_start_time = current_time

        # Handle constant speed phase
        if self.vehicle_phase == "constant_speed":
            # Modulate velocity around the target with a sine wave
            target_vel = self.target_vel_on_segmentation + 2.0 * (-0.5 + 1.0 * sine)

            # Transition to "deceleration" phase after a fixed duration
            if current_time - self.const_velocity_start_time > T:
                self.vehicle_phase = "deceleration"

        # Handle deceleration phase
        if self.vehicle_phase == "deceleration":
            if current_vel < self.target_vel_on_segmentation - 1.0 * abs(
                self.target_acc_on_segmentation
            ):
                # Decrease velocity with a maximum deceleration
                target_vel = current_vel - self.params.a_max / acc_kp_of_pure_pursuit * (
                    1.25 + 0.50 * sine
                )
            else:
                # Decrease velocity with a absolute target acceleration
                target_vel = current_vel - abs(
                    self.target_acc_on_segmentation
                ) / acc_kp_of_pure_pursuit * (1.25 + 0.50 * sine)

            # Reset velocity update flag when deceleration is complete
            if (
                current_vel
                < self.target_vel_on_segmentation
                - 1.0 * abs(self.target_acc_on_segmentation) / acc_kp_of_pure_pursuit
            ):
                self.updated_target_velocity = False

        # Maintain a smoothed velocity by averaging recent values
        self.vel_hist.append(target_vel)
        target_vel = np.mean(self.vel_hist)

        # Special handling for trajectory direction changes
        if self.trajectory_list[2].in_direction is not self.trajectory_list[2].out_direction:
            # Set a fixed target velocity during direction transitions
            target_vel = 3.0 + 1.0 * sine

        # Adjust velocity based on trajectory curvature and lateral acceleration constraints
        if (self.trajectory_list[1].in_direction is not self.trajectory_list[1].out_direction) or (
            self.trajectory_list[2].in_direction is not self.trajectory_list[2].out_direction
        ):
            max_curvature_on_segment = 1e-9  # Initialize to a small value to avoid division by zero
            max_lateral_vel_on_segment = 1e9  # Initialize to a large value as a placeholder

            # Compute the maximum curvature and lateral velocity in a look-ahead segment
            if self.yaw is not None and self.curvature is not None:
                steer_sign = np.sign(
                    self.yaw[nearestIndex + 1] - self.yaw[nearestIndex]
                )  # Steering direction
                max_curvature_on_segment = np.max(
                    self.curvature[
                        nearestIndex : nearestIndex + int(self.look_ahead_distance / self.step)
                    ]
                )
                max_lateral_vel_on_segment = np.sqrt(
                    self.params.max_lateral_accel / max_curvature_on_segment
                )

            # Identify the velocity index for the maximum lateral velocity
            max_vel_idx = np.clip(
                np.digitize(max_lateral_vel_on_segment, self.params.v_bins) - 1,
                0,
                self.params.collecting_data_max_n_v - 1,
            )

            # Compute steering angle and its corresponding index
            steer = np.arctan(self.wheel_base * max_curvature_on_segment) * steer_sign
            steer_idx = np.clip(
                np.digitize(steer, self.params.steer_bins) - 1, 0, len(self.params.steer_bins) - 2
            )

            # Determine the minimum velocity based on collected data
            min_vel = np.min((collected_data_counts_of_vel_steer + 1e9 * (1-mask_vel_steer) )[:max_vel_idx, steer_idx])
            vel_idx = np.where(
                collected_data_counts_of_vel_steer[:max_vel_idx, steer_idx] == min_vel
            )[0]
            target_vel_ = self.params.v_bin_centers[vel_idx[0]]

            # Constrain velocity based on lateral acceleration limits
            max_vel_from_lateral_acc_on_segment = np.sqrt(
                self.params.max_lateral_accel / max_curvature_on_segment
            )
            target_vel = np.min([target_vel_ + 0.5 * sine, max_vel_from_lateral_acc_on_segment])

        # Ensure the target velocity remains above a minimum threshold
        target_vel = np.max([target_vel, 0.5])

        return target_vel

    def update_trajectory_points(
        self,
        nearestIndex,
        yaw_offset,
        rectangle_center_position,
        collected_data_counts_of_vel_acc,
        collected_data_counts_of_vel_steer,
    ):
        # Updates the trajectory points based on the vehicle's progress.
        # Adds new trajectory segments as needed and updates the final trajectory data.

        # If the current position exceeds the length of the first two trajectory segments
        if nearestIndex > self.trajectory_length_list[0] + self.trajectory_length_list[1]:
            # Remove the oldest trajectory segment
            nearestIndex = nearestIndex - self.trajectory_length_list[0]
            self.remove_trajectory()

            # Find the steering angle with the least amount of collected data
            mini_num_data = 1e9
            steer_with_minimum_num_of_data = 1e-9
            for i in range(len(collected_data_counts_of_vel_steer[0])):
                # Calculate the steering angle at index i
                steer_at_idx_i = abs(self.params.steer_bin_centers[i]) + 1e-9

                # Compute the maximum velocity allowed by lateral acceleration
                max_vel_from_lateral_acc = np.sqrt(
                    self.params.max_lateral_accel / (self.wheel_base * np.tan(steer_at_idx_i))
                )

                # Find the index corresponding to this maximum velocity
                vel_idx = np.digitize(max_vel_from_lateral_acc, self.params.v_bins) - 1

                # Compute the average number of data points and find the minimum
                num_data = np.mean(np.minimum(collected_data_counts_of_vel_steer[:vel_idx, i], 50))
                if num_data < mini_num_data:
                    mini_num_data = num_data
                    steer_with_minimum_num_of_data = steer_at_idx_i

            # Add trajectory segments until the list has 4 segments
            while len(self.trajectory_length_list) < 4:
                self.add_trajectory_nearly_straight()
                self.add_trajectory_nearly_straight()
                self.add_trajectory_for_turning(steer_with_minimum_num_of_data)

        # Concatenate trajectory data from the trajectory list
        self.trajectory_points = np.concatenate(
            [self.trajectory_list[i].trajectory for i in range(4)],
            axis=0,
        )
        self.yaw = np.concatenate(
            [self.trajectory_list[i].yaw for i in range(4)],
            axis=0,
        )
        self.parts = np.concatenate([self.trajectory_list[i].parts for i in range(4)], axis=0)
        self.curvature = np.concatenate(
            [self.trajectory_list[i].curvature for i in range(4)], axis=0
        )
        self.achievement_rates = np.concatenate(
            [self.trajectory_list[i].achievement_rates for i in range(4)], axis=0
        )

        # Return the updated nearestIndex and trajectory points
        return nearestIndex, *self.return_trajectory_points(yaw_offset, rectangle_center_position)

    def get_boundary_points(self):
        # Computes the boundary points around the trajectory.
        # This is used to create an external boundary around the trajectory.

        # Return None if trajectory points or yaw is not available
        if self.trajectory_points is None or self.yaw is None:
            return None

        # Define the radius for the boundary circle
        outer_circle_radius = self.enclosing_R
        theta_list = np.linspace(0, 2 * np.pi, 200)
        boundary_points = []

        # Compute the center of the boundary circle
        center_point_x = (self.A[0] + self.B[0] + self.C[0] + self.D[0]) / 4
        center_point_y = (self.A[1] + self.B[1] + self.C[1] + self.D[1]) / 4

        # Calculate the points along the boundary circle
        for theta in theta_list:
            point_x = outer_circle_radius * np.cos(theta) + center_point_x
            point_y = outer_circle_radius * np.sin(theta) + center_point_y
            boundary_points.append([point_x, point_y])

        # Store the boundary points as a numpy array
        self.boundary_points = np.array(boundary_points)
