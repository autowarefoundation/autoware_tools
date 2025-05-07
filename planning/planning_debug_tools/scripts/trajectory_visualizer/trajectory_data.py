from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
import numpy as np
from tf_transformations import euler_from_quaternion
from angles import shortest_angular_distance
from rclpy import time

def _dist(p1: TrajectoryPoint, p2: TrajectoryPoint):
    dx = p1.pose.position.x - p2.pose.position.x
    dy = p1.pose.position.y - p2.pose.position.y
    return np.sqrt(dx**2 + dy**2)

def _triangle_area(p1: TrajectoryPoint, p2: TrajectoryPoint, p3: TrajectoryPoint):
    """Helper function to calculate the area of a triangle defined by three 2D points using shoelace formula."""
    return 0.5 * np.abs(p1.pose.position.x *(p2.pose.position.y - p3.pose.position.y) + p2.pose.position.x*(p3.pose.position.y - p1.pose.position.y) + p3.pose.position.x*(p1.pose.position.y - p2.pose.position.y))

def get_velocites(trajectory: Trajectory):
    return [p.longitudinal_velocity_mps for p in trajectory.points]

def get_arc_lengths(trajectory: Trajectory):
    arc_lengths = [0.0]
    for i in range(0, len(trajectory.points) - 1):
        arc_lengths.append(arc_lengths[-1] + _dist(trajectory.points[i], trajectory.points[i+1]))
    return arc_lengths

def get_times(trajectory: Trajectory):
    times = []
    for p in trajectory.points:
        d = time.Duration.from_msg(p.time_from_start)
        times.append(d.nanoseconds*1e-9)
    return times

def calculate_curvature_2d(trajectory: Trajectory):
    """
    Calculates the curvature at each point of a 2D trajectory.

    The trajectory is given as a list of (x, y) tuples or an Nx2 NumPy array.
    Curvature is calculated using the formula:
        kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    where derivatives are computed numerically using numpy.gradient, assuming
    a unit spacing for the parameter t (e.g., time or arc length segment).
    """

    # Validate input shape
    num_points = len(trajectory.points)

    # Handle trivial cases
    if num_points == 0:
        return np.array([])
    if num_points == 1:
        return np.array([0.0])
    # For num_points == 2 (straight line), the formula will yield 0, so no special handling needed.

    x = [p.pose.position.x for p in trajectory.points]
    y = [p.pose.position.y for p in trajectory.points]

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
    denominator = speed_squared**(1.5)

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

def calculate_menger_curvature(trajectory: Trajectory):
    """
    Calculates the Menger curvature at each point of a 2D trajectory using triplets of points.

    Menger curvature for points P1, P2, P3 is:
        kappa = (4 * Area(P1, P2, P3)) / (|P1P2| * |P2P3| * |P3P1|)
    The curvature is assigned to the middle point (P2).
    Curvature for the first and last points is set to 0.
    """
    num_points = len(trajectory.points)

    if num_points == 0:
        return np.array([])
    if num_points < 3:
        return np.zeros(num_points) # Curvature is 0 for first/last or if not enough points

    curvatures = np.zeros(num_points)
    epsilon = 1e-9 # To avoid division by zero for collinear/coincident points

    for i in range(1, num_points - 1):
        p1 = trajectory.points[i-1]
        p2 = trajectory.points[i]
        p3 = trajectory.points[i+1]

        # Calculate side lengths of the triangle
        d12 = _dist(p1, p2)
        d23 = _dist(p2, p3)
        d13 = _dist(p1, p3)

        # If points are coincident or very close, side length can be zero.
        # If any side length is zero, or points are collinear, area will be zero or product of sides will be zero.
        if d12 < epsilon or d23 < epsilon or d13 < epsilon:
            curvatures[i] = 0.0 # Collinear or coincident points
            continue

        # Calculate area of the triangle
        area = _triangle_area(p1, p2, p3)

        # If area is very small, points are nearly collinear.
        if area < epsilon: # Effectively collinear
            curvatures[i] = 0.0
            continue
        
        denominator_menger = d12 * d23 * d13
        if denominator_menger < epsilon : # Should be caught by individual distance checks too
             curvatures[i] = 0.0
             continue

        curvatures[i] = (4 * area) / denominator_menger

    return curvatures

def get_accelerations(trajectory: Trajectory):
    return [p.acceleration_mps2 for p in trajectory.points]

def _get_yaw_from_quaternion(quaternion_msg):
    quaternion = [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw

def relative_angles(trajectory: Trajectory):
    angles = [0.0]
    for i in range(0, len(trajectory.points)-1):
        yaw0 = _get_yaw_from_quaternion(trajectory.points[i].pose.orientation)
        yaw1 = _get_yaw_from_quaternion(trajectory.points[i+1].pose.orientation)
        angles.append(shortest_angular_distance(yaw0, yaw1))
    return angles

def get_data_functions() -> dict:
    return {
        "Arc Length [m]": get_arc_lengths,
        "Velocity [m/s]": get_velocites,
        "Times [s]": get_times,
        "Curvature (derivatives) [m⁻¹]": calculate_curvature_2d,
        "Curvature (Menger) [m⁻¹]": calculate_menger_curvature,
        "Acceleration [m/s²]": get_accelerations,
        "Relative angles [rad]": relative_angles,
    } 