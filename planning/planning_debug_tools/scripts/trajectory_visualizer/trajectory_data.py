from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
import numpy

def _dist(p1: TrajectoryPoint, p2: TrajectoryPoint):
    dx = p1.pose.position.x - p2.pose.position.x
    dy = p1.pose.position.y - p2.pose.position.y
    return numpy.sqrt(dx**2 + dy**2)

def get_velocites(trajectory: Trajectory):
    return [p.longitudinal_velocity_mps for p in trajectory.points]

def get_arc_lengths(trajectory: Trajectory):
    arc_lengths = [0.0]
    for i in range(0, len(trajectory.points) - 1):
        arc_lengths.append(arc_lengths[-1] + _dist(trajectory.points[i], trajectory.points[i+1]))
    return arc_lengths

def get_data_functions() -> dict:
    return {
        "Arc Lengths": get_arc_lengths,
        "Velocity": get_velocites,
    } 