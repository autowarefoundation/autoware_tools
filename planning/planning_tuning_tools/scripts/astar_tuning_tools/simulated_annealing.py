import math
import random
import argparse
import os
import pathlib
import pickle
import time

from common.common_classes import Result
from common.common_classes import SearchInfo
import freespace_planning_algorithms.astar_search as fp
from geometry_msgs.msg import Pose
import numpy as np
from param.astar_params import astar_param
from param.astar_params import planner_param
from param.astar_params import vehicle_shape
from nav_msgs.msg import OccupancyGrid

costmap = OccupancyGrid()

class SimjulatedAnnealing:
    def __init__(self, ):
        self.best_param = None
        self.best_energy = None
        self.val_data_set = None

    def objective_function(self, param):

        # planner search configs
        planner_param.theta_size = param[0]
        planner_param.angle_goal_range = param[1]
        planner_param.curve_weight = param[2]
        planner_param.reverse_weight = param[3]
        planner_param.lateral_goal_range = param[4]
        planner_param.longitudinal_goal_range = param[5]

        # -- A* search Configurations --
        astar_param.distance_heuristic_weight = param[6]

        astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
        start_pose = Pose()

        total_result = 0
        total_length_rate = 0
        total_forward_backward_change = 0

        for val_data in self.val_data_set:
            astar.setMap(val_data.costmap)
            goal_pose = val_data.goal_pose

            find = astar.makePlan(start_pose, goal_pose)
            waypoints = fp.PlannerWaypoints()
            if find:
                total_result += 1
                waypoints = astar.getWaypoints()
                L2_dist = math.hypot(goal_pose.position.x-start_pose.position.x, goal_pose.position.y-start_pose.position.y)
                total_length_rate += waypoints.compute_length / L2_dist
                total_forward_backward_change += self.count_forawrd_backwrad_change(waypoints)
    
        N = len(self.val_data_set)
        success_rate = total_result / N
        average_length = total_length_rate / total_result
        average_forward_backward_change = total_forward_backward_change/total_result

        return success_rate + average_length + average_forward_backward_change

    # Define the cooling schedule function
    def cooling_schedule(t, initial_temperature):
        return initial_temperature / (1 + t)

    # Simulated Annealing algorithm
    def simulated_annealing(self, initial_param, initial_temperature, iterations):
        initial_param = np.array([144, 6.0, 1.2, 2.0, 0.5, 2.0, 1.0]).transpose

        current_param = initial_param
        current_energy = self.objective_function(current_param)

        best_param = current_param
        best_energy = current_energy

        for t in range(iterations):
            temperature = self.cooling_schedule(t, initial_temperature)

            neighbor_param = current_param + np.random.uniform(-0.5, 0.5, (7, 1)) # Example: Small random change

            neighbor_energy = self.objective_function(neighbor_param)

            if neighbor_energy < current_energy or random.random() < math.exp((current_energy - neighbor_energy) / temperature):
                current_param = neighbor_param
                current_energy = neighbor_energy

                if neighbor_energy < best_energy:
                    best_param = neighbor_param
                    best_energy = neighbor_energy

        return best_param, best_energy
    
    def count_forawrd_backwrad_change(waypoints):
        count = 0
        if len(waypoints.waypoints):
            pre_is_back = waypoints.waypoints[0].is_back
            for waypoint in waypoints.waypoints:
                is_back = waypoint.is_back
                if is_back != pre_is_back:
                    count += 1
                pre_is_back = is_back

        return count

# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="place of save result")
    parser.add_argument(
        "--costmap",
        default="costmap_default",
        type=str,
        help="file name of costmap without extension",
    )
    args = parser.parse_args()

    with open(os.path.dirname(__file__) + "/costmap/" + args.costmap + ".txt", "rb") as f:
        costmap = pickle.load(f)

    initial_param = random.uniform(-10, 10)  # Random initial solution within a range
    initial_temperature = 100.0
    iterations = 1000

    best_param, best_energy = simulated_annealing(objective_function, initial_param, initial_temperature, iterations)
    print("Best solution:", best_param)
    print("Objective value at best solution:", best_energy)
