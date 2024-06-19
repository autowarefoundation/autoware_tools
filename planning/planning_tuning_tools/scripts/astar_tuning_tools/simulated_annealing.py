import argparse
import math
import os
import pickle
import random

import autoware_freespace_planning_algorithms.astar_search as fp
from config.astar_params import astar_param
from config.astar_params import planner_param
from config.astar_params import vehicle_shape
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import numpy as np
from pyquaternion import Quaternion
from tqdm import tqdm

costmap = OccupancyGrid()


class TestData:
    def __init__(self, costmap, goal_pose):
        self.costmap = costmap
        self.goal_pose = goal_pose


class Params:
    def __init__(self, planner_param, astar_param):
        self.planner_param = planner_param
        self.astar_param = astar_param


class SimjulatedAnnealing:
    def __init__(self, test_data_set):
        self.best_param = None
        self.best_energy = None
        self.test_data_set = test_data_set

    def objective_function(self, param):
        planner_param.curve_weight = param[0]
        planner_param.reverse_weight = param[1]

        # -- A* search Configurations --
        astar_param.distance_heuristic_weight = param[2]

        astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
        start_pose = Pose()

        total_result = 0
        total_length_rate = 0
        total_direction_change = 0

        for test_data in self.test_data_set:
            astar.setMap(test_data.costmap)
            goal_pose = test_data.goal_pose

            find = astar.makePlan(start_pose, goal_pose)
            waypoints = fp.PlannerWaypoints()
            if find:
                total_result += 1
                waypoints = astar.getWaypoints()
                L2_dist = math.hypot(
                    goal_pose.position.x - start_pose.position.x,
                    goal_pose.position.y - start_pose.position.y,
                )
                if L2_dist != 0:
                    total_length_rate += waypoints.compute_length() / L2_dist
                total_direction_change += self.count_forawrd_backwrad_change(waypoints)

        N = len(self.test_data_set)
        unsuccess_rate = 1 - total_result / N
        average_length_rate = total_length_rate / total_result
        average_forward_backward_change = total_direction_change / total_result

        return 10 * unsuccess_rate + average_length_rate + 10 * average_forward_backward_change

    # Define the cooling schedule function
    def cooling_schedule(self, t, initial_temperature):
        return initial_temperature / (1 + t)

    # Simulated Annealing algorithm
    def simulated_annealing(self, initial_param, initial_temperature, iterations, weight):
        current_param = initial_param
        current_energy = self.objective_function(current_param)
        print("Initial objective value:", current_energy)

        best_param = current_param
        best_energy = current_energy

        for t in tqdm(range(iterations)):
            temperature = self.cooling_schedule(t, initial_temperature)

            neighbor_param = current_param + weight * np.random.uniform(
                -1, 1, 3
            )  # Small random change
            neighbor_energy = self.objective_function(neighbor_param)

            if neighbor_energy < current_energy or random.random() < math.exp(
                (current_energy - neighbor_energy) / temperature
            ):
                current_param = neighbor_param
                current_energy = neighbor_energy

                if neighbor_energy < best_energy:
                    best_param = neighbor_param
                    best_energy = neighbor_energy

        return best_param, best_energy

    def get_result(self):
        return best_param, best_energy

    def count_forawrd_backwrad_change(self, waypoints):
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
    parser.add_argument(
        "--save_name",
        default="optimal_param_default",
        type=str,
        help="file name without extension to save",
    )
    args = parser.parse_args()

    with open(os.path.dirname(__file__) + "/costmap/" + args.costmap + ".txt", "rb") as f:
        costmap = pickle.load(f)

    test_data_set = []
    for i in range(20):
        goal_pose = Pose()
        x = np.random.uniform(-4, 4)
        y = np.random.uniform(-4, 4)
        yaw = np.random.uniform(-np.pi, np.pi)

        goal_pose.position.x = float(x)
        goal_pose.position.y = float(y)

        quaterinon = Quaternion(axis=[0, 0, 1], angle=yaw)

        goal_pose.orientation.w = quaterinon.w
        goal_pose.orientation.x = quaterinon.x
        goal_pose.orientation.y = quaterinon.y
        goal_pose.orientation.z = quaterinon.z

        test_data_set.append(TestData(costmap, goal_pose))

    initial_temperature = 200.0
    iterations = 300

    # initial_param = np.array([144, 6.0, 1.2, 2.0, 0.5, 2.0, 1.0])
    initial_param = np.array([1.2, 2.0, 1.0])
    weight = 0.1 * initial_param

    simulated_annealing = SimjulatedAnnealing(test_data_set)
    best_param, best_energy = simulated_annealing.simulated_annealing(
        initial_param, initial_temperature, iterations, weight
    )

    file_name = os.path.dirname(__file__) + "/opt_param/" + args.save_name + ".txt"
    with open(file_name, "wb") as file1:
        pickle.dump(best_param, file1)

    print("Best solution:", best_param)
    print("Objective value at best solution:", best_energy)
