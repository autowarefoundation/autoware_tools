# Copyright 2024 TIER IV, Inc. All rights reserved.
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

import argparse
import math
import os
import pickle
import sys

import numpy as np
import yaml

sys.path.append(os.path.dirname(__file__))


class Evaluator:
    def __init__(self, results=None):
        self.indicator = {}
        self.N = None
        self.N_success = None
        self.unsuccess_rate = None
        self.path_length_rate = None
        self.direction_change = None
        self.minus_distance_to_obstacle_average = None
        self.minus_distance_to_obstacle_minimum = None
        self.calculation_time = None
        if results is not None:
            self.set_results(results)

    def set_config(self, config_path):
        with open(config_path) as file:
            all_config = yaml.safe_load(file)

        config_dict = all_config["evaluation"]
        for key in config_dict.keys():
            param_config = config_dict[key]
            if param_config["use_optimization"]:
                self.indicator[key] = param_config["weight"]                

    def evaluate(self):
        if len(self.indicator) == 0:
            print("No config is set...")
            return

        value = 0
        for key, weight in self.indicator.items():
            value += weight * getattr(self, key)
        return value

    def set_results(self, results):
        # flatten datas
        if np.ndim(results) != 1:
            results = np.reshape(results, -1)

        N = len(results)

        N_success = 0
        total_length_rate = 0
        total_direction_change = 0
        total_distance_to_obstacle_average = 0
        total_distance_to_obstacle_minimum = 0

        for result in results:
            if result.find:
                N_success += 1
                waypoints = result.waypoints
                L2_dist = math.hypot(
                    waypoints.waypoints[0].pose.position.x
                    - waypoints.waypoints[-1].pose.position.x,
                    waypoints.waypoints[0].pose.position.y
                    - waypoints.waypoints[-1].pose.position.y,
                )
                if L2_dist != 0:
                    total_length_rate += waypoints.compute_length() / L2_dist
                total_direction_change += self.count_forward_backward_change(waypoints)
                total_distance_to_obstacle_average += np.mean(result.distance_to_obstacles)
                total_distance_to_obstacle_minimum += min(result.distance_to_obstacles)

        self.N = N
        self.N_success = N_success
        if N_success != 0:
            self.unsuccess_rate = 1 - N_success / N
            self.path_length_rate = total_length_rate / N_success
            self.direction_change = total_direction_change / N_success
            self.minus_distance_to_obstacle_average = -total_distance_to_obstacle_average / N_success
            self.minus_distance_to_obstacle_minimum = -total_distance_to_obstacle_minimum / N_success

    def set_results_from_path(self, dir_path):
        # laod search settings
        with open(dir_path + "/info.txt", "rb") as f:
            info = pickle.load(f)
            xs = info.xs
            ys = info.ys
            yaws = info.yaws

        # load results
        results = []
        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                for k, yaw in enumerate(yaws):
                    filename = dir_path + "/" + str(x) + "_" + str(y) + "_" + str(k) + ".txt"
                    with open(filename, "rb") as f:
                        results.append(pickle.load(f))

        self.set_results(results)

    def set_calculation_time(self, calculation_time):
        self.calculation_time = calculation_time

    def count_forward_backward_change(self, waypoints):
        count = 0
        if len(waypoints.waypoints):
            pre_is_back = waypoints.waypoints[0].is_back
            for waypoint in waypoints.waypoints:
                is_back = waypoint.is_back
                if is_back != pre_is_back:
                    count += 1
                pre_is_back = is_back

        return count

    def print_result(self):
        if self.N is None:
            print("No results were given...")
            return

        if self.N_success != 0:
            print("set goal number", self.N)
            print(
                "Success number: ", self.N_success, "(Success rate: ", 1 - self.unsuccess_rate, ")"
            )
            print(
                "Average of path length ratio (path length / start-goal length): ",
                self.path_length_rate,
            )
            print("Average of direction change number: ", self.direction_change)
            print("Average of distance to obstacle: ", -self.minus_distance_to_obstacle_average)
            print(
                "Average of minimium distance to obstacle in path: ",
                -self.minus_distance_to_obstacle_minimum,
            )
        else:
            print("No goals were found...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="place of save result")
    parser.add_argument("--save_name", default="default_dir", type=str, help="saved directory name")
    args = parser.parse_args()

    save_dir = os.path.dirname(__file__) + "/result/" + args.save_name

    evaluator = Evaluator()
    evaluator.set_results_from_path(save_dir)
    evaluator.print_result()
