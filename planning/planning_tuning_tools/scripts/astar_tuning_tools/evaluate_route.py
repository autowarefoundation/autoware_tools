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

from autoware_auto_planning_msgs.msg import Trajectory
import freespace_planning_algorithms.astar_search as fp
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from tqdm import tqdm

sys.path.append(os.path.dirname(__file__))

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="place of save result")
    parser.add_argument("--dir_name", default="default_dir", type=str, help="saved directory name")
    args = parser.parse_args()

    save_dir = os.path.dirname(__file__) + "/result/" + args.dir_name

    # laod search settings
    with open(save_dir + "/info.txt", "rb") as f:
        info = pickle.load(f)
        costmap = info.costmap
        x_range = -costmap.info.origin.position.x
        y_range = -costmap.info.origin.position.y
        xs = info.xs
        ys = info.ys
        yaws = info.yaws
        yaws_d = [int((10 / (2 * math.pi)) * yaw) for yaw in yaws]

    # load results
    results = np.zeros((len(yaws), len(xs), len(ys)))
    waypoints_all = [
        [[fp.PlannerWaypoints() for k in range(len(ys))] for j in range(len(xs))] for i in range(len(yaws))
    ]

    print("loading result datas...")
    for i, x in enumerate(tqdm(xs)):
        for j, y in enumerate(ys):
            for k, yaw in enumerate(yaws):
                filename = save_dir + "/" + str(x) + "_" + str(y) + "_" + str(k) + ".txt"
                with open(filename, "rb") as f:
                    result = pickle.load(f)
                    results[k][i][j] = result.find
                    waypoints_all[k][i][j] = result.waypoints

    # process results
    N = len(xs)*len(ys)*len(yaws)
    print("データ数: ", N)

    total_result = 0
    total_length = 0
    total_forward_backward_change = 0
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, yaw in enumerate(yaws):
                total_result += results[k][i][j]
                if results[k][i][j]:
                    waypoints = waypoints_all[k][i][j]
                    total_length += waypoints.compute_length()
                    total_forward_backward_change += count_forawrd_backwrad_change(waypoints)
    
    print("成功数: ", total_result)
    print("成功率: ", total_result/N)
    print("平均経路長: ", total_length/total_result)
    print("平均切り返し回数: ", total_forward_backward_change/total_result)
