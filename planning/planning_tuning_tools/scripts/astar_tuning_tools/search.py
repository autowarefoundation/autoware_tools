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
import pathlib
import pickle
import time

import autoware_freespace_planning_algorithms.astar_search as fp
from common.common_classes import Result
from common.common_classes import SearchInfo
from common.common_functions import param_vector2obj
from config.astar_params import astar_param
from config.astar_params import planner_param
from config.astar_params import vehicle_shape
from geometry_msgs.msg import Pose
import numpy as np
from pyquaternion import Quaternion
from tqdm import tqdm
import yaml


def pose2yaw(pose):
    qw = pose.orientation.w
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    t0 = +2.0 * (qw * qz + qx * qy)
    t1 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t0, t1)
    return yaw


def search_one(astar, start_pose, goal_pose):
    try:
        find = astar.makePlan(start_pose, goal_pose)
    except RuntimeError:
        find = False

    waypoints = fp.PlannerWaypoints()
    distance_to_obstacles = []
    steerings = []
    if find:
        waypoints = astar.getWaypoints()
        pre_yaw = pose2yaw(waypoints.waypoints[0].pose)
        for waypoint in waypoints.waypoints:
            distance_to_obstacles.append(astar.getDistanceToObstacle(waypoint.pose))
            yaw = pose2yaw(waypoint.pose)
            steerings.append(yaw - pre_yaw)
            pre_yaw = yaw

    x = goal_pose.position.x
    y = goal_pose.position.y
    yaw = pose2yaw(goal_pose)

    return Result(x, y, yaw, find, waypoints, distance_to_obstacles, steerings)


def float_range(start, end, step):
    f_range = [start]
    n = start
    if step >= 0:
        while n + step < end:
            n = n + step
            f_range.append(n)
    else:
        while n + step > end:
            n = n + step
            f_range.append(n)
    return f_range


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="place of save result")
    parser.add_argument(
        "--save_name", default="default_dir", type=str, help="directory name to save"
    )
    parser.add_argument(
        "--costmap",
        default="costmap_default",
        type=str,
        help="file name of costmap without extension",
    )
    parser.add_argument("--x_resolution", default=1.0, type=float, help="interval of goal x")
    parser.add_argument("--y_resolution", default=1.0, type=float, help="interval of goal y")
    parser.add_argument(
        "--yaw_discrete", default=10, type=int, help="the discretized number of yaw"
    )
    parser.add_argument("--opt_param", default=None, type=str, help="name of optimal param")
    args = parser.parse_args()

    # input processing
    save_dir = os.path.dirname(__file__) + "/result/" + args.save_name

    with open(os.path.dirname(__file__) + "/costmap/" + args.costmap + ".txt", "rb") as f:
        costmap = pickle.load(f)

    costmap_height_half = costmap.info.resolution * costmap.info.height / 2
    costmap_width_half = costmap.info.resolution * costmap.info.width / 2

    if args.opt_param is not None:
        with open(os.path.dirname(__file__) + "/opt_param/" + args.opt_param + ".yaml", "rb") as f:
            all_config = yaml.safe_load(f)
        for key in all_config["planner_param"].keys():
            setattr(planner_param, key, all_config["planner_param"][key])
        for key in all_config["astar_param"].keys():
            setattr(astar_param, key, all_config["astar_param"][key])
        for key in all_config["vehicle_shape"].keys():
            setattr(vehicle_shape, key, all_config["vehicle_shape"][key])

    # goal grid
    xs = list(
        reversed(float_range(-args.x_resolution, -costmap_height_half, -args.x_resolution))
    ) + float_range(0, costmap_height_half, args.x_resolution)
    ys = list(
        reversed(float_range(-args.y_resolution, -costmap_width_half, -args.y_resolution))
    ) + float_range(0, costmap_width_half, args.y_resolution)
    yaws = [(2 * np.pi) * i / args.yaw_discrete for i in range(args.yaw_discrete)]

    verbose = False
    # make files if not.
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        for x in xs:
            for y in ys:
                for i, yaw in enumerate(yaws):
                    # filepath: x_y_yawindex.txt
                    filepath = pathlib.Path(
                        save_dir + "/" + str(x) + "_" + str(y) + "_" + str(i) + ".txt"
                    )
                    filepath.touch()

        with open(save_dir + "/info.txt", "wb") as f:
            info = SearchInfo(costmap, xs, ys, yaws)
            pickle.dump(info, f)

        verbose = True

        print("save files were generated!")

    time.sleep(5)

    # Search
    astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)
    astar.setMap(costmap)

    start_pose = Pose()
    goal_pose = Pose()

    start_time = time.monotonic()

    x_iter = tqdm(xs) if verbose else xs
    for x in x_iter:
        for y in ys:
            for i, yaw in enumerate(yaws):
                # mutual exclusion
                filename = save_dir + "/" + str(x) + "_" + str(y) + "_" + str(i) + ".txt"
                if os.access(filename, os.W_OK) and os.path.getsize(filename) < 10:
                    with open(filename, "wb") as f:
                        goal_pose.position.x = float(x)
                        goal_pose.position.y = float(y)

                        quaterinon = Quaternion(axis=[0, 0, 1], angle=yaw)

                        goal_pose.orientation.w = quaterinon.w
                        goal_pose.orientation.x = quaterinon.x
                        goal_pose.orientation.y = quaterinon.y
                        goal_pose.orientation.z = quaterinon.z

                        result = search_one(astar, start_pose, goal_pose)

                        pickle.dump(result, f)

    end_time = time.monotonic()
    if verbose:
        print("search_time : ", end_time - start_time)
