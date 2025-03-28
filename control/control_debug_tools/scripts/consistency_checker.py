#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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


import sys
import argparse
import math
import operator

from ament_index_python.packages import get_package_share_directory
import yaml

# Indicate the source of a parameter
class Source:
    MPC=0
    PID=1
    SIM=2

    def to_string(source):
        match source:
            case Source.MPC: return "MPC"
            case Source.PID: return "PID"
            case Source.SIM: return "Simulator model"
            case _: return "Unknown source"

# Condition between 2 parameters
class Condition:
    def __init__(self, param1, source1, param2, source2, fail_msg, op):
        self.param1 = param1
        self.param2 = param2
        self.source1 = source1
        self.source2 = source2
        self.fail_msg = fail_msg
        self.op = op

    def eval(self, sources):
        val1 = sources[self.source1][self.param1]
        val2 = sources[self.source2][self.param2]
        if(not self.op(val1, val2)):
            return "{}[{}] = {} {} {}[{}] = {}".format(Source.to_string(self.source1), self.param1, val1, self.fail_msg, Source.to_string(self.source2), self.param2, val2)
        return None

def read_yaml(file_path):
    """Read YAML file and return the data."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process the parameters of the controllers and simulator."
    )

    parser.add_argument(
        "--vehicle_description",
        help="Specify the vehicle description package name",
        default="sample_vehicle_description",
    )
    parser.add_argument("--mpc_param_file", help="Override the default MPC parameter file path")
    parser.add_argument("--pid_param_file", help="Override the default PID parameter file path")
    parser.add_argument(
        "--simulator_model_param_file", help="Override the default simulator model parameter file path"
    )
    args = parser.parse_args()

    try:
        autoware_launch_path = get_package_share_directory("autoware_launch")
        vehicle_description_path = get_package_share_directory(args.vehicle_description)
    except Exception as e:
        print(e)
        print("make sure your Autoware workspace is sourced")
        sys.exit(-1)

    mpc_param_file_path = (
        args.mpc_param_file
        if args.mpc_param_file
        else f"{autoware_launch_path}/config/control/trajectory_follower/lateral/mpc.param.yaml"
    )
    pid_param_file_path = (
        args.pid_param_file
        if args.pid_param_file
        else f"{autoware_launch_path}/config/control/trajectory_follower/longitudinal/pid.param.yaml"
    )
    simulator_model_param_file_path = (
        args.simulator_model_param_file
        if args.simulator_model_param_file
        else f"{vehicle_description_path}/config/simulator_model.param.yaml"
    )

    sources = {}
    sources[Source.MPC] = read_yaml(mpc_param_file_path)["/**"]["ros__parameters"]
    sources[Source.PID] = read_yaml(pid_param_file_path)["/**"]["ros__parameters"]
    sources[Source.SIM] = read_yaml(simulator_model_param_file_path)["/**"]["ros__parameters"]
    conditions = [
        # Equality conditions
        Condition("input_delay", Source.MPC, "steer_time_delay", Source.SIM, "should be identical to", operator.eq),
        Condition("delay_compensation_time", Source.PID, "acc_time_delay", Source.SIM, "should be identical to", operator.eq),
        Condition("acceleration_limit", Source.MPC, "vel_rate_lim", Source.SIM, "should be identical to", operator.eq),
        Condition("keep_steer_control_until_converged", Source.MPC, "enable_keep_stopped_until_steer_convergence", Source.PID, "should be identical to", operator.eq),
        # Lower than conditions
        Condition("steer_rate_lim_dps_list_by_curvature", Source.MPC, "steer_rate_lim", Source.SIM, "should be lower than", lambda mpc, sim: max(mpc) * (math.pi / 180.0) < sim),
        Condition("steer_rate_lim_dps_list_by_velocity", Source.MPC, "steer_rate_lim", Source.SIM, "should be lower than", lambda mpc, sim: max(mpc) * (math.pi / 180.0) < sim),
        Condition("max_acc", Source.PID, "vel_rate_lim", Source.SIM, "should be lower than", operator.lt),
        Condition("min_acc", Source.PID, "vel_rate_lim", Source.SIM, "(absolute) should be lower than", lambda pid, sim: abs(pid) < sim),
    ]

    for condition in conditions:
        fail_msg = condition.eval(sources)
        if(fail_msg):
            print(f"\033[91m{fail_msg}\033[0m")
