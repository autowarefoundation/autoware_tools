import math
import argparse
from ament_index_python.packages import get_package_share_directory
import yaml


def read_yaml(file_path):
    """Read YAML file and return the data."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


autoware_launch_path = get_package_share_directory("autoware_launch")
vehicle_description_path = get_package_share_directory("j6_gen1_description")

default_mpc_param_file_path = (
    f"{autoware_launch_path}/config/control/trajectory_follower/lateral/mpc.param.yaml"
)
default_pid_param_file_path = (
    f"{autoware_launch_path}/config/control/trajectory_follower/longitudinal/pid.param.yaml"
)
default_simulator_model_param_file_path = f"{vehicle_description_path}/config/simulator_model.param.yaml"

parser = argparse.ArgumentParser(description='Process the parameters of the controllers and simulator.')

parser.add_argument('--mpc_param_file', help='Override the default MPC parameter file path', default=default_mpc_param_file_path)
parser.add_argument('--pid_param_file', help='Override the default PID parameter file path', default=default_pid_param_file_path)
parser.add_argument('--simulator_model_param_file', help='Override the default simulator model parameter file path', default=default_simulator_model_param_file_path)

args = parser.parse_args()
mpc_param_file_path = args.mpc_param_file
pid_param_file_path = args.pid_param_file
simulator_model_param_file_path = args.simulator_model_param_file

mpc_params = read_yaml(mpc_param_file_path)["/**"]["ros__parameters"]
pid_params = read_yaml(pid_param_file_path)["/**"]["ros__parameters"]
simulator_model_params = read_yaml(simulator_model_param_file_path)["/**"]["ros__parameters"]

results = [
    (
        "[MPC] steer_delay_difference: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["steer_time_delay"] - mpc_params["input_delay"],
            mpc_params["input_delay"],
            simulator_model_params["steer_time_delay"],
        )
    ),
    (
        "[MPC] steer_time_constant_difference: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["steer_time_constant"] - mpc_params["vehicle_model_steer_tau"],
            mpc_params["vehicle_model_steer_tau"],
            simulator_model_params["steer_time_constant"],
        )
    ),
    (
        "[MPC] acceleration_limit_difference: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["vel_rate_lim"] - mpc_params["acceleration_limit"],
            mpc_params["acceleration_limit"],
            simulator_model_params["vel_rate_lim"],
        )
    ),
    (
        "[MPC] max_steer_rate_lim_difference_by_curvature: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["steer_rate_lim"]
            - max(mpc_params["steer_rate_lim_dps_list_by_curvature"]) * (math.pi / 180),
            max(mpc_params["steer_rate_lim_dps_list_by_curvature"]) * (math.pi / 180),
            simulator_model_params["steer_rate_lim"],
        )
    ),
    (
        "[MPC] max_steer_rate_lim_difference_by_velocity: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["steer_rate_lim"]
            - max(mpc_params["steer_rate_lim_dps_list_by_velocity"]) * (math.pi / 180),
            max(mpc_params["steer_rate_lim_dps_list_by_velocity"]) * (math.pi / 180),
            simulator_model_params["steer_rate_lim"],
        )
    ),
    (
        "[PID] max_acc_difference: {}, (controller: {}, simulator: {})".format(
            simulator_model_params["vel_rate_lim"] - pid_params["max_acc"],
            pid_params["max_acc"],
            simulator_model_params["vel_rate_lim"],
        )
    ),
    (
        "[PID] min_acc_difference: {}, (controller: {}, simulator: {})".format(
            -simulator_model_params["vel_rate_lim"] - pid_params["min_acc"],
            pid_params["min_acc"],
            -simulator_model_params["vel_rate_lim"],
        )
    ),
]

for item in results:
    print(item)
