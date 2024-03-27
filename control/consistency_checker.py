import math
import yaml

def read_yaml(file_path):
    """Read YAML file and return the data."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file)

mpc_param_file_path = "/home/shen/pilot-auto.x2/src/autoware/launcher/autoware_launch/config/control/trajectory_follower/lateral/mpc.param.yaml"
pid_param_file_path = "/home/shen/pilot-auto.x2/src/autoware/launcher/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml"
simulator_model_param_file_path = "/home/shen/pilot-auto.x2/src/description/vehicle/j6_gen1_description/j6_gen1_description/config/simulator_model.param.yaml"

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
