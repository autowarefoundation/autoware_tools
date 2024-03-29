import argparse
import math

from ament_index_python.packages import get_package_share_directory
import yaml


def read_yaml(file_path):
    """Read YAML file and return the data."""
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


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


def main():
    args = parser.parse_args()

    autoware_launch_path = get_package_share_directory("autoware_launch")
    vehicle_description_path = get_package_share_directory(args.vehicle_description)

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
                simulator_model_params["steer_time_constant"]
                - mpc_params["vehicle_model_steer_tau"],
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
        (
            "[PID] accelerate_delay_difference: {}, (controller: {}, simulator: {})".format(
                simulator_model_params["acc_time_delay"] - pid_params["delay_compensation_time"],
                pid_params["delay_compensation_time"],
                simulator_model_params["acc_time_delay"],
            )
        ),
    ]

    for item in results:
        description = item.split(",")[0]
        value = float(description.split(":")[1].strip())
        error_message = ""
        if (
            "steer_delay_difference" in item
            or "steer_time_constant_difference" in item
            or "accelerate_delay_difference" in item
        ):
            if value != 0.0:
                error_message = "[ERROR] The parameters of the controller and simulator should be identical.\033[0m"
        if (
            "acceleration_limit_difference" in item
            or "max_steer_rate_lim_difference_by_curvature" in item
            or "max_steer_rate_lim_difference_by_velocity" in item
            or "max_acc_difference" in item
        ):
            if value < 0:
                error_message = "[ERROR] The parameter of the controller should be smaller than the parameter of the simulator.\033[0m"
        if "min_acc_difference" in item and value > 0:
            error_message = "[ERROR] The parameter of the controller should be bigger than the parameter of the simulator.\033[0m"
        print(f"{item}")
        if error_message:
            print(f"\033[91m{error_message}\033[0m\n")


if __name__ == "__main__":
    main()
