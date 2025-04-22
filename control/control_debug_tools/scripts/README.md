# Autoware Parameter Consistency Checker

## Purpose

This script checks for potential inconsistencies between critical parameters defined in different configuration files within an Autoware workspace. Specifically, it compares parameters related to vehicle dynamics and control limits across:

1. **Model Predictive Control (MPC)**: Lateral control parameters.
2. **Proportional–Integral–Derivative (PID)**: Longitudinal control parameters.
3. **Simulator Model**: Vehicle simulation parameters.
4. **Vehicle Command Gate**: Safety limits applied to final commands.

Ensuring these parameters are consistent is crucial for stable and predictable vehicle behavior, both in simulation and on the real vehicle.
Mismatched parameters can lead to unexpected behavior of the ego vehicle.

## Usage

1. **Source Workspace**: Make sure your Autoware workspace (containing `autoware_launch`, the relevant vehicle description package, etc.) is sourced in your terminal:

   ```bash
   source /path/to/your/autoware_ws/install/setup.bash
   ```

2. **Run the Script**: Execute the script using Python 3.

   ```bash
   python3 /path/to/parameter_checker_script.py [OPTIONS]
   ```

3. **Options**:
   - `--vehicle_description <package_name>`: Specify the name of the vehicle description package.
     - _Default_: `autoware_sample_vehicle_description`
   - `--mpc_param_file <path/to/file.yaml>`: Override the default path for the MPC parameter file.
   - `--pid_param_file <path/to/file.yaml>`: Override the default path for the PID parameter file.
   - `--simulator_model_param_file <path/to/file.yaml>`: Override the default path for the simulator model parameter file.
   - `--vehicle_cmd_gate_param_file <path/to/file.yaml>`: Override the default path for the vehicle command gate parameter file.
   - `--vehicle_id <id_string>`: Specify a vehicle ID. If provided, the script looks for MPC/PID parameters in subdirectories named after this ID (e.g., `.../control/trajectory_follower/<vehicle_id>/lateral/mpc.param.yaml`).
     - _Default_: `None`

**Example:**

```bash
# Check parameters for the default sample vehicle
python3 parameter_checker_script.py

# Check parameters for a specific vehicle ID 'my_vehicle'
python3 parameter_checker_script.py --vehicle_id my_vehicle

# Check parameters using a custom vehicle description and overriding the MPC file
python3 parameter_checker_script.py \
  --vehicle_description my_vehicle_description \
  --mpc_param_file /path/to/custom_mpc.param.yaml
```

The script will print the paths of the files being compared and then output any detected inconsistencies in red.

## How it Works

1. **Argument Parsing**: The script uses `argparse` to accept command-line arguments. This allows users to specify:

   - The name of the vehicle description package (`--vehicle_description`).
   - Specific paths to parameter files, overriding the defaults (`--mpc_param_file`, `--pid_param_file`, etc.).
   - A vehicle ID to load vehicle-specific control parameters (`--vehicle_id`).

2. **Package Path Resolution**: It utilizes `ament_index_python.get_package_share_directory` to locate the installation paths (share directories) of the required ROS 2 packages (`autoware_launch` and the specified vehicle description package). This requires the Autoware workspace containing these packages to be sourced beforehand.

3. **Parameter File Location**: Based on the arguments and resolved package paths, the script determines the exact paths to the four YAML parameter files (MPC, PID, Simulator, Vehicle Command Gate). It constructs default paths within the `autoware_launch` and vehicle description packages, allowing for vehicle-specific subdirectories if a `--vehicle_id` is provided.

4. **YAML Parsing**: The `read_yaml` function reads each parameter file. It specifically extracts the dictionary under the `/**"]["ros__parameters"]` key, which is the standard location for ROS 2 node parameters in Autoware YAML files. Error handling is included for file reading issues.

5. **Parameter Sources**: A `Source` class (using an integer enum internally) is defined to represent the origin of each parameter set (MPC, PID, SIM, GATE). A helper function `to_string` converts the source enum to a human-readable string for output messages.

6. **Condition Definition**: A `Condition` class encapsulates a single consistency check. Each `Condition` instance stores:

   - `param1`, `source1`: The name and origin of the first parameter.
   - `param2`, `source2`: The name and origin of the second parameter.
   - `fail_msg`: A string describing the expected relationship (e.g., "should be identical to", "should be lower than").
   - `op`: An operator function (like `operator.eq`, `operator.lt`) or a custom lambda function that takes the two parameter values and returns `True` if the condition is met, `False` otherwise.

7. **Evaluation**:
   - A list of predefined `Condition` objects is created, specifying the exact parameters to compare and the expected relationship between them (e.g., `MPC.input_delay` should equal `SIM.steer_time_delay`).
   - The script iterates through each `Condition`.
   - The `evaluate` method of the `Condition` class retrieves the corresponding parameter values from the loaded `sources` dictionary.
   - It applies the condition's operator (`op`) to the values.
   - If the operator returns `False` (meaning the condition failed), it formats a detailed error message indicating the source, parameter name, value, the failed relationship, and prints it to the console in red text (`\033[91m...\033[0m`).
   - If the operator returns `True`, `evaluate` returns `None`, and nothing is printed for that condition.
