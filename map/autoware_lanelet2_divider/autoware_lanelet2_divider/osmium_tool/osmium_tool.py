import os
import subprocess

from autoware_lanelet2_divider.debug import Debug
from autoware_lanelet2_divider.debug import DebugMessageType

# Error handler dictionary for osmium tool
# Key: Error message
# Value: List of functions to handle the error
#   - First function: Function to log the error
#   - Second function: Function to log the action
#   - Third function: Function to execute the action
ERROR_HANDLERS = {
    "Output directory is missing or not accessible": [
        lambda input_osm_file_path, input_config_file_path, output_dir: Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, Output directory is missing or not accessible",
            DebugMessageType.ERROR,
        ),
        lambda input_osm_file_path, input_config_file_path, output_dir: f"Creating output directory: {output_dir}",
        lambda input_osm_file_path, input_config_file_path, output_dir: os.makedirs(
            output_dir, exist_ok=True
        ),
    ],
    "Way IDs out of order / Relation IDs out of order": [
        lambda input_osm_file_path, input_config_file_path, output_dir: Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, Way IDs out of order",
            DebugMessageType.ERROR,
        ),
        lambda input_osm_file_path, input_config_file_path, output_dir: Debug.log(
            f"Sorting osm file: {input_osm_file_path}", DebugMessageType.INFO
        ),
        lambda input_osm_file_path, input_config_file_path, output_dir: sort_osm_file(
            input_osm_file_path
        ),
    ],
}


def extract_osm_file(
    input_osm_file_path: str,
    input_config_file_path: str,
    output_dir: str,
    args: str = "-v -s complete_ways -S types=any",
) -> bool:
    """
    Extract a specified .osm file using the osmium tool, with given arguments and configurations.

    Parameters:
        input_osm_file_path (str): Path to the input .osm file.
        input_config_file_path (str): Path to the configuration file for the osmium tool.
        output_dir (str): Path to the directory where the output will be stored.
        args (str): Additional arguments for osmium extraction command. Defaults to "-v -s complete_ways -S types=any".

    Returns:
        bool: True if extraction is successful, False otherwise.
    """
    command = f"osmium extract -c {input_config_file_path} --overwrite {input_osm_file_path} {args}"
    result = subprocess.run(
        command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    if result.returncode == 0:
        Debug.log(f"Extracted osm file: {input_osm_file_path}", DebugMessageType.SUCCESS)
        return True

    for error_message, (error_log, info_log, action) in ERROR_HANDLERS.items():
        if any(error in result.stderr for error in error_message.split(" / ")):
            error_log(input_osm_file_path, input_config_file_path, output_dir)  # Log the error
            info_log(input_osm_file_path, input_config_file_path, output_dir)  # Log the action
            action_output = action(
                input_osm_file_path, input_config_file_path, output_dir
            )  # Execute the action
            return extract_osm_file(
                action_output if action_output is not None else input_osm_file_path,
                input_config_file_path,
                output_dir,
            )

    Debug.log(
        f"Cannot extracted osm file: {input_osm_file_path}, {result.stderr}",
        DebugMessageType.ERROR,
    )
    return False


def sort_osm_file(input_osm_file_path: str) -> str:
    """
    Sort a specified .osm file using the osmium tool to handle out-of-order Way or Relation IDs.

    It stores the sorted .osm file in the same directory as the input file with "_sorted.osm" appended to the filename.

    Parameters:
        input_osm_file_path (str): Path to the input .osm file that needs sorting.

    Returns:
        str: Path to the sorted .osm file.
    """
    sorted_osm_file_path = input_osm_file_path.replace(".osm", "_sorted.osm")
    command = f"osmium sort {input_osm_file_path} -o {sorted_osm_file_path}"
    subprocess.run(
        command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    return sorted_osm_file_path
