import os
import subprocess

from autoware_lanelet2_divider.debug import Debug
from autoware_lanelet2_divider.debug import DebugMessageType


def extract_osm_file(
    input_osm_file_path: str,
    input_config_file_path: str,
    output_dir: str,
    args: str = "-v -s complete_ways -S types=any",
) -> bool:
    command = f"osmium extract -c {input_config_file_path} --overwrite {input_osm_file_path} {args}"
    result = subprocess.run(
        command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    if result.returncode == 0:
        Debug.log(
            f"Extracted osm file: {input_osm_file_path}", DebugMessageType.SUCCESS
        )
        return True
    
    if "Output directory is missing or not accessible" in result.stderr:
        Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, Output directory is missing or not accessible",
            DebugMessageType.ERROR,
        )
        Debug.log(f"Creating output directory: {output_dir}", DebugMessageType.INFO)
        os.mkdir(output_dir)
        extract_osm_file(input_osm_file_path, input_config_file_path, output_dir)
        return True
    elif "Way IDs out of order" in result.stderr:
        Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, Way IDs out of order",
            DebugMessageType.ERROR,
        )
        Debug.log(f"Sorting osm file: {input_osm_file_path}", DebugMessageType.INFO)
        sorted_osm_file = sort_osm_file(input_osm_file_path)
        extract_osm_file(sorted_osm_file, input_config_file_path, output_dir)
        return True
    elif "Relation IDs out of order" in result.stderr:
        Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, Way IDs out of order",
            DebugMessageType.ERROR,
        )
        Debug.log(f"Sorting osm file: {input_osm_file_path}", DebugMessageType.INFO)
        sorted_osm_file = sort_osm_file(input_osm_file_path)
        extract_osm_file(sorted_osm_file, input_config_file_path, output_dir)
        return True
    else:
        Debug.log(
            f"Cannot extracted osm file: {input_osm_file_path}, {result.stderr}",
            DebugMessageType.ERROR,
        )
        print(result.stderr)
        return False


def sort_osm_file(input_osm_file_path: str) -> str:
    sorted_osm_file_path = input_osm_file_path.replace(".osm", "_sorted.osm")
    command = f"osmium sort {input_osm_file_path} -o {sorted_osm_file_path}"
    result = subprocess.run(
        command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )

    return sorted_osm_file_path
