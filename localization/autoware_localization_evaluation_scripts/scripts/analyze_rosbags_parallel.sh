#!/bin/bash
set -ux

result_dir=$1
parallelism=${2:-1}

process_directory() {
    local dir=$1

    target_rosbag="$dir/result_bag"
    compare_result_dir="$dir/compare_trajectories"
    mkdir -p "$compare_result_dir"

    ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py "$target_rosbag"

    ros2 run autoware_localization_evaluation_scripts extract_pose_from_rosbag.py \
        "$target_rosbag" \
        --save_dir "$compare_result_dir" \
        --target_topics "/localization/kinematic_state" \
        "/localization/reference_kinematic_state"

    ros2 run autoware_localization_evaluation_scripts compare_trajectories.py \
        "$compare_result_dir/localization__kinematic_state.tsv" \
        "$compare_result_dir/localization__reference_kinematic_state.tsv"
}

export -f process_directory

find "$result_dir" -mindepth 1 -maxdepth 1 -type d | sort |
    xargs -I {} -P "$parallelism" bash -c 'process_directory "{}"'
