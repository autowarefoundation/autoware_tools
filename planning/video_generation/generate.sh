#!/usr/bin/env zsh
set -e

source /opt/ros/humble/setup.zsh
source /home/maximeclement/autoware/diffusion/pilot-auto.x2/install/setup.zsh

script_dir=${0:A:h}
python="$script_dir/.venv/bin/python"
if [[ ! -x $python ]]; then
    python=python3
fi
video_root=/media/maximeclement/PortableSSD1/MPPI/video_generation
export TMPDIR="$video_root/tmp"
mkdir -p "$TMPDIR"

INPUT_ARGS=(
    --project-id x2_dev
    --vehicle-id f4dbd93f-37d7-4948-b92e-a47d65e0bb7b
    --datetime-from 2026-11-12T04:00:00Z
    --datetime-to 2026-11-12T06:00:00Z
)
for arg in "$@"; do
    if [[ $arg == --bag-files ]]; then
        INPUT_ARGS=()
        break
    fi
done
exec "$python" "$script_dir/generate_bag_video.py" \
    "${INPUT_ARGS[@]}" \
    --rviz-config "$script_dir/mppi.rviz" \
    --vehicle-model j6_gen2 \
    --camera-config "$script_dir/mppi_bev_cameras.yaml" \
    --birdseye \
    --trajectory-topic /planning/trajectory_generator/neural_network_based_planner/ml_planner_node/output/trajectory \
    --mppi-trajectory-topic /planning/trajectory_generator/trajectory_modifier/debug/mppi/optimized_trajectory \
    --sensor-calibration-dir /home/maximeclement/autoware/diffusion/pilot-auto.x2/src/autoware/individual_params/individual_params/config/j6_gen2_34/aip_x2_gen2 \
    --plot-config "$script_dir/mppi_plot.yaml" \
    --intermediate-dir "$video_root/intermediate" \
    --output-dir "$script_dir/output/mppi_bev_example" \
    --start-before-motion-s 1 \
    "$@"
