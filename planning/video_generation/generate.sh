#!/usr/bin/env zsh
set -e

source /opt/ros/humble/setup.zsh
source /home/maximeclement/autoware/diffusion/pilot-auto.x2/install/setup.zsh

script_dir=${0:A:h}
python="$script_dir/.venv/bin/python"
if [[ ! -x $python ]]; then
    python=python3
fi
BAGS=(
    /media/maximeclement/PortableSSD1/MPPI/Odaiba_11-05/f4dbd93f-37d7-4948-b92e-a47d65e0bb7b_2026-11-05-10-33-11_p0900_0.mcap
)
exec "$python" "$script_dir/generate_bag_video.py" \
    --bag-files "${BAGS[@]}" \
    --rviz-config "$script_dir/mppi.rviz" \
    --vehicle-model j6_gen2 \
    --camera-config "$script_dir/mppi_bev_cameras.yaml" \
    --birdseye \
    --trajectory-topic /planning/trajectory \
    --mppi-trajectory-topic /planning/trajectory_generator/trajectory_modifier/debug/mppi/optimized_trajectory \
    --plot-config "$script_dir/mppi_plot.yaml" \
    --intermediate-dir "$script_dir/intermediate/mppi_bev_example" \
    --output-dir "$script_dir/output/mppi_bev_example" \
    --start-before-motion-s 1 \
    "$@"
