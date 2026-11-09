#!/usr/bin/env zsh
set -e

source /opt/ros/humble/setup.zsh
source /home/maximeclement/autoware/diffusion/pilot-auto.x2/install/setup.zsh

script_dir=${0:A:h}
python="$script_dir/.venv/bin/python"
if [[ ! -x $python ]]; then
    python=python3
fi
exec "$python" "$script_dir/generate_bag_video.py" "$@"
