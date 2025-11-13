#!/usr/bin/env bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

# Remove ROS paths that interfere
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)


python "$@"