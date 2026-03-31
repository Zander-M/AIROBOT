#!/bin/bash

TRAJECTORY_PATH="data/per_robot"

tmux new-session -d -s exp

# Left pane: simulator
tmux send-keys -t exp "ros2 launch airobot_bringup multi_robot_trajectory_tracking.launch.py trajectory_path:=$TRAJECTORY_PATH" C-m

# Right pane: controller
tmux split-window -h -t exp
tmux send-keys -t exp:0.1 "ros2 run airobot_bringup experiment_controller --ros-args -p trajectory_path:=$TRAJECTORY_PATH" C-m

# Attach
tmux attach -t exp