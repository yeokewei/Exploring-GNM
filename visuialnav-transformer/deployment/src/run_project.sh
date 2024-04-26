#!/bin/bash

# First Parameter: get the launch file name(either cafe or office, default: cafe)
dumpster_launch_name="${1:-cafe}"
# Second Parameter: _speed (between 0 and 1)
speed="${2:-0.5}"
# Third Parameter: _turn (between 0 and 1)
turn="${3:-0.5}"

# Start TMUX session
SESSION_NAME="GNM_simulation"
tmux new-session -d -s $SESSION_NAME

# Setup Window 1: Simulation
tmux rename-window -t 0 'Simulation'
tmux split-window -h
tmux send-keys -t 'Simulation.0' 'conda activate vint_deployment' C-m
tmux send-keys -t 'Simulation.0' 'cd ~/Documents/GitHub/MobileRobotics/catkin_ws' C-m
tmux send-keys -t 'Simulation.0' 'source devel/setup.bash' C-m
tmux send-keys -t 'Simulation.0' "roslaunch rtab_dumpster $dumpster_launch_name.launch" C-m

tmux send-keys -t 'Simulation.1' 'conda activate vint_deployment' C-m
tmux send-keys -t 'Simulation.1' 'cd ~/Documents/GitHub/MobileRobotics/catkin_ws' C-m
tmux send-keys -t 'Simulation.1' 'source devel/setup.bash' C-m
tmux send-keys -t 'Simulation.1' "sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=$speed _turn:=$turn" C-m

tmux splitw -v -p 50 # split it into two halves

tmux send-keys -t 'Simulation.2' 'noetic' C-m

# Attach to session
tmux attach-session -t $SESSION_NAME

