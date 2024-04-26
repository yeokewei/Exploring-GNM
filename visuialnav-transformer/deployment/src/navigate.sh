#!/bin/bash

# 1st Parameter: Launch file name
dumpster_launch_name="${1:-cafe}"
# 2nd Parameter: Model and directory as one string
model_and_dir="$2"
# 3rd Parameter: Navigation method or identifier
nav_rosbag="$3"

# Create a new tmux session
session_name="gnm_navigate"
tmux new-session -d -s $session_name
tmux rename-window -t 0 'Navigate'

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 2    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves

tmux selectp -t 3    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Run the roslaunch command in the first pane
tmux select-pane -t 0
# tmux send-keys "roslaunch vint_locobot.launch" Enter
tmux send-keys "noetic" Enter
tmux send-keys 'conda activate vint_deployment' Enter
tmux send-keys 'cd ~/Documents/GitHub/MobileRobotics/catkin_ws' Enter
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys "roslaunch rtab_dumpster $dumpster_launch_name.launch" Enter


# Run the navigate.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "noetic" Enter
tmux send-keys "conda activate vint_deployment" Enter
# tmux send-keys "cd ~/Documents/GitHub/visualnav-transformer/deployment/src" Enter
# tmux send-keys "sleep 5; python navigate.py $@" Enter
tmux send-keys "sleep 2; python navigate.py $model_and_dir" C-m


# Run the pd_controller.py script in the third pane
tmux select-pane -t 2
tmux send-keys -t 'conda activate vint_deployment' C-m
tmux send-keys -t 'cd ~/Documents/GitHub/MobileRobotics/catkin_ws' C-m
tmux send-keys -t 'source devel/setup.bash' C-m
tmux send-keys -t "sleep 5; rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.5 _turn:=0.5" C-m

# Run the teleop.py script in the fourth pane
tmux select-pane -t 3
tmux send-keys "noetic" Enter
tmux send-keys -t 'Navigate.2' 'conda activate vint_deployment' C-m
tmux send-keys -t 'Navigate.2' 'cd ~/Documents/GitHub/MobileRobotics/catkin_ws' C-m
tmux send-keys -t 'Navigate.2' 'source devel/setup.bash' C-m
tmux send-keys -t 'Navigate.2' 'sleep 2; rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=1 _turn:=0.5' C-m
# tmux send-keys "python joy_teleop.py" Enter


tmux selectp -t 4
tmux send-keys "noetic" Enter
tmux send-keys "conda activate vint_deployment" Enter
tmux send-keys "cd ../nav_rosbag" Enter
tmux send-keys "sleep 6; rosbag record -O $nav_rosbag /odom" Enter
# Attach to the tmux session
tmux -2 attach-session -t $session_name
