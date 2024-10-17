#!/bin/bash

# Name of the tmux session
SESSION_NAME="robot_session"

# Path to your Python script
PYTHON_SCRIPT="/root/real/main_server.py"  # <-- Replace with the actual path

# Optional: Catkin workspace setup (if you're using one)
CATKIN_WS_SETUP="/root/sim/catkin_ws/devel/setup.bash"  # <-- Replace if different



# Check if the session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    # Create a new tmux session and run the Flask-ROS application in the first window
    tmux new-session -d -s $SESSION_NAME -n 'ros-flask'
    
    # Start the application in the first pane
    tmux send-keys -t $SESSION_NAME "python3 /root/real/main_server.py" C-m

    # Split the window vertically (you can change to horizontal with `-h`)
    tmux split-window -v -t $SESSION_NAME

    # Switch to the new pane for interaction
    tmux select-pane -t $SESSION_NAME:0.1
fi

# Attach to the tmux session
tmux attach -t $SESSION_NAME
