#!/bin/bash

# Name of the tmux session
SESSION_NAME="robot_session"

# Kill the tmux session
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "Terminated tmux session: $SESSION_NAME"
