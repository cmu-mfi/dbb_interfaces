#!/bin/bash

# Name of the tmux session to terminate
SESSION_NAME="DDB_UPSTREAM"

# Function to send Ctrl-C to all windows in the session
send_ctrl_c_to_all_windows() {
    # List all windows in the session
    windows=$(tmux list-windows -t "$SESSION_NAME" -F "#{window_index}")

    # Loop through each window and send Ctrl-C
    for win in $windows; do
        tmux send-keys -t "$SESSION_NAME:$win" C-c
    done
}

# Function to kill the tmux session
kill_tmux_session() {
    tmux kill-session -t "$SESSION_NAME"
}

# Send Ctrl-C to all windows
send_ctrl_c_to_all_windows

# Optionally wait a bit to ensure processes have time to handle Ctrl-C
sleep 5

# Kill the tmux session
kill_tmux_session

