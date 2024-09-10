#!/bin/bash


# set errexit so that the script will exit on any error
set -e

# Setup ROS environment
source ~/.bashrc

# Check if a tmux window exists
window_exists() {
        tmux list-windows -t "$1" | grep -q "$2"
}

# Create new tmux window if it doesn't exist
create_window() {
        if ! window_exists "$1" "$2"; then
                tmux new-window -n "$2" -t "$1"
        fi
}

if ! tmux has-session -t DDB_UPSTREAM 2>/dev/null; then
        tmux new-session -d -s DDB_UPSTREAM
fi


# Echo instruction to check file and wait for user to press enter
echo "Check following config files before proceeding:"
echo "1. ./ros_mqtt_pi/mqtt_ros/mqtt_transmitter/scripts/config.yaml"
echo "2. ./ros_mqtt_cfs/config.yaml"
echo "3. ./lfs_mqtt/config.yaml"
echo "Press Enter when done..."
read

echo "Starting DATA STREAMING..."

create_window "DDB_UPSTREAM" "lfs_mqtt"
tmux send-keys -t DDB_UPSTREAM:lfs_mqtt 'cd lfs_mqtt/' C-m
tmux send-keys -t DDB_UPSTREAM:lfs_mqtt 'source venv/bin/activate' C-m
tmux send-keys -t DDB_UPSTREAM:lfs_mqtt 'python3 lfs_mqtt.py' C-m

create_window "DDB_UPSTREAM" "ros_mqtt_cfs"
tmux send-keys -t DDB_UPSTREAM:ros_mqtt_cfs 'cd ros_mqtt_cfs/' C-m
tmux send-keys -t DDB_UPSTREAM:ros_mqtt_cfs 'source venv/bin/activate' C-m
tmux send-keys -t DDB_UPSTREAM:ros_mqtt_cfs 'python3 ros_mqtt_cfs.py' C-m

create_window "DDB_UPSTREAM" "ros_mqtt_pi"
tmux send-keys -t DDB_UPSTREAM:ros_mqtt_pi 'cd ros_mqtt_pi' C-m
tmux send-keys -t DDB_UPSTREAM:ros_mqtt_pi 'docker compose up --build' C-m

# tail -f /dev/null
