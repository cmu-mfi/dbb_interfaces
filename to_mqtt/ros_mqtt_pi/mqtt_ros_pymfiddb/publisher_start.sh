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

if ! tmux has-session -t MQTT-ROS 2>/dev/null; then
        tmux new-session -d -s MQTT-ROS
fi

create_window "MQTT-ROS" "publisher"
# tmux send-keys -t MQTT-ROS:publisher 'cd /root/catkin_ws' C-m
# tmux send-keys -t MQTT-ROS:publisher 'catkin_make' C-m
tmux send-keys -t MQTT-ROS:publisher 'source ~/.bashrc' C-m
tmux send-keys -t MQTT-ROS:publisher '/home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/.venv/bin/python /home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/scripts/mqtt_pub_intercept.py' C-m

