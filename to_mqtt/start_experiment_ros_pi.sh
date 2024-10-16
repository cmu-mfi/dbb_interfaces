# echo "Chekcing mqtt_ros container is running"

# check if container is running
if [ "$(docker ps -q -f name=mqtt_ros)" ]; then
    echo "mqtt_ros container is running"
else
    echo "mqtt_ros container is not running. Please run the ros_mqtt_pi first"
    exit 1
fi

docker exec -it mqtt_ros bash -c 'source /opt/ros/noetic/setup.bash && python3 republish_birth.py'