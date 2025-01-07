#!/usr/bin/env python3

from mfi_ddb import PullStreamToMqttSpb, RosDataObject
# from mfi_ddb.data_objects import RosDataObject
import yaml

if __name__ == "__main__":
    
    ros_config_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/config/config.yaml"
    mqtt_config_file = "/home/mfi/repos/dbb_interfaces/to_mqtt/ros_mqtt_pi/mqtt_ros_pymfiddb/config/config.yaml"
    
    with open(ros_config_file, 'r') as file:
        ros_config = yaml.load(file, Loader=yaml.FullLoader)
    
    data_obj = RosDataObject(ros_config)
    
    PullStreamToMqttSpb(mqtt_config_file, data_obj)