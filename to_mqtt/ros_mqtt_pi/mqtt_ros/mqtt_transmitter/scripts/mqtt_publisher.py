#!/usr/bin/env python3

import os
import time

import ros_callback
import rospy
import yaml
from mqtt_spb_wrapper import MqttSpbEntityDevice
from roslib.message import get_message_class
from spb_device import SPBDevice

if __name__ == "__main__":

    # Create the spB entity object
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(script_dir, 'config.yaml')

    rospy.init_node('mqtt_publisher')

    with open(config_path, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        
    # add global config to each instance
    experiment_class = config['experiment_class']

    for device in config['devices'].keys():
        if 'experiment_class' not in config['devices'][device]['attributes']:
            config['devices'][device]['attributes']['experiment_class'] = experiment_class

    for device in config['devices'].keys():
        spb_device = SPBDevice(config, device)
        spb_device.connect()
        spb_device.publish_birth()        
        spb_device.create_callbacks()

    rospy.spin()
    print("Application finished!")
