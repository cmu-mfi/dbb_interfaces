#!/usr/bin/env python3

import os

import rospy
import yaml
from core.spb_device import SPBDevice


class RepublishBirth:
    def __init__(self) -> None:
        # Create the SPB entity object
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, 'config.yaml')

        rospy.init_node('mqtt_publisher')

        with open(config_path, 'r') as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
        
        self.config = config
        
        devices = input("Enter devices to initialize (comma separated) (example: yk_destroyer, yk_creator): ")
        devices = devices.split(',')

        experiment_class = input("Enter experiment class: ")

        self.init_devices(experiment_class, devices)

        # Wait for user to press Q to quit
        char = input("Press Q to quit when experiment is done: ")
        while char != 'Q':
            char = input("Press Q to quit when experiment is done: ")

    # destructor
    def __del__(self):
        print("Destructor called, restoring defaults")
        # Restore default values
        experiment_class = self.config['experiment_class']
        self.init_devices(experiment_class)
        print("Application finished!")        

    def init_devices(self, experiment_class, devices=None):
        # CHECK IF DEVICES ARE VALID
        invalid_devices = None
        if devices is None:
            devices = self.config['devices'].keys()
        else:
            devices = [device for device in devices if device in self.config['devices'].keys()]
            invalid_devices = [device for device in devices if device not in self.config['devices'].keys()]
            
        if invalid_devices:
            print(f"Invalid devices: {invalid_devices}")

        # INITIALIZING VALID DEVICES
        print(f"Initializing devices: {devices}")    
        
        for device in devices:
            self.config['devices'][device]['attributes']['experiment_class'] = experiment_class
                
        for device in devices:
            spb_device = SPBDevice(self.config, device)
            spb_device.connect()
            spb_device.publish_birth()
