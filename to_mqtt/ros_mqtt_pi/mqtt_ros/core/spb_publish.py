import os

import rospy
from core.spb_device import SPBDevice
from omegaconf import OmegaConf

class SPBPublish:

    def __init__(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        script_dir = os.path.join(script_dir, '../config')
        config_path = os.path.join(script_dir, 'config.yaml')
        rostopics_path = os.path.join(script_dir, 'rostopics.yaml')

        rospy.init_node('mqtt_publisher')

        config = OmegaConf.load(config_path)
        rostopics = OmegaConf.load(rostopics_path)

        # cleanup experiment_class
        if 'experiment_class' in rostopics:
            experiment_class = rostopics['experiment_class']
        elif 'experiment_class' in config:
            experiment_class = config['experiment_class']
        else:
            experiment_class = 'untitled'

        for device in config['devices'].keys():
            if 'experiment_class' not in config['devices'][device]['attributes']:
                config['devices'][device]['attributes']['experiment_class'] = experiment_class

        # get ros topic details and associate with respective devices
        for topic in rostopics['topics']:
            device = topic.split('/')[0]
            topic = topic[len(device)+1:]
            
            if device not in config['devices']:       
                print(f"Device {device} not found in config file. Skipping topic {topic}")
            device_cfg = config['devices'][device]
            
            if 'topics' not in device_cfg:
                device_cfg['topics'] = []
            
            for defined_topic in config['topics'].keys():
                if topic == config['topics'][defined_topic]['rostopic']:
                    if defined_topic not in device_cfg['topics']:
                        device_cfg['topics'].append(defined_topic)
                        

        for device in config['devices'].keys():
            spb_device = SPBDevice(config, device)
            spb_device.connect()
            spb_device.publish_birth()        
            spb_device.create_callbacks()

        rospy.spin()
        print("Application finished!")