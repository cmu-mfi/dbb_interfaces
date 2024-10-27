import time

import core.ros_callback as ros_callback
import rospy
from mqtt_spb_wrapper import MqttSpbEntityDevice
from roslib.message import get_message_class


class SPBDevice:
    def __init__(self, cfg: dict, name: str) -> None:
        self.cfg = cfg
        self.name = name
        self.device: MqttSpbEntityDevice = None

    def connect(self):
        group_name = self.cfg['mqtt']['group_name']
        edge_node_name = self.cfg['mqtt']['node_name']
        mqtt_host = self.cfg['mqtt']['broker_address']
        mqtt_port = int(self.cfg['mqtt']['broker_port'])
        mqtt_user = self.cfg['mqtt']['username']
        mqtt_pass = self.cfg['mqtt']['password']
        mqtt_tls_enabled = self.cfg['mqtt']['tls_enabled']
        debug = self.cfg['mqtt']['debug']
        device_name = self.name

        self.device = MqttSpbEntityDevice(group_name,
                                          edge_node_name,
                                          device_name,
                                          debug)

        # Connect to the broker --------------------------------------------
        _connected = False
        while not _connected:
            if debug:
                print("Trying to connect to broker...")
            _connected = self.device.connect(mqtt_host,
                                             mqtt_port,
                                             mqtt_user,
                                             mqtt_pass,
                                             mqtt_tls_enabled)
            if not _connected:
                print("  Error, could not connect. Trying again in a few seconds ...")
                time.sleep(3)
        if debug:
            print(f"Device {self.name} connected to broker")

    def publish_birth(self):
        attributes = self.cfg['devices'][self.name]['attributes']

        for attr in attributes:
            self.device.attributes.set_value(attr, attributes[attr])

        for topic in self.cfg['devices'][self.name]['topics']:
            topic_cfg = self.cfg['topics'][topic]
            prefix = topic_cfg['spb_prefix']
            for key in topic_cfg['spb_init']:
                self.device.data.set_value(f"{prefix}/{key}",
                                           topic_cfg['spb_init'][key])

        self.device.publish_birth()

    def create_callbacks(self):
        for topic in self.cfg['devices'][self.name]['topics']:
            topic_cfg = self.cfg['topics'][topic]
            prefix = topic_cfg['spb_prefix']
            rostopic = topic_cfg['rostopic']
            ros_type = topic_cfg['ros_type']
            func_name = topic_cfg['function']

            # func = ros_callback.function_mapper().get_function(func_name)
            func = getattr(ros_callback, func_name)            
            ros_msg_class = get_message_class(ros_type)
            
            try:
                if ros_msg_class is not None:
                    rospy.Subscriber(self.name+'/'+rostopic, ros_msg_class,
                                     func, callback_args=(self.device, prefix))
                                    #  lambda msg: func(self.device, msg, prefix))
                    print(f"Subscribed to topic {self.name+'/'+rostopic}, {ros_msg_class}, calling {func_name}")
                else:
                    print(f"Could not find message class for topic {self.name+rostopic}")
            except Exception as e:
                rospy.logerr(f"Error subscribing to topic {rostopic}: {e}")

