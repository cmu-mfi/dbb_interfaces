import os

import rospy
import yaml
from core.audio_handler import AudioHandler
from core.image_handler import ImageHandler
from core.mqtt_publisher import MQTTPublisher
from core.pcd_handler import PCDHandler


class SPBPublish:
    def __init__(self) -> None:

        script_dir = os.path.dirname(os.path.realpath(__file__))
        script_dir = os.path.join(script_dir, '../config')
        config_path = os.path.join(script_dir, 'config.yaml')
        secret_path = os.path.join(script_dir, 'secret.yaml')
        
        with open(secret_path, 'r') as file:
            secret = yaml.safe_load(file)

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # merge secret and config
        config['secret'] = secret
        
        self.config = config
        self.read_config()
        breakpoint()
        # add global config to each instance
        experiment_class = config['experiment_class']
        for topic in config['ros_topics']:
            if 'experiment_class' not in config['ros_topics'][topic]:
                config['ros_topics'][topic]['experiment_class'] = experiment_class

        rospy.init_node('ros_mqtt_cfs', anonymous=False)
        self.mqtt_obj = MQTTPublisher(config['secret'])
        self.on_ready()
        rospy.spin()
        
        while not rospy.is_shutdown():
            pass
        
    def read_config(self):        
        for topic in self.config['ros_topics']:            
            # add default values
            defaults = self.config['ros_topics'][topic]['defaults']
            default_values = self.config['defaults'][defaults]
            for key in default_values:
                if key not in self.config['ros_topics'][topic]:
                    self.config['ros_topics'][topic][key] = default_values[key]
            
            # add rostopic name as a key
            self.config['ros_topics'][topic]['name'] = topic

    def on_ready(self):
        print('ROS connected')
        for topic in self.config['ros_topics']:
            topic_info = self.config['ros_topics'][topic]
            topic_handler = self.get_topichandler(topic)
            try:
                rospy.Subscriber(topic, type(topic_handler.dummy), topic_handler.callback)
                print(f"Subscribed to {topic}")
            except Exception as e:
                print(f"Failed to subscribe to {topic}: {e}")

    def get_topichandler(self, topic):
        topic_config = self.config['ros_topics'][topic]
        if topic_config['type'] == 'sounddevice_ros/AudioData':
            return AudioHandler(topic_config, self.mqtt_obj)            
        elif topic_config['type'] == 'sensor_msgs/PointCloud2':
            return PCDHandler(topic_config, self.mqtt_obj)
        elif topic_config['type'] == 'sensor_msgs/Image':
            return ImageHandler(topic_config, self.mqtt_obj)
 
        else:
            print(f"Unknown topic type: {topic_config['type']}")
            pass