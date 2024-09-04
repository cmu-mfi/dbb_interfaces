import roslibpy
import rospy
import yaml
from audio_handler import AudioHandler
from image_handler import ImageHandler
from mqtt_publisher import MQTTPublisher
from pcd_handler import PCDHandler


class ROSListener:
    def __init__(self, config) -> None:

        self.config = config
        rospy.init_node('ros_mqtt_cfs', anonymous=False)
        self.mqtt_obj = MQTTPublisher(config['secret'])
        self.on_ready()
        rospy.spin()

    def on_ready(self):
        print('ROS connected')
        for topic in config['ros_topics']:
            topic_info = config['ros_topics'][topic]
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
        # elif topic_config['type'] == 'sensor_msgs/PointCloud2':
        #     return PCDHandler(topic_config, self.ros_client, self.mqtt_obj)
        # elif topic_config['type'] == 'sensor_msgs/Image':
        #     return ImageHandler(topic_config, self.ros_client, self.mqtt_obj)
        
        else:
            # raise (f"Unknown topic type: {topic_config['type']}")
            pass


if __name__ == '__main__':

    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)

    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # merge secret and config
    config['secret'] = secret
    
    # add global config to each instance
    experiment_class = config['experiment_class']
    for topic in config['ros_topics']:
        if 'experiment_class' not in config['ros_topics'][topic]:
            config['ros_topics'][topic]['experiment_class'] = experiment_class

    obj = ROSListener(config)

    # try:
    #     while True:
    #         pass
    # except KeyboardInterrupt:
    #     del obj
    
    while rospy.is_shutdown():
        pass