import roslibpy
import yaml
from audio_handler import AudioHandler
from mqtt_publisher import MQTTPublisher
from pcd_handler import PCDHandler


class ROSListener:
    def __init__(self, config) -> None:

        self.config = config
        self.ros_client = roslibpy.Ros(
            host=config['ros_websocket']['host'], port=config['ros_websocket']['port'])

        self.ros_client.on_ready(self.on_ready)

        self.mqtt_obj = MQTTPublisher(config['secret'])

        self.listeners = []

        self.ros_client.run()

    def on_ready(self):
        print('ROS connected:', self.ros_client.is_connected)
        for topic in config['ros_topics']:
            topic_info = config['ros_topics'][topic]
            listener = roslibpy.Topic(
                self.ros_client, topic, topic_info['type'])
            topic_handler = self.get_topichandler(topic)
            listener.subscribe(topic_handler.callback)
            print(f"Subscribed to {topic}")
            self.listeners.append(listener)

    def __del__(self):
        for listener in self.listeners:
            listener.unsubscribe()
        self.ros_client.terminate()
        del self.mqtt_obj

    def get_topichandler(self, topic):
        topic_config = self.config['ros_topics'][topic]
        if topic_config['type'] == 'sounddevice_ros/AudioData':
            return AudioHandler(topic_config, self.ros_client, self.mqtt_obj)
        if topic_config['type'] == 'sensor_msgs/PointCloud2':
            return PCDHandler(topic_config, self.ros_client, self.mqtt_obj)
        
        else:
            raise (f"Unknown topic type: {topic_config['type']}")


if __name__ == '__main__':

    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)

    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # merge secret and config
    config['secret'] = secret

    obj = ROSListener(config)

    try:
        while True:
            pass
    except KeyboardInterrupt:
        del obj