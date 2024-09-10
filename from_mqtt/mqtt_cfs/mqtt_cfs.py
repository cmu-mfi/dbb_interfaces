import base64
import os
import pickle
import time

import numpy as np
import paho.mqtt.client as mqtt
import yaml
from audio_download import AudioDownload
from image_download import ImageDownload
from pcd_download import PCDDownload


class MQTTSubscriber:
    
    def __init__(self, secret, config):
        self.broker = secret['broker']
        self.port = secret['port']
        self.username = secret['username']
        self.password = secret['password']
        self.client = mqtt.Client()
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, self.port, 60)
        
        self.config = config
        
        print(config)
        
        # list of topics to subscribe 
        self.topics = self.config['topics']
        for topic in self.topics:
            print(f"Subscribing to topic '{topic}'")
            self.client.subscribe(topic)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        print(f"Received message on topic '{message.topic}'")       
        
        data = base64.b64decode(message.payload)
        dict = pickle.loads(data)
        
        path = self.config['output_path']
        folder = message.topic.replace('/', '.')
        path = os.path.join(path, folder)
        if not os.path.exists(path):
            os.makedirs(path)
            
        if message.topic in self.config:
            self.custom_handler(dict, self.config[message.topic], path)
            return        
        
        self.general_handler(message, dict, path)
        
    def custom_handler(self, dict, config, output_path):
        if config['type'] == 'sounddevice_ros/AudioData':
            print("AudioData")
            handler = AudioDownload(config)
            
        elif config['type'] == 'sensor_msgs/PointCloud2':
            print("PointCloud2")
            handler = PCDDownload(config)
            
        elif config['type'] == 'sensor_msgs/Image':
            print("Image")
            handler = ImageDownload(config)
            
        handler.save_file(dict, output_path)
    
    def general_handler(self, message, dict, output_path):
        filename = "untitled"
        experiment_class = ''.join(np.random.choice(list('0123456789abcdefghijklmnopqrstuvwxyz'), 4))
        time_val = time.strftime("%Y-%m-%d_%H-%M")
        timestamp = f"{time_val}"
        expected_ext = message.topic.split('/')[-1]
        ext = expected_ext
        
        if 'name' in dict:
            name = dict['name']
            filename = name.split('.')[0]
            ext = name.split('.')[-1]
            if ext == filename:
                ext = expected_ext
                
        if 'experiment_class' in dict:
            if dict['experiment_class'] != '':
                experiment_class = dict['experiment_class']
        filename += f"_{experiment_class}"
        
        if 'timestamp' in dict:
            timestamp = dict['timestamp']
        filename += f"_{timestamp}"

        filename += f".{ext}"        
        with open(os.path.join(output_path, filename), 'wb') as file:
            file.write(dict['file'])
        print(f"File {filename} saved to {output_path}")
            
        
if __name__ == '__main__':
    
    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)
        
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTSubscriber(secret, config)
    
    mqtt_pub.client.loop_forever()
    
