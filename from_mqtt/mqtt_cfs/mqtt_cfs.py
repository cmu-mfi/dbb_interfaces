import paho.mqtt.client as mqtt
import base64
import yaml
import argparse
import pickle
import time
import os

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
            self.client.subscribe(topic)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        print(f"Received message '{message.payload.decode()}' on topic '{message.topic}'")
        data = base64.b64decode(message.payload)
        dict = pickle.loads(data)
        filename = "untitled"
        expected_ext = message.topic.split('/')[-1]
        ext = expected_ext
        
        if 'name' in dict:
            name = dict['name']
            filename = name.split('.')[0]
            ext = name.split('.')[-1]
            if ext == filename:
                ext = expected_ext
        
        if 'timestamp' in dict:
            timestamp = dict['timestamp']
            filename += f"_{timestamp}"
        
        format = "%Y-%m-%d_%H-%M"
        filename += f"_{time.strftime(format)}" 
        filename += f".{ext}"
        
        path = self.config['output_path']
        path = os.path.join(path, expected_ext)
        if not os.path.exists(path):
            os.makedirs(path)
        
        with open(os.path.join(path, filename), 'wb') as file:
            file.write(dict['file'])
        print(f"File {filename} saved to {path}")
        
if __name__ == '__main__':
    
    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)
        
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTSubscriber(secret, config)
    
    mqtt_pub.client.loop_forever()
    
