import paho.mqtt.client as mqtt
import base64
import yaml
import argparse

class MQTTPublisher:
    
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
        
        # list of topics to subscribe 
        self.topics = self.config['topics']
        for topic in self.topics:
            self.client.subscribe(topic)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def on_message(self, client, userdata, message):
        print(f"Received message '{message.payload.decode()}' on topic '{message.topic}'")
        ...


if __name__ == '__main__':
    
    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)
        
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTPublisher(secret, config)
    
    mqtt_pub.client.loop_forever()
    
