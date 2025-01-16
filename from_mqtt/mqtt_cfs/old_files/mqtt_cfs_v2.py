import base64
import json
import os
import pickle
import time
import yaml

import numpy as np
import paho.mqtt.client as mqtt

class MQTTSubscriber:
    def __init__(self, mqtt_cfg, config):
        broker = mqtt_cfg['broker']
        port = mqtt_cfg['port']
        username = mqtt_cfg['username']
        password = mqtt_cfg['password']
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.__on_connect
        self.client.on_message = self.__on_message
        self.client.connect(broker, port, 60)
        
        self.config = config
        print(config)
        
        for topic in config['topics']:
            self.client.subscribe(topic)
        # self.client.subscribe("lfs/keyence")

    def __on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def __on_message(self, client, userdata, message):
        print(f"Received message on topic '{message.topic}'")       
        
        # data = base64.b64decode(message.payload)
        data_dict = pickle.loads(message.payload)
        
        path = self.config['output_path']
        folder = message.topic.replace('/', '.')
        path = os.path.join(path, folder)
        if not os.path.exists(path):
            os.makedirs(path)
            
        topic_type = message.topic.split('/')[0]
        if topic_type == 'lfs':
            self.__lfs_handler(data_dict, path)
        else:
            print(f"Topic '{topic_type}' not supported")
        
    def __lfs_handler(self, data: dict, output_path):

        # Get current time
        time_val = time.strftime("%Y-%m-%d_%H-%M")
        timestamp = f"{time_val}"
        
        # Check msg type, attributes or data
        if len(data) != 1:
            print(f'Uknown keys available in the message: {data.keys()}')
        else:
            msg_type = list(data.keys())[0]
            data = data[msg_type]
            if msg_type == 'data':
                # check if it has the required keys
                required_keys = ['file', 'name']
                if not all(key in data for key in required_keys):
                    print(f"Missing keys in the message: {required_keys}")
                    return
            elif msg_type == 'attributes':
                filename = f'{timestamp}.json'
                with open(os.path.join(output_path, filename), 'w') as file:
                    file.write(json.dumps(data, indent=4))
                return
            else:
                print(f'Uknown message type: {msg_type}')            
                return

        filename = "untitled"
        ext = ""
        experiment_class = ''.join(np.random.choice(list('0123456789abcdefghijklmnopqrstuvwxyz'), 4))
        
        # Appending the filename with the name, experiment_class and timestamp
        if 'name' in data:
            name = data['name']
            filename = name.split('.')[0]
            ext = name.split('.')[1]
                
        if 'experiment_class' in data:
            if data['experiment_class'] != '':
                experiment_class = data['experiment_class']
        filename += f"_{experiment_class}"
        
        if 'timestamp' in data:
            timestamp = data['timestamp']
        filename += f"_{timestamp}"

        filename += f".{ext}"        
        with open(os.path.join(output_path, filename), 'wb') as file:
            file.write(data['file'])
        
        # Saving metadata as a json file
        metadata = data
        metadata.pop('file')
        filename = filename.split('.')[0]
        with open(os.path.join(output_path, f".{filename}_meta.json"), 'w') as file:
            file.write(json.dumps(metadata, indent=4))
        
        print(f"File {filename} saved to {output_path}")  
        
if __name__ == '__main__':
    
    # LOAD CONFIG FILES
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(current_dir, 'config')
    
    mqtt_config = os.path.join(config_dir, 'mqtt.yaml')
    mqtt_cfs_config = os.path.join(config_dir, 'config.yaml')
    
    with open(mqtt_config, 'r') as file:
        secret = yaml.safe_load(file)
        
    with open(mqtt_cfs_config, 'r') as file:
        config = yaml.safe_load(file)


    # MQTT SUBSCRIBER
    mqtt_pub = MQTTSubscriber(secret, config)
    
    mqtt_pub.client.loop_forever()
    