import paho.mqtt.client as mqtt
import base64
import yaml
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import pickle
import platform
import argparse

class MQTTPublisher:
    
    def __init__(self, secret):
        self.broker = secret['broker']
        self.port = secret['port']
        self.username = secret['username']
        self.password = secret['password']
        self.client = mqtt.Client()
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.connect(self.broker, self.port, 60)

    def __del__(self):
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")

    def publish(self, topic, payload):
        print(f"Publishing to topic {topic}")
        encoded_data = base64.b64encode(payload)
        self.client.publish(topic, encoded_data)

class DirectoryWatcher(FileSystemEventHandler):

    def __init__(self, config, mqtt_pub):
        self.path = config['watch_dir']
        self.pub = mqtt_pub
        self.config = config

    def on_created(self, event):
        if event.is_directory:
            return
        
        print(f"New file created: {event.src_path}")
        
        ext = event.src_path.split('.')[-1]    
        topic = 'lfs/' + ext
        data_dict = {}
        
        for key in self.config['message_dict']:
            value = self.get_event_data(event, key)
            if value is not None:
                data_dict[key] = value
                
        data = pickle.dumps(data_dict)
        self.pub.publish(topic, data)
        
    def get_event_data(self, event, key):
        if key == 'name':
            if platform.system() == 'Windows':
                name = event.src_path.split('\\')[-1]
            else:
                name = event.src_path.split('/')[-1]
            print(f"Name: {name}")
            return name
        elif key == 'timestamp':
            format = "%Y-%m-%d_%H-%M"
            return time.strftime(format)
        elif key == 'file':
            with open(event.src_path, 'rb') as file:
                data = file.read()
            return data
        elif key == 'experiment_class':
            try:
                return self.config['experiment_class']
            except:
                return ""
        else:
            return None

if __name__ == '__main__':
    
    with open('secret.yaml', 'r') as file:
        secret = yaml.safe_load(file)
        
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTPublisher(secret)

    observers = []    
    for dir in config['watch_dir']: 
        observer = Observer()
        observer.schedule(DirectoryWatcher(config, mqtt_pub), path=dir, recursive=True)
        observer.start()
        print(f"Watching directory {dir}")
        observers.append(observer)
    
    mqtt_pub.client.loop_forever()
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        for observer in observers:
            observer.stop()
    for observer in observers:
        observer.join()