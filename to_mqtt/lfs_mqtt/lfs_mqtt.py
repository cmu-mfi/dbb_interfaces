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
    """
    A class to handle MQTT publishing with authentication.
    Attributes:
    -----------
    broker : str
        The MQTT broker address.
    port : int
        The port number to connect to the MQTT broker.
    username : str
        The username for MQTT broker authentication.
    password : str
        The password for MQTT broker authentication.
    client : mqtt.Client
        The MQTT client instance.
    Methods:
    --------
    __init__(secret):
        Initializes the MQTTPublisher with broker details and credentials.
    __del__():
        Disconnects the MQTT client upon object deletion.
    on_connect(client, userdata, flags, rc):
        Callback function for when the client receives a CONNACK response from the server.
    publish(topic, payload):
        Publishes a base64 encoded payload to a specified MQTT topic.
    """    
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
    """
    A class to watch a directory for new file creations and publish events to an MQTT topic.
    Attributes:
    -----------
    path : str
        The directory path to watch for new files.
    pub : MQTTClient
        The MQTT client used to publish messages.
    config : dict
        Configuration dictionary containing watch directory, message dictionary, and other settings.
    Methods:
    --------
    on_created(event):
        Handles the event when a new file is created in the watched directory.
    get_event_data(event, key):
        Retrieves specific data from the event based on the provided key.
    """

    def __init__(self, config, mqtt_pub):
        """
        Initializes the DirectoryWatcher with the given configuration and MQTT client.
        Parameters:
        -----------
        config : dict
            Configuration dictionary containing watch directory, message dictionary, and other settings.
        mqtt_pub : MQTTClient
            The MQTT client used to publish messages.
        """
        self.path = config['watch_dir']
        self.pub = mqtt_pub
        self.config = config

    def on_created(self, event):
        """
        Handles the event when a new file is created in the watched directory.
        Parameters:
        -----------
        event : FileSystemEvent
            The event object representing the file creation event.
        """
        if event.is_directory:
            return
        
        print(f"New file created: {event.src_path}")
        
        time.sleep(1)
        
        ext = event.src_path.split('.')[-1]    
        topic = 'lfs/' + ext
        data_dict = {}
        
        for key in self.config['message_dict']:
            value = self.get_event_data(event, key)
            print(f"{key}: {value}")
            if value is not None:
                data_dict[key] = value
                
        data = pickle.dumps(data_dict)
        self.pub.publish(topic, data)
        
    def get_event_data(self, event, key):
        """
        Retrieves specific data from the event based on the provided key.
        Parameters:
        -----------
        event : FileSystemEvent
            The event object representing the file creation event.
        key : str
            The key indicating which data to retrieve from the event.
        Returns:
        --------
        Any
            The data corresponding to the provided key. Returns None if the key is not recognized.
        """
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