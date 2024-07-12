import paho.mqtt.client as mqtt
import base64
import yaml
import argparse
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

class MQTTPublisher:
    
    def __init__(self, config):
        self.broker = config['broker']
        self.port = config['port']
        self.username = config['username']
        self.password = config['password']
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

    def __init__(self, path, mqtt_pub):
        self.path = path
        self.pub = mqtt_pub

    def on_created(self, event):
        if event.is_directory:
            return
        
        print(f"New file created: {event.src_path}")
        
        with open(event.src_path, 'rb') as file:
            data = file.read()
        
        ext = event.src_path.split('.')[-1]    
        topic = 'lfs/' + ext
        
        self.pub.publish(topic, data)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Push new files to an MQTT broker')
    parser.add_argument('--path', type=str, help='Path to the directory to watch')
    args = parser.parse_args()
    
    with open('secret.yaml', 'r') as file:
        config = yaml.safe_load(file)

    mqtt_pub = MQTTPublisher(config)
    
    DirectoryWatcher(args.path, mqtt_pub)
    observer = Observer()
    observer.schedule(DirectoryWatcher(args.path, mqtt_pub), path=args.path, recursive=False)
    observer.start()
    
    print(f"Watching directory {args.path}")
    try:
        while True:
            pass
    except KeyboardInterrupt:
        observer.stop()
    observer.join()