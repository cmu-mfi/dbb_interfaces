import base64
import time
import paho.mqtt.client as mqtt


class MQTTPublisher:

    def __init__(self, secret):
        self.broker = secret['broker']
        self.port = secret['port']
        self.username = secret['username']
        self.password = secret['password']
        self.client = mqtt.Client()
        self.is_connected = False
        self.client.username_pw_set(self.username, self.password)
        self.client.on_connect = self.on_connect
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        
        while not self.is_connected:
            print('Waiting for MQTT connection...')
            time.sleep(1)


        print('MQTT Publisher initialized')

    def __del__(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        self.is_connected = True
        print(f"Connected with result code {rc}")

    def publish(self, topic, payload):
        print(f"Publishing to mqtt topic {topic} of payload size {len(payload)}")
        encoded_data = base64.b64encode(payload)
        result = self.client.publish(topic, encoded_data, qos=1)
        status = result.rc
        if status == 0:
            print(f"Successfully published to MQTT topic {topic}")
        else:
            print(f"Failed to publish to MQTT topic {topic}, return code {status}")