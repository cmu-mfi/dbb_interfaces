import base64
import datetime
import os
import pickle
import threading

import numpy as np
import roslibpy
import soundfile as sf
from mqtt_publisher import MQTTPublisher


class AudioHandler:
    def __init__(self, config, ros_client, mqtt_obj: MQTTPublisher = None) -> None:
        self.config = config
        self.topic = config['name']
        self.interval_sec = config['duration_min'] * 60
        self.audio_data = []
        self.publish_audio_data = []
        self.start_time = None
        self.publish_start_time = None
        self.audio_info = None
        self.ros_client = ros_client

        if mqtt_obj is not None:
            self.mqtt_obj = mqtt_obj

        self.audio_info_sub = roslibpy.Topic(
            self.ros_client, self.config['audio_info_topic'], 'sounddevice_ros/AudioInfo')
        self.audio_info_sub.subscribe(
            lambda message: self.get_audio_info(message))

        self.audio_feature_pub = []
        for i in range(len(config['audio_features'])):
            audio_feature_pub = roslibpy.Topic(
                self.ros_client, config['audio_feature_topics'][i], 'std_msgs/Float32')
            self.audio_feature_pub.append(audio_feature_pub)

        print('Audio handler initialized')

    def __del__(self):
        self.stream_audio()
        self.save_audio()

    def reset(self):
        self.audio_data = []
        self.start_time = None

    def save_audio(self):
        print('Saving audio...')
        time_format = '%Y-%m-%d_%H-%M'
        start_time = self.publish_start_time.strftime(time_format)
        filepath = f"{self.config['fileprefix']}_{start_time}.{self.config['fileext']}"

        dir = 'logs'
        os.makedirs(dir, exist_ok=True)
        filepath = os.path.join(dir, filepath)

        sound_file = sf.SoundFile(filepath, mode='w',
                                  samplerate=self.audio_info['sample_rate'],
                                  channels=self.audio_info['num_channels'],
                                  subtype=None if self.audio_info['subtype'] == '' else self.audio_info['subtype'])

        for data in self.publish_audio_data:
            sound_file.write(np.asarray(data).reshape(
                (-1, self.audio_info['num_channels'])))
            
        print(f'Audio saved to {filepath}') 

    def stream_audio(self):
        audio_dict = {
            'audio_info': self.audio_info,
            'config': self.config,
            'start_time': self.publish_start_time.strftime('%Y-%m-%d_%H-%M'),
            'audio_data': self.publish_audio_data,
        }
        
        size = 0
        for data in self.publish_audio_data:
            size += len(data)
        print(f'Streaming audio of size {size}')
                
        audio_pickle = pickle.dumps(audio_dict)
        self.mqtt_obj.publish(self.config['mqtt_topic'], audio_pickle)

    def get_audio_info(self, message=None):
        print('Audio info received')
        self.audio_info = message
        self.audio_info_sub.unsubscribe()

    def callback(self, message):
        if self.start_time is None:
            print('Recording audio...')
            self.start_time = datetime.datetime.now()

        self.audio_data.append(message['data'])

        now = datetime.datetime.now()
        if (now - self.start_time).seconds > self.interval_sec:
            self.publish_audio_data = self.audio_data
            self.publish_start_time = self.start_time
            save_audio_thread = threading.Thread(target=self.save_audio)
            stream_audio_thread = threading.Thread(target=self.stream_audio)
            if self.config['keep_log']:
                save_audio_thread.start()
            stream_audio_thread.start()
            self.reset()
            

        for i in range(len(self.config['audio_features'])):
            audio_feature = self.config['audio_features'][i]
            audio_feature_pub = self.audio_feature_pub[i]

            if audio_feature == 'rms':
                feature_value = np.sqrt(np.mean(np.square(message['data'])))

            elif audio_feature == 'zcr':
                feature_value = np.mean(
                    np.abs(np.diff(np.sign(message['data']))))
                
            elif audio_feature == 'max_freq':
                feature_value = np.max(np.abs(np.fft.fft(message['data'])))                
            
            audio_feature_pub.publish(
                roslibpy.Message({'data': feature_value}))
