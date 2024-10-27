import base64
import datetime
import os
import pickle
import threading

import numpy as np
import roslibpy
import rospy
import soundfile as sf
from core.mqtt_publisher import MQTTPublisher
from sounddevice_ros.msg import AudioData, AudioInfo
from std_msgs.msg import Float32


class AudioHandler:
    # def __init__(self, config, ros_client, mqtt_obj: MQTTPublisher = None) -> None:
    def __init__(self, config, mqtt_obj: MQTTPublisher = None) -> None:
        self.config = config
        self.topic = config['name']
        self.interval_sec = config['duration_min'] * 60
        self.audio_data = []
        self.publish_audio_data = []
        self.start_time = None
        self.publish_start_time = None
        self.audio_info = None
        self.dummy = AudioData()

        if mqtt_obj is not None:
            self.mqtt_obj = mqtt_obj
        
        rospy.Subscriber(self.config['audio_info_topic'], AudioInfo, self.get_audio_info)

        self.audio_feature_pub = []
        for i in range(len(config['audio_features'])):
            audio_feature_pub = rospy.Publisher(config['audio_feature_topics'][i], Float32, queue_size=10)
            self.audio_feature_pub.append(audio_feature_pub)

        print('Audio handler initialized')

    def __del__(self):
        self.stream_audio()
        if self.config['keep_log']:
            self.save_audio()

    def reset(self):
        self.audio_data = []
        self.start_time = None

    def save_audio(self):
        size = 0
        for data in self.publish_audio_data:
            size += len(data)
        print(f'Saving audio of size {size}')
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
        msg_dict = {
            'num_channels': message.num_channels,
            'sample_rate': message.sample_rate,
            'subtype': message.subtype,
        }
        
        self.audio_info = msg_dict

    def callback(self, message):
        if self.start_time is None:
            print('Recording audio...')
            self.start_time = datetime.datetime.now()
            self.publish_start_time = self.start_time

        data = message.data
        # data = message['data']
        self.audio_data.append(data)

        now = datetime.datetime.now()
        if (now - self.start_time).seconds > self.interval_sec:
            self.publish_audio_data = self.audio_data
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
                feature_value = np.sqrt(np.mean(np.square(data)))

            elif audio_feature == 'zcr':
                feature_value = np.mean(
                    np.abs(np.diff(np.sign(data))))
                
            elif audio_feature == 'max_freq':
                feature_value = np.max(np.abs(np.fft.fft(data)))                

            try:
                audio_feature_pub.publish(feature_value)
            except rospy.ROSException:
                pass
