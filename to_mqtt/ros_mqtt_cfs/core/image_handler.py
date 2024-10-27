import base64
import datetime
import math
import os
import pickle
import struct
import sys
import threading
from ctypes import POINTER, c_float, c_uint32, cast, pointer

import cv2
import numpy as np
import open3d as o3d
from core.mqtt_publisher import MQTTPublisher
from sensor_msgs.msg import Image


class ImageHandler:
    def __init__(self, config, mqtt_obj: MQTTPublisher = None) -> None:
        self.config = config
        self.topic = config['name']
        self.dummy = Image()
        self.last_capture = None
        
        if 'time_interval' in self.config and \
            self.config['time_interval'] > 0:
            self.time_interval = self.config['time_interval']
        else:
            self.time_interval = 0

        if mqtt_obj is not None:
            self.mqtt_obj = mqtt_obj

        print('Image handler initialized')

    def save_image(self, image: np.ndarray, timestamp: datetime.datetime):
        print('Saving pcd...')
        time_format = '%Y-%m-%d_%H-%M-%S'
        start_time = timestamp.strftime(time_format)
        filepath = f"{self.config['fileprefix']}_{start_time}.{self.config['fileext']}"

        dir = 'logs'
        os.makedirs(dir, exist_ok=True)
        filepath = os.path.join(dir, filepath)

        cv2.imwrite(filepath, image)

    def stream_image(self, image: np.ndarray, timestamp: datetime.datetime):
        pcd_dict = {
            'config': self.config,
            'timestamp': timestamp.strftime('%Y-%m-%d_%H-%M-%S'),
            'data': image,
        }

        print(f'Streaming image of shape {image.shape}')

        image_pickle = pickle.dumps(pcd_dict)
        self.mqtt_obj.publish(self.config['mqtt_topic'], image_pickle)

    def callback(self, message):
        img_msg = message
        timestamp = datetime.datetime.now()
        
        if self.last_capture is not None:
            time_diff = timestamp - self.last_capture
            if time_diff.total_seconds() < self.time_interval:
                return
        self.last_capture = timestamp   
        print('Image callback')

        def encoding_to_dtype_with_channels(encoding):
            if encoding == 'mono8':
                return np.uint8, 1
            elif encoding == 'bgr8':
                return np.uint8, 3
            elif encoding == 'rgb8':
                return np.uint8, 3
            elif encoding == 'mono16':
                return np.uint16, 1
            elif encoding == 'rgba8':
                return np.uint8, 4
            elif encoding == 'bgra8':
                return np.uint8, 4
            elif encoding == '32FC1':
                return np.float32, 1
            elif encoding == '32FC2':
                return np.float32, 2
            elif encoding == '32FC3':
                return np.float32, 3
            elif encoding == '32FC4':
                return np.float32, 4
            else:
                raise TypeError(f'Unsupported encoding: {encoding}')
            
        dtype, n_channels = encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = np.dtype(dtype)
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        img_buf = np.asarray(img_msg.data, dtype=dtype) if isinstance(img_msg.data, list) else img_msg.data

        if n_channels == 1:
            im = np.ndarray(shape=(img_msg.height, int(img_msg.step/dtype.itemsize)),
                            dtype=dtype, buffer=img_buf)
            im = np.ascontiguousarray(im[:img_msg.height, :img_msg.width])
        else:
            im = np.ndarray(shape=(img_msg.height, int(img_msg.step/dtype.itemsize/n_channels), n_channels),
                            dtype=dtype, buffer=img_buf)
            im = np.ascontiguousarray(im[:img_msg.height, :img_msg.width, :])

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            im = im.byteswap().newbyteorder()

        image = np.array(im[:,:,0:3])
   
        # save and stream image
        save_image_thread = threading.Thread(
            target=self.save_image, args=(image, timestamp,))
        stream_image_thread = threading.Thread(
            target=self.stream_image, args=(image, timestamp,))
        
        if self.config['keep_log']:
            save_image_thread.start()
        stream_image_thread.start()