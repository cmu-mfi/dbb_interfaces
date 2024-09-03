import os

import cv2
import numpy as np


class ImageDownload:
    def __init__(self, config) -> None:
        self.config = config
        print('Image Handler initialized')

    def save_file(self, message, output_path):
        print('Saving image...')

        if not all(key in message for key in self.config['message_dict_keys']):
            print('Data not saved. Message keys do not match config keys')
            print(f"Message keys: {message.keys()}")
            print(f"Config keys: {self.config['message_dict_keys']}")
            return

        timestamp = message['timestamp']
        filename_prefix = message['config']['fileprefix']
        filename_prefix = filename_prefix.replace('_', '-')
        fileext = message['config']['fileext']

        try:
            experiment_class = message['config']['experiment_class']
        except KeyError:
            # generate random 4 characters alphanumeric string for experiment class
            experiment_class = ''.join(np.random.choice(list('0123456789abcdefghijklmnopqrstuvwxyz'), 4))

        if '_' in experiment_class:
            experiment_class = experiment_class.replace('_', '-')
        
        filename = f"{filename_prefix}_{experiment_class}_{timestamp}.{fileext}"
        filepath = os.path.join(output_path, filename)
        
        image = message['data']        
        cv2.imwrite(filepath, image)
            
        print(f'Image saved as {filename} to {output_path}')

