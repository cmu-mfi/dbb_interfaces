import os

import numpy as np
import soundfile as sf


class AudioDownload:
    def __init__(self, config) -> None:
        self.config = config
        print('Audio handler initialized')

    def save_file(self, message, output_path):
        print('Saving audio...')

        if not all(key in message for key in self.config['message_dict_keys']):
            print('Data not saved. Message keys do not match config keys')
            print(f"Message keys: {message.keys()}")
            print(f"Config keys: {self.config.keys()}")
            return

        start_time = message['start_time']
        audio_info = message['audio_info']
        filename_prefix = message['config']['fileprefix']
        fileext = message['config']['fileext']

        filename = f"{filename_prefix}_{start_time}.{fileext}"
        filepath = os.path.join(output_path, filename)

        sound_file = sf.SoundFile(filepath, mode='w',
                                  samplerate=audio_info['sample_rate'],
                                  channels=audio_info['num_channels'],
                                  subtype=None if audio_info['subtype'] == '' else audio_info['subtype'])

        for data in message['audio_data']:
            sound_file.write(np.asarray(data).reshape(
                (-1, audio_info['num_channels'])))
            
        print(f'Audio Saved as {filename} to {output_path}')

