import os

import numpy as np
import soundfile as sf


class AudioDownload:
    """
    A class to handle the downloading and saving of audio files based on provided configuration and message data.
    Attributes:
        config (dict): Configuration dictionary containing necessary keys and settings.
    Methods:
        __init__(config):
            Initializes the AudioDownload instance with the provided configuration.
        save_file(message, output_path):
            Saves the audio file to the specified output path based on the message data and configuration.
    """

    def __init__(self, config: dict) -> None:     
        self.config = config
        print('Audio handler initialized')

    def save_file(self, message, output_path):
        """
        Saves the audio file to the specified output path based on the message data and configuration.
        Args:
            message (dict): Dictionary containing the audio data and metadata.
                            Expected keys are defined in the config['message_dict_keys'].
            output_path (str): The directory path where the audio file will be saved.
        Returns:
            None
        """        
        print('Saving audio...')

        if not all(key in message for key in self.config['message_dict_keys']):
            print('Data not saved. Message keys do not match config keys')
            print(f"Message keys: {message.keys()}")
            print(f"Config keys: {self.config['message_dict_keys']}")
            return

        start_time = message['start_time']
        audio_info = message['audio_info']
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
        
        filename = f"{filename_prefix}_{experiment_class}_{start_time}.{fileext}"
        filepath = os.path.join(output_path, filename)

        sound_file = sf.SoundFile(filepath, mode='w',
                                  samplerate=audio_info['sample_rate'],
                                  channels=audio_info['num_channels'],
                                  subtype=None if audio_info['subtype'] == '' else audio_info['subtype'])

        for data in message['audio_data']:
            sound_file.write(np.asarray(data).reshape(
                (-1, audio_info['num_channels'])))
            
        print(f'Audio Saved as {filename} to {output_path}')

