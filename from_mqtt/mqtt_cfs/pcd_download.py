import os

import numpy as np
import open3d as o3d


class PCDDownload:
    """
    A class to handle the downloading and saving of point cloud data (PCD) files.
    Attributes:
        config (dict): Configuration dictionary containing necessary keys and settings.
    """
    def __init__(self, config: dict) -> None:
        self.config = config
        print('PCD Handler initialized')

    def save_file(self, message, output_path):
        """
        Saves the point cloud data from the message to a file in the specified output path.
        Args:
            message (dict): Dictionary containing the point cloud data and metadata.
                            Expected keys are defined in the config['message_dict_keys'].            
            output_path (str): The directory path where the file will be saved.
        Returns:
            None
        """
        print('Saving pcd...')

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
        
        points = message['points']
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        o3d.io.write_point_cloud(filepath, pcd)
            
        print(f'Pointcloud Saved as {filename} to {output_path}')

