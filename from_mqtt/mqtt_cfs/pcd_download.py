import os

import numpy as np
import soundfile as sf

import open3d as o3d


class PCDDownload:
    def __init__(self, config) -> None:
        self.config = config
        print('PCD Handler initialized')

    def save_file(self, message, output_path):
        print('Saving pcd...')

        if not all(key in message for key in self.config['message_dict_keys']):
            print('Data not saved. Message keys do not match config keys')
            print(f"Message keys: {message.keys()}")
            print(f"Config keys: {self.config['message_dict_keys']}")
            return

        timestamp = message['timestamp']
        filename_prefix = message['config']['fileprefix']
        fileext = message['config']['fileext']

        filename = f"{filename_prefix}_{timestamp}.{fileext}"
        filepath = os.path.join(output_path, filename)
        
        points = message['points']
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        o3d.io.write_point_cloud(filepath, pcd)
            
        print(f'Pointcloud Saved as {filename} to {output_path}')
