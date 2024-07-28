import base64
import datetime
import math
import os
import pickle
import struct
import sys
import threading
from ctypes import POINTER, c_float, c_uint32, cast, pointer

import numpy as np
import open3d as o3d
from mqtt_publisher import MQTTPublisher


class ImageHandler:
    def __init__(self, config, ros_client, mqtt_obj: MQTTPublisher = None) -> None:
        self.config = config
        self.topic = config['name']
        self.ros_client = ros_client

        if mqtt_obj is not None:
            self.mqtt_obj = mqtt_obj

        print('PCD handler initialized')

    def save_image(self, pcd: o3d.geometry.PointCloud, timestamp: datetime.datetime):
        print('Saving pcd...')
        time_format = '%Y-%m-%d_%H-%M-%S'
        start_time = timestamp.strftime(time_format)
        filepath = f"{self.config['fileprefix']}_{start_time}.{self.config['fileext']}"

        dir = 'logs'
        os.makedirs(dir, exist_ok=True)
        filepath = os.path.join(dir, filepath)

        o3d.io.write_point_cloud(filepath, pcd)

    def stream_image(self, pcd: o3d.geometry.PointCloud, timestamp: datetime.datetime):
        pcd_dict = {
            'config': self.config,
            'timestamp': timestamp.strftime('%Y-%m-%d_%H-%M-%S'),
            'points': np.asarray(pcd.points),
        }

        size = len(pcd.points)
        print(f'Streaming pcd of size {size}')

        pcd_pickle = pickle.dumps(pcd_dict)
        self.mqtt_obj.publish(self.config['mqtt_topic'], pcd_pickle)

    def callback(self, message):
        print('PCD callback')
        pcd_data = message
        timestamp = datetime.datetime.now()

        def convert_rgbUint32_to_tuple(rgb_uint32): return (
            (rgb_uint32 & 0x00FF0000) >> 16,
            (rgb_uint32 & 0x0000FF00) >> 8,
            (rgb_uint32 & 0x000000FF),
        )

        def convert_rgbFloat_to_tuple(rgb_float): return convert_rgbUint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
        )

        # Get cloud data from ros_cloud
        field_names = [field["name"] for field in pcd_data["fields"]]
        cloud_data = list(self.read_points(
            pcd_data, skip_nans=True, field_names=field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()

        if len(cloud_data) == 0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

            # Get xyz
            xyz = [
                (x, y, z) for x, y, z, rgb in cloud_data
            ]  # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if (
                type(cloud_data[0][IDX_RGB_IN_FIELD]) == float
            ):  # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb)
                       for x, y, z, rgb in cloud_data]
            else:
                rgb = [convert_rgbUint32_to_tuple(
                    rgb) for x, y, z, rgb in cloud_data]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(
                np.array(xyz) * 1000)
            open3d_cloud.colors = o3d.utility.Vector3dVector(
                np.array(rgb) / 255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
        
        # save and stream pcd
        save_pcd_thread = threading.Thread(
            target=self.save_pcd, args=(open3d_cloud, timestamp,))
        stream_pcd_thread = threading.Thread(
            target=self.stream_pcd, args=(open3d_cloud, timestamp,))
        save_pcd_thread.start()
        stream_pcd_thread.start()

    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """
        Read points from a {sensor_msgs.PointCloud2} message.
        Implementation based on code from:
        https://github.com/ros/common_msgs/blob/20a833b56f9d7fd39655b8491a2ec1226d2639b3/sensor_msgs/src/sensor_msgs/point_cloud2.py#L61

        @param cloud: The point cloud to read from.
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        _DATATYPES = {}

        _DATATYPES[1] = ('b', 1)  # _DATATYPES[PointField.INT8]    = ('b', 1)
        _DATATYPES[2] = ('B', 1)  # _DATATYPES[PointField.UINT8]   = ('B', 1)
        _DATATYPES[3] = ('h', 2)  # _DATATYPES[PointField.INT16]   = ('h', 2)
        _DATATYPES[4] = ('H', 2)  # _DATATYPES[PointField.UINT16]  = ('H', 2)
        _DATATYPES[5] = ('i', 4)  # _DATATYPES[PointField.INT32]   = ('i', 4)
        _DATATYPES[6] = ('I', 4)  # _DATATYPES[PointField.UINT32]  = ('I', 4)
        _DATATYPES[7] = ('f', 4)  # _DATATYPES[PointField.FLOAT32] = ('f', 4)
        _DATATYPES[8] = ('d', 8)  # _DATATYPES[PointField.FLOAT64] = ('d', 8)

        def get_struct_fmt(is_bigendian, fields, field_names=None):
            fmt = ">" if is_bigendian else "<"

            offset = 0
            for field in (
                f
                for f in sorted(fields, key=lambda f: f["offset"])
                if field_names is None or f["name"] in field_names
            ):
                if offset < field["offset"]:
                    fmt += "x" * (field["offset"] - offset)
                    offset = field["offset"]
                if field["datatype"] not in _DATATYPES:
                    print(
                        "Skipping unknown PointField datatype [%d]" % field["datatype"],
                        file=sys.stderr,
                    )
                else:
                    datatype_fmt, datatype_length = _DATATYPES[field["datatype"]]
                    fmt += field["count"] * datatype_fmt
                    offset += field["count"] * datatype_length

            return fmt

        fmt = get_struct_fmt(cloud["is_bigendian"],
                             cloud["fields"], field_names)
        width, height, point_step, row_step, data, isnan = (
            cloud["width"],
            cloud["height"],
            cloud["point_step"],
            cloud["row_step"],
            cloud["data"],
            math.isnan,
        )
        data = base64.b64decode(data)
        data = np.frombuffer(data, dtype=np.uint8)
        unpack_from = struct.Struct(fmt).unpack_from

        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p
                        offset += point_step
        else:
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in range(height):
                    offset = row_step * v
                    for u in range(width):
                        yield unpack_from(data, offset)
                        offset += point_step
