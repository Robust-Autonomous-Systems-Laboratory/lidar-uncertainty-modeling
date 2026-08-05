from collections import defaultdict
from pathlib import Path
from typing import Any, cast
from rosbags.highlevel import AnyReader 
import numpy as np
import pyvista as pv

class ROSPointCloudLoader:
    """
    Parses ROS 1 PointCloud2 messages from a rosbag into a single, 
    concatenated PyVista PolyData object without requiring ROS.
    """
    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)

    def read_clouds(self, topic_name: str, include_timestamp: bool = True) -> pv.PolyData:
        """
        Reads all PointCloud2 messages for a topic and returns a single merged PolyData.
        
        :param topic_name: The ROS topic to extract.
        :param include_timestamp: Whether to attach point-wise timestamps as scalar data.
        :return: A single merged pv.PolyData mesh.
        """
        field_types = {
            1: np.int8,   2: np.uint8,  3: np.int16, 4: np.uint16, 
            5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64
        }

        points_list = []
        scalars_dict = defaultdict(list)

        with AnyReader([self.bag_path]) as reader:
            connections = [x for x in reader.connections if x.topic == topic_name]
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = cast(Any, reader.deserialize(rawdata, connection.msgtype))

                # Construct structured dtype accounting for field offsets and point stride
                names = [f.name for f in msg.fields]
                formats = [field_types[f.datatype] for f in msg.fields]
                offsets = [f.offset for f in msg.fields]

                dtype = np.dtype({
                    'names': names,
                    'formats': formats,
                    'offsets': offsets,
                    'itemsize': msg.point_step
                })

                # process buffer to numpy format
                cloud_data = np.frombuffer(msg.data, dtype=dtype)

                # Extract XYZ spatial coordinates (using float32 for speed and reduced RAM usage)
                xyz = np.column_stack((cloud_data['x'], cloud_data['y'], cloud_data['z'])).astype(np.float32, copy=False)

                # Filter out invalid NaN points
                valid_mask = ~np.isnan(xyz).any(axis=1)
                xyz_clean = xyz[valid_mask]

                if len(xyz_clean) == 0:
                    continue

                # Store clean coordinate array
                points_list.append(xyz_clean)

                # Store remaining scalar fields (intensity, ring, rgb, etc.)
                for name in names:
                    if name not in ('x', 'y', 'z'):
                        scalars_dict[name].append(cloud_data[name][valid_mask])

                # Record frame timestamp per point
                if include_timestamp:
                    time_sec = timestamp / 1e9
                    scalars_dict['timestamp'].append(np.full(len(xyz_clean), time_sec, dtype=np.float64))

        if not points_list:
            print(f"No valid point cloud data found on topic: '{topic_name}'")
            return pv.PolyData()

        # concatenate 
        merged_xyz = np.vstack(points_list)

        # make a single polydata object with the merged numpy representation
        merged_cloud = pv.PolyData(merged_xyz)

        # attach other data fields besides x,y,z
        for field_name, scalar_list in scalars_dict.items():
            merged_cloud.point_data[field_name] = np.concatenate(scalar_list)

        return merged_cloud

    def get_topics(self):
        with AnyReader([self.bag_path]) as reader:
            # get all topic names and msg types
            topic_types = {conn.topic: conn.msgtype for conn in reader.connections}

            print(f"topic_types Python Type: {type(topic_types)}")

            for topic, msg_type in sorted(topic_types.items()):
                print(f"Topic: {topic} | Message Type: {msg_type}")