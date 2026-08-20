from src.rosbag_loader import ROSPointCloudLoader
import pyvista as pv

def get_target_in_bounds(bag_path: str, topic_name: str, target_box_bounds: list):

    loader = ROSPointCloudLoader(bag_path)
    combined_scan = pv.PolyData()

    # get the combined pointcloud of all scans per bag
    combined_scan = loader.read_clouds(topic_name, True)
    target_subset = combined_scan.clip_box(bounds=target_box_bounds, invert=False)

    return target_subset