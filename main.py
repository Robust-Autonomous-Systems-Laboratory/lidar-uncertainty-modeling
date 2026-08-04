from src.pointcloud_target_processing.rosbag_loader import ROSPointCloudLoader
from functools import partial
import pyvista as pv
import yaml

def box_cb(box_polydata):
    """Callback function that receives the box geometry on adjustment."""
    global final_box_bounds
    # Retrieve the exact bounding coordinates [xmin, xmax, ymin, ymax, zmin, zmax]
    final_box_bounds = box_polydata.bounds
    
    print("\n--- Current Bounding Box Coordinates ---")
    print(f"X Bounds: {final_box_bounds[0]:.3f} to {final_box_bounds[1]:.3f}")
    print(f"Y Bounds: {final_box_bounds[2]:.3f} to {final_box_bounds[3]:.3f}")
    print(f"Z Bounds: {final_box_bounds[4]:.3f} to {final_box_bounds[5]:.3f}")

    #interactive_box.SetHandleSize(0.005) # keep handles the same size even when resized


def close_plot(state):
    plotter.close()


if __name__ == "__main__":
    bag_file = "/home/quin/data/velodyne_test1.bag"
    topic = "/velodyne_points"  # Adjust to your topic

    loader = ROSPointCloudLoader(bag_file)
    combined_scan = pv.PolyData()

    # get the combined pointcloud of all scans per bag
    combined_scan = loader.read_clouds(topic, True)

    # get test yaml locations and extract and plot only points near target
    with open('cfg/test1_target_locations.yaml','r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)


    global interactive_box
    for target in yaml_data["test1"]:
        print(f"Target name: {target}")
        x = yaml_data["test1"][target]["x_target"]
        y = yaml_data["test1"][target]["y_target"]
        z = yaml_data["test1"][target]["z_target"]
        x_extent = yaml_data["test1"][target]["x_extent"]
        y_extent = yaml_data["test1"][target]["y_extent"]
        z_extent = yaml_data["test1"][target]["z_extent"]

        x_min = x - (x_extent/2)
        x_max = x + (x_extent/2)
        y_min = y - (y_extent/2)
        y_max = y + (y_extent/2)
        z_min = z - (z_extent/2)
        z_max = z + (z_extent/2)

        target_box_bounds = [x_min, x_max, y_min, y_max, z_min, z_max]
        target_subset = combined_scan.clip_box(bounds=target_box_bounds, invert=False)

        # ----- Visualize and plot -----

        # Colorize by intensity if available, or fall back to Z-height
        color_scalar = 'intensity' if 'intensity' in target_subset.point_data else None

        plotter = pv.Plotter()
        plotter.add_mesh(
            target_subset, 
            scalars=color_scalar, 
            cmap="turbo", 
            point_size=2.0, 
            render_points_as_spheres=True
        )

        interactive_box = plotter.add_box_widget(
            callback=box_cb,
            bounds=target_subset.bounds,
            rotation_enabled=False
        ) # pyright: ignore[reportCallIssue]
        interactive_box.SetHandleSize(0.005)

        plotter.add_checkbox_button_widget(
            callback=close_plot,
            value=False,
            position=(10,10),
            size=40,
            color_on='red',
            color_off='grey'
        ) # pyright: ignore[reportCallIssue]

        plotter.show()