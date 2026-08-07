from .target_processing import get_target_in_bounds
from .rosbag_loader import ROSPointCloudLoader
from .export_results import save_data
from PyQt5 import QtWidgets
from pyvistaqt import QtInteractor
from pathlib import Path
import pyvista as pv
import yaml
import os

class LidarUncertaintyGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.experiment_file = ''
        self.bag_file = ''
        self.topic_name = ''
        
        self.setWindowTitle("Lidar Uncertainty Experiment Processor")
        self.resize(1000, 700)
        
        # Central widget and main layout (Side-by-side)
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)
        
        # Left side: PyVista 3D Plotting Window (QtInteractor)
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor, stretch=4)
        
        # Right side: Control panel for buttons
        control_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_layout, stretch=1)
        
        # Title label for buttons
        label = QtWidgets.QLabel("<b>Control Panel</b>")
        control_layout.addWidget(label)
        
        # Button 1: Load Experiment File
        self.btn_load_experiment = QtWidgets.QPushButton("Load Experiment")
        self.btn_load_experiment.clicked.connect(self.load_experiment_file)
        control_layout.addWidget(self.btn_load_experiment)
        
        # Button 2: Load Bag
        self.btn_load_bag = QtWidgets.QPushButton("Load Bag")
        self.btn_load_bag.clicked.connect(self.load_bag_file)
        control_layout.addWidget(self.btn_load_bag)
        
        # Button 3: Select Topic
        self.btn_select_topic = QtWidgets.QPushButton("Select Topic")
        self.btn_select_topic.clicked.connect(self.select_topic)
        control_layout.addWidget(self.btn_select_topic)

        # Button 4: Start Target Extraction
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_start.clicked.connect(self.process_targets)
        control_layout.addWidget(self.btn_start)

        # Button 5: Next Target
        self.btn_next_target = QtWidgets.QPushButton("Next Target")
        self.btn_next_target.clicked.connect(self.next_target)
        control_layout.addWidget(self.btn_next_target)
        
        # Spacer to push buttons to the top
        control_layout.addStretch()
        

    def load_experiment_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "./cfg/","YAML Files (*.yaml);;All Files (*)", options=options)
        self.experiment_file = fileName
        if fileName:
            print(fileName)
        

    def load_bag_file(self):
            options = QtWidgets.QFileDialog.Options()
            options |= QtWidgets.QFileDialog.DontUseNativeDialog
            fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "./","Bag Files (*.bag);;All Files (*)", options=options)
            self.bag_file = fileName
            if fileName:
                print(fileName)
            

    def select_topic(self):
        # select topic from list of PC2 messages in selected bag
        pc_loader = ROSPointCloudLoader(self.bag_file)
        pc2_topics = pc_loader.get_pc2_topics()

        item, ok = QtWidgets.QInputDialog.getItem(
            self, "Select Topic", "Choose a PointCloud2 topic from the bag:", pc2_topics, 0, False
        )
        if ok and item:
            print(f"User selected: {item}")
        self.topic_name = str(item)

    def box_cb(self, box):
        # box is a pyvista.PolyData representing the current state of the widget
        self.current_bounds = box.bounds  # (x_min, x_max, y_min, y_max, z_min, z_max)
        self.current_dims = (self.current_bounds[1] - self.current_bounds[0], self.current_bounds[3] - self.current_bounds[2], self.current_bounds[5] - self.current_bounds[4]) # (X,Y,Z) dimensions
        # print(f'New Bounds: {bounds}')
        # print(f'Dimensions (X, Y, Z): {dims}')

    def next_target(self):
        self.current_target_index += 1
        self.plotter.clear()

        # Get extracted points
        control_vol_bbox = self.current_bounds
        control_vol_points = self.target_subset.clip_box(control_vol_bbox, invert=False)

        # Tell Pylance this is guaranteed to be a DataSet
        assert isinstance(control_vol_points, pv.DataSet)

        # Clean strings to prevent path root issues
        exp_stem = Path(self.experiment_file).stem
        topic = str(self.topic_name).strip("/\\")
        target = str(self.target).strip("/\\")

        # Define destination directory
        target_dir = Path.cwd() / "results" / exp_stem / topic / target

        # Delegate saving logic
        save_data(
            pointcloud=control_vol_points,
            output_dir=target_dir,
            name="control_volume_points"
        )

        # onto the next target!
        self.plot_current_target()

    def process_targets(self):
        # get test yaml locations and extract and plot only points near target
        with open(self.experiment_file,'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            self.key_name, self.targets = next(iter(yaml_data.items()))
            self.num_targets = len(self.targets)

        self.current_target_index = 0
        self.plot_current_target()


    def plot_current_target(self):
        # check if all targets have been processed,
        # plot and increment counter if not

        if not (self.current_target_index < self.num_targets):
            # all targets processed
            print("All targets processed")
            self.plotter.clear()
            self.plotter.add_text("All targets processed!", position='lower_edge', font_size=12)
            return

        # clear last mesh
        self.plotter.clear()

        self.target = list(self.targets)[self.current_target_index]
        print(f"Processing target [{self.current_target_index + 1} / {self.num_targets}]")

        x = self.targets[self.target]["x_target"]
        y = self.targets[self.target]["y_target"]
        z = self.targets[self.target]["z_target"]
        x_extent = self.targets[self.target]["x_extent"]
        y_extent = self.targets[self.target]["y_extent"]
        z_extent = self.targets[self.target]["z_extent"]

        x_min, x_max = x - (x_extent/2), x + (x_extent/2)
        y_min, y_max = y - (y_extent/2), y + (y_extent/2)
        z_min, z_max = z - (z_extent/2), z + (z_extent/2)

        self.target_subset = get_target_in_bounds(self.bag_file, self.topic_name, [x_min, x_max, y_min, y_max, z_min, z_max])

        # Colorize by intensity if available, or fall back to Z-height
        color_scalar = 'intensity' if 'intensity' in self.target_subset.point_data else None

        self.plotter.add_mesh(
            self.target_subset,
            scalars=color_scalar,
            cmap='turbo',
            point_size=2.0,
            render_points_as_spheres=True
        )

        interactive_box = self.plotter.add_box_widget(
            callback=self.box_cb,
            bounds=self.target_subset.bounds,
            rotation_enabled=False
        ) # pyright: ignore[reportCallIssue]
        interactive_box.SetHandleSize(0.005)
        interactive_box.GetHandleProperty().SetColor(1.0, 0.0, 0.0) # R,G,B [0,1]
        