from .target_processing import get_target_in_bounds
from .rosbag_loader import ROSPointCloudLoader
from .export_results import save_data
from PyQt5 import QtWidgets, QtCore
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
        self.interactive_box = None
        
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

        # Dimension Controller
        self.setup_dimension_controls(control_layout)
        
        # Spacer to push buttons to the top
        control_layout.addStretch()

    def load_experiment_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "QFileDialog.getOpenFileName()", "./cfg/", "YAML Files (*.yaml);;All Files (*)", options=options
        )
        self.experiment_file = fileName
        if fileName:
            print(fileName)

    def load_bag_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "QFileDialog.getOpenFileName()", "./", "Bag Files (*.bag);;All Files (*)", options=options
        )
        self.bag_file = fileName
        if fileName:
            print(fileName)

    def select_topic(self):
        pc_loader = ROSPointCloudLoader(self.bag_file)
        pc2_topics = pc_loader.get_pc2_topics()

        item, ok = QtWidgets.QInputDialog.getItem(
            self, "Select Topic", "Choose a PointCloud2 topic from the bag:", pc2_topics, 0, False
        )
        if ok and item:
            print(f"User selected: {item}")
        self.topic_name = str(item)

    def setup_dimension_controls(self, control_layout):
        """Creates SpinBox and Slider controls for X, Y, Z box extents."""
        self.dim_controls = {}
        
        dim_group = QtWidgets.QGroupBox("Box Extents (m)")
        grid = QtWidgets.QGridLayout(dim_group)
        
        axes = ['X', 'Y', 'Z']
        for idx, axis in enumerate(axes):
            lbl = QtWidgets.QLabel(f"{axis}:")
            
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(0.1, 100.0)
            spin.setSingleStep(0.1)
            spin.setDecimals(2)
            
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(1, 1000)
            
            spin.valueChanged.connect(lambda val, s=slider: s.setValue(int(val * 10)))
            slider.valueChanged.connect(lambda val, sp=spin: sp.setValue(val / 10.0))
            spin.valueChanged.connect(self.on_gui_bounds_changed)
            
            grid.addWidget(lbl, idx, 0)
            grid.addWidget(spin, idx, 1)
            grid.addWidget(slider, idx, 2)
            
            self.dim_controls[axis.lower()] = {'spin': spin, 'slider': slider}
            
        control_layout.addWidget(dim_group)

    def box_cb(self, box):
        """Callback triggered when moving/scaling the 3D box widget in the viewport."""
        self.current_bounds = box.bounds  # (x_min, x_max, y_min, y_max, z_min, z_max)
        
        dx = self.current_bounds[1] - self.current_bounds[0]
        dy = self.current_bounds[3] - self.current_bounds[2]
        dz = self.current_bounds[5] - self.current_bounds[4]

        # Block signals briefly to prevent recursive callback loops
        for key, val in zip(['x', 'y', 'z'], [dx, dy, dz]):
            if key in self.dim_controls:
                spin = self.dim_controls[key]['spin']
                spin.blockSignals(True)
                spin.setValue(val)
                self.dim_controls[key]['slider'].setValue(int(val * 10))
                spin.blockSignals(False)

    def on_gui_bounds_changed(self):
        """Callback triggered when numeric UI controls are changed by the user."""
        if not hasattr(self, 'interactive_box') or self.interactive_box is None:
            return
            
        if not hasattr(self, 'current_bounds'):
            return

        # Calculate current center
        cx = (self.current_bounds[0] + self.current_bounds[1]) / 2.0
        cy = (self.current_bounds[2] + self.current_bounds[3]) / 2.0
        cz = (self.current_bounds[4] + self.current_bounds[5]) / 2.0

        # Read dimensions from UI
        dx = self.dim_controls['x']['spin'].value()
        dy = self.dim_controls['y']['spin'].value()
        dz = self.dim_controls['z']['spin'].value()

        new_bounds = [
            cx - dx / 2.0, cx + dx / 2.0,
            cy - dy / 2.0, cy + dy / 2.0,
            cz - dz / 2.0, cz + dz / 2.0
        ]
        
        self.current_bounds = new_bounds

        # Place the widget directly on the vtkBoxWidget object
        self.interactive_box.PlaceWidget(new_bounds)
        self.plotter.render()

    def process_targets(self):
        with open(self.experiment_file, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            self.key_name, self.targets = next(iter(yaml_data.items()))
            self.num_targets = len(self.targets)

        self.current_target_index = 0
        self.plot_current_target()

    def next_target(self):
        self.current_target_index += 1
        self.plotter.clear()

        control_vol_bbox = self.current_bounds
        control_vol_points = self.target_subset.clip_box(control_vol_bbox, invert=False)

        assert isinstance(control_vol_points, pv.DataSet)

        exp_stem = Path(self.experiment_file).stem
        topic = str(self.topic_name).strip("/\\")
        target = str(self.target).strip("/\\")

        target_dir = Path.cwd() / "results" / exp_stem / topic / target

        save_data(
            pointcloud=control_vol_points,
            output_dir=target_dir,
            name="control_volume_points"
        )

        self.plot_current_target()

    def plot_current_target(self):
        if not (self.current_target_index < self.num_targets):
            print("All targets processed")
            self.plotter.clear()
            self.plotter.add_text("All targets processed!", position='lower_edge', font_size=12)
            return

        self.plotter.clear()

        self.target = list(self.targets)[self.current_target_index]
        print(f"Processing target [{self.current_target_index + 1} / {self.num_targets}]")

        x = self.targets[self.target]["x_target"]
        y = self.targets[self.target]["y_target"]
        z = self.targets[self.target]["z_target"]
        x_extent = self.targets[self.target]["x_extent"]
        y_extent = self.targets[self.target]["y_extent"]
        z_extent = self.targets[self.target]["z_extent"]

        x_min, x_max = x - (x_extent / 2), x + (x_extent / 2)
        y_min, y_max = y - (y_extent / 2), y + (y_extent / 2)
        z_min, z_max = z - (z_extent / 2), z + (z_extent / 2)

        self.target_subset = get_target_in_bounds(self.bag_file, self.topic_name, [x_min, x_max, y_min, y_max, z_min, z_max])
        self.current_bounds = list(self.target_subset.bounds)

        color_scalar = 'intensity' if 'intensity' in self.target_subset.point_data else None

        self.plotter.add_mesh(
            self.target_subset,
            scalars=color_scalar,
            cmap='turbo',
            point_size=2.0,
            render_points_as_spheres=True
        )

        self.interactive_box = self.plotter.add_box_widget(
            callback=self.box_cb,
            bounds=self.target_subset.bounds,
            rotation_enabled=False
        )
        self.interactive_box.SetHandleSize(0.005)
        self.interactive_box.GetHandleProperty().SetColor(1.0, 0.0, 0.0)

        # Update initial spinbox values to match target dimensions
        for key, val in zip(['x', 'y', 'z'], [x_extent, y_extent, z_extent]):
            if key in self.dim_controls:
                spin = self.dim_controls[key]['spin']
                spin.blockSignals(True)
                spin.setValue(val)
                self.dim_controls[key]['slider'].setValue(int(val * 10))
                spin.blockSignals(False)