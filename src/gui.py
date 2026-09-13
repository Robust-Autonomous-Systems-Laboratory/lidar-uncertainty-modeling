import os
import yaml
import pyvista as pv
from PyQt5 import QtWidgets, QtCore
from pyvistaqt import QtInteractor
from pathlib import Path

from .target_processing import get_target_in_bounds, fit_plane_irls
from .rosbag_loader import ROSPointCloudLoader
from .export_results import save_data

class LidarUncertaintyGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.experiment_file = ""
        self.bag_file = ""
        self.topic_name = ""
        self.processed_cloud = None

        self.setWindowTitle("Lidar Uncertainty Experiment Processor")
        self.resize(1000, 700)

        # Central widget and main layout
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # Left side: PyVista 3D Plotting Window
        self.plotter = QtInteractor(self)
        main_layout.addWidget(self.plotter.interactor, stretch=4)

        # Right side: Control panel
        control_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_layout, stretch=1)

        label = QtWidgets.QLabel("<b>Control Panel</b>")
        control_layout.addWidget(label)

        # Buttons
        self.btn_load_experiment = QtWidgets.QPushButton("Load Experiment")
        self.btn_load_experiment.clicked.connect(self.load_experiment_file)
        control_layout.addWidget(self.btn_load_experiment)

        self.btn_load_bag = QtWidgets.QPushButton("Load Bag")
        self.btn_load_bag.clicked.connect(self.load_bag_file)
        control_layout.addWidget(self.btn_load_bag)

        self.btn_select_topic = QtWidgets.QPushButton("Select Topic")
        self.btn_select_topic.clicked.connect(self.select_topic)
        control_layout.addWidget(self.btn_select_topic)

        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_start.clicked.connect(self.process_targets)
        control_layout.addWidget(self.btn_start)

        self.btn_next_target = QtWidgets.QPushButton("Next Target")
        self.btn_next_target.clicked.connect(self.next_target)
        control_layout.addWidget(self.btn_next_target)

        control_layout.addStretch()

    def closeEvent(self, event):
        """Ensure PyVista/VTK interactor releases resources on app close."""
        if hasattr(self, "plotter") and self.plotter is not None:
            self.plotter.clear()
            self.plotter.close()
        event.accept()

    def load_experiment_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select Experiment YAML",
            "./cfg/",
            "YAML Files (*.yaml);;All Files (*)",
            options=options,
        )
        self.experiment_file = fileName
        if fileName:
            print(f"Loaded experiment file: {fileName}")

    def load_bag_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select ROS Bag",
            "./",
            "Bag Files (*.bag);;All Files (*)",
            options=options,
        )
        self.bag_file = fileName
        if fileName:
            print(f"Loaded bag file: {fileName}")

    def select_topic(self):
        pc_loader = ROSPointCloudLoader(self.bag_file)
        pc2_topics = pc_loader.get_pc2_topics()

        item, ok = QtWidgets.QInputDialog.getItem(
            self,
            "Select Topic",
            "Choose a PointCloud2 topic from the bag:",
            pc2_topics,
            0,
            False,
        )
        if ok and item:
            print(f"User selected: {item}")
            self.topic_name = str(item)

    def process_targets(self):
        with open(self.experiment_file, "r") as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            self.key_name, self.targets = next(iter(yaml_data.items()))
            self.num_targets = len(self.targets)

        self.current_target_index = 0
        self.plot_current_target()

    def next_target(self):
        if self.processed_cloud is None:
            return

        exp_stem = Path(self.experiment_file).stem
        topic = str(self.topic_name).strip("/\\").replace("/", "-")
        target = str(self.target).strip("/\\")

        target_dir = Path.cwd() / "results" / exp_stem / topic / target

        # Save the filtered planar target inliers directly
        save_data(
            pointcloud=self.processed_cloud,
            output_dir=target_dir,
            name="processed_planar_target_points",
        )

        self.current_target_index += 1
        self.plot_current_target()

    def plot_current_target(self):
        if not (self.current_target_index < self.num_targets):
            print("All targets processed")
            self.plotter.clear()
            self.plotter.add_text(
                "All targets processed!", position="lower_edge", font_size=12
            )
            return

        self.plotter.clear()

        self.target = list(self.targets)[self.current_target_index]
        print(
            f"Processing target [{self.current_target_index + 1} / {self.num_targets}]"
        )

        x = self.targets[self.target]["x_target"]
        y = self.targets[self.target]["y_target"]
        z = self.targets[self.target]["z_target"]
        x_extent = self.targets[self.target]["x_extent"]
        y_extent = self.targets[self.target]["y_extent"]
        z_extent = self.targets[self.target]["z_extent"]

        x_min, x_max = x - (x_extent / 2), x + (x_extent / 2)
        y_min, y_max = y - (y_extent / 2), y + (y_extent / 2)
        z_min, z_max = z - (z_extent / 2), z + (z_extent / 2)

        # 1. Fetch raw points inside initial YAML bounding region
        raw_target_subset = get_target_in_bounds(
            self.bag_file,
            self.topic_name,
            [x_min, x_max, y_min, y_max, z_min, z_max],
        )

        # 2. Automatically clean noisy outliers using IRLS plane fitting
        self.processed_cloud, plane_params = fit_plane_irls(
            cloud=raw_target_subset,
            loss_function="tukey",
            inlier_threshold=0.02,
        )

        # 3. Render strictly the cleaned planar inliers with residual distance scalar visualization
        color_scalar = (
            "residuals"
            if "residuals" in self.processed_cloud.point_data
            else None
        )

        self.plotter.add_mesh(
            self.processed_cloud,
            scalars=color_scalar,
            cmap="viridis",
            point_size=3.0,
            render_points_as_spheres=True,
        )
        self.plotter.reset_camera()