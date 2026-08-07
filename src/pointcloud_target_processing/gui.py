from .target_processing import get_target_in_bounds
from .rosbag_loader import ROSPointCloudLoader
from PyQt5 import QtWidgets
from pyvistaqt import QtInteractor
import pyvista as pv
import yaml

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
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;Python Files (*.py)", options=options)
        self.experiment_file = fileName
        if fileName:
            print(fileName)
        

    def load_bag_file(self):
            options = QtWidgets.QFileDialog.Options()
            options |= QtWidgets.QFileDialog.DontUseNativeDialog
            fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;Python Files (*.py)", options=options)
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
        bounds = box.bounds  # (x_min, x_max, y_min, y_max, z_min, z_max)
        dims = (bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])
        print(f'New Bounds: {bounds}')
        print(f'Dimensions (X, Y, Z): {dims}')

    def next_target(self):
        print("Next Target button pressed")
        self.current_target_index += 1
        self.plotter.clear()
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
        print("plot_current_target function entered")

        # check if all targets have been processed,
        # plot and increment counter if not

        if not (self.current_target_index < self.num_targets):
            # all targets processed
            print("All targets processed")
            self.plotter.clear()
            self.plotter.add_text("All targets processed!", position='upper_edge', font_size=12)
            return

        # clear last mesh
        self.plotter.clear()

        target = list(self.targets)[self.current_target_index]
        print(f"Processing target [{self.current_target_index + 1} / {self.num_targets}]")

        x = self.targets[target]["x_target"]
        y = self.targets[target]["y_target"]
        z = self.targets[target]["z_target"]
        x_extent = self.targets[target]["x_extent"]
        y_extent = self.targets[target]["y_extent"]
        z_extent = self.targets[target]["z_extent"]

        x_min, x_max = x - (x_extent/2), x + (x_extent/2)
        y_min, y_max = y - (y_extent/2), y + (y_extent/2)
        z_min, z_max = z - (z_extent/2), z + (z_extent/2)

        target_subset = get_target_in_bounds(self.bag_file, self.topic_name, [x_min, x_max, y_min, y_max, z_min, z_max])

        # Colorize by intensity if available, or fall back to Z-height
        color_scalar = 'intensity' if 'intensity' in target_subset.point_data else None

        self.plotter.add_mesh(
            target_subset,
            scalars=color_scalar,
            cmap='turbo',
            point_size=2.0,
            render_points_as_spheres=True
        )

        interactive_box = self.plotter.add_box_widget(
            callback=self.box_cb,
            bounds=target_subset.bounds,
            rotation_enabled=False
        ) # pyright: ignore[reportCallIssue]
        interactive_box.SetHandleSize(0.005)