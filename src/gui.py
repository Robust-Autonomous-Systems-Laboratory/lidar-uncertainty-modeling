import os
import datetime
import yaml
import pandas as pd
import pyvista as pv
from PyQt5 import QtWidgets, QtCore
from pyvistaqt import QtInteractor
from pathlib import Path

from .target_processing import fit_plane_irls
from .rosbag_loader import ROSPointCloudLoader
from .export_results import save_data
from .repeatability_processor import LiDARRepeatabilityAnalyzer
from .bag_discovery import discover_bags

# Fewer points than this inside a target's search box is treated as "target not seen"
MIN_TARGET_POINTS = 10

# Lidars plotted in the report's per-target charts (legend order). Tables always include
# every lidar. Set to None to chart every lidar.
REPORT_CHART_LIDARS = ["ouster_2048x10", "vlp32_strongest", "ruby", "iris"]


class LidarUncertaintyGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.experiment_file = ""
        self.bag_dir = ""
        self.bag_groups = {}      # {lidar_name: [TrialBag, ...]}, trials ascending
        self.lidar_topics = {}    # {lidar_name: PointCloud2 topic}
        self.lidar_topic_options = {}  # {lidar_name: [topics]} for lidars with several topics
        self.bag_queue = []       # every TrialBag to process, grouped by lidar
        self.bag_index = 0
        self.current_target_index = 0
        self.processed_cloud = None
        self._cloud_cache = None  # (bag path, topic, merged cloud) for the current bag

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

        self.btn_load_bag = QtWidgets.QPushButton("Load Bag Directory")
        self.btn_load_bag.clicked.connect(self.load_bag_directory)
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

        # Saves every remaining target without stopping for review; doubles as Stop
        self.btn_auto_process = QtWidgets.QPushButton("Auto-Process Remaining")
        self.btn_auto_process.clicked.connect(self.toggle_auto_process)
        control_layout.addWidget(self.btn_auto_process)
        self.auto_running = False
        self.auto_stop_requested = False

        # Added Repeatability button
        self.btn_process_repeatability = QtWidgets.QPushButton("Process Repeatability")
        self.btn_process_repeatability.clicked.connect(self.process_repeatability)
        control_layout.addWidget(self.btn_process_repeatability)

        # Progress / status read-out
        self.status_label = QtWidgets.QLabel("No bag directory loaded")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(QtCore.Qt.AlignTop)
        control_layout.addWidget(self.status_label)

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

    def load_bag_directory(self):
        """Select a directory of 'test##_lidar_name.bag' files and group them by lidar."""
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select Bag Directory", "./", options=options
        )
        if not directory:
            return

        groups, skipped, warnings = discover_bags(directory)
        for path in skipped:
            print(f"Skipping bag not named 'test##_lidar_name.bag': {path}")
        for warning in warnings:
            print(f"Warning: {warning}")

        if not groups:
            self.status_label.setText("No 'test##_lidar_name.bag' files found in the selected directory")
            return

        self.bag_dir = directory
        self.bag_groups = groups
        self.bag_queue = []
        self._cloud_cache = None

        # Pick each lidar's PointCloud2 topic from its first bag; lidars publishing
        # several topics are resolved with "Select Topic" (asked once per lidar).
        self.lidar_topics = {}
        self.lidar_topic_options = {}
        for lidar_name, bags in groups.items():
            topics = ROSPointCloudLoader(str(bags[0].path)).get_pc2_topics()
            if len(topics) == 1:
                self.lidar_topics[lidar_name] = topics[0]
            elif topics:
                self.lidar_topic_options[lidar_name] = topics
            else:
                print(f"Warning: no PointCloud2 topic in {bags[0].path.name}; {lidar_name} will be skipped")

        print(f"Loaded bag directory: {directory}")
        for lidar_name, bags in groups.items():
            topic = self.lidar_topics.get(lidar_name, "topic not selected")
            print(f"  {lidar_name}: {len(bags)} trial(s) {[b.trial for b in bags]} on {topic}")

        lines = [f"<b>{len(groups)} lidar(s), {sum(len(b) for b in groups.values())} bags</b>"]
        lines += [f"{name}: {len(bags)} trials" for name, bags in groups.items()]
        if self.lidar_topic_options:
            lines.append("<i>Select Topic needed for: " + ", ".join(self.lidar_topic_options) + "</i>")
        if skipped:
            lines.append(f"Skipped {len(skipped)} file(s) not matching test##_lidar_name.bag")
        self.status_label.setText("<br>".join(lines))

    def select_topic(self):
        """Choose a topic for each lidar that publishes more than one PointCloud2 topic."""
        if not self.bag_groups:
            print("Please load a bag directory first.")
            return
        if not self.lidar_topic_options:
            print("Every lidar has a single PointCloud2 topic; nothing to select.")
            return

        for lidar_name, topics in self.lidar_topic_options.items():
            current = self.lidar_topics.get(lidar_name)
            item, ok = QtWidgets.QInputDialog.getItem(
                self,
                "Select Topic",
                f"Choose the PointCloud2 topic for '{lidar_name}':",
                topics,
                topics.index(current) if current in topics else 0,
                False,
            )
            if ok and item:
                print(f"{lidar_name}: using topic {item}")
                self.lidar_topics[lidar_name] = str(item)

    def _prepare_queue(self) -> bool:
        """Load the experiment targets and queue every bag, grouped by lidar."""
        if not self.experiment_file:
            print("Please select an experiment YAML file first.")
            return False
        if not self.bag_groups:
            print("Please load a bag directory first.")
            return False

        if any(name not in self.lidar_topics for name in self.lidar_topic_options):
            self.select_topic()

        with open(self.experiment_file, "r") as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            self.key_name, self.targets = next(iter(yaml_data.items()))
            self.num_targets = len(self.targets)

        # Process all trials of one lidar before moving on to the next lidar
        self.bag_queue = []
        for lidar_name, bags in self.bag_groups.items():
            if lidar_name not in self.lidar_topics:
                print(f"Skipping {lidar_name}: no topic selected")
                continue
            self.bag_queue.extend(bags)

        self.bag_index = 0
        self.current_target_index = 0
        return bool(self.bag_queue)

    def process_targets(self):
        if self.auto_running:
            return
        if self._prepare_queue():
            self.plot_current_target()

    def next_target(self):
        if self.auto_running or not self.bag_queue or self.bag_index >= len(self.bag_queue):
            return
        self._save_current_target()
        self._advance()
        self.plot_current_target()

    def _save_current_target(self) -> bool:
        """Save the current target's extracted points; returns False if there were none."""
        bag = self.bag_queue[self.bag_index]
        if self.processed_cloud is None:
            print(f"No points saved for {bag.lidar_name} trial {bag.trial} {self.target}")
            return False

        exp_stem = Path(self.experiment_file).stem
        target = str(self.target).strip("/\\")
        target_dir = Path.cwd() / "results" / exp_stem / bag.lidar_name / target

        # Save the filtered planar target inliers, numbered by the bag's trial number
        save_data(
            pointcloud=self.processed_cloud,
            output_dir=target_dir,
            name="inner_planar_target_points",
            trial_num=bag.trial,
        )
        return True

    def _advance(self):
        self.current_target_index += 1
        if self.current_target_index >= self.num_targets:
            self.current_target_index = 0
            self.bag_index += 1

    def toggle_auto_process(self):
        if self.auto_running:
            # Stop takes effect after the current target finishes
            self.auto_stop_requested = True
            self.btn_auto_process.setText("Stopping...")
            return
        self.auto_process()

    def auto_process(self):
        """Save every remaining target without review, from the current position onward.

        A per-target log (saved / skipped and why) is written next to the results so
        unreviewed extractions can be audited afterwards.
        """
        finished = not self.bag_queue or self.bag_index >= len(self.bag_queue)
        if finished and not self._prepare_queue():
            return

        self.auto_running = True
        self.auto_stop_requested = False
        self.btn_auto_process.setText("Stop Auto-Process")
        for button in (self.btn_load_experiment, self.btn_load_bag, self.btn_select_topic,
                       self.btn_start, self.btn_next_target, self.btn_process_repeatability):
            button.setEnabled(False)

        log_rows = []
        try:
            while self.bag_index < len(self.bag_queue) and not self.auto_stop_requested:
                bag = self.bag_queue[self.bag_index]
                QtWidgets.QApplication.processEvents()  # keep the window responsive / see Stop
                ok, message = self._extract_current_target()
                QtWidgets.QApplication.processEvents()
                saved = ok and self._save_current_target()
                log_rows.append({
                    "lidar_name": bag.lidar_name,
                    "trial": bag.trial,
                    "target": self.target,
                    "status": "saved" if saved else "skipped",
                    "points": self.processed_cloud.n_points if saved else 0,
                    "detail": "" if saved else message,
                    "bag": str(bag.path),
                })
                self._advance()
        finally:
            self.auto_running = False
            self.btn_auto_process.setText("Auto-Process Remaining")
            for button in (self.btn_load_experiment, self.btn_load_bag, self.btn_select_topic,
                           self.btn_start, self.btn_next_target, self.btn_process_repeatability):
                button.setEnabled(True)

        stopped = self.auto_stop_requested and self.bag_index < len(self.bag_queue)
        n_saved = sum(r["status"] == "saved" for r in log_rows)
        summary = f"Auto-process {'stopped' if stopped else 'finished'}: {n_saved} saved, {len(log_rows) - n_saved} skipped"

        if log_rows:
            log_dir = Path.cwd() / "results" / Path(self.experiment_file).stem
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"auto_process_log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv"
            pd.DataFrame(log_rows).to_csv(log_path, index=False)
            summary += f"\nLog: {log_path}"
        print(summary)

        # Show where processing paused (or the finished screen) for manual review
        self.plot_current_target()
        if stopped:
            self.status_label.setText(self.status_label.text() + "<br><i>Auto-process stopped here</i>")

    def process_repeatability(self):
        """Handler for processing LiDAR repeatability."""
        if not self.experiment_file:
            print("Please select an experiment YAML file first.")
            return

        exp_stem = Path(self.experiment_file).stem
        target_dir = Path.cwd() / "results" / exp_stem

        print("Processing repeatability analysis...")
        analyzer = LiDARRepeatabilityAnalyzer(root_dir=str(target_dir), experiment_file=self.experiment_file,
                                              chart_lidars=REPORT_CHART_LIDARS)

        # Traverse results directory and compute stats
        analyzer.scan_and_process()

        # Export CSV and convert CSV to PDF (saves in experiment directory)
        csv_path = analyzer.export_summary_csv(str(target_dir / "lidar_repeatability_summary.csv"))
        pdf_path = analyzer.export_pdf_report(csv_path)
        print(f"Repeatability report saved at: {pdf_path}")

    def _load_bag_cloud(self, bag, topic):
        """Read a bag once and reuse its merged cloud for every target."""
        if self._cloud_cache is None or self._cloud_cache[:2] != (bag.path, topic):
            print(f"Reading {bag.path.name} ({topic})...")
            self._cloud_cache = None  # release the previous bag before loading the next
            cloud = ROSPointCloudLoader(str(bag.path)).read_clouds(topic, True)
            self._cloud_cache = (bag.path, topic, cloud)
        return self._cloud_cache[2]

    def plot_current_target(self):
        self.plotter.clear()

        if self.bag_index >= len(self.bag_queue):
            self.processed_cloud = None
            print("All bags processed")
            self._cloud_cache = None
            self.status_label.setText("<b>All bags and targets processed</b>")
            self.plotter.add_text(
                "All bags processed!", position="lower_edge", font_size=12
            )
            return

        ok, message = self._extract_current_target()
        if not ok:
            self.plotter.add_text(
                f"No target points found ({message}).\nNext Target skips without saving.",
                position="upper_left", font_size=10,
            )
            return

        # Render strictly the cleaned planar inliers, coloured by intensity when available
        color_scalar = (
            "intensity"
            if "intensity" in self.processed_cloud.point_data
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

    def _extract_current_target(self) -> tuple[bool, str]:
        """Extract the current target from the current bag into self.processed_cloud.

        :return: (success, reason when unsuccessful)
        """
        self.processed_cloud = None
        bag = self.bag_queue[self.bag_index]
        topic = self.lidar_topics[bag.lidar_name]
        lidar_bags = self.bag_groups[bag.lidar_name]

        self.target = list(self.targets)[self.current_target_index]
        [target_w, target_h] = self.targets[self.target]["size"]

        progress = (
            f"<b>{bag.lidar_name}</b> — trial {bag.trial} "
            f"({lidar_bags.index(bag) + 1}/{len(lidar_bags)})<br>"
            f"{self.target} ({self.current_target_index + 1}/{self.num_targets})<br>"
            f"Bag {self.bag_index + 1}/{len(self.bag_queue)} overall<br>"
            f"<small>{bag.path.name} · {topic}</small>"
        )
        self.status_label.setText(progress)
        print(
            f"Processing {bag.lidar_name} trial {bag.trial}, "
            f"target [{self.current_target_index + 1} / {self.num_targets}]"
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
        bag_cloud = self._load_bag_cloud(bag, topic)
        raw_target_subset = bag_cloud.clip_box(
            bounds=[x_min, x_max, y_min, y_max, z_min, z_max], invert=False
        )

        # 2. Automatically clean noisy outliers using IRLS plane fitting
        try:
            if raw_target_subset.n_points < MIN_TARGET_POINTS:
                raise ValueError(f"only {raw_target_subset.n_points} points in the search box")
            self.processed_cloud, plane_params = fit_plane_irls(
                cloud=raw_target_subset,
                loss_function="tukey",
                inlier_threshold=0.02,
                target_height=target_h,
                target_width=target_w,
            )
            if self.processed_cloud.n_points == 0:
                raise ValueError("no points left after plane fitting")
        except (ValueError, IndexError) as e:
            # Target not visible to this lidar (e.g. outside its field of view)
            self.processed_cloud = None
            print(f"No target extracted for {self.target}: {e}")
            return False, str(e)

        return True, ""
