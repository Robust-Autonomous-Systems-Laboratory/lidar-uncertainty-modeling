import sys
from PyQt5 import QtWidgets
import pyvista as pv
from pyvistaqt import QtInteractor

class MyPyVistaApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
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
        self.btn_load_experiment.clicked.connect(self.load_file)
        control_layout.addWidget(self.btn_load_experiment)
        
        # Button 2: Load Bag
        self.btn_load_bag = QtWidgets.QPushButton("Load Bag")
        self.btn_load_bag.clicked.connect(self.load_file)
        control_layout.addWidget(self.btn_load_bag)
        
        # Button 3: Select Topic
        self.btn_select_topic = QtWidgets.QPushButton("Select Topic")
        self.btn_select_topic.clicked.connect(self.select_topic)
        control_layout.addWidget(self.btn_select_topic)
        
        # Spacer to push buttons to the top
        control_layout.addStretch()
        

    def load_file(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(self,"QFileDialog.getOpenFileName()", "","All Files (*);;Python Files (*.py)", options=options)
        if fileName:
            print(fileName)


    def select_topic(self):
        # iterate over selected bag and display list of all topics and msg types
        print("Select Topics button clicked")
        self.list_widget = QtWidgets.QListWidget()
        self.list_widget.addItems(["Python", "Java", "C++"])
        self.list_widget.itemClicked.connect(lambda item: print(item.text())) # Trigger on click


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MyPyVistaApp()
    window.show()
    sys.exit(app.exec_())
