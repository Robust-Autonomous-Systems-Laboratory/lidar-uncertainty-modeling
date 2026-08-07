from src.pointcloud_target_processing.gui import LidarUncertaintyGUI
from PyQt5 import QtWidgets
import sys

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LidarUncertaintyGUI()
    window.show()
    sys.exit(app.exec_())