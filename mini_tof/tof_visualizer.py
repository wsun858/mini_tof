import time

import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
import rclpy
from PyQt6 import QtCore, QtWidgets
from PyQt6.QtWidgets import QMainWindow
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from mini_tof_interfaces.msg import ToFFrame


class ToFVisualizerNode(Node, QMainWindow):
    """
    This class needs to inherit from both Node and QtWidgets.QMainWindow, so it can have the update
    loops for both. See example: https://stackoverflow.com/a/78539454/8841061

    I'm not sure exactly why, but the order of Node, QMainWindow in the above line matters. Maybe
    this has to do with NRO (https://stackoverflow.com/q/3277367/8841061).
    """

    def __init__(self):
        QMainWindow.__init__(self)
        Node.__init__(self, "tof_visualizer_node")

        """
        for plotting - the histogram in index 1 is in the top left, 4 is in the top middle, etc.
        as if you're reading a book, so the zones look like this:
        2 1 0
        5 4 3
        8 7 6
        This arrangement gives you something like a camera would, the view as you're looking
        *through* the sensor lines up with what's plotted.
        """
        # fmt: off
        self.ZONE_ORDER = [
            2, 1, 0,
            5, 4, 3,
            8, 7, 6
        ]  # bottom row in view is idx 8, 7, 6 from sensor
        # fmt: on

        self.subscriber = self.create_subscription(ToFFrame, "tmf882x", self.sub_callback, 1)

        self.num_zones = 9

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.grid_layout = QtWidgets.QGridLayout(self.central_widget)

        self.plot_widgets = []
        self.lines = []

        grid_size = int(np.ceil(np.sqrt(self.num_zones)))
        pen = pg.mkPen(color=(255, 255, 255))

        for i in range(self.num_zones):
            plot_widget = pg.PlotWidget()
            plot_widget.setBackground("k")
            self.plot_widgets.append(plot_widget)
            self.lines.append(plot_widget.plot([], [], pen=pen))
            row = i // grid_size
            col = i % grid_size
            self.grid_layout.addWidget(plot_widget, row, col)

        # Add the image plot to the right of the 4x4 grid
        self.image_plot_widget = pg.PlotWidget()
        self.image_plot_widget.setBackground("k")
        self.image_item = pg.ImageItem()
        self.image_plot_widget.addItem(self.image_item)
        self.grid_layout.addWidget(
            self.image_plot_widget,  # widget to add
            1,  # row (0-indexed)
            grid_size,  # column (0-indexed)
            2,  # row span
            2,  # column span
        )

        # Set column stretch factors
        for col in range(grid_size):
            self.grid_layout.setColumnStretch(col, 1)
        self.grid_layout.setColumnStretch(grid_size, 3)  # Make the image column wider

    def sub_callback(self, msg):
        hists = np.array([msg.histograms[i].histogram for i in range(9)])
        max_val = np.max(hists)

        argmaxes = np.argmax(hists, axis=1)

        for i, zone in enumerate(self.ZONE_ORDER):
            hist = hists[zone]

            self.lines[i].setData(hist)

        self.image_item.setImage(argmaxes[self.ZONE_ORDER].reshape(3, 3), levels=(0, 128))


def main(args=None):
    app = QtWidgets.QApplication([])
    rclpy.init()
    ui = ToFVisualizerNode()
    app.processEvents()
    ui.show()

    exec = MultiThreadedExecutor()
    exec.add_node(ui)
    while rclpy.ok():
        try:
            exec.wait_for_ready_callbacks(0)
            exec.spin_once()
        except:
            pass
        app.processEvents()
    app.quit()
    exec.remove_node(ui)


if __name__ == "__main__":
    main()
