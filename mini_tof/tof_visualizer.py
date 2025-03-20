"""
This ROS 2 node visualizes ToF sensor data in real-time. The tof_publisher node must be running
for this node to receive data.

ROS 2 subscriptions:
- mini_tof_data: Reads ToFFrame messages and live plots data.
"""

import numpy as np
import pyqtgraph as pg
import rclpy
from PyQt6 import QtWidgets
from PyQt6.QtWidgets import QMainWindow
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.wait_for_message import wait_for_message

from mini_tof_interfaces.msg import ToFFrame


class ToFVisualizerNode(Node, QMainWindow):
    """
    This class needs to inherit from both Node and QtWidgets.QMainWindow, so it can have the update
    loops for both. See example: https://stackoverflow.com/a/78539454/8841061

    The order of Node, QMainWindow in the above line matters. Maybe this has to do with NRO
    (https://stackoverflow.com/q/3277367/8841061).
    """
    def __init__(self):
        QMainWindow.__init__(self)
        Node.__init__(self, "tof_visualizer_node")

        self.subscriber = self.create_subscription(ToFFrame, "mini_tof_data", self.sub_callback, 1)

        # get a single message to determine the number of zones and sensor model, which are used
        # for setting up the plot. This must be done in the init function (not a callback) due to
        # restrictions on making Qt widgets in a separate thread.
        got_msg, msg = wait_for_message(ToFFrame, self, "mini_tof_data", time_to_wait=3.0)
        if not got_msg:
            self.get_logger().error("Did not receive data on mini_tof_data topic - is tof_publisher running?")
            return
        self.num_zones = len(msg.histograms)
        self.sensor_model = msg.sensor_model
        self.set_zone_order()  # set the zone order based on the sensor model and number of zones
        self.get_logger().info(f"Detected {self.sensor_model} sensor, {self.num_zones} zones")

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
        """
        Update plot with new sensor data.
        """
        hists = np.array([msg.histograms[i].histogram for i in range(len(msg.histograms))])
            
        argmaxes = np.argmax(hists, axis=1)

        for i, zone in enumerate(self.zone_order):
            hist = hists[zone]

            self.lines[i].setData(hist)

        self.image_item.setImage(argmaxes[self.zone_order].reshape(3, 3), levels=(0, 128))

    def set_zone_order(self):
        """
        Set the zone order used for plotting, which varies based on the sensor model and number
        of zones
        """

        if self.sensor_model == "TMF882X" and self.num_zones == 9:
            """
            the histogram in index 1 is in the top left, 4 is in the top middle, etc.
            as if you're reading a book, so the zones look like this:
            2 1 0
            5 4 3
            8 7 6
            This arrangement gives you something like a camera would, the view as you're looking
            *through* the sensor lines up with what's plotted.
            """
            # fmt: off
            self.zone_order = [
                2, 1, 0,
                5, 4, 3,
                8, 7, 6
            ]  # bottom row in view is idx 8, 7, 6 from sensor
            # fmt: on

        elif self.sensor_model == "VL53L8CH":
            # don't re-arrange the zones from how they're reported
            self.zone_order = list(range(self.num_zones))
        else:
            self.get_logger().warning(
                f"Got unexpected combination of sensor model ({self.sensor_model}) and number of zones ({self.num_zones}). "
                f"Defaulting to zone order of 0, 1, 2, ... {self.num_zones-1}"
            )
            self.zone_order = list(range(self.num_zones))

def main(args=None):
    """
    This unusual main function is necessary to run the PyQt6 GUI and ROS 2 node in the same file.
    https://stackoverflow.com/a/78539454/8841061
    """
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
