"""
This ROS 2 node connects to a microcontroller via a serial USB connection, and publishes ToF sensor
data to the mini_tof_data topic.

Example of launching this node with ROS 2 parameters:
ros2 run mini_tof tof_publisher --ros-args -p mcu_port:=/dev/ttyACM0

ROS 2 parameters:
- mcu_port: The port to which the microcontroller is connected. Defaults to /dev/ttyACM0.

ROS 2 publications:
- mini_tof_data: Publishes ToF sensor data from sensor connected to connected microcontroller.
"""

import rclpy
from rclpy.node import Node
from readers.tmf8820_reader import TMF8820Reader

from mini_tof_interfaces.msg import ToFHistogram, ToFFrame, DepthEstimate


class ToFPublisher(Node):
    def __init__(self):
        super().__init__("tof_publisher")

        # ROS parameter for the Arduino port. Defaults to /dev/ttyACM0
        self.declare_parameter("mcu_port", rclpy.Parameter.Type.STRING)

        try:
            self.mcu_port = self.get_parameter("mcu_port").value
            self.get_logger().info(f"Using provided Arduino port {self.mcu_port}")
        except:
            self.mcu_port = "/dev/ttyACM0"
            self.get_logger().info(f"No Arduino port provided, using default {self.mcu_port}")

        self.reader = TMF8820Reader(self.mcu_port)

        self.publisher = self.create_publisher(ToFFrame, "mini_tof_data", 1)

        self.timer = self.create_timer(0.005, self.timer_callback)

        self.received_data = False

    def timer_callback(self):
        m = self.reader.get_measurement()
        if m is None:
            return
        hists, dists, timestamp = m
        if not self.received_data:
            self.get_logger().info("Received data from MCU")
            self.received_data = True

        message = ToFFrame()
        message.i2c_address = dists[0]["I2C_address"]
        message.tick = dists[0]["tick"]
        message.num_valid_results = dists[0]["num_valid_results"]
        message.temperature = dists[0]["temperature"]
        message.measurement_num = dists[0]["measurement_num"]
        message.depth_estimates = [
            DepthEstimate(depth_estimates=dists[0]["depths_1"], confidences=dists[0]["confs_1"]),
            DepthEstimate(depth_estimates=dists[0]["depths_2"], confidences=dists[0]["confs_2"]),
        ]
        message.serial_port = self.mcu_port
        if hists[0]:  # if hists is not an empty list (histograms are being reported)
            message.histograms = [ToFHistogram(histogram=hist) for hist in hists[0][1:]]
            message.reference_histogram = ToFHistogram(histogram=hists[0][0])

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    tmf882x_pub = ToFPublisher()
    rclpy.spin(tmf882x_pub)

    tmf882x_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
