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
from mini_tof.readers.tmf882x_reader import TMF882XReader
from mini_tof.readers.vl53l8ch_reader import VL53L8CHReader

from mini_tof_interfaces.msg import ToFFrame

SUPPORTED_SENSOR_MODELS = ["TMF882X", "VL53L8CH"]


class ToFPublisher(Node):
    def __init__(self):
        super().__init__("tof_publisher")

        # ROS parameter for the Arduino port. Defaults to /dev/ttyACM0
        self.declare_parameter("mcu_port", rclpy.Parameter.Type.STRING)

        # ROS parameter for the sensor model. Required.
        self.declare_parameter("sensor_model", rclpy.Parameter.Type.STRING)
        self.sensor_model = self.get_parameter("sensor_model").value
        if self.sensor_model is None:
            raise ValueError("Parameter 'sensor_model' must be provided")
        if self.sensor_model not in SUPPORTED_SENSOR_MODELS:
            raise ValueError(
                f"Unsupported sensor model '{self.sensor_model}'. Supported models are: {SUPPORTED_SENSOR_MODELS}"
            )

        try:
            self.mcu_port = self.get_parameter("mcu_port").value
            self.get_logger().info(f"Using provided Arduino port {self.mcu_port}")
        except:
            self.mcu_port = "/dev/ttyACM0"
            # TODO auto-detect platform (windows, mac, linux) and set default port accordingly
            self.get_logger().info(f"No Arduino port provided, using default {self.mcu_port}")

        if self.sensor_model == "TMF882X":
            self.reader = TMF882XReader(self.mcu_port)
        elif self.sensor_model == "VL53L8CH":
            self.reader = VL53L8CHReader(self.mcu_port)

        self.publisher = self.create_publisher(ToFFrame, "mini_tof_data", 1)

        self.timer = self.create_timer(0.005, self.timer_callback)

        self.received_data = False

    def timer_callback(self):
        m = self.reader.get_measurement()
        if m is None:
            return
        message = self.reader.measurement_to_ros_msg(m, self.mcu_port, self.sensor_model)
        if not self.received_data:
            self.get_logger().info("Received data from MCU")
            self.received_data = True

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    tmf882x_pub = ToFPublisher()
    rclpy.spin(tmf882x_pub)

    tmf882x_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
