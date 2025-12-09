"""
This ROS 2 node connects to a microcontroller via a serial USB connection, and publishes ToF sensor
data to the mini_tof_data topic.

Examples of launching this node with ROS 2 parameters:
ros2 run mini_tof tof_publisher --ros-args -p mcu_port:=/dev/ttyACM0 -p sensor_model:=TMF882X
ros2 run mini_tof tof_publisher --ros-args -p mcu_port:=/dev/ttyUSB0 -p sensor_model:=VL53L8CH -p num_zones:=16

ROS 2 parameters:
- sensor_model: The model of the ToF sensor. Required. Supported models are: TMF882X, VL53L8CH.
- mcu_port: The port to which the microcontroller is connected. Defaults to /dev/ttyACM0.
- num_zones: The number of zones reported by the sensor. Required for VL53L8CH (16 or 64).
    Defaults to 9 for TMF882X.

ROS 2 publications:
- mini_tof_data: Publishes ToF sensor data from sensor connected to connected microcontroller.
"""

import rclpy
from rclpy.node import Node

from mini_tof.readers.tmf882x_reader import TMF882XReader
from mini_tof.readers.tmf882x_distance_only_reader import TMF882XDistanceOnlyReader
from mini_tof.readers.vl53l8ch_reader import VL53L8CHReader, VL53L8CHReaderNoAggregation
from mini_tof_interfaces.msg import ToFFrame

SUPPORTED_SENSOR_MODELS = ["TMF882X", "TMF882X_DISTANCE_ONLY", "VL53L8CH"]


class ToFPublisher(Node):
    def __init__(self):
        super().__init__("tof_publisher")

        # ROS parameter for the microcontroller port. Defaults to /dev/ttyACM0
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

        # ROS parameter for the number of zones reported by the sensor. Required for VL53L8CH.
        self.declare_parameter("num_zones", rclpy.Parameter.Type.INTEGER)
        try:
            self.num_zones = self.get_parameter("num_zones").value
        except:
            self.num_zones = None
        if self.sensor_model == "VL53L8CH" and self.num_zones is None:
            raise ValueError("Parameter 'num_zones' must be provided for VL53L8CH sensor")
        elif self.sensor_model == "VL53L8CH" and self.num_zones not in [16, 64]:
            raise ValueError("VL53L8CH sensor only supports 16 or 64 zones")
        elif self.sensor_model == "TMF882X" or self.sensor_model == "TMF882X_DISTANCE_ONLY":
            self.num_zones = 9  # TMF882X always has 9 zones (with our firmware)

        try:
            self.mcu_port = self.get_parameter("mcu_port").value
            self.get_logger().info(f"Using provided Arduino port {self.mcu_port}")
        except:
            self.mcu_port = "/dev/ttyACM0"
            # TODO auto-detect platform (windows, mac, linux) and set default port accordingly
            self.get_logger().info(f"No Arduino port provided, using default {self.mcu_port}")

        if self.sensor_model == "TMF882X":
            self.reader = TMF882XReader(self.mcu_port)
        elif self.sensor_model == "TMF882X_DISTANCE_ONLY":
            self.reader = TMF882XDistanceOnlyReader(self.mcu_port)
        elif self.sensor_model == "VL53L8CH":
            self.reader = VL53L8CHReaderNoAggregation(self.mcu_port, self.num_zones)

        self.publisher = self.create_publisher(ToFFrame, "mini_tof_data", 1)

        self.timer = self.create_timer(0.001, self.timer_callback)

        self.received_data = False

    def timer_callback(self):
        m = self.reader.get_measurement()
        if m is None:
            return
        message = self.reader.measurement_to_ros_msg(m, self.mcu_port, self.sensor_model)
        if not self.received_data:
            self.get_logger().info("Received data from MCU")
            self.received_data = True
            # Initialize FPS tracking
            self.frame_count = 0
            self.last_time = self.get_clock().now()

        # Update FPS calculation
        self.frame_count += 1
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_time).nanoseconds / 1e9
        if elapsed > 3.0:  # Print FPS every 3 seconds
            fps = self.frame_count / elapsed
            self.get_logger().info(f"FPS: {fps:.2f}")
            self.frame_count = 0
            self.last_time = current_time
        
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    tmf882x_pub = ToFPublisher()
    rclpy.spin(tmf882x_pub)

    tmf882x_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
