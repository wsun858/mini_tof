"""
Class which enables reading and decoding data from a VL53L8CH ToF sensor connected to a
microcontroller via USB serial.
"""

import struct
import time

import serial

from mini_tof_interfaces.msg import DepthEstimate, ToFFrame, ToFHistogram

# TODO support changing the number of zones without modifying source code
NUM_ZONES = 16

class VL53L8CHReader:
    def __init__(self, port):
        self.mcu = serial.Serial(
            port,
            921600,
            bytesize=8,
            parity=serial.PARITY_ODD,
            stopbits=1,
            dsrdtr=True,
            rtscts=True,
            xonxoff=False,
        )
        time.sleep(2)

    def get_measurement(self):
        self.mcu.reset_input_buffer()

        # the sensor only supports 16 or 64 zones
        if not NUM_ZONES in [16, 64]:
            print(f"VL53L8CH sensor only supports 16 or 64 zones ({NUM_ZONES} provided)")
            return

        frame_data = []
        valid_frame = True
        expected_zone_idx = 0

        # Read data for all zones in the frame.
        while expected_zone_idx < NUM_ZONES:
            zone_idx, zone_data, _ = VL53L8CHReader.readline_and_decode(self.mcu)
            if zone_idx != expected_zone_idx:
                # print(f"Warning: Expected zone {expected_zone_idx} but received zone {zone_idx}")
                valid_frame = False
                break

            frame_data.append(zone_data)
            expected_zone_idx += 1

        if valid_frame:
            return frame_data

    def measurement_to_ros_msg(self, m, mcu_port, sensor_model):
        """
        Convert a measurement as returned by get_measurement() into a
        mini_tof_interfaces.msg.ToFFrame ROS message.

        Args:
            m (tuple): The measurement returned by get_measurement().
            mcu_port (str): The port of the microcontroller.
            sensor_model (str): The model of the sensor.
        
        Returns:
            ToFFrame: A ROS message containing the processed measurement data.
        """
        message = ToFFrame()
        message.i2c_address = -1  # VL53L8CH does not use I2C address
        message.tick = -1 # VL53L8CH does not provide tick information
        message.num_valid_results = -1 # does not apply to VL53L8CH
        message.temperature = -1 # VL53L8CH does not provide temperature information
        message.measurement_num = -1 # VL53L8CH does not provide measurement number
        message.depth_estimates = [
            DepthEstimate(depth_estimates=[], confidences=[]),
            DepthEstimate(depth_estimates=[], confidences=[]),
        ]
        message.serial_port = mcu_port
        message.sensor_model = sensor_model
        message.histograms = [ToFHistogram(histogram=hist) for hist in m]
        message.reference_histogram = ToFHistogram(histogram=[]) # VL53L8CH does not provide a reference histogram

        return message


    @classmethod
    def readline_and_decode(cls, ser):
        """
        Given a serial port object denoting a port connected to an MCU with a VL3L8CH connected,
        read a line of data from the port and decode it

        Args:
            ser (serial.Serial): Serial port object

        Returns:
            int: Zone index
            list: List of floats representing the data from the zone
            int: Length of the binary data read
        """

        eol = b"\xFF\xFF\xFF\xFF"

        c = ser.read_until(expected=eol)
        byte_listing = bytearray(c)
        length = len(byte_listing) - len(byte_listing) % 4
        line_as_floats = [
            struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length, 4)
        ]

        # line_as_floats[-1] is always NaN, so remove it
        # TODO figure out why this is and if we're missing out on data by doing this
        return int(line_as_floats[0]), line_as_floats[1:-1], length