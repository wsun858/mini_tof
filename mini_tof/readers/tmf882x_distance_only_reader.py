"""
Distance-only TMF882X reader for 3x3 mode.

This version assumes the microcontroller firmware only prints "#Obj" lines
(i.e., no histograms), and reads a single distance frame at a time.

It is API-compatible with the original TMF882XDistanceOnlyReader used by tof_publisher.py:
    - get_measurement() returns (hists, dists, timestamp)
    - measurement_to_ros_msg(...) builds a mini_tof_interfaces.msg.ToFFrame

Histograms are not provided by the new firmware, so they are left empty / None.
"""

import time
import serial

from mini_tof_interfaces.msg import DepthEstimate, ToFFrame, ToFHistogram

# For compatibility with older code; not actually used for distance-only mode.
TMF882X_CHANNELS = 10
TMF882X_BINS = 128


class TMF882XDistanceOnlyReader:
    def __init__(self, port, baudrate=1000000, timeout=0.05):
        """
        Args:
            port (str): Serial port of the microcontroller (e.g. /dev/ttyACM0).
            baudrate (int): Must match the firmware (default: 1,000,000).
            timeout (float): Serial read timeout in seconds.
        """
        self.mcu = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        # Give the MCU a moment to reset after opening the port
        time.sleep(2.0)

    def get_measurement(self, output=None, flush_input=False, buffer_warnings=True):
        """
        Read a single distance measurement frame from the TMF882X.

        New firmware prints only a single "#Obj" line per measurement, so we:
            - read lines until one starts with "#Obj"
            - parse that line into a distance dict
            - return (hists, dists, timestamp), with hists left empty

        Returns:
            tuple | None:
                (all_processed_hists, all_processed_dists, timestamp)
                or None if no complete measurement was available before timeout.
        """
        if flush_input:
            self.mcu.reset_input_buffer()

        while True:
            line = self.mcu.readline().rstrip()
            if not line:
                # Timeout: no data available this call
                return None

            try:
                decoded = line.decode("utf-8").strip()
            except UnicodeDecodeError:
                # Started in middle of a frame; just skip this line
                continue

            if not decoded:
                continue

            # New firmware outputs ToF results as "#Obj, ..." lines
            if not decoded.startswith("#Obj"):
                # Ignore any other debug or log lines
                continue

            # Reuse the original parsing logic on a single-line "buffer"
            buffer = [line]
            result = TMF882XDistanceOnlyReader.process_raw_dist(buffer)
            if result is None:
                if buffer_warnings:
                    print("WARNING: Failed to parse TMF882X #Obj line, skipping")
                return None

            # No histograms from the firmware; use an empty placeholder so that
            # measurement_to_ros_msg can safely check hists[0].
            all_processed_hists = [[]]  # list with a single empty entry
            all_processed_dists = [result]
            timestamp = time.time()

            if output is not None:
                output[self.mcu.port] = {
                    "hists": all_processed_hists,
                    "dists": all_processed_dists,
                    "timestamp": timestamp,
                }

            return all_processed_hists, all_processed_dists, timestamp

    def measurement_to_ros_msg(self, m, mcu_port, sensor_model):
        """
        Convert a measurement from get_measurement() into a ToFFrame ROS message.

        Histograms are omitted in the new firmware; the corresponding fields in
        the ToFFrame message are left empty.
        """
        hists, dists, timestamp = m

        d0 = dists[0]

        message = ToFFrame()
        message.ambient_light = []  # not provided by TMF882X
        message.i2c_address = d0["I2C_address"]
        message.tick = d0["tick"]
        message.num_valid_results = d0["num_valid_results"]
        message.temperature = d0["temperature"]
        message.measurement_num = d0["measurement_num"]

        # 3x3 sensor => 9 zones per block. We keep the original layout: two
        # DepthEstimate entries, each with up to 9 depths + confidences.
        message.depth_estimates = [
            DepthEstimate(
                depth_estimates=d0["depths_1"],
                confidences=d0["confs_1"],
            ),
            DepthEstimate(
                depth_estimates=d0["depths_2"],
                confidences=d0["confs_2"],
            ),
        ]

        message.serial_port = mcu_port
        message.sensor_model = sensor_model

        # New FW doesn't output histograms; only populate if hists[0] is non-empty.
        if hists and hists[0]:
            message.histograms = [
                ToFHistogram(histogram=hist) for hist in hists[0][1:]
            ]
            message.reference_histogram = ToFHistogram(histogram=hists[0][0])

        return message

    @classmethod
    def process_raw_dist(cls, buffer):
        """
        Parse one or more lines of TMF882X "#Obj" output into a distance dict.

        The original 3x3-mode firmware prints 78 comma-separated entries on
        "#Obj" lines. For robustness we only require that there be at least
        enough columns for the two 3x3 blocks (60 columns).
        """
        for line in buffer:
            data = line.decode("utf-8")
            data = data.replace("\r", "").replace("\n", "")
            d = data.split(",")

            # In 3x3 mode the original code checked for len(d) == 78.
            # Here we allow len(d) >= 60 in case trailing fields were removed.
            if len(d) >= 60 and d[0] == "#Obj":
                try:
                    result = {}
                    result["I2C_address"] = int(d[1])
                    result["measurement_num"] = int(d[2])
                    result["temperature"] = int(d[3])
                    result["num_valid_results"] = int(d[4])
                    result["tick"] = int(d[5])

                    # First 3x3 block
                    result["depths_1"] = [
                        int(x)
                        for x in [
                            d[6],
                            d[8],
                            d[10],
                            d[12],
                            d[14],
                            d[16],
                            d[18],
                            d[20],
                            d[22],
                        ]
                    ]
                    result["confs_1"] = [
                        int(x)
                        for x in [
                            d[7],
                            d[9],
                            d[11],
                            d[13],
                            d[15],
                            d[17],
                            d[19],
                            d[21],
                            d[23],
                        ]
                    ]

                    # Second 3x3 block (if present); otherwise mirror the first.
                    if len(d) >= 60:
                        result["depths_2"] = [
                            int(x)
                            for x in [
                                d[42],
                                d[44],
                                d[46],
                                d[48],
                                d[50],
                                d[52],
                                d[54],
                                d[56],
                                d[58],
                            ]
                        ]
                        result["confs_2"] = [
                            int(x)
                            for x in [
                                d[43],
                                d[45],
                                d[47],
                                d[49],
                                d[51],
                                d[53],
                                d[55],
                                d[57],
                                d[59],
                            ]
                        ]
                    else:
                        # Fallback if firmware only sends a single 3x3 block
                        result["depths_2"] = result["depths_1"]
                        result["confs_2"] = result["confs_1"]

                    return result
                except (ValueError, IndexError):
                    # Malformed line; treat as no result
                    return None

        return None
