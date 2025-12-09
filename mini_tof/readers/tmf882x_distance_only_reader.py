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
        print(f"[TMF882X DEBUG] Attempting to open serial port: {port}")
        print(f"[TMF882X DEBUG] Baudrate: {baudrate}, Timeout: {timeout}")
        try:
            self.mcu = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            print(f"[TMF882X DEBUG] Successfully opened serial port: {port}")
            print(f"[TMF882X DEBUG] Serial port info: {self.mcu}")
        except Exception as e:
            print(f"[TMF882X ERROR] Failed to open serial port {port}: {e}")
            raise
        # Give the MCU a moment to reset after opening the port
        print("[TMF882X DEBUG] Waiting 2 seconds for MCU to reset...")
        time.sleep(2.0)
        print("[TMF882X DEBUG] Initialization complete")

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
            print("[TMF882X DEBUG] Flushing input buffer")
            self.mcu.reset_input_buffer()

        line_count = 0
        while True:
            line = self.mcu.readline().rstrip()
            line_count += 1
            
            if not line:
                # Timeout: no data available this call
                if line_count == 1:
                    print("[TMF882X DEBUG] No data received (timeout on first read)")
                return None

            print(f"[TMF882X DEBUG] Raw line received ({len(line)} bytes): {line[:100]}")  # Print first 100 bytes

            try:
                decoded = line.decode("utf-8").strip()
                print(f"[TMF882X DEBUG] Decoded line: '{decoded[:100]}'")  # Print first 100 chars
            except UnicodeDecodeError as e:
                # Started in middle of a frame; just skip this line
                print(f"[TMF882X DEBUG] Unicode decode error: {e}")
                continue

            if not decoded:
                print("[TMF882X DEBUG] Empty decoded line, continuing")
                continue

            # New firmware outputs ToF results as "#Obj, ..." lines
            if not decoded.startswith("#Obj"):
                # Ignore any other debug or log lines
                print(f"[TMF882X DEBUG] Non-#Obj line (ignored): '{decoded[:50]}'")
                continue
            
            print(f"[TMF882X DEBUG] Found #Obj line! Full content: '{decoded}'")

            # Reuse the original parsing logic on a single-line "buffer"
            buffer = [line]
            print(f"[TMF882X DEBUG] Calling process_raw_dist with buffer of {len(buffer)} lines")
            result = TMF882XDistanceOnlyReader.process_raw_dist(buffer)
            if result is None:
                if buffer_warnings:
                    print("WARNING: Failed to parse TMF882X #Obj line, skipping")
                print(f"[TMF882X DEBUG] process_raw_dist returned None for line: '{decoded}'")
                return None

            print(f"[TMF882X DEBUG] Successfully parsed distance data: {result}")
            
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

            print(f"[TMF882X DEBUG] Returning measurement at timestamp {timestamp}")
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
        message.ambient_lights = []  # not provided by TMF882X
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
        print(f"[TMF882X DEBUG] process_raw_dist: Processing {len(buffer)} lines")
        for line_idx, line in enumerate(buffer):
            data = line.decode("utf-8")
            data = data.replace("\r", "").replace("\n", "")
            d = data.split(",")
            
            print(f"[TMF882X DEBUG] Line {line_idx}: Split into {len(d)} comma-separated fields")
            print(f"[TMF882X DEBUG] First field: '{d[0] if d else 'EMPTY'}'")
            if len(d) > 1:
                print(f"[TMF882X DEBUG] First 10 fields: {d[:10]}")

            # In 3x3 mode the original code checked for len(d) == 78.
            # Here we allow len(d) >= 60 in case trailing fields were removed.
            if len(d) >= 60 and d[0] == "#Obj":
                print(f"[TMF882X DEBUG] Valid #Obj line with {len(d)} fields, parsing...")
                try:
                    result = {}
                    result["I2C_address"] = int(d[1])
                    result["measurement_num"] = int(d[2])
                    result["temperature"] = int(d[3])
                    result["num_valid_results"] = int(d[4])
                    result["tick"] = int(d[5])

                    # First 3x3 block: distances are every other value starting at index 6
                    # e.g., d[6]=129, d[7]=255, d[8]=144, d[9]=255, etc.
                    # Distances: d[6], d[8], d[10], d[12], d[14], d[16], d[18], d[20], d[22]
                    # Confidences: d[7], d[9], d[11], d[13], d[15], d[17], d[19], d[21], d[23]
                    result["depths_1"] = [
                        int(d[6]),
                        int(d[8]),
                        int(d[10]),
                        int(d[12]),
                        int(d[14]),
                        int(d[16]),
                        int(d[18]),
                        int(d[20]),
                        int(d[22]),
                    ]
                    result["confs_1"] = [
                        int(d[7]),
                        int(d[9]),
                        int(d[11]),
                        int(d[13]),
                        int(d[15]),
                        int(d[17]),
                        int(d[19]),
                        int(d[21]),
                        int(d[23]),
                    ]

                    # Second 3x3 block: set to zeros since firmware only provides one block
                    # (all values after index 24 are 0 in the sensor output)
                    result["depths_2"] = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                    result["confs_2"] = [0, 0, 0, 0, 0, 0, 0, 0, 0]

                    print(f"[TMF882X DEBUG] Successfully parsed result: measurement_num={result['measurement_num']}, temp={result['temperature']}")
                    return result
                except (ValueError, IndexError) as e:
                    # Malformed line; treat as no result
                    print(f"[TMF882X DEBUG] Exception during parsing: {e}")
                    return None
            else:
                print(f"[TMF882X DEBUG] Line rejected: len(d)={len(d)}, d[0]='{d[0] if d else 'EMPTY'}'")

        print("[TMF882X DEBUG] No valid #Obj line found in buffer")
        return None
