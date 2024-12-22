import numpy as np
import serial
import time
import struct
import numpy as np

#Dont do distances from VL sensor



class VL53L8CHReader:
    def __init__(self, port):
        self.ser = serial.Serial(
            port,
            921600,
            bytesize=8,
            parity=serial.PARITY_ODD,
            stopbits=1,
            dsrdtr=True,
            rtscts=True,
            xonxoff=False,
        )

        if not self.ser.is_open:
            print("Serial port failed to open")
            return
        
        self.start_time = time.time()
        self.frame_idx = 0
        self.num_skipped_frames = 0

    def get_measurement(self):
        frame_data = {}

        valid_frame = True
        # We expect zone idx 0 to come first - if not, bad frame and throw it out until we see
        # zone idx 0
        expected_zone_idx = 0
        while expected_zone_idx < self.num_zones:
            zone_idx, zone_data, length = VL53L8CHReader.readline_and_decode(self.ser)

            if zone_idx != expected_zone_idx:
                if self.verbose:
                    print(
                        f"Warning: Expected zone {expected_zone_idx} but received zone {zone_idx}"
                    )
                valid_frame = False
                # only add to the number of skipped frames if the expected zone idx is not zero.
                # if the expected zone idx is zero, most likely the previous line was also skipped,
                # so it's not a newly skipped frame, just waiting for zone idx zero to re-appear.
                if expected_zone_idx != 0:
                    self.num_skipped_frames += 1
                break

            frame_data[zone_idx] = zone_data
            expected_zone_idx += 1

        if valid_frame:
            self.frame_idx += 1
            frame_rate = self.frame_idx / (time.time() - self.start_time)
            print(
                f"Skipped frames: {self.num_skipped_frames} / {self.frame_idx} ({(self.num_skipped_frames / (self.num_skipped_frames + self.frame_idx)) * 100:.2f}%)"
            )
            print(f"Frame rate: {frame_rate:.2f} fps")

            if not self.depth_img_only:
                # update line plot data
                for zone_idx in frame_data.keys():
                    self.lines[zone_idx].setData(frame_data[zone_idx])

            # update image plot data
            depth_img = VL53L8CHReader.frame_to_depth_img(frame_data)
            # depth img just gives the bin index for each pixel, so we can set the min vis value 
            # to 0 and the max to the number of bins
            # self.image_item.setImage(depth_img, levels=(0, len(frame_data[0])))

    @classmethod
    def readline_and_decode(ser):
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
    
    @classmethod
    def frame_to_depth_img(frame):
        """
        Convert a frame of data from the sensor to a depth image (numpy array).

        Args:
            frame (dict): Dictionary where keys are zone indices and values are lists of floats
                representing the histogram values

        Returns:
            numpy.ndarray: Depth image where each pixel is the argmax of each histogram
        """

        num_zones = len(frame)

        if not np.sqrt(num_zones).is_integer():
            raise ValueError("Number of zones must be a perfect square")

        depth_img = np.zeros(num_zones)

        for zone_idx, zone_data in frame.items():
            depth_img[zone_idx] = np.argmax(zone_data)

        depth_img = depth_img.reshape(int(np.sqrt(num_zones)), int(np.sqrt(num_zones)))

        return depth_img

