import numpy as np
import serial
import time
import struct
import numpy as np

#Dont do distances from VL sensor
# Return dists as empty, VL sensor does not compute distance
# Eventually setup launch file

"""
            if len(d) == 78 and d[0] == "#Obj":
                result = {}
                result["I2C_address"] = None
                result["measurement_num"] = int(d[2])
                result["temperature"] = None
                result["num_valid_results"] = int(d[4])
                result["tick"] = int(d[5])
                result["depths_1"] = [
                    int(x) for x in [d[6], d[8], d[10], d[12], d[14], d[16], d[18], d[20], d[22]]
                ]
                result["confs_1"] = [
                    int(x) for x in [d[7], d[9], d[11], d[13], d[15], d[17], d[19], d[21], d[23]]
                ]
                # 18 that go in between here are unused, at least in 3x3 mode
                result["depths_2"] = [
                    int(x) for x in [d[42], d[44], d[46], d[48], d[50], d[52], d[54], d[56], d[58]]
                ]
                result["confs_2"] = [
                    int(x) for x in [d[43], d[45], d[47], d[49], d[51], d[53], d[55], d[57], d[59]]
                ]

"""

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
        self.num_zones = 64

    def get_measurement(self):
        frame_data = {}

        valid_frame = True
        # We expect zone idx 0 to come first - if not, bad frame and throw it out until we see
        # zone idx 0
        expected_zone_idx = 0
        while expected_zone_idx < self.num_zones:
            # print(self.ser)
            zone_idx, zone_data, _ = VL53L8CHReader.readline_and_decode(self.ser)

            if zone_idx != expected_zone_idx:
                # if self.verbose:
                #     print(
                #         f"Warning: Expected zone {expected_zone_idx} but received zone {zone_idx}"
                #     )
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
            # self.frame_idx += 1
            ## Will not need frame rate
            # frame_rate = self.frame_idx / (time.time() - self.start_time)
            # print(
            #     f"Skipped frames: {self.num_skipped_frames} / {self.frame_idx} ({(self.num_skipped_frames / (self.num_skipped_frames + self.frame_idx)) * 100:.2f}%)"
            # )
            # print(f"Frame rate: {frame_rate:.2f} fps")

            # if not self.depth_img_only:
            #     # update line plot data
            #     for zone_idx in frame_data.keys():
            #         self.lines[zone_idx].setData(frame_data[zone_idx])

            # update image plot data  
            # depth_img = VL53L8CHReader.frame_to_depth_img(frame_data)
            # depth img just gives the bin index for each pixel, so we can set the min vis value 
            # to 0 and the max to the number of bins
            # self.image_item.setImage(depth_img, levels=(0, len(frame_data[0])))
            # # Adding another array around frame_data because of Tofpublisher

            # Turn dictionary into lists of list
            # TODO: DOuble check that order is preserved
            frame_data = [frame_data[zone_idx] for zone_idx in sorted(frame_data.keys())]
            return frame_data, time.time()
        return None
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
        # print(f"TYPE = {type(ser)}")
        c = ser.read_until(expected=eol)
        byte_listing = bytearray(c)
        length = len(byte_listing) - len(byte_listing) % 4
        line_as_floats = [
            struct.unpack("<f", bytes(byte_listing[i : i + 4]))[0] for i in range(0, length, 4)
        ]

        # line_as_floats[-1] is always NaN, so remove it
        # TODO figure out why this is and if we're missing out on data by doing this
        return int(line_as_floats[0]), line_as_floats[1:-1], length