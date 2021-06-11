import logging
import os
import struct
import sys
import time
import rclpy
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import serial

from epuck_description_py.parameters import (
    MIC_POSITIONS,
    N_BUFFER,
    FS,
    N_MICS,
    WHEEL_DIAMETER,
)
from audio_interfaces_py.messages import create_signals_freq_message
from audio_interfaces.msg import SignalsFreq
from audio_interfaces_py.node_with_params import NodeWithParams

""" Notes on Bluetooth:
press USER button while turning on the robot with ON/OFF button (little black button above buzzer)
find MAC_ADDR from bluetooth menu
sudo rfcomm bind /dev/rfcomm0 MAC_ADDR 2
https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development
PORT = /dev/rfcomm0
"""

PORT = "/dev/ttyACM2"
BAUDRATE = 115200

def live_status_function(show_status, bins, data):
    if show_status:
        print("the data is", data)
        print("the bins are", bins)

class Gateway(NodeWithParams):
    PARAMS_DICT = {
        "buzzer_idx": 0,
        "move_forward": 0,
        "stop": 0,
    }

    def __init__(self):
        self.desired_rate = 1000  # Hz
        self.start_time = time.time()

        super().__init__("gateway")

        # need the reader from the epuck initalized here
        self.get_logger().info(f"initiating connection to port {PORT}")
        try:
            self.port = serial.Serial(PORT, BAUDRATE, timeout=0.5)
            self.get_logger().info(f"connected at baudrate {self.port.baudrate}")
        except Exception as e:
            print(e)
            self.port = None
            self.get_logger().warn("could not successfully connect to the epuck")
            #sys.exit(0)


        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.create_timer(1 / self.desired_rate, self.publish_current_data)

        #if self.port is not None:
        #    self.port.write(b"BUZZR")
        #    time.sleep(8)

    def publish_current_data(self):
        # only the first one is for my publisher
        # if not self.reader_crtp.audio_dict["published"]:
        self.publish_audio_dict()

        # ASK do I still update this because her code is looking for this to launch calcs?
        # self.reader_crtp.audio_dict["published"] = True

    def publish_audio_dict(self):
        # reader_crtp is the bluetooth reader for the crazyflie
        # read audio
        # the format is all the interleaved real values of all four microphones

        try:
            data, size, timestamp = self.read_float_serial()
        except:
            return

        if data is None:
            self.get_logger().warn("Empty audio. Not publishing")
            return

        # extracting all our values from the epuck serial package
        position = 0
        bin_number, position = self.extract_bin_number(data, position)

        # undo the interleaving and create separate matrixes for each microphone
        signals_f, position = self.data_rearrange(data, position, bin_number)
        fbins, *_ = self.extract_bins(data, position, bin_number)

        # read frequencies

        if fbins is None:
            self.get_logger().warn("No data yet. Not publishing")
            return

        all_frequencies = np.fft.fftfreq(n=N_BUFFER, d=1/FS)
        n_frequencies = len(fbins)

        if not np.any(fbins > 0):
            self.get_logger().warn(f"Empty fbins!")
            return
        elif np.any(fbins >= len(all_frequencies)):
            self.get_logger().warn(
                f"Invalid fbins! {fbins[fbins > len(all_frequencies)]}"
            )
            return

        frequencies = all_frequencies[fbins]
        abs_signals_f = np.abs(signals_f)
        mic_positions_arr = np.array(MIC_POSITIONS)
        msg = create_signals_freq_message(
            signals_f.T,
            frequencies,
            mic_positions_arr,
            int(time.time()),
            int(timestamp),
            FS,
        )

        # switch the status to true to receive information on relevant data
        live_status_function(False, fbins, signals_f)

        self.publisher_signals.publish(msg)

        self.get_logger().info(
            f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}"
        )

    # reads the FFT in float32 from the serial
    def read_float_serial(self):
        state = 0

        while state != 5:
            # reads 1 byte
            c1 = self.port.read(1)
            # timeout condition
            if c1 == b"":
                print("Timeout...")
                return []

            if state == 0:
                if c1 == b"S":
                    state = 1
                else:
                    state = 0
            elif state == 1:
                if c1 == b"T":
                    state = 2
                elif c1 == b"S":
                    state = 1
                else:
                    state = 0
            elif state == 2:
                if c1 == b"A":
                    state = 3
                elif c1 == b"S":
                    state = 1
                else:
                    state = 0
            elif state == 3:
                if c1 == b"R":
                    state = 4
                elif c1 == b"S":
                    state = 1
                else:
                    state = 0
            elif state == 4:
                if c1 == b"T":
                    state = 5
                elif c1 == b"S":
                    state = 1
                else:
                    state = 0

        # normally this should unpack as an unsigned in of size 32 bits

        # reads the size
        # converts as short int in little endian the two bytes read
        size = struct.unpack("<h", self.port.read(2))
        timestamp = struct.unpack("<I", self.port.read(4))
        timestamp = timestamp[0]
        size = size[0]

        # reads the data
        rcv_buffer = self.port.read(size * 4)
        data = []
        # if we receive the good amount of data, we convert them in float32
        # send the data in uint16 for the rest
        if len(rcv_buffer) == 4 * size:
            i = 0
            while i < size:
                data.append(struct.unpack_from("<f", rcv_buffer, i * 4)[0])
                i = i + 1
            return data, size, timestamp
        else:
            print("wrong buffer size")
            return None

    # function to rearange the interleaving of the epuck to the actual interleaving we want
    def data_rearrange(self, data, position, bin_number):
        mics = N_MICS
        # check that there is the right number of bins :

        if data is []:
            print("there is nothing in the data")
            return data

        signals_f = np.zeros((mics, bin_number), dtype=np.complex128)

        pos = 0
        for i in range(bin_number):
            for j in range(mics):
                pos = (j * bin_number + i) * 2 + position
                signals_f.real[j, i] = data[pos]
                signals_f.imag[j, i] = data[pos + 1]

        return signals_f, pos + 2

    def extract_bins(self, data, bin_pos, num_bins):
        bins = np.zeros(num_bins, np.int)
        for i in range(0, num_bins):
            bins[i] = int(data[i + bin_pos])

        return bins, bin_pos + num_bins

    def extract_timestamp(self, data, position):
        return data[position], position + 1

    def extract_bin_number(self, data, position):
        return int(data[position]), position + 1

    def send_desired_rate_serial(self):
        self.port.write(b"DRATE")
        self.port.write(struct.pack("<h", self.desired_rate))
        print("sent desired rate to robot", self.desired_rate)

    def move_forward(self, velocity_m):
        # convert velocity to epuck command.
        speed = int(round(velocity_m * 1000 / (100 * WHEEL_DIAMETER * np.pi)))

        if not (0 <= speed <= 10):
            self.get_logger().warn(f"invalid velocity: {velocity_m, speed}")

        self.port.write(b"M")
        #self.port.write(struct.pack("<h", 1))
        #self.port.write(struct.pack("<h", speed))
        self.get_logger().info(f"sent move rate {speed} to robot")

    def move_slight_right(self, *args):
        self.port.write(b"M")
        self.port.write(struct.pack("<h", 4))
        print("sent move rate to robot")

    def stop(self, *args):
        self.port.write(b"S")
        print("stopped the robot from moving with motor")

    def send_buzzer_idx(self, *args):
        if self.port is None:
            self.get_logger().warn("cannot send buzzer index")
            return 
        port = self.port 
        port.write(b'B')
        print("sent buzzer start command")

    def set_params(self, params):
        """ Overwrite the function set_params by NodeWithParams. 
        We need this so we commute new parameters directly to the Crazyflie drone.
        """
        current_params = {param.name: param.value for param in params}
        for param_name, param_value in current_params.items():
            # move by given angle, blocking
            if param_name == "turn_angle":
                if param_value not in [0, None]:
                    self.turn_angle(param_value)
            # move by given velocity, non-blocking
            elif param_name == "move_forward":
                if param_value not in [0, None]:
                    self.move_forward(param_value)
            elif param_name == "stop":
                if param_value not in [0, None]:
                    self.stop(param_value)
            # send buzzer commands
            elif param_name == "buzzer_idx":
                if param_value not in [0, None]:
                    self.send_buzzer_idx(param_value)
            else:
                self.get_logger().warn(f"setting unused parameter {param_name}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    gateway = Gateway()

    rclpy.spin(gateway)
    gateway.destroy_node()
    rclpy.shutdown()
