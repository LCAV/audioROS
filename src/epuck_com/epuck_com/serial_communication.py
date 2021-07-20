import logging
import os
import struct
import sys
import time
from datetime import datetime
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


def read_start(port):
    state = 0
    while state != 5:
        # reads 1 byte
        try:
            c1 = port.read(1)
        except KeyboardInterrupt:
            print("interrupt during read")
            return None

        # timeout condition
        if c1 == b"":
            print("Timeout")
            return False
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
    return True


def read_end(port):
    state = 0
    while state != 3:
        # reads 1 byte
        try:
            c1 = port.read(1)
        except KeyboardInterrupt:
            print("interrupt during read")
            return None

        # timeout condition
        if c1 == b"":
            print("Timeout")
            return False
        if state == 0:
            if c1 == b"E":
                state = 1
            else:
                state = 0
        elif state == 1:
            if c1 == b"N":
                state = 2
            elif c1 == b"E":
                state = 1
            else:
                state = 0
        elif state == 2:
            if c1 == b"D":
                state = 3
            elif c1 == b"E":
                state = 1
            else:
                state = 0
    return True


def live_status_function(show_status, bins, data):
    if show_status:
        print("the bins are", bins)


class Gateway(NodeWithParams):
    PARAMS_DICT = {
        "buzzer_idx": 0,
        "move_forward": 0,
        "stop": 0,
    }

    def __init__(self, port):
        self.desired_rate = 1000  # Hz
        self.start_time = time.time()

        super().__init__("gateway")

        # need the reader from the epuck initalized here
        self.port = port

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.create_timer(1 / self.desired_rate, self.publish_current_data)

        self.buzzer_idx = 0

        # if self.port is not None:
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

        if self.buzzer_idx == 0:
            self.send_stop()
            return

        read_data = self.read_float_serial()
        if read_data is not None:
            data, size, timestamp = read_data
        else:
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

        all_frequencies = np.fft.fftfreq(n=N_BUFFER, d=1 / FS)
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
        live_status_function(True, fbins, signals_f)

        self.publisher_signals.publish(msg)

        self.get_logger().info(
            f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}"
        )


    # reads the FFT in float32 from the serial
    def read_float_serial(self):
        start_detected = read_start(self.port)
        if start_detected is None:
            self.port.close()
            self.destroy_node()
        elif not start_detected:
            print(f"timeout in read_float_serial")
            return None
        
        # normally this should unpack as an unsigned in of size 32 bits
        # reads the size
        # converts as short int in little endian the two bytes read
        res = self.port.read(2)
        if(len(res) > 0):
            size = struct.unpack("<H", res)[0]  # H for unsigned short
            timestamp = struct.unpack("<I", self.port.read(4))[0]  # I for unsigned int
        else:
            print(f'size of data not recieved')
            return None
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
            self.send_acknowledge()
            return data, size, timestamp
        else:
            print(f"wrong buffer size, recieved only {len(rcv_buffer)}")
            self.send_non_acknowledge()
            return None

        if not read_end(self.port):
            self.send_non_acknowledge()
        else:
            self.send_acknowledge()



    def send_non_acknowledge(self):
        self.port.write(b"n")

    def send_acknowledge(self):
        self.port.write(b"a")

    def send_stop(self):
        self.port.write(b"x")

    def send_start(self):
        self.port.write(b"s")

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
                signals_f.real[j, i] = data[pos]
                pos += 1
            for j in range(mics):
                signals_f.imag[j, i] = data[pos]
                pos += 1
        return signals_f, pos + 1

    def extract_bins(self, data, bin_pos, num_bins):
        bins = np.zeros(num_bins, np.int)
        for i in range(0, num_bins):
            bins[i] = int(data[i + bin_pos])
        return bins, bin_pos + num_bins

    def extract_timestamp(self, data, position):
        return data[position], position + 1

    def extract_bin_number(self, data, position):
        return int(data[position]), position + 1

    def move_forward(self, velocity_m):
        # convert velocity to epuck command.
        speed = int(round(velocity_m * 1000 / (100 * WHEEL_DIAMETER * np.pi)))

        if not (0 <= speed <= 10):
            self.get_logger().warn(f"invalid velocity: {velocity_m, speed}")

        self.port.write(b"M")
        # self.port.write(struct.pack("<h", 1))
        # self.port.write(struct.pack("<h", speed))
        self.get_logger().info(f"sent move rate {speed} to robot")

    def move_slight_right(self, *args):
        self.port.write(b"M")
        self.port.write(struct.pack("<h", 4))
        print("sent move rate to robot")

    def send_buzzer_idx(self, buzzer_idx, *args):
        self.buzzer_idx = buzzer_idx

        if self.port is None:
            self.get_logger().warn("cannot send buzzer index")
            return

        if buzzer_idx > 0:
            self.send_start()
            print("sent buzzer start command")
        else:
            self.send_stop()
            print("sent buzzer stop command")

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
                if param_value not in [None]:
                    self.send_buzzer_idx(param_value)
            else:
                self.get_logger().warn(f"setting unused parameter {param_name}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)

    PORT = "/dev/ttyACM"
    BAUDRATE = 115200

    port = None
    for i in range(10):
        port_id = PORT + str(i)
        try:
            port = serial.Serial(port_id, BAUDRATE, timeout=0.5)
            print(f"connected to {port_id} at baudrate {port.baudrate}")

            port.write(b"w")
            detected = read_start(port)
            if detected:
                print("detected start, ok")
                break
            else:
                print("did not detect start")
        except:
            print(f"could not connect to {port_id}")

    if port is None:
        print("could not successfully connect to the epuck")
        sys.exit(0)

    gateway = Gateway(port)

    rclpy.spin(gateway)

    port.close()
    gateway.destroy_node()
    rclpy.shutdown()
