import logging
import os
import struct
import sys
import time
import rclpy
from rclpy.node import Node

import numpy as np
import serial

# from build.audio_interfaces.rosidl_generator_py.audio_interfaces.msg._signals_freq import SignalsFreq
from epuck_description_py.parameters import MIC_POSITIONS, N_BUFFER, FS, N_MICS
from audio_interfaces_py.messages import create_signals_freq_message
from audio_interfaces.msg._signals_freq import SignalsFreq




class Gateway(Node):
    def __init__(self):
        super().__init__("gateway",
                         automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)
        self.start_time = time.time()

        self.publisher_signals = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )

        # need the reader from the epuck initalized here

        self.port = "/dev/ttyACM1"

        print("initiating connection to port", self.port)
        try:
            self.port = serial.Serial(self.port, timeout = 0.5)
        except:
            print("could not successfully connect to the epuck")
            sys.exit(0)


        self.desired_rate = 1000  # Hz
        self.create_timer(1 / self.desired_rate, self.publish_current_data)

    def publish_current_data(self):
        # only the first one is for my publisher
        # if not self.reader_crtp.audio_dict["published"]:
        self.publish_audio_dict()

        # ASK do I still update this because her code is looking for this to launch calcs?
        self.reader_crtp.audio_dict["published"] = True

    def publish_audio_dict(self):
        # reader_crtp is the bluetooth reader for the crazyflie
        # read audio

        # TODO : figure out how to read the audio data from my protocol, and then that data should go in there
        # the format is all the interleaved real values of all four microphones and then all the complex values of all the microphones
        # needs to replace self.reader_crtp
        data_signals, size = self.read_float_serial()

        # signals_f_vect = self.reader_crtp.audio_dict["signals_f_vect"]

        if data_signals is None:
            self.get_logger().warn("Empty audio. Not publishing")
            return
        fbins = np.arange(32)
        # read frequencies

        if fbins is None:
            self.get_logger().warn("No data yet. Not publishing")
            return

        all_frequencies = np.fft.rfftfreq(n=N_BUFFER, d=1 / FS)
        n_frequencies = len(fbins)

        # the only allowed duplicates are 0
        # if len(set(fbins[fbins>0])) < len(fbins[fbins>0]):
        # self.get_logger().warn(f"Duplicate values in fbins! unique values:{len(set(fbins))}")
        # return
        if not np.any(fbins > 0):
            self.get_logger().warn(f"Empty fbins!")
            return
        elif np.any(fbins >= len(all_frequencies)):
            self.get_logger().warn(f"Invalid fbins! {fbins[fbins > len(all_frequencies)]}")
            return

        frequencies = all_frequencies[fbins]

        # undo the interleaving and create separate matrixes for each microphone
        signals_f = self.data_rearrange(data_signals)

        abs_signals_f = np.abs(signals_f)
        if np.any(abs_signals_f > N_BUFFER):
            self.get_logger().warn("Possibly in valid audio:")
            self.get_logger().warn(f"mic 0 {abs_signals_f[0, :5]}")
            self.get_logger().warn(f"mic 1 {abs_signals_f[1, :5]}")

        mic_positions_arr = np.array(MIC_POSITIONS)
        msg = create_signals_freq_message(signals_f.T, frequencies, mic_positions_arr,
                                          self.reader_crtp.audio_dict["timestamp"],
                                          self.reader_crtp.audio_dict["audio_timestamp"], FS)
        self.publisher_signals.publish(msg)

        self.get_logger().info(
            f"{msg.timestamp}: Published audio data with fbins {fbins[[0, 1, 2, -1]]} and timestamp {msg.audio_timestamp}")

    # reads the FFT in float32 from the serial
    def read_float_serial(self):
        state = 0

        while (state != 5):
            # reads 1 byte
            c1 = self.port.read(1)
            # timeout condition
            if (c1 == b''):
                print('Timout...')
                return []

            if (state == 0):
                if (c1 == b'S'):
                    state = 1
                else:
                    state = 0
            elif (state == 1):
                if (c1 == b'T'):
                    state = 2
                elif (c1 == b'S'):
                    state = 1
                else:
                    state = 0
            elif (state == 2):
                if (c1 == b'A'):
                    state = 3
                elif (c1 == b'S'):
                    state = 1
                else:
                    state = 0
            elif (state == 3):
                if (c1 == b'R'):
                    state = 4
                elif c1 == b'S':
                    state = 1
                else:
                    state = 0
            elif (state == 4):
                if (c1 == b'T'):
                    state = 5
                elif (c1 == b'S'):
                    state = 1
                else:
                    state = 0

        # reads the size
        # converts as short int in little endian the two bytes read
        size = struct.unpack('<h', self.port.read(2))

        size = size[0]
        # reads the data
        rcv_buffer = self.port.read(size * 4)
        data = []
        # if we receive the good amount of data, we convert them in float32
        # send the data in uint16 for the rest
        if (len(rcv_buffer) == 4 * size):
            i = 0
            while (i < size):
                data.append(struct.unpack_from('<f', rcv_buffer, i * 4))
                i = i + 1
            return data, size
        else:
            return None

    # function to rearange the interleaving of the epuck to the actual interleaving we want
    def data_rearrange(self, data):
        mics = 4
        bins = 32
        # check that there is the right number of bins :

        if data is []:
            print("there is nothing in the data")
            return data
        smaller_data = []
        # filtering the bins for now centered around a fixed value
        for i in range(32):
            smaller_data.append(data[i])
        if (len(smaller_data) != bins):
            print("the size of the data is", len(data), "while it should be", bins, "stopping program")
            sys.exit(0)

        signals_f = np.zeros((mics, bins), dtype=np.complex128)

        for i in range(len(data)):
            print("Access element is: ", data[i])
        for i in range(bins):
            for j in range(mics):
                pos = (j * bins + i) * 2
                signals_f.real[j, i] = data[pos]
                signals_f.imag[j, i] = data[pos + 1]

        return signals_f




def main(args=None):
    rclpy.init(args=args)

    gateway = Gateway()

    rclpy.spin(gateway)
    gateway.destroy_node()
    rclpy.shutdown()