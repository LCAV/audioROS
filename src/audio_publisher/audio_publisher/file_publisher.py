#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
file_publisher.py: Publish audio data from file
"""
import os
import sys

import numpy as np
from scipy.io.wavfile import read

import rclpy

from .publisher import AudioPublisher

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(current_dir + "/../../../crazyflie-audio/python/"))
import file_parser as fp

class FilePublisher(AudioPublisher):
    def __init__(
        self, file_source, loop=False, n_buffer=256, publish_rate=None, mic_fname=""
    ):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        mic_positions = None
        if mic_fname != "":
            mic_positions = np.load(mic_fname)
        super().__init__(
            "file_publisher",
            mic_positions=mic_positions,
            n_buffer=n_buffer,
            publish_rate=publish_rate,
            Fs=None,
        )

        self.loop = loop
        self.file_idx = 0

        if file_source == "analytical_source":
            self.signals_full, Fs = fp.read_simulation(file_source)
        elif file_source == "recordings_16_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_16_7_20("high", 0, "white_noise")
            Fs = fp.parameters[file_source]["Fs"]
            start_idx = fp.parameters[file_source]["time_index"]
            self.signals_full = signals_all[:, start_idx:]

        self.set_Fs(Fs)
        self.len = self.signals_full.shape[1]

        self.create_timer(1.0 / self.publish_rate, self.publish_loop)

    def publish_loop(self):
        n_buffer = self.n_buffer

        signals = self.signals_full[:, self.file_idx:self.file_idx + n_buffer]

        self.process_signals(signals)

        self.file_idx += self.n_between_buffers
        if self.file_idx + n_buffer >= self.len:
            if self.loop:
                self.file_idx = 0
            else:
                sys.exit()


def main(args=None):
    import os

    rclpy.init(args=args)

    Fs = 44100
    n_buffer = 2 ** 10
    publish_rate = int(Fs / n_buffer)  # 11 # in Hz

    print(f"Publishing audio data from file at {publish_rate}Hz.")

    mic_fname = os.path.abspath(current_dir + "/../../../crazyflie-audio/data/simulated/analytical_mics.npy")

    file_source = "analytical_source"
    file_source = "recordings_16_7_20"
        
    publisher = FilePublisher(
        file_source,
        n_buffer=n_buffer,
        publish_rate=publish_rate,
        loop=True,
        mic_fname=mic_fname,
    )
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
