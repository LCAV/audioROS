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
        self, file_source, Fs, loop=False, n_buffer=256, publish_rate=None, mic_positions=None
    ):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        super().__init__(
            "file_publisher",
            mic_positions=mic_positions,
            n_buffer=n_buffer,
            publish_rate=publish_rate,
            Fs=Fs
        )
        self.loop = loop
        self.read_file_source(file_source)
        self.file_idx = 0
        self.create_timer(1.0 / self.publish_rate, self.publish_loop)

    def read_file_source(self, file_source):
        if file_source in ["analytical", "pyroomacoustics"]:
            signals_props, signals_source, signals_all = fp.read_simulation(file_source)
        elif file_source == "recordings_16_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_16_7_20(
                    loudness="high", 
                    gt_degrees=0, 
                    source="white_noise")
        elif file_source == "recordings_14_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_14_7_20(gt_degrees=0)
        elif file_source == "recordings_9_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_9_7_20(gt_degrees=0)

        signals_full = signals_all
        start_idx = fp.parameters[file_source]["time_index"]
        self.signals_full = signals_full[:, start_idx:]
        self.len = self.signals_full.shape[1]

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
    rclpy.init(args=args)

    n_buffer = 2 ** 10

    file_source = "analytical"
    #file_source = "pyroomacoustics"
    #file_source = "recordings_9_7_20"
    #file_source = "recordings_14_7_20"
    #file_source = "recordings_16_7_20"

    Fs = fp.parameters[file_source]["Fs"]
    mic_positions = fp.parameters[file_source]["mic_positions"]

    publish_rate = int(Fs / n_buffer)  # 11 # in Hz
    print(f"Publishing audio data from file at {publish_rate}Hz.")
        
    publisher = FilePublisher(
        file_source,
        Fs=Fs,
        n_buffer=n_buffer,
        publish_rate=publish_rate,
        loop=True,
        mic_positions=mic_positions,
    )
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
