#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
file_publisher.py: Publish audio data from file
"""
import sys

import numpy as np
from scipy.io.wavfile import read

import rclpy

from .publisher import AudioPublisher


class FilePublisher(AudioPublisher):
    def __init__(
        self, filenames=None, loop=False, n_buffer=256, publish_rate=None, mic_fname=""
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

        # audio file preparation
        self.loop = loop
        self.file_idx = 0
        self.audio_data = {f: {} for f in filenames}
        for fname in filenames:
            self.audio_data[fname]["Fs"], self.audio_data[fname]["data"] = read(fname)

        # read Fs and make sure it matches all files
        Fs = self.audio_data[filenames[0]]["Fs"]
        assert all(self.audio_data[f]["Fs"] == Fs for f in filenames)
        self.set_Fs(Fs)

        # read len and make sure it matches all files
        self.len = len(self.audio_data[filenames[0]]["data"])
        assert all(len(self.audio_data[f]["data"]) == self.len for f in filenames)

        self.create_timer(1.0 / self.publish_rate, self.publish_loop)

    def publish_loop(self):
        n_buffer = self.n_buffer

        signals = np.c_[
            [
                data["data"][self.file_idx : self.file_idx + n_buffer]
                for data in self.audio_data.values()
            ]
        ]  # n_mics x n_samples

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

    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.abspath(current_dir + "/../../../crazyflie-audio/data/simulated")
    fnames = [
        os.path.join(data_dir, f"analytical_source_mic{i}.wav") for i in range(1, 5)
    ]
    mic_fname = os.path.join(data_dir, "analytical_mics.npy")

    publisher = FilePublisher(
        fnames,
        n_buffer=n_buffer,
        publish_rate=publish_rate,
        loop=True,
        mic_fname=mic_fname,
    )
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
