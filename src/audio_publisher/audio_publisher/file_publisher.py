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
from audio_interfaces.msg import PoseRaw

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(current_dir + "/../../../crazyflie-audio/python/"))
import file_parser as fp

GT_DEGREES = 20
LOUDNESS = "high"
SOURCE = "white_noise"

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
        self.create_timer(1.0 / self.publish_rate, self.publish_signals)

        self.publisher_motion_pose_raw = self.create_publisher(
            PoseRaw, "geometry/pose_raw", 10
        )

    def read_file_source(self, file_source):
        if file_source in ["analytical", "pyroomacoustics"]:
            signals_props, signals_source, signals_all = fp.read_simulation(file_source)
        elif file_source == "recordings_16_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_16_7_20(
                    loudness=LOUDNESS, 
                    gt_degrees=GT_DEGREES, 
                    source=SOURCE)
        elif file_source == "recordings_14_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_14_7_20(gt_degrees=GT_DEGREES)
        elif file_source == "recordings_9_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_9_7_20(gt_degrees=GT_DEGREES)

        signals_full = signals_all
        start_idx = fp.parameters[file_source]["time_index"]
        self.signals_full = signals_full[:, start_idx:]
        self.len = self.signals_full.shape[1]

    def publish_signals(self):
        n_buffer = self.n_buffer

        signals = self.signals_full[:, self.file_idx:self.file_idx + n_buffer]

        self.process_signals(signals)
        self.publish_position()

        self.file_idx += self.n_between_buffers
        if self.file_idx + n_buffer >= self.len:
            if self.loop:
                self.file_idx = 0
            else:
                sys.exit()

    def publish_position(self):
        msg_pose_raw = PoseRaw()
        msg_pose_raw.dx = 0.0
        msg_pose_raw.dy = 0.0
        msg_pose_raw.z = 0.0
        msg_pose_raw.yaw_deg = 0.0
        msg_pose_raw.source_direction_deg = 90.0 + GT_DEGREES
        msg_pose_raw.timestamp = self.get_time_ms()
        self.publisher_motion_pose_raw.publish(msg_pose_raw)


def main(args=None):
    rclpy.init(args=args)

    n_buffer = 2 ** 10

    #file_source = "analytical"
    #file_source = "pyroomacoustics"
    #file_source = "recordings_9_7_20"
    #file_source = "recordings_14_7_20"
    file_source = "recordings_16_7_20"

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
