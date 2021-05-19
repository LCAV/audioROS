#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
file_publisher.py: Publish audio data from file
"""
import os
import sys

from scipy.io.wavfile import read

import rclpy

from audio_publisher.publisher import AudioPublisher
from audio_interfaces.msg import PoseRaw

N_BUFFER = 2048

# for older datsets only.
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.abspath(current_dir + "/../../../crazyflie-audio/python/"))
import file_parser as fp

GT_DEGREES = 20
LOUDNESS = "high"
SOURCE = "white_noise"


def read_recordings(exp_name, appendix=""):
    if exp_name == "2021_01_07_snr_study":
        fname = (
            f"experiments/{exp_name}/export/motors_nosnr_noprops_mono1750{appendix}.wav"
        )
    else:
        raise ValueError(exp_name)
    fs_here, signals = read(fname)

    if signals.ndim == 1:
        signals = signals.reshape((1, -1))
    return signals


class FilePublisher(AudioPublisher):
    def __init__(self, file_source, loop=False, publish_rate=None, mic_positions=None):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        PARAMS_DICT = AudioPublisher.PARAMS_DICT
        PARAMS_DICT["appendix"] = ""

        # for old datasets only
        if file_source in fp.parameters.keys():
            Fs = fp.parameters[file_source]["Fs"]
        else:
            sys.path.append(f"experiments/{file_source}/")
            from params import global_params

            Fs = global_params["fs_soundcard"]

        super().__init__(
            "file_publisher",
            mic_positions=mic_positions,
            publish_rate=publish_rate,
            Fs=Fs,
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
                loudness=LOUDNESS, gt_degrees=GT_DEGREES, source=SOURCE
            )
        elif file_source == "recordings_14_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_14_7_20(
                gt_degrees=GT_DEGREES
            )
        elif file_source == "recordings_9_7_20":
            signals_props, signals_source, signals_all = fp.read_recordings_9_7_20(
                gt_degrees=GT_DEGREES
            )
        elif file_source == "2021_01_07_snr_study":
            signals_all = read_recordings(
                file_source, appendix=self.current_params["appendix"]
            )
        else:
            raise ValueError(file_source)

        signals_full = signals_all
        if file_source in fp.parameters.keys():
            start_idx = fp.parameters[file_source]["time_index"]
        else:
            start_idx = 0
        self.signals_full = signals_full[:, start_idx:]
        self.len = self.signals_full.shape[1]

    def publish_signals(self):
        n_buffer = self.current_params["n_buffer"]

        signals = self.signals_full[:, self.file_idx : self.file_idx + n_buffer]
        self.process_signals(signals)
        self.publish_position()

        self.file_idx += n_buffer
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
        msg_pose_raw.source_direction_deg = 90.0 - GT_DEGREES
        msg_pose_raw.timestamp = self.get_time_ms()
        self.publisher_motion_pose_raw.publish(msg_pose_raw)


def main(args=None):
    rclpy.init(args=args)

    # file_source = "analytical"
    # file_source = "pyroomacoustics"
    # file_source = "recordings_9_7_20"
    # file_source = "recordings_14_7_20"
    # file_source = "recordings_16_7_20"
    file_source = "2021_01_07_snr_study"

    publisher = FilePublisher(file_source, loop=True)
    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
