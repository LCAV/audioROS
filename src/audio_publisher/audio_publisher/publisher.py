# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
audio_publisher.py: 
"""

import time

import numpy as np
from scipy.io.wavfile import write

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from audio_interfaces.msg import Signals

N_MICS = 4

MAX_TIMESTAMP_INT = 2 ** 32 - 1  # needs to match Spectrum.msg and Correlation.msg

# number of buffers to save, e.g. 256*1000=256'000 at 32000Hz: 8seconds
MAX_BUFFERS = 0  # set to 0 for no saving

OUT_DIR = "debug"


class AudioPublisher(Node):
    def __init__(
        self,
        name="audio_publisher",
        n_buffer=256,
        publish_rate=None,
        Fs=None,
        mic_positions=None,
    ):
        super().__init__(name)

        if publish_rate is None:
            # by default, adapt publish rate to n_buffer (no samples skipped)
            self.publish_rate = int(Fs / n_buffer)
        else:
            self.publish_rate = publish_rate
        self.n_buffer = n_buffer

        if Fs is not None:
            self.set_Fs(Fs)

        self.mic_positions = mic_positions

        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)

        self.time_idx = 0
        self.raw_data = np.empty((N_MICS, n_buffer * MAX_BUFFERS))

        # create ROS parameters that can be changed from command line.
        self.declare_parameter("n_buffer")
        self.declare_parameter("publish_rate")
        parameters = [
            rclpy.parameter.Parameter(
                "n_buffer", rclpy.Parameter.Type.STRING, str(self.n_buffer)
            ),
            rclpy.parameter.Parameter(
                "publish_rate", rclpy.Parameter.Type.STRING, str(self.publish_rate)
            ),
        ]
        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)

    def set_params(self, params):
        for param in params:
            old_value = (
                self.get_parameter(param.name).get_parameter_value().string_value
            )
            str_value = param.get_parameter_value().string_value
            int_value = param.get_parameter_value().integer_value
            new_value = eval(str_value) if str_value != "" else int_value

            self.get_logger().info(
                f"changing {param.name} from {old_value} to {new_value}"
            )
            if param.name == "n_buffer":
                self.n_buffer = new_value
                self.raw_data = np.empty((N_MICS, self.n_buffer * MAX_BUFFERS))
            elif param.name == "publish_rate":
                self.publish_rate = new_value
        return SetParametersResult(successful=True)

    def set_Fs(self, Fs):
        self.Fs = Fs
        self.n_between_buffers = self.Fs // self.publish_rate

    def process_signals(self, signals):
        n_buffer = signals.shape[1]

        t1 = time.time()

        assert self.Fs is not None, "Need to set Fs before processing."
        assert signals.shape[0] == N_MICS
        if self.mic_positions is None:
            self.get_logger.warn("Did not set mic_positions.")

        # publishing
        msg = Signals()
        msg.timestamp = self.time_idx
        msg.fs = self.Fs
        msg.n_mics = N_MICS
        msg.n_buffer = n_buffer
        msg.signals_vect = list(signals.flatten().astype(float))
        if self.mic_positions is not None:
            msg.mic_positions = list(self.mic_positions.flatten().astype(float))
        else:
            msg.mic_positions = []
        self.publisher_signals.publish(msg)

        if self.time_idx < MAX_BUFFERS:
            self.raw_data[
                :, self.time_idx * n_buffer : (self.time_idx + 1) * n_buffer
            ] = signals
        elif (self.time_idx == MAX_BUFFERS) and (MAX_BUFFERS > 0):
            for i in range(self.raw_data.shape[0]):
                fname = f"{OUT_DIR}/test_mic{i}.wav"
                write(fname, self.Fs, self.raw_data[i])
                self.get_logger().info(f"Saved audio as {fname}")

        self.time_idx += 1
        if self.time_idx >= MAX_TIMESTAMP_INT:
            self.get_logger().error("timestamp overflow.")
            self.time_idx = self.time_idx % MAX_TIMESTAMP_INT

        t2 = time.time()
        processing_time = t2 - t1
        if processing_time > 1.0 / self.publish_rate:
            self.get_logger().warn(
                f"processing time ({processing_time:.1e}) longer than publishing period ({1/self.publish_rate:.1e})"
            )
