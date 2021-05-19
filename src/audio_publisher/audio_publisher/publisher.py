#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
publisher.py: General publisher base class.
"""

import time

import numpy as np
from scipy.io.wavfile import write


from audio_interfaces.msg import Signals
from audio_interfaces_py.messages import create_signals_message
from audio_interfaces_py.node_with_params import NodeWithParams

MAX_UINT32 = 2 ** 32 - 1
N_BUFFER = 2048

# number of buffers to save, for example 5 seconds
SAVE_BUFFERS = int(5 * 44100 / 2048)  # set to 0 for no saving
SAVE_DIR = "debug"


class AudioPublisher(NodeWithParams):
    PARAMS_DICT = {"n_buffer": N_BUFFER}

    def __init__(
        self, name="audio_publisher", publish_rate=None, Fs=None, mic_positions=None,
    ):
        super().__init__(name)

        if publish_rate is None:
            # by default, adapt publish rate to n_buffer (no samples skipped)
            self.publish_rate = Fs / self.current_params["n_buffer"]
        else:
            self.publish_rate = publish_rate

        self.Fs = Fs
        self.mic_positions = mic_positions
        self.n_mics = None  # will be set later
        if mic_positions is not None:
            self.n_mics = mic_positions.shape[0]

        self.publisher_signals = self.create_publisher(Signals, "audio/signals", 10)

        # for debugging only
        self.save_idx = 0
        self.save_data = None  # will be set later

        self.start_time = time.time()

    def get_time_ms(self):
        timestamp = int(1000 * (time.time() - self.start_time))
        if timestamp >= MAX_UINT32:
            self.get_logger().warn("timestamp overflow")
            self.start_time = time.time()
            return 0
        return timestamp

    def process_signals(self, signals):
        t1 = time.time()

        n_buffer = signals.shape[1]
        n_mics = signals.shape[0]
        if self.n_mics is None:
            self.n_mics = n_mics

        assert self.n_mics == n_mics
        assert n_buffer == self.current_params["n_buffer"]
        assert self.Fs is not None, "Need to set Fs before processing."

        # publishing
        msg = create_signals_message(
            signals, self.mic_positions, self.get_time_ms(), self.Fs
        )
        self.publisher_signals.publish(msg)

        # saving
        if SAVE_BUFFERS > 0:
            if self.save_data is None:
                self.save_data = np.empty((self.n_mics, n_buffer * SAVE_BUFFERS))

            if self.save_idx < SAVE_BUFFERS:
                self.save_data[
                    :, self.save_idx * n_buffer : (self.save_idx + 1) * n_buffer
                ] = signals
            elif self.save_idx == SAVE_BUFFERS:
                for i in range(self.save_data.shape[0]):
                    fname = f"{SAVE_DIR}/{self.get_name()}_mic{i}.wav"
                    write(fname, self.Fs, self.save_data[i])
                    self.get_logger().info(f"Saved audio as {fname}")
            self.save_idx += 1

        t2 = time.time()
        processing_time = t2 - t1
        if processing_time > 1.0 / self.publish_rate:
            self.get_logger().warn(
                f"processing time ({processing_time:.1e}) longer than publishing period ({1/self.publish_rate:.1e})"
            )
