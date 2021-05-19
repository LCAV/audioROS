#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
processor.py: Process raw audio signals and publish frequency bins signals_f 

This is the digital twin of what the microphone board does. 
"""

import os
import sys
import time

import numpy as np
from scipy import signal

import rclpy

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from noise_cancellation import filter_iir_bandpass
from bin_selection import select_frequencies as embedded_select_frequencies

from audio_interfaces.msg import Signals, SignalsFreq
from audio_interfaces_py.messages import (
    create_signals_freq_message,
    read_signals_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams
from audio_stack.parameters import TUKEY_ALPHA

# Denoising method, available:
# - "" (no denoising)
# - "bandpass" (apply bandpass filter)
# - "single" (keep only single frequency)
METHOD_NOISE = ""
METHOD_NOISE_DICT = {"bandpass": {"fmin": 100, "fmax": 300, "order": 3}}

# Windowing method, available:
# - "" (no window)
# - "tukey"
# - "hann"
# - "flattop"
METHOD_WINDOW = "tukey"


def get_stft(signals, fs, method_window="", method_noise=""):
    if method_window == "tukey":
        window = signal.tukey(signals.shape[1], alpha=TUKEY_ALPHA)
        signals *= window
    elif method_window == "hann":
        window = signal.hann(signals.shape[1])
        signals *= window
    elif method_window == "flattop":
        window = signal.flattop(signals.shape[1])
        signals *= window
    elif method_window == "":
        pass
    else:
        raise ValueError(method_window)

    if method_noise == "bandpass":
        signals = filter_iir_bandpass(
            signals,
            Fs=fs,
            method="cheby2",
            plot=False,
            **METHOD_NOISE_DICT[method_noise],
        )
    elif method_noise == "single":
        signals = filter_iir_bandpass(
            signals,
            Fs=fs,
            method="single",
            plot=False,
            **METHOD_NOISE_DICT[method_noise],
        )
    elif method_noise == "":
        pass
    else:
        ValueError(method_noise)

    signals_f = np.fft.rfft(signals, n=signals.shape[1], axis=1).T  # n_samples x n_mics
    freqs = np.fft.rfftfreq(n=signals.shape[1], d=1 / fs)
    return signals_f, freqs


class Processor(NodeWithParams):
    """ Node to subscribe to audio/signals or audio/signals_f and publish correlations.
    """

    # Frequency selection
    # set min_freq==max_freq to choose one frequency only.
    PARAMS_DICT = {
        "n_freqs": 32,
        "min_freq": 100,
        "max_freq": 8000,
        "delta_freq": 100,  # not used without thrust
        "filter_snr": 0,  # equivalent to filter_snr_enable
        "thrust": 0,  # if > 0, we use filter_props_enable
    }

    def __init__(self):
        super().__init__("processor")

        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )
        self.publisher_signals_f = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.mic_positions = None

    # below overwrites base class verify_validity function.
    @staticmethod
    def verify_validity(params_dict):
        assert params_dict["min_freq"] <= params_dict["max_freq"]
        assert params_dict["min_freq"] >= 0
        assert params_dict["max_freq"] >= 0
        assert params_dict["thrust"] >= 0
        assert params_dict["filter_snr"] in [0, 1]
        return True

    def listener_callback_signals(self, msg):
        t1 = time.time()

        self.mic_positions, signals = read_signals_message(msg)
        signals_f, freqs = get_stft(
            signals, msg.fs, METHOD_WINDOW, METHOD_NOISE
        )  # n_samples x n_mics

        bins = embedded_select_frequencies(
            msg.n_buffer, msg.fs, buffer_f=signals_f.T, **self.current_params,
        )
        freqs = freqs[bins]
        signals_f = signals_f[bins]

        msg_freq = create_signals_freq_message(
            signals_f, freqs, self.mic_positions, msg.timestamp, None, msg.fs
        )
        self.publisher_signals_f.publish(msg_freq)

        self.get_logger().info(
            f"listener_callback_signals: Published signals_f after {time.time() - t1:.2f}s"
        )


def main(args=None):
    rclpy.init(args=args)

    processor = Processor()
    rclpy.spin(processor)

    # Destroy the node explicitly
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
