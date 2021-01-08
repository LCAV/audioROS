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
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from noise_cancellation import filter_iir_bandpass
from bin_selection import select_frequencies as embedded_select_frequencies

from audio_interfaces.msg import Signals, SignalsFreq, Correlations
from audio_interfaces_py.messages import create_correlations_message, create_signals_freq_message, read_signals_freq_message, read_signals_message
from crazyflie_description_py.parameters import TUKEY_ALPHA

# Denoising method, available: 
# - "" (no denoising)
# - "bandpass" (apply bandpass filter)
# - "single" (keep only single frequency)
METHOD_NOISE = ""
METHOD_NOISE_DICT = {"bandpass": {"fmin": 100, "fmax": 300, "order": 3}}

# Windowing method, available:
# - "" (no window)
# - "tukey" (flat + cosine on borders)
METHOD_WINDOW = "tukey"

# Frequency selection
# set min_freq==max_freq to choose one frequency only.
PARAMS_DICT = {
    "n_freqs": (rclpy.Parameter.Type.INTEGER, 32),
    "min_freq": (rclpy.Parameter.Type.INTEGER, 100),
    "max_freq": (rclpy.Parameter.Type.INTEGER, 8000),
    "delta_freq": (rclpy.Parameter.Type.INTEGER, 100), # not used without prop
    "filter_snr": (rclpy.Parameter.Type.INTEGER, 0), # equivalent to filter_snr_enable
    "thrust": (rclpy.Parameter.Type.INTEGER, 0), # if > 0, we use filter_props_enable
}

def verify_validity(params_dict):
    assert params_dict["min_freq"] <= params_dict["max_freq"]
    assert params_dict["min_freq"] >= 0
    assert params_dict["max_freq"] >= 0
    assert params_dict["thrust"] >= 0
    assert params_dict["filter_snr"] in [0, 1]
    return True


def get_stft(signals, Fs, method_window, method_noise):
    if method_window == "tukey":
        window = signal.tukey(signals.shape[1], alpha=TUKEY_ALPHA)
        signals *= window
    elif method_window == "":
        pass
    else:
        raise ValueError(method_window)

    if method_noise == "bandpass":
        signals = filter_iir_bandpass(
            signals,
            Fs=Fs,
            method="cheby2",
            plot=False,
            **METHOD_NOISE_DICT[method_noise],
        )
    elif method_noise == "single":
        signals = filter_iir_bandpass(
            signals,
            Fs=Fs,
            method="single",
            plot=False,
            **METHOD_NOISE_DICT[method_noise],
        )
    elif method_noise == "":
        pass
    else:
        ValueError(method_noise)

    signals_f = np.fft.rfft(signals, n=signals.shape[1], axis=1).T  # n_samples x n_mics
    freqs = np.fft.rfftfreq(n=signals.shape[1], d=1 / Fs)
    return signals_f, freqs


class Processor(Node):
    """ Node to subscribe to audio/signals or audio/signals_f and publish correlations.
    """
    def __init__(self):
        super().__init__("processor", 
            automatically_declare_parameters_from_overrides=True, allow_undeclared_parameters=True)

        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )
        self.publisher_signals_f = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )

        self.frequency_params = {key: val[1] for key, val in PARAMS_DICT.items()}
        self.set_parameters_callback(self.set_params)
        parameters = self.get_parameters(PARAMS_DICT.keys())
        self.set_parameters(parameters)


    def set_params(self, params):
        new_params = self.frequency_params.copy()
        for param in params:
            # we need this in case this parameter
            # was not set at startup. 
            # then we use the default values.
            if param.type_ == param.Type.NOT_SET:
                param = rclpy.parameter.Parameter(param.name, *PARAMS_DICT[param.name])

            new_params[param.name] = param.value

        if verify_validity(new_params):
            self.frequency_params = new_params
            return SetParametersResult(successful=True)
        else:
            return SetParametersResult(successful=False)

    def listener_callback_signals(self, msg):
        t1 = time.time()

        self.mic_positions, signals = read_signals_message(msg)
        signals_f, freqs = get_stft(signals, msg.fs, METHOD_WINDOW, METHOD_NOISE) # n_samples x n_mics

        bins = embedded_select_frequencies(
            msg.n_buffer,
            msg.fs,
            buffer_f=signals_f.T,
            **self.frequency_params,
        )
        freqs = freqs[bins]
        signals_f = signals_f[bins]

        msg_freq = create_signals_freq_message(signals_f, freqs, self.mic_positions, msg.timestamp, None, msg.fs)
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
