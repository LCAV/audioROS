#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
processor.py: Process raw audio signals and publish frequency bins signals_f 

This is the digital twin of what the microphone board does. 
"""

import os
import sys
import time

from scipy import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from .beam_former import BeamFormer

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from noise_cancellation import filter_iir_bandpass
from bin_selection import select_frequencies as embedded_select_frequencies

from audio_interfaces.msg import Signals, SignalsFreq, Correlations
from audio_interfaces_py.messages import create_correlations_message, create_signals_freq_message, read_signals_freq_message, read_signals_message
from crazyflie_description_py.parameters import TUKEY_ALPHA

# Denoising method
# METHOD_NOISE = "bandpass"
METHOD_NOISE = ""
METHOD_NOISE_DICT = {"bandpass": {"fmin": 100, "fmax": 300, "order": 3}}

# Windowing method, available:
# - "" (no window)
# - "tukey" (flat + cosine on borders)
METHOD_WINDOW = "tukey"

# Frequency selection
THRUST = 50000
MIN_FREQ = 3000
MAX_FREQ = 5000
SINGLE_FREQ = 4000
DELTA_FREQ = 100

METHOD_FREQUENCY = "" #"single"
METHOD_FREQUENCY_DICT = {
    "": {
        "min_freq": MIN_FREQ,
        "max_freq": MAX_FREQ,
        "delta_freq": DELTA_FREQ,
        "filter_snr": False,
        "thrust": 0, 
    },
    "snr": {
        "min_freq": MIN_FREQ,
        "max_freq": MAX_FREQ,
        "delta_freq": DELTA_FREQ,
        "filter_snr": True,
        "thrust": 0, 
    },
    "props": {
        "min_freq": MIN_FREQ,
        "max_freq": MAX_FREQ,
        "delta_freq": DELTA_FREQ,
        "filter_snr": False,
        "thrust": THRUST, 
    },
    "props_snr": {
        "min_freq": MIN_FREQ,
        "max_freq": MAX_FREQ,
        "delta_freq": DELTA_FREQ,
        "filter_snr": True,
        "thrust": THRUST, 
    },
    "single": {
        "min_freq": SINGLE_FREQ,
        "max_freq": SINGLE_FREQ,
        "delta_freq": 100,
        "filter_snr": False,
        "thrust": 0,
    }
}

def verify_validity(params_dict):
    assert all([key in params_dict.keys() for key in METHOD_FREQUENCY_DICT[METHOD_FREQUENCY].keys()])
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
    def __init__(self, plot_freq=False, plot_time=False):
        super().__init__("processor")

        self.plot_freq = plot_freq
        self.plot_time = plot_time
        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )
        self.publisher_signals_f = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )

        self.methods = {
            "noise": METHOD_NOISE,
            "window": METHOD_WINDOW,
            "frequency": METHOD_FREQUENCY,
        }
        parameters = []
        for key, value in self.methods.items():
            self.declare_parameter(key)
            parameters.append(
                rclpy.parameter.Parameter(key, rclpy.Parameter.Type.STRING, value)
            )

        self.frequency_params = METHOD_FREQUENCY_DICT[METHOD_FREQUENCY]
        for key, value in self.frequency_params.items():
            self.declare_parameter(key)
            parameters.append(
                rclpy.parameter.Parameter(key, rclpy.Parameter.Type.INTEGER, value)
            )

        self.set_parameters(parameters)
        self.set_parameters_callback(self.set_params)

        self.beam_former = BeamFormer()

    def set_params(self, params):
        new_params = self.frequency_params.copy()
        for param in params:
            if param.name in self.methods.keys():
                # set high-level parameters
                value = param.get_parameter_value().string_value
                self.methods[param.name] = value

                if param.name == "frequency":
                    for key in new_params.keys():
                        new_params[key] = METHOD_FREQUENCY_DICT[value][key]
            else:
                # set low-level parameters
                self.methods["frequency"] = "custom"
                value = param.get_parameter_value().integer_value

                new_params[param.name] = value

        if verify_validity(new_params):
            self.frequency_params = new_params
            return SetParametersResult(successful=True)
        else:
            return SetParametersResult(successful=False)

    def listener_callback_signals(self, msg):
        t1 = time.time()

        self.mic_positions, signals = read_signals_message(msg)

        # processing
        signals_f, freqs = get_stft(
            signals, msg.fs, self.methods["window"], self.methods["noise"]
        )  # n_samples x n_mics

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

        # check that the above processing pipeline does not
        # take too long compared to the desired publish rate.
        t2 = time.time()
        processing_time = t2 - t1
        self.get_logger().info(
                f"listener_callback_signals: Published signals_f after {processing_time:.2f}s"
        )


def main(args=None):
    rclpy.init(args=args)

    processor = Processor(plot_freq=True, plot_time=False)
    rclpy.spin(processor)

    # Destroy the node explicitly
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
