#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
correlator.py: 
"""

import os
import sys
import time

from scipy import signal
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from noise_cancellation import filter_iir_bandpass
from bin_selection import select_frequencies as embedded_select_frequencies

from audio_interfaces.msg import Signals, SignalsFreq, Correlations

# Denoising method.
# METHOD_NOISE = "bandpass"
METHOD_NOISE = ""
METHOD_NOISE_DICT = {"bandpass": {"fmin": 100, "fmax": 300, "order": 3}}

# Windowing method. Available:
# - "" (no window)
# - "tukey" (flat + cosine on borders)
METHOD_WINDOW = "tukey"

# Frequency selection
THRUST = 43000
METHOD_FREQUENCY = "standard"
METHOD_FREQUENCY_DICT = {
    "standard": {
        "min_freq": 100,
        "max_freq": 10000,
        "delta_freq": 100,
        "filter_snr": True,
        "thrust": THRUST,
    },
    "single": {
        "min_freq": 200,
        "max_freq": 200,
        "delta_freq": 100,
        "filter_snr": False,
        "thrust": 0,
    }
}

# Plotting parameters
MIN_YLIM = 1e-3  # set to -inf for no effect.
MAX_YLIM = 1e5  # set to inf for no effect.
MIN_FREQ = -np.inf # set to -inf for no effect
MAX_FREQ = +np.inf  # set to inf for no effect

def verify_validity(params_dict):
    assert all([key in params_dict.keys() for key in METHOD_FREQUENCY_DICT["standard"].keys()])
    assert params_dict["min_freq"] <= params_dict["max_freq"]
    assert params_dict["min_freq"] >= 0
    assert params_dict["max_freq"] >= 0
    assert params_dict["thrust"] >= 0
    assert params_dict["filter_snr"] in [0, 1]
    return True


def get_stft(signals, Fs, method_window, method_noise):
    if method_window == "tukey":
        window = signal.tukey(signals.shape[1])
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


def create_correlations_message(signals_f, freqs, Fs, n_buffer, method_frequency):

    # calculate Rs for chosen frequency bins
    R = 1 / signals_f.shape[1] * signals_f[:, :, None] @ signals_f[:, None, :].conj()

    msg = Correlations()
    msg.n_mics = int(signals_f.shape[1])
    msg.n_frequencies = len(freqs)
    msg.frequencies = [int(f) for f in freqs]
    msg.corr_real_vect = [float(f) for f in R.real.flatten()]
    msg.corr_imag_vect = [float(f) for f in R.imag.flatten()]
    return msg


class Correlator(Node):
    """ Node to subscribe to audio/signals or audio/signals_f and publish correlations.

    """

    def __init__(self, plot_freq=False, plot_time=False):
        super().__init__("correlator")

        self.plot_freq = plot_freq
        self.plot_time = plot_time
        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )
        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )
        self.publisher_signals_f = self.create_publisher(
            SignalsFreq, "audio/signals_f", 10
        )
        self.publisher_correlations = self.create_publisher(
            Correlations, "audio/correlations", 10
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
        self.mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))

        signals = np.array(msg.signals_vect)
        signals = signals.reshape((msg.n_mics, msg.n_buffer))

        # processing
        signals_f, freqs = get_stft(
            signals, msg.fs, self.methods["window"], self.methods["noise"]
        )  # n_samples x n_mics

        if self.methods["frequency"] != "":
            bins = embedded_select_frequencies(
                msg.n_buffer,
                msg.fs,
                buffer_f=signals_f.T,
                **self.frequency_params,
            )
            freqs = freqs[bins]
            signals_f = signals_f[bins]

        # Create and publish frequency message
        msg_freq = SignalsFreq()
        msg_freq.fs = msg.fs
        msg_freq.timestamp = msg.timestamp
        msg_freq.n_mics = msg.n_mics
        msg_freq.n_frequencies = len(freqs)
        msg_freq.mic_positions = msg.mic_positions
        msg_freq.frequencies = [int(f) for f in freqs]
        # important: signals_f should be of shape n_mics x n_frequencies before flatten() is called.
        msg_freq.signals_real_vect = list(np.real(signals_f.T).astype(float).flatten())
        msg_freq.signals_imag_vect = list(np.imag(signals_f.T).astype(float).flatten())
        self.publisher_signals_f.publish(msg_freq)

        # check that the above processing pipeline does not
        # take too long compared to the desired publish rate.
        t2 = time.time()
        processing_time = t2 - t1
        self.get_logger().info(
                f"listener_callback_signals: Published signals_f after {processing_time:.2f}s"
        )

    def listener_callback_signals_f(self, msg):
        t1 = time.time()
        self.mic_positions = np.array(msg.mic_positions).reshape((msg.n_mics, -1))

        # convert msg format to numpy arrays
        signals_f = np.array(msg.signals_real_vect) + 1j * np.array(
            msg.signals_imag_vect
        )
        signals_f = signals_f.reshape((msg.n_mics, msg.n_frequencies)).T
        freqs = np.array(msg.frequencies)

        self.process_signals_f(signals_f, freqs, msg.fs, msg.timestamp)

        # check that the above processing pipeline does not
        # take too long compared to the desired publish rate.
        t2 = time.time()
        processing_time = t2 - t1
        self.get_logger().info(
                f"listener_callback_signals_f: Published correlations after {processing_time:.2f}s"
        )

    def process_signals_f(self, signals_f, freqs, Fs, timestamp):
        msg_new = create_correlations_message(
            signals_f, freqs, Fs, signals_f.shape[0], self.methods["frequency"]
        )
        msg_new.mic_positions = list(self.mic_positions.astype(float).flatten())
        msg_new.timestamp = timestamp

        # publishing
        self.publisher_correlations.publish(msg_new)


def main(args=None):
    rclpy.init(args=args)

    correlator = Correlator(plot_freq=True, plot_time=False)
    rclpy.spin(correlator)

    # Destroy the node explicitly
    correlator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
