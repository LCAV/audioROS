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

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + '/../../../crazyflie-audio/python/')
from noise_cancellation import filter_iir_bandpass
from algos_beamforming import select_frequencies

from audio_interfaces.msg import Signals, Correlations
from .live_plotter import LivePlotter

# TODO(FD): make below ROS parameters.
METHOD_NOISE = "bandpass"
METHOD_NOISE_DICT = {
    "bandpass": {
        "fmin": 100,
        "fmax": 300,
        "order": 3
    }
}

# Windowing method. Available: 
# - None (no window)
# - tukey (flat + cosine on borders)
METHOD_WINDOW = "tukey" 

# Frequency selection
N_FREQUENCIES = 10
METHOD_FREQUENCY = "single" #"uniform"
METHOD_FREQUENCY_DICT = {
    "uniform": { # uniform frequencies between min and max
        "num_frequencies": N_FREQUENCIES,
        "min_freq": 100,
        "max_freq": 1000,
    },
    "single": { # one single frequency (beam width between min and max)
        "num_frequencies": 1,
        "min_freq": 150,
        "max_freq": 250,
    },
    "between": { # random, between propeller frequencies
        "num_frequencies": N_FREQUENCIES,
        "min_freq": 100,
        "max_freq": 1000,
        "delta": 1
    },
    "between_snr": { # choose highest snr between propeller frequencies
        "num_frequencies": N_FREQUENCIES,
        "min_freq": 100,
        "max_freq": 1000,
        "amp_ratio": 1, 
        "delta": 1
    }
}

# Plotting parameters
MAX_YLIM = 1e5 # set to inf for no effect.
MIN_YLIM = 1e-3 # set to -inf for no effect.
MAX_FREQ = 2000 # set to inf for no effect

def get_stft(signals, Fs):
    if METHOD_WINDOW == "tukey":
        window = signal.tukey(signals.shape[1])
        signals *= window
    elif METHOD_WINDOW is None:
        pass
    else:
        raise ValueError(METHOD_WINDOW)

    if METHOD_NOISE == "bandpass":
        signals = filter_iir_bandpass(signals, Fs=Fs, method='cheby2', plot=False, 
                                      **METHOD_NOISE_DICT[METHOD_NOISE])
    elif METHOD_NOISE == "single":
        signals = filter_iir_bandpass(signals, Fs=Fs, method='single', plot=False, 
                                      **METHOD_NOISE_DICT[METHOD_NOISE])
    elif METHOD_NOISE is None:
        pass
    else:
        ValueError(METHOD_NOISE)

    signals_f = np.fft.rfft(signals, n=signals.shape[1], axis=1).T # n_samples x n_mics
    freqs = np.fft.rfftfreq(n=signals.shape[1], d=1/Fs)
    return signals_f, freqs


def create_correlations_message(signals_f, freqs, Fs, n_buffer): 
    bins = select_frequencies(n_buffer, Fs, METHOD_FREQUENCY, buffer_f=signals_f,
                              **METHOD_FREQUENCY_DICT[METHOD_FREQUENCY])
    frequencies = list(freqs[bins].flatten())
    
    # calculate Rs for chosen frequency bins 
    R = 1 / signals_f.shape[1] * signals_f[bins, :, None] @ signals_f[bins, None, :].conj()

    msg = Correlations()
    msg.n_mics = int(signals_f.shape[1])
    msg.n_frequencies = len(frequencies)
    msg.frequencies = frequencies
    msg.corr_real_vect = list(np.real(R.flatten()))
    msg.corr_imag_vect = list(np.imag(R.flatten()))
    return msg


class Correlator(Node):
    def __init__(self, Fs, plot_freq=False, plot_time=False):
        super().__init__('correlator')

        self.Fs = Fs
        self.plot_freq = plot_freq
        self.plot_time = plot_time
        self.subscription_signals = self.create_subscription(Signals,
            'audio/signals',
            self.listener_callback_signals, 10)
        self.publisher_correlations = self.create_publisher(Correlations, 'audio/correlations', 10)

        if self.plot_freq:
            self.plotter_freq = LivePlotter(MAX_YLIM, MIN_YLIM)
            self.plotter_freq.ax.set_xlabel('frequency [Hz]')
            self.plotter_freq.ax.set_ylabel('magnitude [-]')

        if self.plot_time:
            self.plotter_time = LivePlotter(MAX_YLIM, MIN_YLIM, log=False)
            self.plotter_time.ax.set_xlabel('time idx [-]')
            self.plotter_time.ax.set_ylabel('magnitude [-]')

        self.labels = None

    def listener_callback_signals(self, msg):
        t1 = time.time()

        if (self.labels is None) and (self.plot_time or self.plot_freq):
            self.labels = [f"mic{i}" for i in range(msg.n_mics)]

        signals = np.array(msg.signals_vect)
        signals = signals.reshape((msg.n_mics, msg.n_buffer))

        # processing
        signals_f, freqs = get_stft(signals, self.Fs) # n_samples x n_mics
        msg_new = create_correlations_message(signals_f, freqs, self.Fs, signals.shape[1])
        msg_new.timestamp = msg.timestamp

        mask = freqs < MAX_FREQ
        masked_frequencies = [f for f in msg_new.frequencies if f < MAX_FREQ]

        # plotting
        if self.plot_freq: 
            self.plotter_freq.update_lines(np.abs(signals_f[mask].T), freqs[mask], self.labels)
            self.plotter_freq.update_axvlines(masked_frequencies)
        if self.plot_time: 
            self.plotter_time.update_lines(signals, range(signals.shape[1]), self.labels)

        # publishing
        self.publisher_correlations.publish(msg_new)
        self.get_logger().info(f'Publishing at {msg_new.timestamp}: data from {msg_new.n_mics} mics.')

        # check that the above processing pipeline does not
        # take too long compared to the desired publish rate.
        t2 = time.time()
        processing_time = t2-t1

def main(args=None):
    rclpy.init(args=args)

    Fs = 44100
    correlator = Correlator(Fs, plot_freq=True, plot_time=True)
    rclpy.spin(correlator)

    # Destroy the node explicitly
    correlator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
