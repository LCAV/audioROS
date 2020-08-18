#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
correlator.py: 
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node

from audio_interfaces.msg import SignalsFreq, Correlations
from audio_stack.live_plotter import LivePlotter

# Plotting parameters
MAX_YLIM = 1e5 # set to inf for no effect.
MIN_YLIM = 1e-3 # set to -inf for no effect.
MAX_FREQ = 2000 # set to inf for no effect

# DEBUGGING ONLY
N_MICS = 4
N_FREQUENCIES = 32

def create_correlations_message(signals_f, frequencies): 
    n_frequencies = len(frequencies)
    assert n_frequencies == N_FREQUENCIES
    R = 1 / signals_f.shape[1] * signals_f[:, :, None] @ signals_f[:, None, :].conj()
    assert R.shape == (n_frequencies, N_MICS, N_MICS)

    msg = Correlations()
    msg.n_mics = int(signals_f.shape[1])
    msg.n_frequencies = len(frequencies)
    msg.frequencies = [int(f) for f in frequencies]
    msg.corr_real_vect = list(R.real.astype(float).flatten())
    msg.corr_imag_vect = list(R.imag.astype(float).flatten())
    return msg


class Correlator(Node):
    def __init__(self, plot_freq=False):
        super().__init__('correlator')

        self.plot_freq = plot_freq
        self.subscription_signals = self.create_subscription(SignalsFreq,
            'audio/signals_f', self.listener_callback_signals, 10)
        self.publisher_correlations = self.create_publisher(Correlations, 'audio/correlations', 10)

        if self.plot_freq:
            self.plotter_freq = LivePlotter(MAX_YLIM, MIN_YLIM)
            self.plotter_freq.ax.set_xlabel('frequency [Hz]')
            self.plotter_freq.ax.set_ylabel('magnitude [-]')

        self.labels = None

    def listener_callback_signals(self, msg):
        t1 = time.time()

        if (self.labels is None) and self.plot_freq:
            self.labels = [f"mic{i}" for i in range(msg.n_mics)]

        signals_f = np.array(msg.signals_real_vect) + 1j*np.array(msg.signals_imag_vect)
        signals_f = signals_f.reshape((msg.n_mics, msg.n_frequencies)).T
        freqs = np.array(msg.frequencies)

        msg_new = create_correlations_message(signals_f, freqs)
        msg_new.timestamp = msg.timestamp

        mask = freqs < MAX_FREQ
        masked_frequencies = [f for f in msg_new.frequencies if f < MAX_FREQ]

        # plotting
        if self.plot_freq: 
            self.plotter_freq.update_lines(np.abs(signals_f[mask].T), freqs[mask], self.labels)
            self.plotter_freq.update_axvlines(masked_frequencies)

        # publishing
        self.publisher_correlations.publish(msg_new)
        self.get_logger().info(f'Publishing at {msg_new.timestamp}: data from {msg_new.n_mics} mics.')

        # check that the above processing pipeline does not
        # take too long compared to the desired publish rate.
        t2 = time.time()
        processing_time = t2-t1


def main(args=None):

    rclpy.init(args=args)

    correlator = Correlator(plot_freq=True)
    rclpy.spin(correlator)

    # Destroy the node explicitly
    correlator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
