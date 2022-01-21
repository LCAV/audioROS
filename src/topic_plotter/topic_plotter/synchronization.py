#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
synchronization.py: 
"""

import rclpy
import numpy as np
import matplotlib.pylab as plt

from audio_interfaces_py.messages import (
    read_signals_freq_message,
    read_pose_raw_message,
)
from audio_interfaces.msg import PoseRaw, SignalsFreq
from audio_interfaces_py.node_with_params import NodeWithParams
from .live_plotter import LivePlotter

N_TIMES_POSE = 100
N_TIMES_SIGNALS = 5
FIG_SIZE = (10, 4)


class TimePlotter(NodeWithParams):
    PARAMS_DICT = {
        "n_times_pose": N_TIMES_POSE,
        "n_times_signals": N_TIMES_SIGNALS,
    }

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.fig.set_size_inches(*FIG_SIZE)

        super().__init__("audio_plotter")
        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.listener_callback_pose_raw, 10
        )
        # need to do this here becasue it requires self.axs etc.

        self.lines = {
            name: {
                k: np.full(self.current_params[f"n_times_{name}"], np.nan)
                for k in ["x", "y"]
            }
            for name in ["signals", "pose"]
        }
        self.plotter = LivePlotter(
            2, 0, label="time", log=False, ax=self.ax, fig=self.fig,
        )
        self.plotter.ax.set_xlabel("time [s]")
        self.plotter.ax.set_ylabel("messages")

    def listener_callback_signals_f(self, msg):
        x = msg.timestamp * 1e-3
        y = 0
        self.lines["signals"]["x"] = np.r_[self.lines["signals"]["x"][1:], x]
        self.lines["signals"]["y"] = np.r_[self.lines["signals"]["y"][1:], y]
        self.plotter.update_scatter(
            self.lines["signals"]["x"], self.lines["signals"]["y"], "signals"
        )
        self.set_lims()
        self.fig.canvas.draw()

    def listener_callback_pose_raw(self, msg):
        x = msg.timestamp * 1e-3
        y = 1
        self.lines["pose"]["x"] = np.r_[self.lines["pose"]["x"][1:], x]
        self.lines["pose"]["y"] = np.r_[self.lines["pose"]["y"][1:], y]
        self.plotter.update_scatter(
            self.lines["pose"]["x"], self.lines["pose"]["y"], "pose"
        )
        self.set_lims()
        self.fig.canvas.draw()

    def set_lims(self):
        xmin = np.nanmin([self.lines["pose"]["x"][0], self.lines["signals"]["x"][0]])
        xmax = np.nanmax([self.lines["pose"]["x"][-1], self.lines["signals"]["x"][-1]])
        if np.isnan(xmin):
            xmin = None
        if np.isnan(xmax):
            xmax = None
        self.ax.set_ylim(-1, 2)
        self.ax.set_xlim(xmin, xmax)


def main(args=None):
    rclpy.init(args=args)

    plotter = TimePlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()
