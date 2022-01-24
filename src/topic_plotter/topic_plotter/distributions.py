#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distributions.py: 
"""

import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt

from audio_interfaces.msg import Distribution
from audio_interfaces_py.messages import read_distribution_message

from .live_plotter import LivePlotter

N_TIMES = 30
LOG = False
N_LABELS = 10
# YLABEL = "distance [cm]"
YLABEL = "angle [deg]"


class DistributionsPlotter(Node):
    def __init__(self):
        super().__init__("distributions_plotter")

        self.x_labels = {}
        self.y_labels = {}
        self.results_matrix = {}
        self.plotter_dict = {}

        distributions = ["raw", "moving"]
        self.fig, self.axs = plt.subplots(1, len(distributions))
        self.fig.set_size_inches(5 * len(distributions), 5)

        topic = "results/distribution_raw"
        self._subscription_distribution_raw = self.create_subscription(
            Distribution, topic, self.listener_callback_distribution_raw, 20
        )
        self.get_logger().info(f"subscribed to {topic}")
        topic = "results/distribution_moving"
        self._subscription_distribution_moving = self.create_subscription(
            Distribution, topic, self.listener_callback_distribution_moving, 20
        )
        self.get_logger().info(f"subscribed to {topic}")

        for i, name in enumerate(distributions):
            self.plotter_dict[name] = LivePlotter(
                label=name, log=False, fig=self.fig, ax=self.axs[i],
            )
            self.x_labels[name] = np.zeros(N_TIMES)
            self.y_labels[name] = None
            self.axs[i].set_xlabel("timestamp [s]")
            self.axs[i].set_ylabel(YLABEL)
            self.axs[i].set_title(name)

    def listener_callback_distribution_raw(self, msg_dist):
        return self.listener_callback_distribution(msg_dist, "raw")

    def listener_callback_distribution_moving(self, msg_dist):
        return self.listener_callback_distribution(msg_dist, "moving")

    def listener_callback_distribution(self, msg_dist, name="raw"):
        # self.get_logger().info(f"callback {name}")
        distances, probs = read_distribution_message(msg_dist)
        if self.y_labels[name] is None:
            self.y_labels[name] = distances
            self.results_matrix[name] = np.zeros((len(distances), N_TIMES), np.float)

        self.x_labels[name] = np.r_[
            self.x_labels[name][1:], round(msg_dist.timestamp * 1e-3, 1)
        ]

        assert np.allclose(self.y_labels[name], distances)
        self.results_matrix[name] = np.c_[
            self.results_matrix[name][:, 1:], probs.reshape(-1, 1)
        ]
        self.get_logger().warn(f"Updating for time {msg_dist.timestamp}")

        self.plotter_dict[f"{name}"].update_mesh(
            self.results_matrix[name],
            x_labels=self.x_labels[name],
            y_labels=self.y_labels[name],
            log=LOG,
            n_labels=N_LABELS,
        )
        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)
    plotter = DistributionsPlotter()
    rclpy.spin(plotter)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
