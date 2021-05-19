import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt

from audio_interfaces.msg import SignalsFreq
from geometry_msgs.msg import PoseStamped

from audio_interfaces_py.messages import (
    read_signals_freq_message,
    convert_sec_nanosec_to_ms,
)
from audio_stack.topic_synchronizer import TopicSynchronizer

from .live_plotter import LivePlotter

FREQ_HZ = 2000  # frequency bin to use. if not given, we use the max snr one.
MAX_FREQS_TO_PLOT = 4  # maximum frequencies to plot, less than 10
FIG_SIZE = (10, 10)
XMIN = 0
XMAX = 7000
N_MICS = 4


class WallApproachPlotter(Node):
    def __init__(self):
        super().__init__("wall_plotter")

        self.signals_f_synch = TopicSynchronizer(10)
        self.subscription_signals_f = self.create_subscription(
            SignalsFreq,
            "audio/signals_f",
            self.signals_f_synch.listener_callback,
            10,
        )

        self.subscription_pose = self.create_subscription(
            PoseStamped, "geometry/pose", self.listener_callback_pose, 10
        )

        self.plotter_dict = {}
        self.frequency_colors = {}

        self.fig, self.axs = plt.subplots(N_MICS, sharey=True, sharex=True)
        self.fig.set_size_inches(*FIG_SIZE)
        for i in range(N_MICS):
            self.plotter_dict[i] = LivePlotter(
                label=f"mic{i}",
                max_xlim=XMAX,
                min_xlim=XMIN,
                ax=self.axs[i],
                fig=self.fig,
            )
            self.plotter_dict[i].ax.set_xlabel("y coordinate [cm]")
            self.plotter_dict[i].ax.set_title(f"mic{i}")
        self.plotter_dict[0].ax.set_ylabel("loudness")

    def listener_callback_pose(self, msg_pose):
        timestamp = convert_sec_nanosec_to_ms(
            msg_pose.header.stamp.sec, msg_pose.header.stamp.nanosec
        )

        msg_signals_f = self.signals_f_synch.get_latest_message(
            timestamp, self.get_logger()
        )
        if msg_signals_f is not None:
            self.get_logger().info(f"Processing pose {timestamp}")
            __, signals_f, freqs = read_signals_freq_message(msg_signals_f)
            # remove zero frequencies
            signals_f = signals_f[freqs > 0, :]  # n_freqs x n_mics
            freqs = freqs[freqs > 0]

            if FREQ_HZ is None:
                f_idx = np.argmax(np.sum(np.abs(signals_f), axis=1))
            else:
                f_idx = np.argmin(np.abs(freqs - FREQ_HZ))

            xdata = msg_pose.pose.position.y * 100  # cm
            ydata = np.abs(signals_f[f_idx, :])
            self.get_logger().info(
                f"y coordinate [cm]: {xdata:.0f}, frequency [Hz]: {freqs[f_idx]}"
            )

            if f_idx not in self.frequency_colors.keys():
                if len(self.frequency_colors.keys()) >= MAX_FREQS_TO_PLOT:
                    return
                color_idx = len(self.frequency_colors.keys()) % 10
                color = f"C{color_idx}"
                label = f"{freqs[f_idx]}Hz"
                self.frequency_colors[f_idx] = color
            else:
                color = self.frequency_colors[f_idx]
                label = None

            for i in range(N_MICS):
                self.plotter_dict[i].ax.scatter(
                    xdata, ydata[i], color=color, label=label
                )
            self.plotter_dict[0].ax.legend(loc="upper left")
        else:
            self.get_logger().warn(
                f"No valid signals message for pose {timestamp}"
            )

        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)

    plotter = WallApproachPlotter()

    rclpy.spin(plotter)

    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
