import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt

from audio_interfaces.msg import Spectrum, PoseRaw
from audio_interfaces_py.messages import read_spectrum_message

from audio_stack.beam_former import normalize_rows, combine_rows
from audio_stack.topic_synchronizer import TopicSynchronizer
from .live_plotter import LivePlotter

# combination used across frequencies
COMBINATION_METHOD = "sum" 
#COMBINATION_METHOD = "product" 

# normalization after combination
NORMALIZE = "sum_to_one"  
#NORMALIZE = "zero_to_one"  
#NORMALIZE = "none"

# window of frequencies used for plotting and combination 
MIN_FREQ = -np.inf #500
MAX_FREQ = np.inf #1500

YMIN_SPEC = 1e-5
YMAX_SPEC = 1
FIGSIZE = (15, 10)

class DoaPlotter(Node):
    def __init__(self):
        super().__init__("doa_plotter")

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum_raw", self.listener_callback_spectrum, 10
        )

        self.subscription_spectrum_combined = self.create_subscription(
            Spectrum, "audio/spectrum_combined", self.listener_callback_spectrum_combined, 10
        )

        self.subscription_spectrum_multi = self.create_subscription(
            Spectrum, "audio/spectrum_multi", self.listener_callback_spectrum_multi, 10
        )

        self.plotter_dict = {}
        self.current_n_buffer = None
        self.current_n_frequencies = None

        self.raw_pose_synch = TopicSynchronizer(20)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10)

        self.fig, axs = plt.subplots(2, 3)
        self.axs = {
          "raw line": axs[0, 0],
          "raw heatmap": axs[1, 0],
          "combined line": axs[0, 1],
          "combined heatmap": axs[1, 1],
          "multi line": axs[0, 2],
          "multi heatmap": axs[1, 2]
        }
        timestamp = 0
        for title, ax in self.axs.items():
            ax.set_title(title + f" {timestamp}ms")
        self.fig.set_size_inches(*FIGSIZE)


    def init_plotter(self, name, xlabel='x', ylabel='y', log=True, ymin=-np.inf, ymax=np.inf, xmin=-np.inf, xmax=np.inf):
        if not (name in self.plotter_dict.keys()):
            if name in self.axs.keys():
                fig = self.fig
                ax = self.axs[name]
            else:
                fig = ax = None

            self.plotter_dict[name] = LivePlotter(ymax, ymin, label=name, log=log, max_xlim=xmax, min_xlim=xmin, fig=fig, ax=ax)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)


    def listener_callback_spectrum(self, msg_spec, name="raw"):
        xlabel = "angle [deg]"
        ylabel = "magnitude [-]"
        self.init_plotter(f"{name} line", xlabel=xlabel, ylabel=ylabel, ymin=YMIN_SPEC, ymax=YMAX_SPEC)
        self.init_plotter(f"{name} heatmap", xlabel=xlabel, ylabel=ylabel, ymin=YMIN_SPEC, ymax=YMAX_SPEC)

        spectrum, frequencies, theta_scan = read_spectrum_message(msg_spec)

        mask = (frequencies <= MAX_FREQ) & (frequencies >= MIN_FREQ)
        labels = [f"f={f:.0f}Hz" for f in frequencies[mask]]
        self.plotter_dict[f"{name} heatmap"].update_mesh(
            spectrum[mask], y_labels=labels, log=True
        )

        # compute and plot combinations.
        spectrum_sum = combine_rows(spectrum, COMBINATION_METHOD, keepdims=True)
        spectrum_sum = normalize_rows(spectrum_sum, NORMALIZE)

        labels = [COMBINATION_METHOD]
        self.plotter_dict[f"{name} line"].update_lines(
            spectrum_sum, theta_scan, labels=labels
        )

        message = self.raw_pose_synch.get_latest_message(msg_spec.timestamp, self.get_logger())
        if message is not None:
            orientation = message.source_direction_deg
            self.plotter_dict[f"{name} line"].update_axvlines([orientation])

            orientation_index = np.argmin(abs(theta_scan - orientation))
            self.plotter_dict[f"{name} heatmap"].update_axvlines([orientation_index], color='orange')

        for title, ax in self.axs.items():
            ax.set_title(title + f" {msg_spec.timestamp}ms")
        self.fig.canvas.draw()


    def listener_callback_spectrum_combined(self, msg_spec):
        return self.listener_callback_spectrum(msg_spec, name="combined")


    def listener_callback_spectrum_multi(self, msg_spec):
        return self.listener_callback_spectrum(msg_spec, name="multi")


def main(args=None):
    rclpy.init(args=args)

    plotter = DoaPlotter()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
