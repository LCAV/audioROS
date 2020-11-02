import rclpy
from rclpy.node import Node

import numpy as np

from audio_interfaces.msg import Spectrum, PoseRaw
from audio_interfaces_py.messages import read_spectrum_message
from audio_stack.spectrum_estimator import normalize_rows, combine_rows
from audio_stack.topic_synchronizer import TopicSynchronizer
from .live_plotter import LivePlotter

NORMALIZE = "zero_to_one"
COMBINATION_METHOD = "sum"

# window of frequencies used for plotting and combination 
MIN_FREQ = -np.inf #400
MAX_FREQ = np.inf #600

YMIN_SPEC = 1e-10
YMAX_SPEC = 2

PLOT_LINES = False # make lineplots of spectra (not very readible, can be removed eventually)

class AudioPlotter(Node):
    def __init__(self):
        super().__init__("audio_plotter")

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum_raw", self.listener_callback_spectrum, 10
        )

        self.subscription_spectrum_combined = self.create_subscription(
            Spectrum, "audio/spectrum_combined", self.listener_callback_spectrum_combined, 10
        )

        self.plotter_dict = {}
        self.current_n_buffer = None
        self.current_n_frequencies = None

        self.raw_pose_synch = TopicSynchronizer(10)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10)


    def init_plotter(self, name, xlabel='x', ylabel='y', log=True, ymin=-np.inf, ymax=np.inf, xmin=-np.inf, xmax=np.inf):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(ymax, ymin, label=name, log=log, max_xlim=xmax, min_xlim=xmin)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)


    def listener_callback_spectrum(self, msg_spec, name="static"):
        xlabel = "angle [deg]"
        ylabel = "magnitude [-]"
        self.init_plotter(f"{name} combined spectra", xlabel=xlabel, ylabel=ylabel, ymin=YMIN_SPEC, ymax=YMAX_SPEC)
        self.init_plotter(f"{name} raw spectra heatmap", xlabel=xlabel, ylabel=ylabel, ymin=YMIN_SPEC, ymax=YMAX_SPEC)
        if PLOT_LINES: 
            self.init_plotter(f"{name} raw spectra", xlabel=xlabel, ylabel=ylabel, ymin=YMIN_SPEC, ymax=YMAX_SPEC)

        spectrum, frequencies, theta_scan = read_spectrum_message(msg_spec)

        mask = (frequencies <= MAX_FREQ) & (frequencies >= MIN_FREQ)
        labels = [f"f={f:.0f}Hz" for f in frequencies[mask]]
        self.plotter_dict[f"{name} raw spectra heatmap"].update_mesh(
            spectrum[mask], y_labels=labels, log=True
        )
        if PLOT_LINES: 
            self.plotter_dict[f"{name} raw spectra"].update_lines(
                spectrum[mask], theta_scan, labels=labels
            )

        # compute and plot combinations.
        spectrum_sum = combine_rows(spectrum, COMBINATION_METHOD, keepdims=True)
        spectrum_sum = normalize_rows(spectrum_sum, NORMALIZE)

        labels = [COMBINATION_METHOD]
        self.plotter_dict[f"{name} combined spectra"].update_lines(
            spectrum_sum, theta_scan, labels=labels
        )

        message = self.raw_pose_synch.get_latest_message(msg_spec.timestamp, self.get_logger())
        if message is not None:
            orientation = message.source_direction_deg
            self.plotter_dict[f"{name} combined spectra"].update_axvlines([orientation])
            if PLOT_LINES: 
                self.plotter_dict[f"{name} raw spectra"].update_axvlines([orientation])

            orientation_index = np.argmin(abs(theta_scan - orientation))
            self.plotter_dict[f"{name} raw spectra heatmap"].update_axvlines([orientation_index], color='orange')


    def listener_callback_spectrum_combined(self, msg_spec):
        return self.listener_callback_spectrum(msg_spec, name="dynamic")


def main(args=None):
    rclpy.init(args=args)

    plotter = AudioPlotter()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
