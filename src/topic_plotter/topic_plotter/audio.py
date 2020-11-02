import rclpy
from rclpy.node import Node

import numpy as np

from audio_interfaces.msg import Spectrum, Signals, SignalsFreq, PoseRaw
from audio_interfaces_py.messages import read_signals_message, read_signals_freq_message, read_spectrum_message
from audio_stack.spectrum_estimator import normalize_rows, combine_rows, NORMALIZE
from audio_stack.topic_synchronizer import TopicSynchronizer
from .live_plotter import LivePlotter

MIN_FREQ = -np.inf #400
MAX_FREQ = np.inf #600

YMIN_SPEC = 1e-10
YMAX_SPEC = 2
XMIN_FREQ = 200 # min plotting frequency in Hz
XMAX_FREQ = 3000 # max plotting frequency in Hz

PLOT_LINES = False # make lineplots of spectra (not very readible, can be removed eventually)

class AudioPlotter(Node):
    def __init__(self):
        super().__init__("audio_plotter")

        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )

        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum", self.listener_callback_spectrum, 10
        )

        self.subscription_dynamic_spectrum = self.create_subscription(
            Spectrum, "audio/dynamic_spectrum", self.listener_callback_dynamic_spectrum, 10
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

        # compute and plot combination.
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
        spectrum_sum = combine_rows(spectrum, "sum", keepdims=True)
        spectrum_sum = normalize_rows(spectrum_sum, NORMALIZE)

        labels = ["sum"]
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


    def listener_callback_dynamic_spectrum(self, msg_spec):
        return self.listener_callback_spectrum(msg_spec, name="dynamic")


    def listener_callback_signals_f(self, msg):
        self.init_plotter("signals frequency", xlabel="frequency [Hz]", ylabel="magnitude [-]", ymin=1e-10, ymax=1e3, xmin=XMIN_FREQ, xmax=XMAX_FREQ)

        if msg.n_frequencies != self.current_n_frequencies:
            self.plotter_dict["signals frequency"].clear()

        __, signals_f, freqs = read_signals_freq_message(msg)

        # sort frequencies
        indices = np.argsort(freqs)
        y = np.abs(signals_f[indices, :].T)
        x = freqs[indices]
        labels = [f"mic {i}" for i in range(y.shape[1])]
        self.plotter_dict["signals frequency"].update_lines(y, x, labels, linestyle='-', marker='o')
        self.plotter_dict["signals frequency"].ax.set_title(f"time (ms): {msg.timestamp}")
        self.plotter_dict["signals frequency"].update_axvlines(freqs)
        self.current_n_frequencies = msg.n_frequencies


    def listener_callback_signals(self, msg):
        self.init_plotter("signals time", xlabel="time idx [-]", ylabel="magnitude [-]", log=False)

        __, signals = read_signals_message(msg)
        labels = [f"mic {i}" for i in range(1, 1+msg.n_mics)]

        if msg.n_buffer != self.current_n_buffer:
            self.plotter_dict["signals time"].clear()

        self.plotter_dict["signals time"].update_lines(
            signals, range(signals.shape[1]), labels
        )
        self.current_n_buffer = msg.n_buffer


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
