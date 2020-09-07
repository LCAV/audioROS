import rclpy
from rclpy.node import Node
from audio_interfaces.msg import Spectrum, Signals, SignalsFreq

import numpy as np

from .live_plotter import LivePlotter

# for plotting only
MIN_FREQ = -np.inf #400
MAX_FREQ = np.inf #600

def normalize_spectrum(spectrum):
    return (spectrum - np.min(spectrum, axis=1)[:, None]) / (np.max(spectrum, axis=1)[:, None] - np.min(spectrum, axis=1)[:, None])


class AudioPlotter(Node):
    def __init__(self, plot=False):
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

        self.plotter_dict = {}
        self.current_n_buffer = None
        self.current_n_frequencies = None


    def init_plotter(self, name, xlabel='x', ylabel='y'):
        if not (name in self.plotter_dict.keys()):
            self.plotter_dict[name] = LivePlotter(np.inf, -np.inf, name)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)


    def listener_callback_spectrum(self, msg_spec):
        xlabel = "angle [rad]"
        ylabel = "magnitude [-]"
        self.init_plotter("raw spectra", xlabel=xlabel, ylabel=ylabel)
        self.init_plotter("combined spectra", xlabel=xlabel, ylabel=ylabel)

        frequencies = np.array(msg_spec.frequencies) 
        spectrum = np.array(msg_spec.spectrum_vect).reshape((msg_spec.n_frequencies, msg_spec.n_angles))
        theta_scan = np.linspace(0, 360, msg_spec.n_angles)

        # compute and plot combination.
        mask = (frequencies <= MAX_FREQ) & (frequencies >= MIN_FREQ)
        labels = [f"f={f:.0f}Hz" for f in frequencies[mask]]
        self.plotter_dict["raw spectra"].update_lines(
            spectrum[mask], theta_scan, labels=labels
        )

        # compute and plot combination.
        spectrum_total = normalize_spectrum(spectrum)
        spectrum_product = normalize_spectrum(
                np.product(spectrum_total, axis=0, keepdims=True)
        )
        spectrum_sum = normalize_spectrum(
                np.sum(spectrum_total, axis=0, keepdims=True)
        )
        spectrum_plot = np.r_[spectrum_product, spectrum_sum]
        labels = ["product", "sum"]
        self.plotter_dict["combined spectra"].update_lines(
            spectrum_plot, theta_scan, labels=labels
        )


    def listener_callback_signals_f(self, msg):
        self.init_plotter("signals frequency", xlabel="frequency [Hz]", ylabel="magnitude [-]")

        if msg.n_frequencies != self.current_n_frequencies:
            self.plotter_dict["signals frequency"].clear()

        # sort frequencies
        freqs = np.array(msg.frequencies)
        signals_f = np.array(msg.signals_real_vect) + 1j * np.array(
            msg.signals_imag_vect
        )
        signals_f = signals_f.reshape((msg.n_mics, msg.n_frequencies)).T

        indices = np.argsort(freqs)
        y = np.abs(signals_f[indices, :].T)
        x = freqs[indices]
        self.plotter_dict["signals frequency"].update_lines(y, x, self.labels)
        self.plotter_dict["signals frequency"].ax.set_title(f"time (ms): {msg.timestamp}")
        self.plotter_dict["signals frequency"].update_axvlines(freqs)
        self.current_n_frequencies = msg.n_frequencies


    def listener_callback_signals(self, msg):
        self.init_plotter("signals time", xlabel="time idx [-]", ylabel="magnitude [-]")

        signals = np.array(msg.signals_vect).reshape((msg.n_mics, msg.n_buffer))
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
