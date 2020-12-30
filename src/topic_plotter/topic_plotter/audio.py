import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pylab as plt

from audio_interfaces.msg import Signals, SignalsFreq
from audio_interfaces_py.messages import read_signals_message, read_signals_freq_message 
from .live_plotter import LivePlotter
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)

MIN_FREQ = -np.inf #400
MAX_FREQ = np.inf #600

XMIN_FREQ = 100 #200 # min plotting frequency in Hz
XMAX_FREQ = 5000 # max plotting frequency in Hz
YMIN_FREQ = 1e-10 # min plotting frequency in Hz
YMAX_FREQ = 1e5 # max plotting frequency in Hz
FIG_SIZE = (15, 10)

class AudioPlotter(Node):
    def __init__(self):
        super().__init__("audio_plotter")

        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 10
        )

        self.subscription_signals = self.create_subscription(
            Signals, "audio/signals", self.listener_callback_signals, 10
        )

        #self.fig, ax = plt.subplots()
        self.fig, axs = plt.subplots(2)
        self.axs = {
            #"signals frequency": ax,
            "signals frequency": axs[0],
            "signals time": axs[1],
        }
        self.fig.set_size_inches(*FIG_SIZE)
        self.plotter_dict = {}
        self.current_n_buffer = None
        self.current_n_frequencies = None

        self.title = None


    def init_plotter(self, name, xlabel='x', ylabel='y', log=True, ymin=-np.inf, ymax=np.inf, xmin=-np.inf, xmax=np.inf):

        if not name in self.plotter_dict.keys():
            self.plotter_dict[name] = LivePlotter(ymax, ymin, label=name, log=log, max_xlim=xmax, min_xlim=xmin, ax=self.axs[name], fig=self.fig)
            self.plotter_dict[name].ax.set_xlabel(xlabel)
            self.plotter_dict[name].ax.set_ylabel(ylabel)


    def listener_callback_signals_f(self, msg):
        self.init_plotter("signals frequency", 
                xlabel="frequency [Hz]", 
                ylabel="magnitude [-]", 
                ymin=YMIN_FREQ, 
                ymax=YMAX_FREQ, 
                xmin=XMIN_FREQ, 
                xmax=XMAX_FREQ)

        if msg.n_frequencies != self.current_n_frequencies:
            self.plotter_dict["signals frequency"].clear()

        __, signals_f, freqs = read_signals_freq_message(msg)

        # remove zero frequencies
        signals_f = signals_f[freqs > 0, :]
        freqs = freqs[freqs > 0]

        # sort frequencies
        indices = np.argsort(freqs)
        y = np.abs(signals_f[indices, :].T)
        x = freqs[indices]

        max_freq = x[np.argmax(y, axis=1)] 
        labels = [f"mic {i}" for i in range(y.shape[1])]
        self.plotter_dict["signals frequency"].update_lines(y, x, labels, linestyle='-', marker='o')

        self.title = f"time [ms]: {msg.timestamp}, max at {max_freq}Hz"
        self.fig.suptitle(self.title, y=0.9)

        self.plotter_dict["signals frequency"].ax.set_xticks(np.arange(XMIN_FREQ, XMAX_FREQ, 200))
        self.plotter_dict["signals frequency"].ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        self.plotter_dict["signals frequency"].update_axvlines(freqs)
        self.current_n_frequencies = msg.n_frequencies

        self.plotter_dict["signals frequency"].ax.set_xlim(XMIN_FREQ, XMAX_FREQ)

        #[ax.axis('tight') for ax in self.axs.values()]
        self.fig.canvas.draw()


    def listener_callback_signals(self, msg):
        self.init_plotter("signals time", xlabel="time idx [-]", ylabel="magnitude [-]", log=False)

        __, signals = read_signals_message(msg)
        labels = [f"mic {i}" for i in range(1, 1+msg.n_mics)]

        if msg.n_buffer != self.current_n_buffer:
            self.plotter_dict["signals time"].clear()

        self.plotter_dict["signals time"].update_lines(
            signals, range(signals.shape[1]), labels
        )
        if self.title is None:
            self.fig.suptitle(f"time (ms): {msg.timestamp}", y=0.9)
        self.current_n_buffer = msg.n_buffer
        self.fig.canvas.draw()


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
