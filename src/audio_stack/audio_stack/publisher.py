# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + '/../../../crazyflie-audio/python/')

import matplotlib.pylab as plt
import numpy as np
from scipy.io.wavfile import read
from scipy import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd

from noise_cancellation import filter_iir_bandpass
from algos_beamforming import select_frequencies
from audio_interfaces.msg import Correlations

from .live_plotter import LivePlotter


METHOD_NOISE = "bandpass"
METHOD_NOISE_DICT = {
    "bandpass": {
        "fmin": 100,
        "fmax": 300,
        "order": 3
    }
}

# available: 
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

MAX_YLIM = 1e5 # set to inf for no effect.
MIN_YLIM = 1e-3 # set to -inf for no effect.

MAX_TIMESTAMP_INT = 2**32-1 # needs to match Spectrum.msg and Correlation.msg

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
    msg.real_vect = list(np.real(R.flatten()))
    msg.imag_vect = list(np.imag(R.flatten()))
    return msg


class AudioPublisher(Node):
    def __init__(self, name="audio_publisher", n_buffer=256, publish_rate=None, plot=False, Fs=None):
        super().__init__(name)

        # by default, adapt publish rate to n_buffer (no samples skipped)
        if publish_rate is None:
            self.publish_rate = int(self.Fs / self.n_buffer)
        else:
            self.publish_rate = publish_rate

        self.n_buffer = n_buffer
        self.plot = plot
        self.Fs = None
        if Fs is not None:
            self.set_Fs(Fs)

        if self.plot:
            self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)

        self.publisher_correlations = self.create_publisher(Correlations, 'correlations', 10)
        self.i = 0

    def set_Fs(self, Fs):
        self.Fs = Fs
        self.n_between_buffers = self.Fs // self.publish_rate
        self.seconds_between_buffers  = 1 / self.Fs * self.n_between_buffers

    def process_signal(self, signals):
        assert self.Fs is not None, 'Need to set Fs before processing.'
        assert signals.shape[0] == 4

        signals_f, freqs = get_stft(signals, self.Fs) # n_samples x n_mics
        msg = create_correlations_message(signals_f, freqs, self.Fs, signals.shape[1])
        msg.timestamp = self.i
        
        if self.plot: 
            labels = [f"mic{i}" for i in range(signals_f.shape[1])]
            self.plotter.update_lines(np.abs(signals_f.T), freqs, labels)
            self.plotter.update_axvlines(msg.frequencies)

        self.publisher_correlations.publish(msg)
        self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')

        self.i += self.n_between_buffers
        if self.i >= MAX_TIMESTAMP_INT:
            self.get_logger().warn('timestamp overflow.')
            self.i = self.i % MAX_TIMESTAMP_INT

        # TODO(FD): replace this with ROS sleep function
        time.sleep(self.seconds_between_buffers)


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_message = self.create_publisher(String, 'message', 10)
        self.publisher_correlations = self.create_publisher(Correlations, 'correlations', 10)
        self.Fs = 10
        timer_period = 1/self.Fs  # seconds

        self.i = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_message.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        msg = Correlations()
        msg.n_mics = 4
        msg.n_frequencies = 3
        R = np.empty((msg.n_frequencies, msg.n_mics, msg.n_mics), dtype=np.complex128)
        R[0] = np.array([1.0 + 1j for i in range(msg.n_mics**2)]).reshape((msg.n_mics, msg.n_mics)) 
        R[1] = np.array([2.0 + 2j for i in range(msg.n_mics**2)]).reshape((msg.n_mics, msg.n_mics))
        R[2] = np.array([3.0 + 3j for i in range(msg.n_mics**2)]).reshape((msg.n_mics, msg.n_mics))
        msg.real_vect = list(np.real(R.flatten()))
        msg.imag_vect = list(np.imag(R.flatten()))
        msg.frequencies = [1., 10., 100.]
        msg.timestamp = self.i
        self.publisher_correlations.publish(msg)
        self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')
        self.i += 1


class FilePublisher(AudioPublisher):
    def __init__(self, filenames=None, loop=False, n_buffer=256, publish_rate=None, plot=False):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        super().__init__('file_publisher', n_buffer=n_buffer, publish_rate=publish_rate, plot=plot, Fs=None)
        self.loop = loop

        # read audio from files
        self.audio_data = {f:{} for f in filenames}
        for fname in filenames:
            self.audio_data[fname]['Fs'], self.audio_data[fname]['data'] = read(fname) 

        # read Fs and make sure it matches all files
        Fs = self.audio_data[filenames[0]]['Fs']
        assert all(self.audio_data[f]['Fs']==Fs for f in filenames)
        self.set_Fs(Fs)

        # read len and make sure it matches all files
        self.len = len(self.audio_data[filenames[0]]['data'])
        assert all(len(self.audio_data[f]['data'])==self.len for f in filenames)

        self.publish_loop(n_buffer)

    def publish_loop(self, n_buffer):
        while self.i + self.n_buffer < self.len:
            # read current signal
            signals = np.c_[[
               data['data'][self.i:self.i + self.n_buffer] for data in self.audio_data.values()
            ]] # n_mics x n_samples

            self.process_signal(signals)

            # advance in file
            if self.loop and (self.i + self.n_buffer >= self.len):
                self.i = 0


class StreamPublisher(AudioPublisher):
    def __init__(self, Fs, n_buffer=256, publish_rate=None, plot=False):
        super().__init__('stream_publisher', n_buffer=n_buffer, publish_rate=publish_rate, plot=plot, Fs=Fs)

        n_mics = 4
        duration = 3 # in seconds

        sd.default.device = 'default'
        sd.check_input_settings(sd.default.device, channels=n_mics, samplerate=self.Fs)

        with sd.InputStream(channels=n_mics, callback=self.publish_loop, blocksize=n_buffer):
            sd.sleep(int(duration*1000)) # input is in miliseconds.

    def publish_loop(self, indata, frames, time_correlations, status): 
        if status:
            print(status)
        self.process_signal(indata.T)


def main(args=None):
    import os

    rclpy.init(args=args)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.abspath(current_dir + '/../../../crazyflie-audio/data/simulated')
    n_buffer = 2**10
    plot = False
    publish_rate = 11 #11 # in Hz

    #publisher = DummyPublisher()

    fnames = [os.path.join(data_dir, f'analytical_source_mic{i}.wav') for i in range(1, 5)] 
    #loop = True # loop after file ends.
    #publisher = FilePublisher(fnames, n_buffer=n_buffer, publish_rate=publish_rate, loop=True, plot=plot)

    Fs = 44100
    publisher = StreamPublisher(Fs=Fs, publish_rate=publish_rate, n_buffer=n_buffer, plot=plot)

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
