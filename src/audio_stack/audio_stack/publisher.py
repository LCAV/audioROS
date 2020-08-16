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

# Windowing method. Available: 
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

# Plotting parameters
MAX_YLIM = 1e5 # set to inf for no effect.
MIN_YLIM = 1e-3 # set to -inf for no effect.
MAX_FREQ = 2000 # set to inf for no effect

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

        if publish_rate is None:
            # by default, adapt publish rate to n_buffer (no samples skipped)
            self.publish_rate = int(Fs / n_buffer)
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
        self.time_idx = 0

    def set_Fs(self, Fs):
        self.Fs = Fs
        self.n_between_buffers = self.Fs // self.publish_rate
        self.seconds_between_buffers  = 1 / self.Fs * self.n_between_buffers

    def process_signal(self, signals, max_f=np.inf):
        assert self.Fs is not None, 'Need to set Fs before processing.'
        assert signals.shape[0] == 4

        # processing
        signals_f, freqs = get_stft(signals, self.Fs) # n_samples x n_mics
        msg = create_correlations_message(signals_f, freqs, self.Fs, signals.shape[1])
        msg.timestamp = self.time_idx

        mask = freqs < max_f
        masked_frequencies = [f for f in msg.frequencies if f < max_f]

        # plotting
        if self.plot: 
            labels = [f"mic{i}" for i in range(signals_f.shape[1])]
            self.plotter.update_lines(np.abs(signals_f[mask].T), freqs[mask], labels)
            self.plotter.update_axvlines(masked_frequencies)

        # publishing
        self.publisher_correlations.publish(msg)
        self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')

        # increment index
        self.time_idx += 1
        if self.time_idx >= MAX_TIMESTAMP_INT:
            self.get_logger().error('timestamp overflow.')
            self.time_idx = self.time_idx % MAX_TIMESTAMP_INT


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_message = self.create_publisher(String, 'message', 10)
        self.publisher_correlations = self.create_publisher(Correlations, 'correlations', 10)
        self.Fs = 10
        timer_period = 1/self.Fs  # seconds

        self.time_idx = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.time_idx}'
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
        msg.timestamp = self.time_idx
        self.publisher_correlations.publish(msg)
        self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')
        self.time_idx += 1


class FilePublisher(AudioPublisher):
    def __init__(self, filenames=None, loop=False, n_buffer=256, publish_rate=None, plot=False):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        super().__init__('file_publisher', n_buffer=n_buffer, publish_rate=publish_rate, plot=plot, Fs=None)
        self.loop = loop

        # read audio from files
        self.file_idx = 0
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

        self.create_timer(1./self.publish_rate, self.publish_loop)

    def publish_loop(self, n_buffer):
        signals = np.c_[[
           data['data'][self.file_idx:self.file_idx + self.n_buffer] for data in self.audio_data.values()
        ]] # n_mics x n_samples

        self.process_signal(signals, max_f=MAX_FREQ)
 
        self.file_idx += self.n_between_buffers
        if self.file_idx + self.n_buffer >= self.len:
            if loop:
                self.file_idx = 0
            else:
                sys.exit()


class StreamPublisher(AudioPublisher):
    def __init__(self, Fs, n_buffer=256, publish_rate=None, plot=False):
        super().__init__('stream_publisher', n_buffer=n_buffer, publish_rate=publish_rate, plot=plot, Fs=Fs)

        self.n_mics = 4

        sd.default.device = 'default'
        sd.check_input_settings(sd.default.device, channels=self.n_mics, samplerate=self.Fs)

        # blocking stream
        self.stream = sd.InputStream(channels=self.n_mics, blocksize=self.n_buffer)
        self.stream.start()
        self.create_timer(1./self.publish_rate, self.publish_loop)

        # non-blocking stream
        #self.duration = 60 * 60 # seconds
        #with sd.InputStream(channels=self.n_mics, callback=self.publish_stream, blocksize=self.n_buffer):
            #sd.sleep(self.duration)
            #plt.show()
            # need below return or we will stay in this context forever.
            #return

    def publish_stream(self, signals_T, frames, time_stream, status): 
        self.get_logger().info(f'buffer start time: {time_stream.inputBufferAdcTime}')
        self.get_logger().info(f'currentTime: {time_stream.currentTime}')

        if status:
            print(status)

        self.process_signal(signals_T.T, max_f=MAX_FREQ)

    def publish_loop(self): 

        n_available = self.stream.read_available
        if self.n_buffer > n_available:
            self.get_logger().warn('Requesting more frames ({self.n_buffer}) than available ({n_available})')
        signals_T, overflow = self.stream.read(self.n_buffer) # frames x channels
        if overflow:
            self.get_logger().warn('overflow')

        self.process_signal(signals_T.T, max_f=MAX_FREQ)


def main(args=None):
    import os

    rclpy.init(args=args)

    audio_source = 'stream'
    #audio_source = 'file'
    #audio_source = 'dummy'

    # TODO(FD): make these ROS parameters
    n_buffer = 2**10
    plot = True #False
    publish_rate = None #11 # in Hz

    if audio_source == 'file':
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.abspath(current_dir + '/../../../crazyflie-audio/data/simulated')
        fnames = [os.path.join(data_dir, f'analytical_source_mic{i}.wav') for i in range(1, 5)] 
        loop = True # loop after file ends.
        publisher = FilePublisher(fnames, n_buffer=n_buffer, publish_rate=publish_rate, loop=True, plot=plot)
    elif audio_source == 'stream':
        Fs = 44100
        publisher = StreamPublisher(Fs=Fs, publish_rate=publish_rate, n_buffer=n_buffer, plot=plot)
    elif audio_source == 'dummy':
        publisher = DummyPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
