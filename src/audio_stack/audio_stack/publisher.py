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
from audio_interfaces.msg import Audio

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

def create_audio_message(signals, Fs): 
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

    bins = select_frequencies(signals.shape[1], Fs, METHOD_FREQUENCY, buffer_f=signals_f,
                              **METHOD_FREQUENCY_DICT[METHOD_FREQUENCY])
    frequencies = list(freqs[bins].flatten())
    
    # calculate Rs for chosen frequency bins 
    R = 1 / signals.shape[0] * signals_f[bins, :, None] @ signals_f[bins, None, :].conj()

    msg = Audio()
    msg.n_mics = int(signals.shape[0])
    msg.n_frequencies = len(frequencies)
    msg.frequencies = frequencies
    msg.real_vect = list(np.real(R.flatten()))
    msg.imag_vect = list(np.imag(R.flatten()))
    return msg, signals_f, freqs


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        self.publisher_message = self.create_publisher(String, 'message', 10)
        self.publisher_audio = self.create_publisher(Audio, 'audio', 10)
        Fs = 10
        timer_period = 1/Fs  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_message.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        msg = Audio()
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
        self.publisher_audio.publish(msg)
        self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')

        self.i += 1


class FilePublisher(Node):
    def __init__(self, filenames=None, publish_rate=None, loop=False, plot=False):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        super().__init__('file_publisher')
        self.publisher_audio = self.create_publisher(Audio, 'audio', 10)
        self.plot = plot

        self.i = 0
        self.loop = loop
        n_buffer = 2**8

        self.audio_data = {f:{} for f in filenames}
        for fname in filenames:
            self.audio_data[fname]['Fs'], self.audio_data[fname]['data'] = read(fname) 
        self.Fs = self.audio_data[filenames[0]]['Fs']
        self.len = len(self.audio_data[filenames[0]]['data'])
        assert all(self.audio_data[f]['Fs']==self.Fs for f in filenames)
        assert all(len(self.audio_data[f]['data'])==self.len for f in filenames)

        if self.plot:
            self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)

        if publish_rate is None:
            # by default, adapt publish rate to n_buffer (no samples missed)
            self.publish_rate = int(Fs // n_buffer)
        else:
            self.publish_rate = publish_rate
        self.publish_loop(n_buffer)

    def publish_loop(self, n_buffer):
        n_between_buffers = self.Fs // self.publish_rate
        seconds_between_buffers  = 1 / self.Fs * n_between_buffers

        while self.i+n_buffer < self.len:
            signals = np.c_[[
               data['data'][self.i:self.i+n_buffer] for data in self.audio_data.values()
            ]] # n_mics x n_samples
            msg, signals_f, frequencies = create_audio_message(signals, self.Fs)
            
            if self.plot: 
                labels = [f"mic{i}" for i in range(signals_f.shape[1])]
                self.plotter.update_animation(np.abs(signals_f.T), frequencies, labels)

            self.get_logger().debug(f'Publishing at {msg.timestamp}: frequencies {msg.frequencies}, \n {msg.real_vect}, {msg.imag_vect}')
        
            msg.timestamp = self.i
            self.publisher_audio.publish(msg)
            self.get_logger().info(f'Publishing at {msg.timestamp}: data from {msg.n_mics} mics.')
            self.i += n_between_buffers

            if self.loop and (self.i + n_buffer >= self.len):
                self.i = 0

            # TODO(FD): replace this with ROS sleep function
            time.sleep(seconds_between_buffers)


class StreamPublisher(Node):
    def __init__(self, publish_rate=1):
        super().__init__('stream_publisher')

        self.publish_rate = publish_rate
        self.publisher_audio = self.create_publisher(Audio, 'audio', 10)

        self.Fs = 42000
        self.i = 0
        n_buffer = 2**8
        n_mics = 4
        duration = 3 # in seconds

        sd.default.device = 'default'
        sd.check_input_settings(sd.default.device, channels=n_mics, samplerate=self.Fs)

        with sd.InputStream(channels=n_mics, callback=self.publish_loop, blocksize=n_buffer):
            sd.sleep(int(duration*1000)) # input is in miliseconds.

    def publish_loop(self, indata, frames, time_audio, status): 
        if status:
            print(status)
        n_between_buffers = self.Fs // self.publish_rate
        seconds_between_buffers  = 1 / self.Fs * n_between_buffers

        signals = indata.T
        msg = create_audio_message(signals, self.Fs)
        msg.timestamp = self.i
        self.publisher_audio.publish(msg)

        self.get_logger().info(f'Publishing: {msg.timestamp}')
        self.i += 1
        time.sleep(seconds_between_buffers)


def main(args=None):
    import os

    rclpy.init(args=args)

    data_dir = '/home/duembgen/Documents/PhD/Audio/data/simulated'

    #publisher = DummyPublisher()

    fnames = [
            os.path.join(data_dir, 'analytical_source_mic1.wav'),
            os.path.join(data_dir, 'analytical_source_mic2.wav'),
            os.path.join(data_dir, 'analytical_source_mic3.wav'),
            os.path.join(data_dir, 'analytical_source_mic4.wav'),
    ]
    publish_rate = 11 # in Hz
    loop = True # loop after file ends.
    plot = True
    publisher = FilePublisher(fnames, publish_rate=publish_rate, loop=loop, plot=plot)

    #publisher = StreamPublisher(publish_rate=1)

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
