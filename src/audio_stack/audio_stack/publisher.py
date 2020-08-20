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

import sys
import time

import numpy as np
from scipy.io.wavfile import read, write

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import sounddevice as sd

from audio_interfaces.msg import Signals

N_MICS = 4

# Plotting parameters
MAX_YLIM = 1e5 # set to inf for no effect.
MIN_YLIM = -1e5 # set to -inf for no effect.

MAX_TIMESTAMP_INT = 2**32-1 # needs to match Spectrum.msg and Correlation.msg

# number of buffers to save, e.g. 256*1000=256'000 at 32000Hz: 8seconds
MAX_BUFFERS = 500  # set to 0 for no saving

OUT_DIR = "debug"

class AudioPublisher(Node):
    def __init__(self, name="audio_publisher", n_buffer=256, publish_rate=None, Fs=None):
        super().__init__(name)

        if publish_rate is None:
            # by default, adapt publish rate to n_buffer (no samples skipped)
            self.publish_rate = int(Fs / n_buffer)
        else:
            self.publish_rate = publish_rate
        self.n_buffer = n_buffer

        self.Fs = Fs
        if Fs is not None:
            self.set_Fs(Fs)

        self.publisher_signals = self.create_publisher(Signals, 'audio/signals', 10)
        self.time_idx = 0

        self.raw_data = np.empty((N_MICS, n_buffer*MAX_BUFFERS))

        # create ROS parameters that can be changed from command line.
        self.declare_parameter("n_buffer")
        self.declare_parameter("publish_rate")
        parameters = [
                rclpy.parameter.Parameter("n_buffer", rclpy.Parameter.Type.STRING, str(self.n_buffer)),
                rclpy.parameter.Parameter("publish_rate", rclpy.Parameter.Type.STRING, str(self.publish_rate))
        ]
        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)

    def set_params(self, params):
        for param in params:
            old_value = self.get_parameter(param.name).get_parameter_value().string_value
            str_value = param.get_parameter_value().string_value
            int_value = param.get_parameter_value().integer_value
            new_value = eval(str_value) if str_value != '' else int_value

            self.get_logger().info(f"changing {param.name} from {old_value} to {new_value}")
            if param.name == "n_buffer":
                self.n_buffer = new_value
                self.raw_data = np.empty((N_MICS, self.n_buffer*MAX_BUFFERS))
            elif param.name == "publish_rate":
                self.publish_rate = new_value

        return SetParametersResult(successful=True)

    def set_Fs(self, Fs):
        self.Fs = Fs
        self.n_between_buffers = self.Fs // self.publish_rate
        self.seconds_between_buffers  = 1 / self.Fs * self.n_between_buffers

    def process_signals(self, signals):
        n_buffer = signals.shape[1]

        t1 = time.time()

        assert self.Fs is not None, 'Need to set Fs before processing.'
        assert signals.shape[0] == N_MICS

        # publishing
        msg = Signals()
        msg.timestamp = self.time_idx
        msg.n_mics = N_MICS
        msg.n_buffer = n_buffer
        signals_vect = list(signals.flatten().astype(float))
        msg.signals_vect = signals_vect
        self.publisher_signals.publish(msg)

        if self.time_idx < MAX_BUFFERS:
            self.raw_data[:, self.time_idx*n_buffer:(self.time_idx+1)*n_buffer] = signals
        elif (self.time_idx == MAX_BUFFERS) and (MAX_BUFFERS > 0): 
            for i in range(self.raw_data.shape[0]):
                fname = f"{OUT_DIR}/test_mic{i}.wav" 
                write(fname, self.Fs, self.raw_data[i])
                self.get_logger().info(f"Saved audio as {fname}")

        self.time_idx += 1
        if self.time_idx >= MAX_TIMESTAMP_INT:
            self.get_logger().error('timestamp overflow.')
            self.time_idx = self.time_idx % MAX_TIMESTAMP_INT

        t2 = time.time()
        processing_time = t2-t1
        if processing_time > 1.0/self.publish_rate:
            self.get_logger().warn(f'processing time ({processing_time:.1e}) longer than publishing period ({1/self.publish_rate:.1e})')


class FilePublisher(AudioPublisher):
    def __init__(self, filenames=None, loop=False, n_buffer=256, publish_rate=None):
        """
        :param publish_rate: in Hz, at which rate to publish
        """
        super().__init__('file_publisher', n_buffer=n_buffer, publish_rate=publish_rate, Fs=None)
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

    def publish_loop(self):
        n_buffer = self.n_buffer

        signals = np.c_[[
           data['data'][self.file_idx:self.file_idx + n_buffer] for data in self.audio_data.values()
        ]] # n_mics x n_samples

        self.process_signals(signals)
 
        self.file_idx += self.n_between_buffers
        if self.file_idx + n_buffer >= self.len:
            if self.loop:
                self.file_idx = 0
            else:
                sys.exit()


class StreamPublisher(AudioPublisher):
    def __init__(self, Fs, n_buffer=256, publish_rate=None, blocking=False):
        super().__init__('stream_publisher', n_buffer=n_buffer, publish_rate=publish_rate, Fs=Fs)

        self.duration_ms = 100 * 1000
        self.n_mics = N_MICS

        sd.default.device = 'default'
        sd.check_input_settings(sd.default.device, channels=self.n_mics, samplerate=self.Fs)

        # blocking stream, more ROS-like, better for plotting. However might result in some lost samples 
        if blocking:
            self.stream = sd.InputStream(channels=self.n_mics, blocksize=n_buffer)
            self.stream.start()
            self.create_timer(1./self.publish_rate, self.publish_loop)

        # non-blocking stream, less ROS-like, problematic for plotting. But we do not lose samples. 
        else:
            with sd.InputStream(channels=self.n_mics, callback=self.publish_stream, blocksize=n_buffer) as stream:
                sd.sleep(self.duration_ms)
                # need below return or we will stay in this context forever
                return

    def publish_stream(self, signals_T, frames, time_stream, status): 
        self.get_logger().debug(f'buffer start time: {time_stream.inputBufferAdcTime}')
        self.get_logger().debug(f'currentTime: {time_stream.currentTime}')

        if status:
            print(status)

        self.process_signals(signals_T.T)

    def publish_loop(self): 
        n_buffer = self.n_buffer

        n_available = self.stream.read_available
        if n_buffer > n_available:
            self.get_logger().warn(f'Requesting more frames ({n_buffer}) than available ({n_available})')
        signals_T, overflow = self.stream.read(n_buffer) # frames x channels
        if overflow:
            self.get_logger().warn('overflow')

        self.process_signals(signals_T.T)


def main(args=None):
    import os

    rclpy.init(args=args)

    #audio_source = 'stream'
    audio_source = 'file'

    Fs = 44100
    n_buffer = 2**10
    publish_rate = int(Fs/n_buffer) #11 # in Hz
    blocking = False

    print(f'Publishing audio data from {audio_source} at {publish_rate}Hz.')

    if audio_source == 'file':
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.abspath(current_dir + '/../../../crazyflie-audio/data/simulated')
        fnames = [os.path.join(data_dir, f'analytical_source_mic{i}.wav') for i in range(1, 5)] 
        publisher = FilePublisher(fnames, n_buffer=n_buffer, publish_rate=publish_rate, loop=True)
        rclpy.spin(publisher)
    elif audio_source == 'stream':
        publisher = StreamPublisher(Fs=Fs, publish_rate=publish_rate, n_buffer=n_buffer, blocking=blocking)

    # Destroy the node explicitly
    rclpy.shutdown()
