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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from audio_interfaces.msg import Audio, Spectrum

import matplotlib.pylab as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from .beam_former import BeamFormer
from .live_plotter import LivePlotter

current_dir = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = current_dir + '/../../../crazyflie-audio/data/simulated'
MAX_YLIM = 1 # set to inf for no effect.
MIN_YLIM = 1e-13 # set to -inf for no effect.

class PrintSubscriber(Node):
    def __init__(self):
        super().__init__('print_subscriber')
        self.subscription_message = self.create_subscription(
            String,
            'message',
            self.listener_callback_message,
            10)
        self.subscription_audio = self.create_subscription( Audio,
            'audio',
            self.listener_callback_audio,
            10)
        self.subscription_message # prevent unused variable warning
        self.subscription_audio # prevent unused variable warning

    def listener_callback_message(self, msg):
        self.get_logger().info(f'I heard a message: "{msg.data}"')

    def listener_callback_audio(self, msg):
        self.get_logger().info(f'I heard audio: {msg.timestamp}: {msg.real_vect[0, :2, :2]} etc.')


class PlotSubscriber(Node):
    def __init__(self, mic_positions, publish=False):
        super().__init__('raw_plot_subscriber')

        self.publish=publish

        self.subscription_audio = self.create_subscription(
            Audio, 'audio', self.listener_callback_audio, 10)
        self.subscription_audio # prevent unused variable warning

        if self.publish:
            self.publisher_spectrum = self.create_publisher(Spectrum, 'spectrum', 10)

        self.beam_former = BeamFormer(mic_positions)
        self.spectrum = None
        self.frequencies = None

        self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)

    def handle_close(self, evt):
        plt.close('all')

    def listener_callback_audio(self, msg_audio):
        self.get_logger().info(f'Processing audio: {msg_audio.timestamp}.')

        frequencies = np.array(msg_audio.frequencies).astype(np.float) #[10, 100, 1000]
        R = (np.array(msg_audio.real_vect) + 1j*np.array(msg_audio.imag_vect)).reshape((len(frequencies), msg_audio.n_mics, msg_audio.n_mics))

        self.spectrum = self.beam_former.get_mvdr_spectrum(R, frequencies) # n_frequencies x n_angles 

        labels=[f"f={frequencies[i]:.0f}Hz" for i in range(self.spectrum.shape[0])]
        self.plotter.update_animation(self.spectrum, self.beam_former.theta_scan, labels=labels)

        if not self.publish:
            return

        msg_spec = Spectrum()
        msg_spec.timestamp = msg_audio.timestamp
        msg_spec.n_frequencies = len(frequencies)
        msg_spec.frequencies = frequencies
        msg_spec.spectrum_vect = list(self.spectrum.flatten())
        self.publisher_spectrum.publish(msg_spec)
        self.get_logger().info(f'Published spectrum.')


def main(args=None):
    rclpy.init(args=args)

    #subscriber = PrintSubscriber()

    fname = os.path.join(DATA_DIR, 'analytical_mics.npy')
    mic_positions = np.load(fname)
    subscriber = PlotSubscriber(mic_positions)

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
