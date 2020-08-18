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

import rclpy
from rclpy.node import Node

from audio_interfaces.msg import Correlations, Spectrum

import matplotlib.pylab as plt
import numpy as np

from .beam_former import BeamFormer
from .live_plotter import LivePlotter

MAX_YLIM = 1 # set to inf for no effect.
MIN_YLIM = 1e-13 # set to -inf for no effect.


class DoaEstimator(Node):
    def __init__(self, mic_positions):
        super().__init__('doa_estimator')

        self.subscription_correlations = self.create_subscription(
            Correlations, 'audio/correlations', self.listener_callback_correlations, 10)
        self.subscription_correlations # prevent unused variable warning

        self.publisher_spectrum = self.create_publisher(Spectrum, 'audio/spectrum', 10)

        self.beam_former = BeamFormer(mic_positions)

        self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
        self.plotter.ax.set_xlabel('angle [rad]')
        self.plotter.ax.set_ylabel('magnitude [-]')


    def handle_close(self, evt):
        plt.close('all')

    def listener_callback_correlations(self, msg_correlations):
        self.get_logger().info(f'Processing correlations: {msg_correlations.timestamp}.')

        frequencies = np.array(msg_correlations.frequencies).astype(np.float) #[10, 100, 1000]
        R = (np.array(msg_correlations.corr_real_vect) + 1j*np.array(msg_correlations.corr_imag_vect)).reshape((len(frequencies), msg_correlations.n_mics, msg_correlations.n_mics))

        spectrum = self.beam_former.get_mvdr_spectrum(R, frequencies) # n_frequencies x n_angles 

        labels=[f"f={frequencies[i]:.0f}Hz" for i in range(spectrum.shape[0])]
        self.plotter.update_lines(spectrum, self.beam_former.theta_scan, labels=labels)

        msg_spec = Spectrum()
        msg_spec.timestamp = msg_correlations.timestamp
        msg_spec.n_frequencies = len(frequencies)
        msg_spec.frequencies = list(frequencies)
        msg_spec.spectrum_vect = list(spectrum.flatten())
        self.publisher_spectrum.publish(msg_spec)
        self.get_logger().info(f'Published spectrum.')


def main(args=None):
    import os
    import sys

    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = current_dir + '/../../../crazyflie-audio/data/simulated'

    rclpy.init(args=args)

    fname = os.path.abspath(os.path.join(data_dir, 'analytical_mics.npy'))
    try:
        mic_positions = np.load(fname)
    except FileNotFoundError:
        print(f'Did not find file {fname}.')
        print('You might have to run crazyflie-audio/python/DOASimulation.ipynb first.')
        sys.exit()

    estimator = DoaEstimator(mic_positions)

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
