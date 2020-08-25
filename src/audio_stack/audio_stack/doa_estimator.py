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
from rcl_interfaces.msg import SetParametersResult

import matplotlib.pylab as plt
import numpy as np

from audio_interfaces.msg import Spectrum
from .live_plotter import LivePlotter

MAX_YLIM = 1 # set to inf for no effect.
MIN_YLIM = 1e-13 # set to -inf for no effect.

# for plotting only
MIN_FREQ = 400
MAX_FREQ = 600

class DoaEstimator(Node):
    def __init__(self):
        super().__init__('doa_estimator')

        self.subscription_spectrum = self.create_subscription(
            Spectrum, 'audio/spectrum', self.listener_callback_spectrum, 10)
        self.publisher_spectrum = self.create_publisher(Spectrum, 'audio/combined_spectrum', 10)
        self.spectrum_orientation_list = []

        self.plotter = LivePlotter(MAX_YLIM, MIN_YLIM)
        self.plotter.ax.set_xlabel('angle [rad]')
        self.plotter.ax.set_ylabel('magnitude [-]')

        # create ROS parameters that can be changed from command line.
        self.declare_parameter("combination_n")
        self.combination_n = 5
        self.declare_parameter("combination_method")
        self.combination_method = "sum"
        parameters = [
                rclpy.parameter.Parameter("combination_method", rclpy.Parameter.Type.STRING, self.combination_method),
                rclpy.parameter.Parameter("combination_n", rclpy.Parameter.Type.INTEGER, self.combination_n),
        ]
        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)


    def set_params(self, params):
        for param in params:
            if param.name == "combination_method":
                self.combination_method = param.get_parameter_value().string_value
            if param.name == "combination_n":
                self.combination_n = param.get_parameter_value().integer_value
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)


    def listener_callback_spectrum(self, msg_spec):
        self.get_logger().info(f'Processing spectrum: {msg_spec.timestamp}.')

        spectrum = np.array(msg_spec.spectrum_vect).reshape((msg_spec.n_frequencies, msg_spec.n_angles))
        self.spectrum_orientation_list.append((spectrum, msg_spec.orientation))

        # remove outdated spectra
        while len(self.spectrum_orientation_list) >= self.combination_n:
            self.spectrum_orientation_list.pop(0)

        # the combined spectrum is going to be in the coordinate frame of the latest 
        # spectrum.
        spectra_shifted = [self.spectrum_orientation_list[-1][0]] # latest element.
        o_ref = self.spectrum_orientation_list[-1][1] 
        for spectrum, orientation in self.spectrum_orientation_list[:-1]:
            # TODO(FD) do interpolation rather than nearest neighbor.
            # Note that index can be both positive and negative, both will work.
            index = round((orientation - o_ref) * (msg_spec.n_angles - 1) / 360)
            # TODO(FD) use a rolling buffer (linked list or so) instead of the copies here.
            spectra_shifted.append(np.c_[spectrum[:, index:], spectrum[:, :index]])

        if self.combination_method == "sum":
            combined_spectrum = np.sum(spectra_shifted, axis=0) # n_frequencies x n_angles
        elif self.combination_method == "product":
            combined_spectrum = np.product(spectra_shifted, axis=0) # n_frequencies x n_angles

        # publish
        msg_new = msg_spec
        msg_new.spectrum_vect = list(combined_spectrum.astype(float).flatten())
        self.publisher_spectrum.publish(msg_new)
        self.get_logger().info(f'Published spectrum.')

        # plot
        # TODO: read angles vector from topic? 
        theta_scan = np.linspace(0, 360, msg_new.n_angles)
        frequencies = np.array(msg_new.frequencies)
        assert len(frequencies) == combined_spectrum.shape[0]
        mask = (frequencies <= MAX_FREQ) & (frequencies >= MIN_FREQ)
        labels=[f"f={f:.0f}Hz" for f in frequencies[mask]]
        self.plotter.update_lines(combined_spectrum[mask], theta_scan, labels=labels)

def main(args=None):
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = current_dir + '/../../../crazyflie-audio/data/simulated'

    rclpy.init(args=args)

    estimator = DoaEstimator()

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
