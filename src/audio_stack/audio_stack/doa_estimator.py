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

from audio_interfaces.msg import Spectrum, DoaEstimates
from audio_stack.beam_former import BeamFormer
from .spectrum_estimator import normalize_rows, combine_rows, NORMALIZE

N_ESTIMATES = 3 # number of peaks to detect
COMBINATION_N = 5 # number of spectra to combine
COMBINATION_METHOD = "product" # way to combine spectra


class DoaEstimator(Node):
    def __init__(self):
        super().__init__("doa_estimator")

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum", self.listener_callback_spectrum, 10
        )
        self.publisher_spectrum = self.create_publisher(
            Spectrum, "audio/dynamic_spectrum", 10
        )
        self.publisher_doa = self.create_publisher(
            DoaEstimates, "geometry/doa_estimates", 10
        )
        # all in degrees:
        self.spectrum_orientation_list = []

        self.beam_former = BeamFormer()

        # create ROS parameters that can be changed from command line.
        self.combination_n = COMBINATION_N
        self.combination_method = COMBINATION_METHOD
        self.declare_parameter("combination_n")
        self.declare_parameter("combination_method")
        parameters = [
            rclpy.parameter.Parameter(
                "combination_method",
                rclpy.Parameter.Type.STRING,
                self.combination_method
            ),
            rclpy.parameter.Parameter(
                "combination_n", 
                rclpy.Parameter.Type.INTEGER, 
                self.combination_n 
            ),
        ]
        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)


    def set_params(self, params):
        for param in params:
            if param.name == "combination_method":
                self.combination_method = param.get_parameter_value().string_value
            elif param.name == "combination_n":
                self.combination_n = param.get_parameter_value().integer_value
            else:
                return SetParametersResult(successful=False)

        self.beam_former.init_dynamic_estimate(self.combination_n, self.combination_method)
        return SetParametersResult(successful=True)

    def listener_callback_spectrum(self, msg_spec):

        spectrum, *_ = read_spectrum_message(msg_psec)
        # add latest spectrum and orientation to dynamic estimate
        self.beam_former.add_to_dynamic_estimates(spectrum, msg_spec.orientation)

        # retrieve current estimate
        dynamic_spectrum = self.beam_former.get_dynamic_estimate()
        dynamic_spectrum = normalize_rows(dynamic_spectrum, NORMALIZE)

        # publish
        msg_new = msg_spec
        msg_new.spectrum_vect = list(dynamic_spectrum.astype(float).flatten())
        self.publisher_spectrum.publish(msg_new)
        self.get_logger().info(f"Published dynamic spectrum.")

        # calculate and publish doa estimates
        final_spectrum = combine_rows(dynamic_spectrum, self.combination_method, keepdims=True)
        final_spectrum = normalize_rows(final_spectrum, NORMALIZE)

        angles = np.linspace(0, 360, msg_spec.n_angles)
        sorted_indices = np.argsort(final_spectrum.flatten()) # sorts in ascending order
        doa_estimates = angles[sorted_indices[-N_ESTIMATES:][::-1]]

        msg_doa = create_doa_message(doa_estimates, msg_spec.timestamp)
        self.publisher_doa.publish(msg_doa)
        self.get_logger().info(f"Published doa estimates: {doa_estimates}.")


def main(args=None):
    import os

    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = current_dir + "/../../../crazyflie-audio/data/simulated"

    rclpy.init(args=args)

    estimator = DoaEstimator()

    rclpy.spin(estimator)

    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
