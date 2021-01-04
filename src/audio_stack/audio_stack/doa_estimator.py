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
from audio_interfaces_py.messages import read_spectrum_message, create_doa_message
from audio_stack.beam_former import combine_rows, normalize_rows

N_ESTIMATES = 3 # number of peaks to detect
COMBINATION_METHOD = "sum" # way to combine across frequencies
NORMALIZE = "zero_to_one"

class DoaEstimator(Node):
    def __init__(self):
        super().__init__("doa_estimator")

        self.subscription_spectrum = self.create_subscription(
            Spectrum, "audio/spectrum_combined", self.listener_callback_spectrum, 10
        )

        self.publisher_doa = self.create_publisher(
            DoaEstimates, "geometry/doa_estimates", 10
        )

    def listener_callback_spectrum(self, msg_spec):
        spectrum, *_ = read_spectrum_message(msg_spec)

        # calculate and publish doa estimates
        final_spectrum = combine_rows(spectrum, COMBINATION_METHOD, keepdims=True)
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
