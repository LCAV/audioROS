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

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from audio_interfaces.msg import Correlations, Spectrum, PoseRaw
from .beam_former import BeamFormer
from .topic_synchronizer import TopicSynchronizer


# Beamforming method, available: 
# - "das": delay-and-sum
# - "mvdr": minimum-variacne distortionless response
BF_METHOD = "das"

#NORMALIZE = "zero_to_one_all"
NORMALIZE = "zero_to_one"
#NORMALIZE = "sum_to_one"


def normalize_rows(matrix, method="zero_to_one"):
    if method == "zero_to_one":
        normalized =  (matrix - np.min(matrix, axis=1, keepdims=True)) / (np.max(matrix, axis=1, keepdims=True) - np.min(matrix, axis=1, keepdims=True))
        #assert np.max(normalized) == 1.0
        #assert np.min(normalized) == 0.0
    elif method == "zero_to_one_all":
        normalized =  (matrix - np.min(matrix)) / (np.max(matrix) - np.min(matrix))
        #assert np.max(normalized) == 1.0
        #assert np.min(normalized) == 0.0
    elif method == "sum_to_one":
        # first make sure values are between 0 and 1 (otherwise division can lead to errors)
        denom = np.max(matrix, axis=1, keepdims=True) - np.min(matrix, axis=1, keepdims=True)
        matrix =  (matrix - np.min(matrix, axis=1, keepdims=True)) / denom 
        sum_matrix = np.sum(matrix, axis=1, keepdims=True)
        normalized = matrix / sum_matrix
        np.testing.assert_allclose(np.sum(normalized, axis=1), 1.0, rtol=1e-5)
    else:
        raise ValueError(method)
    return normalized


def combine_rows(matrix, method="product", keepdims=False):
    if method == "product":
        # do the product in log domain for numerical reasons
        # sum(log10(matrix)) = log10(product(matrix))
        combined_matrix = np.power(10, np.sum(np.log10(matrix), axis=0, keepdims=keepdims))
    elif method == "sum":
        combined_matrix = np.sum(matrix, axis=0, keepdims=keepdims)
    else:
        raise ValueError(method)
    return combined_matrix


class SpectrumEstimator(Node):
    def __init__(self, plot=False):
        super().__init__("spectrum_estimator")

        self.subscription_correlations = self.create_subscription(
            Correlations, "audio/correlations", self.listener_callback_correlations, 10
        )

        self.raw_pose_synch = TopicSynchronizer(20)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 10)

        self.publisher_spectrum = self.create_publisher(Spectrum, "audio/spectrum", 10)

        self.beam_former = None

        # create ROS parameters that can be changed from command line.
        self.declare_parameter("bf_method")
        self.bf_method = BF_METHOD
        parameters = [
            rclpy.parameter.Parameter(
                "bf_method", rclpy.Parameter.Type.STRING, self.bf_method
            ),
        ]
        self.set_parameters_callback(self.set_params)
        self.set_parameters(parameters)

    def set_params(self, params):
        for param in params:
            if param.name == "bf_method":
                self.bf_method = param.get_parameter_value().string_value
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def listener_callback_correlations(self, msg_cor):
        t1 = time.time()

        n_frequencies = len(msg_cor.frequencies)
        if n_frequencies >= 2 ** 8:
            self.get_logger().error(f"too many frequencies to process: {n_frequencies}")
            return

        if self.beam_former is None:
            if msg_cor.mic_positions:
                mic_positions = np.array(msg_cor.mic_positions).reshape(
                    (msg_cor.n_mics, -1)
                )
                self.beam_former = BeamFormer(mic_positions)
            else:
                self.get_logger().error(
                    "need to set send mic_positions in Correlation to do DOA"
                )

        frequencies = np.array(msg_cor.frequencies).astype(np.float)  # [10, 100, 1000]
        real_vect = np.array(msg_cor.corr_real_vect)
        imag_vect = np.array(msg_cor.corr_imag_vect)

        R = (real_vect + 1j * imag_vect).reshape(
            (len(frequencies), msg_cor.n_mics, msg_cor.n_mics)
        )

        if self.bf_method == "mvdr":
            spectrum = self.beam_former.get_mvdr_spectrum(
                R, frequencies
            )  # n_frequencies x n_angles
        elif self.bf_method == "das":
            spectrum = self.beam_former.get_das_spectrum(
                R, frequencies
            )  # n_frequencies x n_angles
        else:
            raise ValueError(self.bf_method)

        message = self.raw_pose_synch.get_latest_message(msg_cor.timestamp, self.get_logger())
        if message is None:
            orientation = 0
        else:
            orientation = message.yaw_deg

        #spectrum = normalize_rows(spectrum, NORMALIZE)
        spectrum = normalize_each_row(spectrum, NORMALIZE)

        # publish
        msg_spec = Spectrum()
        msg_spec.timestamp = msg_cor.timestamp
        msg_spec.n_frequencies = n_frequencies
        msg_spec.n_angles = spectrum.shape[1]
        msg_spec.orientation = float(orientation)
        msg_spec.frequencies = list(frequencies)
        msg_spec.spectrum_vect = list(spectrum.flatten())
        self.publisher_spectrum.publish(msg_spec)

        t2 = time.time()
        processing_time = t2 - t1
        self.get_logger().info(f"Published spectrum after {processing_time:.2f}s.")


def main(args=None):
    import os

    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = current_dir + "/../../../crazyflie-audio/data/simulated"

    rclpy.init(args=args)

    estimator = SpectrumEstimator(plot=True)

    rclpy.spin(estimator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
