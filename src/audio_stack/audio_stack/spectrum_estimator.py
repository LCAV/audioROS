#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spectrum_estimator.py: Calcualte spatial beamforming spectrum based on correlations.

"""
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np

from audio_interfaces.msg import Correlations, Spectrum, SignalsFreq, PoseRaw
from audio_interfaces_py.messages import create_spectrum_message, read_signals_freq_message 
from audio_stack.beam_former import BeamFormer, combine_rows, normalize_rows
from audio_stack.topic_synchronizer import TopicSynchronizer

# Beamforming method, available: 
# - "das": delay-and-sum
# - "mvdr": minimum-variance distortionless response
#BF_METHOD = "das"
BF_METHOD = "mvdr"

COMBINATION_N = 5 # number of spectra to combine
COMBINATION_METHOD = "sum" # way to combine spectra

#NORMALIZE = "zero_to_one_all"
#NORMALIZE = "zero_to_one"
NORMALIZE = "none"
#NORMALIZE = "sum_to_one"

class SpectrumEstimator(Node):
    def __init__(self, plot=False):
        super().__init__("spectrum_estimator")

        self.subscription_signals_f = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback_signals_f, 1
        )

        self.raw_pose_synch = TopicSynchronizer(allowed_lag=20)
        self.subscription = self.create_subscription(PoseRaw, "geometry/pose_raw", self.raw_pose_synch.listener_callback, 1)

        self.publisher_spectrum_raw = self.create_publisher(Spectrum, "audio/spectrum_raw", 10)
        self.publisher_spectrum_combined = self.create_publisher(Spectrum, "audio/spectrum_combined", 10)
        self.publisher_spectrum_multi = self.create_publisher(Spectrum, "audio/spectrum_multi", 10)

        self.beam_former = None

        # create ROS parameters that can be changed from command line.
        self.combination_n = COMBINATION_N
        self.combination_method = COMBINATION_METHOD
        self.bf_method = BF_METHOD

        self.declare_parameter("bf_method")
        self.declare_parameter("combination_n")
        self.declare_parameter("combination_method")
        parameters = [
            rclpy.parameter.Parameter(
                "bf_method", rclpy.Parameter.Type.STRING, self.bf_method
            ),
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
            if param.name == "bf_method":
                self.bf_method = param.get_parameter_value().string_value
            elif param.name == "combination_method":
                self.combination_method = param.get_parameter_value().string_value
            elif param.name == "combination_n":
                self.combination_n = param.get_parameter_value().integer_value
            else:
                return SetParametersResult(successful=False)

        self.beam_former = None
        return SetParametersResult(successful=True)

    def listener_callback_signals_f(self, msg):
        t1 = time.time()

        mic_positions, signals_f, frequencies = read_signals_freq_message(msg)

        signals_f = signals_f[frequencies > 0, :]
        frequencies = frequencies[frequencies > 0]

        if msg.n_frequencies >= 2 ** 8:
            self.get_logger().error(f"too many frequencies to process: {n_frequencies}")
            return

        if self.beam_former is None:
            if mic_positions is not None:
                self.beam_former = BeamFormer(mic_positions)
                self.beam_former.init_dynamic_estimate(frequencies, self.combination_n, self.combination_method)
            else:
                self.get_logger().error(
                    "need to set send mic_positions in Correlation to do DOA"
                )
        R = self.beam_former.get_correlation(signals_f)

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

        spectrum = normalize_rows(spectrum, NORMALIZE)

        # publish raw spectrum
        msg_spec = create_spectrum_message(spectrum, frequencies, msg.timestamp)
        self.publisher_spectrum_raw.publish(msg_spec)

        t2 = time.time()
        processing_time = t2 - t1
        self.get_logger().info(f"Published raw spectrum after {processing_time:.2f}s.")
        return

        #### multi-spectra
        #TODO(FD) fill in 

        #### combined specra
        pose_message = self.raw_pose_synch.get_latest_message(msg.timestamp, self.get_logger())
        if pose_message is None:
            return
        else:
            orientation = pose_message.yaw_deg

        # add latest spectrum and orientation to dynamic estimate
        self.beam_former.add_to_dynamic_estimates(spectrum, orientation)

        # retrieve current estimate
        dynamic_spectrum = self.beam_former.get_dynamic_estimate()
        dynamic_spectrum = normalize_rows(dynamic_spectrum, NORMALIZE)

        # publish
        msg_new = msg_spec
        msg_new.spectrum_vect = list(dynamic_spectrum.astype(float).flatten())
        self.publisher_spectrum_combined.publish(msg_new)
        self.get_logger().info(f"Published dynamic spectrum.")


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
