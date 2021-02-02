#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stream_publisher.py: 
"""



import rclpy

import sounddevice as sd

from .publisher import AudioPublisher


N_MICS = 1
N_BUFFER = 2048


class StreamPublisher(AudioPublisher):
    def __init__(
        self, Fs, publish_rate=None, blocking=False, mic_positions=None
    ):
        super().__init__(
            "stream_publisher",
            mic_positions=mic_positions,
            publish_rate=publish_rate,
            Fs=Fs,
        )

        self.n_mics = N_MICS
        self.duration_ms = 100 * 1000

        sd.default.device = "default"
        sd.check_input_settings(
            sd.default.device, channels=self.n_mics, samplerate=self.Fs
        )

        n_buffer = self.current_params["n_buffer"]

        # blocking stream, more ROS-like, better for plotting. However might result in some lost samples
        if blocking:
            self.stream = sd.InputStream(channels=self.n_mics, blocksize=n_buffer)
            self.stream.start()
            self.create_timer(1.0 / self.publish_rate, self.publish_signals_timer)

        # non-blocking stream, less ROS-like, problematic for plotting. But we do not lose samples.
        else:
            with sd.InputStream(
                channels=self.n_mics, callback=self.publish_signals_callback, blocksize=n_buffer
            ) as stream:
                sd.sleep(self.duration_ms)
                # need below return or we will stay in this context forever
                return

    def publish_signals_callback(self, signals_T, frames, time_stream, status):
        self.get_logger().debug(f"buffer start time: {time_stream.inputBufferAdcTime}")
        self.get_logger().debug(f"currentTime: {time_stream.currentTime}")

        if status:
            print(status)

        self.process_signals(signals_T.T)

    def publish_signals_timer(self):
        n_buffer = self.current_params["n_buffer"]
        n_available = self.stream.read_available
        if n_buffer > n_available:
            self.get_logger().warn(
                f"Requesting more frames ({n_buffer}) than available ({n_available})"
            )
        signals_T, overflow = self.stream.read(n_buffer)  # frames x channels
        if overflow:
            self.get_logger().warn("overflow")

        self.process_signals(signals_T.T)


def main(args=None):
    rclpy.init(args=args)

    Fs = 44100
    blocking = False

    publisher = StreamPublisher(
        Fs=Fs,
        blocking=blocking,
    )

    rclpy.shutdown()
