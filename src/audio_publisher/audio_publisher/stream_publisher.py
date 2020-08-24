#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
stream_publisher.py: 
"""

import sys
import time

import numpy as np
from scipy.io.wavfile import read, write

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import sounddevice as sd

from audio_interfaces.msg import Signals
from .publisher import AudioPublisher

N_MICS = 4

class StreamPublisher(AudioPublisher):
    def __init__(self, Fs, n_buffer=256, publish_rate=None, blocking=False, mic_positions=None):
        super().__init__('stream_publisher', mic_positions=mic_positions, n_buffer=n_buffer, publish_rate=publish_rate, Fs=Fs)

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
    rclpy.init(args=args)

    Fs = 44100
    n_buffer = 2**10
    publish_rate = int(Fs/n_buffer) #11 # in Hz
    blocking = False

    print(f'Publishing audio data from stream at {publish_rate}Hz.')

    mic_positions = np.zeros((N_MICS, 2))

    publisher = StreamPublisher(Fs=Fs, publish_rate=publish_rate, n_buffer=n_buffer, blocking=blocking, mic_positions=mic_positions)

    rclpy.shutdown()
