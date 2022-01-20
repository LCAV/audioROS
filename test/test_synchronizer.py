#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_synchronizer.py:  
"""

import unittest
from audio_stack.topic_synchronizer import TopicSynchronizer
from audio_interfaces.msg import SignalsFreq


class Test(unittest.TestCase):
    def test_correct_timestamp(self):
        ts = TopicSynchronizer(allowed_lag_ms=2, n_buffer=10)
        for i in range(20):
            print(i)
            msg = SignalsFreq()
            msg.timestamp = i
            ts.listener_callback(msg)
            ts.print_times()

            msg = ts.get_latest_message(i)
            assert msg.timestamp == i

            # request a message from the past.
            # ===============================
            # at time 3, request time 0, etc.
            if i > 3:
                msg = ts.get_latest_message(i - 3)
                assert msg.timestamp == i - 3

            # at time 10, buffer is: [10, 1, 2, 3, 4, 5, 6, 7, 8, 9]
            # at time 11, buffer is: [10, 11, 2, 3, 4, 5, 6, 7, 8]
            # at time 12, buffer is: [10, 11, 12, 3, 4, 5, 6, 7, 8]
            if i > 10:
                msg = ts.get_latest_message(i - 10)  # at 10, ask 0, get 1.
                assert msg.timestamp == i - 10 + 1

            # at time 11, request time 0. latest time in buffer: 2
            # at time 12, request time 1. latest time in buffer: 3
            if i > 11:
                msg = ts.get_latest_message(i - 11)  # at 12, ask 1, get 3
                assert msg.timestamp == i - 11 + 2

            # at time 1, request time 2:
            msg = ts.get_latest_message(i + 1)
            assert msg.timestamp == i  # allowed lag

            # at time 1, request time 3:
            msg = ts.get_latest_message(i + 2)
            assert msg.timestamp == i  # allowed lag

            msg = ts.get_latest_message(i + 3)
            assert msg is None


if __name__ == "__main__":
    unittest.main()
