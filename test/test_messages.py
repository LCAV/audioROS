#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_messages.py:  
"""

import time

import unittest
from audio_interfaces_py.messages import *


class Test(unittest.TestCase):
    def setUp(self):
        pass

    def test_timestamp(self):
        timestamp_ms = int(round(time.time() * 1e3))
        msg = create_pose_message_from_arrays(
            np.zeros(4), np.zeros(3), timestamp=timestamp_ms
        )
        timestamp = convert_stamp_to_ms(msg.header.stamp)
        self.assertEqual(timestamp, timestamp_ms)

    def test_multiple(self):
        for i in range(100):
            self.test_timestamp()


if __name__ == "__main__":
    unittest.main()
