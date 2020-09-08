#! /usr/bin/env python3
# -*- coding: utf-8 -*-

ALLOWED_LAG_MS = 20  # allowed lag between latest message and given timestamp

class TopicSynchronizer(object):
    """ Helper class to keep track of the latest message on a given topic within an admissible delay. """
    def __init__(self, allowed_lag=ALLOWED_LAG_MS):
        self.allowed_lag = allowed_lag
        self.latest_time_and_message = None

    def get_latest_message(self, timestamp):
        """ Get latest message since timestamp-self.allow_lag, or None if it doesn't exist. """
        latest = self.latest_time_and_message
        if (latest is not None) and (abs(timestamp - latest[0]) < self.allowed_lag):
            return latest[1]
        return None

    def listener_callback(self, msg):
        self.latest_time_and_message = (msg.timestamp, msg)

