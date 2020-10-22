#! /usr/bin/env python3
# -*- coding: utf-8 -*-

ALLOWED_LAG_MS = 20  # allowed lag between latest message and given timestamp

class TopicSynchronizer(object):
    """ Helper class to keep track of the latest message on a given topic within an admissible delay. """
    def __init__(self, allowed_lag=ALLOWED_LAG_MS):
        self.allowed_lag = allowed_lag
        self.latest_time_and_message = None

    def get_latest_message(self, timestamp, logger=None):
        """ Get latest message since timestamp-self.allow_lag, or None if it doesn't exist. """
        latest = self.latest_time_and_message
        if latest is None:
            return

        latest_time, latest_message = latest
        if (latest_time >= (timestamp - self.allowed_lag)):
            return latest_message
        elif (logger is not None):
            logger.warn(f"Did not register message in valid time window {timestamp-self.allowed_lag, timestamp}. Latest one: {latest_time}")
        return None

    def listener_callback(self, msg):
        self.latest_time_and_message = (msg.timestamp, msg)
