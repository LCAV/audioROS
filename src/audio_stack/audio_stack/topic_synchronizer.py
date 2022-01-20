#! /usr/bin/env python3
# -*- coding: utf-8 -*-


from audio_interfaces_py.messages import convert_sec_nanosec_to_ms

ALLOWED_LAG_MS = 20  # allowed lag between latest message and given timestamp
N_BUFFER = 1  # number of data to keep, set to 1 for latest message only


def get_time(msg):
    if msg is None:
        return None
    try:
        return msg.timestamp
    except Exception as e:  # for messages without a timestamp
        print(e)
        stamp = msg.header.stamp
        return convert_sec_nanosec_to_ms(stamp.sec, stamp.nanosec)


class TopicSynchronizer(object):
    """ Helper class to keep track of the latest message on a given topic within an admissible delay. """

    def __init__(self, allowed_lag_ms=ALLOWED_LAG_MS, logger=None, n_buffer=N_BUFFER):
        self.allowed_lag_ms = allowed_lag_ms
        self.latest_message = None
        self.logger = logger
        self.n_buffer = n_buffer
        if n_buffer > 1:
            self.buffer = [None] * n_buffer
            self.buffer_idx = 0

    def get_latest_message(
        self, timestamp, logger=None, allow_reuse=True, verbose=False
    ):
        """ Get latest message since timestamp - self.allowed_lag_ms, or None if it doesn't exist. """
        if self.latest_message is None:
            return None
        if logger is None:
            logger = self.logger
        else:
            print("Depcrated use of logger, initialize class with this")

        latest_time = get_time(self.latest_message)

        min_time = max(timestamp - self.allowed_lag_ms, 0)

        # normal behavior:
        if timestamp > latest_time >= min_time:
            return self.latest_message
        # latest_time is already higher than requested timestamp. Need to go back in history.
        elif timestamp < latest_time:
            if verbose and logger:
                logger.warn(
                    f"Latest time {latest_time} bigger than requested {timestamp}."
                )
                logger.warn(f"All times currently in buffer: {self.get_times()}")
            if self.n_buffer > 1:
                min_idx = (self.buffer_idx - 1) % self.n_buffer
                max_idx = (self.buffer_idx - self.n_buffer) % self.n_buffer
                if verbose and logger:
                    logger.warn(
                        f"Checking {get_time(self.buffer[max_idx])} to {get_time(self.buffer[min_idx])}"
                    )
                # Starting with latest time (buffer_idx - 1), check until the time
                # is acceptable. Worse case, we return buffer_idx + 1, which is the
                # oldest element in the buffer.
                for i in range(1, self.n_buffer + 1):
                    idx = (self.buffer_idx - i) % self.n_buffer
                    time = get_time(self.buffer[idx])
                    if verbose and logger:
                        logger.warn(f"Checking {min_time} < {time} < {timestamp}")
                    if (time is not None) and (time < timestamp):
                        if verbose and logger:
                            logger.warn("ok")
                        break
                    elif (time is not None) and allow_reuse and (time == timestamp):
                        break

                    # example: [msg0, msg1, None0, None1, None2]
                    # check msg1 (idx 0), too new, msg0 (idx 1), too new, check None2: return msg0
                    elif time is None:
                        idx = (idx + 1) % self.n_buffer
                        break
                return self.buffer[idx]
            else:
                return self.latest_message
        elif latest_time == timestamp:
            if logger and verbose:
                logger.info(
                    f"Asking for message at same timestamp as before ({timestamp})."
                )
            return self.latest_message if allow_reuse else None

        if verbose and logger:
            logger.warn(
                f"Did not register message in valid time window {timestamp-self.allowed_lag_ms, timestamp}. Latest one: {latest_time}"
            )
            logger.warn(f"All times currently in buffer: {self.get_times()}")
        return None

    def listener_callback(self, msg):
        self.latest_message = msg

        if self.n_buffer > 1:
            self.buffer[self.buffer_idx] = msg
            self.buffer_idx = (self.buffer_idx + 1) % self.n_buffer

    def get_times(self):
        if self.n_buffer > 1:
            times = [get_time(msg) for msg in self.buffer]
            return times
