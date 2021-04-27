#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pyroom_helpers.py: Wrapper functions for real-time simulation of pyroomacoustics.
"""

from math import ceil
import time

import numpy as np


# TODO(FD) see if we can replace below with a suitable upper bound such as
# int(ceil(fs * (pyroom.max_order + 1) * np.linalg.norm(room_dim) / pyroom.c))
def get_max_delay(room, n_extra_samples=100):
    # compute image sources
    if not room.simulator_state["ism_done"]:
        room.image_source_model()

    t_max = 0  # maximum delay in seconds
    for mic in room.mic_array.R.T:
        for src in room.sources:

            # ignore silent sources in calculation
            if src.signal is not None and np.any(src.signal):
                dist = np.sqrt(np.sum((src.images - mic[:, None]) ** 2, axis=0))
                t_max = max(max(dist) / room.c, t_max)

    n_max = int(ceil(t_max * room.fs)) + n_extra_samples
    return n_max


def simulate_truncated(pyroom, start_idx, n_buffer, verbose=False):
    """ Simulate audio if we are interested in n_buffer samples starting from start_idx. """
    from copy import deepcopy

    max_delay = get_max_delay(pyroom)

    # assert max_delay <= start_idx, (max_delay, start_idx)

    min_idx = max(start_idx - max_delay, 0)
    for i in range(len(pyroom.sources)):
        signal = pyroom.sources[i].signal
        if signal is not None:
            max_idx = min(start_idx + n_buffer, signal.shape[0])
            assert max_idx > min_idx, f"max, {max_idx}, min, {min_idx}"
            pyroom.sources[i].signal = pyroom.sources[i].signal[min_idx:max_idx]

    t1 = time.time()
    pyroom.simulate()

    if verbose:
        print(f"length {max_idx - min_idx}: took {(time.time() - t1) * 1000:.0f}ms")

    simulated_signal = deepcopy(
        pyroom.mic_array.signals[:, max_delay : max_delay + n_buffer]
    )
    assert simulated_signal.shape[1] == n_buffer, f"{simulated_signal.shape}, {n_buffer}, {max_delay}, {min_idx}"
    return simulated_signal
