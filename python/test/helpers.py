#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
helpers.py: Functions common to particle and moving estimator tests.
"""

import numpy as np
from scipy.stats import norm

from utils.geometry import get_deltas_from_global

DISTANCE_GLOB = 30
ANGLE_GLOB = 90

DELTA_GRID = np.arange(100)


def measure_wall(pose):
    """ 
    simplified function for measuring the wall distance and angle in 
    local coordinates. Works for this geometry only.
    """
    distance = DISTANCE_GLOB - pose[1]
    angle = ANGLE_GLOB - pose[2]
    angle %= 360
    return distance, angle


def get_delta_distribution(distance, angle, mic_idx, prob_method="delta", scale=10):
    delta = (
        get_deltas_from_global(
            azimuth_deg=angle, distances_cm=distance, mic_idx=mic_idx
        )[0]
        * 1e2
    )
    if prob_method == "delta":
        probs = np.zeros(len(DELTA_GRID))
        probs[np.argmin(np.abs(DELTA_GRID - delta))] = 1.0
    elif prob_method == "normal":
        probs = norm.pdf(DELTA_GRID, loc=delta, scale=scale)
    return DELTA_GRID, probs
