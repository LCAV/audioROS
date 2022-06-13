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
ANGLE_GRID = np.arange(360)

DIFF_STD = 3.0
ANGLE_STD = 10

MIC_IDX = [0, 1]


def get_diff_dict(distance, angle, prob_method):
    diff_dict = {}
    for mic_idx in MIC_IDX:
        delta_grid, probs = get_delta_distribution(
            distance, angle, mic_idx, prob_method
        )
        diff_dict[mic_idx] = (delta_grid, probs)
    return diff_dict


def measure_wall(pose):
    """
    simplified function for measuring the wall distance and angle in
    local coordinates. Works for this geometry only.
    """
    distance = DISTANCE_GLOB - pose[1]
    angle = ANGLE_GLOB - pose[2]
    angle %= 360
    return distance, angle

def get_angle_distribution(angle, prob_method="delta", scale=ANGLE_STD):
    if prob_method == "delta":
        probs = np.zeros(len(ANGLE_GRID))
        probs[np.argmin(np.abs(ANGLE_GRID - angle))] = 1.0
    elif prob_method == "normal":
        probs = norm.pdf(ANGLE_GRID, loc=angle, scale=ANGLE_STD)
    return ANGLE_GRID, probs

def get_delta_distribution(distance, angle, mic_idx, prob_method="delta", scale=DIFF_STD):
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
