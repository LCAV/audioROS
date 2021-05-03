#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geometry.py:  
"""

import numpy as np
import sys, os

sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../"))

from geometry import *


def conversion(setup="random"):
    n_mics = 4
    dim = 2

    if setup == "random":
        context = Context.get_random_setup(n_mics=n_mics, dim=dim)
    elif setup == "crazyflie":
        context = Context.get_crazyflie_setup(dim=dim)
    elif setup == "standard":
        context = Context.get_standard_setup(dim=dim)

    for distance in context.get_possible_distances():
        for azimuth_deg in np.arange(0, 360, step=15):

            for mic_idx in range(n_mics):
                # get path difference from wall located at "normal"
                delta = context.get_delta(
                    azimuth_deg=azimuth_deg,
                    distances_cm=distance * 1e2,
                    mic_idx=mic_idx,
                )
                distance_source_m = context.get_source_distance(
                    delta, azimuth_deg, mic_idx
                )

                # get distance, given angle
                distance_total = context.get_total_distance(delta, azimuth_deg, mic_idx)
                np.testing.assert_allclose(distance_total, distance)

                # get angle, given distance_source
                azimuth_est = context.get_angles(delta, distance_source_m, mic_idx)
                assert azimuth_deg in azimuth_est, (azimuth_deg, azimuth_est)


def test_crazyflie():
    conversion("crazyflie")


def test_standard():
    conversion("standard")


def test_random():
    conversion("random")


if __name__ == "__main__":
    test_standard()
    test_crazyflie()
    test_random()
