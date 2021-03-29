#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geometry.py:  
"""

import numpy as np
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../'))

from geometry import *

def test_conversion(setup='random'):
    n_mics = 4
    dim = 2
    for distance in range(3, 10): 
        for yaw_deg in np.arange(0, 360, step=15):
            normal = get_normal(distance, yaw_deg)[:dim]

            if setup == 'random':
                context = Context.get_random_setup(n_mics=n_mics, dim=dim)
            elif setup == 'crazyflie': 
                context = Context.get_crazyflie_setup(dim=dim)
            elif setup == 'standard': 
                context = Context.get_standard_setup(dim=dim)

            for mic_idx in range(n_mics):
                # get path difference from wall located at "normal"
                delta = context.get_delta_from_normal(normal, mic_idx)
                distance_source = context.get_source_distance(delta, yaw_deg, mic_idx)

                # get distance, given angle
                distance_total = context.get_total_distance(delta, yaw_deg, mic_idx) 
                assert np.allclose(distance_total, distance)

                # get angle, given distance_source
                yaw_est = context.get_angles(delta, distance_source, mic_idx)
                assert yaw_deg in yaw_est, (yaw_deg, yaw_est)


def test_crazyflie():
    test_conversion('crazyflie')

def test_standard(): 
    test_conversion('standard')
