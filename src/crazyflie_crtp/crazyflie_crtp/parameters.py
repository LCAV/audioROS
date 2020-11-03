#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters specific to the Crazyflie drone.
"""

import numpy as np

MIC_D = 0.108  # distance between mics (meters)
MIC_POSITIONS = MIC_D / 2 * np.c_[[1, -1], [-1, -1], [1, 1], [-1, 1]].T
N_MICS = 4
FS = 32000
N = 2048
