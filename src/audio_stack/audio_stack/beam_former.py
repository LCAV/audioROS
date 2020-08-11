#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
beam_former.py: 
"""

import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + '/../../../crazyflie-audio/python/')

import numpy as np

from algos_beamforming import get_lcmv_beamformer, get_das_beamformer, get_powers

class BeamFormer(object):
    def __init__(self, mic_positions): 
        self.mic_positions = mic_positions
        self.n_mics = mic_positions.shape[0]

        self.theta_scan = np.linspace(0, 360, 181) * np.pi / 180.

    def get_mvdr_spectrum(self, R, frequencies_hz): 
        """ Get MVDR spatial spectrum.

        :param R: autocorrelation tensor (n_frequencies x n_mics x n_mics)
        :param frequencies_hz: list of frequencies (in Hz)
        """
        spectrum = np.empty((len(frequencies_hz), len(self.theta_scan)))
        for i, theta in enumerate(self.theta_scan): 
            constraints = [(theta, 1)]
            H_mvdr, *_ = get_lcmv_beamformer(R, frequencies_hz, self.mic_positions, constraints, lamda=1e-3)
            spectrum[:, i] = get_powers(H_mvdr, R)
        return spectrum

    def get_das_spectrum(self, R, frequencies): 
        """ Get DAS spatial spectrum.

        see get_mvdr_spectrum for parameters. 
        """
        spectrum = np.empty((len(frequencies), len(self.theta_scan)))
        for i, theta in enumerate(self.theta_scan): 
            H_das = get_das_beamformer(theta, frequencies, self.mic_positions)
            spectrum[:, i] = get_powers(H_das, R)
        return spectrum
