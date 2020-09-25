#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
beam_former.py: 
"""

import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")

import numpy as np

from algos_beamforming import get_lcmv_beamformer_fast, get_das_beamformer, get_powers


class BeamFormer(object):
    def __init__(self, mic_positions=None):
        """
        :param mic_positions: array of mic positions, n_mics x dimension
        """
        self.mic_positions = mic_positions

        if mic_positions is not None:
            self.n_mics = mic_positions.shape[0]

        self.theta_scan = np.linspace(0, 360, 181) * np.pi / 180.0

    def get_mvdr_spectrum(self, R, frequencies_hz):
        """ Get MVDR spatial spectrum.

        :param R: autocorrelation tensor (n_frequencies x n_mics x n_mics)
        :param frequencies_hz: list of frequencies (in Hz)
        """
        spectrum = np.empty((len(frequencies_hz), len(self.theta_scan)))
        for i, theta in enumerate(self.theta_scan):
            constraints = [(theta, 1)]
            H_mvdr = get_lcmv_beamformer_fast(
                R, frequencies_hz, self.mic_positions, constraints, lamda=1e-3
            )
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

    def get_correlation(self, signals_f):
        """ Get autocorrelation tensor. 
        :param signals_f: frequency response (n_frequencies x n_mics)
        """
        if signals_f.shape[0] < signals_f.shape[1]:
            print("Warning: less frequency bins than mics. Did you forget to transpose signals_f?")
        return 1 / signals_f.shape[1] * signals_f[:, :, None] @ signals_f[:, None, :].conj()

    def init_dynamic_estimate(self, combination_n, combination_method, normalization_method='zero_to_one'):
        self.spectrum_orientation_list = []
        self.combination_n = combination_n
        self.combination_method = combination_method
        self.normalization_method = normalization_method

    def add_to_dynamic_estimates(self, spectrum, orientation=0):
        """ Add new spectrum to list and remove outdated ones.

        :param spectrum: spatial spectrum of shape (n_frequencies, n_angles)
        :param orientation: drone orientation in degrees
        """
        self.spectrum_orientation_list.append((spectrum, orientation))
        while len(self.spectrum_orientation_list) > self.combination_n:
            self.spectrum_orientation_list.pop(0)

    def get_dynamic_estimate(self):
        """ Get current estimate

        :return: spectrum estimate of shape (n_angles,)
        """
        from audio_stack.spectrum_estimator import combine_rows, normalize_rows

        # the combined spectrum is going to be in the coordinate frame of the latest
        # spectrum.
        spectra_shifted = [self.spectrum_orientation_list[-1][0]]  # latest element.
        o_ref = self.spectrum_orientation_list[-1][1]
        n_angles = spectra_shifted[0].shape[1]

        if len(self.spectrum_orientation_list) < self.combination_n:
            print(f"Warning: using only {len(self.spectrum_orientation_list)}/{self.combination_n}")

        for spectrum, orientation in self.spectrum_orientation_list[:-1]:
            # TODO(FD) do interpolation rather than nearest neighbor.
            # Note that index can be both positive and negative, both will work.
            index = round((orientation - o_ref) * (n_angles - 1) / 360)
            spectra_shifted.append(np.c_[spectrum[:, index:], spectrum[:, :index]])

        spectra_shifted = normalize_rows(spectra_shifted, method=self.normalization_method)
        return combine_rows(spectra_shifted, self.combination_method, keepdims=False) # n_frequencies x n_angles
