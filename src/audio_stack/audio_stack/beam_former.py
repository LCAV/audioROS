#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
beam_former.py: Beamformer class for direction-of-arrival (DOA) estimation.
"""

from copy import deepcopy
import sys
import os


import numpy as np
from scipy.spatial.transform import Rotation

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from algos_beamforming import get_lcmv_beamformer_fast, get_das_beamformer, get_powers

def rotate_mics(mics, orientation_deg=0):
    """
    :param mics: mic positions (n_mics, 2)
    :return mics_rotated: (n_mics, 2)
    """
    rot = Rotation.from_euler('z', orientation_deg, degrees=True)
    R = rot.as_matrix() # 3 x 3
    mics_aug = np.c_[mics, np.ones(mics.shape[0])].T # 3 x 4
    mics_rotated = R.dot(mics_aug)[:2, :] # 2 x 4
    return mics_rotated.T

class BeamFormer(object):
    # TODO(FD): make this somewhat more flexible
    theta_scan = np.linspace(0, 360, 181) * np.pi / 180.0

    def __init__(self, mic_positions=None):
        """
        :param mic_positions: array of mic positions, n_mics x dimension
        """
        self.mic_positions = mic_positions

        if mic_positions is not None:
            self.n_mics = mic_positions.shape[0]

        self.theta_scan = BeamFormer.theta_scan

    def get_mvdr_spectrum(self, R, frequencies_hz, mic_positions=None):
        """ Get MVDR spatial spectrum.

        :param R: autocorrelation tensor (n_frequencies x n_mics x n_mics)
        :param frequencies_hz: list of frequencies (in Hz)

        :return: spectrum of shape (n_frequencies x n_angles)
        """
        if mic_positions is None:
            mic_positions = self.mic_positions
        spectrum = np.empty((len(frequencies_hz), len(self.theta_scan)))
        for i, theta in enumerate(self.theta_scan):
            constraints = [(theta, 1)]
            H_mvdr = get_lcmv_beamformer_fast(
                R, frequencies_hz, mic_positions, constraints, lamda=1e-3
            )
            spectrum[:, i] = get_powers(H_mvdr, R)
        return spectrum

    def get_das_spectrum(self, R, frequencies, mic_positions=None):
        """ Get DAS spatial spectrum.

        see get_mvdr_spectrum for parameters. 
        """
        if mic_positions is None:
            mic_positions = self.mic_positions

        spectrum = np.empty((len(frequencies), len(self.theta_scan)))
        for i, theta in enumerate(self.theta_scan):
            H_das = get_das_beamformer(theta, frequencies, mic_positions)
            spectrum[:, i] = get_powers(H_das, R)
        return spectrum

    def get_correlation(self, signals_f):
        """ Get autocorrelation tensor. 
        :param signals_f: frequency response (n_frequencies x n_mics)
        """
        if signals_f.shape[0] < signals_f.shape[1]:
            #print("Warning: less frequency bins than mics. Did you forget to transpose signals_f?")
            pass
        return 1 / signals_f.shape[1] * signals_f[:, :, None] @ signals_f[:, None, :].conj()

    def shift_spectrum(self, spectrum, delta_deg):
        """ shift spectrum by delta_deg. 

        :param spectrum: spatial spectrum (n_frequencies x n_angles)
        :param delta_deg: by how many angles to shfit the spectrum

        """
        # TODO(FD) do interpolation rather than nearest neighbor.
        # Note that index can be both positive and negative, both will work.
        n_angles = len(self.theta_scan) 
        index = int(round(delta_deg * (n_angles - 1) / 360))
        return np.c_[spectrum[:, index:], spectrum[:, :index]]

    def init_dynamic_estimate(self, combination_n, combination_method, normalization_method='zero_to_one'):
        self.spectra_shifted = []
        self.combination_n = combination_n
        self.combination_method = combination_method
        self.normalization_method = normalization_method

    def add_to_dynamic_estimates(self, spectrum, orientation_deg=0):
        """ Add new spectrum to list and remove outdated ones.

        :param spectrum: spatial spectrum of shape (n_frequencies, n_angles)
        :param orientation_deg: drone orientation_deg in degrees
        """
        self.spectra_shifted.append(self.shift_spectrum(spectrum, -orientation_deg))
        while len(self.spectra_shifted) > self.combination_n:
            self.spectra_shifted.pop(0)

    def get_dynamic_estimate(self):
        """ Get current estimate

        :return: spectrum estimate of shape (n_angles,)
        """
        from audio_stack.spectrum_estimator import combine_rows, normalize_rows
        if len(self.spectra_shifted) < self.combination_n:
            print(f"Warning: using only {len(self.spectra_shifted)}/{self.combination_n} spectra")

        spectra_shifted = normalize_rows(self.spectra_shifted, method=self.normalization_method)
        return combine_rows(spectra_shifted, self.combination_method, keepdims=False) # n_frequencies x n_angles

    def init_multi_estimate(self, frequencies):
        self.frequencies = frequencies
        self.signals_f_aligned = np.empty((len(frequencies), 0)) 
        self.multi_mic_positions = np.empty((0, self.mic_positions.shape[1]))

    def add_to_multi_estimate(self, signals_f, frequencies, time_sec, orientation_deg):
        np.testing.assert_allclose(frequencies, self.frequencies)

        # delay the signals according to recording times
        exp_factor = np.exp(1j * 2 * np.pi * frequencies * time_sec)
        signals_f_aligned = np.multiply(signals_f, exp_factor[:, np.newaxis])  # frequencies x n_mics
        self.signals_f_aligned = np.c_[self.signals_f_aligned, signals_f_aligned]

        # add the new microphone position according to "orientation" 
        moved_mic_positions = rotate_mics(self.mic_positions, orientation_deg)
        self.multi_mic_positions = np.r_[self.multi_mic_positions, moved_mic_positions]

    def get_multi_estimate(self, method='mvdr'):
        """ Get current estimate

        :return: spectrum of shape (n_frequencies x n_angles)
        """
        Rtot = self.get_correlation(self.signals_f_aligned)
        if method == 'mvdr':
            return self.get_mvdr_spectrum(Rtot, self.frequencies, self.multi_mic_positions)
        elif method == 'das':
            return self.get_das_spectrum(Rtot, self.frequencies, self.multi_mic_positions)
        else:
            raise ValueError(method)
