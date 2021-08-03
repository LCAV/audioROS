#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
beam_former.py: Beamformer class for direction-of-arrival (DOA) estimation.
"""

import sys
import os


import numpy as np
from scipy.spatial.transform import Rotation

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir + "/../../../crazyflie-audio/python/")
from algos_beamforming import get_lcmv_beamformer_fast, get_das_beamformer, get_powers


LAMDA = 1  # 1e-10
INVERSE = "pinv"  # use standard pseudoinverse
# INVERSE = 'low-rank' # use own low-rank inverse (rank in each freq bin is assuemd one)


def normalize_rows(matrix, method):
    """ Normalizes last dimension of matrix (can be more than 2-dimensional) """
    if np.all(np.isnan(matrix)):
        print("Warning: not normalizeing, all nan.")
        return matrix

    if method == "zero_to_one":
        normalized = matrix - np.nanmin(matrix, axis=1, keepdims=True)
        denom = np.nanmax(matrix, axis=1, keepdims=True) - np.nanmin(
            matrix, axis=1, keepdims=True
        )
        if np.any(denom > 0):
            normalized /= denom
            np.testing.assert_allclose(np.nanmax(normalized, axis=1), 1)
            np.testing.assert_allclose(np.nanmin(normalized, axis=1), 0)
    elif method == "zero_to_one_all":
        denom = np.nanmax(matrix) - np.nanmin(matrix)
        if denom > 0:
            normalized = (matrix - np.nanmin(matrix)) / denom
            assert np.nanmax(normalized) == 1, np.nanmax(normalized)
            assert np.nanmin(normalized) == 0, np.nanmin(normalized)
    elif method == "sum_to_one":
        # first make sure values are between 0 and 1 (otherwise division can lead to errors)
        denom = np.nanmax(matrix, axis=1, keepdims=True) - np.nanmin(
            matrix, axis=1, keepdims=True
        )
        matrix = (matrix - np.nanmin(matrix, axis=1, keepdims=True)) / denom
        sum_matrix = np.sum(matrix, axis=1, keepdims=True)
        normalized = matrix / sum_matrix
        # np.testing.assert_allclose(np.sum(normalized, axis=1), 1.0, rtol=1e-5)
    elif method in ["none", None]:
        normalized = matrix
    else:
        raise ValueError(method)

    if np.any(np.isnan(normalized)):
        print("Warning: problem in normalization", method)
    return normalized


def combine_rows(matrix, method, keepdims=False):
    if method == "product":
        # do the product in log domain for numerical reasons
        # sum(log10(matrix)) = log10(product(matrix))
        combined_matrix = np.power(
            10, np.nansum(np.log10(matrix), axis=0, keepdims=keepdims)
        )
    elif method == "sum":
        combined_matrix = np.nansum(matrix, axis=0, keepdims=keepdims)
    else:
        raise ValueError(method)
    return combined_matrix


def rotate_mics(mics, orientation_deg=0):
    """
    :param mics: mic positions (n_mics, 2)
    :return mics_rotated: (n_mics, 2)
    """
    rot = Rotation.from_euler("z", orientation_deg, degrees=True)
    R = rot.as_matrix()  # 3 x 3
    mics_aug = np.c_[mics, np.ones(mics.shape[0])].T  # 3 x 4
    mics_rotated = R.dot(mics_aug)[:2, :]  # 2 x 4
    return mics_rotated.T


class BeamFormer(object):
    # TODO(FD): make this somewhat more flexible
    theta_scan_deg = np.linspace(0, 360, 361)
    theta_scan = theta_scan_deg * np.pi / 180.0

    def __init__(self, mic_positions=None):
        """
        :param mic_positions: array of mic positions, n_mics x dimension
        """
        self.mic_positions = mic_positions

        if mic_positions is not None:
            self.n_mics = mic_positions.shape[0]

        self.theta_scan = BeamFormer.theta_scan
        self.params = {}

    def get_mvdr_spectrum(
        self, R, frequencies_hz, mic_positions=None, lamda=LAMDA, inverse=INVERSE
    ):
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
                R,
                frequencies_hz,
                mic_positions,
                constraints,
                lamda=lamda,
                inverse=inverse,
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
            # print("Warning: less frequency bins than mics. Did you forget to transpose signals_f?")
            pass
        return (
            1
            / signals_f.shape[1]
            * signals_f[:, :, None]
            @ signals_f[:, None, :].conj()
        )

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

    def init_dynamic_estimate(
        self,
        frequencies,
        combination_n,
        combination_method,
        normalization_method="none",
    ):
        self.spectra_aligned = np.full(
            (combination_n, len(frequencies), len(BeamFormer.theta_scan)), np.nan
        )
        self.index_dynamic = 0

        # We do not really need to keep track of frequencies because the algorithm should also work when combine
        # different frequency bins. But we might want to issue a Warning at a later point.
        self.frequencies_dynamic = frequencies
        self.params["dynamic"] = dict(
            combination_n=combination_n,
            combination_method=combination_method,
            normalization_method=normalization_method,
        )

    def add_to_dynamic_estimates(self, spectrum, orientation_deg=0):
        """ Add new spectrum to list and remove outdated ones.

        :param spectrum: spatial spectrum of shape (n_frequencies, n_angles)
        :param orientation_deg: drone orientation_deg in degrees
        """
        self.spectra_aligned[self.index_dynamic, :, :] = self.shift_spectrum(
            spectrum, -orientation_deg
        )
        self.index_dynamic = (self.index_dynamic + 1) % self.params["dynamic"][
            "combination_n"
        ]
        return self.shift_spectrum(spectrum, -orientation_deg)

    def add_signals_to_dynamic_estimates(
        self, signals_f, frequencies, orientation_deg=0, method="das"
    ):
        """ Add new spectrum to list and remove outdated ones.

        :param orientation_deg: drone orientation_deg in degrees
        """
        R = self.get_correlation(signals_f)
        if method == "das":
            spectrum = self.get_das_spectrum(R, frequencies)
        elif method == "mvdr":
            spectrum = self.get_mvdr_spectrum(R, frequencies)
        return self.add_to_dynamic_estimates(spectrum, orientation_deg)

    def get_dynamic_estimate(self):
        """ Get current estimate

        :return: spectrum estimate of shape (n_angles,)
        """
        rows = ~np.all(np.isnan(self.spectra_aligned), axis=(1, 2))
        spectra_aligned = self.spectra_aligned[rows, ...]
        spectra_aligned = normalize_rows(
            spectra_aligned, method=self.params["dynamic"]["normalization_method"]
        )
        return combine_rows(
            spectra_aligned,
            self.params["dynamic"]["combination_method"],
            keepdims=False,
        )  # n_frequencies x n_angles

    def init_multi_estimate(self, frequencies, combination_n):
        self.frequencies_multi = frequencies
        self.signals_f_aligned = np.full(
            (len(frequencies), combination_n * self.mic_positions.shape[0]),
            np.nan,
            dtype=np.complex,
        )
        self.multi_mic_positions = np.full(
            (combination_n * self.mic_positions.shape[0], self.mic_positions.shape[1]),
            np.nan,
        )
        self.index_multi = 0
        self.params["multi"] = dict(combination_n=combination_n,)

    def add_to_multi_estimate(self, signals_f, frequencies, time_sec, orientation_deg):
        np.testing.assert_allclose(frequencies, self.frequencies_multi)

        # delay the signals according to recording times
        exp_factor = np.exp(-2j * np.pi * frequencies * time_sec)
        signals_f_aligned = np.multiply(
            signals_f, exp_factor[:, np.newaxis]
        )  # frequencies x n_mics

        n_mics = self.mic_positions.shape[0]
        self.signals_f_aligned[
            :, n_mics * self.index_multi : n_mics * (self.index_multi + 1)
        ] = signals_f_aligned

        # add the new microphone position according to "orientation"
        moved_mic_positions = rotate_mics(self.mic_positions, orientation_deg)
        self.multi_mic_positions[
            n_mics * self.index_multi : n_mics * (self.index_multi + 1), :
        ] = moved_mic_positions

        self.index_multi = (self.index_multi + 1) % self.params["multi"][
            "combination_n"
        ]

    def get_multi_R(self):
        if np.any(np.isnan(self.signals_f_aligned)):
            # print('Warning: not done yet filling self.signals_f_aligned')
            # need to filter out nan rows (not filled yet), because otherwise the rest of the method
            # will not work.
            signals_f_aligned = self.signals_f_aligned[
                :, np.all(~np.isnan(self.signals_f_aligned), axis=0)
            ]
        else:
            signals_f_aligned = self.signals_f_aligned
        return self.get_correlation(signals_f_aligned)

    def get_multi_estimate(self, method="mvdr", lamda=LAMDA):
        """ Get current estimate

        :return: spectrum of shape (n_frequencies x n_angles)
        """
        Rtot = self.get_multi_R()

        if np.any(np.isnan(self.multi_mic_positions)):
            multi_mic_positions = self.multi_mic_positions[
                np.all(~np.isnan(self.multi_mic_positions), axis=1), :
            ]
        else:
            multi_mic_positions = self.multi_mic_positions

        if method == "mvdr":
            return self.get_mvdr_spectrum(
                Rtot, self.frequencies_multi, multi_mic_positions, lamda=lamda
            )
        elif method == "das":
            return self.get_das_spectrum(
                Rtot, self.frequencies_multi, multi_mic_positions
            )
        else:
            raise ValueError(method)
