#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
base_estimator.py: Provides abstract base class for hisogram and particle filter classes. 
"""
import numpy as np
import scipy.interpolate

from .constants import PLATFORM
from .geometry import Context

import warnings


def from_0_to_360(angle):
    if np.ndim(angle) > 0:
        angle = np.mod(angle, 360)
        angle[angle < 0] += 360
    else:
        angle %= 360
        angle = angle + 360 if angle < 0 else angle
    return angle


def from_0_to_180(angle):
    if np.ndim(angle) > 0:
        angle = np.mod(angle, 360)  # between
        angle[angle >= 180] = 360 - angle[angle >= 180]
    else:
        angle %= 360
        angle = 360 - angle if angle > 180 else angle
    return angle


def get_std_sample(values, probs, means, unbiased=True):
    if np.isclose(np.nansum(probs), 1) or (
        np.nansum(probs) < 1
    ):  # for distributions (might be unnormalized)
        norm = 1.0
    else:  # for histograms
        norm = (
            (np.nansum(probs) - 1) if unbiased else np.nansum(probs)
        )  # for histograms
    if not norm:
        return None if np.ndim(means) == 0 else np.full(len(means), None)

    if np.ndim(means) > 0:
        coeffs = np.multiply(probs[:, None], (values[:, None] - means[None, :]) ** 2)
        var = np.nansum(coeffs, axis=0) / norm
    else:
        var = np.nansum(np.multiply(probs, (values - means) ** 2), axis=0) / norm
    return np.sqrt(var)


def get_std_of_peaks(values, probs, peaks):
    widths, *__ = scipy.signal.peak_widths(probs, peaks)
    fwhm = widths * (values[1] - values[0])  # assumes uniform values
    return fwhm / 2 / np.sqrt(2 * np.log(2))




def get_estimate(values, probs, method, unbiased=True):
    # distribution with only one value, return that value.
    if len(values) == 1:
        return values[0], 0.0

    if np.nansum(probs) == 0:
        print("Warning: all zeros")
        return None, None

    if method == "mean":
        mean = np.nansum(np.multiply(probs, values)) / np.nansum(probs)
        std = get_std_sample(values, probs, means=mean, unbiased=unbiased)
        return mean, std
    elif method == "max":
        max_prob = np.nanmax(probs)
        estimates = values[np.where(probs == max_prob)[0]]
        stds = get_std_sample(values, probs, means=estimates, unbiased=unbiased)
    elif method == "peak":
        indices, __ = scipy.signal.find_peaks(probs)
        if not len(indices):
            return None, None
        prob_estimates = probs[indices]
        max_prob = np.max(prob_estimates)
        indices = np.where(probs == max_prob)[0]
        estimates = values[indices]
        stds = get_std_of_peaks(values, probs, indices)
    else:
        raise ValueError(method)

    if len(estimates) == len(probs):
        print(f"Warning: uniform distribution?")
        return None, None
    elif len(estimates) > 1:
        pass
        # print(f"Warning: ambiguous distribution, {len(estimates)} maxima")
    elif not len(estimates):
        # print(f"Warning: no valid estimates detected: {values}, {probs}")
        return None, None
    return estimates[0], stds[0]


def get_estimates(values, probs, method, sort=True, n_estimates=None):
    """
    :param n_estimates: number of estimates (i.e. peaks) to return. None: return all.
    :param sort: sort estimates by probability (most likely first).
    """
    if n_estimates == 1:
        mean, std = get_estimate(values, probs, method=method)
        return [mean], [std]
    if n_estimates is None or n_estimates > 1:
        if method != "peak":
            warnings.warn(
                f"Using method peak instead of {method} for n_estimates={n_estimates}"
            )

        indices, properties = scipy.signal.find_peaks(probs, width=True)
        if n_estimates and len(indices) < n_estimates:
            print(f"Warning: found less peaks than requested: {len(indices)}")

        stds = get_std_of_peaks(values, probs, indices)
        estimates = values[indices]
        prob_estimates = probs[indices]
        if sort:
            sort_indices = prob_estimates.argsort()[::-1]
            estimates = estimates[sort_indices]
            prob_estimates = prob_estimates[sort_indices]
            stds = stds[sort_indices]

        if n_estimates is None:
            return estimates, stds
        else:
            return estimates[:n_estimates], stds[:n_estimates]


def get_normal_vector(angle_deg):
    return np.r_[
        np.cos(angle_deg / 180 * np.pi),
        np.sin(angle_deg / 180 * np.pi),
    ]

def get_normal_matrix(angles_vec):
    """return N x 2"""
    return np.c_[
        np.cos(angles_vec / 180 * np.pi), np.sin(angles_vec / 180 * np.pi)
    ]


class BaseEstimator(object):
    def __init__(
        self,
        n_window=2,
        platform=PLATFORM,
    ):
        self.difference_p = {n: {} for n in range(n_window)}
        self.angle_probs = {n: None for n in range(n_window)}
        self.positions = np.full((n_window, 2), None)
        self.rotations = np.full(n_window, None)
        self.times = np.full(n_window, None)

        self.dim = 2
        self.reference = None

        self.counter = 0  # total counter
        self.index = -1  # rolling index
        self.n_window = n_window
        self.filled = False

        self.context = Context.get_platform_setup(platform)

    def add_distributions(
        self,
        diff_dictionary,
        position_cm=[0, 0],
        rot_deg=0,
    ):
        """
        diff_dictionary:
        {
            mic_idx1: (diff1_cm, prob1),
            mic_idx2: (diff2_cm, prob2),
            ...
        }
        """
        if len(position_cm) > 2:
            position_cm = position_cm[:2]

        self.index += 1
        if self.index == self.n_window - 1:
            self.filled = True
        elif self.index == self.n_window:  # rolling window
            self.index = 0

        self.positions[self.index, :] = position_cm
        self.rotations[self.index] = rot_deg
        self.times[self.index] = self.counter
        self.counter += 1

        for mic_idx, (diff, prob) in diff_dictionary.items():
            if not any(diff > 1):
                print("Warning: need to pass diff in centimeters")
            self.difference_p[self.index][mic_idx] = scipy.interpolate.interp1d(
                diff.astype(float),
                prob.astype(float),
                fill_value=0,
                bounds_error=False,
                kind="linear",
            )

    def add_angle_distribution(self, angles, probs):
        self.angle_probs[self.index] = scipy.interpolate.interp1d(
            angles, probs, fill_value=0, bounds_error=False, kind="linear"
        )

    def enough_measurements(self):
        return self.filled

    def get_ordered_index(self):
        """Get the list of indices that the current elements in the buffer correspond to.
        For instance, if self.positions contains: [p7 p1 p2 p3 p4 p5 p6]
        it returns ordered_index=[6, 0, 1, 2, 3, 4, 5], such that positions[ordered_index]
        corresponds to the positions ordered chronologically: [p1, p2, p3, p4, p5, p6, p7],
        with the oldest position first.
        """
        if self.filled:
            n_elements = self.n_window
        else:
            n_elements = self.index + 1
        ordered_index = np.roll(range(n_elements), -self.index - 1)
        return ordered_index

    def get_forward_angle(self):
        # TODO(FD) make this more general.
        if self.positions[self.index][1] < 0:
            return 270
        else:
            return 90

    def get_local_forward_angle(self):
        current = self.index
        if not self.filled and (self.index < 2):
            # print("Warning: cannot simplify angles yet because buffer not filled.")
            return 0
        # if self.positions is [p5, p6, p2, p3, p4] with current=2, return [p2, p3, p4, p5, p6]
        ordered_positions = self.positions[self.get_ordered_index()]

        forward_dir_global = np.mean(np.diff(ordered_positions, axis=0), axis=0)
        angle_global_deg = (
            180 / np.pi * np.arctan2(forward_dir_global[1], forward_dir_global[0])
        )
        angle_local_deg = angle_global_deg - self.rotations[current]
        return angle_local_deg
