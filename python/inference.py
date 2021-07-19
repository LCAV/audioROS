#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
inference.py: Get probability distributions and estimates of angle or distance from distance-frequency measurements.
"""

import numpy as np

from crazyflie_description_py.experiments import WALL_ANGLE_DEG
from simulation import get_deltas_from_global

EPS = 1e-30
WALL_ANGLE_DEG = None


def eps_normalize(proba, eps=EPS):
    proba_norm = (proba - np.min(proba) + eps) / (np.max(proba) - np.min(proba) + eps)
    return proba_norm


def standardize_vec(d_slice):
    d_slice_norm = d_slice - np.nanmean(d_slice)
    std = np.nanstd(d_slice_norm)
    if std > 0:
        d_slice_norm /= std
    return d_slice_norm


class Inference(object):
    def __init__(self):
        self.slices = None  # mics x n_data
        self.values = None  # n_data
        self.stds = None  # n_data
        self.distance_range = None  # [min, max]
        self.is_calibrated = False
        self.calibration_function = None

    def add_data(self, slices, values, stds=None, distances=None):
        """
        :param slices: interference slices of shape (n_mics, n_values)
        :param values: values of shape (n_values, )
        :param stds: standard deviations (n_mics, )
        """
        assert slices.shape[1] == len(values), slices.shape
        self.slices = slices
        self.values = values
        self.stds = stds
        self.distances = distances
        self.valid_idx = np.ones(len(values), dtype=bool)
        self.is_calibrated = False

    def add_geometry(self, distance_range, azimuth_deg):
        self.distance_range = distance_range
        self.azimuth_deg = azimuth_deg

    def add_calibration_function(self, calibration_function):
        if calibration_function in [""]:
            self.calibration_function = None
        else:
            self.calibration_function = calibration_function

    def calibrate(self):
        if self.calibration_function is None:
            return
        valid_idx = (self.values >= min(self.calibration_function.x)) & (
            self.values <= max(self.calibration_function.x)
        )
        self.valid_idx &= valid_idx

        f_calib = self.calibration_function(self.values[self.valid_idx])
        self.slices[:, self.valid_idx] /= f_calib
        self.is_calibrated = True

    def filter_out_freqs(self, freq_ranges):
        """
        :param freq_ranges: list of frequency ranges to filter out
        """
        for freq_range in freq_ranges:
            self.valid_idx &= (freq_range[1] <= self.values) | (
                self.values <= freq_range[0]
            )

    def do_inference(self, algorithm="", mic_idx=0, calibrate=True, normalize=True):
        """
        Perform inference.


        """
        if calibrate and not self.is_calibrated:
            self.calibrate()

        valid = self.valid_idx & np.all(~np.isnan(self.slices), axis=0)

        if algorithm == "bayes":
            sigma = self.stds[mic_idx] if self.stds is not None else None
            dists, proba, diffs = get_probability_bayes(
                self.slices[mic_idx, valid],
                self.values[valid],
                mic_idx=mic_idx,
                distance_range=self.distance_range,
                sigma=sigma,
                azimuth_deg=self.azimuth_deg,
            )
        elif algorithm == "cost":
            distances = self.distances[valid] if self.distances is not None else None

            # make sure we use a reasonable distance range (need to add d0 to not
            # "go inside wall")
            __, d0 = get_deltas_from_global(self.azimuth_deg, np.array([100]), mic_idx)
            d0_cm = round(d0 * 1e2)
            dists = np.arange(self.distance_range[0] + d0_cm, self.distance_range[-1])
            diffs_m, __ = get_deltas_from_global(self.azimuth_deg, dists, mic_idx)
            diffs = diffs_m * 1e2
            proba = get_probability_cost(
                self.slices[mic_idx, valid],
                self.values[valid],
                dists,
                mic_idx=mic_idx,
                relative_ds=distances,
                absolute_yaws=None,
                azimuth_deg=self.azimuth_deg,
            )
        else:
            raise ValueError(algo_name)

        if normalize:
            proba = eps_normalize(proba)
        return dists, proba, diffs

    def plot(self, i_mic, ax, label=None, standardize=False, **kwargs):
        from copy import deepcopy

        slice_mic = deepcopy(self.slices[i_mic, self.valid_idx])
        if standardize:
            slice_mic = standardize_vec(slice_mic)

        ax.plot(
            self.values[self.valid_idx], slice_mic, label=label, **kwargs,
        )


def get_abs_fft(f_slice, n_max=1000, norm=True):
    if norm:
        f_slice_norm = f_slice - np.nanmean(f_slice)
    else:
        f_slice_norm = f_slice
    n = max(len(f_slice), n_max)
    return np.abs(np.fft.rfft(f_slice_norm, n=n))


def get_differences(frequencies, n_max=1000):
    from constants import SPEED_OF_SOUND

    n = max(len(frequencies), n_max)
    df = np.mean(frequencies[1:] - frequencies[:-1])
    deltas_cm = np.fft.rfftfreq(n, df) * SPEED_OF_SOUND * 100
    return deltas_cm


def convert_differences_to_distances(differences_cm, mic_idx, azimuth_deg):
    from geometry import get_orthogonal_distance_from_global

    distances = get_orthogonal_distance_from_global(
        azimuth_deg=azimuth_deg, deltas_cm=differences_cm, mic_idx=mic_idx
    )
    return distances


def get_posterior(abs_fft, sigma=None, data=None):
    N = len(abs_fft)
    periodogram = 1 / N * abs_fft ** 2
    # print('periodogram:', np.min(periodogram), np.max(periodogram))

    if sigma is not None:
        if np.any(sigma > 0):
            periodogram /= sigma ** 2
            # TODO(FD) we do below for numerical reasons. its effect
            # is undone by later exponentiation anyways. Make sure
            # this really as no effect on the result.
            periodogram -= np.max(periodogram)
            posterior = np.exp(periodogram)
        else:  # this is the limit of exp for sigma to 0
            posterior = np.zeros(len(periodogram))
            posterior[np.argmax(periodogram)] = 1.0
    else:
        d_bar = 1 / len(data) * np.sum((data - np.mean(data)) ** 2)
        arg = 1 - 2 * periodogram / (N * d_bar)

        # arg may not be negative.
        if np.any(arg <= 0):
            valid = np.where(arg > 0)[0]
            print("Warning, arg is non-positive at", arg[arg <= 0])
            arg[arg <= 0] = np.min(arg[arg > 0])

        posterior = (arg) ** ((2 - N) / 2)
        # posterior = np.exp(periodogram)
    return posterior


def get_probability_bayes(
    f_slice,
    frequencies,
    mic_idx=1,
    distance_range=None,
    n_max=1000,
    sigma=None,
    azimuth_deg=WALL_ANGLE_DEG,
    interpolate=True,
):
    assert f_slice.ndim == 1

    if interpolate:
        import scipy.interpolate

        frequencies_grid = np.arange(min(frequencies), max(frequencies), step=50.0)
        interpolator = scipy.interpolate.interp1d(frequencies, f_slice)
        f_slice_grid = interpolator(frequencies_grid)

        abs_fft = get_abs_fft(f_slice_grid, n_max=n_max, norm=True)
        differences = get_differences(frequencies_grid, n_max=n_max)
        posterior = get_posterior(abs_fft, sigma, data=f_slice_grid)
    else:
        abs_fft = get_abs_fft(f_slice, n_max=n_max, norm=True)

        # get path interference differences corresponding to used frequencies
        differences = get_differences(frequencies, n_max=n_max)

        # convert absolute fft to posterior (no correction yet!)
        posterior = get_posterior(abs_fft, sigma, data=f_slice)

    # convert differences to distances, for immediate evaluations.
    distances = convert_differences_to_distances(
        differences, mic_idx, azimuth_deg=azimuth_deg
    )

    if distance_range is not None:
        mask = (distances >= distance_range[0]) & (distances <= distance_range[1])
        distances = distances[mask]
        posterior = posterior[mask]
        differences = differences[mask]

    posterior /= np.sum(posterior)
    return distances, posterior, differences


def get_probability_cost(
    f_slice,
    frequencies,
    distances,
    mic_idx=1,
    azimuth_deg=WALL_ANGLE_DEG,
    relative_ds=None,
    absolute_yaws=None,
    ax=None,
):
    if np.any(distances < 1):
        raise ValueError("Reminder to change the distance input to include offset!")
    from simulation import get_freq_slice_theory

    if absolute_yaws is not None:
        azimuth_deg = azimuth_deg - absolute_yaws

    f_slice_norm = f_slice - np.mean(f_slice)
    f_slice_norm /= np.std(f_slice_norm)

    probs = []
    for d in distances:
        if relative_ds is not None:
            d += relative_ds
        f_slice_theory = get_freq_slice_theory(frequencies, d, azimuth_deg)[:, mic_idx]
        f_slice_theory -= np.mean(f_slice_theory)
        f_slice_theory /= np.std(f_slice_theory)
        probs.append(np.exp(-np.linalg.norm(f_slice_theory - f_slice_norm)))

        if ax is not None:
            ax.plot(frequencies, f_slice_theory, color="black")

    if ax is not None:
        ax.plot(frequencies, f_slice_norm, color="green")

    probs /= np.sum(probs)
    return probs


def get_periods_fft(
    d_slice, frequency, relative_distances_cm, n_max=1000, bayes=False, sigma=None,
):
    # the distribution over measured period.
    d_m = np.mean(relative_distances_cm[1:] - relative_distances_cm[:-1]) * 1e-2
    n = max(len(d_slice), n_max)

    # periods_m = np.fft.rfftfreq(n=n, d=d_m) # equivalent to below
    periods_m = (np.arange(0, n // 2 + 1)) / (d_m * n)  # 1/m in terms of orthogonal

    abs_fft = get_abs_fft(d_slice, n_max=1000, norm=True)
    if bayes:
        prob = get_posterior(abs_fft, sigma, d_slice)
        prob /= np.sum(prob)
    else:
        print(
            "Deprecation warning: do not use this function without bayes option anymore"
        )
        prob = abs_fft / np.sum(abs_fft)
    return periods_m, prob


def get_approach_angle_fft(
    d_slice,
    frequency,
    relative_distances_cm,
    n_max=1000,
    bayes=False,
    sigma=None,
    reduced=False,
    interpolate=True,
):
    """ 
    Get probabilities over approach angles.

    :param d_slice: amplitude measurements along distance
    :param frequency: operating frequency
    :param relative_distances_cm: relative distance measurements
    :param interpolate: interpolate measurements on 1cm-grid before inference.
    """
    from constants import SPEED_OF_SOUND

    period_theoretical = frequency / SPEED_OF_SOUND  # 1/m in terms of delta

    if interpolate:
        import scipy.interpolate

        relative_distances_cm_grid = np.arange(
            min(relative_distances_cm), max(relative_distances_cm), step=1.0
        )
        interpolator = scipy.interpolate.interp1d(relative_distances_cm, d_slice)
        d_slice_grid = interpolator(relative_distances_cm_grid)
        periods_m, probs = get_periods_fft(
            d_slice_grid, frequency, relative_distances_cm_grid, n_max, bayes, sigma
        )
    else:
        periods_m, probs = get_periods_fft(
            d_slice, frequency, relative_distances_cm, n_max, bayes, sigma
        )
    ratios = periods_m / period_theoretical
    if not reduced:
        return ratios, probs
    else:
        return get_gamma_distribution(ratios, probs)


def get_approach_angle_cost(
    d_slice,
    frequency,
    relative_distances_cm,
    start_distances_grid_cm,
    gammas_grid_deg,
    mic_idx=1,
    ax=None,
    azimuth_deg=WALL_ANGLE_DEG,
):
    from simulation import get_dist_slice_theory

    d_slice_norm = standardize_vec(d_slice)

    probs = np.zeros((len(start_distances_grid_cm), len(gammas_grid_deg)))
    for i, start_distance_cm in enumerate(start_distances_grid_cm):
        for j, gamma_deg in enumerate(gammas_grid_deg):
            distances_cm = start_distance_cm - relative_distances_cm * np.sin(
                gamma_deg / 180 * np.pi
            )
            assert np.all(distances_cm >= 0)
            d_slice_theory = get_dist_slice_theory(
                frequency, distances_cm, azimuth_deg
            )[:, mic_idx]

            d_slice_theory = standardize_vec(d_slice_theory)

            assert d_slice_theory.shape == d_slice_norm.shape
            loss = np.linalg.norm(d_slice_theory - d_slice_norm)
            probs[i, j] = np.exp(-loss)

            if ax is not None:
                ax.plot(
                    distances_cm,
                    d_slice_theory,
                    label=f"{start_distance_cm}cm, {gamma_deg}deg",
                )
    probs_angle = np.nanmax(probs, axis=0)  # take maximum across distances
    probs_angle /= np.nansum(probs_angle)
    return probs_angle


def get_gamma_distribution(ratios, probs, factor=2, eps=1e-1):
    ratios /= factor
    valid = ratios <= 1 + eps
    ratios[valid & (ratios > 1)] = 1.0
    probs[~valid] = 0
    return np.arcsin(ratios[valid]) * 180 / np.pi, probs[valid]
