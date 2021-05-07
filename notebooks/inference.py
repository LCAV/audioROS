#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
inference.py: Get probability distributions and estimates of angle or distance from distance-frequency measurements.
"""

import numpy as np

from crazyflie_description_py.experiments import WALL_ANGLE_DEG

EPS = 1e-30


class Inference(object):
    def __init__(self):
        self.slices = None  # mics x n_data
        self.values = None  # n_data
        self.stds = None  # n_data
        self.distance_range = None
        self.is_calibrated = False

    def add_data(self, slices, values, stds=None, distances=None):
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
        self.calibration_function = calibration_function

    def calibrate(self):
        valid_idx = (self.values >= min(self.calibration_function.x)) & (
            self.values <= max(self.calibration_function.x)
        )
        self.valid_idx &= valid_idx

        f_calib = self.calibration_function(self.values[self.valid_idx])
        self.slices[:, self.valid_idx] /= f_calib
        self.is_calibrated = True

    def filter_out_freqs(self, freq_range):
        valid_idx = (freq_range[1] <= self.values) | (self.values <= freq_range[0])
        self.valid_idx &= valid_idx

    def do_inference(self, algorithm="", mic_idx=0, calibrate=True, normalize=True):

        if calibrate and not self.is_calibrated:
            self.calibrate()

        valid = self.valid_idx & np.all(~np.isnan(self.slices), axis=0)

        sigma = self.stds[mic_idx] if self.stds is not None else None

        if algorithm == "bayes":
            dists, proba, diffs = get_probability_bayes(
                self.slices[mic_idx, valid],
                self.values[valid],
                mic_idx=mic_idx,
                distance_range=self.distance_range,
                sigma=sigma,
                azimuth_deg=self.azimuth_deg,
            )
        else:
            raise ValueError(algo_name)

        if normalize:
            proba = (proba - np.min(proba) + EPS) / (
                np.max(proba) - np.min(proba) + EPS
            )
        return dists, proba, diffs

    def plot(self, i_mic, ax, label=None, **kwargs):
        ax.plot(
            self.values[self.valid_idx],
            self.slices[i_mic, self.valid_idx],
            label=label,
            **kwargs,
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


def convert_differences_to_distances(
    differences_cm, mic_idx, distance_range, azimuth_deg
):
    from geometry import get_orthogonal_distance_from_global

    distances = get_orthogonal_distance_from_global(
        azimuth_deg=azimuth_deg, deltas_cm=differences_cm, mic_idx=mic_idx
    )
    mask = None
    if distance_range is not None:
        mask = (distances >= distance_range[0]) & (distances <= distance_range[1])
        distances = distances[mask]
    return distances, mask


def get_probability_fft(
    f_slice,
    frequencies,
    mic_idx=1,
    distance_range=None,
    n_max=1000,
    azimuth_deg=WALL_ANGLE_DEG,
):
    print("Deprecation warning: do not use this function anymore")
    assert f_slice.ndim == 1
    abs_fft = get_abs_fft(f_slice, n_max)
    differences = get_differences(frequencies, n_max=n_max)

    distances, mask = convert_differences_to_distances(
        differences, mic_idx, distance_range, azimuth_deg=azimuth_deg
    )
    if mask is not None:
        abs_fft = abs_fft[mask]
        differences = differences[mask]

    prob = abs_fft / np.sum(abs_fft)
    return distances, prob, differences


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
):
    assert f_slice.ndim == 1
    abs_fft = get_abs_fft(f_slice, n_max=n_max, norm=True)
    differences = get_differences(frequencies, n_max=n_max)
    posterior = get_posterior(abs_fft, sigma, data=f_slice)
    distances, mask = convert_differences_to_distances(
        differences, mic_idx, distance_range, azimuth_deg=azimuth_deg
    )
    if mask is not None:
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
        # print("Deprecation warning: do not use this function anymore")
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
):
    from constants import SPEED_OF_SOUND

    period_theoretical = frequency / SPEED_OF_SOUND  # 1/m in terms of delta
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

    d_slice_norm = d_slice - np.mean(d_slice)
    d_slice_norm /= np.std(d_slice_norm)

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
            d_slice_theory -= np.nanmean(d_slice_theory)
            std = np.nanstd(d_slice_theory)
            if std > 0:
                d_slice_theory /= std
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
