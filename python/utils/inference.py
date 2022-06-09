#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
inference.py: Get probability distributions and estimates of angle or distance from distance-frequency measurements.
"""

import numpy as np

from .constants import PLATFORM, SPEED_OF_SOUND
from .geometry import get_orthogonal_distance_from_global
from .simulation import get_deltas_from_global
from .simulation import get_dist_slice_theory
from .simulation import get_freq_slice_theory

EPS = 1e-30

# BAD_FREQ_RANGES = []
if PLATFORM == "crazyflie":
    # for 2021_07_08_stepper results:
    BAD_FREQ_RANGES = [[0, 2995], [3630, 3870], [4445, 5000]]
    # for later results:
    # BAD_FREQ_RANGES = [[0, 2995]]
else:
    BAD_FREQ_RANGES = [[0, 2500]]

INTERPOLATE = True
N_MAX = 1000


def get_uniform_grid(xvalues):
    """Fill too large spaces of xvalues with uniform sampling of the median spacing of xvalues.
    Example:
    [0, 2, 4, 10, 12, 14]
    becomes
    [0, 2, 4, 6, 8, 10, 12, 14]
    """
    max_count = 10  # to not loop forever
    for i in range(max_count):
        diffs = np.diff(xvalues)
        median = np.median(diffs)
        diffs = np.r_[median, diffs]
        split = np.where(diffs > median * 1.5)[0]
        if not len(split):
            return xvalues
        s = split[0]
        xvalues = np.r_[
            xvalues[: s - 1],
            np.arange(xvalues[s - 1], xvalues[s], step=median),
            xvalues[s:],
        ]
    raise ValueError(f"Did not finish in {max_count} iterations")


def interpolate_parts(xvalues, values, step=None, verbose=False):
    """Smart interpolation scheme.

    Interpolate at uniform grid, leaving out points that are further
    from recorded data than two times the average or given spacing.

    """
    import scipy.interpolate

    if step is not None:
        print("Warning: giving step is depcreated")

    xvalues_grid = get_uniform_grid(xvalues)
    step = np.median(np.diff(xvalues_grid))

    # valid points are no more than 2*step from actual data.
    valid = np.any(np.abs(xvalues_grid[:, None] - xvalues[None, :]) <= 2 * step, axis=1)
    # if np.sum(~valid):
    #    print("Warning: removing some values before interpolation!")

    if verbose:
        print(f"uniform: {len(xvalues_grid)}, original: {len(xvalues)}")
        print(f"interpolating at {np.sum(valid)} points")

    xvalues_grid = xvalues_grid[valid]
    assert np.abs(np.median(np.diff(xvalues_grid)) - step) < 1e-10, (
        np.median(np.diff(xvalues_grid)),
        step,
    )

    interpolator = scipy.interpolate.interp1d(
        xvalues, values, kind="linear", fill_value="extrapolate"
    )
    values_grid = interpolator(xvalues_grid)
    return xvalues_grid, values_grid


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
        self.distance_range_cm = None  # [min, max]
        self.is_calibrated = False
        self.calibration_function = None

    def add_data(self, slices, values, stds=None, distances=None, mics=range(4)):
        """
        :param slices: interference slices of shape (n_mics, n_values), non-squared.
        :param values: values of shape (n_values, )
        :param stds: standard deviations (n_mics, )
        """
        assert slices.shape[1] == len(values), slices.shape
        self.slices = slices
        self.values = values
        self.stds = stds
        self.distances_cm = distances
        self.valid_idx = np.ones(len(values), dtype=bool)
        self.is_calibrated = False
        self.mics = mics

    def add_geometry(self, distance_range, azimuth_deg):
        self.distance_range_cm = distance_range
        self.azimuth_deg = azimuth_deg

    def add_calibration_function(self, calibration_function):
        if calibration_function in ["", False, None]:
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
        f_calib_mics = f_calib[self.mics, :]
        self.slices[:, self.valid_idx] /= f_calib_mics
        self.is_calibrated = True

    def filter_out_freqs(self, freq_ranges=BAD_FREQ_RANGES, verbose=False):
        """
        :param freq_ranges: list of frequency ranges to filter out
        """
        if verbose:
            print("number of frequencies before:", np.sum(self.valid_idx))
        for freq_range in freq_ranges:
            self.valid_idx &= (freq_range[1] <= self.values) | (
                self.values <= freq_range[0]
            )
        if verbose:
            print("number of frequencies after:", np.sum(self.valid_idx))

    def do_inference(self, algorithm, mic_idx, calibrate=True, normalize=True, ax=None):
        """
        Perform distance inference on current data.
        """
        if calibrate and not self.is_calibrated:
            self.calibrate()

        valid = self.valid_idx & np.all(~np.isnan(self.slices), axis=0)

        if algorithm == "bayes":
            sigma = self.stds[mic_idx] if self.stds is not None else None
            dists_cm, proba, diffs_cm = get_probability_bayes(
                self.slices[mic_idx, valid] ** 2,
                self.values[valid],
                mic_idx=mic_idx,
                distance_range=self.distance_range_cm,
                sigma=sigma,
                azimuth_deg=self.azimuth_deg,
            )
        elif algorithm == "cost":
            distances_cm = (
                self.distances_cm[valid] if self.distances_cm is not None else None
            )

            dists_cm = np.arange(
                self.distance_range_cm[0], self.distance_range_cm[-1] + 1
            )
            diffs_m, __ = get_deltas_from_global(self.azimuth_deg, dists_cm, mic_idx)
            diffs_cm = diffs_m * 1e2
            proba = get_probability_cost(
                self.slices[mic_idx, valid] ** 2,
                self.values[valid],
                dists_cm,
                mic_idx=mic_idx,
                relative_ds=distances_cm,
                absolute_yaws=None,
                azimuth_deg=self.azimuth_deg,
                ax=ax,
            )
        elif algorithm == "cost_2d":
            distances_cm = (
                self.distances_cm[valid] if self.distances_cm is not None else None
            )

            dists_cm = np.arange(
                self.distance_range_cm[0], self.distance_range_cm[-1] + 1
            )
            diffs_m, __ = get_deltas_from_global(self.azimuth_deg, dists_cm, mic_idx)
            diffs_cm = diffs_m * 1e2
            azimuth_degs = np.arange(360, step=10)
            proba_2d = get_probability_cost_2d(
                self.slices[mic_idx, valid],
                self.values[valid],
                dists_cm,
                mic_idx=mic_idx,
                relative_ds=distances_cm,
                absolute_yaws=None,
                azimuth_degs=azimuth_degs,
                ax=ax,
            )

            proba = np.sum(proba_2d, axis=0)
        else:
            raise ValueError(algorithm)

        if "cost" in algorithm:
            # remove high frequency components
            proba_fft = np.fft.rfft(proba)
            freq = np.fft.rfftfreq(len(proba))
            proba_fft[len(proba_fft) // 5 :] = 0  # freq > 0.1 pi
            proba = np.fft.irfft(proba_fft)

        if normalize:
            proba = eps_normalize(proba)
        return dists_cm, proba, diffs_cm

    def plot(self, i_mic, ax, label=None, standardize=False, **kwargs):
        from copy import deepcopy

        slice_mic = deepcopy(self.slices[i_mic, self.valid_idx])
        if standardize:
            slice_mic = standardize_vec(slice_mic)

        ax.plot(
            self.values[self.valid_idx],
            slice_mic,
            label=label,
            **kwargs,
        )


def get_abs_fft(f_slice, n_max=N_MAX):
    f_slice_norm = f_slice - np.nanmean(f_slice)
    n = max(len(f_slice), n_max)
    return np.abs(np.fft.rfft(f_slice_norm, n=n))


def get_differences(frequencies, n_max=N_MAX):

    n = max(len(frequencies), n_max)
    df = np.median(np.diff(frequencies))
    deltas_cm = np.fft.rfftfreq(n, df) * SPEED_OF_SOUND * 100
    return deltas_cm


def convert_differences_to_distances(differences_cm, mic_idx, azimuth_deg):

    distances = get_orthogonal_distance_from_global(
        azimuth_deg=azimuth_deg, deltas_cm=differences_cm, mic_idx=mic_idx
    )
    return distances


def get_posterior(abs_fft, sigma=None, data=None):
    N = len(abs_fft)
    periodogram = 1 / N * abs_fft**2
    # print('periodogram:', np.min(periodogram), np.max(periodogram))

    if sigma is not None:
        if np.any(sigma > 0):
            periodogram /= sigma**2
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
        if d_bar == 0:
            return np.ones_like(periodogram)

        arg = 1 - 2 * periodogram / (N * d_bar)

        # arg may not be negative.
        if np.any(arg <= 0):
            valid = np.where(arg > 0)[0]
            print(
                "Warning, arg is non-positive at", arg[arg <= 0], np.where(arg <= 0)[0]
            )
            arg[arg <= 0] = np.min(arg[arg > 0])

        posterior = (arg) ** ((2 - N) / 2)
        # posterior = np.exp(periodogram)
    return posterior


def get_probability_bayes(
    f_slice,
    frequencies,
    mic_idx=1,
    distance_range=None,
    n_max=N_MAX,
    sigma=None,
    azimuth_deg=None,
    interpolate=INTERPOLATE,
):
    assert f_slice.ndim == 1

    if interpolate:
        frequencies_grid, f_slice_grid = interpolate_parts(
            frequencies,
            f_slice,  # step=20
        )
        abs_fft = get_abs_fft(f_slice_grid, n_max=n_max)
        differences = get_differences(frequencies_grid, n_max=n_max)
        posterior = get_posterior(abs_fft, sigma, data=f_slice_grid)
    else:
        abs_fft = get_abs_fft(f_slice, n_max=n_max)

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


def get_probability_cost_2d(
    f_slice,
    frequencies,
    distances,
    mic_idx=1,
    azimuth_degs=None,
    relative_ds=None,
    absolute_yaws=None,
    ax=None,
):
    costs = np.empty((len(azimuth_degs), len(distances)))
    for i, azimuth_deg in enumerate(azimuth_degs):
        costs[i] = get_probability_cost(
            f_slice,
            frequencies,
            distances,
            mic_idx,
            azimuth_deg,
            relative_ds,
            absolute_yaws,
            ax,
        )
    return costs


def get_probability_cost(
    f_slice,
    frequencies,
    distances,
    mic_idx=1,
    azimuth_deg=None,
    relative_ds=None,
    absolute_yaws=None,
    ax=None,
):
    if np.any(distances < 1):
        raise ValueError("Reminder to change the distance input to include offset!")

    if absolute_yaws is not None:
        azimuth_deg = azimuth_deg - absolute_yaws

    f_slice_norm = f_slice - np.mean(f_slice)
    f_slice_norm /= np.std(f_slice_norm)

    probs = np.empty(len(distances))
    theory = np.empty((len(distances), len(frequencies)))
    for i, d in enumerate(distances):
        if relative_ds is not None:
            d += relative_ds
        f_slice_theory = get_freq_slice_theory(
            frequencies, d, azimuth_deg=azimuth_deg, chosen_mics=[mic_idx]
        ).flatten()
        f_slice_theory -= np.mean(f_slice_theory)
        f_slice_theory /= np.std(f_slice_theory)
        probs[i] = np.exp(-np.linalg.norm(f_slice_theory - f_slice_norm))
        theory[i, :] = f_slice_theory

    if ax is not None:
        ax.plot(frequencies, f_slice_norm, color="black", marker="o")

        indices = np.argsort(probs)[::-1]  # biggest first
        assert probs[indices[0]] >= probs[indices[1]]
        for i in indices[:3]:
            ax.plot(frequencies, theory[i], label=f"p{i}: distance {distances[i]}")
        ax.legend()

    probs /= np.sum(probs)
    return probs


def get_periods_fft(
    d_slice,
    frequency,
    relative_distances_cm,
    n_max=N_MAX,
    bayes=False,
    sigma=None,
):
    # the distribution over measured period.
    d_m = np.mean(np.diff(relative_distances_cm)) * 1e-2
    n = max(len(d_slice), n_max)

    # periods_m = np.fft.rfftfreq(n=n, d=d_m) # equivalent to below
    periods_m = np.arange(0, n // 2 + 1) / (
        d_m * n
    )  # 1/m in terms of orthogonal distance

    abs_fft = get_abs_fft(d_slice, n_max=n_max)
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
    n_max=N_MAX,
    bayes=True,
    sigma=None,
    reduced=False,
    interpolate=INTERPOLATE,
    factor=2,
):
    """
    Get probabilities over approach angles.

    :param d_slice: amplitude measurements along distance
    :param frequency: operating frequency
    :param relative_distances_cm: relative distance measurements
    :param interpolate: interpolate measurements on 1cm-grid before inference.
    """

    # in terms of delta, we have c/f [m], but in terms of orthogonal we have c/2f [m]
    freq_theoretical = (
        factor * frequency / SPEED_OF_SOUND
    )  # 1/m in terms of orthogonal distance

    if interpolate:
        # important to choose small enough step size to insure
        # to capture interference!
        step = 1 / freq_theoretical * 1e2 / 20
        relative_distances_cm_grid, d_slice_grid = interpolate_parts(
            relative_distances_cm, d_slice, step=step
        )
        freqs_m, probs = get_periods_fft(
            d_slice_grid, frequency, relative_distances_cm_grid, n_max, bayes, sigma
        )
    else:
        freqs_m, probs = get_periods_fft(
            d_slice, frequency, relative_distances_cm, n_max, bayes, sigma
        )

    probs = probs[freqs_m > 0]
    freqs_m = freqs_m[freqs_m > 0]

    import scipy.interpolate

    interpolator = scipy.interpolate.interp1d(
        freqs_m, probs, kind="linear", fill_value="extrapolate"
    )

    n_integral = 10  # number of points used to approximate integral
    ns_integral = np.arange(n_integral) / n_integral
    gamma_delta = 1  # number of points
    gammas_deg = np.arange(1, 90 + gamma_delta, step=gamma_delta)
    probs_gamma = np.empty(len(gammas_deg))
    for i_g, gamma in enumerate(gammas_deg):
        sampling_points = (1 - ns_integral) * np.sin(
            np.deg2rad(gamma - gamma_delta / 2)
        ) + ns_integral * np.sin(np.deg2rad(gamma + gamma_delta / 2))
        sampling_points *= freq_theoretical
        probs_gamma[i_g] = np.sum(interpolator(sampling_points))
    return gammas_deg, probs_gamma


def get_approach_angle_cost(
    d_slice,
    frequency,
    relative_distances_cm,
    start_distances_grid_cm,
    gammas_grid_deg,
    mic_idx=1,
    ax=None,
    azimuth_deg=None,
):
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


def get_1d_spectrum(spec):
    """Get one-dimensional angle distribution from freq-angle spectrum.

    :param spec: spectrum of shape n_freqs x n_angles
    :returns: a distribution of length n_angles

    """
    tol = max(np.min(spec[spec > 0]), 1e-2)
    spec[spec < tol] = tol
    vals = np.sum(np.log10(spec), axis=0)
    if ((np.nanmax(vals) - np.nanmin(vals)) > 0):
        vals = (vals - np.nanmin(vals)) / (np.nanmax(vals) - np.nanmin(vals))
    return vals


def get_angle_distribution(signals_f, frequencies, mics):
    from audio_stack.beam_former import BeamFormer

    assert (
        mics.shape[1] == signals_f.shape[1]
    ), "mics should be dxN, signals_df should be FxN"
    assert (
        len(frequencies) == signals_f.shape[0]
    ), "signals_df should be FxN, frequencies of length F"

    beamformer = BeamFormer(mic_positions=mics.T)
    R = beamformer.get_correlation(signals_f)
    spectrum = beamformer.get_lcmv_spectrum(
        R, frequencies_hz=frequencies, extra_constraints=[], cancel_centre=True
    )
    probs = get_1d_spectrum(spectrum)
    return beamformer.theta_scan_deg, probs
