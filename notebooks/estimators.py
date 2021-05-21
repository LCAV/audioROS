#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distance_estimator.py: 
"""

import warnings

import numpy as np
from scipy.interpolate import interp1d

from geometry import Context

# method to combine probability distributions
METHOD = "product"  # TODO(FD) not working yet
# METHOD = "sum"
EPS = 1e-30  # smallest value for probability distribution


def get_estimate(values, probs):
    return values[np.argmax(probs)]


def extract_pdf(distribution, method=METHOD):
    """
    Extract pdf from distribution. 
    Distribution is of form:
    {
        x1: [p_1(x1), p_2(x1), ...]
        x2: [p_1(x2), p_2(x2), ...]
    }

    """
    values = np.fromiter(distribution.keys(), dtype=float)
    sorted_idx = np.argsort(values)
    values = values[sorted_idx]

    # get equally distributed values in the same range.
    values_interp = np.linspace(values[0], values[-1], len(values))
    probabilities = np.empty(len(values))

    max_count = np.max([len(p_list) for p_list in distribution.values()])
    for i, p_list in enumerate(distribution.values()):
        if method == "product":
            probabilities[i] = np.product(p_list)
            # equivalent but potentially more stable numerically:
            # probabilities[i] = 10**np.sum(np.log10(p_list))
        elif method == "sum":
            probabilities[i] = np.mean(p_list)
        else:
            raise ValueError(method)
    probabilities /= np.sum(probabilities)
    return values, probabilities[sorted_idx]


class DistanceEstimator(object):
    def __init__(self):
        self.data = {}  # structure: mic: (path_differences, probabilities)
        self.context = Context.get_crazyflie_setup()

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print("Warning: make sure path_differences_m is in meters!")
        self.data[mic_idx] = (path_differences_m, probabilities)

    def get_distance_distribution(
        self,
        chosen_mics=None,
        verbose=False,
        method=METHOD,
        azimuth_deg=None,
        distances_m=None,
    ):
        if azimuth_deg is None:
            azimuths_deg = np.arange(-180, 180, step=10)
        else:
            azimuths_deg = [azimuth_deg]

        if distances_m is None:
            # get points for interpolation.
            distances_all = []
            for mic_idx, (deltas_m, delta_probs) in self.data.items():
                for azimuth_deg in azimuths_deg:
                    ds_m = (
                        self.context.get_distance(deltas_m * 1e2, azimuth_deg, mic_idx)
                        * 1e-2
                    )
                    distances_all += list(ds_m)
            distances_m = np.linspace(
                min(distances_all), max(distances_all), len(distances_all)
            )
        distribution = {d: [] for d in distances_m}

        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            for azimuth_deg in azimuths_deg:
                ds_m = (
                    self.context.get_distance(deltas_m * 1e2, azimuth_deg, mic_idx)
                    * 1e-2
                )

                # interpolate (ds_m, delta_probs) at fixed distances.
                interp1d_func = interp1d(
                    ds_m, delta_probs, kind="linear", fill_value="extrapolate"
                )
                delta_probs_interp = interp1d_func(distances_m)

                # gradient due to change of variables.
                correction_factor = self.context.get_delta_gradient(
                    azimuth_deg, distances_m * 1e2, mic_idx
                )
                delta_probs_interp *= correction_factor

                # make sure probabiliy is positive.
                delta_probs_interp[delta_probs_interp < 0] = EPS
                [
                    distribution[d].append(prob)
                    for d, prob in zip(distances_m, delta_probs_interp)
                ]
        return extract_pdf(distribution, method)

    def get_angle_distribution(
        self, distance_estimate_m, chosen_mics=None, method=METHOD, azimuths_deg=None
    ):
        assert (
            np.linalg.norm(self.context.source) == 0
        ), "function only works for source at origin!"
        # azimuths_deg = np.arange(-180, 180, step=self.resolution_deg)

        if azimuths_deg is None:
            azimuths_deg = np.arange(-180, 180)

        distribution = {a: [] for a in azimuths_deg}

        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            r0 = self.context.get_direct_path(mic_idx)

            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            # for a given distance, only certain deltas are possible.
            mask = (deltas_m <= 2 * distance_estimate_m) & (
                deltas_m >= 2 * distance_estimate_m - 2 * r0
            )

            thetas_deg = []
            probs = []
            # each delta and probability corresponds to one or two angles.
            for delta_m, prob in zip(deltas_m[mask], delta_probs[mask]):
                theta_deg = self.context.get_angles(
                    delta_m=delta_m, distance_m=distance_estimate_m, mic_idx=mic_idx,
                )
                if theta_deg is None:
                    warnings.warn("no theta")
                    continue
                thetas_deg += list(theta_deg)
                probs += [prob] * len(theta_deg)

            if not len(thetas_deg):
                warnings.warn("no valid thetas found")
                continue
            # interpolate (ds_m, delta_probs) at current distances
            interp1d_func = interp1d(
                thetas_deg, probs, kind="linear", fill_value="extrapolate"
            )
            probs_interp = interp1d_func(azimuths_deg)

            # TODO(FD) add the gradient here
            # correction_factor = self.context.get_delta_gradient_angle(distance_estimate_m, azimuths_deg, mic_idx)
            # probs_interp *= correction_factor

            # make sure probabiliy is positive.
            probs_interp[probs_interp < 0] = EPS
            [
                distribution[a].append(prob)
                for a, prob in zip(azimuths_deg, probs_interp)
            ]
        return extract_pdf(distribution, method)

    def get_distance_estimate(self, chosen_mics=None):
        ds, probs = self.get_distance_distribution(chosen_mics)
        return get_estimate(ds, probs)

    def get_angle_estimate(self, chosen_mics=None):
        ts, probs = self.get_angle_distribution(chosen_mics)
        return get_estimate(ts, probs)


class AngleEstimator(object):
    def __init__(self):
        self.data = {}  # structure: mic: (gammas, probabilities, frequency)
        self.context = Context.get_crazyflie_setup()

    def add_distribution(self, gammas, probabilities, mic_idx, frequency):
        self.data[mic_idx] = (gammas, probabilities, frequency)

    def get_angle_distribution(self, chosen_mics=None, method=METHOD):
        gammas_deg = np.arange(1, 90)
        distribution = {g: [] for g in gammas_deg}
        for mic_idx, (gammas_deg_here, probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            interp1d_func = interp1d(
                gammas_deg_here, probs, kind="linear", fill_value="extrapolate"
            )
            probs_interp = interp1d_func(gammas_deg)
            [distribution[g].append(prob) for g, prob in zip(gammas_deg, probs_interp)]
        return extract_pdf(distribution, method)
