#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distance_estimator.py: 
"""

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

        self.resolution_m = 1e-2  # resolution of distances, in meters.
        self.resolution_deg = 2  # resolution of angles, in degrees

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print("Warning: make sure path_differences_m is in meters!")
        self.data[mic_idx] = (path_differences_m, probabilities)

    def get_distance_distribution(
        self, chosen_mics=None, verbose=False, method=METHOD, azimuth_deg=None
    ):
        if azimuth_deg is None:
            azimuths_deg = np.arange(-180, 180, step=2)
        else:
            azimuths_deg = [azimuth_deg]

        # get points for interpolation.
        distances_all = []
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            for azimuth_deg in azimuths_deg:
                ds_m = self.context.get_total_distance(deltas_m, azimuth_deg, mic_idx)
                distances_all += list(ds_m)
        distances_interp = np.linspace(
            min(distances_all), max(distances_all), len(distances_all)
        )
        distribution = {d: [] for d in distances_interp}

        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            for azimuth_deg in azimuths_deg:
                ds_m = self.context.get_total_distance(deltas_m, azimuth_deg, mic_idx)

                # interpolate (ds_m, delta_probs) at current distances
                interp1d_func = interp1d(
                    ds_m, delta_probs, kind="linear", fill_value="extrapolate"
                )
                delta_probs_interp = interp1d_func(distances_interp)
                # make sure probabiliy is positive.
                delta_probs_interp[delta_probs_interp < 0] = EPS

                new_distribution = dict(zip(distances_interp, delta_probs_interp))
                [
                    distribution[d].append(prob)
                    for d, prob in zip(distances_interp, delta_probs_interp)
                ]
        return extract_pdf(distribution, method)

    def get_angle_distribution(
        self, distance_estimate_m, chosen_mics=None, method=METHOD
    ):
        assert (
            np.linalg.norm(self.context.source) == 0
        ), "function only works for source at origin!"
        # azimuths_deg = np.arange(-180, 180, step=self.resolution_deg)
        distribution = {}  # {azimuth_deg: 0 for azimuth_deg in azimuths_deg}
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue
            # deltas_m = [deltas_m[np.argmax(delta_probs)]]
            # delta_probs = [1.0]
            for delta_m, prob in zip(deltas_m, delta_probs):
                thetas_deg = self.context.get_angles(
                    delta_m=delta_m,
                    source_distance_m=distance_estimate_m,
                    mic_idx=mic_idx,
                )
                if thetas_deg is None:
                    continue

                for t in thetas_deg:
                    t_rounded = round(t / self.resolution_deg) * self.resolution_deg
                    if t_rounded not in distribution.keys():
                        distribution[t_rounded] = []

                    distribution[t_rounded].append(prob)
        return extract_pdf(distribution, method)

    def get_distance_estimate(self, chosen_mics=None):
        ds, probs = self.get_distance_distribution(chosen_mics)
        return get_estimate(ds, probs)

    def get_angle_estimate(self, chosen_mics=None):
        ts, probs = self.get_angle_distribution(chosen_mics)
        return get_estimate(ts, probs)


# TODO(FD) below is not tested yet
class AngleEstimator(object):
    def __init__(self):
        self.data = {}  # structure: mic: (path_differences, probabilities)
        self.context = Context.get_crazyflie_setup(yaw_offset=0)

        # resolution in sin(gamma)
        self.resolution = 1e-2

    def add_distribution(self, periods_m, probabilities, mic_idx, frequency):
        self.data[mic_idx] = (periods_m, probabilities, frequency)

    def get_angle_distribution(self, chosen_mics=None):
        from constants import SPEED_OF_SOUND

        azimuths_deg = np.arange(-180, 180)
        distribution = {}
        for mic_idx, (periods_m, period_probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            period_theoretical = SPEED_OF_SOUND / frequency  # m in terms of delta

            for azimuth_deg in azimuths_deg:
                periods_m_ortho = context.get_delta(
                    azimuth_deg=azimuth_deg,
                    distance=starting_distance + periods_m,
                    mic_idx=mic_idx,
                )
                sines_gamma = periods_m_ortho / period_theoretical
                for s, prob in zip(sines_gamma, period_probs):
                    s_rounded = round(s / self.resolution) * self.resolution
                    distribution[s_rounded] = distribution.get(s_rounded, 0) + prob

        # gammas = np.full(len(sines_gamma), 90)
        # gammas[sines_gamma <= 1] = np.arcsin(sines_gamma[sines_gamma <= 1]) * 180 / np.pi
        return distribution
