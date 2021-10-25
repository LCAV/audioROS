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


def extract_pdf(distribution, method=METHOD, verbose=False):
    """
    Extract pdf from distribution. 
    Distribution is of form:
    {
        x1: [p_1(x1), p_2(x1), ...]
        x2: [p_1(x2), p_2(x2), ...]
    }

    For example
    distance 1: [p_mic1(distance 1), p_mic2(distance 2), ...]

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
    DISTANCES_M = np.arange(7, 101) * 1e-2
    AZIMUTHS_DEG = np.arange(-180, 180, step=10)

    def __init__(self):
        self.data = {}  # structure: mic: (path_differences, probabilities)
        self.context = Context.get_platform_setup()

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print("Warning: make sure path_differences_m is in meters!")

        current_data = self.data.get(mic_idx, ([], []))
        self.data[mic_idx] = (
            np.r_[current_data[0], path_differences_m],
            np.r_[current_data[1], probabilities],
        )

    def get_distance_distribution(
        self,
        chosen_mics=None,
        verbose=False,
        method=METHOD,
        azimuth_deg=None,
        distances_m=None,
    ):
        if azimuth_deg is None:
            azimuths_deg = self.AZIMUTHS_DEG
        else:
            azimuths_deg = [azimuth_deg]

        if distances_m is None:
            distances_m = self.DISTANCES_M

        distribution = {d: [] for d in distances_m}

        if verbose:
            import matplotlib.pylab as plt

            fig, axs = plt.subplots(1, 2)
            fig.set_size_inches(10, 3)

        # go over all data saved.
        for mic_idx, (deltas_m, delta_probs) in self.data.items():

            deltas_m, inverse = np.unique(deltas_m, return_inverse=True)
            if verbose:
                axs[0].scatter(deltas_m, delta_probs, label=mic_idx)
            if len(np.unique(inverse)) != len(inverse):
                print("found non-unique deltas!")
            delta_probs = np.bincount(inverse, delta_probs)
            if verbose:
                axs[0].plot(deltas_m, delta_probs, label=mic_idx)
                axs[0].set_xlabel("delta [m]")

            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            # "integral" over possible angles
            for azimuth_deg in azimuths_deg:
                ds_m = (
                    self.context.get_distance(deltas_m * 1e2, azimuth_deg, mic_idx)
                    * 1e-2
                )

                # interpolate (ds_m, delta_probs) at fixed distances distances_m
                if verbose:
                    print(
                        f"interpolating {min(ds_m):.2f}:{np.mean(ds_m[1:] - ds_m[:-1]):.2f}:{max(ds_m):.2f}"
                    )
                    print(
                        f"on            {min(distances_m):.2f}:{np.mean(distances_m[1:] - distances_m[:-1]):.2f}:{max(distances_m):.2f}"
                    )

                interp1d_func = interp1d(
                    ds_m, delta_probs, kind="linear", fill_value="extrapolate"
                )
                delta_probs_interp = interp1d_func(distances_m)

                # gradient due to change of variables.
                # TODO(FD) removed to be consistent with angle inference,
                # since it had negligible effect anyways.
                # correction_factor = self.context.get_delta_gradient(
                #    azimuth_deg, distances_m * 1e2, mic_idx
                # )
                # print('correction factors:', correction_factor)
                # delta_probs_interp *= correction_factor

                # make sure probabiliy is positive.
                delta_probs_interp[delta_probs_interp <= 0] = EPS
                [
                    distribution[d].append(prob)
                    for d, prob in zip(distances_m, delta_probs_interp)
                ]
                if verbose:
                    axs[1].plot(ds_m, delta_probs)
                    axs[1].scatter(distances_m, delta_probs_interp)
                    axs[1].set_xlabel("distance [m]")
                    axs[1].legend()
        return extract_pdf(distribution, method, verbose)

    def get_angle_distribution(
        self, distance_estimate_m, chosen_mics=None, method=METHOD, azimuths_deg=None
    ):
        if distance_estimate_m is None:
            distances_m = self.DISTANCES_M
        else:
            distances_m = [distance_estimate_m]

        assert (
            np.linalg.norm(self.context.source) == 0
        ), "function only works for source at origin!"

        if azimuths_deg is None:
            azimuths_deg = self.AZIMUTHS_DEG
            # azimuths_deg = np.arange(-180, 180)

        distribution = {a: [] for a in azimuths_deg}

        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            r0 = self.context.get_direct_path(mic_idx)

            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            for distance_estimate_m in distances_m:

                # for a given distance, only certain deltas are possible.
                mask = (deltas_m <= 2 * distance_estimate_m) & (
                    deltas_m >= 2 * distance_estimate_m - 2 * r0
                )

                thetas_deg = []
                probs = []

                # each delta and probability corresponds to one or two angles.
                for delta_m, prob in zip(deltas_m[mask], delta_probs[mask]):
                    theta_deg = self.context.get_angles(
                        delta_m=delta_m,
                        distance_m=distance_estimate_m,
                        mic_idx=mic_idx,
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

                # TODO(FD) understand why below works better without correction.
                # correction_factor = self.context.get_delta_gradient_angle(
                #    distance_estimate_m, azimuths_deg, mic_idx
                # )
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
    ANGLES_DEG = np.arange(1, 91)

    def __init__(self):
        self.data = {}  # structure: mic: (gammas, probabilities, frequency)
        self.context = Context.get_platform_setup()

    def add_distribution(self, gammas, probabilities, mic_idx, frequency):
        self.data[mic_idx] = (gammas, probabilities, frequency)

    def get_angle_distribution(
        self, chosen_mics=None, method=METHOD, mics_left_right=None
    ):
        distribution = {g: [] for g in self.ANGLES_DEG}
        for mic_idx, (gammas_deg_here, probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            interp1d_func = interp1d(
                gammas_deg_here, probs, kind="linear", fill_value="extrapolate"
            )
            probs_interp = interp1d_func(self.ANGLES_DEG)
            [
                distribution[g].append(prob)
                for g, prob in zip(self.ANGLES_DEG, probs_interp)
            ]

        gammas, probs = extract_pdf(distribution, method)
        argmax = np.argmax(probs)
        gamma = gammas[argmax]

        if mics_left_right is not None:
            score_left = 0
            score_right = 0
            for mic_left in mics_left_right[0]:
                arg = np.argmin(abs(gamma - self.data[mic_left][0]))
                score_left += self.data[mic_left][1][arg]
            for mic_right in mics_left_right[1]:
                arg = np.argmin(abs(gamma - self.data[mic_right][0]))
                score_right += self.data[mic_right][1][arg]

            if score_right > score_left:
                # print("wall is on the right")
                gammas = 180 - gammas
            else:
                # print("wall is on the left")
                pass
        # TODO(FD): maybe recalculate the distribution using
        # only the correct mics?
        return gammas, probs
