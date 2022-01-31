#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
moving_estimators.py: Integrate moving into the distance- and angle estimation. 
"""
import warnings

import numpy as np
import scipy.interpolate
import scipy.signal

from .geometry import Context

# Factor to use in probability distribution multiplication.
# 1 tends to lead to over-confident estimates, 0.5 was found to give good results.
ALPHA = 0.8
ANGLE_WINDOW_DEG = 20  # set to zero to use only the forward direction. o
ANGLE_RESOLUTION_DEG = 20

ESTIMATION_METHOD = "peak"
# ESTIMATION_METHOD = "mean"
# ESTIMATION_METHOD = "max"


def get_std_sample(values, probs, means, unbiased=True):
    if np.isclose(np.nansum(probs), 1):  # for distributions
        norm = 1.0
    else:  # for histograms
        norm = (np.nansum(probs) - 1) if unbiased else np.nansum(probs)
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


def get_estimate(values, probs, method=ESTIMATION_METHOD, unbiased=True):
    if method == "mean":
        mean = np.sum(np.multiply(probs, values)) / np.sum(probs)
        std = get_std_sample(values, probs, means=mean, unbiased=unbiased)
        return mean, std
    elif method == "max":
        max_prob = np.max(probs)
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
    if len(estimates) > 1:
        print(f"Warning: ambiguous distribution, {len(estimates)} maxima")
    elif not len(estimates):
        print(f"Warning: no valid estimates detected: {values}, {probs}")
        return None, None
    return estimates[0], stds[0]


def get_estimates(values, probs, sort=True, n_estimates=None, method=ESTIMATION_METHOD):
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
            warnings.warn("Found less peaks than requested")
            return None, None
        stds = get_std_of_peaks(indices)

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


def from_0_to_360(angle):
    angle %= 360
    angle = angle + 360 if angle < 0 else angle
    return angle


def get_normal(angle_deg):
    return -np.r_[
        np.cos(angle_deg / 180 * np.pi), np.sin(angle_deg / 180 * np.pi),
    ]


class MovingEstimator(object):
    DISTANCES_CM = np.arange(100, step=2)
    ANGLES_DEG = np.arange(360, step=10)

    def __init__(
        self,
        n_window=2,
        platform="crazyflie",
        distances_cm=DISTANCES_CM,
        angles_deg=ANGLES_DEG,
    ):
        self.difference_p = {n: {} for n in range(n_window)}
        self.positions = np.full((n_window, 2), None)
        self.rotations = np.full(n_window, None)

        self.distances_cm = distances_cm
        self.angles_deg = angles_deg

        self.dim = 2
        self.reference = None

        self.index = -1
        self.n_window = n_window
        self.filled = False

        self.context = Context.get_platform_setup(platform)

    def add_distributions(
        self, diff_dictionary, position_cm=[0, 0], rot_deg=0,
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

    def enough_measurements(self):
        return self.filled

    def get_distributions(self, verbose=False, simplify_angles=True):
        def clean_distribution(dist):
            if not np.any(~np.isnan(dist)):
                dist = np.ones_like(dist)
            sum_ = np.sum(dist)
            if sum_:
                dist /= sum_
            return dist

        # get angle distribution (according to when distributions align best)
        distributions, angles_deg, distances_cm = self.get_joint_distribution(
            verbose=verbose, simplify_angles=simplify_angles
        )

        # softmax
        # probs_angles = np.exp(np.max(distributions, axis=1))
        probs_angles = np.sum(distributions, axis=1)
        probs_angles = clean_distribution(probs_angles)

        angle_idx = np.argwhere(probs_angles == np.nanmax(probs_angles))[0]
        if len(angle_idx) > 1:
            print(
                "Warning: ambiguous result for angle distribution! Taking average for distance."
            )
        probs_dist = np.mean(distributions[angle_idx, :], axis=0)
        probs_dist = clean_distribution(probs_dist)
        return distances_cm, probs_dist, angles_deg, probs_angles

    def get_distance_estimate(self, dist, method=ESTIMATION_METHOD):
        return get_estimate(self.distances_cm, dist, method=method)

    def get_angle_estimate(self, dist, method=ESTIMATION_METHOD):
        return get_estimate(self.angles_deg, dist, method=method)

    def get_joint_distribution(self, verbose=False, simplify_angles=True):
        if not self.filled:
            print("Warning: not enough measurements yet.")

        current = self.index
        distance_p = {n: {} for n in range(self.n_window)}

        if not self.filled:
            # if we are at 2, use [0, 1] (n_window=5)
            others = list(range(self.index))
        else:
            # if we are at 2, use [0, 1, 3, 4] (n_window=5)
            others = list(range(self.n_window))
            others.remove(current)

        if verbose:
            print(
                f"current: {current}: ({self.positions[current]}, {self.rotations[current]})"
            )

        # search only aroundo the forward direction instead of all 360 degrees. This makes sense
        # because the only "dangerous" angles are in front of us.
        if simplify_angles:
            angle_local_deg = self.get_local_forward_angle()
            angles_deg = np.arange(
                angle_local_deg - ANGLE_WINDOW_DEG / 2,
                angle_local_deg + ANGLE_WINDOW_DEG / 2 + ANGLE_RESOLUTION_DEG,
                step=ANGLE_RESOLUTION_DEG,
            )
        else:
            angles_deg = self.angles_deg

        joint_distribution = np.ones((len(angles_deg), len(self.distances_cm)))
        for a, angle_local in enumerate(angles_deg):  # in local coordinates
            # wall azimuth angle in global coordinates
            angle = angle_local + self.rotations[current]

            if verbose:
                print(f"\n==== treating global angle {angle}, local: {angle_local}====")

            normal_absolute = get_normal(angle)

            # convert the differences at the current index to
            # local distance distributions.
            for mic, difference_p in self.difference_p[current].items():
                distance_p[current][mic] = np.array(
                    [
                        difference_p(self.context.get_delta(angle_local, d, mic))
                        for d in self.distances_cm
                    ]
                )
                if verbose:
                    print(
                        "  distance estimate for current position:",
                        self.get_distance_estimate(distance_p[current][mic]),
                    )

            # for previous positions, a bit more work is needed:
            for previous in others:
                if verbose:
                    print(
                        f"previous: {previous}: ({self.positions[previous]}, {self.rotations[previous]})"
                    )

                # wall azimuth angle in local coordinates
                angle_local_prev = angle - self.rotations[previous]

                # local distances previous position
                delta_pos = self.positions[current] - self.positions[previous]
                distances_local_prev = self.distances_cm - normal_absolute.dot(
                    delta_pos
                )

                if verbose:
                    print(f"  local angle at position {previous}:", angle_local_prev)
                    print(
                        f"  evaluating distances at {previous}:",
                        np.min(distances_local_prev),
                        np.max(distances_local_prev),
                    )

                for mic, difference_p2 in self.difference_p[previous].items():
                    deltas = [
                        self.context.get_delta(angle_local_prev, d, mic)
                        for d in distances_local_prev
                    ]
                    distance_p[previous][mic] = np.array(
                        [difference_p2(d) for d in deltas]
                    )
                    if verbose:
                        print(
                            f"  distance estimate from position {previous}:",
                            self.get_distance_estimate(distance_p[previous][mic]),
                        )
            # multiply over mics and positions to get one final distance distribution
            # for this angle. ALPHA is used to not get over-confident estimates.
            for i, probs_mics in distance_p.items():
                for mic, probs in probs_mics.items():
                    joint_distribution[a] = (
                        np.multiply(joint_distribution[a] ** ALPHA, probs ** ALPHA) ** 1
                        / ALPHA
                    )
        return joint_distribution, angles_deg, self.distances_cm

    def get_local_forward_angle(self):
        current = self.index
        if not self.filled:
            print("Warning: cannot simplify angles yet because buffer not filled.")
            return 0
        # if self.positions is [5, 6, 2, 3, 4] with current=2, return [2, 3, 4, 5, 6]
        ordered_positions = np.roll(self.positions, -current - 1, axis=0)
        forward_dir_global = np.mean(np.diff(ordered_positions, axis=0), axis=0)
        angle_global_deg = (
            180 / np.pi * np.arctan2(forward_dir_global[1], forward_dir_global[0])
        )
        angle_local_deg = angle_global_deg - self.rotations[current]
        # print(f"local angle: {angle_local_deg:.1f} deg")
        return angle_local_deg
