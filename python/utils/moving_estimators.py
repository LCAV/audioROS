#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
moving_estimators.py: Integrate moving into the distance- and angle estimation. 
"""

import numpy as np
import scipy.interpolate

from .geometry import Context

# Factor to use in probability distribution multiplication.
# 1 tends to lead to over-confident estimates, 0.5 was found to give good results.
ALPHA = 0.9


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

    def __init__(self, n_window=2, platform="crazyflie"):
        self.difference_p = {n: {} for n in range(n_window)}
        self.positions = [[]] * n_window
        self.rotations = [[]] * n_window

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

        # detect if we are still at the same position.
        self.index += 1
        if self.index == self.n_window - 1:
            self.filled = True
        elif self.index == self.n_window:  # rolling window
            self.index = 0

        self.positions[self.index] = np.array(position_cm)
        self.rotations[self.index] = rot_deg

        for mic_idx, (diff, prob) in diff_dictionary.items():
            if not any(diff > 1):
                print("Warning: need to pass diff in centimeters")
            self.difference_p[self.index][mic_idx] = scipy.interpolate.interp1d(
                diff.astype(np.float),
                prob.astype(np.float),
                fill_value=0,
                bounds_error=False,
                kind="linear",
            )

    def enough_measurements(self):
        return self.filled

    def get_distributions(self, local=False, verbose=False):
        # get angle distribution (according to when distributions align best)
        distributions = self.get_joint_distribution(verbose=verbose)

        # softmax
        # probs_angles = np.exp(np.max(distributions, axis=1))
        probs_angles = np.sum(distributions, axis=1)
        probs_angles /= np.sum(probs_angles)

        angle_idx = np.argwhere(probs_angles == np.amax(probs_angles))[0]
        if len(angle_idx) > 1:
            print(
                "Warning: ambiguous result for angle distribution! Taking average for distance."
            )
        probs_dist = np.mean(distributions[angle_idx, :], axis=0)
        probs_dist /= np.sum(probs_dist)

        # convert distribution to local coordinates.
        if local:
            position = self.positions[self.index]
            angle = self.ANGLES_DEG[angle_idx]
            # print("angle", angle)
            normal_absolute = get_normal(angle)
            starting_distance = -normal_absolute.dot(position)
            idx = np.where(self.DISTANCES_CM > starting_distance)[0]
            if len(idx) < 1:
                raise ValueError("moved out of range")
            # shift probability at starting_distance to 0, etc.
            probs_dist = np.roll(probs_dist, shift=-idx[0])
            probs_dist[-idx[0] :] = 1e-3
        return probs_dist, probs_angles

    def get_distance_estimate(self, dist):
        max_dist = np.max(dist)
        estimates = self.DISTANCES_CM[np.where(dist == max_dist)[0]]
        if len(estimates) > 1:
            print(f"Warning: ambiguous distribution, {len(estimates)} maxima")
        return estimates[0]

    def get_angle_estimate(self, dist):
        max_dist = np.max(dist)
        estimates = self.ANGLES_DEG[np.where(dist == max_dist)[0]]
        if len(estimates) > 1:
            print(f"Warning: ambiguous distribution, {len(estimates)} maxima")
        return estimates[0]

    def get_joint_distribution(self, verbose=False):
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
                f"current: {current}, ({self.positions[current]}, {self.rotations[current]})"
            )

        distributions = np.ones((len(self.ANGLES_DEG), len(self.DISTANCES_CM)))

        for a, angle in enumerate(self.ANGLES_DEG):  # in global coordinates
            # wall azimuth angle in local coordinates
            angle_local = angle - self.rotations[current]

            if verbose:
                print(f"\n==== treating global angle {angle}, local: {angle_local}====")

            normal_absolute = get_normal(angle)

            distances_local = self.DISTANCES_CM + normal_absolute.dot(
                self.positions[current]
            )
            # convert the differences at the current index to
            # global distance distributions.
            for mic, difference_p in self.difference_p[current].items():
                distance_p[current][mic] = np.array(
                    [
                        difference_p(self.context.get_delta(angle_local, d, mic))
                        for d in distances_local
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
                        f"previous: {previous}, ({self.positions[previous]}, {self.rotations[previous]})"
                    )

                # wall azimuth angle in local coordinates
                angle_local_prev = angle - self.rotations[previous]

                # local distances current position
                distances_local_prev = self.DISTANCES_CM + normal_absolute.dot(
                    self.positions[previous]
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
            # for this angle.
            for i, probs_mics in distance_p.items():
                for mic, probs in probs_mics.items():

                    distributions[a] = (
                        np.multiply(distributions[a] ** ALPHA, probs ** ALPHA) ** 1
                        / ALPHA
                    )
        return distributions
