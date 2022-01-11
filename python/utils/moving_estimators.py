#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
moving_estimators.py: Integrate moving into the distance- and angle estimation. 
"""

import numpy as np
import scipy.interpolate

from geometry import Context


def from_0_to_360(angle):
    angle %= 360
    angle = angle + 360 if angle < 0 else angle
    return angle


class MovingEstimator(object):
    DISTANCES_CM = np.arange(40, step=10)
    ANGLES_DEG = np.arange(360, step=90)

    def __init__(self, n_window=2, platform="crazyflie"):
        self.difference_p = {n: {} for n in range(n_window)}
        self.positions = [[]] * n_window
        self.rotations = [[]] * n_window

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

    def get_distributions(self, verbose=False):
        # get angle distribution (according to when distributions align best)
        distributions = self.get_joint_distribution(verbose=verbose)
        softmax = np.exp(np.max(distributions, axis=1))
        softmax /= np.sum(softmax)
        probs_angles = softmax

        angle_idx = np.argwhere(probs_angles == np.amax(probs_angles))[0]
        if len(angle_idx) > 1:
            print(
                "Warning: ambiguous result for angle distribution! Taking average for distance."
            )
        probs_dist = np.mean(distributions[angle_idx, :], axis=0)
        return probs_dist, probs_angles

    def get_distance_estimate(self, dist):
        return self.DISTANCES_CM[np.argmax(dist)]

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
            print("current:", current, "others:", others)

        distributions = np.ones((len(self.ANGLES_DEG), len(self.DISTANCES_CM)))
        for a, angle in enumerate(self.ANGLES_DEG):
            if verbose:
                print(f"\n==== treating angle {angle} ====")
            # wall azimuth angle in global coordinates
            angle_absolute = self.rotations[current] + angle
            normal_absolute = -np.array(
                [
                    np.cos(angle_absolute / 180 * np.pi),
                    np.sin(angle_absolute / 180 * np.pi),
                ]
            )

            # convert the differences at the current index to
            # distance distributions.
            for mic, difference_p in self.difference_p[current].items():
                distance_p[current][mic] = np.array(
                    [
                        difference_p(self.context.get_delta(angle, d, mic))
                        for d in self.DISTANCES_CM
                    ]
                )
                if verbose:
                    print(
                        "  distance estimate for latest position:",
                        self.get_distance_estimate(distance_p[current][mic]),
                    )

                # for previous positions, a bit more work is needed:
                for previous in others:
                    delta_rot = self.rotations[current] - self.rotations[previous]
                    delta_pos = self.positions[current] - self.positions[previous]
                    delta_pos_angle = (
                        np.arctan2(delta_pos[1], delta_pos[0]) * 180 / np.pi
                    )
                    delta_pos_d = np.linalg.norm(delta_pos)

                    # wall azimuth angle as seen from this position
                    angle_trans = angle + delta_rot

                    # distances converted to the current position
                    distances_trans = self.DISTANCES_CM - normal_absolute.dot(delta_pos)
                    if verbose:
                        print("  translated angle:", angle_trans)
                        print(
                            "  translated distances:",
                            np.min(distances_trans),
                            np.max(distances_trans),
                        )

                    for mic, difference_p2 in self.difference_p[previous].items():
                        deltas = [
                            self.context.get_delta(angle_trans, d, mic)
                            for d in distances_trans
                        ]
                        distance_p[previous][mic] = np.array(
                            [difference_p2(d) for d in deltas]
                        )
                        if verbose:
                            print(
                                "  distance estimate for previous position:",
                                self.get_distance_estimate(distance_p[previous][mic]),
                            )
            # multiply over mics and positions to get one final distance distribution
            # for this angle.
            for i, probs_mics in distance_p.items():
                for mic, probs in probs_mics.items():
                    # distributions[a] = np.multiply(distributions[a], probs)
                    if verbose:
                        print(
                            f"mic{mic}, position{i}: adding to row {a} (angle {angle}):",
                            probs,
                        )
                    distributions[a] = distributions[a] + probs

        return distributions
