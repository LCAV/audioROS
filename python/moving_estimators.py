#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
moving_estimators.py: 
"""

import numpy as np
import scipy.interpolate


def from_0_to_360(angle):
    angle %= 360
    angle = angle + 360 if angle < 0 else angle
    return angle


class MovingEstimator(object):
    DISTANCES_CM = np.arange(100)
    ANGLES_DEG = np.arange(360, step=15)

    def __init__(self, n_window=2):
        self.dist_p = [[]] * n_window
        self.angle_p = [[]] * n_window
        self.positions = [[]] * n_window
        self.rotations = [[]] * n_window

        self.reference = None

        self.index = -1
        self.n_window = n_window
        self.filled = False

    def add_distributions(
        self,
        angle_deg=None,
        angle_p=None,
        dist_cm=None,
        dist_p=None,
        position_cm=[0, 0],
        rot_deg=0,
    ):
        self.index += 1
        if self.index == self.n_window - 1:
            self.filled = True
        elif self.index == self.n_window:
            self.index = 0
        self.positions[self.index] = np.array(position_cm)
        self.rotations[self.index] = rot_deg

        if (angle_deg is not None) and (angle_p is not None):
            self.angle_p[self.index] = scipy.interpolate.interp1d(
                angle_deg, angle_p, fill_value=0, bounds_error=False
            )
        else:
            # uniform distribution
            self.angle_p[self.index] = lambda x: 1

        if (dist_cm is not None) and (dist_p is not None):
            self.dist_p[self.index] = scipy.interpolate.interp1d(
                dist_cm, dist_p, fill_value=0, bounds_error=False
            )
        else:
            # uniform distribution
            self.dist_p[self.index] = lambda x: 1

    def enough_measurements(self):
        return self.filled

    def _get_distribution(self, distances_cm=None, angles_deg=None, verbose=False):

        if not self.filled:
            print("Warning: not enough measurements yet.")

        def translate_dist(d, angle):
            return (
                d
                + np.cos(
                    (delta_pos_angle - self.rotations[current] - angle) / 180 * np.pi
                )
                * delta_pos_d
            )

        def translate_angle(angle):
            return from_0_to_360(angle + delta_rot)

        current = self.index

        probs_dist = probs_angle = None
        if distances_cm is not None:
            probs_dist = [
                self.dist_p[current](d) for d in distances_cm
            ]  # np.ones(len(distances_cm))
        if angles_deg is not None:
            probs_angle = [
                self.angle_p[current](a) for a in angles_deg
            ]  # np.ones(len(angles_deg))

        if not self.filled:
            # if we are at 2, use [0, 1] (n_window=5)
            others = list(range(self.index))
        else:
            # if we are at 2, use [0, 1, 3, 4] (n_window=5)
            others = list(range(self.n_window))
            others.remove(current)
        if verbose:
            print(f"at {current}, combining with {others}")

        for previous in others:
            delta_rot = self.rotations[current] - self.rotations[previous]
            delta_pos = self.positions[current] - self.positions[previous]
            delta_pos_angle = np.arctan2(delta_pos[1], delta_pos[0]) * 180 / np.pi
            delta_pos_d = np.linalg.norm(delta_pos)

            # for each distance, integrate over angle grid
            if probs_dist is not None:
                for i, d in enumerate(distances_cm):
                    # calculate p(distance) by integrating over angles
                    p = 0
                    for angle in self.ANGLES_DEG:
                        angle_p0 = self.angle_p[previous](translate_angle(angle))
                        dist_p0 = self.dist_p[previous](translate_dist(d, angle))
                        p += angle_p0 * dist_p0

                    # multiply with previous distances
                    probs_dist[i] *= p

            # for each angle, integrate over distance grid
            if probs_angle is not None:
                for i, angle in enumerate(angles_deg):
                    p = 0
                    for d in self.DISTANCES_CM:
                        angle_p0 = self.angle_p[previous](translate_angle(angle))
                        dist_p0 = self.dist_p[previous](translate_dist(d, angle))
                        p += angle_p0 * dist_p0

                    probs_angle[i] *= p

        if probs_dist is not None:
            if np.sum(probs_dist) > 0:
                probs_dist /= np.sum(probs_dist)
            else:
                print("Warning: distance proba all-zero")

        if probs_angle is not None:
            if np.sum(probs_angle) > 0:
                probs_angle /= np.sum(probs_angle)
            else:
                print("Warning: angle proba all-zero")
        return distances_cm, probs_dist, angles_deg, probs_angle

    def get_distance_distribution(self, distances_cm=None, verbose=False):
        if distances_cm is None:
            distances_cm = self.DISTANCES_CM
        return self._get_distribution(distances_cm=distances_cm, verbose=verbose)[:2]

    def get_angle_distribution(self, angles_deg=None, verbose=False):
        if angles_deg is None:
            angles_deg = self.ANGLES_DEG
        return self._get_distribution(angles_deg=angles_deg, verbose=verbose)[2:]
