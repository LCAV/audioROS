#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_estimators.py:  Test estimators
"""

import sys, os

import numpy as np
from scipy.stats import norm

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "..", "python")))
sys.path.append(os.path.join(os.getcwd(), "python"))
from moving_estimators import MovingEstimator


def test_moving_delta():
    print("==== testing with delta ====")
    do_test_moving("delta", n_window=2)
    do_test_moving("delta", n_window=3)


def test_moving_normal():
    print("==== testing with normal ====")
    do_test_moving("normal", n_window=2)
    do_test_moving("normal", n_window=3)


def do_test_moving(prob_method, n_window):
    """ Test on simple toy example """

    moving_estimator = MovingEstimator(n_window=n_window)  # 2 or 3 works.
    angle_grid = np.arange(360, step=15)
    dist_grid = np.arange(50)

    def get_measurements(distance, angle, prob_method="delta"):
        if prob_method == "delta":
            angle_p0 = np.zeros(len(angle_grid))
            dist_p0 = np.zeros(len(dist_grid))
            angle_p0[angle_grid == angle] = 1.0
            dist_p0[dist_grid == distance] = 1.0
        elif prob_method == "normal":
            angle_p0 = norm.pdf(angle_grid, loc=angle, scale=10)
            dist_p0 = norm.pdf(dist_grid, loc=distance, scale=2)
        return angle_p0, dist_p0

    # toy example
    angle_p0_gt = 90
    dist_p0_gt = 10
    angle_p0, dist_p0 = get_measurements(dist_p0_gt, angle_p0_gt, prob_method)
    position_0 = [5, 4]
    rot_0 = 0
    angle_p1_gt = 30
    dist_p1_gt = 9
    angle_p1, dist_p1 = get_measurements(dist_p1_gt, angle_p1_gt, prob_method)
    position_1 = [3, 5]
    rot_1 = 60

    print("adding first measurement")
    moving_estimator.add_distributions(
        angle_grid, angle_p0, dist_grid, dist_p0, position_0, rot_0
    )

    # move back to second position
    print("adding second measurement")
    moving_estimator.add_distributions(
        angle_grid, angle_p1, dist_grid, dist_p1, position_1, rot_1
    )
    dist_tot, dist_ptot = moving_estimator.get_distance_distribution(verbose=True)
    assert dist_tot[np.argmax(dist_ptot)] == dist_p1_gt, dist_tot[np.argmax(dist_ptot)]

    angle_tot, angle_ptot = moving_estimator.get_angle_distribution(verbose=True)
    assert angle_tot[np.argmax(angle_ptot)] == angle_p1_gt, angle_tot[
        np.argmax(angle_ptot)
    ]
    print("first position worked ok")

    # move back to first position
    moving_estimator.add_distributions(
        angle_grid, angle_p0, dist_grid, dist_p0, position_0, rot_0
    )
    dist_tot, dist_ptot = moving_estimator.get_distance_distribution(verbose=True)
    assert dist_tot[np.argmax(dist_ptot)] == dist_p0_gt, dist_tot[np.argmax(dist_ptot)]

    angle_tot, angle_ptot = moving_estimator.get_angle_distribution(verbose=True)
    assert angle_tot[np.argmax(angle_ptot)] == angle_p0_gt, angle_tot[
        np.argmax(angle_ptot)
    ]
    print("second position worked ok")

    # move back to second position
    moving_estimator.add_distributions(
        angle_grid, angle_p1, dist_grid, dist_p1, position_1, rot_1
    )
    dist_tot, dist_ptot = moving_estimator.get_distance_distribution(verbose=True)
    assert dist_tot[np.argmax(dist_ptot)] == dist_p1_gt, dist_tot[np.argmax(dist_ptot)]

    angle_tot, angle_ptot = moving_estimator.get_angle_distribution(verbose=True)
    assert angle_tot[np.argmax(angle_ptot)] == angle_p1_gt, angle_tot[
        np.argmax(angle_ptot)
    ]
    print("second position worked ok")


if __name__ == "__main__":
    test_moving_delta()
    test_moving_normal()
    print("done")
