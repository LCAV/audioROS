#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_estimators.py:  Test moving estimator class.
"""

import sys, os

import numpy as np
from scipy.stats import norm

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.moving_estimators import MovingEstimator
from utils.geometry import get_deltas_from_global

MIC_IDX = [0, 1]

DELTA_GRID = np.arange(100)
DISTANCE_GLOB = 30
ANGLE_GLOB = 90


def measure_wall(pose):
    """ 
    simplified function for measuring the wall distance and angle in 
    local coordinates. Works for this geometry only .
    """
    distance = DISTANCE_GLOB - pose[1]
    angle = ANGLE_GLOB - pose[2]
    return distance, angle


def get_delta_distribution(distance, angle, mic_idx, prob_method="delta"):
    delta = (
        get_deltas_from_global(
            azimuth_deg=angle, distances_cm=distance, mic_idx=mic_idx
        )[0]
        * 1e2
    )
    if prob_method == "delta":
        probs = np.zeros(len(DELTA_GRID))
        probs[np.argmin(np.abs(DELTA_GRID - delta))] = 1.0
    elif prob_method == "normal":
        probs = norm.pdf(DELTA_GRID, loc=delta, scale=10)
    return probs


def test_rotation_delta():
    print("==== testing rotation with delta ====")
    do_test_rotation("delta", n_window=2)
    do_test_rotation("delta", n_window=3)
    do_test_rotation("delta", n_window=4)


def test_rotation_normal():
    print("==== testing rotation with normal ====")
    do_test_rotation("normal", n_window=2)
    do_test_rotation("normal", n_window=3)
    do_test_rotation("normal", n_window=4)


def test_linear_delta():
    print("==== testing linear with delta ====")
    do_test_linear("delta", n_window=2)
    # do_test_linear("delta", n_window=3)
    # do_test_linear("delta", n_window=4)


def test_linear_normal():
    print("==== testing linear with normal ====")
    do_test_linear("normal", n_window=2)
    do_test_linear("normal", n_window=3)
    do_test_linear("normal", n_window=4)


def do_test_linear(prob_method, n_window):
    """ Test on rotating toy example """

    movement_dir_deg = 70
    yaw_deg = movement_dir_deg - 90

    angles_deg = [ANGLE_GLOB - yaw_deg]  # np.arange(360, step=10)
    distances_cm = np.arange(0, 50, step=0.5)
    moving_estimator = MovingEstimator(
        n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
    )
    moving_estimator.ANGLE_WINDOW_DEG = 40
    moving_estimator.ANGLE_RESOLUTION_DEG = 10

    poses = [
        [
            step * np.cos(movement_dir_deg / 180 * np.pi),
            step * np.sin(movement_dir_deg / 180 * np.pi),
            yaw_deg,
        ]
        for step in np.linspace(0, 20, 5)
    ]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = {}
        for mic_idx in MIC_IDX:
            probs = get_delta_distribution(distance, angle, mic_idx, prob_method)
            diff_dict[mic_idx] = (DELTA_GRID, probs)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < n_window - 1:
            continue

        print("--- Test without simplified angles ---")
        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = moving_estimator.get_distributions(verbose=True, simplify_angles=False)

        distance_estimate, __ = moving_estimator.get_distance_estimate(prob_distances)
        angle_estimate, __ = moving_estimator.get_angle_estimate(prob_angles)
        assert (
            abs(distance_estimate - distance) < 1
        ), f"distance at position {i}: {distance_estimate} != true {distance}"
        assert (
            abs(angle_estimate - angle) < 1
        ), f"angle at position {i}: {angle_estimate} != true {angle}"

        print("---- Test with simplified angles ----")
        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = moving_estimator.get_distributions(verbose=True, simplify_angles=True)

        distance_estimate, __ = moving_estimator.get_distance_estimate(prob_distances)
        angle_estimate, __ = moving_estimator.get_angle_estimate(prob_angles)
        assert (
            abs(distance_estimate - distance) < 1
        ), f"distance at position {i}: {distance_estimate} != true {distance}"
        assert (
            abs(angle_estimate - angle) < 1
        ), f"angle at position {i}: {angle_estimate} != true {angle}"


def do_test_rotation(prob_method, n_window):
    """ Test on rotating toy example """
    angles_deg = np.arange(360, step=10)
    distances_cm = np.arange(100, step=11)
    moving_estimator = MovingEstimator(
        n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
    )

    # toy example: the robot does a diamond-shaped movement,
    # and rotate by -90 degrees at each position.
    # the wall is located at 30cm north from the origin, horizontal.

    poses = [[0, 10, -90], [10, 20, -180], [20, 10, 90], [10, 0, 0]]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = {}
        for mic_idx in MIC_IDX:
            probs = get_delta_distribution(distance, angle, mic_idx, prob_method)
            diff_dict[mic_idx] = (DELTA_GRID, probs)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance, angle} at pose {i}")

        if i < n_window - 1:
            continue

        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = moving_estimator.get_distributions(verbose=True, simplify_angles=False)

        distance_estimate, __ = moving_estimator.get_distance_estimate(prob_distances)
        angle_estimate, __ = moving_estimator.get_angle_estimate(prob_angles)
        assert (
            distance_estimate == distance
        ), f"distance at position {i}: {distance_estimate} != true {distance}"
        assert (
            angle_estimate == angle
        ), f"angle at position {i}: {angle_estimate} != true {angle}"


if __name__ == "__main__":
    test_linear_delta()
    test_linear_normal()

    test_rotation_delta()
    test_rotation_normal()
    print("done")
