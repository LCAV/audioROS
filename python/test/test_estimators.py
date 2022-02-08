#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_estimators.py:  Test moving estimator class.
"""

import sys, os

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.moving_estimators import MovingEstimator, get_estimate

from helpers import measure_wall, get_delta_distribution

MIC_IDX = [0, 1]

VERBOSE = False

ESTIMATION_METHOD = "max"  # only max works for these examples.


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


def test_simplify_angles():
    print("==== testing simplified angles ====")
    for movement_dir_deg in [90, 270]:
        do_test_linear(
            "delta", n_window=2, movement_dir_deg=movement_dir_deg, simplify_angles=True
        )
        do_test_linear(
            "normal",
            n_window=2,
            movement_dir_deg=movement_dir_deg,
            simplify_angles=True,
        )

        do_test_linear(
            "delta", n_window=3, movement_dir_deg=movement_dir_deg, simplify_angles=True
        )
        do_test_linear(
            "normal",
            n_window=3,
            movement_dir_deg=movement_dir_deg,
            simplify_angles=True,
        )


def test_linear_many_directions():
    print("==== testing many directions ====")
    for movement_dir_deg in np.arange(360, step=20):
        do_test_linear("normal", n_window=3, movement_dir_deg=movement_dir_deg)
        do_test_linear("delta", n_window=3, movement_dir_deg=movement_dir_deg)


def test_linear_delta():
    print("==== testing linear with delta ====")
    do_test_linear("delta", n_window=2)
    do_test_linear("delta", n_window=3)
    do_test_linear("delta", n_window=4)


def test_linear_normal():
    print("==== testing linear with normal ====")
    do_test_linear("normal", n_window=2)
    do_test_linear("normal", n_window=3)
    do_test_linear("normal", n_window=4)


def do_test_linear(
    prob_method, n_window, movement_dir_deg=70, simplify_angles=False, verbose=VERBOSE
):
    """ Test on rotating toy example """
    print(f"testing {prob_method}, {n_window}, {movement_dir_deg}, {simplify_angles}")
    yaw_deg = movement_dir_deg - 90

    step_angle_deg = 10
    step_distance_cm = 0.5
    # angles_deg = [ANGLE_GLOB - yaw_deg]  # ground truth only
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.arange(0, 60, step=step_distance_cm)
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
        for step in np.linspace(0, 19, 3)
    ]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)

        diff_dict = {}
        for mic_idx in MIC_IDX:
            delta_grid, probs = get_delta_distribution(
                distance, angle, mic_idx, prob_method
            )
            diff_dict[mic_idx] = (delta_grid, probs)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        # print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < n_window - 1:
            continue

        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = moving_estimator.get_distributions(
            verbose=verbose, simplify_angles=simplify_angles
        )

        distance_estimate, __ = moving_estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        if simplify_angles:
            angle_estimate, __ = get_estimate(
                angles, prob_angles, method=ESTIMATION_METHOD
            )
        else:
            angle_estimate, __ = moving_estimator.get_angle_estimate(
                prob_angles, method=ESTIMATION_METHOD
            )

        try:
            assert (
                abs(distance_estimate - distance) <= 2 * step_distance_cm
            ), f"distance at position {i}: {distance_estimate} != true {distance}. Correct angle: {angle}"
            assert (
                abs(angle_estimate - angle) <= 2 * step_angle_deg
            ), f"angle at position {i}: {angle_estimate} != true {angle}. Correct distance: {distance}"
        # except AssertionError as e:
        except Exception as e:
            print(f"Caught Exception {e}, running again with verbose.")
            (
                distances,
                prob_distances,
                angles,
                prob_angles,
            ) = moving_estimator.get_distributions(
                verbose=True, simplify_angles=simplify_angles, plot_angles=[angle]
            )
            import matplotlib.pylab as plt

            fig, axs = plt.subplots(2)
            axs[0].plot(angles, prob_angles)
            axs[0].axvline(angle_estimate, color="r")
            axs[0].axvline(angle, color="g", ls=":")
            axs[0].set_xlabel("angle [deg]")
            axs[0].grid(which="both")
            axs[1].plot(distances, prob_distances)
            axs[1].axvline(distance_estimate, color="r")
            axs[1].axvline(distance, color="g", ls=":")
            axs[1].set_xlabel("distance [cm]")
            axs[1].grid(which="both")
            plt.show(block=True)
            raise e


def do_test_rotation(prob_method, n_window):
    """ Test on rotating toy example """
    step_angle_deg = 10
    step_distance_cm = 10
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.arange(100, step=step_distance_cm)
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
            delta_grid, probs = get_delta_distribution(
                distance, angle, mic_idx, prob_method
            )
            diff_dict[mic_idx] = (delta_grid, probs)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance, angle} at pose {i}")

        if i < n_window - 1:
            continue

        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = moving_estimator.get_distributions(verbose=VERBOSE, simplify_angles=False)

        distance_estimate, __ = moving_estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        angle_estimate, __ = moving_estimator.get_angle_estimate(
            prob_angles, method=ESTIMATION_METHOD
        )
        assert (
            abs(distance_estimate - distance) <= 2 * step_distance_cm
        ), f"distance at position {i}: {distance_estimate} != true {distance}"
        assert (
            abs(angle_estimate - angle) <= 2 * step_angle_deg
        ), f"angle at position {i}: {angle_estimate} != true {angle}"


if __name__ == "__main__":
    # from utils.geometry import Context
    # print(Context.get_platform_setup().mics)

    test_linear_delta()
    test_linear_normal()
    test_linear_many_directions()
    test_simplify_angles()

    test_rotation_delta()
    test_rotation_normal()
    print("done")
