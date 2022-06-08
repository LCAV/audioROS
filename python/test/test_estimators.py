#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_estimators.py:  Test moving estimator class.
"""

import sys, os

import numpy as np
import matplotlib.pylab as plt

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.moving_estimators import MovingEstimator
from utils.histogram_estimators import HistogramEstimator
from utils.base_estimator import get_estimate

from helpers import measure_wall, get_diff_dict

VERBOSE = False
ESTIMATION_METHOD = "max"  # only max works for these examples.


def test_rotation_delta(estimator="moving"):
    print("==== testing rotation with delta ====")
    do_test_rotation("delta", n_window=2, estimator=estimator)
    do_test_rotation("delta", n_window=3, estimator=estimator)
    do_test_rotation("delta", n_window=4, estimator=estimator)


def test_rotation_normal(estimator="moving"):
    print("==== testing rotation with normal ====")
    do_test_rotation("normal", n_window=2, estimator=estimator)
    do_test_rotation("normal", n_window=3, estimator=estimator)
    do_test_rotation("normal", n_window=4, estimator=estimator)


def test_simplify_angles(estimator="moving"):
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


def test_linear_many_directions(estimator="moving"):
    print("==== testing many directions ====")
    for movement_dir_deg in np.arange(360, step=20):
        do_test_linear(
            "normal", n_window=3, movement_dir_deg=movement_dir_deg, estimator=estimator
        )
        do_test_linear(
            "delta", n_window=3, movement_dir_deg=movement_dir_deg, estimator=estimator
        )


def test_linear_delta(estimator="moving"):
    print("==== testing linear with delta ====")
    do_test_linear("delta", n_window=2, estimator=estimator)
    do_test_linear("delta", n_window=3, estimator=estimator)
    do_test_linear("delta", n_window=4, estimator=estimator)


def test_linear_normal(estimator="moving"):
    print("==== testing linear with normal ====")
    if estimator == "moving":
        do_test_linear("normal", n_window=2, estimator=estimator)
        do_test_linear("normal", n_window=3, estimator=estimator)
        do_test_linear("normal", n_window=4, estimator=estimator)
    else:
        do_test_linear("normal", n_window=2, estimator=estimator)


def test_histogram_predict(prob_method="normal", n_window=2, verbose=False):
    angles_deg = np.arange(360, step=30)
    distances_cm = np.arange(50, step=10)
    moving_estimator = HistogramEstimator(
        angles_deg=angles_deg, distances_cm=distances_cm
    )
    poses = [[0, 10, -90], [10, 20, -180], [20, 10, 90], [10, 0, 0]]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance, angle} at pose {i}")

        print("predicting slow...")
        prior1 = moving_estimator.predict_slow(verbose=verbose)
        print("predicting fast...")
        prior2 = moving_estimator.predict(verbose=verbose)
        try:
            np.testing.assert_allclose(prior1, prior2)
        except:
            plt.figure()
            plt.semilogy(prior1)
            plt.semilogy(prior2)
            plt.show()
            return


def test_histogram_distributions(verbose=False, movement_dir_deg=90):
    yaw_deg = movement_dir_deg - 90
    prob_method = "normal"

    angles_deg = [90 + yaw_deg]
    step_distance_cm = 1
    distances_cm = np.arange(35, step=step_distance_cm)
    estimator = HistogramEstimator(angles_deg=angles_deg, distances_cm=distances_cm)

    estimator = HistogramEstimator(angles_deg=angles_deg, distances_cm=distances_cm)

    poses = [
        [
            step * np.cos(movement_dir_deg / 180 * np.pi),
            step * np.sin(movement_dir_deg / 180 * np.pi),
            yaw_deg,
        ]
        for step in np.arange(26, step=5)  # 0, 5, 15, 20, 25
    ]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        assert angle == angles_deg[0]
        diff_dict = get_diff_dict(distance, angle, prob_method)
        estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < 1:
            continue

        # plt.figure()
        # plt.plot(estimator.distances_cm, estimator.prior, label="prior before predict")
        # plt.plot(estimator.distances_cm, estimator.posterior, label="posterior before predict")

        print(f"running inference...")
        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = estimator.get_distributions(verbose=verbose, simplify_angles=False)

        # plt.plot(estimator.distances_cm, estimator.prior, label="prior after predict")
        # plt.plot(estimator.distances_cm, estimator.posterior, label="posterior after update")
        # plt.legend()
        # plt.show()

        distance_estimate, __ = estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        if distance_estimate is None:
            continue
        else:
            print("distance estimate:", distance_estimate)
        angle_estimate, __ = estimator.get_angle_estimate(
            prob_angles, method=ESTIMATION_METHOD
        )
        try:
            assert (
                abs(distance_estimate - distance) <= 2 * step_distance_cm
            ), f"distance at position {i}: {distance_estimate} != true {distance}. Correct angle: {angle}"
        except Exception as e:
            print(e)


def do_test_linear(
    prob_method,
    n_window,
    movement_dir_deg=90,
    simplify_angles=False,
    verbose=VERBOSE,
    estimator="moving",
):
    """Test on rotating toy example"""
    print(f"testing {prob_method}, {n_window}, {movement_dir_deg}, {simplify_angles}")
    yaw_deg = movement_dir_deg - 90

    step_angle_deg = 10
    step_distance_cm = 0.5
    # angles_deg = [ANGLE_GLOB - yaw_deg]  # ground truth only
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.arange(0, 50, step=step_distance_cm)
    if estimator == "moving":
        moving_estimator = MovingEstimator(
            n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
        )
        moving_estimator.ANGLE_WINDOW_DEG = 40
        moving_estimator.ANGLE_RESOLUTION_DEG = 10
    elif estimator == "histogram":
        moving_estimator = HistogramEstimator(
            angles_deg=angles_deg, distances_cm=distances_cm
        )

    poses = [
        [
            step * np.cos(movement_dir_deg / 180 * np.pi),
            step * np.sin(movement_dir_deg / 180 * np.pi),
            yaw_deg,
        ]
        for step in np.arange(20, step=5)  # 0, 5, 15
    ]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

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
        if distance_estimate is None:
            continue
        else:
            print("distance estimate:", distance_estimate)
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


def do_test_rotation(prob_method, n_window, estimator="moving"):
    """Test on rotating toy example"""
    step_angle_deg = 30
    step_distance_cm = 10
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.arange(100, step=step_distance_cm)
    if estimator == "moving":
        moving_estimator = MovingEstimator(
            n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
        )
    elif estimator == "histogram":
        moving_estimator = HistogramEstimator(
            angles_deg=angles_deg, distances_cm=distances_cm
        )

    # toy example: the robot does a diamond-shaped movement,
    # and rotate by -90 degrees at each position.
    # the wall is located at 30cm north from the origin, horizontal.
    poses = [[0, 10, -90], [10, 20, -180], [20, 10, 90], [10, 0, 0]]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        moving_estimator.add_distributions(diff_dict, pose[:2], pose[2])
        print(f"added {distance, angle} at pose {i}")

        if i < n_window - 1:
            continue

        print(f"running inference...")
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
        try:
            assert (
                abs(distance_estimate - distance) <= 2 * step_distance_cm
            ), f"distance at position {i}: {distance_estimate} != true {distance}"
            assert (
                abs(angle_estimate - angle) <= 2 * step_angle_deg
            ), f"angle at position {i}: {angle_estimate} != true {angle}"
        except Exception as e:
            print(e)


if __name__ == "__main__":

    estimator = "histogram"
    test_histogram_predict(verbose=True)
    test_histogram_distributions()
    test_linear_normal(estimator)
    test_rotation_normal(estimator)

    estimator = "moving"
    test_linear_delta(estimator)
    test_linear_normal(estimator)
    test_linear_many_directions(estimator)
    test_simplify_angles(estimator)

    test_rotation_normal(estimator)
    test_rotation_delta(estimator)
    print("done")
