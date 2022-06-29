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

from helpers import measure_wall, get_diff_dict, DISTANCE_GLOB

VERBOSE = False
ESTIMATION_METHOD = "max"  # only max works for these examples.

def test_example_delta(method="moving"):
    print("==== testing example with delta ====")
    do_test_example("delta", n_window=2, method=estimator)
    do_test_example("delta", n_window=3, method=estimator)
    do_test_example("delta", n_window=4, method=estimator)


def test_example_normal(method="moving"):
    print("==== testing example with normal ====")
    if method == "moving":
        do_test_example("normal", n_window=2, method=estimator)
        do_test_example("normal", n_window=3, method=estimator)
        do_test_example("normal", n_window=4, method=estimator)
    else:
        do_test_example("normal", n_window=2, method=estimator)


def test_simplify_angles(method="moving"):
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


def test_linear_many_directions(method="moving"):
    print("==== testing many directions ====")
    for movement_dir_deg in np.arange(360, step=45):
        do_test_linear(
            "normal", n_window=3, movement_dir_deg=movement_dir_deg, method=estimator
        )
        do_test_linear(
            "delta", n_window=3, movement_dir_deg=movement_dir_deg, method=estimator
        )


def test_linear_delta(method="moving"):
    print("==== testing linear with delta ====")
    do_test_linear("delta", n_window=2, method=estimator)
    do_test_linear("delta", n_window=3, method=estimator)
    do_test_linear("delta", n_window=4, method=estimator)


def test_linear_normal(method="moving"):
    print("==== testing linear with normal ====")
    if method == "moving":
        do_test_linear("normal", n_window=2, method=estimator)
        do_test_linear("normal", n_window=3, method=estimator)
        do_test_linear("normal", n_window=4, method=estimator)
    else:
        do_test_linear("normal", n_window=2, method=estimator)

def test_rotation_normal(method="moving"):
    print("==== testing rotation with normal ====")
    if method == "moving":
        do_test_rotation("normal", n_window=2, method=estimator)
        do_test_rotation("normal", n_window=3, method=estimator)
        do_test_rotation("normal", n_window=4, method=estimator)
    else:
        do_test_rotation("normal", n_window=2, method=estimator)


def test_histogram_predict(prob_method="normal", n_window=2, verbose=False):
    angles_deg = np.arange(360, step=30)
    distances_cm = np.arange(50, step=10)
    estimator = HistogramEstimator(
        angles_deg=angles_deg, distances_cm=distances_cm
    )
    poses = [[0, 10, -90], [10, 20, -180], [20, 10, 90], [10, 0, 0]]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        estimator.add_distributions(diff_dict, pose[:2], pose[2])
        if verbose:
            print(f"added {distance, angle} at pose {i}")

        prior1 = estimator.predict_slow(verbose=verbose)
        prior2 = estimator.predict(verbose=verbose)
        try:
            np.testing.assert_allclose(prior1, prior2)
        except:
            plt.figure()
            plt.semilogy(prior1, label="prior slow")
            plt.semilogy(prior2, label="prior fast")
            plt.legend()
            plt.show()
            raise 


def test_histogram_distributions(verbose=False, movement_dir_deg=90):
    yaw_deg = movement_dir_deg - 90
    prob_method = "normal"

    angles_deg = np.array([90 + yaw_deg])
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
        if verbose:
            print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < 1:
            continue

        # plt.figure()
        # plt.plot(estimator.distances_cm, estimator.prior, label="prior before predict")
        # plt.plot(estimator.distances_cm, estimator.posterior, label="posterior before predict")
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
    method="moving",
):
    """ Test on linear movement towards the wall. """
    if verbose:
        print(f"testing {prob_method}, {n_window}, {movement_dir_deg}, {simplify_angles}")
    yaw_deg = movement_dir_deg - 90

    step_angle_deg = 10
    step_distance_cm = 0.5
    # angles_deg = [ANGLE_GLOB - yaw_deg]  # ground truth only
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.arange(0, 50, step=step_distance_cm)
    if method == "moving":
        estimator = MovingEstimator(
            n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
        )
        estimator.ANGLE_WINDOW_DEG = 40
        estimator.ANGLE_RESOLUTION_DEG = 10
    elif method == "histogram":
        estimator = HistogramEstimator(
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
        estimator.add_distributions(diff_dict, pose[:2], pose[2])
        if verbose:
            print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < n_window - 1:
            continue

        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = estimator.get_distributions(
            verbose=verbose, simplify_angles=simplify_angles
        )

        distance_estimate, __ = estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        if distance_estimate is None:
            print(f"Warning: for pose {i} and movement angle {movement_dir_deg}, distance estimate is None")
            continue

        if simplify_angles:
            angle_estimate, __ = get_estimate(
                angles, prob_angles, method=ESTIMATION_METHOD
            )
        else:
            angle_estimate, __ = estimator.get_angle_estimate(
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
            ) = estimator.get_distributions(
                verbose=False, simplify_angles=simplify_angles, plot_angles=[angle]
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


def do_test_rotation(
    prob_method,
    n_window,
    static_distance_cm=30,
    verbose=VERBOSE,
    method="moving",
):
    """ Test on linear movement towards the wall. """
    if verbose:
        print(f"testing {prob_method}, {n_window}, {static_distance_cm}")

    step_angle_deg = 10
    angles_deg = np.arange(360, step=step_angle_deg)
    distances_cm = np.array([static_distance_cm])
    if method == "moving":
        estimator = MovingEstimator(
            n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
        )
        estimator.ANGLE_WINDOW_DEG = 40
        estimator.ANGLE_RESOLUTION_DEG = 10
    elif method == "histogram":
        estimator = HistogramEstimator(
            angles_deg=angles_deg, distances_cm=distances_cm
        )

    poses = [
        [
            0,
            DISTANCE_GLOB-static_distance_cm,
            yaw_deg,
        ]
        for yaw_deg in np.arange(360, step=step_angle_deg)  # 0, 5, 15
    ]
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        estimator.add_distributions(diff_dict, pose[:2], pose[2])
        if verbose:
            print(f"added {distance:.2f}, {angle:.0f} at pose {i}")

        if i < n_window - 1:
            continue

        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = estimator.get_distributions(
            verbose=verbose, simplify_angles=False
        )

        distance_estimate, __ = estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        angle_estimate, __ = estimator.get_angle_estimate(
            prob_angles, method=ESTIMATION_METHOD
        )

        assert (
            abs(distance_estimate - distance) == 0
        ), f"distance at position {i}: {distance_estimate} != true {distance}. Correct angle: {angle}"
        assert (
            abs(angle_estimate - angle) <= 2 * step_angle_deg
        ), f"angle at position {i}: {angle_estimate} != true {angle}. Correct distance: {distance}"


def do_test_example(prob_method, n_window, method="moving", verbose=VERBOSE):
    """Test on rotating toy example"""
    step_angle_deg = 30 
    step_distance_cm = 5
    angles_deg = np.arange(0, 360, step=step_angle_deg)
    distances_cm = np.arange(70, step=step_distance_cm)
    if method == "moving":
        estimator = MovingEstimator(
            n_window=n_window, angles_deg=angles_deg, distances_cm=distances_cm
        )
    elif method == "histogram":
        estimator = HistogramEstimator(
            angles_deg=angles_deg, distances_cm=distances_cm
        )

    # toy example: the robot does a diamond-shaped movement,
    # and rotate by -90 degrees at each position.
    # the wall is located at 30cm north from the origin, horizontal.
    # see test_estimator.png for drawing.

    poses = [[0, 10, -90], [10, 20, -180], [20, 10, 90], [10, 0, 0]]
    #poses = [[10, 20, -180], [20, 10, 90]] # backward
    #poses = [[0, 10, -90], [10, 20, -180]] # forward
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        diff_dict = get_diff_dict(distance, angle, prob_method)
        if verbose:
            print(f"adding {distance, angle} at pose {i}")
        estimator.add_distributions(diff_dict, pose[:2], pose[2])

        if i < n_window - 1:
            continue

        if verbose:
            print(f"doing inference...")
        (
            distances,
            prob_distances,
            angles,
            prob_angles,
        ) = estimator.get_distributions(verbose=VERBOSE, simplify_angles=False)

        distance_estimate, __ = estimator.get_distance_estimate(
            prob_distances, method=ESTIMATION_METHOD
        )
        angle_estimate, __ = estimator.get_angle_estimate(
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

            plt.figure()
            plt.plot(angles, prob_angles, color="C0", label="posterior")
            if method == "histogram":
                prior = estimator.get_marginal(posterior=False, distance_cm=distance_estimate)
                posterior = estimator.get_marginal(posterior=True, distance_cm=distance_estimate)
                plt.plot(angles, prior, color="C1", label="prior marginal")
                plt.plot(angles, posterior, color="C2", label="posterior marginal")
            plt.axvline(angle_estimate, color='red', label="estimated")
            plt.axvline(angle, color='green', label="correct")
            plt.title(f"pose {i}")
            plt.legend()
            plt.show()

if __name__ == "__main__":
    import sys

    print("---------------- Testing histogram filter -----------------")
    estimator = "histogram"
    test_histogram_predict()
    test_histogram_distributions()
    test_linear_normal(estimator)
    test_rotation_normal(estimator)
    test_example_normal(estimator)

    print("---------------- Testing moving filter -----------------")
    estimator = "moving"
    test_linear_delta(estimator)
    test_linear_normal(estimator)
    test_rotation_normal(estimator)
    test_linear_many_directions(estimator)
    test_simplify_angles(estimator)

    test_example_normal(estimator)
    test_example_delta(estimator)
    print("done")
