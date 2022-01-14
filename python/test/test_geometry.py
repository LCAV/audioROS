#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geometry.py: Test geometrical conversions from distances to path differences etc.
"""
import numpy as np
import sys, os

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.geometry import *

EPS = 1e-10


def create_context(setup):
    n_mics = 4
    dim = 2
    if setup == "random":
        context = Context.get_random_setup(n_mics=n_mics, dim=dim)
    elif setup == "crazyflie":
        context = Context.get_crazyflie_setup(dim=dim)
    elif setup == "standard":
        context = Context.get_standard_setup(dim=dim)
    return context


def conversion(context):
    for azimuth_deg in np.arange(0, 360, step=10):
        for distance in context.get_possible_distances():
            for mic_idx in range(context.mics.shape[0]):
                # get path difference from wall located at "normal"
                delta_cm = context.get_delta(
                    azimuth_deg=azimuth_deg,
                    distances_cm=distance * 1e2,
                    mic_idx=mic_idx,
                )

                # get distance, given angle
                distance_total = context.get_distance(delta_cm, azimuth_deg, mic_idx)
                # distance_total = context.get_total_distance(delta, azimuth_deg, mic_idx)
                np.testing.assert_allclose(distance_total, distance * 1e2)

                # get angle, given distance_source
                azimuth_est = context.get_angles(delta_cm * 1e-2, distance, mic_idx)
                assert azimuth_deg in azimuth_est, (azimuth_deg, azimuth_est)

                # compare the different ways to compute delta.
                normal = context.get_normal(distance * 1e2, azimuth_deg)
                source_image = context.get_source_image(normal)
                source = context.source
                mic = context.mics[mic_idx, :]

                delta_pos = np.linalg.norm(source_image - mic) - np.linalg.norm(
                    source - mic
                )
                np.testing.assert_allclose(
                    delta_pos * 1e2, delta_cm, rtol=EPS, atol=EPS
                )


def delta_gradient(context):
    err_tol = 1e-5
    mic_idx = 1

    for azimuth_deg in np.arange(0, 360, step=10):
        for distance_cm in context.get_possible_distances() * 1e2:
            gradient = context.get_delta_gradient(azimuth_deg, distance_cm, mic_idx)
            delta0 = context.get_delta(azimuth_deg, distance_cm, mic_idx)
            # decrease delta until convergence
            finite_diff_prev = None
            converged = False
            for diff_cm in np.logspace(-13, -1, 13)[::-1]:
                delta1 = context.get_delta(azimuth_deg, distance_cm + diff_cm, mic_idx)

                finite_diff = np.abs((delta1 - delta0) / diff_cm)
                if (
                    finite_diff_prev is not None
                    and abs(finite_diff - finite_diff_prev) < 1e-5
                ):
                    converged = True
                    break
                finite_diff_prev = finite_diff

            assert converged
            error = abs(gradient - finite_diff)
            if error > err_tol:
                print(
                    f"{azimuth_deg}/{distance_cm}: warning, error above {err_tol:.2e}: {error:.2e}"
                )
            assert error <= err_tol


def test_simple_cases():
    """
      1   1   1   1
    s   m           s'
    x---0---|--- ---x

    """

    context = Context(dim=2)
    context.mics = np.array([1e-2, 0]).reshape((1, 2))
    context.source = np.array([0, 0])
    distance_cm = 2
    azimuth_deg = 0

    np.testing.assert_allclose(
        context.get_normal(distance_cm, azimuth_deg), [distance_cm * 1e-2, 0]
    )

    np.testing.assert_allclose(context.get_direct_path(mic_idx=0), 1e-2)
    np.testing.assert_allclose(context.get_theta0(mic_idx=0), 0)
    np.testing.assert_allclose(
        context.get_delta(azimuth_deg, distance_cm, mic_idx=0), 2
    )


def test_crazyflie():
    context = create_context("crazyflie")

    conversion(context)
    delta_gradient(context)


def test_standard():
    context = create_context("standard")
    context.source = np.zeros_like(context.source)

    conversion(context)
    delta_gradient(context)


def test_random():
    context = create_context("random")
    context.source = np.zeros_like(context.source)

    conversion(context)
    delta_gradient(context)


if __name__ == "__main__":
    # test_simple_cases()
    # test_standard()
    test_crazyflie()
    # test_random()
