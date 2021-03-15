#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
calibration.py: methods for gain calibration
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd


def plot_calibration(x, ys, function, ax):
    uniform_x = np.linspace(min(x), max(x), 100)
    uniform_ys = function(uniform_x)
    for i in range(ys.shape[0]):
        ax.semilogy(x, ys[i], color=f"C{i}", label=f"mic{i}")
        ax.scatter(uniform_x, uniform_ys[i], color=f"C{i}")
    ax.grid("both")
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("amplitude")
    ax.set_ylim(1e-3, 1e2)
    ax.legend()


def get_calibration_function(ax=None):
    from scipy.interpolate import interp1d
    from pandas_utils import filter_by_dict

    calib_df = pd.read_pickle("results/calibration_results.pkl")
    chosen_dict = {
        "source": "sweep_buzzer",
        "method": np.median,
    }
    calib_df = filter_by_dict(calib_df, chosen_dict)
    assert len(calib_df) == 1, calib_df
    row = calib_df.iloc[0]

    calib_psd = row.psd
    calib_f = row.frequencies
    calib_function = interp1d(
        x=calib_f, y=calib_psd, kind="linear", fill_value="extrapolate"
    )

    if ax is not None:
        plot_calibration(row.frequencies, row.psd, calib_function, ax)
    return calib_function


def get_calibration_function_matrix(df_matrix, df_freq, ax=None):
    from scipy.interpolate import interp1d
    from wall_detector import prune_df_matrix

    df_matrix_pruned, df_freq, __ = prune_df_matrix(df_matrix, df_freq)
    median_values = np.nanmedian(df_matrix_pruned, axis=2)  # n_mics x n_freqs
    calib_function = interp1d(
        df_freq, median_values, kind="quadratic", fill_value="extrapolate"
    )
    if ax is not None:
        plot_calibration(df_freq, median_values, calib_function, ax=ax)
    np.testing.assert_allclose(calib_function(df_freq[0]), median_values[:, 0])
    return calib_function


def get_calibration_function_dict(ax=None, **filter_dict):
    from pandas_utils import filter_by_dict

    fname = "results/wall_analysis.pkl"
    results_df = pd.read_pickle(fname)
    df = filter_by_dict(results_df, filter_dict)
    assert len(df) == 1, df
    row = df.iloc[0]
    return get_calibration_function_matrix(row.df_matrix, row.df_freq, ax=ax)


def fit_distance_slice(
    slice_exp,
    distances_cm,
    yaw_deg,
    frequency,
    chosen_mics,
    method="brute",
    optimize_absorption=False,
):
    from scipy.optimize import minimize, brute
    from simulation import get_dist_slice_theory, WALL_ABSORPTION

    def distance_slice_cost(coeffs, d_slice_exp_flat, chosen_mics):
        if not optimize_absorption:
            coeffs = [WALL_ABSORPTION, *coeffs]

        d_slice = get_dist_slice_theory(
            frequency,
            distances_cm=distances_cm + coeffs[2],
            yaw_deg=yaw_deg,
            wall_absorption=coeffs[0],
            gain_x=coeffs[1],
            chosen_mics=chosen_mics,
        )
        # Need to pass d_slice flattened because otherwise
        # there is an error with args.
        d_slice_exp = d_slice_exp_flat.reshape((-1, 4))[:, chosen_mics]

        # sanity check
        assert d_slice.shape == d_slice_exp.shape
        return np.linalg.norm(d_slice - d_slice_exp, ord=1)

    assert slice_exp.shape[1] <= 4, "input has to be of shape n_freqs x n_mics!"

    bounds = [[0.1, 10], [-1, 1]]
    if optimize_absorption:
        bounds = [[0, 1.0]] + bounds
    if method == "brute":
        coeffs = brute(
            distance_slice_cost, bounds, args=(slice_exp.flatten(), chosen_mics)
        )
    elif method == "minimize":
        x0 = [(b[1] + b[0] / 2) for b in bounds]
        sol = minimize(
            distance_slice_cost,
            x0,
            args=(slice_exp.flatten(), chosen_mics),
            bounds=bounds,
        )
        if not sol.success:
            print("Warning: did not converge", sol.message)
        coeffs = sol.x

    if not optimize_absorption:
        coeffs = [WALL_ABSORPTION, *coeffs]

    d_slice = get_dist_slice_theory(
        frequency,
        distances_cm=distances_cm + coeffs[2],
        yaw_deg=yaw_deg,
        wall_absorption=coeffs[0],
        gain_x=coeffs[1],
        chosen_mics=chosen_mics,
    )
    return (
        coeffs,
        d_slice,
        distance_slice_cost(coeffs, slice_exp.flatten(), chosen_mics),
    )
