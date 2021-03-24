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
    d_slice_exp,
    distances_cm,
    yaw_deg,
    frequency,
    chosen_mics,
    method="brute",
    optimize_absorption=False,
    fit_one_gain=False,
):
    """
    :param d_slice_exp: matrix of magnitude measurements. shape: n_distances x n_mics x n_meas

    """
    from scipy.optimize import minimize, brute
    from simulation import get_dist_slice_theory, WALL_ABSORPTION

    def distance_slice_cost(coeffs, chosen_mics):
        if not optimize_absorption:
            coeffs = [WALL_ABSORPTION, *coeffs]

        d_slice = get_dist_slice_theory(
            frequency,
            distances_cm=distances_cm + coeffs[1],
            yaw_deg=yaw_deg,
            wall_absorption=coeffs[0],
            gains=coeffs[2:],
            chosen_mics=chosen_mics,
        )
        # sanity check
        assert d_slice.shape[:2] == d_slice_exp.shape[:2], (
            d_slice.shape,
            d_slice_exp.shape,
        )
        if d_slice_exp.ndim == 3:
            diff = (
                d_slice[:, :, None] - d_slice_exp
            )  # can have multiple meas. along 3rd dimension
        else:
            diff = d_slice - d_slice_exp
        return np.linalg.norm(diff[d_slice_exp > 0], ord=1)

    assert d_slice_exp.shape[1] == len(
        chosen_mics
    ), f"{d_slice_exp.shape}, {len(chosen_mics)}"

    if fit_one_gain:
        bounds = [[-1, 1], [0.1, 10]]  # offset, gains
    else:
        bounds = [[-1, 1]] + [[0.1, 10]] * len(chosen_mics)

    if optimize_absorption:
        bounds = [
            [0, 0.99]
        ] + bounds  # 1 would mean all energy lost and no interference.

    if method == "brute":
        coeffs = brute(distance_slice_cost, bounds, args=(chosen_mics,))
    elif method == "minimize":
        x0 = [(b[1] + b[0]) / 2 for b in bounds]
        sol = minimize(distance_slice_cost, x0, args=(chosen_mics), bounds=bounds,)
        if not sol.success:
            print("Warning: did not converge", sol.message)
        coeffs = sol.x

    if not optimize_absorption:
        coeffs = [WALL_ABSORPTION, *coeffs]

    d_slice = get_dist_slice_theory(
        frequency,
        distances_cm=distances_cm + coeffs[1],
        yaw_deg=yaw_deg,
        wall_absorption=coeffs[0],
        gains=coeffs[2:],
        chosen_mics=chosen_mics,
    )
    return coeffs, d_slice, distance_slice_cost(coeffs, chosen_mics)
