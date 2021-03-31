#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
calibration.py: methods for gain calibration
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd

OFFSET_BOUNDS = [-1, 1]
GAIN_BOUNDS = [0.1, 10]
# wall absorption bounds: 1 would mean all energy lost and no interference.
ABS_BOUNDS = [0.0, 0.8]
YAW_DEG = 0


def plot_calibration(x, ys, function, ax):
    uniform_x = np.linspace(min(x), max(x), 100)
    uniform_ys = function(uniform_x)
    for i in range(ys.shape[0]):
        ax.scatter(x, ys[i], color=f"C{i}", label=f"mic{i}")
        ax.plot(uniform_x, uniform_ys[i], color=f"C{i}")
    ax.grid("both")
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("amplitude")
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


# TODO(FD): deprecated
def get_calibration_function_new(exp_name, mic_type, method="fit", ax=None, motors=0):
    from wall_detector import WallDetector

    wall_detector = WallDetector()
    wall_detector.fill_from_backup(exp_name, mic_type, motors=motors)
    return wall_detector.get_calib_function(method=method, ax=ax)


def get_calibration_function_fit(
    exp_name, mic_type, ax=None, motors=0, fit_one_gain=False
):
    from wall_detector import WallDetector, prune_df_matrix
    from scipy.interpolate import interp1d

    wall_detector = WallDetector()
    wall_detector.fill_from_backup(exp_name, mic_type, motors=motors)

    matrix, distances, frequencies = wall_detector.get_df_matrix()
    matrix, frequencies, *_ = prune_df_matrix(matrix, frequencies)

    mics = wall_detector.get_mics()
    gains = np.zeros((len(mics), len(frequencies)))
    for i, f in enumerate(frequencies):
        slices_raw = matrix[:, i, :]
        coeffs, slices_fit, cost = fit_distance_slice(
            slices_raw.T,
            distances,
            method="minimize",
            yaw_deg=YAW_DEG,
            frequency=f,
            chosen_mics=mics,
            optimize_absorption=True,
            fit_one_gain=fit_one_gain,
        )
        gains[:, i] = coeffs[2:]

    calib_function = interp1d(
        frequencies, gains, kind="linear", fill_value="extrapolate", assume_sorted=True
    )
    if ax is not None:
        plot_calibration(frequencies, gains, calib_function, ax=ax)
    return calib_function


def get_calibration_function_median(
    exp_name, mic_type, ax=None, motors=0, fit_one_gain=False
):
    from wall_detector import WallDetector, prune_df_matrix
    from scipy.interpolate import interp1d

    wall_detector = WallDetector()
    wall_detector.fill_from_backup(exp_name, mic_type, motors=motors)

    matrix, distances, frequencies = wall_detector.get_df_matrix()
    matrix, frequencies, *_ = prune_df_matrix(matrix, frequencies)

    if fit_one_gain:
        gains = np.repeat(
            np.nanmedian(matrix, axis=[0, 2])[None, :], matrix.shape[0], axis=0
        )
    else:
        gains = np.nanmedian(matrix, axis=2)

    calib_function = interp1d(
        frequencies, gains, kind="linear", fill_value="extrapolate", assume_sorted=True
    )
    if ax is not None:
        plot_calibration(frequencies, gains, calib_function, ax=ax)
    return calib_function


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
    set missing measurements to zero or nan

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

    if np.any(np.isnan(d_slice_exp)):
        d_slice_exp[np.isnan(d_slice_exp)] = 0
    if fit_one_gain:
        bounds = [OFFSET_BOUNDS, GAIN_BOUNDS]  # offset, gains
    else:
        bounds = [OFFSET_BOUNDS] + [GAIN_BOUNDS] * len(chosen_mics)

    if optimize_absorption:
        bounds = [ABS_BOUNDS] + bounds

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
