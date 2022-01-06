#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
calibration.py: methods for gain calibration
"""

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from scipy.optimize import minimize, brute

from .constants import PLATFORM
from .data_collector import DataCollector, prune_df_matrix
from .pandas_utils import filter_by_dict
from .simulation import get_dist_slice_theory, WALL_ABSORPTION

if PLATFORM == "epuck":
    MOTORS = "sweep_and_move"
else:
    MOTORS = 0


# wall absorption bounds: 1 would mean all energy lost and no interference.
ABS_BOUNDS = [0.1, 0.99]
OFFSET_BOUNDS = [0, 20]
GAIN_BOUNDS = [1, 100]
YAW_DEG = 0

# interpolation parameters
FILL_VALUE = "extrapolate"  # 0.0  # below and above
KIND = "linear"
BOUNDS_ERROR = False


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


def get_calibration_function_median(
    exp_name, mic_type, ax=None, motors=MOTORS, snr="", fit_one_gain=False
):
    data_collector = DataCollector()
    data_collector.fill_from_backup(exp_name, mic_type, motors=motors, snr=snr)

    # FYI, matrix has non-squared values.
    matrix, distances, frequencies = data_collector.get_df_matrix()
    matrix, frequencies, *_ = prune_df_matrix(matrix, frequencies)

    if fit_one_gain:
        gains = np.repeat(
            np.nanmedian(matrix, axis=[0, 2])[None, :], matrix.shape[0], axis=0
        )
    else:
        gains = np.nanmedian(matrix, axis=2)

    calib_function = interp1d(
        frequencies, gains, kind=KIND, fill_value=FILL_VALUE, bounds_error=BOUNDS_ERROR,
    )
    if ax is not None:
        plot_calibration(frequencies, gains, calib_function, ax=ax)
    return calib_function, frequencies


def get_calibration_function_moving(
    exp_name,
    ax=None,
    motors=MOTORS,
    fit_one_gain=False,
    appendix_list=[""],
    check_height=True,
    check_crash=True,
    verbose=False,
):
    results_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
    rows = results_df.loc[
        (results_df.motors == motors) & (results_df.appendix.isin(appendix_list)), :
    ]

    valid_stfts = []
    for i, row in rows.iterrows():
        print(i)
        valid = np.ones(row.positions.shape[0], dtype=bool)
        if check_height:
            valid &= row.positions[:, 2] > 0.3
            # print(f'using {stft.shape[0]} out of {row.stft.shape[0]}')
            if verbose:
                print(f"{np.sum(valid)} / {row.positions.shape[0]} flying")
        if check_crash:
            magnitudes = np.sum(np.mean(np.abs(row.stft), axis=1), axis=-1)  # dist
            crash = np.where(
                (magnitudes - np.mean(magnitudes)) > (2 * np.std(magnitudes))
            )[0]
            if len(crash) > 0:
                crash = crash[0]
                if verbose:
                    print(f"crash at {crash}")
            else:
                crash = 0
                if verbose:
                    print("no crash")
            valid[crash:] = 0
        stft = row.stft[valid, ...]  # dist, mic, freq
        valid_stfts.append(stft)
    matrix = np.abs(np.concatenate(valid_stfts, axis=0))
    if fit_one_gain:
        gains = np.repeat(
            np.nanmedian(matrix, axis=[0, 1])[None, :], matrix.shape[1], axis=0
        )
    else:
        gains = np.nanmedian(matrix, axis=0)  # n_mics x n_freqs

    # all are the same
    freq = rows.iloc[0].frequencies_matrix[0, :]

    calib_function = interp1d(
        freq[freq > 0],
        gains[:, freq > 0],
        kind=KIND,
        fill_value=FILL_VALUE,
        bounds_error=BOUNDS_ERROR,
    )
    if ax is not None:
        plot_calibration(freq, gains, calib_function, ax=ax)
    return calib_function, freq


def fit_distance_slice(
    d_slice_exp,
    distances_cm,
    azimuth_deg,
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

    def distance_slice_cost(coeffs, chosen_mics):
        if not optimize_absorption:
            coeffs = [WALL_ABSORPTION, *coeffs]

        d_slice = get_dist_slice_theory(
            frequency,
            distances_cm=distances_cm + coeffs[1],
            azimuth_deg=azimuth_deg,
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
        azimuth_deg=azimuth_deg,
        wall_absorption=coeffs[0],
        gains=coeffs[2:],
        chosen_mics=chosen_mics,
    )
    return coeffs, d_slice, distance_slice_cost(coeffs, chosen_mics)
