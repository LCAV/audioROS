#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
calibration.py: methods for gain calibration
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd


def get_calibration_function(plot=False):
    from scipy.interpolate import interp1d
    from pandas_utils import filter_by_dicts

    calib_df = pd.read_pickle("results/calibration_results.pkl")
    chosen_dict = {
        "source": "sweep_buzzer",
        "method": np.median,
    }
    calib_df = filter_by_dicts(calib_df, [chosen_dict])
    assert len(calib_df) == 1
    row = calib_df.iloc[0]

    calib_psd = row.psd
    calib_f = row.frequencies
    uniform_f = np.linspace(1000, 5000, 50)
    calib_function = interp1d(
        x=calib_f, y=calib_psd, kind="linear", fill_value="extrapolate"
    )
    uniform_psd = calib_function(uniform_f)

    if plot:
        fig, axs = plt.subplots(
            1, row.psd.shape[0], squeeze=False, sharey=True, sharex=True
        )
        fig.set_size_inches(10, 5)
        for i in range(row.psd.shape[0]):
            axs[0, i].semilogy(row.frequencies, row.psd[i], color=f"C{i}")
            axs[0, i].set_xlabel("frequency [Hz]")
            axs[0, i].set_title(f"mic{i}")

            axs[0, i].scatter(uniform_f, uniform_psd[i], color=f"C{i}")
            axs[0, i].grid("both")

        axs[0, 0].set_ylabel("amplitude")
        axs[0, i].set_ylim(1e-3, 20)

    return calib_function
