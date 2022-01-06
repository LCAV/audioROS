#!/usr/bin/env python
# coding: utf-8

import itertools
import math
import sys

import matplotlib.pylab as plt
import numpy as np
import pandas as pd
import progressbar

from audio_stack.beam_former import BeamFormer, normalize_rows, combine_rows
from utils.evaluate_data import read_df, get_positions, add_pose_to_df

IIR_ALPHA = 0.5
MIC_SELECTION = range(4)
NORMALIZATION_METHOD = "none"
COMBINATION_METHOD = "sum"
#COMBINATION_METHOD = "product" # combination across frequencies, and across time for dynamic

VERBOSE = True

EXP_NAME='2021_10_12_doa_stepper'; 
SOURCE='mono3000'
BIN_SELECTION=3

FREQUENCY_SELECTION = 3000 # Set to None to use highest amplitude

N_COLUMNS = None # set to None to consider full dataset

START_ANGLE = -132 # setup in lab

def get_offsets(df, method="raw"):
    if method in [-300, 300]: # 30cm
        duration = 80  # in seconds, time for stepper motor to do full movement
        end_idx = np.where(df.second > duration)[0][0]
        positions = np.zeros((len(df), 2))
        positions[:end_idx, 0] = np.linspace(
            0, -method*1e-3, end_idx
        )
        positions[end_idx:, 0] = -method*1e-3
    elif method == "raw":
        positions = np.r_[df.x.values, df.y.values]
    else:
        positions = np.full((len(df), 2), method)
    return positions 

def get_orientations(df, method='raw'):
    from utils.evaluate_data import integrate_yaw
    
    # stepper motor
    if method in [-360, 360]:
        duration = 23 # in seconds, time for stepper motor to do full rotation
        end_idx = np.where(df.second.values > duration)[0][0]
        end_angle = method 
        orientations = np.empty(len(df))
        orientations[:end_idx] = START_ANGLE + np.linspace(0, -method, end_idx)
        orientations[end_idx:] = START_ANGLE - method
    elif method == 0:
        orientations = np.full(len(df), START_ANGLE)
    elif method == 'raw': 
        # TODO: need to figure out if we need to do + or - (depends on drone's convention)
        orientations = START_ANGLE + df.yaw_deg.values.copy()
    elif method == 'rate':
        # see TODO above.
        orientations = START_ANGLE + integrate_yaw(df.timestamp.values,
                                     df.yaw_rate_deg.values)
    else:
        raise ValueError(method)
                                                       
    orientations = orientations % 360 # makes sure it is positive too
    orientations[~np.isnan(orientations)] = np.unwrap(
        orientations[~np.isnan(orientations)] / 180 * np.pi) * 180 / np.pi
    return orientations


def perform_experiment(df, combination_n, method, averaging, verbose=False):
    n_columns = N_COLUMNS or len(df)

    stft = np.array([*df['signals_f']]) 
    n_times, n_mics, n_freqs = stft.shape

    if FREQUENCY_SELECTION is None:
        max_bin = np.argmax(np.sum(stft, axis=(0, 1)))
    else:
        max_bin = np.argmin(np.abs(df.iloc[0].frequencies - FREQUENCY_SELECTION))
    freqs = df.iloc[0].frequencies[[max_bin]]
    if verbose:
        print("using frequencies", freqs)

    mic_positions = df.iloc[0].mic_positions[MIC_SELECTION, :]
    beam_former = BeamFormer(mic_positions=mic_positions)

    beam_former.init_dynamic_estimate(freqs, combination_n, COMBINATION_METHOD, NORMALIZATION_METHOD)
    beam_former.init_multi_estimate(freqs, combination_n)

    R_idx = 0; R_list = [[]] * combination_n
    R = None

    angles = BeamFormer.theta_scan * 180 / np.pi
    raw_heatmap = np.zeros((len(angles), n_columns))
    dynamic_heatmap = np.zeros((len(angles), n_columns))
    multi_heatmap = np.zeros((len(angles), n_columns))

    dynamic_spectrum = None
    multi_spectrum = None

    with progressbar.ProgressBar(max_value=n_columns, redirect_stdout=True) as p:
        for i, (__, row) in enumerate(df.iterrows()):
            signals_f = row.signals_f[:, [max_bin]] # n_mics x n_freqs
            signals_f = signals_f[MIC_SELECTION, :]

            orientation_deg = row.orientation #row.yaw_deg
            position_m = np.array([row.position_x, row.position_y])
            timestamp = row.timestamp

            R_new = beam_former.get_correlation(signals_f.T) # n_freqs x n_mics x n_mics
            if averaging == 'iir':
                if R is None:
                    R = R_new
                else:
                    R = IIR_ALPHA * R_new + (1 - IIR_ALPHA) * R
            elif averaging == 'ma':
                R_list[R_idx] = R_new
                R_idx = (R_idx + 1) % combination_n
                R = np.mean([R_elem for R_elem in R_list if len(R_elem)], axis=0)
            elif averaging == 'none':
                R = R_new
            else:
                raise ValueError(averaging)

            if method == 'mvdr':
                raw_spectrum = beam_former.get_mvdr_spectrum(R, freqs)
            elif method == 'das':
                raw_spectrum = beam_former.get_das_spectrum(R, freqs)
            else:
                raise ValueError(method)

            time_sec = (row.audio_timestamp - df.iloc[0].audio_timestamp) * 1e-3

            if not pd.isna(orientation_deg):
                #### DYNAMIC
                beam_former.add_to_dynamic_estimates(raw_spectrum, orientation_deg=orientation_deg)
                dynamic_spectrum = beam_former.get_dynamic_estimate()
                dynamic_spectrum = combine_rows(dynamic_spectrum, COMBINATION_METHOD, keepdims=True)
                dynamic_spectrum = normalize_rows(dynamic_spectrum, method="zero_to_one")

                #### MULTI

                # TODO(FD) figure out correct sign here
                beam_former.add_to_multi_estimate(signals_f.T, freqs, time_sec, orientation_deg, position=position_m)
                multi_spectrum = beam_former.get_multi_estimate(method=method)
                multi_spectrum = combine_rows(multi_spectrum, COMBINATION_METHOD, keepdims=True)
                multi_spectrum = normalize_rows(multi_spectrum, method="zero_to_one")
            else:
                print('not updating dynamic and multi:', i)

            #### RAW
            raw_spectrum = combine_rows(raw_spectrum, COMBINATION_METHOD, keepdims=True)
            raw_spectrum = normalize_rows(raw_spectrum, method="zero_to_one")

            raw_heatmap[:, i] = raw_spectrum.flatten()
            if dynamic_spectrum is not None:
                dynamic_heatmap[:, i] = dynamic_spectrum.flatten()
            if multi_spectrum is not None:
                multi_heatmap[:, i] = multi_spectrum.flatten()
            if i >= n_columns - 1:
                break
            p.update(i)

        results_dict = dict(
                raw_heatmap=raw_heatmap,
                multi_heatmap=multi_heatmap,
                dynamic_heatmap=dynamic_heatmap,
        )
        return results_dict

if __name__ == "__main__":
    params_list = [
        # experiment where we rotate by 360 degress, at fixed distance
        dict(degree=360, distance=15, motors=False, appendix="_new"),
        dict(degree=-360, distance=15, motors=True, appendix="_new4"),
        # experiment where we move laterally by 30cm, with fixed angle
        dict(degree=0, distance=-300, motors=True, appendix=""),
        dict(degree=0, distance=300, motors=False, appendix="")
    ]

    combination_n_list = [3, 5, 10]
    method_list = ["das", "mvdr"]
    averaging_list = ["none", "ma", "iir"]

    results_df = pd.DataFrame(columns=[
        'degree', 'distance', 'motors', 'appendix', 'combination_n', 'method', 'covariance_averaging', 'exp_name', 'raw_heatmap', 'dynamic_heatmap', 'multi_heatmap']
    )

    for params in params_list:
        params_read = params
        params_read.update(dict(
            props=False,
            bin_selection=BIN_SELECTION,
            source=SOURCE,
            exp_name=EXP_NAME
        ))
        df, df_pos = read_df(**params_read)

        index_start = df.iloc[0]['index']
        #df_pos = df_pos.loc[df_pos.index >= index_start]
        add_pose_to_df(df, df_pos, verbose=True)
        df.loc[:, "second"] = (df.timestamp - df.timestamp.iloc[0]) * 1e-3
        df.loc[:, "orientation"] = get_orientations(df, method=params["degree"])
        positions = get_offsets(df, method=params["distance"])
        df.loc[:, "position_x"] = positions[:, 0]
        df.loc[:, "position_y"] = positions[:, 1]
        n_columns = len(df)

        for combination_n, method, averaging in itertools.product(combination_n_list, 
                method_list, 
                averaging_list):
            print("performing experiment", combination_n, method, averaging)
            results_dict = perform_experiment(df, combination_n, method, averaging, verbose=VERBOSE)
            results_dict.update(params)
            results_dict.update(
                combination_n=combination_n,
                method=method,
                covariance_averaging=averaging,
            )
            results_df.loc[len(results_df), :] = results_dict

            fname = f"results/doa_analysis_{EXP_NAME}.pkl"
            results_df.to_pickle(fname)
            print("saved intermediate results as", fname)
        print("done")
