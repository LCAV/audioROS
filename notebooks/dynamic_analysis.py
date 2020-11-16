#!/usr/bin/env python
# coding: utf-8

import itertools
import os

import numpy as np
import pandas as pd

MAX_ALLOWED_LAG_MS = 20

def add_pose_to_df(df, df_pos, max_allowed_lag_ms=MAX_ALLOWED_LAG_MS):
    for i, row in df.iterrows():
        timestamp = row.timestamp

        # most recent position timestamp
        if not len(df_pos[df_pos.timestamp <= timestamp]):
            df.loc[i, 'yaw_deg'] = None
            print('no position before first audio:', timestamp, df_pos.timestamp.values)
            continue
        pos_idx = df_pos[df_pos.timestamp <= timestamp].index[-1]
        lag = timestamp - df_pos.loc[pos_idx].timestamp
        if lag <= MAX_ALLOWED_LAG_MS:
            df.loc[i, 'yaw_deg'] = df_pos.loc[pos_idx].yaw_deg
            df.loc[i, 'dx'] = df_pos.loc[pos_idx].dx
            df.loc[i, 'dy'] = df_pos.loc[pos_idx].dy
        else:
            df.loc[i, 'yaw_deg'] = None
            print('too high time lag (in ms):', lag)


def update_df(df, new_values, **filter_v):
    print('filter:', filter_v)
    for filt, val in filter_v.items():
        print(filt, df.loc[df[filt] == val])
    print(len(df.loc[(df[list(filter_v)] == pd.Series(filter_v)).all(axis=1)]))
    if len(df.loc[(df[list(filter_v)] == pd.Series(filter_v)).all(axis=1)]):
        df.loc[(df[list(filter_v)] == pd.Series(filter_v)).all(axis=1), :] = new_values
        print('updated existing row')
    else:
        df = df.loc[len(df), :] = new_values.update(filter_v)
        print('added new line')
        assert len(df.loc[(df[list(filter_v)] == pd.Series(filter_v)).all(axis=1)]) == 1
    return df 


if __name__ == "__main__":
    from evaluate_data import read_df, get_spec
    from audio_stack.beam_former import BeamFormer, normalize_rows, combine_rows
    import progressbar

    #exp_name = '2020_10_30_dynamic'; degree=0; start_idx = 0# correct mic positions, but only one degree
    #exp_name = '2020_10_30_dynamic_test'; degree=0; start_idx=0 #degree=90; 
    #exp_name = '2020_10_30_dynamic_move'; degree=0; start_idx=0
    #exp_name = '2020_11_03_sweep_old'; degree=90; start_idx = 20
    #exp_name = '2020_11_03_sweep'; degree=90; start_idx=20
    #exp_name = '2020_11_10_buzzer'; degree=None; start_idx=20
    exp_name = '2020_11_12_speaker360'; degree=360; start_idx=20

    fname = f'DynamicAnalaysis_{exp_name}.pkl'

    params = dict(
        degree = degree,
        props = False,
        snr = False,
        motors = None,
        source = None,
        exp_name = exp_name
    )

    if os.path.isfile(fname):
        input(f'File {fname} exists, are you sure you want to overwrite?')

    df_results = pd.DataFrame(columns=[
        'combination_n',
        'raw_heatmap',
        'dynamic_heatmap',
        'multi_heatmap',
        'spectrogram',
        'condition_numbers',
        'ranks',
        'method', 
        'covariance_averaging'
    ] + list(params.keys()))

    ##### variables
    motors_list = [True, False]
    #degree_list = [0, 45, 90] if degree is None else [degree]
    degree_list = [0, 90, 360] if degree is None else [degree]
    combination_n_list = [1, 2, 5] #, 5, 10] #range(1, 11)
    method_list = ["das"]
    covariance_averaging_list = ['none'] #['ma']

    ###### constants
    n_columns = None  # means we will use all dataset
    combination_method = "sum"
    normalization_method = "none"
    bin_selection = "fixed"; selected_hz = [4100]
    lamda = 1e-3 # for mvdr
    alpha = 0.5 # for iir

    for motors, degree in itertools.product(motors_list, degree_list):
        params['motors'] = motors
        params['degree'] = degree

        #### dataset reading and preprocessing
        df, df_pos = read_df(**params)

        df = df.iloc[start_idx:]
        index_start = df.iloc[0]['index']
        df_pos = df_pos.loc[df_pos.index >= index_start]

        add_pose_to_df(df, df_pos)

        if n_columns is None:
            n_columns = len(df)

        mic_positions = df.iloc[0].mic_positions

        for combination_n, method, covariance_averaging in itertools.product(
                combination_n_list, 
                method_list, 
                covariance_averaging_list):
            print(f'running combination_n={combination_n}, method={method}, averaging={covariance_averaging}') 

            #### method initialization
            if covariance_averaging == 'iir':
                R = None
            elif covariance_averaging == 'ma':
                R_list = [[]] * combination_n
                R_idx = 0
            beam_former = BeamFormer(mic_positions=mic_positions)
            if bin_selection == "fixed":
                selected_idx = [np.argmin(np.abs(df.iloc[0].frequencies - hz)) for hz in selected_hz]
                frequencies = df.iloc[0].frequencies[selected_idx] 
                beam_former.init_dynamic_estimate(frequencies, combination_n, combination_method, normalization_method)
                beam_former.init_multi_estimate(frequencies, combination_n)
            else:
                # TODO(FD): implement good initialization
                pass

            angles = beam_former.theta_scan
            times = np.empty(n_columns)
            raw_heatmap = np.zeros((len(angles), n_columns))
            dynamic_heatmap = np.zeros((len(angles), n_columns))
            multi_heatmap = np.zeros((len(angles), n_columns))
            condition_numbers = np.zeros((3, len(frequencies), n_columns)) # first row: raw, second row: averaged, third: multi
            ranks = np.zeros((3, len(frequencies), n_columns)) # first row: raw, second row: averaged, third: multi
            spectrogram = np.zeros((len(frequencies), mic_positions.shape[0], n_columns), dtype=np.complex)
            dynamic_spectrum = None
            multi_spectrum = None
            i_col = 0

            with progressbar.ProgressBar(max_value=n_columns, redirect_stdout=True) as p:
                for __, row in df.iterrows():
                    
                    if bin_selection == "fixed":
                        signals_f = row.signals_f[:, selected_idx] # n_mics x n_freqs
                        freqs = row.frequencies[selected_idx]
                    else:
                        # TODO(FD): implement this
                        pass

                    # TODO(FD) read this orientation estimate from stepper motor
                    orientation_deg = row.yaw_deg
                    timestamp = row.timestamp
                    
                    R_new = beam_former.get_correlation(signals_f.T) # n_freqs x n_mics x n_mics
                    if covariance_averaging == 'iir':
                        if R is None:
                            R = R_new
                        else:
                            R = alpha*R_new + (1-alpha)*R
                    elif covariance_averaging == 'ma':
                        R_list[R_idx] = R_new
                        R_idx = (R_idx + 1) % combination_n
                        R = np.mean([R_elem for R_elem in R_list if len(R_elem)], axis=0)
                    elif covariance_averaging == 'none':
                        R = R_new

                    
                    if method == 'mvdr':
                        raw_spectrum = beam_former.get_mvdr_spectrum(R, freqs, lamda=lamda)
                    elif method == 'das':
                        raw_spectrum = beam_former.get_das_spectrum(R, freqs)
                    time_sec = row.audio_timestamp / 1e6
                    
                    if not pd.isna(row.yaw_deg):
                        #### DYNAMIC
                        beam_former.add_to_dynamic_estimates(raw_spectrum, orientation_deg=orientation_deg)
                        dynamic_spectrum = beam_former.get_dynamic_estimate()
                        dynamic_spectrum = combine_rows(dynamic_spectrum, combination_method, keepdims=True)
                        dynamic_spectrum = normalize_rows(dynamic_spectrum, method="zero_to_one")

                        #### MULTI_NEW
                        beam_former.add_to_multi_estimate(signals_f.T, freqs, time_sec, orientation_deg)
                        multi_spectrum = beam_former.get_multi_estimate(method=method, lamda=lamda)
                        multi_spectrum = combine_rows(multi_spectrum, combination_method, keepdims=True)
                        multi_spectrum = normalize_rows(multi_spectrum, method="zero_to_one")
                    else:
                        print('not updating dynamic and multi:', i_col)
                            
                    #### RAW
                    raw_spectrum = combine_rows(raw_spectrum, combination_method, keepdims=True)
                    raw_spectrum = normalize_rows(raw_spectrum, method="zero_to_one")

                    raw_heatmap[:, i_col] = raw_spectrum.flatten()
                    if dynamic_spectrum is not None:
                        dynamic_heatmap[:, i_col] = dynamic_spectrum.flatten()
                    if multi_spectrum is not None:
                        multi_heatmap[:, i_col] = multi_spectrum.flatten()
                        # can only update this if we have successfully ipdated multi_spectrum
                        condition_numbers[2, :, i_col] = np.linalg.cond(beam_former.get_multi_R()) # averaged
                        ranks[2, :, i_col] = np.linalg.matrix_rank(beam_former.get_multi_R()) # averaged
                    times[i_col] = time_sec
                    spectrogram[:, :, i_col] = signals_f.T
                    condition_numbers[0, :, i_col] = np.linalg.cond(R_new) # new
                    condition_numbers[1, :, i_col] = np.linalg.cond(R) # averaged
                    ranks[0, :, i_col] = np.linalg.matrix_rank(R_new) # new
                    ranks[1, :, i_col] = np.linalg.matrix_rank(R) # averaged
                    
                    i_col += 1

                    if i_col >= n_columns:
                        break
                        
                    p.update(i_col)

            df_results.loc[len(df_results), :] = {
              'raw_heatmap': raw_heatmap,
              'dynamic_heatmap': dynamic_heatmap,
              'multi_heatmap': multi_heatmap,
              'spectrogram': spectrogram,
              'condition_numbers': condition_numbers,
              'ranks': ranks,
              'combination_n': combination_n,
              'method': method,
              'covariance_averaging': covariance_averaging,
               **params
            }
            df_results.to_pickle(fname)
            print('saved intermediate as', fname)
