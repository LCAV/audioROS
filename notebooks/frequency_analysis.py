#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
frequency_analysis.py: 
"""

import pandas as pd
import numpy as np

def get_spectrogram_varying_bins(frequencies_matrix, stft):
    """
    :param frequencies_matrix: frequencies matrix (n_times x n_freqs)
    :param stft: blockwise fft (n_times x n_mics x n_freqs)
    """
    assert stft.shape[1] in [1, 4]
    assert stft.shape[2] in (32, 1025, 1412)
    all_frequencies = np.unique(frequencies_matrix)

    spectrogram = np.zeros((len(all_frequencies), stft.shape[1], stft.shape[0]), dtype=float)

    for t_idx in range(stft.shape[0]):
        freqs = frequencies_matrix[t_idx, :]
        for i, f in enumerate(freqs):
            f_idx =  np.argmin(np.abs(f - all_frequencies))
            spectrogram[f_idx, :, t_idx] = np.abs(stft[t_idx, :, i])
    return spectrogram, all_frequencies

# below works for snr=0,1 and 2.
def get_spectrogram(df):
    """ 
    :param df: pandas dataframe with signals_f (n_mics x n_freqs) and frequencies (n_freqs) in each row.
    :return: spectrogram (absolute value squared) n_freqs x n_mics x n_times  
    """
    frequencies_matrix = np.array([*df.frequencies.values])
    frequencies_start = df.iloc[0].frequencies
    any_different = np.any(np.any(frequencies_start[None, :] - frequencies_matrix, axis=0))

    stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
    if not any_different:
        spectrogram = np.abs(np.transpose(stft, (2, 1, 0))) # n_freqs x n_mics x n_times
        return spectrogram, frequencies_start 
    else:
        return get_spectrogram_varying_bins(frequencies_matrix, stft)

def add_spectrogram(row):
    """  Add spectrogram to rows (preprocessed, so they have stft and frequencies_matrix).

    Usage: 
    df = df.apply(add_spectrogram, axis=1)

    new spectrogram column is of shape: n_freqs x n_mics x n_times
    """
    # TODO(FD) remove this sanity check 
    assert row.stft.shape[1] in (1, 4)
    assert row.stft.shape[2] in (32, 1025, 1412), row.stft.shape

    if row.frequencies_matrix is None:
        row.spectrogram = np.abs(row.stft.T)
    else:
        spectrogram, freqs = get_spectrogram_varying_bins(row.frequencies_matrix, row.stft)
        row.spectrogram = spectrogram
    return row

# TODO(FD) works for snr=0 only. potentially generalize? 
# TODO(FD) replace below with the spectrogram version? 
# first apply mask, then get the spectrogram.
def extract_linear_psd(stft, frequencies, slope, offset, delta=50, ax=None, times=None):
    eps = 1e-2
    times_window = (frequencies - offset) / slope
    # freqs_window = offset + slope * times
    
    if ax is not None:
        ax.plot(times_window-delta, frequencies, color='red')
        ax.plot(times_window+delta, frequencies, color='red')

    psd = np.zeros(stft.shape[1:]) # 4 x 32
    if times is None:
        times = np.arange(stft.shape[0])
    for i, t in enumerate(times_window):

        # reduce the signals to a valid window given by offset and slope
        valid_times = (times <= t + delta) & (times >= t - delta)
        signals_window = np.abs(stft[valid_times, : , i]) # n_times x n_mics

        # within the window, choose only nonzero indices for average
        signals_nonzero = signals_window[np.mean(signals_window, axis=1) > eps, :]
        #print('reduced from:', signals_window.shape[0], signals_nonzero.shape[0])
        psd[:, i] = np.mean(signals_nonzero**2, axis=0)
    return psd


# TODO(FD) replace below with the spectrogram version!
def extract_psd_dict(stft, frequencies_matrix, min_t=0, max_t=None, n_freq=1, ax=None):
    """
    Extract distance-frequency information from raw signals.

    structure of output: 
    | mic | frequency | distance | time | magnitude

    :param stft: fft signals n_times x n_mics x n_frequencies
    :param return: psd_df
    """
    distance = 0
    n_mics = stft.shape[1]
    all_frequencies = np.unique(frequencies_matrix.flatten())

    psd_df = pd.DataFrame(columns=['time', 'counter', 'mic', 'frequency', 'distance', 'magnitude'])
    if max_t is None:
        max_t = frequencies_matrix.shape[0]
        
    if ax is not None:
        ax.plot(frequencies_matrix[:, 0])
        ax.axvline(min_t, color='k')
        ax.axvline(max_t, color='k')

    for i_t in range(min_t, max_t): # n_times x n_freqs 
        # save strongest n_freq frequencies
        fs = frequencies_matrix[i_t, :n_freq]
        for i_mic in range(n_mics):
            for f_idx, f in enumerate(fs):
                psd_df.loc[len(psd_df), :] = {
                    'mic': i_mic,
                    'time': i_t, 
                    'frequency': f,
                    'counter': len(psd_df.loc[psd_df.frequency==f]),
                    'magnitude': np.abs(stft[i_t, i_mic, f_idx]),
                    'distance': distance
                }
    psd_df = psd_df.apply(pd.to_numeric, axis=0, downcast='integer')
    return psd_df


def psd_df_from_spec(spec, freqs, mask, min_t=0, max_t=None, n_freq=1):
    """
    Extract distance-frequency information from spectrogram and mask.

    :param spec: spectrogram (n_freqs x n_mics x n_times)
    :param freqs: frequencies (n_freqs)
    :param mask: mask containing the max amplitude indices at each time. (n_freq x n_times)

    structure of output: 
    | mic | frequency | distance | time | magnitude

    :param return: psd_df
    """

    if mask.ndim == 1:
        assert n_freq == 1
        mask = mask.reshape((1, -1))

    assert mask.shape[0] >= n_freq 

    distance = 0
    n_mics = spec.shape[1]

    psd_df = pd.DataFrame(columns=['time', 'counter', 'mic', 'frequency', 'distance', 'magnitude'])
    if max_t is None:
        max_t = mask.shape[1]
        
    for i_t in range(min_t, max_t): # n_times x n_freqs 
        for i_f in mask[:n_freq, i_t]:
            f = freqs[i_f]
            for i_mic in range(n_mics):
                psd_df.loc[len(psd_df), :] = {
                    'mic': i_mic,
                    'time': i_t, 
                    'counter': len(psd_df.loc[psd_df.frequency==f]),
                    'frequency': f,
                    'magnitude': np.abs(spec[i_f, i_mic, i_t]),
                    'distance': distance
                }
    psd_df = psd_df.apply(pd.to_numeric, axis=0, downcast='integer')
    return psd_df
                

def extract_psd(psd_df, verbose=False, method='median'):
    """
    Combine dataframe into one big hashtable, and return the unique keys
    and statistics of its values.

    :param psd_df: output of extract_psd_dict
    """
    # extract all the different frequencies from psd_dict_list.
    n_mics = len(psd_df.mic.unique())
    frequencies = np.sort(psd_df.frequency.unique())
    
    psd = np.zeros((n_mics, len(frequencies)))
    psd_std = np.zeros((n_mics, len(frequencies)))
    for (i_mic, f, distance), df in psd_df.groupby(['mic', 'frequency', 'distance']): 
        f_idx = np.where(f==frequencies)[0][0]
        vals = df.magnitude.values
        if verbose:
            print(f'for frequency {f}, mic{i_mic}, found {vals}')
        if len(vals):
            if ('reject' in method) and (len(vals) > 4):
                vals = vals[2:-2]
            if 'median' in method: 
                psd[i_mic, f_idx] = np.median(vals) 
            if 'mean' in method: 
                psd[i_mic, f_idx] = np.mean(vals) 
            psd_std[i_mic, f_idx] = np.std(vals) 
                
    # remove the frequencies for which we have no data
    mask = np.any(psd > 0, axis=0)
    return psd[:, mask], frequencies[mask], psd_std[:, mask]


# TODO(FD) delete, only used by old experiments (in WallAnalysis.ipynb)
def get_psd(stft, frequencies, ax=None, fname='real'):
    """ 
    :param stft: tensor of signals of shape n_times x n_mics x n_frequencies
    :param frequencies: frequencies vector in Hz.
    """

    # read off from plot: 
    if 'simulated' in fname:
        slope = (4000 - 1000) / (200 - 50)
        offset = -500
    elif 'real' in fname:
        # old dataset (2020_11_23_wall2)
        #slope = (4000 - 1000) / (285 - 90)
        #offset = -500
        slope = (4000 - 1000) / (250 - 50)
        offset = 200
    else:
        raise ValueError(exp_name)

    return extract_linear_psd(stft, frequencies, slope, offset, delta=50, ax=ax)
