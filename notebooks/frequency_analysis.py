#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
frequency_analysis.py: 
"""

import numpy as np

# TODO(FD) delete, duplicate
def get_spec(degree=0, props=True, snr=True, motors=True, source=True, exp_name=""):
    from scipy.signal import stft
    from scipy.io import wavfile

    WAV_DIRNAME = f"../experiments/{exp_name}/export"
    filename = get_fname(degree, props, snr, motors, source)
    fname = f"{WAV_DIRNAME}/{filename}.wav"

    fs, source_data = wavfile.read(fname)
    
    n_buffer = 1024
    
    f, t, source_stft = stft(source_data, fs, nperseg=n_buffer, axis=0)
    mask = (f > 200) & (f < 7000)
    source_stft = source_stft[mask, :]
    return f[mask], t, source_stft


def get_spectrogram(df):
    """ 
    :param df: pandas dataframe with signals_f (n_mics x n_freqs) and frequencies (n_freqs) in each row.
    :return: spectrogram (absolute value squared) n_freqs x n_mics x n_times  
    """
    frequencies_matrix = np.array([*df.frequencies.values])
    frequencies_start = df.iloc[0].frequencies
    any_different = np.any(np.any(frequencies_start[None, :] - frequencies_matrix, axis=0))
    if not any_different:
        stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
        spectrogram = np.abs(stft.T)**2 # n_freqs x n_mics x n_times
        return spectrogram, frequencies_start  # average over n_mics: n_freqs x n_times
    else:
        all_frequencies = np.unique(frequencies_matrix)
        spectrogram = np.zeros((len(all_frequencies), len(df)), dtype=float)
        for t_idx in range(len(df)):
            signals_f = df.iloc[t_idx].signals_f
            freqs = df.iloc[t_idx].frequencies
            for i, f in enumerate(freqs):
                f_idx =  np.argmin(np.abs(f - all_frequencies))
                spectrogram[f_idx, t_idx] = np.mean(np.abs(signals_f[:, i])**2)
        return spectrogram, all_frequencies


# TODO(FD) below is for snr=0. potentially generalize? 
def extract_linear_psd(signals_f, frequencies, slope, offset, delta=50, ax=None, times=None):
    eps = 1e-2
    times_window = (frequencies - offset) / slope
    # freqs_window = offset + slope * times
    
    if ax is not None:
        ax.plot(times_window-delta, frequencies, color='red')
        ax.plot(times_window+delta, frequencies, color='red')

    psd = np.zeros(signals_f.shape[1:]) # 4 x 32
    if times is None:
        times = np.arange(signals_f.shape[0])
    for i, t in enumerate(times_window):

        # reduce the signals to a valid window given by offset and slope
        valid_times = (times <= t + delta) & (times >= t - delta)
        signals_window = np.abs(signals_f[valid_times, : , i]) # n_times x n_mics

        # within the window, choose only nonzero indices for average
        signals_nonzero = signals_window[np.mean(signals_window, axis=1) > eps, :]
        #print('reduced from:', signals_window.shape[0], signals_nonzero.shape[0])
        psd[:, i] = np.mean(signals_nonzero**2, axis=0)
    return psd


# TODO(FD) below is for snr=1. potentially generalize this to snr=2? 
def extract_psd_dict(signals_f, frequencies_matrix, min_t=0, max_t=None, n_freq=1, ax=None):
    """
    Extract a hash table from the signals and frequencies information, one for 
    each microphone.

    structure of output: 
    [
         { #mic0
            f0: [val0, val1],
            f1: [val0, val1, val2], 
            f2: [...]
         },
         { #mic1
            f0: 
            ...
         }
         ...
    ]

    :param signals_f: fft signals n_times x n_mics x n_frequencies
    :param return: psd_dict_list
    """
    n_mics = signals_f.shape[1]
    all_frequencies = np.unique(frequencies_matrix.flatten())

    psd_dict_list = [{f:[] for f in all_frequencies} for i in range(n_mics)]

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
                psd_dict_list[i_mic][f].append(np.abs(signals_f[i_t, i_mic, f_idx])**2)
    return psd_dict_list
                

def extract_psd(psd_dict_list, verbose=False, method='median'):
    """
    Combine hash tables in given list into one big hashtable, and return the unique keys
    and statistics of its values.

    :param psd_dict_list: output of extract_psd_dict
    """
    # extract all the different frequencies from psd_dict_list.
    n_mics = len(psd_dict_list[0])
    frequencies = set().union(*(psd_dict[i].keys() for psd_dict in psd_dict_list for i in range(n_mics)))
    frequencies = np.sort(list(frequencies))
    
    psd = np.zeros((n_mics, len(frequencies)))
    psd_std = np.zeros((n_mics, len(frequencies)))
    for i_mic in range(n_mics):
        for j, f in enumerate(frequencies):
            
            # combine all values at this f and mic
            vals = []
            for psd_dict in psd_dict_list:
                if f in psd_dict[i_mic].keys():
                    vals += psd_dict[i_mic][f]
                    
            if verbose:
                print(f'for frequency {f}, mic{i_mic}, found {vals}')
            if len(vals):
                
                if ('reject' in method) and (len(vals) > 4):
                    vals = vals[2:-2]
                if 'median' in method: 
                    psd[i_mic, j] = np.median(vals) 
                if 'mean' in method: 
                    psd[i_mic, j] = np.mean(vals) 
                    
                psd_std[i_mic, j] = np.std(vals) 
                
    # remove the frequencies for which we have no data
    mask = np.any(psd > 0, axis=0)
    return psd[:, mask], frequencies[mask], psd_std[:, mask]


# TODO(FD) delete, outdated.
def get_psd(signals_f, frequencies, ax=None, fname='real'):
    """ 
    :param signals_f: tensor of signals of shape n_times x n_mics x n_frequencies
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

    return extract_linear_psd(signals_f, frequencies, slope, offset, delta=50, ax=ax)


# TODO(FD) delete, duplicate.
def add_spectrogram(row):
    """  Add snr-based or normal spectrogram to row.
    Usage: 
    df = df.apply(add_spectrogram, axis=1)

    new spectrogram column is of shape: n_times x n_mics x n_frequencies
    """
    from crazyflie_description_py.parameters import N_BUFFER, FS 
    # TODO(FD) remove this sanity check 
    assert row.signals_f.shape[1] in (1, 4)
    assert row.signals_f.shape[2] in (32, 1025)

    if row.frequencies_matrix is None:
        row.spectrogram = np.abs(row.signals_f)
    else:
        all_frequencies = np.fft.rfftfreq(N_BUFFER, 1/FS) 
        
        # spectrogram is of shape 1025 x n_times x n_mics
        spectrogram = np.zeros((row.signals_f.shape[0], row.signals_f.shape[1], len(all_frequencies)), dtype=float) 
        for t_idx in range(row.signals_f.shape[0]):
            freqs = row.frequencies_matrix[t_idx, :]
            for i, f in enumerate(freqs):
                f_idx =  np.argmin(np.abs(f - all_frequencies))
                if abs(all_frequencies[f_idx] - f) > 1: 
                    print(f'Warning: frequency {f} is far from {all_frequencies}')

                if 0: #np.any(spectrogram[f_idx, t_idx, :] > 0):
                    print('overwriting', t_idx, f_idx, f, freqs[0])
                    err = np.max(np.abs(np.abs(row.signals_f[t_idx, :, i]) - spectrogram[t_idx, :, f_idx]))
                    # TODO(FD) find out why there is a difference between the
                    # first freq. bin (forced) and the one selected by snr scheme. 
                    if err > 0.1:
                        print('big error:', err)
                spectrogram[t_idx, :, f_idx] = np.abs(row.signals_f[t_idx, :, i])
        row.spectrogram = spectrogram
    return row
