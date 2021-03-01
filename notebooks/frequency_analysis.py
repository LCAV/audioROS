#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
frequency_analysis.py: 
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd


def plot_spectrogram(x, y, matrix, ax=None):
    if ax is None:
        fig, ax = plt.subplots()

    if matrix.ndim > 2:
        assert matrix.shape[1] in [1, 4]  # n_mics
        matrix = np.mean(matrix, axis=tuple(range(1, matrix.ndim - 1)))

    if x is None:
        x = np.arange(matrix.shape[1])
    if y is None:
        y = np.arange(matrix.shape[0])

    matrix_log10 = np.full(matrix.shape, np.nan)
    matrix_log10[matrix > 0] = np.log10(matrix[matrix > 0])
    try:
        ax.pcolorfast(x, y, matrix_log10)
    except:
        ax.pcolorfast(x, y, matrix_log10[:-1, :-1])
    return ax


def reset_lims(ax, frequencies, times):
    ax.set_xlim(times[0], times[-1])
    ax.set_ylim(frequencies[0], frequencies[-1])


def get_bin(freqs, freq):
    error = np.abs(freqs - freq)
    bin_ = np.argmin(error)
    if error[bin_] > 1:
        print(f"Warning: chosen bin differs from {freq} by {error[bin_]}")
    return bin_


def _get_spectrogram_varying_bins(frequencies_matrix, stft):
    """
    :param frequencies_matrix: frequencies matrix (n_times x n_freqs)
    :param stft: blockwise fft (n_times x n_mics x n_freqs)
    """
    assert stft.shape[1] in [1, 4]
    assert stft.shape[2] in (32, 1025, 1412)
    all_frequencies = np.unique(frequencies_matrix)

    spectrogram = np.zeros(
        (len(all_frequencies), stft.shape[1], stft.shape[0]), dtype=float
    )

    for t_idx in range(stft.shape[0]):
        freqs = frequencies_matrix[t_idx, :]
        for i, f in enumerate(freqs):
            f_idx = np.argmin(np.abs(f - all_frequencies))
            spectrogram[f_idx, :, t_idx] = np.abs(stft[t_idx, :, i])
    return spectrogram, all_frequencies


def _get_spectrogram_constant_bins(stft):
    return np.abs(np.transpose(stft, (2, 1, 0)))  # n_freqs x n_mics x n_times


def get_spectrogram(df):
    """ 
    :param df: pandas dataframe with signals_f (n_mics x n_freqs) and frequencies (n_freqs) in each row.
    :return: spectrogram (absolute value squared) n_freqs x n_mics x n_times  
    """
    frequencies_matrix = np.array([*df.frequencies.values])
    frequencies_start = df.iloc[0].frequencies
    any_different = np.any(
        np.any(frequencies_start[None, :] - frequencies_matrix, axis=0)
    )
    stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
    if not any_different:
        spectrogram = _get_spectrogram_constant_bins(stft)        
        return spectrogram, frequencies_start
    else:
        return _get_spectrogram_varying_bins(frequencies_matrix, stft)


def get_spectrogram_raw(frequencies_matrix, stft):
    """ 
    """
    frequencies_start = frequencies_matrix[0, :]
    varying_bins = np.any(
        np.any(frequencies_start[None, :] - frequencies_matrix, axis=0)
    )
    if varying_bins:
        return _get_spectrogram_varying_bins(frequencies_matrix, stft)
    else:
        frequencies = frequencies_matrix[0, :]
        return _get_spectrogram_constant_bins(stft), frequencies


def add_spectrogram(row):
    """  Add spectrogram to rows (preprocessed, so they have stft and frequencies_matrix).

    Usage: 
    df = df.assign(spectrogram=None)
    df = df.apply(add_spectrogram, axis=1)

    spectrogram is of shape: n_freqs x n_mics x n_times
    """
    # TODO(FD) remove this sanity check
    assert row.stft.shape[1] in (1, 4)
    assert row.stft.shape[2] in (32, 1025, 1412), row.stft.shape

    spectrogram, freqs = get_spectrogram_raw(row.frequencies_matrix, row.stft)
    row.spectrogram = spectrogram
    return row


def get_index_matrix(spec):
    spec_avg = np.mean(spec, axis=1)
    return np.argsort(spec_avg, axis=0)[::-1]


def apply_linear_mask(spec, frequencies, slope, offset, delta=50, ax=None, times=None):
    times_window = (frequencies - offset) / slope

    psd = np.zeros((spec.shape[1], spec.shape[0]))  # 4 x 32
    if times is None:
        times = np.arange(spec.shape[2])
    for i, t in enumerate(times_window):
        # reduce the signals to a valid window given by offset and slope
        invalid_times = (times > t + delta) | (times < t - delta)
        spec[i, :, invalid_times] = 0

    if ax is not None:
        ax.plot(times_window - delta, frequencies, color="red")
        ax.plot(times_window + delta, frequencies, color="red")
        reset_lims(ax, frequencies, times)

    mask = np.any(spec, axis=(1, 2))
    frequencies = frequencies[mask]
    spec = spec[mask, ...]
    return spec, frequencies


def apply_box_mask(
    spec,
    frequencies,
    times=None,
    min_freq=None,
    max_freq=None,
    min_time=None,
    max_time=None,
    ax=None,
):
    if times is None:
        times = np.arange(spec.shape[2])

    if min_freq is not None:
        spec[frequencies < min_freq, ...] = 0
    if max_freq is not None:
        spec[frequencies > max_freq, ...] = 0
    if min_time is not None:
        spec[..., times < min_time] = 0
    if max_time is not None:
        spec[..., times > max_time] = 0

    if ax is not None:
        [
            ax.axhline(freq, color="red")
            for freq in [min_freq, max_freq]
            if freq is not None
        ]
        [
            ax.axvline(time, color="red")
            for time in [min_time, max_time]
            if time is not None
        ]
        reset_lims(ax, frequencies, times)

    mask = np.any(spec, axis=(1, 2))
    frequencies = frequencies[mask]
    spec = spec[mask, ...]
    return spec, frequencies


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
    np.unique(frequencies_matrix.flatten())

    psd_df = pd.DataFrame(
        columns=["time", "counter", "mic", "frequency", "distance", "magnitude"]
    )
    if max_t is None:
        max_t = frequencies_matrix.shape[0]

    if ax is not None:
        ax.plot(frequencies_matrix[:, 0])
        ax.axvline(min_t, color="k")
        ax.axvline(max_t, color="k")

    for i_t in range(min_t, max_t):  # n_times x n_freqs
        # save strongest n_freq frequencies
        fs = frequencies_matrix[i_t, :n_freq]
        for i_mic in range(n_mics):
            for f_idx, f in enumerate(fs):
                psd_df.loc[len(psd_df), :] = {
                    "mic": i_mic,
                    "time": i_t,
                    "frequency": f,
                    "counter": len(psd_df.loc[psd_df.frequency == f]),
                    "magnitude": np.abs(stft[i_t, i_mic, f_idx]),
                    "distance": distance,
                }
    psd_df = psd_df.apply(pd.to_numeric, axis=0, downcast="integer")
    return psd_df


# TODO(FD) replace below with the spectrogram masking version.
def extract_linear_psd(stft, frequencies, slope, offset, delta=50, ax=None, times=None):
    eps = 1e-2
    times_window = (frequencies - offset) / slope
    # freqs_window = offset + slope * times

    if ax is not None:
        ax.plot(times_window - delta, frequencies, color="red")
        ax.plot(times_window + delta, frequencies, color="red")

    psd = np.zeros(stft.shape[1:])  # 4 x 32
    if times is None:
        times = np.arange(stft.shape[0])
    for i, t in enumerate(times_window):

        # reduce the signals to a valid window given by offset and slope
        valid_times = (times <= t + delta) & (times >= t - delta)
        signals_window = np.abs(stft[valid_times, :, i])  # n_times x n_mics

        # within the window, choose only nonzero indices for average
        signals_nonzero = signals_window[np.mean(signals_window, axis=1) > eps, :]
        # print('reduced from:', signals_window.shape[0], signals_nonzero.shape[0])
        psd[:, i] = np.mean(signals_nonzero ** 2, axis=0)
    return psd


def psd_df_from_spec(spec, freqs, index_matrix, min_t=0, max_t=None, n_freq=1):
    """
    Extract distance-frequency information from spectrogram and index_matrix.

    :param spec: spectrogram (n_freqs x n_mics x n_times)
    :param freqs: frequencies (n_freqs)
    :param index_matrix: index_matrix containing the max amplitude indices at each time. (n_freq x n_times)

    structure of output: 
    | mic | frequency | distance | time | magnitude

    :param return: psd_df
    """

    if index_matrix.ndim == 1:
        assert n_freq == 1
        index_matrix = index_matrix.reshape((1, -1))

    assert index_matrix.shape[0] >= n_freq, f"index matrix shape: {index_matrix.shape}"

    distance = 0
    n_mics = spec.shape[1]

    upper_bound = spec.shape[1] * max(spec.shape[0], spec.shape[2])
    psd_df = pd.DataFrame(
        columns=["time", "counter", "mic", "frequency", "distance", "magnitude"],
        index=range(upper_bound),
    )
    if max_t is None:
        max_t = index_matrix.shape[1]

    i_loc = 0
    counter_dict = {}
    for i_t in range(min_t, max_t):  # n_times x n_freqs
        for i_f in index_matrix[:n_freq, i_t]:
            counter_dict[i_f] = counter_dict.get(i_f, 0) + 1

            f = freqs[i_f]
            for i_mic in range(n_mics):

                psd_df.loc[i_loc, :] = {
                    "mic": i_mic,
                    "time": i_t,
                    "counter": counter_dict[i_f],
                    "frequency": f,
                    "magnitude": np.abs(spec[i_f, i_mic, i_t]),
                    "distance": distance,
                }
                i_loc += 1
    if i_loc > upper_bound:
        print("Warning: upper bound was too small")
    psd_df = psd_df.apply(pd.to_numeric, axis=0, downcast="integer")
    return psd_df


def extract_psd(psd_df, verbose=False, method="median"):
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
    for (i_mic, f, distance), df in psd_df.groupby(["mic", "frequency", "distance"]):
        f_idx = np.where(f == frequencies)[0][0]
        vals = df.magnitude.values
        if verbose:
            print(f"for frequency {f}, mic{i_mic}, found {vals}")
        if len(vals):
            if ("reject" in method) and (len(vals) > 4):
                vals = vals[2:-2]
            if "median" in method:
                psd[i_mic, f_idx] = np.median(vals)
            if "mean" in method:
                psd[i_mic, f_idx] = np.mean(vals)
            psd_std[i_mic, f_idx] = np.std(vals)

    # remove the frequencies for which we have no data
    mask = np.any(psd > 0, axis=0)
    return psd[:, mask], frequencies[mask], psd_std[:, mask]
