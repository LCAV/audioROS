#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
frequency_analysis.py: 
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd

PAD_FACTOR = 5  # for interpolating peaks


def interpolate_peak(spec_slice, freqs, pad_factor=PAD_FACTOR, ax=None):
    from crazyflie_description_py.parameters import FS, N_BUFFER

    assert len(spec_slice) == len(freqs), (len(spec_slice), len(freqs))

    # create full frequency response by padding with zeros
    freqs_all = np.fft.rfftfreq(N_BUFFER, 1 / FS)
    f_indices = np.argmin(np.abs(freqs[:, None] - freqs_all[None, :]), axis=1)
    max_freq_error = np.max(freqs_all[f_indices] - freqs)
    assert max_freq_error < 1.0, f"max frequency error too high: {max_freq_error}"
    spec_all = np.zeros(len(freqs_all), dtype=np.complex)
    spec_all[f_indices] = spec_slice

    if ax is not None:
        ax.plot(freqs_all, np.abs(spec_all), color="C0")
        ax.scatter(freqs, np.abs(spec_slice), color="C0")

    # go to time domain, do zero-padding, and back to frequency domain.
    time_slice = np.fft.irfft(spec_all)
    n = int(pad_factor * len(time_slice))
    spec_slice_cont = np.fft.rfft(time_slice, n=n)
    freq_cont = np.fft.rfftfreq(n=n, d=1 / FS)

    # find the maximum of this higher-resolved frequency response
    max_idx = np.argmax(np.abs(spec_slice_cont))
    if ax is not None:
        ax.plot(freq_cont, np.abs(spec_slice_cont))
    max_val = np.abs(spec_slice_cont[max_idx])
    return max_val, freq_cont[max_idx]


def fit_peak(abs_spec_slice, bin_max=None):
    """ 
    Peak parabola fitting as explained in 
    https://ccrma.stanford.edu/~jos/sasp/Quadratic_Interpolation_Spectral_Peaks.html 
    """
    assert not np.any(
        np.iscomplex(abs_spec_slice)
    ), "need to give magnitude to fit_peak"
    if bin_max is None:
        bin_max = np.argmax(abs_spec_slice)
    if bin_max == 0:
        return 0.0, 0.0

    alpha, beta, gamma = abs_spec_slice[bin_max - 1 : bin_max + 2]
    assert beta >= alpha
    assert beta >= gamma
    if alpha - 2 * beta + gamma == 0:
        p = 0
    else:
        p = 0.5 * (alpha - gamma) / (alpha - 2 * beta + gamma)
    return beta - (alpha - gamma) * p / 4, p


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
        (len(all_frequencies), stft.shape[1], stft.shape[0]), dtype=complex
    )

    for t_idx in range(stft.shape[0]):
        freqs = frequencies_matrix[t_idx, :]
        for i, f in enumerate(freqs):
            f_idx = np.argmin(np.abs(f - all_frequencies))
            spectrogram[f_idx, :, t_idx] = stft[t_idx, :, i]
    return spectrogram, all_frequencies


def _get_spectrogram_constant_bins(stft):
    return np.transpose(stft, (2, 1, 0))  # n_freqs x n_mics x n_times


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
    assert spec.ndim == 3
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


def psd_df_from_spec(
    spec, freqs, min_t=0, max_t=None, interpolation="", verbose=False, times=None,
):
    """
    Extract distance-frequency information from spectrogram.

    :param spec: spectrogram (n_freqs x n_mics x n_times)
    :param freqs: frequencies (n_freqs,)

    structure of output: 
    | mic | frequency | distance | time | magnitude

    :param return: psd_df
    """

    n_mics = spec.shape[1]
    n_times = spec.shape[2]
    upper_bound = n_mics * n_times
    psd_df = pd.DataFrame(
        columns=[
            "time",
            "counter",
            "mic",
            "frequency",
            "distance",
            "angle",
            "magnitude",
        ],
        index=range(upper_bound),
    )
    if max_t is None:
        max_t = n_times

    counter_dict = {}
    for i_t in range(min_t, max_t):
        for i_mic in range(n_mics):
            spec_slice = spec[:, i_mic, i_t]
            i_f = np.argmax(np.abs(spec_slice))
            counter_dict[i_f] = counter_dict.get(i_f, 0) + 1

            max_amp = np.abs(spec_slice[i_f])
            max_f = freqs[i_f]
            if interpolation == "lagrange":
                magnitude_estimate, frequency = interpolate_peak(spec_slice, freqs)
                if verbose and (frequency != 0):
                    print(
                        f"peak estimate {frequency}:{magnitude_estimate} instead of {max_f}:{max_amp}"
                    )
            elif interpolation == "quadratic":
                magnitude_estimate, p = fit_peak(np.abs(spec_slice))
                # TODO(FD) not exactly correct, should be using p from above.
                # to be changed if we decide to use fit_peak.
                frequency = max_f
            else:
                magnitude_estimate = max_amp
                frequency = max_f

            if times is not None:
                time = times[i_t]
            else:
                time = i_t

            update_dict = {
                "mic": i_mic,
                "time": time,
                "counter": counter_dict[i_f],
                "frequency": frequency,
                "magnitude": magnitude_estimate,
            }
            psd_df.loc[len(psd_df), list(update_dict.keys())] = list(
                update_dict.values()
            )
    psd_df.dropna(axis=1, inplace=True, how="all")
    psd_df = psd_df.apply(pd.to_numeric, axis=0, downcast="integer", errors="ignore")
    return psd_df
