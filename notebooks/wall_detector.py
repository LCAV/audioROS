#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_detector.py: Create and exploit distance-frequency matrix.
"""

import time

import pandas as pd
import scipy.interpolate

from frequency_analysis import *

kwargs_standard = {
    'slope': 4000/30,
    'offset': 400,
    'delta': 6,
    'min_freq': 1500,
    'max_freq': 4800,
}

kwargs_datasets = {
    '2020_12_9_rotating': {
        **kwargs_standard,
        'min_time': 12,
        'max_time': 40, 
     },
    '2020_11_26_wall': {
        **kwargs_standard,
    },
}    


ANGLE = 0
DISTANCE = 0
FREQUENCY = 0
N_MICS = 4
METHOD = np.nanmedian

def normalize_df_matrix(df_matrix, freqs, method='calibration'): 
    df_matrix_normalized = np.zeros_like(df_matrix)
    if method == 'calibration': 
        from calibration import get_calibration_function
        calib_function = get_calibration_function(plot=False)
        calib_values = calib_function(list(freqs))

        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i, :, None]

    # we already pass the interpolation function (more efficient)
    elif type(method) == scipy.interpolate.interpolate.interp1d:
        calib_values = method(list(freqs))
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i, :, None]

    elif method == 'sum_to_one':
        calib_values = np.nansum(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]
        # sanity check
        np.testing.assert_allclose(np.nansum(df_matrix_normalized, axis=2), 1.0)

    elif method == 'mean':
        calib_values = np.nanmean(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif method == 'standardize':
        calib_values = np.nanmean(df_matrix, axis=2)[:, :, None]
        calib_std = np.nanstd(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = (df_matrix[i] - calib_values) / calib_std

        # sanity check
        new_std = np.nanstd(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(new_std[~np.isnan(new_std)], 1.0)
        new_mean = np.nanmean(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(new_mean[~np.isnan(new_mean)], 0.0)
    else:
        raise ValueError(method)

    return df_matrix_normalized, calib_values


def sort_and_clip(df, min_val, max_val, name='frequency'):
    df = df.sort_values(by=name, axis=0, inplace=False)
    if min_val is None:
        min_val = min(df[name])
    if max_val is None:
        max_val = max(df[name])
    return df.loc[(df[name] > min_val) & (df[name] < max_val)]


def sort_and_clip_both(df, min_freq, max_freq, min_dist, max_dist):
    df = df.sort_values(by=['frequency', 'distance'], axis=0, inplace=False)
    if min_freq is None:
        min_freq = min(df['frequency'])
    if max_freq is None:
        max_freq = max(df['frequency'])
    if min_dist is None:
        min_dist = min(df['distance'])
    if max_dist is None:
        max_dist = max(df['distance'])
    return df.loc[(df['frequency'] > min_freq) 
                & (df['frequency'] < max_freq)
                & (df['distance'] < max_dist)
                & (df['distance'] < max_dist)]


def normalized_std(values, method=METHOD):
    method_values = method(values)
    if method_values > 0:
        return np.nanstd(values) / method_values
    else:
        return 0


class WallDetector(object): 
    def __init__(self, params={}, exp_name=None, mic_type='audio_deck'):
        self.df = pd.DataFrame(columns=['time', 'counter', 'mic', 'frequency', 'distance', 'angle', 'magnitude'])
        self.params = params
        self.n_mics = 4 if (mic_type == 'audio_deck') else 1
        #self.n_spurious = 1 if (mic_type == 'audio_deck') else 10
        self.mic_indices = range(4) if (mic_type == 'audio_deck') else [1]
        if exp_name is not None:
            self.params.update(kwargs_datasets[exp_name])


    def get_linear_kwargs(self):
        return {
            key:self.params.get(key, None) for key in ['delta', 'offset', 'slope']
        }


    def get_box_kwargs(self):
        return {
            key:self.params.get(key, None) for key in ['min_freq', 'max_freq', 'min_time', 'max_time']
        }


    def fill_from_row(self, row, verbose=False, mask=True):
        distance = row.get('distance', DISTANCE)
        if distance is None:  # otherwise these rows get excluded in groupby operations
            distance = DISTANCE
        angle = row.get('angle', ANGLE)
        return self.fill_from_data(row.frequencies_matrix, row.stft, distance=distance, angle=angle, times=row.seconds, verbose=verbose, mask=mask)


    def fill_from_data(self, frequencies_matrix, stft, distance=DISTANCE, angle=ANGLE, times=None, verbose=False, mask=True):
        if verbose: 
            t1 = time.time()
        spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)
        if mask: 
            spec_masked, freqs_masked = apply_linear_mask(spec, freqs, times=times,
                                                          **self.get_linear_kwargs())
            spec_masked, freqs_masked = apply_box_mask(spec_masked, freqs_masked, times=times,
                                                       **self.get_box_kwargs())
        else:
            spec_masked = spec
            freqs_masked = freqs

        if verbose:
            print(f'after masking: found {len(freqs_masked)} bins.')
        index_matrix = get_index_matrix(spec_masked)
        self.fill_from_spec(spec_masked, freqs_masked, index_matrix, distance, angle, verbose)
        if verbose: 
            print('fill_from_data time:', time.time() - t1); t1 = time.time()
        return spec_masked, freqs_masked


    def fill_from_spec(self, spec, freqs, index_matrix, distance=DISTANCE, angle=ANGLE, verbose=False):
        df = psd_df_from_spec(spec, freqs, index_matrix) 
        assert np.all(df.magnitude.values >= 0)
        if verbose:
            print(f'filling with {len(df)} new rows')

        df.loc[:, 'distance'] = distance  
        df.loc[:, 'angle'] = angle

        df = df.apply(pd.to_numeric, axis=0, downcast='integer')
        # need ignore_index here to make sure that the final index is unique.
        self.df = pd.concat([self.df, df], ignore_index=True)


    def get_distances(self, min_dist=None, max_dist=None):
        return sort_and_clip(self.df, min_dist, max_dist, 'distance').distance.unique()


    def get_frequencies(self, min_freq=None, max_freq=None):
        return sort_and_clip(self.df, min_freq, max_freq, 'frequency').frequency.unique()


    def get_frequency_slice(self, distance=None, min_freq=None, max_freq=None, method=METHOD):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        df = sort_and_clip(self.df, min_freq, max_freq, name='frequency')

        distances = df.distance.unique()
        if distance is None:
            if len(distances) > 1:
                print(f'Warning: taking frequency slice over multiple distances: {distances}')
        elif distance not in distances:
            print(f'Warning: did not find distance {distance}cm')
            bin_ = np.argmin(np.abs(distances - distance))
            distance = distances[bin_]
            print(f'Closest match: {distance}cm')
            df = df.loc[df.distance == distance]
        else:
            df = df.loc[df.distance == distance]

        slice_f = df.pivot_table(index='mic', columns='frequency', values='magnitude', aggfunc=method).values
        freqs = df.frequency.unique()
        return slice_f, freqs


    def get_frequency_slice_with_std(self, distance=None, min_freq=None, max_freq=None, method=METHOD):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), std dev (mic x frequencies), frequencies
        """
        slice_f, freqs = self.get_frequency_slice(distance, min_freq, max_freq, method=METHOD)
        std_f, std_freqs = self.get_frequency_slice(distance, min_freq, max_freq, 
                method=lambda x: normalized_std(x, method=METHOD))
        np.testing.assert_equal(freqs, std_freqs)
        return slice_f, std_f, freqs


    def get_distance_slice(self, frequency, min_dist=None, max_dist=None, method=METHOD):
        """ 
        :return: slice along one frequency, of shape (mic x distances), distances
        """
        df = sort_and_clip(self.df, min_dist, max_dist, name='distance')

        frequencies = df.frequency.unique()
        if frequency is None:
            if len(frequencies) > 1:
                print(f'Warning: taking frequency slice over multiple frequencies: {frequencies}')
        elif frequency not in frequencies:
            print(f'Warning: did not find frequency {frequency}Hz')
            bin_ = np.argmin(np.abs(frequencies - frequency))
            frequency = frequencies[bin_]
            print(f'Closest match: {frequency}Hz')
            df = df.loc[df.frequency == frequency]
        else:
            df = df.loc[df.frequency == frequency]

        slice_d = df.pivot_table(index='mic', columns='distance', values='magnitude', aggfunc=method).values
        return slice_d, df.distance.unique()


    def get_df_matrix(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD):
        df = sort_and_clip_both(self.df, min_dist, max_dist, min_freq, max_freq)

        frequencies = np.array(df.frequency.unique(), dtype=np.float)
        distances = np.array(df.distance.unique(), dtype=np.float)

        # Attention: below only works if ferquency and distance "masks" are the same for each mic. 
        # This is currently the case. 
        df_matrix = np.zeros((self.n_mics, len(frequencies), len(distances)))
        for i_mic, df_mic in df.groupby('mic'):
            df_matrix[i_mic, :, :] = df_mic.pivot_table(
                    index='frequency', 
                    columns='distance', 
                    values='magnitude', 
                    aggfunc=method).values
        return df_matrix, distances, frequencies

        # TODO(FD) test above, otherwise return to below version.
        for (d, f, m), df in self.df.groupby(['distance', 'frequency', 'mic']):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            df_matrix[m, f_i, d_i] = method(df.magnitude.values)
        return distances, frequencies, df_matrix
   

    def remove_spurious_freqs(self, n_spurious=None, verbose=False):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        remove_rows = []

        # average number of measurements per frequency and distance. 
        # choose any column for 'magnitude', it doesn't matter.
        values = self.df.loc[self.df.mic==0].groupby(['frequency', 'distance']).agg('count')['magnitude'].values
        if n_spurious is None:
            n_spurious = np.quantile(values, 0.7)

        if verbose:
            print(f'remove frequencies with less than {n_spurious} measurements. {np.min(values), np.max(values), np.median(values)}')

        for (freq, mic, distance), df in self.df.groupby(['frequency', 'mic', 'distance']):
            if len(df) < n_spurious: 
                #if verbose:
                #   print('removing', df)
                remove_rows += list(df.index.values)
        if verbose:
            print(f'removing {len(remove_rows)} rows.')
        self.df = self.df.drop(index=remove_rows, inplace=False)
        return len(remove_rows)


    def fill_from_backup(self, exp_name, mic_type=""):
        fname = f'results/backup_{exp_name}_{mic_type}.pkl'
        self.df = pd.read_pickle(fname)
        print('read', fname)


    def backup(self, exp_name, mic_type=""):
        fname = f'results/backup_{exp_name}_{mic_type}.pkl'
        pd.to_pickle(self.df, fname)
        print('saved', fname)