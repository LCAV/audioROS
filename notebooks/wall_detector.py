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
        'audio_deck': {
            **kwargs_standard,
            'min_time': 9,
            'max_time': 36, 
         },
        'measurement': {
            **kwargs_standard,
            'min_time': 7,
            'max_time': 34, 
         },
    },
    '2020_11_26_wall': {
        'audio_deck': {
            **kwargs_standard,
            'min_time': 9,
            'max_time': 36, 
        },
        'measurement': {
            **kwargs_standard,
            'min_time': 6,
            'max_time': 31.5, 
         },
    },
}    


ANGLE = 0
DISTANCE = 0
METHOD = np.nanmedian

def normalize_df_matrix(df_matrix, freqs, method='calibration-offline'): 
    df_matrix_normalized = np.zeros_like(df_matrix)
    if method == 'calibration-offline': 
        from calibration import get_calibration_function
        calib_function = get_calibration_function(plot=False)
        calib_values = calib_function(list(freqs))[i, :, None]

        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    # we already pass the interpolation function (more efficient)
    elif type(method) == scipy.interpolate.interpolate.interp1d:
        calib_values = method(list(freqs))[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif method == 'calibration-online':
        calib_values = np.nanmedian(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]
        # sanity check
        np.testing.assert_allclose(np.nanmedian(df_matrix_normalized, axis=2), 1.0)

    elif method == 'zero_to_one':
        calib_values = np.nanmin(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            min_ = np.nanmin(df_matrix[i], axis=-1)
            max_ = np.nanmax(df_matrix[i], axis=-1)
            df_matrix_normalized[i] = (df_matrix[i] - min_) / (max_ - min_)

        # sanity check
        np.testing.assert_allclose(np.nanmax(df_matrix_normalized, axis=2), 1.0)
        np.testing.assert_allclose(np.nanmin(df_matrix_normalized, axis=2), 0.0)

    elif method == 'zero_median':
        calib_values = np.nanmedian(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] - calib_values[i]

    elif method == 'zero_mean':
        calib_values = np.nanmean(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] - calib_values[i]

    elif method == 'standardize':
        calib_median = np.nanmedian(df_matrix, axis=2)[:, :, None]
        calib_std = np.nanstd(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = (df_matrix[i] - calib_median[i]) / calib_std[i]
        calib_values = calib_std

        # sanity check
        new_std = np.nanstd(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(new_std[~np.isnan(new_std)], 1.0)
        new_mean = np.nanmedian(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(new_mean[~np.isnan(new_mean)], 0.0, rtol=1, atol=1e-10)

    elif method == 'normalize':
        calib_values = np.linalg.norm(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]
    else:
        raise ValueError(method)

    return df_matrix_normalized, calib_values

def clip(df, min_val, max_val, name='frequency'):
    if min_val is None:
        min_val = min(df[name])
    if max_val is None:
        max_val = max(df[name])
    return df.loc[(df[name] >= min_val) & (df[name] <= max_val)]

def clip_both(df, min_freq, max_freq, min_dist, max_dist):
    df = clip(df, min_freq, max_freq, name='frequency')
    return clip(df, min_dist, max_dist, name='distance')

def sort_and_clip(df, min_val, max_val, name='frequency'):
    df = df.sort_values(by=name, axis=0, inplace=False)
    if min_val is None:
        min_val = min(df[name])
    if max_val is None:
        max_val = max(df[name])
    return df.loc[(df[name] >= min_val) & (df[name] <= max_val)]


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
            self.params.update(kwargs_datasets[exp_name][mic_type])


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
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)

        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        # Attention: below only works if ferquency and distance "masks" are the same for each mic. 
        # This is currently the case. 
        # Otherwise, get_df_matrix_old can be used. 
        df_matrix = np.full((self.n_mics, len(frequencies), len(distances)), np.nan)
        for i_mic, df_mic in df.groupby('mic', sort=True):
            pt = df_mic.pivot_table(
                    index='frequency', 
                    columns='distance', 
                    values='magnitude', 
                    aggfunc=method)
            frequencies_pt = pt.index.values

            distances_pt =  pt.columns.values
            np.testing.assert_allclose(distances_pt, distances)

            # find frequencies mask
            if (len(frequencies_pt)  != len(frequencies)) or not np.allclose(frequencies, frequencies_pt):
                print(f'Warning: mismatch between freqs for mic{i_mic}')
                f_indices = np.where((frequencies[None, :] == frequencies_pt[:, None]))[0]
                np.testing.assert_allclose(frequencies[f_indices], frequencies_pt)
                df_matrix[i_mic, f_indices, :] = pt.values
            else:
                df_matrix[i_mic, :, :] = pt.values

        return df_matrix, distances, frequencies


    def get_df_matrix_old(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD):
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)

        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        df_matrix = np.full((self.n_mics, len(frequencies), len(distances)), np.nan)
        for (d, f, m), df in self.df.groupby(['distance', 'frequency', 'mic']):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            df_matrix[m, f_i, d_i] = method(df.magnitude.values)
        return df_matrix, distances, frequencies
   

    def remove_spurious_freqs(self, n_spurious=None, verbose=False, dryrun=False):
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
        if not dryrun: 
            self.df = self.df.drop(index=remove_rows, inplace=False)
        return len(remove_rows)

    def remove_bad_freqs(self, mag_thresh=1e-10, std_thresh=100, verbose=False, dryrun=False):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        remove_rows = []

        if verbose:
            print(f'remove frequencies with less than {mag_thresh} average magnitude and more than {std_thresh} std.')

        for freq, df in self.df.groupby('frequency'):
            vals = df.magnitude.values
            if verbose:
                print(f'{freq}Hz: median {np.nanmedian(vals):.2e}, std {normalized_std(vals):.2e}')
            if np.nanmedian(vals) < mag_thresh: 
                remove_rows += list(df.index.values)
            elif normalized_std(vals) > std_thresh:
                remove_rows += list(df.index.values)
        if verbose:
            print(f'removing {len(remove_rows)} rows.')
        if not dryrun: 
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
