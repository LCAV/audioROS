#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_detector.py: Create and exploit distance-frequency matrix.
"""

import pandas as pd
from frequency_analysis import *

kwargs_standard = {
    'slope': 4000/30,
    'offset': 400,
    'delta': 6,
}
kwargs_datasets = {
    '2020_12_9_rotating': {
        **kwargs_standard,
        'min_freq': 1500,
        'max_freq': 4800,
        'min_time': 12,
        'max_time': 40, 
     },
    '2020_11_26_wall': 
    {
        **kwargs_standard,
        'min_freq': 1500,
        'max_freq': 4800,
    }
}    


ANGLE = 0
DISTANCE = 0
FREQUENCY = 0
N_MICS = 4
METHOD = np.nanmedian

def sort_and_clip(array, min_, max_):
    array = np.sort(array)
    if max_ is not None:
        array = array[array < max_]
    if min_ is not None:
        array = array[array > min_]
    return array

def normalized_std(values):
    method_values = METHOD(values)
    if method_values > 0:
        return np.nanstd(values) / method_values
    else:
        return 0


class WallDetector(object): 
    def __init__(self, params={}, exp_name=None, mic_type='audio_deck'):
        self.df = pd.DataFrame(columns=['time', 'counter', 'mic', 'frequency', 'distance', 'angle', 'magnitude'])
        self.params = params
        self.n_mics = 4 if (mic_type == 'audio_deck') else 1
        self.n_spurious = 1 if (mic_type == 'audio_deck') else 10
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


    def fill_from_row(self, row, verbose=False):
        distance = row.get('distance', DISTANCE)
        angle = row.get('angle', ANGLE)
        return self.fill_from_data(row.frequencies_matrix, row.stft, distance=distance, angle=angle, times=row.seconds, verbose=verbose)


    def fill_from_data(self, frequencies_matrix, stft, distance=DISTANCE, angle=ANGLE, times=None, verbose=False):
        spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)
        spec_masked, freqs_masked = apply_linear_mask(spec, freqs, times=times,
                                                      **self.get_linear_kwargs())
        spec_masked, freqs_masked = apply_box_mask(spec_masked, freqs_masked, times=times,
                                                   **self.get_box_kwargs())
        if verbose:
            print(f'after masking: found {len(freqs_masked)} bins.')
        index_matrix = get_index_matrix(spec_masked)
        self.fill_from_spec(spec_masked, freqs_masked, index_matrix, distance, angle, verbose)
        return spec_masked, freqs_masked


    def fill_from_spec(self, spec, freqs, index_matrix, distance=DISTANCE, angle=ANGLE, verbose=False):
        df = psd_df_from_spec(spec, freqs, index_matrix) 
        if verbose:
            print(f'filling with {len(df)} new rows')
        df.loc[:, 'distance'] = distance  
        df.loc[:, 'angle'] = angle
        # need ignore_index to make sure that the final index is unique.
        self.df = pd.concat([self.df, df], ignore_index=True)


    def get_distances(self, min_dist=None, max_dist=None):
        return sort_and_clip(self.df.distance.unique(), min_dist, max_dist)


    def get_frequencies(self, min_freq=None, max_freq=None):
        return sort_and_clip(self.df.frequency.unique(), min_freq, max_freq)


    def get_frequency_slice(self, distance=None, min_freq=None, max_freq=None, method=METHOD):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        distances = self.get_distances()
        if distance is None:
            if len(distances) > 1:
                print(f'Warning: taking frequency slice over multiple distances: {distances}')
            else:
                distance = distances[0]
        elif distance not in distances:
            print(f'Warning: did not find distance {distance}cm')
            bin_ = np.argmin(np.abs(distances - distance))
            distance = distances[bin_]
            print(f'Closest match: {distance}cm')

        df = self.df.loc[self.df.distance == distance]

        freqs = sort_and_clip(df.frequency.unique(), min_freq, max_freq)

        freq_slice = np.empty((self.n_mics, len(freqs)))
        for j, f in enumerate(freqs):
            for i in range(self.n_mics):
                values = df.loc[(df.frequency == f) & (df.mic == i)].magnitude.values
                freq_slice[i, j] = method(values)
        return freq_slice, freqs


    def get_distance_slice(self, frequency, min_dist=None, max_dist=None, method=METHOD):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        frequencies = self.get_frequencies()
        if frequency not in frequencies:
            print(f'Warning: did not find frequency {frequency}Hz')
            bin_ = np.argmin(np.abs(frequencies - frequency))
            frequency = frequencies[bin_]
            print(f'Closest match: {frequency}Hz')

        df = self.df.loc[self.df.frequency == frequency]

        distances = sort_and_clip(df.distance.unique(), min_dist, max_dist)

        distance_slice = np.empty((self.n_mics, len(distances)))
        for j, d in enumerate(distances):
            for i in range(self.n_mics):
                values = df.loc[(df.distance == d) & (df.mic == i)].magnitude.values
                distance_slice[i, j] = method(values)
        return distance_slice, distances


    def get_df_matrix(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD):
        distances = self.get_distances(min_dist, max_dist)
        frequencies = self.get_frequencies(min_freq, max_freq)

        df_matrix = np.zeros((self.n_mics, len(frequencies), len(distances)))

        for (d, f, m), df in self.df.groupby(['distance', 'frequency', 'mic']):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            df_matrix[m, f_i, d_i] = method(df.magnitude.values)
        return distances, frequencies, df_matrix

        for k, d in enumerate(distances):
            psd, freqs = self.get_frequency_slice(d)
            for i in range(self.n_mics):
                for j, f in enumerate(freqs):
                    f_idx = np.where(f == frequencies)[0][0]
                    df_matrix[i, f_idx, k] = psd[i, j]
        return distances, frequencies, df_matrix
    

    def remove_spurious_freqs(self, verbose=False):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        if verbose:
            print(f'remove frequencies with less than {self.n_spurious} measurements')
        remove_rows = []
        for (freq, mic, distance), df in self.df.groupby(['frequency', 'mic', 'distance']):
            if len(df) < self.n_spurious: 
                if verbose:
                    print('removing', df)
                remove_rows += list(df.index.values)
        if verbose:
            print(f'removing {len(remove_rows)} rows')
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
