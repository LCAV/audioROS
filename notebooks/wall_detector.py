#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_detector.py: Create and exploit distance-frequency matrix.
"""

import pandas as pd
from frequency_analysis import *


ANGLE = 0
DISTANCE = 0
FREQUENCY = 0
N_MICS = 4

def sort_and_clip(array, min_, max_):
    array = np.sort(array)
    if max_ is not None:
        array = array[array < max_]
    if min_ is not None:
        array = array[array > min_]
    return array


class WallDetector(object): 
    def __init__(self, params={}, n_mics=N_MICS):
        self.df = pd.DataFrame(columns=['time', 'counter', 'mic', 'frequency', 'distance', 'angle', 'magnitude'])
        self.params = params
        self.n_mics = n_mics


    def fill_from_raw(self, frequencies_matrix, stft, distance=DISTANCE, angle=ANGLE, times=None, verbose=False):
        spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)
        spec_masked, freqs_masked = apply_linear_mask(spec, freqs, self.params['slope'], self.params['offset'], self.params['delta'], times=times)
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
        # need ignore_index to make sure that the final index is unique.
        self.df = pd.concat([self.df, df], ignore_index=True)


    def get_distances(self, min_dist=None, max_dist=None):
        return sort_and_clip(self.df.distance.unique(), min_dist, max_dist)


    def get_frequencies(self, min_freq=None, max_freq=None):
        return sort_and_clip(self.df.frequency.unique(), min_freq, max_freq)


    def get_frequency_slice(self, distance, min_freq=None, max_freq=None):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        distances = self.get_distances()
        if distance not in distances:
            print(f'Warning: did not find {distance}')
            bin_ = np.argmin(np.abs(distances - distance))
            distance = distances[bin_]
            print(f'Closest match: {distance}')

        df = self.df.loc[self.df.distance == distance]

        freqs = sort_and_clip(df.frequency.unique(), min_freq, max_freq)

        freq_slice = np.empty((self.n_mics, len(freqs)))
        for j, f in enumerate(freqs):
            for i in range(self.n_mics):
                values = df.loc[(df.frequency == f) & (df.mic == i)].magnitude.values
                freq_slice[i, j] = np.median(values)
        return freq_slice, freqs


    def get_distance_slice(self, frequency, min_dist=None, max_dist=None):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        frequencies = self.get_frequencies()
        if frequency not in frequencies:
            print(f'Warning: did not find {frequency}')
            bin_ = np.argmin(np.abs(frequencies - frequency))
            frequency = frequencies[bin_]
            print(f'Closest match: {frequency}')

        df = self.df.loc[self.df.frequency == frequency]

        distances = sort_and_clip(df.distance.unique(), min_dist, max_dist)

        distance_slice = np.empty((self.n_mics, len(distances)))
        for j, d in enumerate(distances):
            for i in range(self.n_mics):
                values = df.loc[(df.distance == d) & (df.mic == i)].magnitude.values
                distance_slice[i, j] = np.median(values)
        return distance_slice, distances


    def get_df_matrix(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=np.nanmedian):
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
    

    def fill_from_backup(self, exp_name, appendix=""):
        fname = f'results/backup_{exp_name}{appendix}.pkl'
        self.df = pd.read_pickle(fname)
        print('read', fname)


    def backup(self, exp_name, appendix=""):
        fname = f'results/backup_{exp_name}{appendix}.pkl'
        pd.to_pickle(self.df, fname)
        print('saved', fname)


    def remove_spurious_freqs(self, n_min=10):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        remove_rows = []
        for (freq, mic, distance), df in self.df.groupby(['frequency', 'mic', 'distance']):
            if len(df) < n_min: 
                remove_rows += list(df.index.values)
        print(f'removing {len(remove_rows)} rows')
        self.df = self.df.drop(index=remove_rows, inplace=False)
        return len(remove_rows)
