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


    def fill_from_raw(self, frequencies_matrix, stft, distance=DISTANCE, angle=ANGLE):
        spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)
        spec_masked, freqs_masked = apply_linear_mask(spec, freqs, self.params['slope'], self.params['offset'], self.params['delta'])
        index_matrix = get_index_matrix(spec_masked)
        self.fill_from_spec(spec_masked, freqs_masked, index_matrix, distance, angle)
        return spec_masked, freqs_masked


    def fill_from_spec(self, spec, freqs, index_matrix, distance=DISTANCE, angle=ANGLE):
        df = psd_df_from_spec(spec, freqs, index_matrix) 
        df.loc[:, 'distance'] = distance  
        self.df = pd.concat([self.df, df])


    def get_distances(self, min_dist=None, max_dist=None):
        return sort_and_clip(self.df.distance.unique(), min_dist, max_dist)


    def get_frequencies(self, min_freq=None, max_freq=None):
        return sort_and_clip(self.df.frequency.unique(), min_freq, max_freq)


    def get_frequency_slice(self, distance, min_freq=None, max_freq=None):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        df = self.df.loc[self.df.distance == distance]
        if not len(df):
            return 

        freqs = self.get_frequencies(min_freq, max_freq)
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
        df = self.df.loc[self.df.frequency == frequency]
        if not len(df):
            return 
        distances = self.get_distances(min_dist, max_dist)

        distance_slice = np.empty((self.n_mics, len(distances)))
        for j, d in enumerate(distances):
            for i in range(self.n_mics):
                values = df.loc[(df.distance == d) & (df.mic == i)].magnitude.values
                distance_slice[i, j] = np.median(values)
        return distance_slice, distances


    def get_df_matrix(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None):
        distances = self.get_distances(min_dist, max_dist)
        frequencies = self.get_frequencies(min_freq, max_freq)

        df_matrix = np.zeros((self.n_mics, len(frequencies), len(distances)))

        for (d, f, m), df in self.df.groupby(['distance', 'frequency', 'mic']):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            median = np.nanmedian(df.magnitude.values)
            df_matrix[m, f_i, d_i] = median
        return distances, frequencies, df_matrix

        for k, d in enumerate(distances):
            psd, freqs = self.get_frequency_slice(d)
            for i in range(self.n_mics):
                for j, f in enumerate(freqs):
                    f_idx = np.where(f == frequencies)[0][0]
                    df_matrix[i, f_idx, k] = psd[i, j]
        return distances, frequencies, df_matrix
