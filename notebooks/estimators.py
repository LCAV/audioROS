#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distance_estimator.py: 
"""

import numpy as np

from geometry import Context

METHOD = 'sum' #'product'
EPS = 1e-20  # smallest value for probability distribution

def get_estimate(values, probs):
    return values[np.argmax(probs)]

def extract_pdf(distribution):
    values = np.fromiter(distribution.keys(), dtype=float) 
    sorted_idx = np.argsort(values)
    probabilities = np.fromiter(distribution.values(), dtype=float)
    if METHOD == 'product':
        probabilities -= np.max(probabilities)
        probabilities = 10 ** probabilities
    probabilities /= np.sum(probabilities)
    return values[sorted_idx], probabilities[sorted_idx]

class DistanceEstimator(object): 
    def __init__(self):
        self.data = {} # structure: mic: (path_differences, probabilities)
        self.context = Context.get_crazyflie_setup()

        self.resolution_m = 1e-2 # resolution of distances, in meters.
        self.resolution_deg = 1 # resolution of angles, in degrees

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print('Warning: make sure path_differences_m is in meters!')
        self.data[mic_idx] = (path_differences_m, probabilities)

    def get_distance_distribution(self, chosen_mics=None):
        yaws_deg = np.arange(-180, 180, step=self.resolution_deg) 
        distribution = {}
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue

            for azimuth_deg in yaws_deg:
                ds = self.context.get_total_distance(deltas_m, azimuth_deg, mic_idx)

                for d, prob in zip(ds, delta_probs): 
                    d_rounded = round(d / self.resolution_m) * self.resolution_m

                    # add the probability to this distance 
                    if METHOD == 'product':
                        distribution[d_rounded] = distribution.get(d_rounded, np.log10(EPS)) + np.log10(prob)
                    elif METHOD == 'sum':
                        distribution[d_rounded] = distribution.get(d_rounded, EPS) + prob
                    else:
                        raise ValueError(METHOD)
        return extract_pdf(distribution)

    def get_angle_distribution(self, distance_estimate_m, chosen_mics=None):
        assert np.linalg.norm(self.context.source) == 0, 'function only works for source at origin!'
        #yaws_deg = np.arange(-180, 180, step=self.resolution_deg) 
        distribution = {} #{azimuth_deg: 0 for azimuth_deg in yaws_deg}
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue
            # for each angle, get the theoretical delta and the two angle possibilities.
            #deltas_m = [deltas_m[np.argmax(delta_probs)]]
            #delta_probs = [1.0]
            for delta_m, prob in zip(deltas_m, delta_probs):
                thetas_deg = self.context.get_angles(delta=delta_m, source_distance=distance_estimate_m, mic_idx=mic_idx)
                if thetas_deg is None:
                    continue

                for t in thetas_deg:
                    t_rounded = round(t / self.resolution_deg) * self.resolution_deg
                    if METHOD == 'product':
                        distribution[t_rounded] = distribution.get(t_rounded, np.log10(EPS)) + np.log10(prob)
                    elif METHOD == 'sum':
                        distribution[t_rounded] = distribution.get(t_rounded, EPS) + prob
                    else:
                        raise ValueError(METHOD)
        return extract_pdf(distribution)

    def get_distance_estimate(self, chosen_mics=None):
        ds, probs = self.get_distance_distribution(chosen_mics)
        return get_estimate(ds, probs)

    def get_angle_estimate(self, chosen_mics=None):
        ts, probs = self.get_angle_distribution(chosen_mics)
        return get_estimate(ts, probs)


# TODO(FD) below is not tested yet
class AngleEstimator(object): 
    def __init__(self):
        self.data = {} # structure: mic: (path_differences, probabilities)
        self.context = Context.get_crazyflie_setup()

        # resolution in sin(gamma)
        self.resolution = 1e-2 

    def add_distribution(self, periods_m, probabilities, mic_idx, frequency):
        self.data[mic_idx] = (periods_m, probabilities, frequency)

    def get_angle_distribution(self, chosen_mics=None):
        from constants import SPEED_OF_SOUND

        yaws_deg = np.arange(-180, 180)
        distribution = {}
        for mic_idx, (periods_m, period_probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue

            period_theoretical = SPEED_OF_SOUND / frequency  # m in terms of delta

            for azimuth_deg in yaws_deg:
                periods_m_ortho = context.get_delta(azimuth_deg=azimuth_deg, distance=starting_distance + periods_m, mic_idx=mic_idx)
                sines_gamma = periods_m_ortho / period_theoretical
                for s, prob in zip(sines_gamma, period_probs):
                    s_rounded = round(s / self.resolution) * self.resolution
                    distribution[s_rounded] = distribution.get(s_rounded, 0) + prob

        #gammas = np.full(len(sines_gamma), 90)
        #gammas[sines_gamma <= 1] = np.arcsin(sines_gamma[sines_gamma <= 1]) * 180 / np.pi
        return distribution
