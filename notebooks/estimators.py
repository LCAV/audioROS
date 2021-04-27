#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distance_estimator.py: 
"""

import numpy as np

from geometry import Context

# method to combine probability distributions
#METHOD = 'product' # TODO(FD) not working yet
METHOD = 'sum'
EPS = 1e-30 # smallest value for probability distribution

def get_estimate(values, probs):
    return values[np.argmax(probs)]

def extract_pdf(distribution, method=METHOD):
    values = np.fromiter(distribution.keys(), dtype=float) 
    sorted_idx = np.argsort(values)
    values = values[sorted_idx]

    probabilities = np.empty(len(values))
    max_count = np.max([len(p_list) for p_list in distribution.values()])
    for i, p_list in enumerate(distribution.values()):
        if method == 'product':
            #TODO(FD) fix this.
            p = np.sum(np.log10(p_list)) + (max_count - len(p_list))*np.log10(EPS)
        elif method == 'sum': 
            p = np.mean(p_list)
        probabilities[i] = p

    if method == 'product': 
        probabilities -= np.max(probabilities) # from 0 to 1
        probabilites = 10 ** probabilities

    probabilities /= np.sum(probabilities)
    return values, probabilities[sorted_idx]

class DistanceEstimator(object): 
    def __init__(self):
        self.data = {} # structure: mic: (path_differences, probabilities)
        self.context = Context.get_crazyflie_setup()

        self.resolution_m = 1e-2 # resolution of distances, in meters.
        self.resolution_deg = 2 # resolution of angles, in degrees

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print('Warning: make sure path_differences_m is in meters!')
        self.data[mic_idx] = (path_differences_m, probabilities)

    def get_distance_distribution(self, chosen_mics=None, verbose=False):
        azimuths_deg = np.arange(0, 180, step=1) 
        distribution = {}
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue

            for azimuth_deg in azimuths_deg:

                # TODO(FD) do uniform sampling along distances and interpolation instead of this. 
                ds_m = self.context.get_total_distance(deltas_m, azimuth_deg, mic_idx)

                for d_m, prob in zip(ds_m, delta_probs): 
                    d_rounded_m = round(d_m / self.resolution_m) * self.resolution_m

                    if verbose:
                        print(f'rounded {d_m}cm to {d_rounded_m}')

                    if d_rounded_m not in distribution.keys():
                        distribution[d_rounded_m] = []

                    # TODO(FD) add derivative here
                    distribution[d_rounded_m].append(prob)
        return extract_pdf(distribution, METHOD)

    def get_angle_distribution(self, distance_estimate_m, chosen_mics=None):
        assert np.linalg.norm(self.context.source) == 0, 'function only works for source at origin!'
        #azimuths_deg = np.arange(-180, 180, step=self.resolution_deg) 
        distribution = {} #{azimuth_deg: 0 for azimuth_deg in azimuths_deg}
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue
            #deltas_m = [deltas_m[np.argmax(delta_probs)]]
            #delta_probs = [1.0]
            for delta_m, prob in zip(deltas_m, delta_probs):
                thetas_deg = self.context.get_angles(delta_m=delta_m, source_distance_m=distance_estimate_m, mic_idx=mic_idx)
                if thetas_deg is None:
                    continue

                for t in thetas_deg:
                    t_rounded = round(t / self.resolution_deg) * self.resolution_deg
                    if t_rounded not in distribution.keys():
                        distribution[t_rounded] = []

                    distribution[t_rounded].append(prob)
        return extract_pdf(distribution, METHOD)

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

        azimuths_deg = np.arange(-180, 180)
        distribution = {}
        for mic_idx, (periods_m, period_probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics): 
                continue

            period_theoretical = SPEED_OF_SOUND / frequency  # m in terms of delta

            for azimuth_deg in azimuths_deg:
                periods_m_ortho = context.get_delta(azimuth_deg=azimuth_deg, distance=starting_distance + periods_m, mic_idx=mic_idx)
                sines_gamma = periods_m_ortho / period_theoretical
                for s, prob in zip(sines_gamma, period_probs):
                    s_rounded = round(s / self.resolution) * self.resolution
                    distribution[s_rounded] = distribution.get(s_rounded, 0) + prob

        #gammas = np.full(len(sines_gamma), 90)
        #gammas[sines_gamma <= 1] = np.arcsin(sines_gamma[sines_gamma <= 1]) * 180 / np.pi
        return distribution
