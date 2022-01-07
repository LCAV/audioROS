"""
estimators.py: Combine angle or distance distributions from multiple mics to yield one final distribution.

"""
import warnings
import time

import numpy as np
from scipy.interpolate import interp1d

from utils.geometry import Context

METHOD = "sum" # method used to combine probability distributions
EPS = 1e-30  # smallest value for probability distribution

N_INTEGRAL = 10 # number of points used to approximate integral.

def get_estimate(values, probs):
    return values[np.argmax(probs)]


def get_window(points, i):
    """
    Get window around index i, splitting each interval in half. 
    At boundaries: use nearest interval's width for extrapolation.
    """
    p = points[i]
    if i > 0:
        point_min = p - (points[i] - points[i-1]) / 2
    else:
        point_min = p - (points[i+1] - points[i]) / 2

    if i < len(points) - 1:
        point_max = p + (points[i+1] - points[i]) / 2
    else:
        point_max = p + (points[i] - points[i-1]) / 2
    return point_min, point_max

def extract_probs(distribution, method):
    if method == "sum":
        probs = np.sum(distribution, axis=0)
    elif method == "product": 
        probs = np.product(distribution, axis=0)
    probs /= np.sum(probs)
    return probs

class DistanceEstimator(object):
    DISTANCES_M = np.arange(7, 101) * 1e-2
    DISTANCES_M_PRIOR = np.arange(7, 101, step=10) * 1e-2 
    AZIMUTHS_DEG = np.arange(-180, 180, step=2)
    AZIMUTHS_DEG_PRIOR = np.arange(-180, 180, step=10)

    def __init__(self):
        self.data = {}  # structure: mic: (path_differences, probabilities)
        self.context = Context.get_platform_setup()

    def add_distribution(self, path_differences_m, probabilities, mic_idx):
        if np.any(path_differences_m > 100):
            print("Warning: make sure path_differences_m is in meters!")

        current_data = self.data.get(mic_idx, ([], []))
        self.data[mic_idx] = (
            np.r_[current_data[0], path_differences_m],
            np.r_[current_data[1], probabilities],
        )

    def get_distance_distribution(
        self,
        chosen_mics=None,
        verbose=False,
        method=METHOD,
        azimuth_deg=None,
        distances_m=None,
    ):
        if azimuth_deg is None:
            azimuths_deg = self.AZIMUTHS_DEG_PRIOR
        else:
            azimuths_deg = [azimuth_deg]

        if distances_m is None:
            distances_m = self.DISTANCES_M

        fill_count = 0
        distribution = np.empty((len(azimuths_deg)*len(self.data), len(distances_m)))

        # go over all data saved.
        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            deltas_m, inverse = np.unique(deltas_m, return_inverse=True)
            delta_probs = np.bincount(inverse, delta_probs)

            deltas_interp = interp1d(
                deltas_m, delta_probs, kind="linear", fill_value="extrapolate"
            )

            # "integral" over possible angles
            for azimuth_deg in azimuths_deg:
                probabilities = np.empty(len(distances_m))
                for i in range(len(distances_m)):
                    distance_min, distance_max = get_window(distances_m, i)
                    delta_min, delta_max = self.context.get_delta(azimuth_deg, np.array([distance_min, distance_max]) * 1e2, mic_idx=mic_idx)
                    delta_min *= 1e-2 # convert to meteres
                    delta_max *= 1e-2

                    # accumulate probabilities over delta that will be mapped into the integration area in terms in distances.
                    ns = np.arange(N_INTEGRAL) / N_INTEGRAL
                    probabilities[i] = np.sum(deltas_interp((1 - ns) * delta_min + ns * delta_max))
                distribution[fill_count, :] = probabilities
                fill_count += 1
        return distances_m, extract_probs(distribution, method)

    def get_angle_distribution(
        self, distance_estimate_m, chosen_mics=None, method=METHOD, azimuths_deg=None
    ):
        if distance_estimate_m is None:
            distances_m = self.DISTANCES_M_PRIOR
        else:
            distances_m = [distance_estimate_m]

        assert (
            np.linalg.norm(self.context.source) == 0
        ), "function only works for source at origin (in relative coordinates)!"

        if azimuths_deg is None:
            azimuths_deg = self.AZIMUTHS_DEG

        fill_count = 0
        distribution = np.empty((len(distances_m)*len(self.data), len(azimuths_deg)))

        for mic_idx, (deltas_m, delta_probs) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            deltas_m, inverse = np.unique(deltas_m, return_inverse=True)
            delta_probs = np.bincount(inverse, delta_probs)

            deltas_interp = interp1d(
                deltas_m, delta_probs, kind="linear", fill_value="extrapolate"
            )
            for distance_estimate_m in distances_m:

                probabilities = np.empty(len(azimuths_deg))
                for i in range(len(azimuths_deg)):
                    # calculate integration area, considering border effects
                    azimuth_min, azimuth_max = get_window(azimuths_deg, i)

                    delta_min = self.context.get_delta(azimuth_min, distance_estimate_m * 1e2, mic_idx=mic_idx)
                    delta_max = self.context.get_delta(azimuth_min, distance_estimate_m * 1e2, mic_idx=mic_idx)
                    delta_min *= 1e-2 # convert to meteres
                    delta_max *= 1e-2

                    # accumulate probabilities over delta that will be mapped into the integration area in terms in distances.
                    ns = np.arange(N_INTEGRAL) / N_INTEGRAL
                    probabilities[i] = np.sum(deltas_interp((1 - ns) * delta_min + ns * delta_max))

                distribution[fill_count, :] = probabilities
                fill_count += 1
        return azimuths_deg, extract_probs(distribution, method)

    def get_distance_estimate(self, chosen_mics=None):
        ds, probs = self.get_distance_distribution(chosen_mics)
        return get_estimate(ds, probs)

    def get_angle_estimate(self, chosen_mics=None):
        ts, probs = self.get_angle_distribution(chosen_mics)
        return get_estimate(ts, probs)


class AngleEstimator(object):
    ANGLES_DEG = np.arange(1, 91)

    def __init__(self):
        self.data = {}  # structure: mic: (gammas, probabilities, frequency)
        self.context = Context.get_platform_setup()

    def add_distribution(self, gammas, probabilities, mic_idx, frequency):
        self.data[mic_idx] = (gammas, probabilities, frequency)

    def get_angle_distribution(
        self, chosen_mics=None, method=METHOD, mics_left_right=None
    ):
        gammas = self.ANGLES_DEG

        fill_count = 0
        distribution = np.empty((len(self.data), len(gammas)))
        for mic_idx, (gammas_deg_here, probs, frequency) in self.data.items():
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            interp1d_func = interp1d(
                gammas_deg_here, probs, kind="linear", fill_value="extrapolate"
            )
            probabilities = interp1d_func(gammas)
            distribution[fill_count, :] = probabilities
            fill_count += 1

        probs = extract_probs(distribution, method)

        argmax = np.argmax(probs)
        gamma = gammas[argmax]

        all_gammas = np.arange(180).astype(int)
        all_probs = np.zeros(180)
        if mics_left_right is not None:
            score_left = 0
            score_right = 0
            for mic_left in mics_left_right[0]:
                arg = np.argmin(abs(gamma - self.data[mic_left][0]))
                score_left += self.data[mic_left][1][arg]
            for mic_right in mics_left_right[1]:
                arg = np.argmin(abs(gamma - self.data[mic_right][0]))
                score_right += self.data[mic_right][1][arg]
            if score_right > score_left:
                # below only works if gammas are integer-valued
                all_probs[(180 - gammas).astype(int)] = probs
            else:
                # print("wall is on the left")
                pass
            return all_gammas, all_probs

        # TODO(FD): maybe recalculate the distribution using
        # only the correct mics?
        return gammas, probs
