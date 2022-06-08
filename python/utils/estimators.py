"""
estimators.py: Combine angle or distance distributions from multiple mics to yield one final distribution.

"""
import warnings
import time

import numpy as np
from scipy.interpolate import interp1d

from .geometry import Context

EPS = 1e-30  # smallest value for probability distribution

N_INTEGRAL = 10  # number of points used to approximate integral.


def get_estimate(values, probs):
    return values[np.argmax(probs)]


def get_window(points, i, max_delta=10):
    """
    Get window around index i, splitting each interval in half.
    At boundaries: use nearest interval's width for extrapolation.

    :param max_delta: maximum (double) width of window to use for integration. important when
    we use coarse discretization grids.
    """
    if i > 0:
        delta = min((points[i] - points[i - 1]) / 2, max_delta)
    else:
        delta = min((points[i + 1] - points[i]) / 2, max_delta)
    point_min = points[i] - delta

    if i < len(points) - 1:
        delta = min((points[i + 1] - points[i]) / 2, max_delta)
    else:
        delta = min((points[i] - points[i - 1]) / 2, max_delta)
    point_max = points[i] + delta
    return point_min, point_max


def extract_probs(distribution):
    # first axis: marginalization axis (distances or angles)
    probs = np.sum(distribution, axis=0)
    # second axis: mic axis
    probs = np.product(probs, axis=0)
    sum_ = np.sum(probs)
    if sum_ > 0:
        probs /= sum_
    return probs


class DistanceEstimator(object):
    DISTANCES_CM = np.arange(7, 101)
    DISTANCES_CM_PRIOR = np.arange(7, 101, step=10)
    ANGLES_DEG = np.arange(-180, 180, step=2)
    ANGLES_DEG_PRIOR = np.arange(-180, 180, step=10)

    def __init__(self, distances_cm=DISTANCES_CM, angles_deg=ANGLES_DEG):
        self.data = {}  # structure: mic: (path_differences, probabilities)
        self.context = Context.get_platform_setup()
        self.distances_cm = distances_cm
        self.angles_deg = angles_deg

    def add_distributions(self, signals_f, frequencies, azimuth_deg, chosen_mics=None):
        from .inference import get_probability_bayes

        if chosen_mics is None:
            chosen_mics = range(signals_f.shape[1])

        for i, mic_idx in enumerate(chosen_mics):
            slice_f = np.abs(signals_f[:, i]) ** 2
            d_bayes, p_bayes, diff_cm = get_probability_bayes(
                slice_f, frequencies, azimuth_deg=azimuth_deg
            )
            self.add_distribution(diff_cm * 1e-2, p_bayes, mic_idx)

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
        angle_deg=None,
    ):
        if angle_deg is None:
            angles_deg = self.ANGLES_DEG_PRIOR
        else:
            angles_deg = [angle_deg]

        distribution = np.empty(
            (len(angles_deg), len(self.data), len(self.distances_cm))
        )

        # go over all data saved.
        for i_mic, (mic_idx, (deltas_m, delta_probs)) in enumerate(self.data.items()):
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            deltas_m, inverse = np.unique(deltas_m, return_inverse=True)
            delta_probs = np.bincount(inverse, delta_probs)

            deltas_interp = interp1d(
                deltas_m, delta_probs, kind="linear", fill_value=0.0, bounds_error=False
            )

            # marginalize over possible angles
            for i_angle, angle_deg in enumerate(angles_deg):
                probabilities = np.empty(len(self.distances_cm))
                for i in range(len(self.distances_cm)):
                    distance_min, distance_max = get_window(
                        self.distances_cm, i, max_delta=0.5
                    )
                    delta_min, delta_max = self.context.get_delta(
                        angle_deg,
                        np.array([distance_min, distance_max]),
                        mic_idx=mic_idx,
                    )
                    delta_min *= 1e-2  # convert to meteres
                    delta_max *= 1e-2

                    # accumulate probabilities over delta that will be mapped into the integration area in terms in distances.
                    ns = np.arange(N_INTEGRAL) / N_INTEGRAL
                    distribution[i_angle, i_mic, i] = np.sum(
                        deltas_interp((1 - ns) * delta_min + ns * delta_max)
                    )
        return self.distances_cm, extract_probs(distribution)

    def get_angle_distribution(self, distance_estimate_cm, chosen_mics=None):
        if distance_estimate_cm is None:
            distances_cm = self.DISTANCES_CM_PRIOR
        else:
            distances_cm = [distance_estimate_cm]

        assert (
            np.linalg.norm(self.context.source) == 0
        ), "function only works for source at origin (in relative coordinates)!"

        distribution = np.empty(
            (len(distances_cm), len(self.data), len(self.angles_deg))
        )
        for i_mic, (mic_idx, (deltas_m, delta_probs)) in enumerate(self.data.items()):
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            deltas_m, inverse = np.unique(deltas_m, return_inverse=True)
            delta_probs = np.bincount(inverse, delta_probs)

            deltas_interp = interp1d(
                deltas_m, delta_probs, kind="linear", fill_value=0.0, bounds_error=False
            )
            for i_distance, distance_estimate_cm in enumerate(distances_cm):
                for i in range(len(self.angles_deg)):
                    # calculate integration area, considering border effects
                    azimuth_min, azimuth_max = get_window(
                        self.angles_deg, i, max_delta=5
                    )

                    delta1 = self.context.get_delta(
                        azimuth_min, distance_estimate_cm, mic_idx=mic_idx
                    )
                    delta2 = self.context.get_delta(
                        azimuth_max, distance_estimate_cm, mic_idx=mic_idx
                    )
                    # minimum angle doesn't necessarily corresponds to minimum delta
                    delta_min = min(delta1, delta2) * 1e-2
                    delta_max = max(delta1, delta2) * 1e-2

                    # accumulate probabilities over delta that will be mapped into the integration area in terms in distances.
                    ns = np.arange(N_INTEGRAL) / N_INTEGRAL
                    distribution[i_distance, i_mic, i] = np.sum(
                        deltas_interp((1 - ns) * delta_min + ns * delta_max)
                    )
        return self.angles_deg, extract_probs(distribution)

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

    def get_angle_distribution(self, chosen_mics=None, mics_left_right=None):
        gammas = self.ANGLES_DEG

        distribution = np.empty((len(self.data), len(gammas)))
        for i_mic, (mic_idx, (gammas_deg_here, probs, frequency)) in enumerate(
            self.data.items()
        ):
            if (chosen_mics is not None) and (mic_idx not in chosen_mics):
                continue

            interp1d_func = interp1d(
                gammas_deg_here, probs, kind="linear", fill_value="extrapolate"
            )
            probabilities = interp1d_func(gammas)
            distribution[i_mic, :] = probabilities

        probs = np.product(distribution, axis=0)
        sum_ = np.sum(probs)
        if sum_ > 0:
            probs /= sum_

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
