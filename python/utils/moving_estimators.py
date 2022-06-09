#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moving_estimators.py: Provides moving window filter (MovingEstimator) class.
"""
import warnings

import numpy as np
import scipy.signal

from .base_estimator import (
    BaseEstimator,
    get_normal_vector,
    get_estimate,
    get_estimates,
    from_0_to_360,
)

# RELATIVE_MOVEMENT_STD implicityly defines the forgetting factor when
# combining multiple measurements. The relative weights of the
# latest to the oldest measurements are, for instance:
# for 1:
# [1, 0.5, 0.25, 0.125, 0.0625]
# for 0.3:
# [1... 0.7]
# for 0.1:
# [1, 0.99, 0.98, 0.97, 0.96]
# for 0:
# [1, 1, 1, 1, 1]
RELATIVE_MOVEMENT_STD = 0.3  #


def get_normal(angle_deg):
    return -get_normal_vector(angle_deg)


class MovingEstimator(BaseEstimator):
    ANGLE_WINDOW_DEG = 0  # set to zero to use only the forward direction. o
    ANGLE_RESOLUTION_DEG = 20

    DISTANCES_CM = np.arange(100, step=2)
    ANGLES_DEG = np.arange(360, step=10)

    # ESTIMATION_METHOD = "peak"
    # ESTIMATION_METHOD = "mean"
    ESTIMATION_METHOD = "max"

    def __init__(
        self,
        n_window=2,
        platform="crazyflie",
        distances_cm=DISTANCES_CM,
        angles_deg=ANGLES_DEG,
        relative_movement_std=RELATIVE_MOVEMENT_STD,
    ):
        super().__init__(
            n_window=n_window,
            platform=platform,
        )

        self.relative_movement_std = relative_movement_std
        self.distances_cm = distances_cm
        self.angles_deg = angles_deg

    def get_distributions(self, verbose=False, simplify_angles=True, plot_angles=[]):
        def clean_distribution(dist):
            if not np.any(~np.isnan(dist)):
                dist = np.ones_like(dist)
            # sum_ = np.sum(dist)
            # if sum_:
            #    dist /= sum_
            return dist

        # get angle distribution (according to when distributions align best)
        distributions, angles_deg, distances_cm = self.get_joint_distribution(
            verbose=verbose, simplify_angles=simplify_angles, plot_angles=plot_angles
        )

        # the more 'peaky', the better the distance measurements match for, therefore the more likely
        # this angle is the correct one.
        # probs_angles = np.exp(np.max(distributions, axis=1))  # softmax
        # probs_angles = np.sum(distributions, axis=1)
        probs_angles = np.max(distributions, axis=1)

        probs_angles = clean_distribution(probs_angles)
        if verbose:
            argmax = np.argmax(probs_angles)

        angle_idx = np.argwhere(probs_angles == np.nanmax(probs_angles))[0]
        if len(angle_idx) > 1:
            print(
                "Warning: ambiguous result for angle distribution! Taking average for distance."
            )
        # probs_dist = clean_distribution(probs_dist)

        if verbose and len(plot_angles):
            import matplotlib.pylab as plt

            weighted_probs_dist = np.sum(distributions * probs_angles[:, None], axis=0)
            raw_probs_dist = np.sum(distributions[angle_idx], axis=0)
            plt.plot(
                distances_cm,
                weighted_probs_dist / np.sum(weighted_probs_dist),
                label="joint weighted",
                color="k",
                ls=":",
            )
            plt.plot(
                distances_cm,
                raw_probs_dist / np.sum(raw_probs_dist),
                label="joint at argmax",
                color="k",
                ls="--",
            )
            plt.legend()

        probs_dist = np.sum(distributions[angle_idx, :], axis=0)
        # probs_dist = np.sum(distributions * probs_angles[:, None], axis=0)
        # make sure window is symemtric around angle_idx but does not pass limits
        # window = min(min(angle_idx, 3), distributions.shape[0] - angle_idx)
        # probs_dist = np.sum(distributions[int(angle_idx-window):int(angle_idx+window+1), :], axis=0)
        probs_dist = clean_distribution(probs_dist)
        return distances_cm, probs_dist, angles_deg, probs_angles

    def get_distance_estimate(self, dist, method=ESTIMATION_METHOD):
        return get_estimate(self.distances_cm, dist, method=method)

    def get_angle_estimate(self, dist, method=ESTIMATION_METHOD):
        return get_estimate(self.angles_deg, dist, method=method)

    def get_joint_distribution(
        self, verbose=False, simplify_angles=True, plot_angles=[]
    ):
        if not self.filled:
            # print("Warning: not enough measurements yet.")
            pass

        current = self.index
        distance_p = {n: {} for n in range(self.n_window)}

        if not self.filled:
            # if we are at 2, use [0, 1] (n_window=5)
            others = list(range(self.index))
        else:
            # if we are at 2, use [0, 1, 3, 4] (n_window=5)
            others = list(range(self.n_window))
            others.remove(self.index)

        # search only around the forward direction instead of all 360 degrees. This makes sense
        # because the only "dangerous" walls are in front of us.
        if simplify_angles:
            # angle_local_deg = self.get_local_forward_angle()
            # angles_deg = np.arange(
            #   self.ANGLE_WINDOW_DEG + self.ANGLE_RESOLUTION_DEG,
            #   step=self.ANGLE_RESOLUTION_DEG,
            #   dtype=float,
            # )
            # angles_deg += angle_local_deg - angles_deg[len(angles_deg) // 2]
            angles_deg = np.array([self.get_forward_angle()])
        else:
            angles_deg = self.angles_deg

        joint_distribution = np.ones((len(angles_deg), len(self.distances_cm)))
        for a, angle_local in enumerate(angles_deg):  # in local coordinates
            # wall azimuth angle in global coordinates
            angle = angle_local + self.rotations[current]
            if verbose and (angle_local in plot_angles):
                import matplotlib.pylab as plt

                fig, ax = plt.subplots()

            if verbose:
                print(f"\n==== treating local angle {angle_local} ====")

            normal_absolute = get_normal(angle)

            # convert the differences at the current index to
            # local distance distributions.
            for mic, difference_p in self.difference_p[current].items():
                deltas = [
                    self.context.get_delta(angle_local, d, mic)
                    for d in self.distances_cm
                ]
                distance_p[current][mic] = np.array(difference_p(deltas))

                # print(distance_p[current][mic])
                if verbose:
                    print(
                        f"  distance estimate for current position, mic{mic}:",
                        self.get_distance_estimate(distance_p[current][mic])[0],
                    )

            # for previous positions, a bit more work is needed:
            for previous in others:
                # wall azimuth angle in local coordinates
                angle_local_prev = angle - self.rotations[previous]

                # local distances previous position
                delta_pos = self.positions[current] - self.positions[previous]
                distances_local_prev = self.distances_cm - normal_absolute.dot(
                    delta_pos
                )

                if verbose:
                    print(f"  local angle at position {previous}:", angle_local_prev)
                    print(
                        f"  evaluating distances at {previous}: [{distances_local_prev[0]:.1f}, {distances_local_prev[-1]:.1f}]",
                    )

                for mic, difference_p2 in self.difference_p[previous].items():
                    deltas = [
                        self.context.get_delta(angle_local_prev, d, mic)
                        for d in distances_local_prev
                    ]
                    distance_p[previous][mic] = np.array(
                        [difference_p2(d) for d in deltas]
                    )
                    # print(distance_p[previous][mic])
                    if verbose:
                        print(
                            f"  distance estimate from position {previous}, mic{mic}:",
                            self.get_distance_estimate(distance_p[previous][mic])[0],
                        )

            current_time = self.times[self.index]
            lambdas = []
            for i_time, probs_mics in distance_p.items():  # all measurement times
                # account for movement uncertainty. this gives more weight to
                # most recent measurements. The higher the movement std the
                # higher the dropoff.
                if len(probs_mics):
                    time_lag = current_time - self.times[i_time]
                    lambdas.append(
                        1 / (1 + self.relative_movement_std**2) ** time_lag
                    )
                for mic, probs in probs_mics.items():  # all mics
                    if verbose and angle_local in plot_angles:
                        ax.plot(
                            self.distances_cm,
                            probs ** lambdas[-1],
                            label=f"{i_time}, {mic}",
                        )
                    joint_distribution[a] *= probs ** lambdas[-1]

            joint_distribution[a] = joint_distribution[a] ** (1 / np.sum(lambdas))
            if verbose and angle_local in plot_angles:
                # joint_distribution[a] /= np.sum(joint_distribution[a])
                ax.plot(
                    self.distances_cm,
                    joint_distribution[a] / np.sum(joint_distribution[a]),
                    label=f"joint (normalized) at {angle_local}",
                    color="k",
                )
                plt.grid(which="both")
                plt.legend()
                plt.show(block=False)
        return joint_distribution, angles_deg, self.distances_cm
