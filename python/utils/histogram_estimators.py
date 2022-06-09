#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
histogram_estimators.py: Provides histogram filter (HistogramEstimator) class.
"""

import numpy as np
from scipy.stats import norm

from .base_estimator import (
    BaseEstimator,
    get_estimate,
    get_normal_matrix,
    get_normal_vector,
    from_0_to_180,
    from_0_to_360,
)

SIGMA_D = 5.0
SIGMA_A = 10.0  # in degrees


def normal_dist(argument, std):
    return (
        1
        / (std * np.sqrt(2 * np.pi))
        * np.exp(-argument.astype(float) ** 2 / (2 * std**2))
    )


class HistogramEstimator(BaseEstimator):
    DISTANCES_CM = np.arange(100, step=10)
    ANGLES_DEG = np.arange(360, step=10)
    ESTIMATION_METHOD = "max"

    def __init__(
        self,
        n_window=2,
        platform="crazyflie",
        angles_deg=ANGLES_DEG,
        distances_cm=DISTANCES_CM,
    ):
        super().__init__(
            n_window=n_window,
            platform=platform,
        )

        self.distances_cm = distances_cm
        self.angles_deg = angles_deg
        self.initialize_states(angles_deg, distances_cm)

    def initialize_states(self, angles_deg, distances_cm):
        aa, dd = np.meshgrid(angles_deg, distances_cm)
        self.states = np.c_[aa.flatten(), dd.flatten()] # N x 2
        self.n_states = self.states.shape[0]
        self.prior = np.full(self.n_states, 1 / self.n_states)
        self.posterior = np.full(self.n_states, 1 / self.n_states)

    def add_distributions(self, *args, **kwargs):
        super().add_distributions(*args, **kwargs)
        # self.predict_slow()
        self.predict()
        self.update()
        # dist = self.get_distance_estimate()[0]
        # angle = self.get_angle_estimate()[0]
        # print("predicted and updated. Current estimates:", dist, angle)

    def predict_slow(self, verbose=False):
        if self.filled:
            self.index = np.mod(self.index, self.n_window)
            previous = np.mod(self.index - 1, self.n_window)
        elif self.index == 0:
            return self.prior
        else:
            self.index = self.index
            previous = self.index - 1
        sigma_d = sigma_a = 1.0

        u_t = self.positions[self.index] - self.positions[previous]
        u_a = self.rotations[self.index] - self.rotations[previous]

        for k, (a_k, d_k) in enumerate(self.states):
            sum_ = 0

            for i, (a_i, d_i) in enumerate(self.states):

                # worked for backwards
                # a_glob_i = a_i - self.rotations[previous]
                # mu_d = d_i - u_t.dot(get_normal_vector(a_glob_i)) # mean of current distance estimate

                # mean of current distance and angle estimates
                a_glob_i = a_i + self.rotations[previous]
                mu_d = d_i - u_t.dot(get_normal_vector(a_glob_i))
                mu_a = a_i - u_a

                norm_d = normal_dist(d_k - mu_d, SIGMA_D)
                norm_a = normal_dist(from_0_to_180(a_k - mu_a), SIGMA_A)

                sum_ += norm_d * norm_a * self.posterior[i]

                # if (d_i == 20) and (d_k == 10) and (a_i == 180) :
                #    print(f"global {a_glob_i:.0f}?=90, {a_i:.0f}?=180, {self.rotations[previous]:.0f}?=270")
                #    print(f"for current angle {a_k}: previous global {a_glob_i}?=90 {mu_d:.0f}?=10, {mu_a:.0f}?=270")
                # if (d_i == 10) and (d_k == 20) and (a_i == 270) :
                # print(f"for current angle {a_k}: previous global {a_glob_i}?=90 {mu_d:.0f}?=20, {mu_a:.0f}?=0")
            self.prior[k] = sum_
        self.prior /= np.sum(self.prior)
        return self.prior

    def predict(self, verbose=False):

        if (not self.filled) and (self.index == 0):
            return self.prior

        # rolling window with two elements:
        previous = 1 if self.index == 0 else 0

        pos_delta = self.positions[self.index] - self.positions[previous]
        rot_delta = self.rotations[self.index] - self.rotations[previous]

        # print("distance correction:", -normal_matrix(self.states[0, :]) @ u_t)
        # a_glob_i = a_i + self.rotations[previous]
        a_global = self.rotations[previous] + self.states[:, 0] 
        mu_a = from_0_to_360(self.states[:, 0] - rot_delta)
        mu_d = self.states[:, 1] - get_normal_matrix(a_global) @ pos_delta

        arg_d_mat = self.states[:, 1][:, None] - mu_d[None, :]
        norm_d = normal_dist(arg_d_mat.astype(float), SIGMA_D)

        arg_a_mat = from_0_to_180(self.states[:, 0][:, None] - mu_a[None, :])
        norm_a = normal_dist(arg_a_mat.astype(float), SIGMA_A)

        self.prior = np.sum(norm_d * norm_a * self.posterior[None, :], axis=1)
        self.prior /= np.sum(self.prior)
        return self.prior

    def update(self, verbose=False):
        probs = np.empty(self.n_states)
        for k, (a_local, d_local) in enumerate(self.states):
            prob = 1.0
            for mic, diff_p in self.difference_p[self.index].items():
                delta_local_cm = self.context.get_delta(a_local, d_local, mic_idx=mic)
                prob *= diff_p(delta_local_cm) # interpolate at delta

            if self.angle_probs[self.index] is not None:
                prob *= self.angle_probs[self.index](a_local) # inteprolate at angle

            probs[k] = self.prior[k] * prob
        self.posterior = probs / np.sum(probs)

    def get_marginal(self, distance_cm=None, angle_deg=None, posterior=True):
        if posterior:
            joint = self.posterior.reshape(
                (len(self.distances_cm), len(self.angles_deg))
            )
        else:
            joint = self.prior.reshape((len(self.distances_cm), len(self.angles_deg)))
        if (distance_cm) and (angle_deg):
            i_d = np.argmin(np.abs(self.distances_cm - distance_cm))
            a_d = np.argmin(np.abs(self.angles_deg - angle_deg))
            return joint[i_d, a_d]
        elif distance_cm:
            i_d = np.argmin(np.abs(self.distances_cm - distance_cm))
            return joint[i_d, :]
        elif angle_deg:
            a_d = np.argmin(np.abs(self.angles_deg - angle_deg))
            return joint[:, a_d]

    def get_distributions(self, simplify_angles=False, verbose=False, plot_angles=[]):
        # self.predict(verbose=verbose)
        # self.update(verbose=verbose)
        # posterior = self.posterior.reshape((len(self.angles_deg), len(self.distances_cm)))
        # prob_angle = np.sum(posterior, axis=1)
        # prob_dist = np.sum(posterior, axis=0)
        if simplify_angles:
            raise NotImplementedError(simplify_angles)
        if len(plot_angles):
            raise NotImplementedError("plot angles")

        posterior = self.posterior.reshape(
            (len(self.distances_cm), len(self.angles_deg))
        )
        prob_angle = np.sum(posterior, axis=0)
        prob_dist = np.sum(posterior, axis=1)
        return self.distances_cm, prob_dist, self.angles_deg, prob_angle

        posterior = self.posterior.reshape(
            (len(self.angles_deg), len(self.distances_cm))
        )
        prob_angle = np.sum(posterior, axis=1)
        prob_dist = np.sum(posterior, axis=0)
        return self.distances_cm, prob_dist, self.angles_deg, prob_angle

    def get_distance_estimate(self, prob=None, method=ESTIMATION_METHOD):
        if prob is None:
            __, prob, *__ = self.get_distributions()
        return get_estimate(self.distances_cm, prob, method=method)

    def get_angle_estimate(self, prob=None, method=ESTIMATION_METHOD):
        if prob is None:
            __, __, __, prob = self.get_distributions()
        return get_estimate(self.angles_deg, prob, method=method)
