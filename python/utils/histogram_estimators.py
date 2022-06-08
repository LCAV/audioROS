#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
histogram_estimators.py: Provides histogram filter (HistogramEstimator) class.
"""

import numpy as np
from scipy.stats import norm

from .base_estimator import BaseEstimator, get_estimate, get_normal_vector

SIGMA_D = 5.0
SIGMA_A = 10.0  # in degrees


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
        self.states = np.r_[[aa.flatten()], [dd.flatten()]]
        self.prior = np.ones(self.states.shape[1])
        self.posterior = np.ones(self.states.shape[1])

    def predict_slow(self, verbose=False):
        if self.filled:
            t_1 = np.mod(self.index, self.n_window)
            t_0 = np.mod(self.index - 1, self.n_window)
        else:
            t_1 = self.index
            t_0 = max(self.index - 1, 0)
        sigma_d = sigma_a = 1.0

        u_t = self.positions[t_1] - self.positions[t_0]
        u_a = self.rotations[t_1] - self.rotations[t_0]

        for k in range(self.states.shape[1]):
            sum_ = 0
            a_k, d_k = self.states[:, k]

            for i in range(self.states.shape[1]):
                a_i, d_i = self.states[:, i]
                mu_a = a_i - u_a
                mu_d = d_i - u_t.dot(get_normal_vector(a_i))

                norm_d = norm.pdf(d_k, loc=mu_d, scale=SIGMA_D)
                norm_a = norm.pdf(a_k, loc=mu_a, scale=SIGMA_A)
                sum_ += norm_d * norm_a * self.posterior[i]
            self.prior[k] = sum_
        self.prior /= np.sum(self.prior)
        return self.prior

    def add_distributions(self, *args, **kwargs):
        super().add_distributions(*args, **kwargs)
        self.predict()
        self.update()

    def predict(self, verbose=False):
        def normal_matrix(angles_vec):
            """return N x 2"""
            return np.c_[
                np.cos(angles_vec / 180 * np.pi), np.sin(angles_vec / 180 * np.pi)
            ]

        if self.filled:
            t_1 = np.mod(self.index, self.n_window)
            t_0 = np.mod(self.index - 1, self.n_window)
        elif self.index == 0:
            return self.prior
        else:
            t_1 = self.index
            t_0 = self.index - 1

        u_t = self.positions[t_1] - self.positions[t_0]
        u_a = self.rotations[t_1] - self.rotations[t_0]

        # print("distance correction:", -normal_matrix(self.states[0, :]) @ u_t)
        mu_d = self.states[1, :] - normal_matrix(self.states[0, :]) @ u_t
        mu_a = self.states[0, :] - u_a

        mu_d_mat = (self.states[1, :][:, None] - mu_d[None, :]) ** 2.0 / (
            2 * SIGMA_D**2
        )
        norm_d = (
            1 / (SIGMA_D * np.sqrt(2 * np.pi)) * np.exp(-mu_d_mat.astype(float))
        )  # without this float conversion, numpy raises an obscure error
        norm_a = (
            1
            / (SIGMA_A * np.sqrt(2 * np.pi))
            * np.exp(
                -((self.states[0, :][:, None] - mu_a[None, :]) ** 2.0)
                / (2 * SIGMA_A**2)
            )
        )

        self.prior = np.sum(norm_d * norm_a * self.posterior[None, :], axis=1)
        self.prior /= np.sum(self.prior)
        return self.prior

    def update(self, verbose=False):
        probs = np.empty(self.states.shape[1])
        for k in range(self.states.shape[1]):
            a_k, d_k = self.states[:, k]
            prob = self.prior[k]
            for mic, diff_p in self.difference_p[self.index].items():
                delta_k_cm = self.context.get_delta(a_k, d_k, mic_idx=mic)
                # note that diff_p is in centimeters.
                prob *= diff_p(delta_k_cm)
            if self.angle_probs[self.index] is not None:
                prob *= self.angle_probs[self.index](a_k)
            probs[k] = prob
        self.posterior = probs / np.sum(probs)
        return

    def get_distributions(self, simplify_angles=False, verbose=False, plot_angles=[]):
        # self.predict(verbose=verbose)
        # self.update(verbose=verbose)
        # posterior = self.posterior.reshape((len(self.angles_deg), len(self.distances_cm)))
        # prob_angle = np.sum(posterior, axis=1)
        # prob_dist = np.sum(posterior, axis=0)
        posterior = self.posterior.reshape(
            (len(self.distances_cm), len(self.angles_deg))
        )
        prob_angle = np.sum(posterior, axis=0)
        prob_dist = np.sum(posterior, axis=1)
        return self.distances_cm, prob_dist, self.angles_deg, prob_angle

    def get_distance_estimate(self, prob, method=ESTIMATION_METHOD):
        return get_estimate(self.distances_cm, prob, method=method)

    def get_angle_estimate(self, prob, method=ESTIMATION_METHOD):
        return get_estimate(self.angles_deg, prob, method=method)
