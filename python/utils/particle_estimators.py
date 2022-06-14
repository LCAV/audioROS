#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
particle_estimators.py: Provides particle filter implementation
"""
from enum import Enum

import numpy as np
from scipy.stats import norm

from utils.base_estimator import BaseEstimator, get_normal_matrix, from_0_to_360
from utils.constants import PLATFORM


# simplest, but expensive and might miss important particles:
# from filterpy.monte_carlo import multinomial_resample as resample
# better runtime and more uniform, but might miss important samples:
# from filterpy.monte_carlo import residual_resample as resample
# very uniform, doesn't miss important samples:
from filterpy.monte_carlo import stratified_resample as resample

DISTANCES_CM = np.arange(7, 80, step=1.0)
ANGLES_DEG = np.arange(360, step=1.0)

STD_DISTANCE_CM = 5.0
STD_ANGLE_DEG = 20.0

# if True, ignore pose estimates and predict particles by adding
# uniform disturbance.
PREDICT_UNIFORM = False

INIT_UNIFORM = True

# always create this percentage of random particles to make sure we don't get
# get stuck in local minima. We replace the particles of lowest weight.
RANDOM_PARTICLE_RATIO = 0.1

class State(Enum):
    NEED_UPDATE = 0
    NEED_PREDICT = 1
    NEED_RESAMPLE = 2
    READY = 3
    NEED_INIT = 4


def get_bins(centers):
    # centers: 1, 2, 3, 4
    # bin_widths: 0.5, 0.5, 0.5
    # bins = 0.5, 1.5, 2.5,
    if len(centers) == 1:
        return centers
    bin_widths = np.diff(centers)
    bins = np.r_[
        centers[:-1] - bin_widths / 2,
        centers[-1] - bin_widths[-1] / 2,
        centers[-1] + bin_widths[-1] / 2,
    ]
    assert len(bins) == len(centers) + 1
    return bins


def resample_from_index(states, weights, indices):
    states[:] = states[indices]
    weights.fill(1.0 / len(weights))


class ParticleEstimator(BaseEstimator):
    def __init__(
        self,
        n_particles,
        platform=PLATFORM,
        distances_cm=DISTANCES_CM,
        angles_deg=ANGLES_DEG,
        predict_uniform=PREDICT_UNIFORM,
    ):
        super().__init__(
            n_window=2,
            platform=platform,
        )

        # yields an initial distribution that is not perfectly distributed
        # (more localized in a sphere round center) but it is good enough.
        self.n_particles = n_particles
        self.states = np.empty((n_particles, 2))
        
        self.weights = np.ones(n_particles) / n_particles

        self.distances_cm = distances_cm
        self.angles_deg = angles_deg

        self.predict_uniform = predict_uniform

        # state machine to make sure predict, update and resample are done
        # in the correct order and not more than necessary.
        if INIT_UNIFORM:
            dd, aa = np.meshgrid(distances_cm, angles_deg)
            self.states[:, 0] = np.random.choice(
                aa.flatten(), size=n_particles, replace=True
            )
            self.states[:, 1] = np.random.choice(
                dd.flatten(), size=n_particles, replace=True
            )
            self.state = State.READY
        else:
            self.state = State.NEED_INIT

    def effective_n(self):
        return 1.0 / np.sum(self.weights ** 2)

    def add_distributions(self, *args, **kwargs):
        super().add_distributions(*args, **kwargs)
        if (self.state != State.NEED_INIT):
            self.state = State.NEED_PREDICT

    def get_distributions(self, simplify_angles=False, method="histogram"):

        if simplify_angles:
            # sample angles from current direction.
            angle_deg = self.get_forward_angle()
            self.states[:, 0] = np.random.normal(
                loc=angle_deg, scale=STD_ANGLE_DEG, size=self.n_particles
            )

        # sample from the first measurements for initialization.
        if (self.state == State.NEED_INIT):
            # use only first mic and the approximation that delta = 2*d
            print("initializing from measurements")
            mics = list(self.difference_p[self.index].keys())
            difference_p = self.difference_p[self.index][mics[0]]
            difference_hist = difference_p(self.distances_cm)
            difference_hist /= np.sum(difference_hist)
            self.states[:, 1] = np.random.choice(self.distances_cm, self.n_particles, p=difference_hist)

            # use angles from beamforming if possible, otherwise uniform.
            if self.angle_probs[self.index] is not None:
                print("using angles from beamforming")
                angle_p = self.angle_probs[self.index]  
                angle_hist = angle_p(self.angles_deg)
                angle_hist /= np.sum(angle_hist)
                self.states[:, 0] = np.random.choice(self.angles_deg, self.n_particles, p=angle_hist)
            else:
                self.states[:, 0] = np.random.choice(self.angles_deg, self.n_particles)
            self.state = State.NEED_PREDICT

        if not self.state == State.READY:
            self.predict()
            self.update(simplify_angles=simplify_angles)
            self.resample()

        if method == "histogram":
            bins = get_bins(self.distances_cm)
            if len(bins) == 1:
                probs_dist = [1.0]
            else:
                probs_dist, __ = np.histogram(
                    self.states[:, 1], bins=bins, weights=self.weights
                )

            bins = get_bins(self.angles_deg)
            if len(bins) == 1:
                probs_angles = [1.0]
            else:
                probs_angles, __ = np.histogram(
                    self.states[:, 0], bins=bins, weights=self.weights
                )
        elif method == "gaussian":
            (mean_dist, mean_angle), (var_dist, var_angle) = self.estimate()
            norm_dist = norm(loc=mean_dist, scale=np.sqrt(var_dist))
            probs_dist = norm_dist.pdf(self.distances_cm)
            probs_dist /= np.sum(probs_dist)

            norm_angles = norm(loc=mean_angle, scale=np.sqrt(var_angle))
            probs_angles = norm_angles.pdf(self.angles_deg)
            probs_angles /= np.sum(probs_angles)
        else:
            raise ValueError(method)
        return self.distances_cm, probs_dist, self.angles_deg, probs_angles

    def update(self, simplify_angles=False):
        """
        Update particle weights based on path difference measurements.
        """
        if not self.state == State.NEED_UPDATE:
            return

        for k, (a_local, d_local) in enumerate(self.states):
            # print(f"for global {d_particle:.1f}, local distance and angle: {d_local:.1f}, {a_local:.0f}")
            prob = 1.0
            for mic, diff_dist in self.difference_p[self.index].items():
                delta_local_cm = self.context.get_delta(a_local, d_local, mic_idx=mic)
                prob *= diff_dist(delta_local_cm)  # interpolate at delta

            if self.angle_probs[self.index] is not None:
                prob *= self.angle_probs[self.index](a_local)  # interpolate at angle

            self.weights[k] *= prob
        self.weights /= np.sum(self.weights)

        # take the 30% of lowest weight particles and distribute them uniformly.
        if RANDOM_PARTICLE_RATIO > 0:
            n_uniform = int(round(self.n_particles * RANDOM_PARTICLE_RATIO))
            indices = np.argsort(self.weights)
            self.states[indices[:n_uniform], 0] = np.random.choice(self.angles_deg, n_uniform)
            self.states[indices[:n_uniform], 0] = np.random.choice(self.distances_cm, n_uniform)

        # more robust if we always resample.
        if True: #self.effective_n() < self.states.shape[0] / 2:
            self.state = State.NEED_RESAMPLE
        else:
            self.state = State.READY

    def predict(self):
        """Predict new particles based on movement estimates."""
        if not self.state == State.NEED_PREDICT:
            return

        if (not self.predict_uniform) and self.filled:
            # rolling window with two elements:
            previous = 1 if self.index == 0 else 0
            pos_delta = self.positions[self.index] - self.positions[previous]
            rot_delta = self.rotations[self.index] - self.rotations[previous]

            a_global = self.rotations[previous] + self.states[:, 0]
            self.states[:, 0] = from_0_to_360(self.states[:, 0] - rot_delta)
            self.states[:, 1] = self.states[:, 1] - get_normal_matrix(a_global) @ pos_delta

        self.states[:, 0] += np.random.normal(
            scale=STD_ANGLE_DEG, size=self.n_particles
        )
        self.states[:, 1] += np.random.normal(
            scale=STD_DISTANCE_CM, size=self.n_particles
        )

        self.states[:, 1] = np.clip(
            self.states[:, 1],
            a_min=self.distances_cm[0],
            a_max=self.distances_cm[-1],
        )

        self.state = State.NEED_UPDATE

    def estimate(self):
        """Returns mean and variance of the weighted particles."""
        import scipy.stats

        if not self.state == State.READY:
            print("Warning: not ready for estimation yet.")
            return None

        mean_distance = np.average(self.states[:, 1], weights=self.weights)
        var_distance = np.average(
                (self.states[:, 1] - mean_distance) ** 2, weights=self.weights
        )

        sin_ = np.sum(
            np.multiply(np.sin(self.states[:, 0] / 180 * np.pi), self.weights)
        )
        cos_ = np.sum(
            np.multiply(np.cos(self.states[:, 0] / 180 * np.pi), self.weights)
        )
        mean_angle = 180 / np.pi * np.arctan2(sin_, cos_)
        mean_angle %= 360
        var_angle = 180 / np.pi * np.sqrt(sin_**2 + cos_**2)
        return (mean_distance, mean_angle), (var_distance, var_angle)

    def resample(self):
        if not self.state == State.NEED_RESAMPLE:
            return
        try:
            indices = resample(self.weights)
        except:
            raise
        resample_from_index(self.states, self.weights, indices)
        self.state = State.READY
