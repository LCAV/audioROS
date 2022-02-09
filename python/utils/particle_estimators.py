#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
particle_estimators.py: 
"""

import numpy as np
from scipy.stats import norm

from utils.moving_estimators import BaseEstimator, get_normal


# simplest, but expensive and might miss important particles
# from filterpy.monte_carlo import multinomial_resample as resample
# better runtime and more uniform, but might miss important samples
# from filterpy.monte_carlo import residual_resample as resample
# very uniform, doesn't miss important samples.
from filterpy.monte_carlo import stratified_resample as resample

STD_DISTANCE_CM = 5.0
STD_ANGLE_DEG = 5.0


def get_bins(centers):
    # centers: 1, 2, 3, 4
    # bin_widths: 0.5, 0.5, 0.5
    # bins = 0.5, 1.5, 2.5,
    bin_widths = np.diff(centers)
    bins = np.r_[
        centers[:-1] - bin_widths / 2,
        centers[-1] - bin_widths[-1] / 2,
        centers[-1] + bin_widths[-1] / 2,
    ]
    assert len(bins) == len(centers) + 1
    return bins


def resample_from_index(particles, weights, indices):
    particles[:] = particles[indices]
    weights.resize(len(particles))
    weights.fill(1.0 / len(weights))


class ParticleEstimator(BaseEstimator):
    DISTANCE_RANGE_CM = [0, 100]
    # ANGLES_DEG = np.arange(360, step=1.0)
    # ANGLES_DEG = np.arange(360, step=90)
    ANGLES_DEG = np.arange(360, step=90)

    def __init__(self, n_particles, platform="crazyflie", global_=True):
        super().__init__(
            n_window=1, platform=platform,
        )

        self.n_particles = n_particles
        self.particles = np.empty((n_particles, 2))
        self.particles[:, 0] = np.random.uniform(
            *self.DISTANCE_RANGE_CM, size=n_particles
        )
        self.particles[:, 1] = np.random.choice(self.ANGLES_DEG, size=n_particles)
        self.weights = np.ones(n_particles) / n_particles

        # express particles in global reference frame.
        # if false, will express particles in local reference frame.
        self.global_ = global_

    def effective_n(self):
        return 1.0 / np.sum(np.square(self.weights))

    def get_distances_cm(self):
        return np.arange(*self.DISTANCE_RANGE_CM, step=1.0)

    def get_distributions(self, simplify_angles=False, method="histogram"):
        if simplify_angles:
            # sample angles from current direction.
            angle_deg = self.get_forward_angle()
            self.particles[:, 1] = np.random.normal(
                loc=angle_deg, scale=STD_ANGLE_DEG, size=self.n_particles
            )

        self.predict()
        self.update(simplify_angles=simplify_angles)
        self.resample()

        distances_cm = self.get_distances_cm()
        angles_deg = self.ANGLES_DEG
        if method == "histogram":
            bins = get_bins(distances_cm)
            probs_dist, __ = np.histogram(
                self.particles[:, 0], bins=bins, weights=self.weights
            )

            bins = get_bins(angles_deg)
            probs_angles, __ = np.histogram(
                self.particles[:, 1], bins=bins, weights=self.weights
            )
        elif method == "gaussian":
            (mean_dist, mean_angle), (var_dist, var_angle) = self.estimate()
            norm_dist = norm(loc=mean_dist, scale=np.sqrt(var_dist))
            probs_dist = norm_dist.pdf(distances_cm)
            probs_dist /= np.sum(probs_dist)

            norm_angles = norm(loc=mean_angle, scale=np.sqrt(var_angle))
            probs_angles = norm_angles.pdf(angles_deg)
            probs_angles /= np.sum(probs_angles)
        else:
            raise ValueError(method)
        return distances_cm, probs_dist, angles_deg, probs_angles

    def update(self, simplify_angles=False):
        for i, (d_particle, a_particle) in enumerate(self.particles):
            if self.global_:
                normal_global = get_normal(a_particle)
                a_local = a_particle - self.rotations[0]
                d_local = d_particle + normal_global.dot(self.positions[0])

            else:
                a_local = a_particle
                d_local = d_particle
            # print(f"for global {d_particle:.1f}, local distance and angle: {d_local:.1f}, {a_local:.0f}")

            weight = 1.0
            for mic, diff_dist in self.difference_p[0].items():
                delta = self.context.get_delta(a_local, d_local, mic)
                weight *= diff_dist(delta)
            self.weights[i] *= weight
        self.weights /= np.sum(self.weights)

    def predict(self):
        self.particles[:, 0] += np.random.normal(
            scale=STD_DISTANCE_CM, size=self.n_particles
        )
        self.particles[:, 0] = np.clip(
            self.particles[:, 0],
            a_min=self.DISTANCE_RANGE_CM[0],
            a_max=self.DISTANCE_RANGE_CM[1],
        )
        self.particles[:, 1] += np.random.normal(
            scale=STD_ANGLE_DEG, size=self.n_particles
        )
        self.particles[:, 1] %= 360

    def estimate(self):
        """ Returns mean and variance of the weighted particles. """
        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean) ** 2, weights=self.weights, axis=0)
        return mean, var

    def resample(self):
        if self.effective_n() < self.particles.shape[0] / 2:
            # print("resampling")
            indices = resample(self.weights)
            resample_from_index(self.particles, self.weights, indices)
        else:
            pass
            # print("not resampling")
