#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
particle_estimators.py: 
"""

import numpy as np

from utils.moving_estimators import BaseEstimator, get_normal

from filterpy.monte_carlo import systematic_resample

STD_DISTANCE_CM = 5.0
STD_ANGLE_DEG = 5.0


def resample_from_index(particles, weights, indices):
    particles[:] = particles[indices]
    weights.resize(len(particles))
    weights.fill(1.0 / len(weights))


def neff(weights):
    return 1.0 / np.sum(np.square(weights))


class ParticleEstimator(BaseEstimator):
    DISTANCE_RANGE_CM = [0, 100]
    # ANGLES_DEG = np.arange(360, step=1.0)
    # ANGLES_DEG = np.arange(360, step=90)
    ANGLES_DEG = [90]  # np.arange(360, step=90)

    def __init__(self, n_particles, platform="crazyflie"):

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

    def update(self):
        for i, (d_global, a_global) in enumerate(self.particles):

            normal_global = get_normal(a_global)
            a_local = a_global - self.rotations[0]
            d_local = d_global + normal_global.dot(self.positions[0])
            # print(f"for global {d_global:.1f}, local distance and angle: {d_local:.1f}, {a_local:.0f}")

            weight = 1.0
            for mic, diff_dist in self.difference_p[0].items():
                delta = self.context.get_delta(a_local, d_local, mic)
                weight *= diff_dist(delta)
            self.weights[i] *= weight
        self.weights /= np.sum(self.weights)

    def predict(self):
        for i in range(self.particles.shape[0]):
            self.particles[i][0] += np.random.normal(scale=STD_DISTANCE_CM)
            self.particles[i][0] = np.clip(
                self.particles[i][0],
                a_min=self.DISTANCE_RANGE_CM[0],
                a_max=self.DISTANCE_RANGE_CM[1],
            )

            self.particles[i][1] += np.random.normal(scale=STD_ANGLE_DEG)
            self.particles[i][1] %= 360

    def estimate(self):
        """ Returns mean and variance of the weighted particles. """
        mean = np.average(self.particles, weights=self.weights, axis=0)
        var = np.average((self.particles - mean) ** 2, weights=self.weights, axis=0)
        return mean, var

    def resample(self):
        if neff(self.weights) < self.particles.shape[0] / 2:
            # print("resampling")
            indices = systematic_resample(self.weights)
            resample_from_index(self.particles, self.weights, indices)
        else:
            pass
            # print("not resampling")
