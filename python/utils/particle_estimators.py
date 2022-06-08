#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
particle_estimators.py: Provides particle filter implementation
"""
from enum import Enum

import numpy as np
from scipy.stats import norm

from utils.base_estimator import BaseEstimator, get_normal_vector


# simplest, but expensive and might miss important particles:
# from filterpy.monte_carlo import multinomial_resample as resample
# better runtime and more uniform, but might miss important samples:
# from filterpy.monte_carlo import residual_resample as resample
# very uniform, doesn't miss important samples:
from filterpy.monte_carlo import stratified_resample as resample

# corresponds to discretization "fine":
DISTANCES_CM = np.arange(7, 80, step=2.0)
ANGLES_DEG = np.arange(360, step=10)


STD_DISTANCE_CM = 5.0
STD_ANGLE_DEG = 5.0

# ignore pose estimates and predict particles by adding
# uniform disturbance.
PREDICT_UNIFORM = True


class State(Enum):
    NEED_UPDATE = 0
    NEED_PREDICT = 1
    NEED_RESAMPLE = 2
    READY = 3


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
    particles[:] = particles[indices, :]
    weights.resize(particles.shape[0])
    weights.fill(1.0 / len(weights))


class ParticleEstimator(BaseEstimator):
    def __init__(
        self,
        n_particles,
        platform="crazyflie",
        global_=True,
        distances_cm=DISTANCES_CM,
        angles_deg=ANGLES_DEG,
        predict_uniform=PREDICT_UNIFORM,
    ):
        super().__init__(
            n_window=2,
            platform=platform,
        )

        self.n_particles = n_particles
        self.particles = np.empty((n_particles, 2))
        self.particles[:, 0] = np.linspace(
            distances_cm[0], distances_cm[-1], n_particles
        )

        # yields an initial distribution that is not perfectly distributed
        # (more localized in a sphere round center) but it is good enough.
        dd, aa = np.meshgrid(distances_cm, angles_deg)
        self.particles[:, 0] = np.random.choice(
            dd.flatten(), size=n_particles, replace=True
        )
        self.particles[:, 1] = np.random.choice(
            aa.flatten(), size=n_particles, replace=True
        )

        self.weights = np.ones(n_particles) / n_particles

        # express particles in global reference frame.
        # if false, will express particles in local reference frame.
        self.global_ = global_

        self.distances_cm = distances_cm
        self.angles_deg = angles_deg

        self.predict_uniform = predict_uniform

        # state machine to make sure predict, update and resample are done
        # in the correct order and not more than necessary.
        self.state = State.READY

    def effective_n(self):
        return 1.0 / np.sum(np.square(self.weights))

    def add_distributions(self, *args, **kwargs):
        super().add_distributions(*args, **kwargs)
        self.state = State.NEED_PREDICT

    def get_distributions(self, simplify_angles=False, method="histogram"):
        if simplify_angles:
            # sample angles from current direction.
            angle_deg = self.get_forward_angle()
            self.particles[:, 1] = np.random.normal(
                loc=angle_deg, scale=STD_ANGLE_DEG, size=self.n_particles
            )

        if not self.state == State.READY:
            self.predict()
            self.update(simplify_angles=simplify_angles)
            self.resample()

        if method == "histogram":
            bins = get_bins(self.distances_cm)
            probs_dist, __ = np.histogram(
                self.particles[:, 0], bins=bins, weights=self.weights
            )

            bins = get_bins(self.angles_deg)
            probs_angles, __ = np.histogram(
                self.particles[:, 1], bins=bins, weights=self.weights
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

        If particles are in global reference frame, use the equation:
        a_local = a_global - rotation
        d_local = d_global - n(a_global).dot(position)
        """
        if not self.state == State.NEED_UPDATE:
            return

        for i, (d_particle, a_particle) in enumerate(self.particles):
            if self.global_:
                normal_global = get_normal_vector(a_particle)
                a_local = a_particle - self.rotations[self.index]
                d_local = d_particle - normal_global.dot(self.positions[self.index])
            else:
                a_local = a_particle
                d_local = d_particle
            # print(f"for global {d_particle:.1f}, local distance and angle: {d_local:.1f}, {a_local:.0f}")
            weight = 1.0
            for mic, diff_dist in self.difference_p[self.index].items():
                delta = self.context.get_delta(a_local, d_local, mic)
                weight *= diff_dist(delta)  # interpolate at delta

            if self.angle_probs[self.index] is not None:
                weight *= self.angle_probs[self.index](a_local)  # interpolate at angle

            self.weights[i] *= weight
        self.weights /= np.sum(self.weights)

        self.state = State.NEED_RESAMPLE

    def predict(self):
        """Predict new particles based on movement estimates."""
        if not self.state == State.NEED_PREDICT:
            return

        if (not self.predict_uniform) and self.filled and (not self.global_):
            # rolling window with two elements:
            previous = 1 if self.index == 0 else 0
            pos_delta = self.positions[self.index] - self.positions[previous]
            rot_delta = self.rotations[self.index] - self.rotations[previous]
            for i in range(self.particles.shape[0]):
                a_local = self.particles[i, 1]
                d_corr = get_normal_vector(self.rotations[previous] + a_local).dot(
                    pos_delta
                )
                self.particles[i, 0] -= d_corr
                self.particles[i, 1] -= rot_delta

        self.particles[:, 0] += np.random.normal(
            scale=STD_DISTANCE_CM, size=self.n_particles
        )
        self.particles[:, 1] += np.random.normal(
            scale=STD_ANGLE_DEG, size=self.n_particles
        )

        self.particles[:, 0] = np.clip(
            self.particles[:, 0],
            a_min=self.distances_cm[0],
            a_max=self.distances_cm[-1],
        )
        self.particles[:, 1] %= 360

        self.state = State.NEED_UPDATE

    def estimate(self):
        """Returns mean and variance of the weighted particles."""
        import scipy.stats

        if not self.state == State.READY:
            print("Warning: not ready for estimation yet.")
            return None

        mean_distance = np.average(self.particles[:, 0], weights=self.weights)
        var_distance = np.average(
            (self.particles[:, 0] - mean_distance) ** 2, weights=self.weights
        )

        sin_ = np.sum(
            np.multiply(np.sin(self.particles[:, 1] / 180 * np.pi), self.weights)
        )
        cos_ = np.sum(
            np.multiply(np.cos(self.particles[:, 1] / 180 * np.pi), self.weights)
        )
        mean_angle = 180 / np.pi * np.arctan2(sin_, cos_)
        mean_angle %= 360
        var_angle = 180 / np.pi * np.sqrt(sin_**2 + cos_**2)
        return (mean_distance, mean_angle), (var_distance, var_angle)

    def resample(self):
        if not self.state == State.NEED_RESAMPLE:
            return

        if True:  # self.effective_n() < self.particles.shape[0] / 2:
            try:
                indices = resample(self.weights)
            except:
                print(self.weights)
                raise
            resample_from_index(self.particles, self.weights, indices)
        else:
            print("Warning: not resampling")
            pass

        self.state = State.READY
