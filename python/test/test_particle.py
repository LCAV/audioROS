#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_particle.py: Test moving particle filter class. 
"""
import sys, os

import numpy as np
import matplotlib.pylab as plt

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.particle_estimators import ParticleEstimator

from helpers import get_delta_distribution, measure_wall, DISTANCE_GLOB, ANGLE_GLOB

MIC_IDX = [0, 1]
SCALE = 20


def test_particle_filter():
    n_particles = 100
    movement_dir_deg = 90
    yaw_deg = movement_dir_deg - 90
    poses = [
        [
            step * np.cos(movement_dir_deg / 180 * np.pi),
            step * np.sin(movement_dir_deg / 180 * np.pi),
            yaw_deg,
        ]
        for step in np.arange(20, step=3.0)
    ]

    ParticleEstimator.DISTANCE_RANGE_CM = [0, 40]
    particle_filter = ParticleEstimator(n_particles=n_particles)

    fig, (ax1, ax2) = plt.subplots(1, 2)
    ax1.scatter(
        [0] * n_particles,
        particle_filter.particles[:, 0],
        s=particle_filter.weights * SCALE,
    )
    ax2.scatter(
        [0] * n_particles,
        particle_filter.particles[:, 1],
        s=particle_filter.weights * SCALE,
    )

    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        # print("local distance and angle", distance, angle)

        diff_dict = {}
        for mic_idx in MIC_IDX:
            delta_grid, probs = get_delta_distribution(
                distance, angle, mic_idx, prob_method="normal"
            )
            diff_dict[mic_idx] = (delta_grid, probs)

        particle_filter.add_distributions(diff_dict, pose[:2], pose[2])

        particle_filter.predict()
        ax1.scatter(
            [i + 1] * n_particles,
            particle_filter.particles[:, 0],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )
        ax2.scatter(
            [i + 1] * n_particles,
            particle_filter.particles[:, 1],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )
        particle_filter.update()
        ax1.scatter(
            [i + 1.3] * n_particles,
            particle_filter.particles[:, 0],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )
        ax2.scatter(
            [i + 1.3] * n_particles,
            particle_filter.particles[:, 1],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )
        particle_filter.resample()
        ax1.scatter(
            [i + 1.6] * n_particles,
            particle_filter.particles[:, 0],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )
        ax2.scatter(
            [i + 1.6] * n_particles,
            particle_filter.particles[:, 1],
            color=f"C{i+1}",
            s=particle_filter.weights * SCALE,
        )

        distance_estimate = particle_filter.estimate()[0][0]
        ax1.scatter(
            i + 1.5, distance_estimate, color=f"C{i+1}", marker="x", label="estimate"
        )

        angle_estimate = particle_filter.estimate()[0][1]
        ax2.scatter(
            i + 1.5, angle_estimate, color=f"C{i+1}", marker="x", label="estimate"
        )
    ax1.axhline(DISTANCE_GLOB, color=f"k", label="ground truth")
    ax2.axhline(ANGLE_GLOB, color=f"k", label="ground truth")
    ax2.set_ylim(0, 360)
    plt.show()


if __name__ == "__main__":

    test_particle_filter()
