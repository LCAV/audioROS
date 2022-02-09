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
SCALE = 500


def test_particle_filter():
    def plot(x, col):
        for axs, particle_filter in zip(
            [axs_glob, axs_loc], [particle_filter_global, particle_filter_local]
        ):
            axs[0].scatter(
                [x] * n_particles,
                particle_filter.particles[:, 0],
                color=f"C{col}",
                s=particle_filter.weights * SCALE,
                alpha=0.5,
            )
            axs[1].scatter(
                [x] * n_particles,
                particle_filter.particles[:, 1],
                color=f"C{col}",
                s=particle_filter.weights * SCALE,
                alpha=0.5,
            )

    def plot_estimate(x, col):
        for axs, particle_filter in zip(
            [axs_glob, axs_loc], [particle_filter_global, particle_filter_local]
        ):
            distance_estimate, angle_estimate = particle_filter.estimate()[0]
            axs[0].scatter(
                x, distance_estimate, color=f"C{col}", marker="x", label="estimate"
            )
            axs[1].scatter(
                x, angle_estimate, color=f"C{col}", marker="x", label="estimate"
            )

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
    particle_filter_global = ParticleEstimator(n_particles=n_particles, global_=True)
    particle_filter_local = ParticleEstimator(n_particles=n_particles, global_=False)

    fig, axs_loc = plt.subplots(1, 2)
    fig, axs_glob = plt.subplots(1, 2)
    fig, axs = plt.subplots(len(poses), 2)
    plot(0, 0)
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        # print("local distance and angle", distance, angle)

        diff_dict = {}
        for mic_idx in MIC_IDX:
            delta_grid, probs = get_delta_distribution(
                distance, angle, mic_idx, prob_method="normal"
            )
            diff_dict[mic_idx] = (delta_grid, probs)

        particle_filter_local.add_distributions(diff_dict, pose[:2], pose[2])
        particle_filter_global.add_distributions(diff_dict, pose[:2], pose[2])

        particle_filter_local.predict()
        particle_filter_global.predict()
        plot(i + 1, i + 1)
        particle_filter_local.update()
        particle_filter_global.update()
        plot(i + 1.3, i + 1)
        particle_filter_local.resample()
        particle_filter_global.resample()
        plot(i + 1.6, i + 1)

        plot_estimate(i + 1.5, i + 1)
        axs_loc[0].scatter(
            i + 1.5, distance, color=f"k", label="ground truth", marker="+"
        )
        axs_loc[1].scatter(i + 1.5, angle, color=f"k", label="ground truth", marker="+")

        axs[i, 0].set_xlabel("distance [cm]")
        axs[i, 1].set_xlabel("angle [deg]")
        for method in ["histogram", "gaussian"]:
            (
                distances_cm,
                probs_dist,
                angles_deg,
                probs_angles,
            ) = particle_filter_local.get_distributions(method=method)
            axs[i, 0].plot(distances_cm, probs_dist, label=method)
            axs[i, 1].plot(angles_deg, probs_angles, label=method)
        axs[i, 1].set_xticks(angles_deg)
        axs[i, 1].set_xticklabels(angles_deg)

    axs_glob[0].axhline(DISTANCE_GLOB, color=f"k", label="ground truth")
    axs_glob[1].axhline(ANGLE_GLOB, color=f"k", label="ground truth")
    axs_glob[1].set_ylim(0, 360)
    axs_loc[1].set_ylim(0, 360)
    plt.show()


if __name__ == "__main__":
    test_particle_filter()
