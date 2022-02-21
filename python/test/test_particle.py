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

PREDICT_UNIFORM = False


def test_particle_filter():
    distances_cm = np.arange(40)
    angles_deg = np.arange(360, step=10)

    n_particles = 100
    particle_filter = ParticleEstimator(
        n_particles=n_particles,
        global_=False,
        predict_uniform=True,
        distances_cm=distances_cm,
        angles_deg=angles_deg,
    )
    do_test(particle_filter, title="local, uniform", global_=False)

    particle_filter = ParticleEstimator(
        n_particles=n_particles,
        global_=False,
        predict_uniform=False,
        distances_cm=distances_cm,
        angles_deg=angles_deg,
    )
    do_test(particle_filter, title="local", global_=False)

    particle_filter = ParticleEstimator(
        n_particles=n_particles,
        global_=True,
        predict_uniform=True,
        distances_cm=distances_cm,
        angles_deg=angles_deg,
    )
    do_test(particle_filter, title="global", global_=True)
    plt.show()


def do_test(particle_filter, title="", global_=True):
    def plot(x, col):
        axs[0].scatter(
            [x] * particle_filter.n_particles,
            particle_filter.particles[:, 0],
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )
        axs[1].scatter(
            [x] * particle_filter.n_particles,
            particle_filter.particles[:, 1],
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )

    def plot_final(col):
        axs[2].scatter(
            particle_filter.particles[:, 0]
            * np.cos(particle_filter.particles[:, 1] / 180 * np.pi),
            particle_filter.particles[:, 0]
            * np.sin(particle_filter.particles[:, 1] / 180 * np.pi),
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )

    def plot_estimate(x, col):
        distance_estimate, angle_estimate = particle_filter.estimate()[0]
        axs[0].scatter(x, distance_estimate, color=f"k", marker="x", label="estimate")
        axs[1].scatter(x, angle_estimate, color=f"k", marker="x", label="estimate")

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

    # particle_filter_global = ParticleEstimator(n_particles=n_particles, global_=True, predict_uniform=PREDICT_UNIFORM)

    fig, axs = plt.subplots(1, 3)
    fig.suptitle(title)
    plot(0, 0)
    plot_final(0)

    # fig_pos, axs_pos = plt.subplots(len(poses), 2)
    # fig_pos.suptitle(f"{title} over time")
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        print("local distance and angle", distance, angle)
        print("pose:", pose)

        diff_dict = {}
        for mic_idx in MIC_IDX:
            delta_grid, probs = get_delta_distribution(
                distance, angle, mic_idx, prob_method="normal"
            )
            diff_dict[mic_idx] = (delta_grid, probs)

        particle_filter.add_distributions(diff_dict, pose[:2], pose[2])

        particle_filter.predict()
        plot(i + 1, i + 1)
        particle_filter.update()
        plot(i + 1.3, i + 1)
        particle_filter.resample()
        plot(i + 1.6, i + 1)
        plot_estimate(i + 1.5, i + 1)

        if (i % 5 == 0) or (i == len(poses) - 1):
            plot_final(i + 1)

        if not global_:
            axs[0].scatter(
                i + 1.5, distance, color=f"k", label="ground truth", marker="+"
            )
            axs[1].scatter(i + 1.5, angle, color=f"k", label="ground truth", marker="+")

        # axs_pos[i, 0].set_xlabel("distance [cm]")
        # axs_pos[i, 1].set_xlabel("angle [deg]")
        # for method in ["histogram", "gaussian"]:
        #    (
        #        distances_cm,
        #        probs_dist,
        #        angles_deg,
        #        probs_angles,
        #    ) = particle_filter.get_distributions(method=method)
        #    axs_pos[i, 0].plot(distances_cm, probs_dist, label=method)
        #    axs_pos[i, 1].plot(angles_deg, probs_angles, label=method)
        # axs_pos[i, 1].set_xticks(angles_deg)
        # axs_pos[i, 1].set_xticklabels(angles_deg)

    if global_:
        axs[0].axhline(DISTANCE_GLOB, color=f"k", label="ground truth")
        axs[1].axhline(ANGLE_GLOB, color=f"k", label="ground truth")
        axs[1].set_ylim(0, 360)

    axs[1].set_ylim(0, 360)


if __name__ == "__main__":
    test_particle_filter()
