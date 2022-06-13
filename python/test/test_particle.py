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

from helpers import measure_wall, get_diff_dict, get_angle_distribution

MIC_IDX = [0, 1]
SCALE = 500

PREDICT_UNIFORM = False

BEAMFORM = True


def test_particle_filter():
    distances_cm = np.arange(50)
    angles_deg = np.arange(360)
    n_particles = 100

    #particle_filter = ParticleEstimator(
    #    n_particles=n_particles,
    #    predict_uniform=True,
    #    distances_cm=distances_cm,
    #    angles_deg=angles_deg,
    #)
    #do_test(particle_filter, title="predict uniform")

    particle_filter = ParticleEstimator(
        n_particles=n_particles,
        predict_uniform=False,
        distances_cm=distances_cm,
        angles_deg=angles_deg,
    )
    do_test(particle_filter, title="predict non-uniform")
    plt.show()


def do_test(particle_filter, title=""):
    def plot(x, col):
        axs[0].scatter(
            [x] * particle_filter.n_particles,
            particle_filter.states[:, 0],
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )
        axs[1].scatter(
            [x] * particle_filter.n_particles,
            particle_filter.states[:, 1],
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )

    def plot_final(col):
        axs[2].scatter(
            particle_filter.states[:, 1]
            * np.cos(particle_filter.states[:, 0] / 180 * np.pi),
            particle_filter.states[:, 0]
            * np.sin(particle_filter.states[:, 0] / 180 * np.pi),
            color=f"C{col}",
            s=particle_filter.weights * SCALE,
            alpha=0.5,
        )

    def plot_estimate(x, col):
        distance_estimate, angle_estimate = particle_filter.estimate()[0]
        axs[0].scatter(x, angle_estimate, color=f"k", marker="x", label="estimate")
        axs[1].scatter(x, distance_estimate, color=f"k", marker="x", label="estimate")

    movement_dir_deg = 90
    yaw_deg = movement_dir_deg - 90

    rots = np.arange(0, 20)
    steps = np.arange(0, 20)
    poses = [
        [
            step * np.cos(movement_dir_deg / 180 * np.pi),
            step * np.sin(movement_dir_deg / 180 * np.pi),
            rot,
        ]
        for rot, step in zip(rots, steps)
    ]

    fig, axs = plt.subplots(1, 3)
    fig.suptitle(title)
    plot(0, 0)

    # fig_pos, axs_pos = plt.subplots(len(poses), 2)
    # fig_pos.suptitle(f"{title} over time")
    for i, pose in enumerate(poses):
        distance, angle = measure_wall(pose)
        print("local distance and angle", distance, angle)

        diff_dict = get_diff_dict(distance, angle, prob_method="normal")
        particle_filter.add_distributions(diff_dict, pose[:2], pose[2])

        angles, probs = get_angle_distribution(angle, prob_method="normal")
        particle_filter.add_angle_distribution(angles, probs)

        particle_filter.predict()
        plot(i + 1, i + 1)
        particle_filter.update()
        plot(i + 1.3, i + 1)
        particle_filter.resample()
        plot(i + 1.6, i + 1)
        plot_estimate(i + 1.5, i + 1)

        axs[1].scatter(
            i + 1.5, distance, color=f"k", label="ground truth", marker="+"
        )
        axs[0].scatter(i + 1.5, angle, color=f"k", label="ground truth", marker="+")
    plot_final(0)

    axs[0].set_title("angle particle evolution")
    axs[1].set_title("distance particle evolution")
    axs[2].set_title("final plane estimate")
    axs[2].set_xlabel("x [cm]")
    axs[2].set_ylabel("y [cm]")

    axs[0].set_ylim(0, 360)
    axs[1].set_ylim(0, 50)


if __name__ == "__main__":
    test_particle_filter()
