#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
velocity_commands.py: Create velocity commands resulting in constant acceleration.
"""

import numpy as np
import matplotlib.pylab as plt


def compute_velocity_commands(times, accelerations, control_time=0.1, ax=None):
    """
    Compute velocity commands that approximate the given accelerations. 

    :param times: list of durations (seconds)
    :param accelerations: list of corresponding accelerations (m/s^2)
    :param control_time: time (seconds) between two velocity commands (the smaller the better the approximation)
    :param ax: optional ax to plot results on.

    :returns: 
        control_times: list of control times (simply range of times with given spacing)
        control_velocities: list of velocities to be sent at given control times. 
    """

    end_velocity = 0
    end_time = 0
    control_velocities = np.array([0])
    control_times = [0]
    for t, a in zip(times, accelerations):
        new_times = control_time + np.arange(0, t, step=control_time)
        new_velocities = control_velocities[-1] + new_times * a
        if ax is not None:
            ax.scatter(new_times + control_times[-1], new_velocities, label=f"{a}")

        control_velocities = np.r_[control_velocities, new_velocities]
        control_times += list(new_times + control_times[-1])

    control_times = np.arange(0, len(accelerations) + control_time, step=control_time)
    return control_times, control_velocities


if __name__ == "__main__":
    control_time = 0.1  # seconds
    duration = 1  # seconds
    max_acc = 0.1  # m/s**2
    acc_up = np.linspace(0, max_acc, 3)
    accelerations = np.r_[acc_up, acc_up[1::-1], -acc_up[1:], -acc_up[1::-1]]
    accelerations = np.r_[accelerations, -accelerations]
    times = [duration] * len(accelerations)

    fig, ax = plt.subplots()
    control_times, control_velocities = compute_velocity_commands(
        times, accelerations, control_time=control_time, ax=ax
    )
    ax.legend(bbox_to_anchor=[1.0, 1.0], loc="upper left")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("velocity command [m/s]")
    ax.grid()

    plt.figure()
    plt.scatter(control_times, control_velocities)
    plt.grid()
    plt.show()
