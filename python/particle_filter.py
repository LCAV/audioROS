"""
Particle filter implementation adapted from 

https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
"""

import numpy as np
import matplotlib.pylab as plt

from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn, uniform
import scipy.stats


def create_uniform_particles(x_range, y_range, hdg_range, N):
    particles = np.empty((N, 3))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
    particles[:, 2] %= 2 * np.pi
    return particles


def predict(particles, u, std, dt=1.0):
    """ Move according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity). """

    N = len(particles)
    # update heading
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * np.pi

    # move in the (noisy) commanded direction
    dist = (u[1] * dt) + (randn(N) * std[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist


def update(particles, weights, z, R, landmarks):
    for i, landmark in enumerate(landmarks):
        distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
        weights *= scipy.stats.norm(distance, R).pdf(z[i])

    weights += 1.0e-300  # avoid round-off to zero
    weights /= sum(weights)  # normalize


def estimate(particles, weights):
    """ Returns mean and variance of the weighted particles. """
    pos = particles[:, 0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var = np.average((pos - mean) ** 2, weights=weights, axis=0)
    return mean, var


def neff(weights):
    return 1.0 / np.sum(np.square(weights))


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights.resize(len(particles))
    weights.fill(1.0 / len(weights))


def run_pf1(
    N,
    iters=18,
    sensor_std_err=0.1,
    do_plot=True,
    plot_particles=False,
    xlim=(0, 20),
    ylim=(0, 20),
    initial_x=None,
):
    landmarks = np.array([[-1, 2], [5, 10], [12, 14], [18, 21]])
    NL = len(landmarks)

    plt.figure()

    # create particles and weights
    if initial_x is not None:
        particles = create_gaussian_particles(
            mean=initial_x, std=(5, 5, np.pi / 4), N=N
        )
    else:
        particles = create_uniform_particles((0, 20), (0, 20), (0, 6.28), N)
    weights = np.ones(N) / N

    if plot_particles:
        alpha = 0.20
        if N > 5000:
            alpha *= np.sqrt(5000) / np.sqrt(N)
        plt.scatter(particles[:, 0], particles[:, 1], alpha=alpha, color="g")

    xs = []
    robot_pos = np.array([0.0, 0.0])
    for x in range(iters):
        robot_pos += (1, 1)

        # distance from robot to each landmark
        zs = norm(landmarks - robot_pos, axis=1) + (randn(NL) * sensor_std_err)

        # move diagonally forward to (x+1, x+1)
        predict(particles, u=(0.00, 1.414), std=(0.2, 0.05))

        # incorporate measurements
        update(particles, weights, z=zs, R=sensor_std_err, landmarks=landmarks)

        # resample if too few effective particles
        if neff(weights) < N / 2:
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)
            assert np.allclose(weights, 1 / N)
        mu, var = estimate(particles, weights)
        xs.append(mu)

        if plot_particles:
            plt.scatter(particles[:, 0], particles[:, 1], color="k", marker=",", s=1)
        p1 = plt.scatter(robot_pos[0], robot_pos[1], marker="+", color="k", s=180, lw=3)
        p2 = plt.scatter(mu[0], mu[1], marker="s", color="r")

    xs = np.array(xs)
    # plt.plot(xs[:, 0], xs[:, 1])
    plt.legend([p1, p2], ["Actual", "PF"], loc=4, numpoints=1)
    plt.xlim(*xlim)
    plt.ylim(*ylim)
    print("final position error, variance:\n\t", mu - np.array([iters, iters]), var)
    plt.show()


if __name__ == "__main__":
    from numpy.random import seed

    seed(2)
    run_pf1(N=5000, plot_particles=True)
