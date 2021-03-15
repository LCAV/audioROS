#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
generate_simulation_results.py: investigate distance inference performance for different types and levels of noise
and different algorithms.
"""

import itertools

import numpy as np
import pandas as pd

import progressbar

from simulation import get_df_theory_simple, get_deltas_from_global
from wall_detector import get_probability_cost, get_probability_fft


def simulate_frequency_slice(
    distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances
):
    np.random.seed(1)

    yaw_deg = 0
    mic_idx = 1
    n_methods = 2
    distances_grid = np.arange(100)
    deltas_m, d0 = get_deltas_from_global(yaw_deg, distances_cm, mic_idx)

    n_total = (
        n_instances
        * len(distances_cm)
        * len(sigmas_delta_cm)
        * len(sigmas_f)
        * len(sigmas_y)
        * n_methods
    )

    results_df = pd.DataFrame(
        columns=[
            "sigmadelta",
            "sigmaf",
            "sigmay",
            "method",
            "error",
            "counter",
            "distance",
        ]
    )

    # for normalizing sigmay:

    i = 0
    with progressbar.ProgressBar(max_value=n_total) as p:
        for distance_cm, delta_m in zip(distances_cm, deltas_m):
            for (sigma_delta_cm, sigma_f, sigma_y) in itertools.product(
                sigmas_delta_cm, sigmas_f, sigmas_y
            ):
                for counter in range(n_instances):
                    delta_m_noisy = (
                        delta_m + np.random.normal(scale=sigma_delta_cm) * 1e-2
                    )
                    frequencies_noisy = frequencies + np.random.normal(
                        scale=sigma_f, size=len(frequencies)
                    )

                    slice_f = get_df_theory_simple(
                        delta_m_noisy, frequencies_noisy, flat=True, d0=d0
                    )
                    slice_f += np.random.normal(scale=sigma_y, size=len(slice_f))

                    distances_fft, probs_fft = get_probability_fft(
                        slice_f,
                        frequencies_noisy,
                        mic_idx=mic_idx,
                        distance_range=[min(distances_grid), max(distances_grid)],
                    )

                    probs_cost = get_probability_cost(
                        slice_f, frequencies_noisy, distances_grid, mic_idx=mic_idx
                    )

                    for method, probs, dist in zip(
                        ["fft", "cost"],
                        [probs_fft, probs_cost],
                        [distances_fft, distances_grid],
                    ):

                        d_estimate = dist[np.argmax(probs)]
                        error = np.abs(d_estimate - distance_cm)

                        results_df.loc[len(results_df), :] = dict(
                            counter=counter,
                            distance=distance_cm,
                            sigmadelta=sigma_delta_cm,
                            sigmaf=sigma_f,
                            sigmay=sigma_y,
                            method=method,
                            error=error,
                        )
                        p.update(i)
                        i += 1

    results_df = results_df.apply(pd.to_numeric, errors="ignore", axis=1)
    return results_df


def simulate_distance_slice(
    gammas_deg, start_distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances, ax=None
):
    from wall_detector import get_approach_angle_fft
    from wall_detector import get_approach_angle_cost
    np.random.seed(1)
    yaw_deg = 0
    mic_idx = 1
    n_methods = 2

    relative_distances_cm = np.arange(20) 

    start_distances_grid = np.arange(40) 
    gammas_grid = np.linspace(0, 180, 180)

    n_total = (
        n_instances
        * len(start_distances_cm)
        * len(frequencies)
        * len(sigmas_delta_cm)
        * len(sigmas_f)
        * len(sigmas_y)
        * n_methods
    )

    results_df = pd.DataFrame(
        columns=[
            "sigmadelta",
            "sigmaf",
            "sigmay",
            "method",
            "error",
            "counter",
            "start_distance",
            "gamma",
            "frequency", 
        ]
    )

    i = 0
    with progressbar.ProgressBar(max_value=n_total) as p:
        for start_distance, gamma in itertools.product(start_distances_cm, gammas_deg): 
            distances_cm = start_distance + relative_distances_cm * np.cos(gamma)
            deltas_m, d0 = get_deltas_from_global(yaw_deg, distances_cm, mic_idx)
            for (sigma_delta_cm, sigma_f, sigma_y, frequency) in itertools.product(
                sigmas_delta_cm, sigmas_f, sigmas_y, frequencies
            ):
                for counter in range(n_instances):
                    deltas_m_noisy = (
                        deltas_m + np.random.normal(scale=sigma_delta_cm, size=len(deltas_m)) * 1e-2
                    )
                    frequency_noisy = frequency + np.random.normal(scale=sigma_f)
                    slice_d = get_df_theory_simple(
                        deltas_m_noisy, frequency_noisy, flat=True, d0=d0
                    )
                    slice_d += np.random.normal(scale=sigma_y, size=len(slice_d))

                    cosines_gamma, probs_fft = get_approach_angle_fft(
                       slice_d, frequency, relative_distances_cm
                    )
                    gammas_fft = np.arccos(cosines_gamma) * 180 / np.pi

                    probs_cost = get_approach_angle_cost(
                        slice_d, frequency, relative_distances_cm, 
                        start_distances_grid, gammas_grid, mic_idx=mic_idx
                    ) # is of shape n_start_distances x n_gammas_grid
                    probs_cost = np.sum(probs_cost, axis=0)

                    for method, probs, gammas in zip(
                        ["fft", "cost"],
                        [probs_fft, probs_cost],
                        [gammas_fft, gammas_grid],
                    ):
                        gamma_estimate = gammas[np.argmax(probs)]
                        error = np.diff(np.unwrap([gamma_estimate, gamma]))

                        results_df.loc[len(results_df), :] = dict(
                            counter=counter,
                            gamma=gamma,
                            start_distance=start_distance,
                            sigmadelta=sigma_delta_cm,
                            sigmaf=sigma_f,
                            sigmay=sigma_y,
                            method=method,
                            error=error,
                        )
                        p.update(i)
                        i += 1

    results_df = results_df.apply(pd.to_numeric, errors="ignore", axis=1)
    return results_df


def compare_timing(n_instances):
    import time

    yaw_deg = 0
    distance_cm = 10
    mic_idx = 0
    delta_m, d0 = get_deltas_from_global(yaw_deg, distance_cm, mic_idx)
    distances_grid = np.arange(100)

    times = {"fft": [], "cost": []}
    with progressbar.ProgressBar(max_value=n_instances) as p:
        for counter in range(n_instances):
            slice_f = get_df_theory_simple(delta_m, frequencies, flat=True, d0=d0)

            t0 = time.time()
            distances_fft, probs_fft = get_probability_fft(
                slice_f,
                frequencies,
                mic_idx=mic_idx,
                distance_range=[min(distances_grid), max(distances_grid)],
            )
            d_estimate = distances_fft[np.argmax(probs_fft)]
            times["fft"].append(time.time() - t0)

            t0 = time.time()
            probs_cost = get_probability_cost(
                slice_f, frequencies, distances_grid, mic_idx=mic_idx
            )
            d_estimate = distances_grid[np.argmax(probs_cost)]
            times["cost"].append(time.time() - t0)

            p.update(counter)
    return times


if __name__ == "__main__":
    import sys
    np.random.seed(1)
    frequencies = np.linspace(1000, 5000, 100)

    start_distances_cm = np.array([30, 40, 50], dtype=float)
    start_distances_cm += np.random.uniform(low=-1, high=1, size=len(start_distances_cm))
    gammas_deg = [30, 50, 90]

    fname = "results/simulation/angle_noise.pkl"
    sigmas_delta_cm = [0]  # np.arange(20, step=2) # in meters!
    sigmas_f = [0]  # np.arange(100, step=20)
    sigmas_y = np.linspace(0, 2, 5)
    n_instances = 3  
    print('generating', fname)
    results_df = simulate_distance_slice(
    gammas_deg, start_distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances
    )
    pd.to_pickle(results_df, fname)
    print('saved as', fname)
    sys.exit()



    ### amplitude noise study
    fname = "results/simulation/amplitude_noise.pkl"
    sigmas_delta_cm = [0]  # np.arange(20, step=2) # in meters!
    sigmas_f = [0]  # np.arange(100, step=20)
    #sigmas_y = np.arange(1.05, step=0.05)
    sigmas_y = np.linspace(0, 2, 100)
    n_instances = 10  
    #print('generating', fname)
    #results_df = simulate_frequency_slice(distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances) 
    #pd.to_pickle(results_df, fname)
    #print('saved as', fname)

    ### delta noise study
    #fname = "results/simulation/delta_noise.pkl"
    #sigmas_delta_cm = np.arange(30, step=2)
    #n_instances = 100
    fname = "results/simulation/delta_noise_high.pkl"
    sigmas_delta_cm = np.arange(100, step=5)
    n_instances = 10
    sigmas_f = [0]  # np.arange(100, step=20)
    sigmas_y = [0]  # np.arange(10)
    print('generating', fname)
    results_df = simulate_frequency_slice(distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances)
    pd.to_pickle(results_df, fname)
    print('saved as', fname)

    ### frequency noise study
    fname = "results/simulation/frequency_noise.pkl"
    sigmas_delta_cm = [0]  # np.arange(20)
    sigmas_f = np.arange(200, step=10)
    sigmas_y = [0]  # np.arange(10)
    n_instances = 10
    #results_df = simulate_frequency_slice(distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances)
    #pd.to_pickle(results_df, fname)
    #print('saved as', fname)

    fname = "results/simulation/joint_noise.pkl"
    sigmas_f = [0]
    sigmas_delta_cm = np.arange(30, step=2)
    sigmas_y = [0.1, 0.3, 0.5]
    n_instances = 100
    #print('generating', fname)
    #results_df = simulate_frequency_slice(distances_cm, frequencies, sigmas_delta_cm, sigmas_f, sigmas_y, n_instances)
    #pd.to_pickle(results_df, fname)
    #print('saved as', fname)

    n_instances = 100
    times = compare_timing(n_instances)
    for method, time_list in times.items():
        print(f"average time for {method}: {np.mean(time_list)/n_instances:.3e}s")
