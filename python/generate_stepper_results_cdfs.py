#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_stepper_results_cdf.py: Calculate and save distance estimates for all measured stepper data.
"""

from crazyflie_description_py.experiments import (
    WALL_ANGLE_DEG_STEPPER,
    WALL_DISTANCE_CM_STEPPER,
)

import time

import numpy as np
import pandas as pd
import progressbar

from utils.estimators import DistanceEstimator, get_estimate
from utils.inference import Inference
from utils.simulation import get_freq_slice_theory

from utils.moving_estimators import MovingEstimator
from utils.particle_estimators import ParticleEstimator

DISCRETIZATIONS = dict(
    superfine=(1.0, 5),
    fine=(2.0, 10),
    medium=(3.0, 20),
    coarse=(5.0, 30),
    supercoarse=(10.0, 90),
)

PARAMETERS_ALL = dict(
    discretizations=["superfine", "fine", "medium", "coarse", "supercoarse"],
    n_windows=[1, 3, 5],
    methods=["raw", "theoretical", "calibrated"],
)

MAG_THRESH = 1e-3

# for debugging only.
# TODO(FD) remove this option, because it gives worse results.
USE_DATA_COLLECTOR = False


def get_magnitudes(stft, mag_thresh=MAG_THRESH):
    stft[np.abs(stft) < mag_thresh] = np.nan
    return np.nanmedian(np.abs(stft), axis=0)


def generate_results(df_chosen, fname="", parameters=PARAMETERS_ALL):
    def save():
        save_df = err_df.apply(pd.to_numeric, axis=0, errors="ignore")
        save_df.to_pickle(fname)
        print("Saved intermediate as", fname)

    def angle_error(a1_deg, a2_deg):
        return (
            np.diff(np.unwrap([a1_deg / 180 * np.pi, a2_deg / 180 * np.pi]))[0]
            * 180
            / np.pi
        )

    def fill_distance(err_df, probs_dist, method):
        d = get_estimate(distances_cm, probs_dist)
        err_df.loc[len(err_df), :] = {
            "error": d - distance,
            "mic": "all",
            "distance": distance,
            "method": method,
            "algorithm": algo + " distance",
            "runtime": runtime,
            "discretization": discretization,
        }

    def fill_angle(err_df, probs_angles, method):
        a = get_estimate(angles_deg, probs_angles)
        error_deg = angle_error(a, azimuth_deg)
        runtime = time.time() - t1
        err_df.loc[len(err_df), :] = {
            "error": error_deg,
            "mic": "all",
            "distance": distance,
            "method": method,
            "algorithm": algo + " angle",
            "runtime": runtime,
            "discretization": discretization,
        }

    def fill_random_and_fixed(err_df):
        for d in err_df.distance.unique():
            err_df.loc[len(err_df), :] = dict(
                method="random",
                mic="all",
                distance=d,
                error=np.random.choice(distances_cm) - d,
                algorithm=f"{algo} distance",
                runtime=0,
                discretization=discretization,
            )
            err_df.loc[len(err_df), :] = dict(
                method="random",
                mic="all",
                distance=d,
                error=np.random.choice(angles_deg) - azimuth_deg,
                algorithm=f"{algo} angle",
                runtime=0,
                discretization=discretization,
            )
            # fill with constant zero distance estimate
            err_df.loc[len(err_df), :] = dict(
                method="fixed",
                mic="all",
                distance=d,
                error=np.mean(distances_cm) - d,
                algorithm=f"{algo} distance",
                runtime=0,
                discretization=discretization,
            )
            # fill with constant zero angle estimate
            err_df.loc[len(err_df), :] = dict(
                method="fixed",
                mic="all",
                distance=d,
                error=180,  # worse case error
                algorithm=f"{algo} angle",
                runtime=0,
                discretization=discretization,
            )

    azimuth_deg = WALL_ANGLE_DEG_STEPPER
    chosen_mics = range(4)  # [0, 1, 3]
    algo = "bayes"  # could do cost to
    use_uniform_prior = True

    err_df = pd.DataFrame(
        columns=[
            "method",
            "mic",
            "distance",
            "error",
            "algorithm",
            "runtime",
            "discretization",
        ]
    )

    inf_machine = Inference()

    if USE_DATA_COLLECTOR:
        from utils.calibration import get_calibration_function_median

        calib_function_median, __ = get_calibration_function_median(
            mic_type="audio_deck", exp_name="2021_07_08_stepper_fast", motors=0, snr=5
        )
        df_matrix, distances, frequencies = df_chosen.get_df_matrix()
    else:
        distances = df_chosen.distance.values
        df_chosen.loc[:, "distance"] += WALL_DISTANCE_CM_STEPPER
        all_magnitudes = np.abs(np.concatenate([*df_chosen.stft.values]))
        calibration_magnitudes = np.median(all_magnitudes, axis=0)
        frequencies = df_chosen.iloc[0].frequencies_matrix[0, :]

    inf_machine.add_geometry([min(distances), max(distances)], azimuth_deg)

    for discretization in parameters["discretizations"]:
        step_cm, step_deg = DISCRETIZATIONS[discretization]
        print(f"----------------- discretization {discretization} -------------------")

        distances_cm = np.arange(7, 100, step=step_cm)
        angles_deg = np.arange(360, step=step_deg)
        n_particles = len(distances_cm) * len(angles_deg) // 2
        print(f"Nd = {len(distances_cm)}, Na = {len(angles_deg)} -> Np = {n_particles}")

        for method in parameters["methods"]:
            print("running", method)

            estimator_dict = {
                f"histogram {n_window}": MovingEstimator(
                    n_window=n_window, distances_cm=distances_cm, angles_deg=angles_deg
                )
                for n_window in parameters["n_windows"]
            }
            estimator_dict[f"particle {n_particles}"] = ParticleEstimator(
                n_particles=n_particles,
                global_=False,
                distances_cm=distances_cm,
                angles_deg=angles_deg,
            )

            p = progressbar.ProgressBar(maxval=len(distances))
            p.start()

            for i_d, distance in enumerate(distances):
                if method == "theoretical":
                    magnitudes = get_freq_slice_theory(
                        frequencies,
                        distance,
                        azimuth_deg=azimuth_deg,
                        chosen_mics=chosen_mics,
                    )
                    magnitudes = np.sqrt(magnitudes.T)  # now it's 4x32
                    freqs = frequencies
                else:

                    if USE_DATA_COLLECTOR:
                        magnitudes, *_ = df_chosen.get_frequency_slice_fixed(
                            frequencies, distance, mics=chosen_mics
                        )
                    else:
                        stft_exp = df_chosen.loc[
                            df_chosen.distance == distance, "stft"
                        ].iloc[0]
                        magnitudes = get_magnitudes(stft_exp)[chosen_mics, :]

                if method == "calibrated" and not USE_DATA_COLLECTOR:
                    magnitudes /= calibration_magnitudes[chosen_mics, :]
                elif USE_DATA_COLLECTOR:
                    if method == "calibrated":
                        inf_machine.add_calibration_function(calib_function_median)
                    else:
                        inf_machine.add_calibration_function(None)

                inf_machine.add_data(magnitudes, frequencies, mics=chosen_mics)
                inf_machine.filter_out_freqs()

                distance_estimator = DistanceEstimator(
                    distances_cm=distances_cm, angles_deg=angles_deg
                )
                diff_dict = {}
                for i_mic, mic_idx in enumerate(chosen_mics):
                    distances_cm_here, proba, diff_cm = inf_machine.do_inference(
                        algorithm=algo, mic_idx=i_mic
                    )
                    diff_dict[mic_idx] = (diff_cm, proba)
                    distance_estimator.add_distribution(diff_cm * 1e-2, proba, mic_idx)
                    # uncomment to add individual mics.
                    # d = get_estimate(distances_cm_here, proba)
                    # err_df.loc[len(err_df), :] = {
                    #    'error': d - distance,
                    #    'mic': mic_idx,
                    #    'distance': distance,
                    #    'method': method,
                    #    'algorithm': algo
                    # }
                for estimator in estimator_dict.values():
                    estimator.add_distributions(diff_dict, position_cm=[0, -distance])

                angle_deg_here = None if use_uniform_prior else azimuth_deg

                t1 = time.time()
                __, proba = distance_estimator.get_distance_distribution(
                    angle_deg=angle_deg_here
                )
                runtime = time.time() - t1
                fill_distance(err_df, proba, method=f"{method} single")

                # angle distribution doesn't work with uniform prior!
                # this is normal, there is no information on angle without some distance information.
                t1 = time.time()
                __, proba = distance_estimator.get_angle_distribution(
                    distance_estimate_cm=distance
                )
                runtime = time.time() - t1
                fill_angle(err_df, proba, method=f"{method} single")

                for name, estimator in estimator_dict.items():
                    t1 = time.time()
                    __, probs_dist, __, probs_angles = estimator.get_distributions(
                        simplify_angles=False
                    )
                    runtime = time.time() - t1
                    fill_distance(err_df, probs_dist, method=f"{method} {name}")
                    fill_angle(err_df, probs_angles, method=f"{method} {name}")

                p.update(i_d)

            if fname != "":
                save()

        fill_random_and_fixed(err_df)
        if fname != "":
            save()


if __name__ == "__main__":

    exp_name = "2021_07_08_stepper_fast"
    motors = "all45000"
    bin_selection = 5

    if USE_DATA_COLLECTOR:
        from utils.data_collector import DataCollector

        mic_type = "audio_deck"
        df_chosen = DataCollector()
        df_chosen.fill_from_backup(exp_name, mic_type, motors, bin_selection)
    else:
        df_all = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
        df_chosen = df_all.loc[
            (df_all.motors == motors) & (df_all.bin_selection == bin_selection)
        ].copy()

    parameters = dict(
        discretizations=["superfine", "fine", "medium", "coarse", "supercoarse"],
        n_windows=[1, 3, 5],
        methods=["calibrated"],
    )
    fname = "results/stepper_results_timing.pkl"
    generate_results(df_chosen, fname=fname, parameters=parameters)

    parameters = dict(
        discretizations=["superfine", "fine", "medium", "coarse", "supercoarse"],
        n_windows=[1, 3, 5],
        methods=["raw", "theoretical", "calibrated"],
    )
    fname = "results/stepper_results_cdfs.pkl"
    generate_results(df_chosen, fname=fname, parameters=parameters)
