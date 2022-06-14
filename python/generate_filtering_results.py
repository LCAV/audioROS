#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_filtering_results.py: Calculate and save distance estimates for all measured stepper data.
"""


import itertools
import time

import numpy as np
import pandas as pd
import progressbar

from utils.constants import PLATFORM

if PLATFORM == "crazyflie":
    from crazyflie_description_py.experiments import DISTANCES_CM
    from crazyflie_description_py.experiments import WALL_ANGLE_DEG_STEPPER
elif PLATFORM == "epuck":
    from epuck_description_py.experiments import DISTANCES_CM
    from epuck_description_py.experiments import WALL_ANGLE_DEG_STEPPER

from utils.estimators import get_estimate
from utils.simulation import get_freq_slice_theory, get_freq_slice_pyroom, WIDEBAND_FILE
from utils.pandas_utils import save_pickle
from utils.plotting_tools import make_dirs

from utils.moving_estimators import MovingEstimator
from utils.pandas_utils import save_pickle
from utils.particle_estimators import ParticleEstimator
from utils.split_particle_estimators import SplitParticleEstimator
from utils.histogram_estimators import HistogramEstimator

from crazyflie_demo.wall_detection import WallDetection, MAG_THRESH

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
    chosen_mics=[[0,1,2,3]],
)

# d_max is c / 4 * \delta_f = 330 / (4 * 62) = 1.33m  for stepper experiments,
# d_max is c / 4 * \delta_f = 330 / (4 * 94) = 0.91m for flying experiments
DMIN = 7
DMAX = 80

CALIBRATION = "iir"
ALPHA_IIR = 0.3
BEAMFORM = True

USE_PYROOMACOUSTICS = True

def average_signals(stft, mag_thresh=MAG_THRESH):
    stft[np.abs(stft) < mag_thresh] = np.nan
    return  np.nanmedian(stft, axis=0)

def angle_error(a1_deg, a2_deg):
    error = (a2_deg - a1_deg) % 360  # between 0 and 360
    if np.ndim(error) == 0:
        if error > 180:
            error -= 360  # between -180 and 180
    else:
        error[error > 180] -= 360
    return error


def generate_results(df_chosen, fname="", parameters=PARAMETERS_ALL, beamform=BEAMFORM):
    if beamform:
        print("running with beamforming")
    else:
        print("running without beamforming")

    # to ensure reproducibility for particle filter
    np.random.seed(1)

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
            "chosen_mics": chosen_mics
        }

    def fill_angle(err_df, probs_angles, method):
        a = get_estimate(angles_deg, probs_angles)
        error_deg = angle_error(a, gt_azimuth_deg)
        runtime = time.time() - t1
        err_df.loc[len(err_df), :] = {
            "error": error_deg,
            "mic": "all",
            "distance": distance,
            "method": method,
            "algorithm": algo + " angle",
            "runtime": runtime,
            "discretization": discretization,
            "chosen_mics": chosen_mics
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
                chosen_mics=chosen_mics
            )
            err_df.loc[len(err_df), :] = dict(
                method="random",
                mic="all",
                distance=d,
                error=np.random.choice(angles_deg) - gt_azimuth_deg,
                algorithm=f"{algo} angle",
                runtime=0,
                discretization=discretization,
                chosen_mics=chosen_mics
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
                chosen_mics=chosen_mics
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
                chosen_mics=chosen_mics
            )

    gt_distances_cm = DISTANCES_CM
    assert len(DISTANCES_CM) == len(df_chosen.distance.unique())

    gt_azimuth_deg = WALL_ANGLE_DEG_STEPPER
    algo = "bayes"  # bayes or cost 
    err_df = pd.DataFrame(
        columns=[
            "method",
            "mic",
            "distance",
            "error",
            "algorithm",
            "runtime",
            "discretization",
            "chosen_mics"
        ]
    )

    df_chosen.sort_values("distance", ascending=False, inplace=True)
    frequencies = df_chosen.iloc[0].frequencies_matrix[0, :]

    for discretization in parameters["discretizations"]:
        step_cm, step_deg = DISCRETIZATIONS[discretization]
        print(f"----------------- discretization {discretization} -------------------")
        distances_cm = np.arange(DMIN, DMAX, step=step_cm)
        angles_deg = np.arange(360, step=step_deg)
        n_particles = len(distances_cm) * len(angles_deg) // 2
        print(f"Nd = {len(distances_cm)}, Na = {len(angles_deg)} -> Np = {n_particles}")

        WallDetection.CALIBRATION = CALIBRATION  # fixed, iir, window
        WallDetection.N_CALIBRATION = 2  # for iir, at least two.
        WallDetection.ALPHA_IIR = ALPHA_IIR
        WallDetection.MASK_BAD = None
        WallDetection.SIMPLIFY_ANGLES = False
        WallDetection.BEAMFORM = beamform

        if USE_PYROOMACOUSTICS:
            try:
                signal = np.load(WIDEBAND_FILE)
            except Exception as e:
                from utils.simulation import create_wideband_signal
                signal = create_wideband_signal(frequencies)
                make_dirs(WIDEBAND_FILE)
                np.save(WIDEBAND_FILE, signal)
                print("saved new file as", WIDEBAND_FILE)

        for method in parameters["methods"]:
            print("running", method)
            for chosen_mics in parameters["chosen_mics"]:
                estimator_dict = {}
                for n_window in parameters["n_windows"]:
                    estimator_dict[f"histogram {n_window}"] = MovingEstimator(
                        n_window=n_window, distances_cm=distances_cm, angles_deg=angles_deg
                    )
                estimator_dict[f"particle {n_particles}"] = ParticleEstimator(
                    n_particles=n_particles,
                    distances_cm=distances_cm,
                    angles_deg=angles_deg,
                )
                #estimator_dict[f"split particle {n_particles}"] = SplitParticleEstimator(
                #    n_particles=n_particles,
                #    distances_cm=distances_cm,
                #    angles_deg=angles_deg,
                #)
                ##estimator_dict["histogram"] = HistogramEstimator(
                #    distances_cm=distances_cm,
                #    angles_deg=angles_deg
                #)
                
                wall_detection_dict = {}
                for key, value in estimator_dict.items():
                    try: 
                        wall_detection_dict[key] = WallDetection(python_only=True, estimator=value)
                    except:
                        print(f"Skipping {key}")

                print(f"Using mics: {chosen_mics}")
                p = progressbar.ProgressBar(maxval=len(df_chosen.distance))
                p.start()
                for i, (distance_wrong, df_dist) in enumerate(df_chosen.groupby("distance", sort=False)):
                    distance = gt_distances_cm[i]
                    #print("wrong:", distance_wrong, "right:", distance)
                    if method == "theoretical":
                        if USE_PYROOMACOUSTICS:
                            signals_f = get_freq_slice_pyroom(
                                frequencies,
                                distance,
                                azimuth_deg=gt_azimuth_deg,
                                chosen_mics=chosen_mics,
                                signal=signal,
                                return_complex=True
                            )
                        else:
                            if beamform:
                                raise ValueError("Beamforming and theoretical simulation not supported. Use pyroom instead")
                            signals_f = get_freq_slice_theory(
                                frequencies,
                                distance,
                                azimuth_deg=gt_azimuth_deg,
                                chosen_mics=chosen_mics,
                            )
                            signals_f = np.sqrt(signals_f)  # now it's 4x32
                    else:
                        stft_exp = df_dist.loc[:, "stft"].iloc[0] # verified: 3 x 4 x 34
                        signals_f = average_signals(stft_exp)[chosen_mics, :]

                    position_cm = [0, -distance, 50]
                    yaw_deg = 0
                    for name, wall_detection in wall_detection_dict.items():
                        assert wall_detection.CALIBRATION == CALIBRATION
                        assert wall_detection.MASK_BAD is None
                        t1 = time.time()
                        
                        res = wall_detection.listener_callback_offline(
                            signals_f, frequencies, position_cm, yaw_deg, chosen_mics=chosen_mics
                        )
                        if res is None:
                            print("problem with",position_cm, yaw_deg, chosen_mics)
                            continue
                        probs_dist = res["prob_dist_moving"]
                        probs_angles = res["prob_angle_moving"]
                        runtime = time.time() - t1
                        fill_distance(err_df, probs_dist, method=f"{method} {name}")
                        fill_angle(err_df, probs_angles, method=f"{method} {name}")

                    p.update(i)

                if fname != "":
                    save_pickle(err_df, fname)

        fill_random_and_fixed(err_df)
        if fname != "":
            save_pickle(err_df, fname)


if __name__ == "__main__":
    if PLATFORM == "crazyflie": 
        exp_name = "2021_07_08_stepper_fast"
        motors = "all45000"
        bin_selection = 5
        name = "stepper"
    elif PLATFORM == "epuck":
        exp_name = "2021_07_27_epuck_wall"
        motors = "sweep_and_move"
        bin_selection = 0
        name = "epuck"

    df_all = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
    df_all = df_all.reindex(index=df_all.index[::-1])

    parameters = dict(
        discretizations=[
            "superfine",
            "fine",
            "medium",
            "coarse",
            "supercoarse",
        ],  
        #n_windows=[1, 3, 5],
        n_windows=[],
        methods=["calibrated", "theoretical"],
        #methods=["calibrated"],
        chosen_mics=[[0,1,2,3]],
    )
    for beamform in [True, False]:
        df_chosen = df_all.loc[
            (df_all.motors == motors) & (df_all.bin_selection == bin_selection)
        ].copy()
        if beamform:
            fname = f"results/{name}_results_beamform.pkl"
        else:
            fname = f"results/{name}_results.pkl"
        generate_results(df_chosen, fname=fname, parameters=parameters, beamform=beamform)


    parameters = dict(
        discretizations=["fine"],
        n_windows=[], # use only particle 
        #methods=["theoretical", "calibrated"],
        methods=["calibrated"],
        #chosen_mics= [m for n in range(1, 5) for m in itertools.combinations(range(4), n)]
        chosen_mics= [m for n in range(1, 5) for m in itertools.combinations(range(4), n)]
    )
    for beamform in [True, False]:
    #for beamform in [False]:
        if beamform:
            fname = f"results/{fname}_results_mics_ablation_beamform.pkl"
        else:
            fname = f"results/{name}_results_mics_ablation.pkl"
        df_chosen = df_all.loc[
            (df_all.motors == motors) & (df_all.bin_selection == bin_selection)
        ].copy()
        generate_results(df_chosen, fname=fname, parameters=parameters, beamform=beamform)
