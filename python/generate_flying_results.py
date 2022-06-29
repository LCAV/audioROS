import itertools
import sys

import numpy as np
import pandas as pd
from progressbar import ProgressBar

from crazyflie_demo.wall_detection import WallDetection, MAG_THRESH

from utils.constants import PLATFORM
from utils.moving_estimators import get_estimate

#SIMULATE = "pyroom"
SIMULATE = ""

#ESTIMATORS = ["moving"] 
ESTIMATORS = ["particle"]
#ESTIMATORS = ["particle", "histogram", "moving"]  

def get_max_signals(stft, frequencies_matrix):
    signals_f_list = []
    freqs_list = []
    for t in range(stft.shape[0]):
        freq_bin = np.argmax(np.sum(np.abs(stft[t]), axis=0)) # sum across mics
        f = frequencies_matrix[t, freq_bin]
        #print(f"amplitude of {f}", np.sum(np.abs(stft[t, :, freq_bin]), axis=0))
        freqs_list.append(f)
        signals_f_list.append(stft[t, :, freq_bin])
    signals_f = np.array(signals_f_list) # freqs x 4
    freqs = np.array(freqs_list)
    return signals_f, freqs

def combine_stepper_df(data_df, motors="all45000", bin_selection=5, merge="average", platform=PLATFORM):
    """Combine all rows of a stepper dataset to one, as in flying datasets."""
    if platform == "crazyflie":
        print("crazyflie platform")
        from crazyflie_description_py.experiments import WALL_ANGLE_DEG_STEPPER, DISTANCES_CM
        height = 0.5
    elif platform == "epuck": 
        print("epuck platform")
        from epuck_description_py.experiments import WALL_ANGLE_DEG_STEPPER, DISTANCES_CM
        height = 0.5 # to emulate flying

    chosen_df = data_df.loc[
        (data_df.motors == motors) & (data_df.bin_selection == bin_selection)
        ][::-1] # reverse, to emulate wall approach.
    data_row = pd.Series(index=chosen_df.columns, dtype=object)

    #yaw_deg = 90 - WALL_ANGLE_DEG_STEPPER
    yaw_deg = 0

    # assumption: stft of all data has to be of the same shape!! 
    first_stft = chosen_df.iloc[0].stft
    n_mics = first_stft.shape[1]
    if bin_selection == 5:
        # at each time, all frequencies are played.
        n_freqs = first_stft.shape[2]
    else:
        # at each time, a new frequency is played.
        n_freqs = first_stft.shape[0]

    data_row.positions = np.empty((0, 4))
    data_row.stft = np.empty((0, n_mics, n_freqs))
    data_row.frequencies_matrix = np.empty((0, n_freqs))
    data_row.seconds = np.empty(0)
    for i, (distance_wrong, df) in enumerate(chosen_df.groupby("distance", sort=False)):
        assert len(df) == 1
        row = df.iloc[0]
        n_times = row.stft.shape[0]

        distance_cm = DISTANCES_CM[i]

        # with below values, we assume that wall is north and horizontal and the movement is vertical downwards.
        new_positions = np.array([0, -distance_cm*1e-2, height, yaw_deg]).reshape(1, -1)

        if bin_selection == 5:
            new_frequencies = row.frequencies_matrix[[0], :] # should be the same! 

            if merge == "average":
                valid_stft = row.stft.copy()
                valid_stft[valid_stft < MAG_THRESH] = np.nan
                new_stft = np.nanmedian(valid_stft, axis=0)[None, :, :] # 1 x 4 x 32
                new_seconds = row.seconds[[0]]
            elif merge == "middle":
                middle = row.stft.shape[0] // 2
                new_stft = row.stft[[middle], :, :] # 1 x 4 x 32
                new_seconds = row.seconds[[middle]]
            elif merge == "all":
                new_stft = row.stft # 3 x 4 x 32
                new_seconds = row.seconds
                new_positions = np.tile(new_positions, (n_times, 1))

        # for e-puck, we still use bin_selection 0, so we need to get the best bin at each time.
        elif bin_selection == 0:
            signals_f, freqs = get_max_signals(row.stft, row.frequencies_matrix)
            new_stft = signals_f.T[None, :, :]
            new_frequencies = freqs[None, :]
            new_seconds = [row.seconds[0]]

        data_row.positions = np.concatenate(
                [data_row.positions, new_positions], axis=0
        )
        data_row.frequencies_matrix = np.concatenate(
                [data_row.frequencies_matrix, new_frequencies], axis=0
        )
        data_row.stft = np.concatenate(
                [data_row.stft, new_stft], axis=0
        )
        data_row.seconds = np.concatenate([data_row.seconds, new_seconds], axis=0)
    return data_row

def generate_matrix_results(data_row, parameters, estimator, fname="", verbose=False, beamform=False, platform=PLATFORM):
    from utils.simulation import get_freq_slice_pyroom, get_freq_slice_theory, WIDEBAND_FILE
    if platform == "crazyflie":
        from crazyflie_description_py.experiments import WALL_ANGLE_DEG_STEPPER
    elif platform == "epuck": 
        from epuck_description_py.experiments import WALL_ANGLE_DEG_STEPPER

    n_rows = len(list(itertools.product(*parameters.values())))
    result_df = pd.DataFrame(
        columns=[
            "calibration name",
            "calibration param",
            "mask bad",
            "outlier factor",
            "n window",
            "simplify angles",
            "matrix distances",
            "matrix angles",
            "distances_cm",
            "angles_deg",
            "gt_distances_cm",
            "gt_angles_deg",
        ],
        index=range(n_rows),
    )
    counter = 0
    for calib, mask_bad, n_window, simplify in itertools.product(
        *parameters.values()
    ):
        WallDetection.CALIBRATION = calib[0]
        if calib[0] == "iir":
            WallDetection.N_CALIBRATION = 2
            WallDetection.ALPHA_IIR = calib[1]
        else:
            WallDetection.N_CALIBRATION = calib[1]
        WallDetection.MASK_BAD = mask_bad[0]
        WallDetection.OUTLIER_FACTOR = mask_bad[1]
        WallDetection.N_WINDOW = n_window
        WallDetection.SIMPLIFY_ANGLES = simplify
        WallDetection.BEAMFORM = beamform

        wall_detection = WallDetection(python_only=True, estimator=estimator)
        wall_detection.print_params()

        result_df.at[counter, "calibration name"] = calib[0]
        result_df.at[counter, "calibration param"] = calib[1]
        result_df.at[counter, "mask bad"] = str(mask_bad[0])
        result_df.at[counter, "outlier factor"] = mask_bad[1]
        result_df.at[counter, "n window"] = n_window
        result_df.at[counter, "simplify angles"] = simplify
        result_df.at[counter, "distances_cm"] = wall_detection.estimator.distances_cm
        result_df.at[counter, "angles_deg"] = wall_detection.estimator.angles_deg

        matrix_distances = None
        matrix_angles = None

        freqs_all = data_row.frequencies_matrix[0, :]  # is same across times
        freqs = freqs_all[freqs_all > 0]
        n_positions = data_row.positions.shape[0]
        times = []
        gt_distances = []
        gt_angles = []
        progressbar = ProgressBar(maxval=n_positions)
        progressbar.start()
        for i in range(n_positions):
            position_cm = data_row.positions[i, :3] * 1e2
            yaw_deg = data_row.positions[i, 3]
            time = data_row.seconds[i]

            if SIMULATE == "pyroom":
                signal = np.load(WIDEBAND_FILE)
                signals_f = get_freq_slice_pyroom(
                    freqs,
                    -position_cm[1],
                    azimuth_deg=WALL_ANGLE_DEG_STEPPER,
                    signal=signal,
                    return_complex=True
                )
                signals_f = signals_f.T
            elif SIMULATE == "theory":
                signals_f = get_freq_slice_theory(
                    freqs,
                    -position_cm[1],
                    azimuth_deg=WALL_ANGLE_DEG_STEPPER,
                )
                signals_f = np.sqrt(signals_f)  # now it's 4x32
            elif SIMULATE == "":
                signals_f = data_row.stft[i, :, freqs_all > 0]
            else:
                raise ValueError(SIMULATE)

            res = wall_detection.listener_callback_offline(
                signals_f.T,
                freqs,
                position_cm,
                yaw_deg,
                timestamp=time,
            )
            if res is None:  # if flight check did not pass, for instance
                continue

            gt_distances.append(-position_cm[1])
            gt_angles.append(WALL_ANGLE_DEG_STEPPER)

            if matrix_distances is None:
                matrix_distances = res["prob_dist_moving"].reshape((-1, 1))
            else:
                matrix_distances = np.c_[
                    matrix_distances, res["prob_dist_moving"].reshape((-1, 1))
                ]
            if matrix_angles is None:
                matrix_angles = res["prob_angle_moving"].reshape((-1, 1))
            else:
                matrix_angles = np.c_[matrix_angles, res["prob_angle_moving"].reshape((-1, 1))]
            progressbar.update(i)

        result_df.at[counter, "matrix distances"] = matrix_distances
        result_df.at[counter, "matrix angles"] = matrix_angles
        result_df.at[counter, "average_time"] = np.mean(times)
        result_df.at[counter, "gt_distances_cm"] = np.array(gt_distances)
        result_df.at[counter, "gt_angles_deg"] = np.array(gt_angles)
        counter += 1
        if fname != "":
            result_df.to_pickle(fname)
            print(f"Saved intermediate {counter}/{n_rows} as {fname}")
    return result_df


if __name__ == "__main__":
    np.random.seed(1) # to reproduce particle filter results

    from utils.custom_argparser import exp_parser, check_platform

    parser = exp_parser(
        "Apply the moving or particle estimator to flying or stepper datasets."
    )
    args = parser.parse_args()

    check_platform(args)

    beamform = False

    for exp_name in args.experiment_names:
        if exp_name == "2022_01_27_demo":
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            for appendix in ["test3", "test4"]:
                data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]

                for estimator in ESTIMATORS:
                    parameters = {
                        "calibration": [("iir", 0.3)]
                        + [("window", n_calib) for n_calib in [7]],
                        "mask_bad": [
                            ("fixed", None),
                        ],
                        "n_window": [1] if estimator != "moving" else [1, 3, 5],
                        "simplify": [True, False],
                    }
                    if beamform:
                        fname = f"results/demo_results_matrices_{estimator}{appendix}_beamform_new.pkl"
                    else:
                        fname = f"results/demo_results_matrices_{estimator}{appendix}_new.pkl"
                    matrix_df = generate_matrix_results(
                        data_row,
                        parameters,
                        fname=fname,
                        verbose=True,
                        estimator=estimator,
                        beamform=beamform,
                    )
        elif exp_name == "2021_10_12_flying":
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            #for appendix in data_df.appendix.unique():#[f"_{i}" for i in range(1, 7)] + ["_new3", "_new6"]:
            for appendix in [f"_{i}" for i in range(1, 7)] + ["_new3", "_new6"]:
                data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]
                for estimator in ESTIMATORS:
                    parameters = {
                        "calibration": [("iir", 0.3)],
                        "mask_bad": [("fixed", None)],
                        "n_window": [1] if estimator == "particle" else [5], 
                        "simplify": [False],
                    }
                    if beamform:
                        fname = f"results/flying_results_matrices_{estimator}{appendix}_beamform_new.pkl"
                    else:
                        fname = f"results/flying_results_matrices_{estimator}{appendix}_new.pkl"
                    matrix_df = generate_matrix_results(
                        data_row,
                        parameters,
                        fname=fname,
                        verbose=True,
                        estimator=estimator,
                        beamform=beamform,
                    )
        elif exp_name == "2021_07_08_stepper_fast":
            appendix = ""
            df_all = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            #data_row = combine_stepper_df(df_all, motors="all45000", bin_selection=5, merge="all")
            data_row = combine_stepper_df(df_all, motors="all45000", bin_selection=5, merge="middle")
            for estimator in ESTIMATORS:
                parameters = {
                    "calibration": [("iir", 0.3)],
                    "mask_bad": [(None, None)],
                    "n_window": [1] if estimator == "particle" else [5], 
                    "simplify": [False],
                }
                if beamform:
                    fname = f"results/stepper_results_matrices_{estimator}{appendix}{SIMULATE}_beamform_new.pkl"
                else:
                    fname = f"results/stepper_results_matrices_{estimator}{appendix}{SIMULATE}_new.pkl"
                matrix_df = generate_matrix_results(
                    data_row, parameters, fname=fname, verbose=True, estimator=estimator, beamform=beamform,
                )
        elif exp_name == "2021_07_27_epuck_wall":
            assert PLATFORM == "epuck"
            appendix = ""
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            data_row = combine_stepper_df(data_df, motors="sweep_and_move", bin_selection=0)
            for estimator in ESTIMATORS:
                parameters = {
                    "calibration": [("iir", 0.3)],
                    "mask_bad": [("fixed_epuck", None)],
                    "n_window": [1] if estimator == "particle" else [5], 
                    "simplify": [False],
                }
                if beamform:
                    fname = f"results/epuck_results_matrices_{estimator}{appendix}{SIMULATE}_beamform_new.pkl"
                else:
                    fname = f"results/epuck_results_matrices_{estimator}{appendix}{SIMULATE}_new.pkl"
                matrix_df = generate_matrix_results(
                    data_row, parameters, fname=fname, verbose=True, estimator=estimator, beamform=beamform,
                )
        else:
            raise ValueError(
                "Only use one of the allowed datasets: 2022_01_27_demo, 2021_07_08_stepper_fast, 2021_10_12_flying"
            )
