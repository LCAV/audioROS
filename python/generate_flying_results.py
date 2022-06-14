import itertools
import sys

import numpy as np
import pandas as pd
from progressbar import ProgressBar

from crazyflie_demo.wall_detection import WallDetection

from utils.constants import PLATFORM
from utils.moving_estimators import get_estimate

# DATASET = "flying" #exp_name = "2021_10_12_flying":
# DATASET = "stepper" #exp_name = "2021_07_08_stepper_fast"
# DATASET = "demo" # exp_name = "2022_01_27_demo"

#ESTIMATORS = ["moving"] 
#ESTIMATORS = ["particle"]  
ESTIMATORS = ["particle", "histogram", "moving"]  

def combine_stepper_df(data_df, motors="all45000", bin_selection=5, average=True, platform=PLATFORM):
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
    ] #TODONOW do we need to invert this?
    data_row = pd.Series(index=chosen_df.columns, dtype=object)
    # with below values, we assume that wall is north and horizontal and the movement is vertical downwards.
    data_row.positions = np.concatenate(
        [np.r_[[[0, -d*1e-2, height, 90-WALL_ANGLE_DEG_STEPPER]]] for d in DISTANCES_CM]
    )
    if bin_selection == 5:
        if average:
            data_row.stft = np.concatenate(
                [np.median(stft, axis=0)[None, :, :] for stft in chosen_df.stft.values], 
                axis=0
            )
        else:
            middle = chosen_df.iloc[0].stft.shape[0] // 2
            data_row.stft = np.concatenate(
                [stft[middle][None, :, :] for stft in chosen_df.stft.values],
                axis=0
            )
        data_row.frequencies_matrix = np.concatenate(
            [
                np.r_[[frequencies_mat[0, :]]]
                for frequencies_mat in chosen_df.frequencies_matrix.values
            ],
            axis=0,
        )

    # for e-puck, we still use bin_selection 0, so we need to get the best bin at each time.
    elif bin_selection == 0:
        stft = None
        frequencies_matrix = None
        for i, row in chosen_df.iterrows():
            signals_f_list = []
            freqs_list = []
            #print(row.stft.shape)
            for t in range(row.stft.shape[0]):
                freq_bin = np.argmax(np.sum(np.abs(row.stft[t]), axis=0)) # sum across mics
                f = row.frequencies_matrix[t, freq_bin]
                #print(f"amplitude of {f}", np.sum(np.abs(row.stft[t, :, freq_bin]), axis=0))
                freqs_list.append(f)
                signals_f_list.append(row.stft[t, :, freq_bin])
            signals_f = np.array(signals_f_list) # freqs x 4

            if stft is None:
                stft = signals_f.T[None, :]
            else:
                stft = np.concatenate([stft, signals_f.T[None, :]], axis=0)

            if frequencies_matrix is None:
                frequencies_matrix = np.array(freqs_list)[None, :]
            else:
                frequencies_matrix = np.concatenate([frequencies_matrix, np.array(freqs_list)[None, :]], axis=0)
        data_row.frequencies_matrix = frequencies_matrix 
        data_row.stft = stft

    data_row.seconds = chosen_df.seconds.values
    return data_row


def generate_matrix_results(data_row, parameters, estimator, fname="", verbose=False, beamform=False):
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
        progressbar = ProgressBar(maxval=n_positions)
        progressbar.start()
        for i in range(n_positions):
            position_cm = data_row.positions[i, :3] * 1e2

            signals_f = data_row.stft[i, :, freqs_all > 0]
            yaw_deg = data_row.positions[i, 3]
            time = data_row.seconds[i]

            res = wall_detection.listener_callback_offline(
                signals_f.T,
                freqs,
                position_cm,
                yaw_deg,
                timestamp=time,
            )
            if res is None:  # if flight check did not pass, for instance
                continue

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

        result_df.loc[counter, "matrix distances"] = [matrix_distances]
        result_df.loc[counter, "matrix angles"] = [matrix_angles]
        result_df.loc[counter, "average_time"] = np.mean(times)
        counter += 1
        if fname != "":
            result_df.to_pickle(fname)
            print(f"Saved intermediate {counter}/{n_rows} as {fname}")
    return result_df


if __name__ == "__main__":
    np.random.seed(1) # to reproduce particle filter results

    from utils.custom_argparser import exp_parser, check_platform

    beamform = True
    parser = exp_parser(
        "Apply the moving or particle estimator to flying or stepper datasets."
    )
    args = parser.parse_args()

    #check_platform(args)

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
                        beamform=beamform
                    )
        elif exp_name == "2021_10_12_flying":
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            #for appendix in data_df.appendix.unique():#[f"_{i}" for i in range(1, 7)] + ["_new3", "_new6"]:
            for appendix in [f"_{i}" for i in range(1, 7)] + ["_new3", "_new6"]:
                data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]
                for estimator in ESTIMATORS:
                    parameters = {
                        "calibration": [("iir", 0.3)],
                        "mask_bad": [
                            ("fixed", None),
                        ],
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
                        beamform=beamform
                    )
        elif exp_name == "2021_07_08_stepper_fast":
            assert PLATFORM == "crazyflie"
            appendix = ""
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            data_row = combine_stepper_df(data_df, motors="all45000", bin_selection=5)
            for estimator in ESTIMATORS:
                parameters = {
                    "calibration": [("iir", 0.3)],
                    "mask_bad": [("fixed", None)],
                    "n_window": [1] if estimator == "particle" else [5], 
                    "simplify": [False],
                }
                if beamform:
                    fname = f"results/stepper_results_matrices_{estimator}{appendix}_beamform_new.pkl"
                else:
                    fname = f"results/stepper_results_matrices_{estimator}{appendix}_new.pkl"
                matrix_df = generate_matrix_results(
                    data_row, parameters, fname=fname, verbose=True, estimator=estimator, beamform=beamform
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
                    fname = f"results/epuck_results_matrices_{estimator}{appendix}_beamform_new.pkl"
                else:
                    fname = f"results/epuck_results_matrices_{estimator}{appendix}_new.pkl"
                matrix_df = generate_matrix_results(
                    data_row, parameters, fname=fname, verbose=True, estimator=estimator, beamform=beamform
                )
        else:
            raise ValueError(
                "Only use one of the allowed datasets: 2022_01_27_demo, 2021_07_08_stepper_fast, 2021_10_12_flying"
            )
