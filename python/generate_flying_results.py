import itertools
import sys

import numpy as np
import pandas as pd
from progressbar import ProgressBar

from crazyflie_demo.wall_detection import WallDetection
from utils.moving_estimators import get_estimate

#DATASET = "flying" #exp_name = "2021_10_12_flying":
#DATASET = "stepper" #exp_name = "2021_07_08_stepper_fast"
#DATASET = "demo" # exp_name = "2022_01_27_demo"

ESTIMATORS = ["moving", "particle"]  # ["moving", "particle"]  # or moving

REL_STD = 0.0  # was found not to have a significant influce on results.


def combine_stepper_df(data_df, motors="all45000", bin_selection=5):
    """ combine all rows of a stepper dataset to one, as in flying datasets."""
    chosen_df = data_df.loc[
        (data_df.motors == motors) & (data_df.bin_selection == bin_selection)
    ]
    data_row = pd.Series(index=chosen_df.columns, dtype=object)
    data_row.positions = np.concatenate(
        [np.r_[[[0, -distance, 50, 0]]] for distance in chosen_df.distance.values]
    )
    data_row.stft = np.concatenate(
        [np.r_[[np.median(stft, axis=0)]] for stft in chosen_df.stft.values]
    )
    data_row.frequencies_matrix = np.concatenate(
        [
            np.r_[[frequencies_mat[0, :]]]
            for frequencies_mat in chosen_df.frequencies_matrix.values
        ],
        axis=0,
    )
    data_row.seconds = chosen_df.seconds.values
    return data_row


def generate_matrix_results(data_row, parameters, estimator, fname="", verbose=False):
    n_rows = len(list(itertools.product(*parameters.values())))
    result_df = pd.DataFrame(
        columns=[
            "calibration name",
            "calibration param",
            "mask bad",
            "outlier factor",
            "n window",
            "relative std",
            "simplify angles",
            "matrix distances",
            "matrix angles",
            "distances_cm",
            "angles_deg",
        ],
        index=range(n_rows),
    )
    counter = 0
    progressbar = ProgressBar(maxval=n_rows)
    progressbar.start()
    for calib, mask_bad, n_window, std, simplify in itertools.product(
        *parameters.values()
    ):
        # print("experiment", calib, mask_bad, n_window, std, simplify)
        WallDetection.CALIBRATION = calib[0]
        if calib[0] == "iir":
            WallDetection.N_CALIBRATION = 2
            WallDetection.ALPHA_IIR = calib[1]
        else:
            WallDetection.N_CALIBRATION = calib[1]
        WallDetection.MASK_BAD = mask_bad[0]
        WallDetection.OUTLIER_FACTOR = mask_bad[1]
        WallDetection.N_WINDOW = n_window
        WallDetection.RELATIVE_MOVEMENT_STD = std
        WallDetection.SIMPLIFY_ANGLES = simplify

        wall_detection = WallDetection(python_only=True, estimator=estimator)

        result_df.at[counter, "calibration name"] = calib[0]
        result_df.at[counter, "calibration param"] = calib[1]
        result_df.at[counter, "mask bad"] = str(mask_bad[0])
        result_df.at[counter, "outlier factor"] = mask_bad[1]
        result_df.at[counter, "n window"] = n_window
        result_df.at[counter, "simplify angles"] = simplify
        result_df.at[counter, "relative std"] = std
        result_df.at[counter, "distances_cm"] = wall_detection.estimator.distances_cm
        result_df.at[counter, "angles_deg"] = wall_detection.estimator.angles_deg

        matrix_distances = None
        matrix_angles = None

        freqs_all = data_row.frequencies_matrix[0, :]  # is same across times
        freqs = freqs_all[freqs_all > 0]
        n_positions = data_row.positions.shape[0]
        times = []
        for i in range(n_positions):
            # print(f"{i+1} / {n_positions}")
            position_cm = data_row.positions[i, :3] * 1e2

            signals_f = data_row.stft[i, :, freqs_all > 0]
            yaw_deg = data_row.positions[i, 3]
            time = data_row.seconds[i]

            res = wall_detection.listener_callback_offline(
                signals_f, freqs, position_cm, yaw_deg, timestamp=time,
            )
            if res is None:  # if flight check did not pass, for instance
                continue

            distances_cm_raw, prob_raw_dist, prob_moving_dist, prob_moving_angle = res
            if matrix_distances is None:
                matrix_distances = prob_moving_dist.reshape((-1, 1))
            else:
                matrix_distances = np.c_[
                    matrix_distances, prob_moving_dist.reshape((-1, 1))
                ]
            if matrix_angles is None:
                matrix_angles = prob_moving_angle.reshape((-1, 1))
            else:
                matrix_angles = np.c_[matrix_angles, prob_moving_angle.reshape((-1, 1))]

        result_df.loc[counter, "matrix distances"] = [matrix_distances]
        result_df.loc[counter, "matrix angles"] = [matrix_angles]
        result_df.loc[counter, "average_time"] = np.mean(times)
        counter += 1
        progressbar.update(counter)
        if fname != "":
            result_df.to_pickle(fname)
            print(f"Saved intermediate {counter}/{n_rows} as {fname}")
    return result_df


if __name__ == "__main__":

    np.random.seed(1)
    
    from utils.custom_argparser import exp_parser, check_platform
    parser = exp_parser("Apply the moving or particle estimator to flying or stepper datasets.")
    args = parser.parse_args()

    check_platform(args)

    for exp_name in args.experiment_names:
        if exp_name == "2022_01_27_demo":
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            for appendix in ["test3", "test4"]:
                data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]

                for estimator in ESTIMATORS:
                    parameters = {
                        "calibration": [  # [("iir", 0.2)],
                            ("iir", alpha_iir) for alpha_iir in [0.3]
                        ]
                        + [("window", n_calib) for n_calib in [7]],
                        # + [("fixed", n_calib) for n_calib in np.arange(1, 10, step=2)],
                        "mask_bad": [
                            ("fixed", None),
                            # ("adaptive", 2),
                            # ("adaptive", 4),
                            # ("adaptive", 10),
                            # (None, None),
                        ],
                        "n_window": [1] if estimator == "particle" else [1, 3, 5],
                        "std": [REL_STD],  # only for moving
                        "simplify": [True, False],
                    }
                    fname = f"results/demo_results_matrices_{estimator}{appendix}.pkl"
                    matrix_df = generate_matrix_results(
                        data_row, parameters, fname=fname, verbose=True, estimator=estimator
                    )

        elif exp_name == "2021_10_12_flying":
            for appendix in [f"_{i}" for i in range(1, 7)] + ["_new3", "_new6"]:
                data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
                data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]
                for estimator in ESTIMATORS:
                    parameters = {
                        "calibration": [  # [("iir", 0.2)],
                            ("iir", alpha_iir) for alpha_iir in [0.3]  # [0.3, 0.5, 0.7]
                        ],
                        # + [("window", n_calib) for n_calib in [5, 7]],
                        "mask_bad": [("fixed", None),],
                        "n_window": [1] if estimator == "particle" else [5],  # [1, 3, 5],
                        "std": [REL_STD],
                        "simplify": [True, False],
                    }
                    fname = f"results/flying_results_matrices_{estimator}{appendix}.pkl"
                    matrix_df = generate_matrix_results(
                        data_row, parameters, fname=fname, verbose=True, estimator=estimator
                    )
        elif exp_name == "2021_07_08_stepper_fast":
            appendix = ""
            data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
            data_row = combine_stepper_df(data_df, motors="all45000", bin_selection=5)
            for estimator in ESTIMATORS:
                parameters = {
                    "calibration": [  # [("iir", 0.2)],
                        ("iir", alpha_iir) for alpha_iir in np.arange(0.1, 1.0, step=0.2)
                    ]
                    + [("window", n_calib) for n_calib in np.arange(1, 10, step=2)]
                    + [("fixed", n_calib) for n_calib in np.arange(1, 10, step=2)],
                    "mask_bad": [
                        ("fixed", None),
                        # ("adaptive", 2),
                        # ("adaptive", 4),
                        # ("adaptive", 10),
                        # (None, None),
                    ],
                    "n_window": [1] if estimator == "particle" else [1, 3, 5],
                    "std": [REL_STD],  # only used for "moving"
                    "simplify": [False],
                }
                fname = f"results/stepper_results_matrices_{estimator}{appendix}.pkl"
                matrix_df = generate_matrix_results(
                    data_row, parameters, fname=fname, verbose=True, estimator=estimator
                )
        else:
            print("got:", exp_name)
            raise ValueError("Only use one of the allowed datasets: 2022_01_27_demo, 2021_07_08_stepper_fast, 2021_10_12_flying")
