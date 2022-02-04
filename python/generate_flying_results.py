import itertools

import numpy as np
import pandas as pd
from progressbar import ProgressBar

from crazyflie_demo.wall_detection import WallDetection
from utils.moving_estimators import get_estimate


def generate_matrix_results(exp_name, appendix, parameters, fname=""):
    data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
    data_row = data_df.loc[data_df.appendix == appendix, :].iloc[0]

    n_cols = len(list(itertools.product(*parameters.values())))
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
        ]
    )
    counter = 0
    progressbar = ProgressBar(maxval=n_cols)
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

        wall_detection = WallDetection(python_only=True)

        result_df.loc[counter, "calibration name"] = calib[0]
        result_df.loc[counter, "calibration param"] = calib[1]
        result_df.loc[counter, "mask bad"] = str(mask_bad[0])
        result_df.loc[counter, "outlier factor"] = mask_bad[1]
        result_df.loc[counter, "n window"] = n_window
        result_df.loc[counter, "simplify angles"] = simplify
        result_df.loc[counter, "relative std"] = std

        matrix_distances = None
        matrix_angles = None

        freqs_all = data_row.frequencies_matrix[0, :]  # is same across times
        freqs = freqs_all[freqs_all > 0]
        n_positions = data_row.positions.shape[0]
        for i in range(n_positions):
            # print(f"{i+1} / {n_positions}")
            position_cm = data_row.positions[i, :3] * 1e2

            signals_f = data_row.stft[i, :, freqs_all > 0]
            yaw_deg = data_row.positions[i, 3]
            time = data_row.seconds[i]

            res = wall_detection.listener_callback_offline(
                signals_f, freqs, position_cm, yaw_deg, timestamp=time,
            )
            if res is None:
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
        counter += 1
        progressbar.update(counter)
        if fname != "":
            result_df.to_pickle(fname)
            print(f"Saved intermediate as {fname}")
    return result_df


if __name__ == "__main__":
    exp_name = "2022_01_27_demo"
    appendix = "test4"

    parameters = {
        "calibration": [  # [("iir", 0.2)],
            ("iir", alpha_iir) for alpha_iir in np.arange(0.1, 0.9, step=0.1)
        ]
        + [("window", n_calib) for n_calib in np.arange(2, 11, step=1)]
        + [("fixed", n_calib) for n_calib in np.arange(2, 11, step=1)],
        "mask_bad": [
            ("fixed", None),
            ("adaptive", 2),
            ("adaptive", 4),
            ("adaptive", 10),
            (None, None),
        ],
        "n_window": [3, 5, 10],
        "std": [0.0, 1.0],
        "simplify": [True, False],
    }

    fname = "results/DistanceFlying_matrices.pkl"
    matrix_df = generate_matrix_results(exp_name, appendix, parameters, fname=fname)
