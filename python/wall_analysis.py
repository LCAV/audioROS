import itertools

import numpy as np
import pandas as pd
from audio_bringup.helpers import get_filename
from crazyflie_description_py.parameters import N_BUFFER
from dynamic_analysis import add_pose_to_df
from evaluate_data import get_positions_absolute
from evaluate_data import read_df, read_df_from_wav

FILENAME = "../experiments/datasets.csv"
DEFAULT_DICT = {
    "degree": {0},
    "disance": {0},
    "mic_type": {"audio_deck"},  # , "measurement"]
    "props": {0},
    "bin_selection": {0},
    "window_type": {0},
}


def extract_unique(params_list):
    params = {}
    for p in params_list:
        for key, val in p.items():
            if key in params:
                params[key].update({val})
            else:
                params[key] = {val}
    return params


def read_dataset_csv(exp_name):
    df = pd.read_csv(FILENAME, index_col=0, squeeze=True)
    df = df.apply(lambda row: row.str.strip(), axis=1)
    row = df.loc[df.exp_name == exp_name, DEFAULT_DICT.keys()].iloc[0]
    return row.to_dict()


def load_params(exp_name):
    """ load parameters module at the experiment of interest """
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "params", f"../experiments/{exp_name}/params.py"
    )
    params = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(params)
    return params


def clean_stft(stft, max_value=N_BUFFER):
    """
    The values in stft are normally between -N_BUFFER and N_BUFFER, 
    so values outside of this range are due to communication errors.
    """
    stft[np.isnan(stft)] = 0.0
    stft[np.abs(stft) > max_value] = 0.0
    return stft


def parse_experiments(exp_name="2020_12_9_moving"):
    from crazyflie_description_py.parameters import N_BUFFER
    from audio_stack.parameters import WINDOW_TYPES, WINDOW_CORRECTION

    params_file = load_params(exp_name)
    params_from_file = extract_unique(params_file.params_list)
    params_all = DEFAULT_DICT
    params_all.update(**params_from_file)

    df_total = pd.DataFrame(
        columns=list(params_all.keys())  # categories
        + ["seconds", "frequencies_matrix", "stft", "positions"]  # data
    )

    params = {"exp_name": exp_name}
    for cat_values in itertools.product(*params_all.values()):
        params.update(dict(zip(params_all.keys(), cat_values)))

        try:
            # for experiments where window types were changed by appendix
            if "window" in params.get("appendix", ""):
                params["window_type"] = int(params["appendix"].replace("_window", ""))
            if "bin" in params.get("appendix", ""):
                params["bin_selection"] = int(params["appendix"].replace("_bin", "")[0])

            positions = None
            if params["mic_type"] == "audio_deck":
                df, df_pos = read_df(**params)
                add_pose_to_df(
                    df, df_pos
                )  # synchronize position and audio measurements
                positions = get_positions_absolute(df)  # get positions as matrix
            elif params["mic_type"] == "measurement":
                fname = get_filename(**params)
                wav_fname = f"../experiments/{exp_name}/export/{fname}.wav"
                df = read_df_from_wav(
                    wav_fname,
                    n_buffer=N_BUFFER,
                    method_window=WINDOW_TYPES[params["window_type"]],
                )
        except FileNotFoundError as e:
            print("skipping", e)
            continue

        if not "signals_f" in df.columns:
            print("error, signals_f is empty. skipping...")
            continue

        df.signals_f /= WINDOW_CORRECTION[WINDOW_TYPES[params["window_type"]]]

        stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
        stft = clean_stft(stft)

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:, "frequencies"]])

        all_items = dict(
            seconds=seconds,
            frequencies_matrix=frequencies_matrix,
            stft=stft,
            positions=positions,
            **params,
        )
        df_total.loc[len(df_total), :] = all_items

    return df_total


if __name__ == "__main__":
    import os

    exp_names = [
        "2021_07_08_stepper_slow",
        # "2021_07_08_stepper_fast",
        # "2021_07_08_stepper",
        # "2021_07_07_stepper",
        # "2021_06_19_stepper",
        # "2021_06_17_stepper",
        # "2021_06_09_stepper",
        # "2021_05_04_linear",
        # "2021_05_04_flying",
        # "2021_04_30_hover",
        # "2021_04_30_stepper",
        # "2021_03_01_flying",
        # "2021_02_25_wall",
        # "2021_02_23_wall",
        # "2021_02_19_windows",
        #'2021_02_09_wall_tukey',
        #'2021_02_09_wall',
        #'2020_12_2_chirp',
        #'2020_12_11_calibration',
        #'2020_12_9_rotating',
        #'2020_12_18_flying',
        #'2020_12_18_stepper',
        #'2020_11_26_wall',
    ]
    for exp_name in exp_names:
        # fname = f'results/{exp_name}_real.pkl'
        fname = f"../experiments/{exp_name}/all_data.pkl"
        # fname = f'../experiments/{exp_name}/battery_data.pkl'

        dirname = os.path.dirname(fname)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)
            print("created directory", dirname)

        print("parsing", exp_name)
        if (
            exp_name == "2021_02_09_wall_tukey"
        ):  # there is an issue with space when trying to read all distances
            df_total = parse_experiments(exp_name=exp_name, max_distance=30)
        else:
            df_total = parse_experiments(exp_name=exp_name)
        pd.to_pickle(df_total, fname)
        print("saved as", fname)