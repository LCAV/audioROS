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
    "appendix_list": [""],
    "snr_list": [0],
    "props_list": [0],
    "wav": True,
    "method_window": "flattop",
}


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


def parse_experiments(
    exp_name="2020_12_9_moving", save_intermediate="", max_distance=None
):
    from crazyflie_description_py.parameters import N_BUFFER
    from audio_stack.parameters import WINDOW_TYPES, WINDOW_CORRECTION

    method_window = "hann"

    params_file = load_params(exp_name)
    params = {
        "degree": params_file.DEGREE_LIST,
        "distance": params_file.DISTANCE_LIST,
        "motors": params_file.MOTORS_LIST,
        "source": [str(s) for s in params_file.SOURCE_LIST],
    }
    overload_params = read_dataset_csv(exp_name)
    params.update(overload_params)

    if save_intermediate != "":
        counter = 0

    if params["wav"]:
        mic_type_list = ["measurement", "audio_deck"]
    else:
        mic_type_list = ["audio_deck"]

    cat_columns = {
        "snr": snr_list,
        "props": props_list,
        "mic_type": mic_type_list,
    }
    df_total = pd.DataFrame(
        columns=list(cat_columns.keys())  # categories
        + ["seconds", "frequencies_matrix", "stft", "positions"]  # data
    )

    params = {"exp_name": exp_name}
    for cat_values in itertools.product(*cat_columns.values()):
        params.update(dict(zip(cat_columns.keys(), cat_values)))

        if (max_distance is not None) and (params["distance"] > max_distance):
            continue

        try:
            # for experiments where window types were changed by appendix
            if "window" in params.get("appendix", ""):
                method_window = WINDOW_TYPES[
                    int(params["appendix"].replace("_window", ""))
                ]
            if "bin" in params.get("appendix", ""):
                params["snr"] = int(params["appendix"].replace("_bin", "")[0])
                print("snr:", params["snr"])

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
                    wav_fname, n_buffer=N_BUFFER, method_window=method_window
                )
        except FileNotFoundError as e:
            print("skipping", e)
            continue

        if not "signals_f" in df.columns:
            print("error, signals_f is empty. skipping...")
            continue

        df.signals_f /= WINDOW_CORRECTION[method_window]

        stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
        stft = clean_stft(stft)

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:, "frequencies"]])

        all_items = dict(
            seconds=seconds,
            frequencies_matrix=frequencies_matrix,
            stft=stft,
            positions=positions,
        )
        all_items.update(params)
        df_total.loc[len(df_total), :] = all_items

        if save_intermediate == "":
            continue

        counter += 1
        if counter % 10 == 0:
            pd.to_pickle(df_total, save_intermediate)
            print("saved intermediate as", save_intermediate)

    return df_total


def parse_experiments_wav(exp_name="2020_12_9_rotating", n_buffer=44100):
    if exp_name == "2020_12_9_rotating":
        appendix_list = ["", "_new"]
        snr_list = [0]
        props_list = [0]
    elif exp_name == "2020_11_26_wall":
        appendix_list = [""]
        snr_list = [0]
        props_list = [0]
    elif exp_name == "2021_02_09_wall":
        appendix_list = [""]
        snr_list = [0]
        props_list = [0]
    elif exp_name == "2021_02_09_wall_tukey":
        appendix_list = [""]
        snr_list = [0]
        props_list = [0]
    else:
        raise ValueError(exp_name)
    params_file = load_params(exp_name)

    cat_columns = {
        "appendix": appendix_list,
        "degree": params_file.DEGREE_LIST,
        "distance": params_file.DISTANCE_LIST,
        "motors": params_file.MOTORS_LIST,
        "source": [str(s) for s in params_file.SOURCE_LIST],
        "snr": snr_list,
        "props": props_list,
        "mic_type": ["measurement"],
    }
    df_total = pd.DataFrame(
        columns=list(cat_columns.keys())
        + ["seconds", "frequencies_matrix", "stft"]  # categories
    )
    params = {"exp_name": exp_name}

    for cat_values in itertools.product(*cat_columns.values()):
        params.update(dict(zip(cat_columns.keys(), cat_values)))
        try:
            fname = get_filename(**params)
            wav_fname = f"../experiments/{exp_name}/export/{fname}.wav"
            df = read_df_from_wav(wav_fname, n_buffer=n_buffer, fs_ref=None)
        except FileNotFoundError:
            print("skipping", params)
            continue

        stft = np.array([*df.signals_f.values])  # n_times x n_mics x n_freqs
        stft = clean_stft(stft)

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:, "frequencies"]])

        all_items = dict(
            seconds=seconds, frequencies_matrix=frequencies_matrix, stft=stft,
        )
        all_items.update(params)
        df_total.loc[len(df_total), :] = all_items

    return df_total


if __name__ == "__main__":
    import os

    exp_names = [
        "2021_07_08_stepper_fast",
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
