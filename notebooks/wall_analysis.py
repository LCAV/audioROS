import itertools

import numpy as np
import pandas as pd

from audio_bringup.helpers import get_filename
from crazyflie_description_py.parameters import N_BUFFER
from evaluate_data import read_df, read_df_from_wav
from evaluate_data import get_positions_absolute
from dynamic_analysis import add_pose_to_df

VELOCITY = 0.05  # [m/s], parameter in crazyflie
D_START = 0.6  # [m], starting distance
FILTERS = ["mic_type", "snr", "motors"]


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


# TODO(FD) probably deprecated, can be removed.
def get_df_matrices(exp_name, max_distance=None, plot=False):
    import progressbar
    from pandas_utils import fill_df
    from wall_detector import WallDetector

    fname = f"../experiments/{exp_name}/all_data.pkl"
    df_freq = pd.read_pickle(fname)

    results_df = pd.DataFrame(
        columns=[
            "mic_type",
            "snr",
            "motors",
            "exp_name",
            "df_matrix",
            "df_dist",
            "df_freq",
        ]
    )

    for chosen_tuple, df_mic in df_freq.groupby(FILTERS, sort=False):
        filter_dict = dict(zip(FILTERS, chosen_tuple))

        wall_detector = WallDetector(
            exp_name=exp_name, mic_type=filter_dict["mic_type"]
        )

        if filter_dict["mic_type"] == "measurement" and filter_dict["snr"] == 1:
            continue

        d_idx = 0

        if max_distance is not None:
            df_mic = df_mic[df_mic.distance < max_distance]
        max_value = len(df_mic)
        with progressbar.ProgressBar(max_value=max_value) as p:
            for counter, (i, row) in enumerate(df_mic.iterrows()):
                if max_distance is not None and (row.distance > max_distance):
                    continue
                try:
                    spec_masked_all, freqs_masked = wall_detector.fill_from_row(row)
                except Exception as e:
                    print("error with row", row)
                    print(e)
                    continue
                p.update(counter)
                if plot:
                    fig, ax = plt.subplots()
                    ax.pcolorfast(
                        range(spec_masked_all.shape[-1]),
                        freqs_masked,
                        np.log10(spec_masked_all[:, 0, :]),
                    )
                    ax.set_title(f"distance={row.distance}cm")
        wall_detector.cleanup()
        df_amp, distances, frequencies = wall_detector.get_df_matrix()

        filter_dict.update({"exp_name": exp_name})
        fill_dict = {"df_dist": distances, "df_freq": frequencies, "df_matrix": df_amp}
        print("filling results_df...")
        results_df = fill_df(results_df, filter_dict, fill_dict)
    return results_df


def parse_experiments(
    exp_name="2020_12_9_moving", save_intermediate="", max_distance=None
):
    method_window = "hann"
    if exp_name == "2020_12_7_moving":
        appendix_list = ["", "_new"]
        snr_list = [0, 1]
        props_list = [0]
        wav = True
    elif exp_name == "2020_12_9_rotating":
        appendix_list = ["", "_new"]
        snr_list = [0, 1]
        props_list = [0, 1]
        wav = True
    elif exp_name == "2020_12_18_flying":
        appendix_list = ["", "_new"]
        snr_list = [2]
        props_list = [0, 1]
        wav = False
    elif exp_name == "2020_12_18_stepper":
        appendix_list = ["", "_new"]
        snr_list = [2]
        props_list = [0, 1]
        wav = True
    elif exp_name == "2020_11_26_wall":
        appendix_list = [""]
        snr_list = [0]
        props_list = [0]
        wav = True
    elif exp_name == "2020_12_11_calibration":
        appendix_list = ["", "_BC329", "_HALL", "_HALL2", "_HALL3"]
        snr_list = [0, 1]
        props_list = [0, 1]
        wav = False
    elif exp_name == "2020_12_2_chirp":
        appendix_list = [""]
        snr_list = [0]
        props_list = [0]
        wav = True
    elif exp_name == "2021_02_09_wall":
        appendix_list = [""]
        snr_list = [3]
        props_list = [0]
        wav = True
        method_window = ""
    elif exp_name == "2021_02_09_wall_tukey":
        appendix_list = ["", "_afterbug", "_afterbug2", "_with_3cm", "_second shot"]
        snr_list = [3]
        props_list = [0]
        wav = True
        method_window = ""
    elif exp_name == "2021_02_19_windows":
        appendix_list = [f"_window{i}" for i in range(4)]
        snr_list = [3]
        props_list = [0]
        wav = True
    elif exp_name == "2021_02_19_windows_newbuzzer":
        appendix_list = [f"_window{i}" for i in range(4)]
        snr_list = [3]
        props_list = [0]
        wav = True
    elif exp_name == "2021_02_23_wall":
        appendix_list = [""]
        snr_list = [3]
        props_list = [0]
        wav = True
        method_window = "flattop"
    elif exp_name == "2021_02_25_wall":
        appendix_list = ["", "_externalpsu"]
        snr_list = [3]
        props_list = [0]
        wav = True
        method_window = "flattop"
    elif exp_name == "2021_03_01_flying":
        appendix_list = [
            "_30cm-paper",
            "_50cm-paper",
            "_30cm-nopaper",
            "_50cm-nopaper",
            "_30cm-newbuzzer",
            "_50cm-newbuzzer",
        ]
        snr_list = [3]
        props_list = [0]
        wav = False
        method_window = "flattop"
    else:
        raise ValueError(exp_name)

    if save_intermediate != "":
        counter = 0

    if wav:
        mic_type_list = ["measurement", "audio_deck"]
    else:
        mic_type_list = ["audio_deck"]

    from crazyflie_description_py.parameters import (
        N_BUFFER,
        WINDOW_TYPES,
        WINDOW_CORRECTION,
    )

    params_file = load_params(exp_name)

    # TODO(FD) remove this when we use more angles again.
    params_file.DEGREE_LIST = [0]

    cat_columns = {
        "appendix": appendix_list,
        "degree": params_file.DEGREE_LIST,
        "distance": params_file.DISTANCE_LIST,
        "motors": params_file.MOTORS_LIST,
        "source": [str(s) for s in params_file.SOURCE_LIST],
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
        # "2021_03_01_flying",
        # "2021_02_25_wall",
        # "2021_02_23_wall",
        "2021_02_19_windows",
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
