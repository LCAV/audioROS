import itertools

import numpy as np
import pandas as pd

from pandas_utils import filter_by_dict
from wall_detector import WallDetector

mag_threshs = {
    '2021_02_25_wall': 1,
    '2021_02_23_wall': 1e-2
}

def wall_detector_from_df(df_all, exp_name, mic_type, motors):
    chosen_dict = {
        "degree": DEGREE,
        "mic_type": mic_type,
        "motors": motors,
    }
    wall_detector = WallDetector(exp_name, mic_type)

    df_filtered = filter_by_dict(df_all, chosen_dict)
    if len(df_filtered) == 0:
        return []

    for i_row, row in df_filtered.iterrows():
        wall_detector.fill_from_row(row)
    wall_detector.cleanup(mag_thresh=mag_threshs[exp_name], verbose=True)
    return wall_detector


if __name__ == "__main__":
    DEGREE = 0
    mic_types = ["audio_deck"]
    motors_types = [0, "all45000"]

    # exp_name = '2021_02_09_wall_tukey';
    # exp_name = '2021_02_09_wall';
    # exp_name = "2021_03_01_flying"
    # exp_name = '2020_12_9_rotating';
    # exp_name = '2020_11_26_wall';
    for exp_name in ["2021_02_23_wall", "2021_02_25_wall"]:  # old buzzer  # new buzzer
        fname = f"../experiments/{exp_name}/all_data.pkl"
        try:
            df_all = pd.read_pickle(fname)
            print("read", fname)
        except Exception as e:
            print(e)
            print("Error: run wall_analysis.py to parse experiments.")

        for mic_type, motors in itertools.product(mic_types, motors_types):
            wall_detector = wall_detector_from_df(df_all, exp_name, mic_type, motors)
            wall_detector.backup(exp_name, mic_type, motors)
