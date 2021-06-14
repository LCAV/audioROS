import itertools
import sys
import warnings

import pandas as pd
import progressbar
from data_collector import DataCollector
from pandas_utils import filter_by_dict

OVERWRITE_RAW = True  # regenerate raw results instead of reading from backup

# this corresponds to the setup in BC325 with stepper motor:
D_OFFSET = 0.08  # actual distance at zero-distance, in meters


def data_collector_from_df(df_all, exp_name, mic_type, motors):
    data_collector = DataCollector(exp_name, mic_type)
    if not OVERWRITE_RAW:
        backup_found = data_collector.fill_from_backup(
            exp_name, mic_type, motors, appendix="_raw"
        )
    elif OVERWRITE_RAW or not backup_found:
        print("overwrite")
        chosen_dict = {
            "degree": DEGREE,
            "mic_type": mic_type,
            "motors": motors,
        }
        df_filtered = filter_by_dict(df_all, chosen_dict)
        if len(df_filtered) == 0:
            return None

        max_index = df_filtered.iloc[-1].name
        with progressbar.ProgressBar(max_value=max_index) as p:
            for i_row, row in df_filtered.iterrows():
                row.distance += D_OFFSET * 100
                data_collector.fill_from_row(row)
                p.update(i_row)
        data_collector.backup(exp_name, mic_type, motors, appendix="_raw")
    else:
        raise ValueError()
    return data_collector


if __name__ == "__main__":
    DEGREE = 0
    mic_types = ["audio_deck", "measurement"]
    motors_types = [0, "all45000"]
    exp_names = [
        # "2021_02_23_wall",
        # "2021_02_25_wall"
        # "2021_04_30_stepper"
        "2021_06_08_epuck_stepper"
    ]

    # exp_name = '2021_02_09_wall_tukey';
    # exp_name = '2021_02_09_wall';
    # exp_name = "2021_03_01_flying"
    # exp_name = '2020_12_9_rotating';
    # exp_name = '2020_11_26_wall';
    for exp_name in exp_names:
        fname = f"../experiments/{exp_name}/all_data.pkl"
        try:
            df_all = pd.read_pickle(fname)
            print("read", fname)
        except Exception as e:
            print("Error: run wall_analysis.py to parse experiments.")
            sys.exit()

        for mic_type, motors in itertools.product(mic_types, motors_types):
            data_collector = data_collector_from_df(df_all, exp_name, mic_type, motors)
            if data_collector is None:
                warnings.warn(f"no data found for {mic_type}, {motors}")
                continue
            data_collector.cleanup(verbose=False)
            data_collector.backup(exp_name, mic_type, motors)
