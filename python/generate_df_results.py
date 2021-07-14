import itertools
import sys

import pandas as pd
import progressbar
from data_collector import DataCollector
from pandas_utils import filter_by_dict

OVERWRITE_RAW = True  # regenerate raw results instead of reading from backup

# this corresponds to the setup in BC325 with stepper motor:
D_OFFSET = 0.08  # actual distance at zero-distance, in meters


def data_collector_from_df(df_all, exp_name, mic_type, motors, bin_selection=""):
    data_collector = DataCollector(exp_name, mic_type)
    if not OVERWRITE_RAW:
        backup_found = data_collector.fill_from_backup(
            exp_name, mic_type, motors, bin_selection, appendix="_raw"
        )
    elif OVERWRITE_RAW or not backup_found:
        chosen_dict = {
            "degree": DEGREE,
            "mic_type": mic_type,
            "motors": motors,
            "bin_selection": bin_selection,
        }
        for key in chosen_dict.keys():
            assert key in df_all.columns, (key, df_all.columns)
        df_filtered = filter_by_dict(df_all, chosen_dict)
        if len(df_filtered) == 0:
            return None

        max_index = df_filtered.iloc[-1].name
        with progressbar.ProgressBar(max_value=max_index) as p:
            for i_row, row in df_filtered.iterrows():

                # for bin_selection=5 and 6, we have a full sweep in each package, so we want to use all of the values.
                # otherwise, we take the maximum per package.
                mode = "maximum" if row.bin_selection < 5 else "all"

                row.distance += D_OFFSET * 100
                data_collector.fill_from_row(row, mode=mode)
                p.update(i_row)
        data_collector.backup(
            exp_name, mic_type, motors, bin_selection, appendix="_raw"
        )
    return data_collector


if __name__ == "__main__":

    ## choose for which data we want to generate results
    DEGREE = 0
    mic_types = ["audio_deck"]  # , "measurement"]
    motors_types = [0, "all45000"]
    bin_selection_types = [5, 6]
    exp_names = [
        # "2021_02_23_wall",
        # "2021_02_25_wall"
        # "2021_04_30_stepper"
        # "2021_06_09_stepper"
        # "2021_06_17_stepper"
        # "2021_06_19_stepper"
        # "2021_07_07_stepper"
        # "2021_07_08_stepper"
        "2021_07_08_stepper_fast",
        "2021_07_08_stepper_slow"
        # "2021_07_07_rotating"
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

        for mic_type, motors, bin_selection in itertools.product(
            mic_types, motors_types, bin_selection_types
        ):
            data_collector = data_collector_from_df(
                df_all, exp_name, mic_type, motors, bin_selection
            )
            if data_collector is not None:
                data_collector.cleanup(verbose=False)
                data_collector.backup(exp_name, mic_type, motors, bin_selection)
