import itertools

import numpy as np
import pandas as pd

from audio_stack.spectrum_estimator import combine_rows, normalize_rows
from audio_stack.beam_former import BeamFormer

CSV_DIRNAME = "../experiments/2020_09_17_white-noise-static/csv_files"
WAV_DIRNAME = "../experiments/2020_09_17_white-noise-static/export"
FS = 32000

degree_list = [0, 20, 45]
props_list = [True, False]
snr_list = [True, False]
motors_list = [True, False]
combine_list = ["product", "sum"]
normalize_list = ["sum_to_one", "none", "zero_to_one", "zero_to_one_all"]
method_list = ["mvdr", "das"]

def read_df(degree=0, props=True, snr=True, motors=True, source=True):
    def convert(row):
        arrays = ["signals_real_vect", "signals_imag_vect", "frequencies"]
        ints = ["n_mics", "n_frequencies", "timestamp"]
        for array_name in arrays:
            row[array_name] = np.fromstring(
                row[array_name].replace("[", "").replace("]", ""),
                sep=" ",
                dtype=np.float32,
            )
        for int_name in ints:
            row[int_name] = int(row[int_name])

        signals_f = row.signals_real_vect + 1j * row.signals_imag_vect
        signals_f = signals_f.reshape((row.n_mics, row.n_frequencies))
        row["signals_f"] = signals_f
        return row

    ending = "" if degree == 0 else f"_{degree}"
    props_flag = "" if props else "no"
    motors_flag = "" if motors else "no"
    source_flag = "" if source else "no"
    snr_flag = "" if snr else "no"
    fname = f"{CSV_DIRNAME}/{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}source{ending}.csv"
    print("reading", fname)

    df = pd.read_csv(fname)
    df = df.apply(convert, axis=1)
    df.drop(["signals_real_vect", "signals_imag_vect"], axis=1, inplace=True)
    return df


def get_spectrogram(df):
    stft = np.array([*df.loc[:, "signals_f"]])  # n_times x n_mics x n_freqs
    return np.mean(np.abs(stft), axis=1).T  # average over n_mics, n_freqs x n_times


def add_soundlevel(df, threshold=1e-2, duration=1000):
    # detect the end time and cut fixed time before it
    spectrogram = get_spectrogram(df)  # n_freqs x n_times
    sound_level = np.mean(spectrogram, axis=0)  # average over n_frequencies
    df.loc[:, "sound_level"] = sound_level
    end_index = np.where(sound_level > threshold)[0][-1]
    end_time = df.timestamp[end_index]
    df.loc[:, "timestamp_s"] = (df.timestamp - end_time + duration) / 1000


def evaluate_data(fname=""):
    duration = 20 * 1e3  # miliseconds

    # TODO(FD) will be read from the topic in the future
    mic_d = 0.108  # distance between mics (meters)
    mic_positions = mic_d / 2 * np.c_[[1, 1], [1, -1], [-1, 1], [-1, -1]].T
    beam_former = BeamFormer(mic_positions=mic_positions)

    result_df = pd.DataFrame(
        columns=[
            "index",
            "degree",
            "props",
            "snr",
            "motors",
            "combine",
            "normalize",
            "method",
            "spectrum",
            "spectrum_raw",
        ]
    )

    for degree, props, snr, motors in itertools.product(
        degree_list, props_list, snr_list, motors_list
    ):
        try:
            df = read_df(
                degree=degree, props=props, snr=snr, motors=motors, source=True
            )
        except FileNotFoundError:
            print("skipping!")
            continue

        add_soundlevel(df, duration=duration)
        df = df[(df.timestamp_s > 0) & (df.timestamp_s <= (duration / 1000))]

        for i, row in df.iterrows():
            signals_f = row.signals_f
            freqs = row.frequencies

            arg_idx = np.argsort(freqs)
            freqs = freqs[arg_idx]
            signals_f = signals_f[:, arg_idx]

            R = beam_former.get_correlation(signals_f.T)

            for method in method_list:
                if method == "mvdr":
                    # n_frequencies x n_thetas
                    spectrum_raw = beam_former.get_mvdr_spectrum(R, freqs)
                elif method == "das":
                    # n_frequencies x n_thetas
                    spectrum_raw = beam_former.get_das_spectrum(R, freqs)
                else:
                    raise ValueError(method)

                for normalize in normalize_list:
                    if normalize != "none":
                        spectrum_norm = normalize_rows(spectrum_raw, method=normalize)
                    else:
                        spectrum_norm = spectrum_raw

                    for combine in combine_list:
                        spectrum = combine_rows(spectrum_norm, method=combine)

                        result_df.loc[len(result_df), :] = {
                            "index": i,
                            "degree": degree,
                            "props": props,
                            "snr": snr,
                            "motors": motors,
                            "combine": combine,
                            "normalize": normalize,
                            "method": method,
                            "spectrum": spectrum,
                            "spectrum_raw": spectrum_raw,
                        }

        if fname != "":
            result_df.to_pickle(fname)
            print(f"saved intermediate as {fname}")
    return result_df


if __name__ == "__main__":
    fname = f"results/static_spectra_raw.pkl"
    result_df = evaluate_data(fname)
