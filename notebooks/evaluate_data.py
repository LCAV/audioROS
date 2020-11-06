import itertools
import os
import sys

import numpy as np
import pandas as pd

from audio_stack.beam_former import BeamFormer, combine_rows, normalize_rows

EXP_NAME = "2020_09_17_white-noise-static"
#EXP_NAME = "2020_10_14_static"
#EXP_NAME = "2020_10_14_static_new"
#EXP_NAME = "2020_10_30_dynamic"

RES_DIRNAME = f"../experiments/{EXP_NAME}/results"

sys.path.append(f"../experiments/{EXP_NAME}/")
from params import global_params, SOURCE_LIST, DEGREE_LIST

FS = 32000
DURATION_SEC = global_params['duration'] # duration before end to be kept

# expeirments
source_list = SOURCE_LIST
degree_list = DEGREE_LIST
props_list = [True, False]
snr_list = [True, False]
motors_list = [True, False]

# results
combine_list = ["product", "sum"]
normalize_list = ["sum_to_one", "none", "zero_to_one", "zero_to_one_all"]
method_list = ["mvdr", "das"]

def get_fname(degree, props, snr, motors, source):
    ending = "" if degree == 0 else f"_{degree}"
    props_flag = "" if props else "no"
    motors_flag = "" if motors else "no"
    if source == "None":
        source_flag = "nosource"
    elif source == True:
        source_flag = "source"
    elif source == False:
        source_flag = "nosource"
    elif source is None:
        source_flag = "None"
    else:
        source_flag = source
    snr_flag = "" if snr else "no"
    return f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}"


def get_fname_old(degree, props, snr, motors, source):
    ending = "" if degree == 0 else f"_{degree}"
    props_flag = "" if props else "no"
    motors_flag = "" if motors else "no"
    if (source == None) or (source == False):
        source_flag = "nosource"
    else:
        source_flag = "source"
    snr_flag = "" if snr else "no"
    return f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}"


def read_df(degree=0, props=True, snr=True, motors=True, source=True, exp_name=EXP_NAME):
    def convert_audio(row):
        arrays = ["signals_real_vect", "signals_imag_vect", "frequencies", "mic_positions"]
        ints = ["n_mics", "n_frequencies", "timestamp", "audio_timestamp"]

        arrays = [a for a in arrays if a in row.index]
        ints = [i for i in ints if i in row.index]
        for array_name in arrays:
            
            row[array_name] = np.fromstring(
                row[array_name].replace("[", "").replace("]", ""),
                sep=" ",
                dtype=np.float32,
            )

        for int_name in ints:
            row[int_name] = int(row[int_name])

        if 'signals_real_vect' in row.index:
            signals_f = row.signals_real_vect + 1j * row.signals_imag_vect
            signals_f = signals_f.reshape((row.n_mics, row.n_frequencies))
            row["signals_f"] = signals_f

        if 'mic_positions' in row.index: 
            mic_positions = row.mic_positions.reshape((row.n_mics, -1))
            row["mic_positions"] = mic_positions
        return row

    CSV_DIRNAME = f"../experiments/{exp_name}/csv_files"
    if exp_name == "2020_09_17_white-noise-static":
        filename = get_fname_old(degree, props, snr, motors, source)
    else:
        filename = get_fname(degree, props, snr, motors, source)
    fname = f"{CSV_DIRNAME}/{filename}.csv"
    df = pd.read_csv(fname)
    print('read', fname)
    df_audio = df.loc[df.topic=='audio/signals_f']
    df_audio = df_audio.apply(convert_audio, axis=1)
    df_audio.drop(["signals_real_vect", "signals_imag_vect"], axis=1, inplace=True)

    if 'source_direction-deg' in df.columns:
        df.rename(columns={'source_direction-deg':'source_direction_deg'}, inplace=True)

    pose_columns = [p for p in ['dx', 'dy', 'yaw_deg', 'source_direction_deg', 'timestamp', 'index', 'topic']
                    if p in df.columns]
    df_pose = df.loc[df.topic=='geometry/pose_raw', pose_columns]
    return df_audio, df_pose


def get_spec(degree=0, props=True, snr=True, motors=True, source=True, exp_name=EXP_NAME):
    from scipy.signal import stft
    from scipy.io import wavfile

    WAV_DIRNAME = f"../experiments/{exp_name}/export"
    filename = get_fname(degree, props, snr, motors, source)
    fname = f"{WAV_DIRNAME}/{filename}.wav"

    fs, source_data = wavfile.read(fname)
    
    n_buffer = 1024
    
    f, t, source_stft = stft(source_data, fs, nperseg=n_buffer, axis=0)
    mask = (f > 200) & (f < 7000)
    source_stft = source_stft[mask, :]
    return f[mask], t, source_stft


def get_spectrogram(df):
    stft = np.array([*df.loc[:, "signals_f"]])  # n_times x n_mics x n_freqs
    return np.mean(np.abs(stft), axis=1).T  # average over n_mics: n_freqs x n_times


def add_soundlevel(df, threshold=1e-4, duration=1000):
    spectrogram = get_spectrogram(df)  # n_freqs x n_times
    sound_level = np.mean(spectrogram**2, axis=0)  # average over n_frequencies
    df.loc[:, "sound_level"] = sound_level

    # detect the end time and cut fixed time before it
    try:
        end_index = np.where(sound_level > threshold)[0][-1]
    except:
        end_index = -1
    end_time = df.timestamp.values[end_index]
    df.loc[:, "timestamp_s"] = (df.timestamp - end_time + duration) / 1000


def evaluate_data(fname=""):
    duration = DURATION_SEC * 1e3  # miliseconds

    beam_former = None 

    result_df = pd.DataFrame(
        columns=[
            "index",
            "degree",
            "props",
            "snr",
            "motors",
            "source",
            "combine",
            "normalize",
            "method",
            "spectrum",
            "spectrum_raw",
            "frequencies",
        ]
    )

    for degree, props, snr, motors, source in itertools.product(
        degree_list, props_list, snr_list, motors_list, source_list
    ):
        try:
            df, df_pos = read_df(
                degree=degree, props=props, snr=snr, motors=motors, source=source
            )
        except FileNotFoundError:
            print("skipping:", degree, props, snr, motors, source)
            continue

        add_soundlevel(df, duration=duration)
        df = df[(df.timestamp_s > 0) & (df.timestamp_s <= (duration / 1000))]

        for i, row in df.iterrows():
            signals_f = row.signals_f
            freqs = row.frequencies
            if (beam_former is None) and ('mic_positions' in row.index): 
                beam_former = BeamFormer(mic_positions=row.mic_positions)
                print("Created beamformer with published mic_positions.")
            elif beam_former is None:
                from crazyflie_description_py.parameters import MIC_POSITIONS
                beam_fomer = BeamFormer(mic_positions=np.array(MIC_POSITIONS))
                print("Created beamformer from crazyflie_description_py.")

            arg_idx = np.argsort(freqs)
            freqs = freqs[arg_idx]
            signals_f = signals_f[:, arg_idx]

            R = beam_former.get_correlation(signals_f.T)

            for method in method_list:
                # spectrum_raw is of shape n_frequencies x n_thetas
                if method == "mvdr":
                    spectrum_raw = beam_former.get_mvdr_spectrum(R, freqs)
                elif method == "das":
                    spectrum_raw = beam_former.get_das_spectrum(R, freqs)
                else:
                    raise ValueError(method)

                for normalize in normalize_list:
                    spectrum_norm = normalize_rows(spectrum_raw, method=normalize)

                    for combine in combine_list:
                        spectrum = combine_rows(spectrum_norm, method=combine)

                        result_df.loc[len(result_df), :] = {
                            "index": i,
                            "degree": degree,
                            "props": props,
                            "snr": snr,
                            "motors": motors,
                            "source": source,
                            "combine": combine,
                            "normalize": normalize,
                            "method": method,
                            "spectrum": spectrum,
                            "spectrum_raw": spectrum_raw,
                            "frequencies": freqs,
                        }

        if fname != "":
            if not os.path.exists(RES_DIRNAME):
                os.makedirs(RES_DIRNAME)
            result_df.to_pickle(fname)
            print(f"saved intermediate as {fname}")
    return result_df


if __name__ == "__main__":
    #fname = f"{RES_DIRNAME}/static_spectra.pkl"
    fname = ""
    result_df = evaluate_data(fname)
