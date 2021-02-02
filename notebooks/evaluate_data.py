import itertools
import os
import sys

import numpy as np
import pandas as pd

from audio_stack.beam_former import BeamFormer, combine_rows, normalize_rows
from crazyflie_description_py.parameters import FS

#EXP_NAME = "2020_09_17_white-noise-static"
#EXP_NAME = "2020_10_14_static"
#EXP_NAME = "2020_10_14_static_new"
#EXP_NAME = "2020_10_30_dynamic"
EXP_NAME = "2020_11_30_wall_hover"

RES_DIRNAME = f"../experiments/{EXP_NAME}/results"

# results
combine_list = ["product", "sum"]
normalize_list = ["sum_to_one", "none", "zero_to_one", "zero_to_one_all"]
method_list = ["mvdr", "das"]

def get_fname(degree, props, snr, motors, source, distance=None, appendix="", **kwargs):
    from audio_bringup.doa_pipeline import get_filename
    return get_filename(degree=degree, props=props, snr=snr, motors=motors, source=source,distance=distance, appendix=appendix, **kwargs)


def get_fname_old(degree, props, snr, motors, source, **kwargs):
    ending = "" if degree == 0 else f"_{degree}"
    props_flag = "" if props else "no"
    motors_flag = "" if motors else "no"
    if (source == None) or (source == False):
        source_flag = "nosource"
    else:
        source_flag = "source"
    snr_flag = "" if snr else "no"
    return f"{motors_flag}motors_{snr_flag}snr_{props_flag}props_{source_flag}{ending}"


def read_full_df(degree=0, props=True, snr=True, motors=True, source=True, exp_name=EXP_NAME, distance=None, appendix=""):
    CSV_DIRNAME = f"../experiments/{exp_name}/csv_files"
    if exp_name == "2020_09_17_white-noise-static":
        filename = get_fname_old(degree, props, snr, motors, source)
    else:
        filename = get_fname(degree, props, snr, motors, source, distance=distance, appendix=appendix)
    fname = f"{CSV_DIRNAME}/{filename}.csv"
    df = pd.read_csv(fname)
    print('read', fname)
    return df


def read_df(degree=0, props=True, snr=True, motors=True, source=True, exp_name=EXP_NAME, distance=None, appendix="", **kwargs):
    #for key, val in kwargs.items():
    #    print(f'unused {key}:{val}')

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

    df = read_full_df(degree=degree, props=props, snr=snr, motors=motors, source=source, exp_name=exp_name, distance=distance, appendix=appendix)

    df_audio = df.loc[df.topic=='audio/signals_f']
    df_audio = df_audio.apply(convert_audio, axis=1)
    df_audio.drop(["signals_real_vect", "signals_imag_vect"], axis=1, inplace=True, errors='ignore')

    if 'source_direction-deg' in df.columns:
        df.rename(columns={'source_direction-deg':'source_direction_deg'}, inplace=True)
    pose_columns = set(['dx', 'dy', 'z', 'yaw_deg', 'yaw_rate_deg', 'source_direction_deg', 'timestamp', 'index', 'topic']).intersection(df.columns)
    df_pose = df.loc[df.topic=='geometry/pose_raw', pose_columns]
    return df_audio, df_pose


def read_df_others(degree=0, props=True, snr=True, motors=True, source=True, exp_name=EXP_NAME, distance=None, appendix=""):
    def convert_row(row):
        arrays = ["motors_pwm", "motors_thrust"]
        ints = ["timestamp"]
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
        return row

    df = read_full_df(degree=degree, props=props, snr=snr, motors=motors, source=source, exp_name=exp_name, distance=distance, appendix=appendix)
    
    status_columns = ['timestamp', 'index', 'topic', 'vbat']
    df_status = df.loc[df.topic=='crazyflie/status', status_columns]

    motors_columns = [c for c in ['timestamp', 'index', 'topic', 'motors_pwm', 'motors_thrust'] if c in df.columns]
    df_motors = df.loc[df.topic=='crazyflie/motors', motors_columns]
    df_motors = df_motors.apply(convert_row, axis=1)
    return df_status, df_motors


def read_df_from_wav(fname, n_buffer=2048, method_window="hann"):
    from audio_stack.processor import get_stft
    from scipy.io import wavfile
    
    n_mics = 1
    fs, source_data = wavfile.read(fname)
    print(f'read {fname}')

    # correct for different sampling frequencies so that we 
    # get roughly the same frequency bins. 
    n_buffer_corr = int(n_buffer * fs / FS)

    n_frames = len(source_data) // n_buffer_corr

    df = pd.DataFrame(columns=["index", "timestamp", "n_mics", "topic", "signals_f", "frequencies", "n_frequencies"])
    for i in range(n_frames):
        # n_mics x n_frequencies
        this_buffer = np.copy(source_data[i*n_buffer_corr:(i+1)*n_buffer_corr].reshape((1, -1)))
        signals_f, source_freq = get_stft(this_buffer, fs, method_window=method_window, method_noise="")
        df.loc[len(df), :] = {
            "index": i,
            "timestamp": i * n_buffer_corr/fs * 1000, # miliseconds
            "n_mics": n_mics,
            "topic": "measurement_mic",
            "signals_f": signals_f.T,  
            "frequencies": source_freq,
            "n_frequencies": len(source_freq)
        }
    return df


def read_signal_from_wav(fname, n_buffer=2048):
    from scipy.io import wavfile
    n_mics = 1
    fs, source_data = wavfile.read(fname)
    print(f'read {fname}')

    n_buffer_corr = int(n_buffer * fs / FS)
    n_frames = len(source_data) // n_buffer_corr

    signals = np.empty((n_frames, n_mics, n_buffer_corr))
    for i in range(n_frames):
        # n_mics x n_frequencies
        this_buffer = np.copy(source_data[i*n_buffer_corr:(i+1)*n_buffer_corr].reshape((1, -1)))
        signals[i, :, :] = this_buffer
    return signals


# TODO(FD) delete
def add_soundlevel(df, threshold=1e-4, duration=1000):
    from frequency_analysis import get_spectrogram
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


def get_positions(df_pos):
    """ Get absoltue positions from relative estimates. 
    """ 
    if isinstance(df_pos, pd.DataFrame):
        yaw_degs = df_pos.yaw_deg.values
        dxs = df_pos.dx.values
        dys = df_pos.dy.values
        zs = df_pos.z.values
    elif isinstance(df_pos, pd.Series): 
        yaw_degs = df_pos.yaw_deg
        dxs = df_pos.dx
        dys = df_pos.dy
        zs = df_pos.z

    # compute positions based on relative movement
    start_pos = np.array([0, 0])
    positions = np.empty([len(dxs), 3])
    for i, (yaw_deg, dx, dy, z) in enumerate(zip(yaw_degs, dxs, dys, zs)):
        yaw_rad = yaw_deg / 180 * np.pi
        length = np.sqrt(dx**2 + dy**2)
        new_pos = start_pos + length * np.array(
            [np.cos(yaw_rad), np.sin(yaw_rad)])
        positions[i, :2] = new_pos
        positions[i, 2] = z
    return positions


def integrate_yaw(times, yaw_rates):
    t_0 = times[0]
    yaw = 0
    integrated_yaw = [yaw]
    for yaw_rate, t_1 in zip(yaw_rates[1:], times[1:]):
        yaw_delta = yaw_rate * (t_1 - t_0) * 1e-3
        yaw += yaw_delta
        integrated_yaw.append(yaw)
        t_0 = t_1
    return np.array(integrated_yaw)


def evaluate_data(fname=""):
    sys.path.append(f"../experiments/{EXP_NAME}/")
    from params import SOURCE_LIST, DEGREE_LIST, global_params
    DURATION_SEC = global_params.get('duration', 30) # duration before end to be kept

    duration = DURATION_SEC * 1e3  # miliseconds

    source_list = SOURCE_LIST
    degree_list = DEGREE_LIST
    props_list = [True, False]
    snr_list = [True, False]
    motors_list = [True, False]

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
