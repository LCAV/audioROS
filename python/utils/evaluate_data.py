import itertools
import os
import sys
import warnings

import numpy as np
import pandas as pd

from audio_bringup.helpers import get_filename
from .constants_crazyflie import FS, N_BUFFER

MAX_ALLOWED_LAG_MS = 20

# results
combine_list = ["product", "sum"]
normalize_list = ["sum_to_one", "none", "zero_to_one", "zero_to_one_all"]
method_list = ["mvdr", "das"]


def get_fname_old(degree, props, bin_selection, motors, source, **kwargs):
    ending = "" if degree == 0 else f"_{degree}"
    props_flag = "" if props else "no"
    motors_flag = "" if motors else "no"
    if (source == None) or (source == False):
        source_flag = "nosource"
    else:
        source_flag = "source"
    bin_selection_flag = "" if bin_selection else "no"
    return f"{motors_flag}motors_{bin_selection_flag}snr_{props_flag}props_{source_flag}{ending}"


def read_full_df(
    degree=0,
    props=True,
    bin_selection=1,
    motors=True,
    source=True,
    exp_name="",
    distance=None,
    appendix="",
):
    CSV_DIRNAME = f"../datasets/{exp_name}/csv_files"
    filename = get_filename(
        degree=degree,
        props=props,
        bin_selection=bin_selection,
        motors=motors,
        source=source,
        distance=distance,
        appendix=appendix,
    )
    fname = f"{CSV_DIRNAME}/{filename}.csv"
    df = pd.read_csv(fname)
    print("read", fname)
    return df


def read_df(
    filename=None,
    degree=0,
    props=True,
    bin_selection=True,
    motors=True,
    source=True,
    exp_name="",
    distance=None,
    appendix="",
    **kwargs,
):
    def convert_audio(row):
        arrays = [
            "signals_real_vect",
            "signals_imag_vect",
            "frequencies",
            "mic_positions",
        ]
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

        if "signals_real_vect" in row.index:
            signals_f = row.signals_real_vect + 1j * row.signals_imag_vect
            signals_f = signals_f.reshape((row.n_mics, row.n_frequencies))
            row["signals_f"] = signals_f

        if "mic_positions" in row.index:
            mic_positions = row.mic_positions.reshape((row.n_mics, -1))
            row["mic_positions"] = mic_positions
        return row

    if filename is None:
        df = read_full_df(
            degree=degree,
            props=props,
            bin_selection=bin_selection,
            motors=motors,
            source=source,
            exp_name=exp_name,
            distance=distance,
            appendix=appendix,
        )
    else:
        df = pd.read_csv(filename)

    df_audio = df.loc[df.topic == "audio/signals_f"]
    df_audio = df_audio.apply(convert_audio, axis=1)
    df_audio.drop(
        ["signals_real_vect", "signals_imag_vect"],
        axis=1,
        inplace=True,
        errors="ignore",
    )
    pose_columns = list(
        set(
            [
                "vx",
                "vy",
                "x",
                "y",
                "z",
                "yaw_deg",
                "yaw_rate_deg",
                "timestamp",
                "index",
                "topic",
            ]
        ).intersection(df.columns)
    )
    df_pose = df.loc[df.topic == "geometry/pose_raw", pose_columns]
    return df_audio, df_pose


def read_df_others(
    degree=0,
    props=True,
    bin_selection=True,
    motors=True,
    source=True,
    exp_name="",
    distance=None,
    appendix="",
):
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

    df = read_full_df(
        degree=degree,
        props=props,
        bin_selection=bin_selection,
        motors=motors,
        source=source,
        exp_name=exp_name,
        distance=distance,
        appendix=appendix,
    )

    status_columns = ["timestamp", "index", "topic", "vbat"]
    df_status = df.loc[df.topic == "crazyflie/status", status_columns]

    motors_columns = [
        c
        for c in ["timestamp", "index", "topic", "motors_pwm", "motors_thrust"]
        if c in df.columns
    ]
    df_motors = df.loc[df.topic == "crazyflie/motors", motors_columns]
    df_motors = df_motors.apply(convert_row, axis=1)
    return df_status, df_motors


def read_df_from_wav(fname, n_buffer=N_BUFFER, method_window="hann", fs_ref=FS):
    from audio_stack.processor import get_stft
    from scipy.io import wavfile

    n_mics = 1
    fs, source_data = wavfile.read(fname)
    print(f"read {fname}, fs:{fs}Hz")

    # correct for different sampling frequencies so that we
    # get roughly the same frequency bins.
    if fs_ref is not None:
        n_buffer_corr = int(n_buffer * fs / fs_ref)
    else:
        n_buffer_corr = n_buffer

    n_frames = len(source_data) // n_buffer_corr

    df = pd.DataFrame(
        columns=[
            "index",
            "timestamp",
            "n_mics",
            "topic",
            "signals_f",
            "frequencies",
            "n_frequencies",
        ]
    )
    for i in range(n_frames):
        # n_mics x n_frequencies
        this_buffer = np.copy(
            source_data[i * n_buffer_corr : (i + 1) * n_buffer_corr].reshape((1, -1))
        )
        signals_f, source_freq = get_stft(
            this_buffer, fs, method_window=method_window, method_noise=""
        )
        df.loc[len(df), :] = {
            "index": i,
            "timestamp": i * n_buffer_corr / fs * 1000,  # miliseconds
            "n_mics": n_mics,
            "topic": "measurement_mic",
            "signals_f": signals_f.T,
            "frequencies": source_freq,
            "n_frequencies": len(source_freq),
        }
    return df


def read_signal_from_wav(fname, n_buffer=N_BUFFER, fs_ref=FS):
    from scipy.io import wavfile

    n_mics = 1
    fs, source_data = wavfile.read(fname)
    print(f"read {fname}")

    if fs_ref is not None:
        n_buffer_corr = int(n_buffer * fs / fs_ref)
    else:
        n_buffer_corr = n_buffer

    n_frames = len(source_data) // n_buffer_corr

    signals = np.empty((n_frames, n_mics, n_buffer_corr))
    for i in range(n_frames):
        # n_mics x n_frequencies
        this_buffer = np.copy(
            source_data[i * n_buffer_corr : (i + 1) * n_buffer_corr].reshape((1, -1))
        )
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
    """Get absoltue positions from relative estimates."""
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
        new_pos = start_pos + length * np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
        positions[i, :2] = new_pos
        positions[i, 2] = z
    return positions


def get_positions_absolute(df_pos):
    """Get absolute positions"""
    if isinstance(df_pos, pd.DataFrame):
        if not len({"x", "y", "z", "yaw_deg"}.intersection(df_pos.columns.values)):
            # warnings.warn("no position information found")
            n_times = len(df_pos)
            return np.zeros((n_times, 4))
        xs = df_pos.x.values
        ys = df_pos.y.values
        zs = df_pos.z.values
        yaws = df_pos.yaw_deg.values
    elif isinstance(df_pos, pd.Series):
        if not len({"x", "y", "z", "yaw_deg"}.intersection(df_pos.index.values)):
            warnings.warn("no position information found")
            n_times = len(df_pos)
            return np.zeros((n_times, 4))
        xs = df_pos.x
        ys = df_pos.y
        zs = df_pos.z
        yaws = df_pos.yaw_deg
    positions = np.c_[xs, ys, zs, yaws]
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


def add_pose_to_df(df, df_pos, max_allowed_lag_ms=MAX_ALLOWED_LAG_MS, verbose=False):
    """For each row in df, add the latest position estimate,
    as long as it is within an allowed time window."""
    last_idx = None
    for i, row in df.iterrows():
        timestamp = row.timestamp

        # most recent position timestamp
        if not len(df_pos[df_pos.timestamp <= timestamp]):
            if verbose:
                print(
                    "Warning: no position before first audio:",
                    timestamp,
                    df_pos.timestamp.values,
                )
            continue
        pos_idx = df_pos[df_pos.timestamp <= timestamp].index[-1]
        if pos_idx == last_idx:
            if verbose:
                print(f"Warning: using {pos_idx} again.")
        pos_row = df_pos.loc[pos_idx]
        lag = timestamp - pos_row.timestamp
        # print(f"for audio {timestamp}, using position {pos_row.timestamp}")

        # find position columns
        pos_columns = list(set(pos_row.index).intersection(df.columns))
        if lag <= MAX_ALLOWED_LAG_MS:
            df.loc[i, pos_columns] = pos_row[pos_columns]
        else:
            if verbose:
                print(f"Warning for {pos_idx}: too high time lag {lag}ms")
        last_idx = pos_idx
